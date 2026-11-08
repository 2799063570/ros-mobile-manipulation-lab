"""Bounded service dispatch independent from MQTT's network callback thread."""
import threading
import time

from .protocol import validate_rpc


class Dispatcher:
    def __init__(self, journal, execute, reply, report, context, enabled=False):
        self.journal, self.execute = journal, execute
        self.reply, self.report, self.context = reply, report, context
        self.enabled = enabled
        self.lock = threading.Lock()
        self.active = set()
        self.stop_epoch = 0

    def submit(self, request_id, document, retained=False):
        try:
            session, available = self.context()
            method, params = validate_rpc(document, session, int(time.time() * 1000))
            if retained:
                raise ValueError('retained_command')
            if not self.enabled:
                raise ValueError('commands_disabled')
            if not available:
                raise ValueError('sorting_unavailable')
            lane = 'stop' if method == 'stop' else 'motion'
            with self.lock:
                # Look up duplicates even when the original worker is still running.
                fresh, result = self.journal.reserve(document)
                if not fresh:
                    self.reply(request_id, {**result, 'duplicate': True})
                    return
                if lane in self.active:
                    result = {'command_id': params['command_id'], 'status': 'rejected', 'reason': 'bridge_busy'}
                    self.journal.finish(params['command_id'], result)
                    self.reply(request_id, result)
                    return
                self.active.add(lane)
                if lane == 'stop':
                    self.stop_epoch += 1
                epoch = self.stop_epoch
            # Monotonic deadline prevents a later wall clock adjustment extending a queued command.
            deadline = time.monotonic() + (params['expires_at_ms'] - time.time() * 1000) / 1000
            worker = threading.Thread(target=self._run, args=(request_id, method, params, lane, epoch, deadline), daemon=True)
            worker.start()
        except (ValueError, TypeError):
            self.reply(request_id, {'status': 'rejected', 'reason': 'invalid_or_unavailable_command'})
        except Exception:
            # Fail closed if the persistent journal cannot be committed.
            self.reply(request_id, {'status': 'rejected', 'reason': 'journal_or_dispatch_error'})

    def _run(self, request_id, method, params, lane, epoch, deadline):
        command_id = params['command_id']

        def guard():
            session, available = self.context()
            with self.lock:
                not_cancelled = lane == 'stop' or self.stop_epoch == epoch
            return (available and session == params['session'] and not_cancelled
                    and time.monotonic() < deadline and time.time() * 1000 < params['expires_at_ms'])

        result = {'command_id': command_id, 'method': method}
        try:
            if not guard():
                result.update(status='rejected', reason='expired_disconnected_or_cancelled')
            else:
                success, message = self.execute(method, guard)
                # ROS Trigger only acknowledges the request. Observe state/progress for completion.
                result.update(status='accepted' if success else 'rejected', message=str(message)[:1000])
        except Exception:
            # A broken service connection may occur after the robot accepted the request.
            result.update(status='unknown', reason='service_outcome_unknown_check_robot_state')
        try:
            self.journal.finish(command_id, result)
            self.report(result)
            self.reply(request_id, result)
        finally:
            with self.lock:
                self.active.discard(lane)
