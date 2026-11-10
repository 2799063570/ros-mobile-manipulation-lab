"""Local web task provider; transport-independent and guarded by the ROS journal."""
import copy
import errno
import fcntl
import json
import math
from pathlib import Path
import threading
import time
import uuid

from aubo_cloud_bridge.commands import Dispatcher
from aubo_cloud_bridge.protocol import CommandJournal
from aubo_huawei_bridge.iot import decode_command, result_summary
from aubo_huawei_bridge.telemetry import HuaweiTelemetry


class JournalInUseError(ValueError):
    """Another process still owns the command journal; never bypass its lock."""


class WebTasks:
    poll_seconds = 2
    fresh_seconds = 5

    def __init__(self, directory, execute, enabled=False, owner_wait=0):
        if not 0 <= owner_wait <= 5:
            raise ValueError('invalid journal owner wait')
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.owner = (directory / 'dashboard.lock').open('a')
        deadline = time.monotonic() + owner_wait
        while True:
            try:
                fcntl.flock(self.owner.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError as error:
                if error.errno not in (errno.EACCES, errno.EAGAIN):
                    self.owner.close()
                    raise
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.owner.close()
                    raise JournalInUseError('another dashboard owns this journal directory')
                time.sleep(min(.05, remaining))
        self.lock = threading.RLock()
        self.journal = CommandJournal(str(directory / 'dashboard_commands.sqlite3'))
        self.telemetry = HuaweiTelemetry()
        self.enabled = enabled
        self.session = uuid.uuid4().hex
        self.available = False
        self.checked = 0.0
        self.closed = False
        self.last = {}
        self.cell = {}
        self.pending = {}
        self.dispatcher = Dispatcher(self.journal, execute, self.reply, self.report,
                                     self.context, enabled=enabled)

    def health(self, available):
        with self.lock:
            if self.closed:
                return
            if bool(available) != self.available or time.monotonic() - self.checked > 4:
                self.session = uuid.uuid4().hex
            self.available, self.checked = bool(available), time.monotonic()

    def context(self):
        with self.lock:
            return self.session, (not self.closed and self.available and time.monotonic() - self.checked < 4)

    def snapshot(self):
        with self.lock:
            session, available = self.context()
            values = self.telemetry.snapshot()
            values.update(bridge_session=session, sorting_available=int(available),
                          commands_enabled=int(self.enabled), last_command=json.dumps(self.last),
                          bridge_wall_ms=str(int(time.time() * 1000)) if available else '0',
                          target_cache_truncated=0)
            if not available:
                values.update(state='UNKNOWN', state_detail='本机分拣接口未就绪，请检查分拣程序')
            return values

    def set_cell(self, center, size, frame):
        valid = (isinstance(center, (list, tuple)) and isinstance(size, (list, tuple))
                 and len(center) >= 2 and len(size) >= 2
                 and all(type(v) in (int, float) and math.isfinite(v) for v in list(center[:2]) + list(size[:2])))
        with self.lock:
            self.cell = ({'table_center': list(center[:2]), 'table_size': list(size[:2]),
                          'frame': str(frame)[:64]} if valid and all(0 < v <= 100 for v in size[:2]) else {})

    def workcell(self):
        with self.lock:
            return copy.deepcopy(self.cell)

    def report(self, result):
        with self.lock:
            self.last = result_summary(dict(result, wall_stamp_ms=int(time.time() * 1000)))

    def reply(self, request_id, result):
        with self.lock:
            waiter = self.pending.get(request_id)
            if waiter:
                waiter['result'] = copy.deepcopy(result)
                waiter['event'].set()

    def command(self, envelope):
        document = decode_command(envelope)
        request_id = uuid.uuid4().hex
        waiter = {'event': threading.Event()}
        with self.lock:
            # Keep HTTP waiters bounded independently of the dispatcher's two task lanes.
            if len(self.pending) >= 8:
                return {'response': {'result_code': 1, 'paras': {'result': {'status': 'rejected', 'reason': 'web_busy'}}}}
            self.pending[request_id] = waiter
        try:
            self.dispatcher.submit(request_id, document)
            if not waiter['event'].wait(12):
                raise TimeoutError('outcome unknown; inspect last_command')
            result = waiter['result']
            return {'response': {'result_code': {'accepted': 0, 'rejected': 1}.get(result['status'], 2),
                                 'paras': {'result': result}}}
        finally:
            with self.lock:
                self.pending.pop(request_id, None)

    def close(self):
        with self.lock:
            self.closed = True
            self.available = False
            self.session = ''
