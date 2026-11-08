"""No ROS/MQTT dependencies. Wall time for RPC expiry, monotonic time for ages."""
import hashlib
import json
import math
import re
import sqlite3
import threading
import time

METHODS = frozenset(('move_to_observation', 'start', 'stop', 'home'))
TELEMETRY_TOPIC = 'v1/devices/me/telemetry'
RPC_PREFIX = 'v1/devices/me/rpc/request/'
RESPONSE_PREFIX = 'v1/devices/me/rpc/response/'


def decode_json(data):
    def reject_constant(value):
        raise ValueError('non-finite JSON number')
    return json.loads(data, parse_constant=reject_constant)


def encode_json(data):
    return json.dumps(data, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def validate_rpc(document, session, now_ms):
    if not isinstance(document, dict) or document.get('method') not in METHODS:
        raise ValueError('unknown_method')
    params = document.get('params')
    if not isinstance(params, dict) or set(params) != {
            'command_id', 'session', 'issued_at_ms', 'expires_at_ms'}:
        raise ValueError('invalid_params')
    if not isinstance(params['command_id'], str) or not re.fullmatch(
            r'[A-Za-z0-9_-]{16,80}', params['command_id']):
        raise ValueError('invalid_command_id')
    if not session or params['session'] != session:
        raise ValueError('stale_session')
    issued, expires = params['issued_at_ms'], params['expires_at_ms']
    if type(issued) is not int or type(expires) is not int:
        raise ValueError('invalid_timestamp')
    if not 0 < expires - issued <= 30000 or issued > now_ms + 5000 or expires <= now_ms:
        raise ValueError('expired_or_clock_skew')
    return document['method'], params


class CommandJournal:
    """Commit before calling ROS. A crash leaves UNKNOWN; never replay that action.

    One process owns a DB. Rows intentionally survive reconnects/restarts. Do not
    put secrets in the journal. The deployment must use a local persistent disk.
    """
    def __init__(self, path):
        self.lock = threading.Lock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('CREATE TABLE IF NOT EXISTS commands '
                        '(id TEXT PRIMARY KEY, digest TEXT NOT NULL, result TEXT NOT NULL)')
        self.db.commit()

    def reserve(self, document):
        command_id = document['params']['command_id']
        digest = hashlib.sha256(json.dumps(document, sort_keys=True).encode()).hexdigest()
        with self.lock:
            row = self.db.execute('SELECT digest,result FROM commands WHERE id=?', (command_id,)).fetchone()
            if row:
                if row[0] != digest:
                    return False, {'status': 'rejected', 'reason': 'command_id_conflict'}
                return False, decode_json(row[1])
            result = {'command_id': command_id, 'status': 'unknown',
                      'reason': 'reserved_or_interrupted_do_not_retry_with_new_id'}
            self.db.execute('INSERT INTO commands VALUES (?,?,?)', (command_id, digest, encode_json(result)))
            self.db.commit()
            return True, result

    def finish(self, command_id, result):
        with self.lock:
            self.db.execute('UPDATE commands SET result=? WHERE id=?', (encode_json(result), command_id))
            self.db.commit()


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.values = {'state': 'UNKNOWN', 'state_detail': '', 'phase': '',
                       'description': '', 'failure': '', 'detection_summary': '',
                       'target_cache': '{}', 'queue_total': 0,
                       **{'queue_' + s: 0 for s in ('candidate', 'ready', 'reserved', 'done', 'review')}}
        self.received = {}

    def update(self, kind, raw, now=None):
        now = time.monotonic() if now is None else now
        if len(raw.encode('utf-8')) > 240000:
            raise ValueError('ROS payload too large')
        values = {}
        if kind == 'state':
            state, _, detail = raw.partition('|')
            values = {'state': state.strip()[:64], 'state_detail': detail.strip()[:2000]}
        elif kind == 'progress':
            data = decode_json(raw)
            if not isinstance(data, dict) or not isinstance(data.get('phase'), str) or not isinstance(data.get('description'), str):
                raise ValueError('invalid progress')
            values = {'phase': data['phase'][:128], 'description': data['description'][:2000]}
        elif kind == 'target_cache':
            data = decode_json(raw)
            if not isinstance(data, dict) or data.get('schema_version') != 2 or not isinstance(data.get('targets'), dict):
                raise ValueError('expected target_cache schema 2')
            targets = data['targets']
            if len(targets) > 200 or any(not isinstance(t, dict) for t in targets.values()):
                raise ValueError('invalid targets')
            counts = {s: 0 for s in ('candidate', 'ready', 'reserved', 'done', 'review')}
            # A compact, bounded snapshot; never upload a camera frame or point cloud.
            rows = []
            for target in targets.values():
                status = target.get('status', '')
                if status in counts:
                    counts[status] += 1
                row = {key: target.get(key) for key in ('id', 'category', 'status', 'detection_confidence')}
                if not isinstance(row['category'], str) or len(row['category']) > 256:
                    raise ValueError('invalid category')
                confidence = row['detection_confidence']
                if confidence is not None and (type(confidence) not in (int, float) or not math.isfinite(confidence)):
                    raise ValueError('invalid confidence')
                rows.append(row)
            compact = encode_json({'targets': rows})
            if len(compact.encode('utf-8')) > 40000:
                raise ValueError('compact target snapshot too large')
            values = {'target_cache': compact, 'queue_total': len(rows),
                      **{'queue_' + s: count for s, count in counts.items()}}
        elif kind in ('failure', 'detection_summary'):
            values = {kind: raw[:2000]}
        else:
            raise ValueError('unknown telemetry kind')
        with self.lock:
            self.values.update(values)
            self.received[kind] = now

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            return {**self.values, **{kind + '_age_s': round(max(0, now - self.received[kind]), 1)
                                      if kind in self.received else -1
                                      for kind in ('state', 'progress', 'target_cache')}}
