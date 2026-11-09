"""Huawei wire protocol. Pure functions are usable without ROS or cloud credentials."""
from datetime import datetime, timezone
import hashlib
import hmac
import re
import uuid
from aubo_cloud_bridge.protocol import decode_json, encode_json, METHODS

SERVICE = 'Sorting'
COMMAND = 'task_command'
MAX_REPORT_BYTES = 3800
TEXT_LIMITS = {'state': 64, 'state_detail': 160, 'phase': 96, 'description': 160,
               'failure': 160, 'detection_summary': 160}
QUEUES = ('total', 'candidate', 'ready', 'reserved', 'done', 'review')


def limited(value, length):
    text = ''.join(char if ord(char) >= 32 else ' ' for char in str(value))
    return text.encode('utf-8')[:length].decode('utf-8', errors='ignore')


def credentials(device_id, secret, stamp=None):
    if not isinstance(device_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', device_id):
        raise ValueError('invalid_device_id')
    if not isinstance(secret, str) or not secret or secret.startswith('YOUR_'):
        raise ValueError('missing_device_secret')
    stamp = stamp or datetime.now(timezone.utc).strftime('%Y%m%d%H')
    # Signature type 0 is supported by IoTDA and survives reconnects after the UTC hour changes.
    password = hmac.new(stamp.encode('ascii'), secret.encode('utf-8'), hashlib.sha256).hexdigest()
    return device_id + '_0_0_' + stamp, device_id, password


def topics(device_id):
    base = '$oc/devices/' + device_id + '/sys/'
    return {'report': base + 'properties/report', 'subscription': base + 'commands/#',
            'request_prefix': base + 'commands/request_id=',
            'reply_prefix': base + 'commands/response/request_id='}


def request_id(topic, prefix):
    if not topic.startswith(prefix):
        raise ValueError('not_a_command_topic')
    token = topic[len(prefix):]
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', token):
        raise ValueError('invalid_request_id')
    return token


def decode_command(message):
    if (not isinstance(message, dict) or message.get('service_id') != SERVICE
            or message.get('command_name') != COMMAND or 'object_device_id' in message):
        raise ValueError('invalid_service_or_command')
    paras = message.get('paras')
    if not isinstance(paras, dict) or set(paras) != {
            'method', 'command_id', 'session', 'issued_at_ms', 'expires_at_ms'}:
        raise ValueError('invalid_command_parameters')
    params = {key: value for key, value in paras.items() if key != 'method'}
    # Cloud model uses string timestamps to preserve 13-digit milliseconds in console forms.
    for key in ('issued_at_ms', 'expires_at_ms'):
        value = params[key]
        if not isinstance(value, str) or not re.fullmatch(r'[0-9]{13}', value):
            raise ValueError('invalid_timestamp')
        params[key] = int(value)
    return {'method': paras['method'], 'params': params}


def make_command(method, session, now_ms, ttl=25):
    if method not in METHODS or not re.fullmatch(r'[a-f0-9]{32}', session or '') or not 1 <= ttl <= 30:
        raise ValueError('invalid_command')
    return {'service_id': SERVICE, 'command_name': COMMAND, 'paras': {
        'method': method, 'command_id': uuid.uuid4().hex, 'session': session,
        'issued_at_ms': str(now_ms), 'expires_at_ms': str(now_ms + ttl * 1000)}}


def result_summary(result):
    limits = {'command_id': 80, 'method': 32, 'status': 16, 'reason': 100, 'message': 160}
    summary = {key: limited(value, limits[key]) if key in limits else value
               for key, value in result.items() if key in limits or key in ('duplicate', 'wall_stamp_ms')}
    for key in ('message', 'reason'):
        while len(encode_json(summary).encode()) > 512 and summary.get(key):
            summary[key] = limited(summary[key], len(summary[key].encode()) // 2)
    return summary


def command_reply(result):
    return {'result_code': {'accepted': 0, 'rejected': 1, 'unknown': 2}.get(result.get('status'), 2),
            'response_name': 'COMMAND_RESPONSE', 'paras': {'result': encode_json(result_summary(result))}}


def property_report(snapshot, session, available, enabled, last, now_ms):
    values = {key: limited(snapshot.get(key, ''), length) for key, length in TEXT_LIMITS.items()}
    values.update({'queue_' + key: int(snapshot.get('queue_' + key, 0)) for key in QUEUES})
    values.update(bridge_wall_ms=str(now_ms), bridge_session=session,
                  commands_enabled=int(enabled), sorting_available=int(available),
                  target_cache_age_s=min(2147483647, float(snapshot.get('target_cache_age_s', -1))),
                  last_command=encode_json(result_summary(last)))
    rows = decode_json(snapshot.get('target_cache', '{}')).get('targets', [])
    shown = []
    for row in rows:
        candidate = {**row, 'category': limited(row.get('category', ''), 64)}
        if len(encode_json({'targets': shown + [candidate]}).encode()) > 1200:
            break
        shown.append(candidate)
    values['target_cache'] = encode_json({'targets': shown})
    values['target_cache_truncated'] = int(len(shown) != len(rows))
    payload = {'services': [{'service_id': SERVICE, 'properties': values}]}
    # Escapes inside string-valued JSON can expand payloads. Keep the complete report under one 4KB unit.
    while len(encode_json(payload).encode()) > MAX_REPORT_BYTES:
        if shown:
            shown.pop()
            values['target_cache'] = encode_json({'targets': shown})
            values['target_cache_truncated'] = 1
        else:
            for key in ('state_detail', 'description', 'failure', 'detection_summary'):
                values[key] = limited(values[key], 32)
            if len(encode_json(payload).encode()) > MAX_REPORT_BYTES:
                raise ValueError('report_exceeds_one_billing_unit')
    return payload
