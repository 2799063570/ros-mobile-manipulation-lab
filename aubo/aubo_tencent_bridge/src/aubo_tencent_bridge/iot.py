"""IoT Explorer thing-model envelopes and bounded UTF-8 property values."""
import re
from aubo_cloud_bridge.protocol import decode_json, encode_json

ACTION = 'task_command'
STRING_LIMITS = {
    'state': 64, 'state_detail': 512, 'phase': 128, 'description': 512,
    'failure': 512, 'detection_summary': 512,
}
QUEUES = ('total', 'candidate', 'ready', 'reserved', 'done', 'review')


def utf8_limit(value, limit):
    return str(value).encode('utf-8')[:limit].decode('utf-8', errors='ignore')


def bounded_result(result):
    # Preserve a parseable result, including the command identity, when ROS returns long text.
    return {key: (utf8_limit(value, 512) if isinstance(value, str) else value)
            for key, value in result.items()
            if key in ('command_id', 'method', 'status', 'message', 'reason', 'duplicate', 'wall_stamp_ms')}


def properties(snapshot, session, available, enabled, last_command, now_ms):
    result = {key: utf8_limit(snapshot.get(key, ''), limit) for key, limit in STRING_LIMITS.items()}
    result.update({'queue_' + key: int(snapshot.get('queue_' + key, 0)) for key in QUEUES})
    result.update(bridge_wall_ms=str(now_ms), bridge_session=session,
                  sorting_available=int(available), commands_enabled=int(enabled),
                  target_cache_age_s=float(snapshot.get('target_cache_age_s', -1)),
                  last_command=encode_json(bounded_result(last_command)))
    # All control eligibility fields live in one property to avoid mixing cloud snapshots.
    result['bridge_context'] = encode_json(dict(session=session, wall_ms=now_ms,
        available=bool(available), enabled=bool(enabled), state=result['state']))
    source = decode_json(snapshot.get('target_cache', '{}')).get('targets', [])
    rows = []
    for row in source:
        candidate = {**row, 'category': utf8_limit(row.get('category', ''), 80)}
        if len(encode_json({'targets': rows + [candidate]}).encode('utf-8')) > 2048:
            break
        rows.append(candidate)
    result['target_cache'] = encode_json({'targets': rows})
    result['target_cache_shown'] = len(rows)
    result['target_cache_truncated'] = int(len(rows) != len(source))
    return result


def topics(product_id, device_name):
    return {key: '$thing/%s/%s/%s/%s' % (direction, kind, product_id, device_name)
            for key, direction, kind in (
                ('report', 'up', 'property'), ('report_reply', 'down', 'property'),
                ('request', 'down', 'action'), ('reply', 'up', 'action'))}


def valid_token(value):
    return isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_:.-]{1,128}', value) is not None


def action_request(message):
    if (not isinstance(message, dict) or message.get('method') != 'action'
            or message.get('actionId') != ACTION or not valid_token(message.get('clientToken'))):
        raise ValueError('invalid_action')
    params = message.get('params')
    if not isinstance(params, dict) or set(params) != {'request'}:
        raise ValueError('invalid_action_input')
    raw = params['request']
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > 1024:
        raise ValueError('invalid_action_input')
    document = decode_json(raw)
    if not isinstance(document, dict):
        raise ValueError('invalid_command')
    return message['clientToken'], document


def action_reply(token, result):
    return {'method': 'action_reply', 'clientToken': token, 'code': 0, 'status': '',
            'response': {'result': encode_json(bounded_result(result))}}
