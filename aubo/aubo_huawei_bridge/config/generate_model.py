"""Generate IoTDA CreateProduct/UpdateProduct API request bodies, not a console ZIP archive."""
import json
from pathlib import Path


def service():
    strings = {'state': 64, 'state_detail': 160, 'phase': 96, 'description': 160,
        'failure': 160, 'detection_summary': 160, 'bridge_wall_ms': 20,
        'bridge_session': 32, 'target_cache': 1200, 'last_command': 512}
    props = [{'property_name': name, 'data_type': 'string', 'method': 'R',
              'required': False, 'max_length': size} for name, size in strings.items()]
    for name in ('commands_enabled', 'sorting_available', 'target_cache_truncated'):
        props.append({'property_name': name, 'data_type': 'int', 'method': 'R',
                      'required': False, 'min': '0', 'max': '1'})
    for kind in ('total', 'candidate', 'ready', 'reserved', 'done', 'review'):
        props.append({'property_name': 'queue_' + kind, 'data_type': 'int', 'method': 'R',
                      'required': False, 'min': '0', 'max': '200'})
    props.append({'property_name': 'target_cache_age_s', 'data_type': 'decimal', 'method': 'R',
                  'required': False, 'min': '-1', 'max': '2147483647', 'step': .1})
    inputs = [{'para_name': name, 'data_type': 'string', 'required': True, 'max_length': length}
              for name, length in (('method', 32), ('command_id', 80), ('session', 32),
                                   ('issued_at_ms', 13), ('expires_at_ms', 13))]
    return {'service_id': 'Sorting', 'service_type': 'Sorting', 'properties': props,
        'commands': [{'command_name': 'task_command', 'paras': inputs,
            'responses': [{'response_name': 'COMMAND_RESPONSE', 'paras': [
                {'para_name': 'result', 'data_type': 'string', 'required': True, 'max_length': 512}]}]}]}


def product():
    return {'name': 'AUBO_Sorting', 'device_type': 'AUBOSorter', 'protocol_type': 'MQTT',
            'data_format': 'json', 'manufacturer_name': 'AUBO',
            'service_capabilities': [service()]}


if __name__ == '__main__':
    root = Path(__file__).resolve().parent
    for name, value in (('create_product.json', product()),
                        ('update_product.json', {'service_capabilities': [service()]})):
        (root / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
