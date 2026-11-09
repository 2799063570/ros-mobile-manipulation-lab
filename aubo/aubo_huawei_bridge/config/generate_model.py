"""Generate IoTDA API request bodies and a console-uploadable product profile ZIP."""
import json
from pathlib import Path
import zipfile

DEVICE_TYPE = 'AUBOSorter'
MANUFACTURER_ID = 'AuboLab'
ARCHIVE_NAME = DEVICE_TYPE + '_' + MANUFACTURER_ID + '.zip'


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
    return {'name': 'AUBO_Sorting', 'device_type': DEVICE_TYPE, 'protocol_type': 'MQTT',
            'data_format': 'json', 'manufacturer_name': 'AUBO',
            'service_capabilities': [service()]}


def console_profiles():
    """The console ZIP uses camelCase fields; the application API uses snake_case."""
    capability = service()
    field_names = {'property_name': 'propertyName', 'data_type': 'dataType',
                   'max_length': 'maxLength', 'enum_list': 'enumList', 'para_name': 'paraName'}

    def parameter(value):
        defaults = dict(required=False, min=None, max=None, step=0.0,
                        maxLength=0, unit=None, enumList=None)
        return {**defaults, **{field_names.get(key, key): item for key, item in value.items()}}

    commands = [{'commandName': command['command_name'],
        'paras': [parameter(p) for p in command['paras']],
        'responses': [{'responseName': response['response_name'],
                       'paras': [parameter(p) for p in response['paras']]}
                      for response in command.get('responses', [])]}
        for command in capability['commands']]
    device = {'devices': [{'manufacturerId': MANUFACTURER_ID, 'manufacturerName': 'AUBO',
        'model': 'AUBOSorting', 'protocolType': 'MQTT', 'deviceType': DEVICE_TYPE,
        'serviceTypeCapabilities': [{'serviceId': capability['service_id'],
            'serviceType': capability['service_type'], 'option': 'Mandatory'}], 'omCapability': None}]}
    services = {'services': [{'serviceType': capability['service_type'],
        'description': 'AUBO sorting telemetry and bounded task commands',
        'properties': [parameter(p) for p in capability['properties']], 'commands': commands, 'events': []}]}
    return {'profile/devicetype-capability.json': device,
            'service/Sorting/profile/servicetype-capability.json': services}


def write_artifacts(root):
    for name, value in (('create_product.json', product()),
                        ('update_product.json', {'service_capabilities': [service()]})):
        (root / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with zipfile.ZipFile(root / ARCHIVE_NAME, 'w') as archive:
        for name, value in console_profiles().items():
            # Stable metadata makes regeneration reproducible and avoids spurious binary changes.
            entry = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            entry.external_attr = 0o100644 << 16
            archive.writestr(entry, json.dumps(value, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    write_artifacts(Path(__file__).resolve().parent)
