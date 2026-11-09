"""Generate an importable IoT Explorer product model; no account access."""
import argparse
import json
from pathlib import Path


def build(product_id='YOUR_PRODUCT_ID'):
    strings = {'state': 64, 'state_detail': 512, 'phase': 128, 'description': 512,
        'failure': 512, 'detection_summary': 512, 'bridge_wall_ms': 20,
        'bridge_session': 64, 'bridge_context': 512, 'target_cache': 2048, 'last_command': 1024}
    props = [{'id': key, 'name': key, 'desc': '', 'required': False, 'mode': 'r',
              'define': {'type': 'string', 'min': '0', 'max': str(limit)}}
             for key, limit in strings.items()]
    for key in ('commands_enabled', 'sorting_available', 'target_cache_truncated'):
        props.append({'id': key, 'name': '目标列表已截断' if key == 'target_cache_truncated' else key,
                      'required': False, 'mode': 'r',
                      'define': {'type': 'bool', 'mapping': {'0': '否', '1': '是'}}})
    for key in ['queue_' + k for k in ('total', 'candidate', 'ready', 'reserved', 'done', 'review')] + ['target_cache_shown']:
        props.append({'id': key, 'name': key, 'required': False, 'mode': 'r', 'define': {
            'type': 'int', 'min': '0', 'max': '200', 'start': '0', 'step': '1', 'unit': ''}})
    props.append({'id': 'target_cache_age_s', 'name': '快照接收年龄', 'required': False, 'mode': 'r',
                  'define': {'type': 'float', 'min': '-1', 'max': '2147483647',
                             'start': '-1', 'step': '0.1', 'unit': 's'}})
    return {'version': '1.0', 'profile': {'ProductId': product_id}, 'properties': props, 'events': [],
        'actions': [{'id': 'task_command', 'name': '分拣任务请求', 'desc': '接收反馈不表示机械动作完成',
            'input': [{'id': 'request', 'name': '带会话及有效期的请求',
                       'define': {'type': 'string', 'min': '1', 'max': '1024'}}],
            'output': [{'id': 'result', 'name': '任务请求反馈',
                        'define': {'type': 'string', 'min': '1', 'max': '1024'}}]}]}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--product-id', default='YOUR_PRODUCT_ID')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('thing_model.json'))
    args = parser.parse_args()
    args.output.write_text(json.dumps(build(args.product_id), ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
