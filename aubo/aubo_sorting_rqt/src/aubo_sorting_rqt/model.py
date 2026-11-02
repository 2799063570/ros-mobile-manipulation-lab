"""Presentation rules shared by the panel and offline tests (no ROS imports)."""
import json
import math

STATES = {
    'INITIALIZING': '初始化', 'IDLE': '等待操作', 'READY': '已就绪',
    'OBSERVING': '前往观察位', 'DETECTING': '识别与队列', 'SORTING': '分拣中',
    'PICKING': '抓放执行', 'OPENING': '夹爪全开', 'PREPARING': '前往工作位',
    'HOMING': '返回结束位', 'STOPPED': '已停止', 'ERROR': '任务失败',
}
IDLE_STATES = frozenset(('IDLE', 'READY', 'STOPPED', 'ERROR'))
QUEUE_STATES = {'candidate': '待确认', 'ready': '可领取', 'reserved': '执行中',
                'done': '已完成', 'review': '待复核'}
PHASES = {'OPEN_GRIPPER': '夹爪全开', 'PREGRASP': '接近目标', 'DESCEND': '下降抓取',
          'GRASP': '夹紧目标', 'LIFT': '抬升目标', 'PREPLACE': '前往容器',
          'PLACE': '下降放置', 'RELEASE': '全开释放目标', 'RETREAT': '退离容器',
          'PICK_DONE': '本次抓放指令完成', **STATES}
ACTIONS = (
    ('move_to_observation', '前往观察位'), ('start', '开始队列分拣'),
    ('stop', '停止任务'), ('prepare_work', '前往工作位'),
    ('home', '返回结束位'), ('open_gripper', '夹爪全开'),
)
# key, label, minimum, maximum, step, decimal places
PARAMETERS = (
    ('velocity_scaling', '运动速度比例', .01, 1., .01, 2),
    ('acceleration_scaling', '运动加速度比例', .01, 1., .01, 2),
    ('grasp_offset_x', '抓取 X 修正 / m', -.05, .05, .001, 3),
    ('grasp_offset_y', '抓取 Y 修正 / m', -.05, .05, .001, 3),
    ('minimum_gripper_height', '夹爪 TCP 最低离桌 / m', 0., .5, .005, 3),
    ('detection_timeout', '检测等待上限 / s', 1., 120., 1., 1),
    ('target_cache_min_observations', '队列稳定确认帧数', 2, 30, 1, 0),
    ('queue_max_age', '待领取目标有效期 / s', .2, 30., .2, 1),
    ('queue_merge_distance', '重复目标合并距离 / m', .001, .04, .001, 3),
    ('queue_merge_overlap', '较小框面积重叠比例', .05, 1., .05, 2),
)


def add_plugin_arguments(parser):
    parser.add_argument('--task-node', default='/color_sorting_task')
    parser.add_argument('--sorting-ns', default='/sorting')
    parser.add_argument('--image-topic', default='/sorting/debug_image')
    parser.add_argument('--hardware-topic', default='/aubo_i5/aubo_hw_node/flow_diagnostics')


def state_code(message):
    return str(message).partition('|')[0].strip()


def available_actions(state, connected, pending=False, hardware_fault=False):
    result = {key: False for key, _ in ACTIONS}
    # Stop remains independently callable even when another service hangs.
    result['stop'] = connected
    if connected and not pending and not hardware_fault and state in IDLE_STATES:
        for key in result:
            result[key] = key != 'start' or state == 'READY'
    return result


def queue_rows(message):
    document = json.loads(message) if isinstance(message, str) else message
    if not isinstance(document, dict) or not isinstance(document.get('targets'), dict):
        raise ValueError('目标队列消息缺少 targets 对象')
    result = []
    for key, value in document['targets'].items():
        if not isinstance(value, dict):
            raise ValueError('目标记录格式无效')
        position = value.get('position', [])
        if len(position) != 3 or not all(math.isfinite(float(v)) for v in position):
            raise ValueError('目标坐标无效')
        row = dict(value)
        row.update(id=int(value.get('id', key)), position=[float(v) for v in position])
        row['category'] = str(value.get('category', value.get('color', '')))
        row['age'] = float(value.get('age', 0.))
        row['observations'] = int(value.get('observations', 0))
        if 'detection_confidence' in row:
            row['detection_confidence']=float(row['detection_confidence'])
            if not math.isfinite(row['detection_confidence']) or not 0<=row['detection_confidence']<=1:
                raise ValueError('检测置信度无效')
        if row['id'] < 0 or not math.isfinite(row['age']) or row['age'] < 0 or row['observations'] < 0:
            raise ValueError('目标编号、年龄或确认帧数无效')
        for field in ('grasp_angle', 'grasp_width'):
            number = None
            raw = row.get(field)
            if row.get('grasp_geometry_valid') is True and raw is not None and not isinstance(raw, bool):
                try:
                    candidate = float(raw)
                    if math.isfinite(candidate) and (field != 'grasp_width' or candidate > 0):
                        number = candidate
                except (ValueError, TypeError, OverflowError):
                    pass
            row[field] = number
        result.append(row)
    return document.get('frame_id', ''), sorted(result, key=lambda row: row['id'])


def parameter_changes(requested, accepted):
    return [key for key, value in requested.items()
            if key not in accepted or not math.isclose(float(value), float(accepted[key]),
                                                       rel_tol=1e-9, abs_tol=1e-9)]


def process_step(state, progress=None):
    if state == 'OBSERVING':
        return 0
    if state in ('DETECTING', 'SORTING'):
        return 1
    if state == 'PICKING':
        if (progress or {}).get('phase') in ('PREPLACE', 'PLACE', 'RELEASE', 'RETREAT', 'PICK_DONE'):
            return 3
        description = (progress or {}).get('description', '')
        return 3 if any(word in description for word in ('pre-place', ' place', 'retreat')) else 2
    return 4 if state == 'HOMING' else -1
