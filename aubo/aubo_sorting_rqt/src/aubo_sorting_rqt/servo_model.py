"""Servo UI units, action gates and approximate sight-line geometry; no ROS IO."""
import math

# key, label, display minimum/maximum/step/decimals, display units per SI unit
PARAMETERS = (
    ('linear_gain', '位置增益', 0., 5., .05, 2, 1.),
    ('angular_gain', '姿态增益', 0., 5., .05, 2, 1.),
    ('max_linear_velocity', '最大线速度 / mm/s', 0., 100., 1., 1, 1000.),
    ('max_angular_velocity', '最大角速度 / °/s', 0., 30., .5, 1, 180./math.pi),
    ('position_deadband', '位置死区 / mm', 0., 100., .5, 1, 1000.),
    ('orientation_deadband', '姿态死区 / °', 0., 28., .5, 1, 180./math.pi),
    ('alignment_hold_time', '对齐稳定时间 / s', 0., 5., .05, 2, 1.),
    ('dls_lambda', '雅可比阻尼', .001, 1., .005, 3, 1.),
    ('target_timeout', '目标超时 / ms', 10., 1000., 10., 0, 1000.),
    ('coast_duration', '丢失后速度衰减时段 / ms', 0., 2000., 10., 0, 1000.),
    ('coast_decay_time', '速度衰减时间常数 / ms', 10., 1000., 10., 0, 1000.),
    ('recovery_delay', '衰减后保持时间 / ms', 0., 5000., 10., 0, 1000.),
    ('target_offset_x', 'TCP 相对目标 X / mm', -300., 300., 5., 1, 1000.),
    ('target_offset_y', 'TCP 相对目标 Y / mm', -300., 300., 5., 1, 1000.),
    ('target_offset_z', 'TCP 高于表面 / mm', 0., 500., 5., 1, 1000.),
    ('desired_roll', 'TCP Roll / °', -180., 180., 5., 1, 180./math.pi),
    ('desired_pitch', 'TCP Pitch / °', -180., 180., 5., 1, 180./math.pi),
    ('desired_yaw', 'TCP Yaw / °', -180., 180., 5., 1, 180./math.pi),
)
POSE_KEYS = frozenset(('target_offset_x', 'target_offset_y', 'target_offset_z',
                       'desired_roll', 'desired_pitch', 'desired_yaw', 'use_orientation_control'))
STATES = {'DISABLED': '已禁用', 'WAITING_FOR_SDK': '等待 SDK 就绪', 'WAITING': '等待目标',
          'TRACKING': '跟踪中', 'ALIGNED': '已对齐', 'STOPPING': '减速中',
          'HOLD': '保持 / 保护', 'COAST': '丢失后减速', 'SEARCH_INITIAL': '前往观察位',
          'SEARCH_RECOVERY': '恢复搜索', 'PLANNING': '规划中', 'APPROACH': '接近中'}
STATES.update(MOVE_TO_OBSERVATION='前往保存的观测位', RETURN_TO_OBSERVATION='目标丢失，返回观测位')


def add_servo_arguments(parser):
    parser.add_argument('--controller-node', default='/aubo_visual_servo')
    parser.add_argument('--servo-ns', default='/visual_servo')
    parser.add_argument('--image-topic', default='/visual_servo/debug_image')
    parser.add_argument('--geometry-node', default='/grasp_geometry')


def action_gates(data, now):
    elapsed = now-data.get('status_at', -math.inf)
    fresh = data.get('connected', False) and elapsed < .75
    status = data.get('status') or {}
    healthy = fresh and status.get('feedback_valid', False) and status.get('feedback_age', 0.)+elapsed <= \
        data.get('cell', {}).get('feedback_timeout', .15)
    target = status.get('target_valid', False) and status.get('target_age', 0.)+elapsed <= \
        data.get('config', {}).get('target_timeout', .2)
    services = data.get('services', set())
    idle = healthy and data.get('state') == 'DISABLED'
    observe_first = data.get('cell', {}).get('initial_search_enabled', False)
    can_track = target and not data.get('selection_pending') and not data.get('perception', '').startswith('DISABLED|')
    return dict(start=idle and (observe_first or can_track) and not data.get('pending') and
                'set_enabled' in services,
                stop='set_enabled' in services,
                reset=idle and not data.get('pending') and 'reset' in services,
                tune=fresh and bool(data.get('config')) and not data.get('pending'),
                pose=idle and bool(data.get('config')) and not data.get('pending'))


def vector(value):
    if len(value) != 3 or not all(math.isfinite(float(v)) for v in value):
        raise ValueError('几何坐标必须是三个有限值')
    return tuple(float(v) for v in value)


def sub(a, b): return tuple(x-y for x, y in zip(a, b))
def dot(a, b): return sum(x*y for x, y in zip(a, b))


def point_segment_distance2(p, a, b):
    u, w = sub(b, a), sub(p, a)
    t = max(0., min(1., dot(w, u)/dot(u, u))) if dot(u, u) > 1e-16 else 0.
    d = tuple(w[i]-t*u[i] for i in range(3))
    return dot(d, d)


def segment_distance(p, q, a, b):
    p, q, a, b = map(vector, (p, q, a, b))
    result = min(point_segment_distance2(p, a, b), point_segment_distance2(q, a, b),
                 point_segment_distance2(a, p, q), point_segment_distance2(b, p, q))
    u, v, w = sub(q, p), sub(b, a), sub(p, a)
    aa, bb, cc, dd, ee = dot(u, u), dot(u, v), dot(v, v), dot(u, w), dot(v, w)
    det = aa*cc-bb*bb
    if det > 1e-16:
        s, t = (bb*ee-cc*dd)/det, (aa*ee-bb*dd)/det
        if 0 <= s <= 1 and 0 <= t <= 1:
            d = tuple(w[i]+s*u[i]-t*v[i] for i in range(3))
            result = min(result, dot(d, d))
    return math.sqrt(max(0., result))


def sight_clearance(camera, target, links, radius=.06):
    if len(links) < 2 or not math.isfinite(radius) or radius <= 0:
        raise ValueError('缺少完整机械臂 TF 或无效包络半径')
    return min(segment_distance(camera, target, a, b)-radius for a, b in zip(links, links[1:]))


def downward_draft(yaw=math.pi/2):
    return dict(desired_roll=math.pi, desired_pitch=0., desired_yaw=yaw,
                use_orientation_control=True)


def visibility_draft(camera, target, height=.15, lateral=.08, current_yaw=0.):
    camera, target = vector(camera), vector(target)
    if not .05 <= height <= .5 or not 0. <= lateral <= .3:
        raise ValueError('草稿高度应为50–500 mm，侧移应为0–300 mm')
    dx, dy = camera[0]-target[0], camera[1]-target[1]
    distance = math.hypot(dx, dy)
    if camera[2] <= target[2]+.05 or distance < .01:
        raise ValueError('相机高度或水平视线不足，需手动选择侧移方向')
    phi = math.atan2(dy, dx)
    wrap = lambda angle: math.atan2(math.sin(angle), math.cos(angle))
    # The URDF gripper body is ~24 mm in X, ~86 mm in Y. Align its long
    # horizontal Y axis with the camera ray, choosing the nearer equivalent yaw.
    yaw = min((wrap(phi+math.pi/2), wrap(phi-math.pi/2)),
              key=lambda angle: abs(wrap(angle-current_yaw)))
    result = downward_draft(yaw)
    result.update(target_offset_x=-lateral*dx/distance,
                  target_offset_y=-lateral*dy/distance, target_offset_z=height)
    return result


def tool_axis(rpy):
    r, p, y = vector(rpy)
    return (math.cos(y)*math.sin(p)*math.cos(r)+math.sin(y)*math.sin(r),
            math.sin(y)*math.sin(p)*math.cos(r)-math.cos(y)*math.sin(r),
            math.cos(p)*math.cos(r))


def quaternion_rpy(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('无效姿态四元数')
    length = math.sqrt(sum(v*v for v in q))
    if length < 1e-9:
        raise ValueError('姿态四元数为零')
    x, y, z, w = (v/length for v in q)
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
