"""aubo_delta_pose_v1: base-frame translation and left-composed rotation vector.

All utilities are pure Python. A candidate is a preview, never an executable
command; collision, IK and control ownership are outside this module.
"""
import math

SCHEMA = 'aubo_delta_pose_v1'


def vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError('%s must contain %d numbers' % (name, size))
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or
           not math.isfinite(x) for x in value):
        raise ValueError('%s must contain finite numbers' % name)
    return [float(x) for x in value]


def quaternion(value):
    q = vector(value, 4, 'quaternion xyzw')
    norm = math.sqrt(sum(x*x for x in q))
    if norm < 1e-12:
        raise ValueError('Zero quaternion')
    return [x/norm for x in q]


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def from_rotvec(r):
    angle = math.sqrt(sum(x*x for x in r))
    if angle < 1e-12:
        return [0., 0., 0., 1.]
    scale = math.sin(angle/2)/angle
    return [x*scale for x in r] + [math.cos(angle/2)]


def to_rotvec(q):
    q = quaternion(q)
    # q and -q encode the same rotation; choose the shortest arc.
    if q[3] < 0:
        q = [-x for x in q]
    length = math.sqrt(sum(x*x for x in q[:3]))
    if length < 1e-12:
        return [0., 0., 0.]
    scale = 2*math.atan2(length, q[3])/length
    return [x*scale for x in q[:3]]


def pose_delta(before, after, gripper):
    p0 = vector(before['position'], 3, 'before position')
    p1 = vector(after['position'], 3, 'after position')
    q0, q1 = quaternion(before['orientation']), quaternion(after['orientation'])
    g = vector([gripper], 1, 'gripper')[0]
    if not 0 <= g <= 1:
        raise ValueError('Gripper must be in [0, 1]')
    rotation = to_rotvec(multiply(q1, [-q0[0], -q0[1], -q0[2], q0[3]]))
    return [b-a for a, b in zip(p0, p1)] + rotation + [g]


def candidate(pose, action, bounds_min, bounds_max, max_translation, max_rotation):
    action = vector(action, 7, 'action')
    p = vector(pose['position'], 3, 'TCP position')
    q = quaternion(pose['orientation'])
    low, high = vector(bounds_min, 3, 'bounds_min'), vector(bounds_max, 3, 'bounds_max')
    limits = vector([max_translation, max_rotation], 2, 'step limits')
    if any(a >= b for a, b in zip(low, high)) or min(limits) <= 0:
        raise ValueError('Invalid workspace or step limits')
    if not 0 <= action[6] <= 1:
        raise ValueError('Gripper must be in [0, 1]')
    if math.sqrt(sum(x*x for x in action[:3])) > max_translation:
        raise ValueError('Translation step exceeds limit')
    if math.sqrt(sum(x*x for x in action[3:6])) > max_rotation:
        raise ValueError('Rotation step exceeds limit')
    target = [x+dx for x, dx in zip(p, action[:3])]
    if any(not a <= x <= b for a, x, b in zip(low, p, high)):
        raise ValueError('Current TCP outside workspace')
    if any(not a <= x <= b for a, x, b in zip(low, target, high)):
        raise ValueError('Candidate outside workspace')
    return dict(position=target,
                orientation=quaternion(multiply(from_rotvec(action[3:6]), q)),
                gripper=action[6])


def validate_model_contract(result, revision, unnorm_key, action_dt):
    if result.get('mock') is not False or result.get('observation_only') is not True:
        raise ValueError('Only real observation results may be previewed')
    if result.get('action_schema') != SCHEMA:
        raise ValueError('Model has not declared the AUBO action schema')
    if not revision or not unnorm_key or unnorm_key == 'bridge_orig':
        raise ValueError('Configure a verified AUBO revision and statistics key')
    if result.get('revision') != revision or result.get('unnorm_key') != unnorm_key:
        raise ValueError('Model revision or statistics key mismatch')
    expected = vector([action_dt], 1, 'action_dt')[0]
    actual = vector([result.get('action_dt')], 1, 'model action_dt')[0]
    if expected <= 0 or abs(actual-expected) > 1e-6:
        raise ValueError('Model action horizon mismatch')
