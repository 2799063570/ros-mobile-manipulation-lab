"""Explicit denominators and independent labels for thesis measurements (stdlib)."""
import math
import statistics
from collections import defaultdict


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('nonfinite measurement')
    return value


def quantile(values, p):
    values = sorted(values)
    if not values:
        return None
    x = (len(values)-1)*p
    a = int(x)
    return values[a] + (values[min(a+1, len(values)-1)]-values[a])*(x-a)


def describe(values):
    values = [finite(v) for v in values]
    if not values:
        return {'n': 0, 'mean': None, 'rmse': None, 'median': None, 'p95': None, 'max_abs': None}
    return {'n': len(values), 'mean': statistics.mean(values),
            'rmse': math.sqrt(statistics.mean(v*v for v in values)),
            'median': quantile(values, .5), 'p95': quantile(values, .95),
            'max_abs': max(abs(v) for v in values)}


def boolean(value):
    if str(value).strip().lower() in ('1', 'true'):
        return True
    if str(value).strip().lower() in ('0', 'false'):
        return False
    raise ValueError('boolean label must be 0/1 or false/true')


def localization(rows, angle_threshold_deg):
    """Rows include independent ground truth and failures, not just detections."""
    threshold = finite(angle_threshold_deg)
    if not 0 < threshold <= 90:
        raise ValueError('angle threshold must be in (0,90] degrees')
    seen, xyz, yaw, width = set(), [[], [], []], [], []
    valid = correct_class = correct_angle = 0
    for row in rows:
        key = row['sample_id']
        if not key or key in seen:
            raise ValueError('missing or duplicate sample_id')
        seen.add(key)
        if not row['true_class']:
            raise ValueError('independent true_class is required')
        # Require a recorded class decision even for failed geometry.
        correct_class += row['pred_class'] == row['true_class']
        if not boolean(row['valid']):
            continue
        if row['frame_id'] != row['true_frame_id'] or not row['frame_id']:
            raise ValueError('prediction and truth must use the same explicit frame')
        valid += 1
        for i, axis in enumerate('xyz'):
            xyz[i].append(finite(row['pred_'+axis])-finite(row['true_'+axis]))
        # Short-side gripper direction has pi-periodicity, not 2*pi.
        error = abs((finite(row['pred_yaw_rad'])-finite(row['true_yaw_rad'])+math.pi/2) % math.pi-math.pi/2)
        yaw.append(math.degrees(error))
        correct_angle += math.degrees(error) <= threshold
        predicted_width, true_width = finite(row['pred_width_m']), finite(row['true_width_m'])
        if min(predicted_width,true_width) <= 0:
            raise ValueError('valid geometry needs positive metric widths')
        width.append(predicted_width-true_width)
    n = len(seen)
    distance = [math.sqrt(sum(v*v for v in p)) for p in zip(*xyz)]
    return {'attempts': n, 'geometry_valid': valid, 'geometry_coverage': valid/n if n else None,
            'classification_correct': correct_class, 'classification_rate_all': correct_class/n if n else None,
            'position_units': 'm', 'axis_signed_error': dict(zip('xyz', map(describe, xyz))),
            'position_distance': describe(distance), 'yaw_abs_error_deg': describe(yaw),
            'angle_threshold_deg': threshold, 'angle_correct_all': correct_angle/n if n else None,
            'angle_correct_valid_only': correct_angle/valid if valid else None,
            'width_signed_error_m': describe(width)}


def summarize_events(events, annotations=()):
    begins, ends, spans, span_begins, labels = {}, {}, {}, {}, {}
    tracking = defaultdict(list)
    for row in annotations:
        key = row['attempt_id']
        if key in labels:
            raise ValueError('duplicate attempt annotation')
        labels[key] = {k: boolean(row[k]) for k in ('class_correct','grasp_success','placed_correct')}
    for event in events:
        kind = event.get('kind')
        if kind in ('attempt_begin', 'attempt_end'):
            target = begins if kind == 'attempt_begin' else ends
            key = event['attempt_id']
            if key in target:
                raise ValueError('duplicate attempt boundary: '+key)
            target[key] = event
        elif kind in ('stage_begin', 'stage_end'):
            key = (event['run_id'], event['span_id'])
            target = span_begins if kind == 'stage_begin' else spans
            if key in target:
                raise ValueError('duplicate span boundary')
            target[key] = event
        elif kind == 'joint_feedback':
            if len(event['joint_names']) != len(event['error_positions']):
                raise ValueError('joint feedback names/error dimensions differ')
            for name, value in zip(event['joint_names'], event['error_positions']):
                tracking[name].append(finite(value))
    if set(labels)-set(begins):
        raise ValueError('annotations refer to unrecorded attempts')
    diagnostics = {'incomplete_attempts': sorted(set(begins)-set(ends)),
                   'end_without_begin': sorted(set(ends)-set(begins)),
                   'incomplete_spans': len(set(span_begins)-set(spans)),
                   'span_end_without_begin': len(set(spans)-set(span_begins))}
    grouped = defaultdict(list)
    for key, begin in begins.items():
        grouped[begin['condition']].append((key, ends.get(key)))
    children = defaultdict(float)
    for key, span in spans.items():
        if span.get('parent_span_id'):
            children[(span['run_id'], span['parent_span_id'])] += finite(span['elapsed_s'])
    stage_values = defaultdict(lambda: {'inclusive': [], 'exclusive': []})
    for key, span in spans.items():
        inclusive = finite(span['elapsed_s'])
        if inclusive < 0:
            raise ValueError('negative stage duration')
        exclusive = inclusive-children[key]
        if exclusive < -1e-6:
            raise ValueError('child durations exceed parent duration')
        group = stage_values[(span['condition'], span['stage'])]
        group['inclusive'].append(inclusive)
        group['exclusive'].append(max(0.0, exclusive))
    summary = {}
    for condition, attempts in grouped.items():
        n = len(attempts)
        annotated = [labels[key] for key, _ in attempts if key in labels]
        controller_ok = sum(end is not None and end['status'] == 'controller_success' for _, end in attempts)
        actual = sum(all(label.values()) for label in annotated)
        summary[condition] = {'attempts': n, 'controller_success': controller_ok,
            'controller_completion_rate': controller_ok/n, 'independently_annotated': len(annotated),
            'annotation_coverage': len(annotated)/n,
            'verified_sort_successes': actual,
            'verified_sort_rate': actual/n if len(annotated) == n else None,
            'verified_sort_rate_annotated_subset': actual/len(annotated) if annotated else None}
    return {'conditions': summary, 'diagnostics': diagnostics,
            'stages': [{'condition': c, 'stage': s, 'inclusive_s': describe(v['inclusive']),
                        'exclusive_s': describe(v['exclusive'])} for (c,s), v in sorted(stage_values.items())],
            'tracking_error_rad': {k: describe(v) for k,v in sorted(tracking.items())},
            'tracking_aggregation': 'pooled action-feedback samples; not equal-weight trajectory means',
            'timing_scope': 'operation includes waits/recovery; pick_place begins after target selection; nested durations must not be added'}
