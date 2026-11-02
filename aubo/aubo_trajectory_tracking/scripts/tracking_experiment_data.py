#!/usr/bin/env python3
"""ROS/Qt-free storage, validation and comparison for tracking experiments."""
import csv
import hashlib
import json
import math
import statistics
import uuid
from pathlib import Path

TCP_FIELDS = ['time_s', 'reference_x_m', 'reference_y_m', 'reference_z_m',
              'actual_x_m', 'actual_y_m', 'actual_z_m', 'error_m', 'actual_speed_m_s',
              'reference_speed_m_s', 'command_speed_m_s', 'trend_speed_m_s',
              'feedback_age_s', 'phase_settling', 'tangential_error_m', 'normal_error_m']
JOINT_FIELDS = ['time_s'] + [kind+'_j'+str(j+1)+unit for kind, unit in
                           [('target', '_rad'), ('actual', '_rad'), ('error', '_rad'),
                            ('target_velocity', '_rad_s'), ('actual_velocity', '_rad_s')]
                           for j in range(6)] + ['target_interpolation_gap_s',
                           'backend_0_gazebo_1_ros_control_2_sdk', 'velocity_source_1_message_0_difference']
SERVO_FIELDS = ['time_s'] + [kind+'_'+axis+'_m_s' for kind in
                           ('requested', 'resolved', 'smoothed') for axis in 'xyz'] + [
                           'sigma_min', 'dls_lambda', 'velocity_scale',
                           'backend_0_gazebo_1_ros_control_2_sdk']
FIELDS = {'tcp': TCP_FIELDS, 'joints': JOINT_FIELDS, 'servo': SERVO_FIELDS}
TUNING = ('kp', 'ki', 'kd', 'acceleration_ff_time')
TERMINAL = ('COMPLETED', 'STOPPED', 'FAULT')


def write_json(path, data):
    path = Path(path)
    temp = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    temp.replace(path)


def normalize_metadata(data):
    """Boost property_tree serializes numbers as strings; normalize explicitly."""
    data = json.loads(json.dumps(data))
    if int(data.get('schema_version', 0)) != 1:
        raise ValueError('Unsupported frozen run metadata schema')
    data['run_start_ns'] = str(int(data['run_start_ns']))
    for key, value in data['parameters'].items():
        if key not in ('shape', 'plane'):
            data['parameters'][key] = float(value)
    for key in ('origin_m', 'start_orientation_xyzw'):
        if key in data:
            data[key] = [float(v) for v in data[key]]
    json.dumps(data, allow_nan=False)
    return data


def stats(values, scale=1.):
    values = [float(v)*scale for v in values]
    if not values:
        return {'n': 0, 'rms': None, 'peak_abs': None, 'mean': None, 'p95_abs': None}
    ordered = sorted(abs(v) for v in values)
    return dict(n=len(values), rms=math.sqrt(statistics.mean(v*v for v in values)),
                peak_abs=max(ordered), mean=statistics.mean(values),
                p95_abs=ordered[max(0, math.ceil(.95*len(ordered))-1)])


def read_rows(folder, kind):
    path = Path(folder)/(kind+'.csv')
    if not path.exists():
        return []
    rows = []
    with path.open() as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != FIELDS[kind]:
            raise ValueError('Invalid CSV schema: '+str(path))
        for record in reader:
            values = [float(record[k]) for k in FIELDS[kind]]
            if not all(math.isfinite(v) for v in values) or values[0] < 0:
                raise ValueError('Invalid numeric sample: '+str(path))
            if rows and values[0] <= rows[-1][0]:
                raise ValueError('Non-increasing sample time: '+str(path))
            rows.append(values)
    return rows


def radial_errors(rows, metadata):
    params = (metadata.get('frozen') or {}).get('parameters', {})
    origin = (metadata.get('frozen') or {}).get('origin_m')
    if params.get('shape') != 'circle' or origin is None:
        return []
    a, b = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}[params['plane']]
    r = float(params['radius'])
    return [math.hypot(row[4+a]-origin[a]+r, row[4+b]-origin[b])-r for row in rows]


def analyze_run(folder):
    folder = Path(folder)
    meta = json.loads((folder/'metadata.json').read_text())
    tcp = read_rows(folder, 'tcp')
    running = [r for r in tcp if r[13] == 0]
    settling = [r for r in tcp if r[13] == 1]
    params = (meta.get('frozen') or {}).get('parameters', meta.get('parameter_snapshot', {}))
    summary = {'run_id': meta['run_id'], 'folder': str(folder.resolve()),
               'label': meta.get('label', ''), 'state': meta.get('state', 'OPEN'),
               'parameters': params, 'parameter_source': 'frozen' if meta.get('frozen') else 'snapshot_only',
               'quality': [], 'tcp': {}}
    quality = summary['quality']
    for name, rows in [('all', tcp), ('running', running), ('settling', settling)]:
        peak = max(rows, key=lambda r: r[7]) if rows else None
        summary['tcp'][name] = dict(total_mm=stats([r[7] for r in rows], 1000),
            tangential_mm=stats([r[14] for r in rows], 1000),
            normal_mm=stats([r[15] for r in rows], 1000),
            radial_mm=stats(radial_errors(rows, meta), 1000),
            peak_time_s=peak[0] if peak else None,
            xyz_error_mm=[stats([r[1+j]-r[4+j] for r in rows], 1000) for j in range(3)])
    summary['feedback_age_ms'] = stats([r[12] for r in tcp], 1000)
    summary['tcp_interval_ms'] = stats([b[0]-a[0] for a, b in zip(tcp, tcp[1:])], 1000)
    summary['final_error_mm'] = tcp[-1][7]*1000 if tcp else None
    joints, servo = read_rows(folder, 'joints'), read_rows(folder, 'servo')
    summary['joints'] = {'n': len(joints), 'generated_minus_measured_rad': [
        stats([r[13+j] for r in joints]) for j in range(6)]}
    summary['joints']['running_generated_minus_measured_rad'] = [
        stats([r[13+j] for r in joints if r[0] < float(params.get('duration', 0))]) for j in range(6)]
    summary['servo'] = {'n': len(servo),
        'minimum_velocity_scale': min((r[12] for r in servo), default=None),
        'scaled_sample_fraction': statistics.mean(r[12] < .999 for r in servo) if servo else None,
        'minimum_sigma': min((r[10] for r in servo), default=None)}
    if not tcp:
        quality.append('no_tcp_samples')
    duration = float(params.get('duration', 0))
    if not running or running[0][0] > .1 or duration <= 0 or running[-1][0] < duration-.1:
        quality.append('incomplete_running_coverage')
    if not meta.get('frozen'):
        quality.append('parameters_not_frozen')
    if not (meta.get('frozen') or {}).get('origin_m'):
        quality.append('start_pose_unavailable')
    if meta.get('joined_late'):
        quality.append('collector_joined_late')
    if meta.get('queue_dropped', 0):
        quality.append('collector_queue_overflow')
    if any(meta.get('rejected', {}).values()):
        quality.append('rejected_samples')
    if not summary['state'].startswith('COMPLETED:'):
        quality.append('not_completed')
    if not joints:
        quality.append('no_joint_samples')
    if not servo:
        quality.append('no_servo_samples')
    for name, rows in [('joints', joints), ('servo', servo)]:
        if rows and (rows[0][0] > .2 or rows[-1][0] < duration-.1):
            quality.append(name+'_incomplete_coverage')
    if any(b[0]-a[0] > .1 for a,b in zip(running,running[1:])):
        quality.append('tcp_gap_over_100ms')
    if meta.get('close_reason') != 'terminal_tail_finished':
        quality.append('tail_not_complete')
    # Missing diagnostics do not turn a valid TCP capture into a successful
    # full experiment. All rows remain visible in comparison.csv.
    summary['eligible_for_aggregate'] = not quality
    summary['near_peak_logs'] = []
    peak_t = summary['tcp']['running']['peak_time_s']
    if peak_t is not None and (folder/'events.jsonl').exists():
        with (folder/'events.jsonl').open() as stream:
            for line in stream:
                event = json.loads(line)
                if event['kind'] != 'rosout':
                    continue
                payload = event['data']
                relative = payload['stamp_s']-int(meta['run_id'])*1e-9
                if abs(relative-peak_t) <= .25:
                    summary['near_peak_logs'].append({'time_s': relative, 'text': payload['text']})
        summary['near_peak_logs'] = summary['near_peak_logs'][-30:]
    write_json(folder/'summary.json', summary)
    return summary


class RunWriter:
    """Streaming files. Only the collector's worker thread calls this class."""
    def __init__(self, root, run_id, event, snapshot, dropped=0, label=''):
        self.folder = Path(root)/('run_'+run_id)
        self.folder.mkdir(exist_ok=False)
        self.meta = dict(schema_version=1, run_id=run_id, label=label, state='OPEN',
                         parameter_snapshot=snapshot.get('parameters', {}),
                         environment=snapshot.get('environment', {}),
                         joined_late=event['ros_s']-int(run_id)*1e-9 > .5,
                         collector_started_ros_s=event['ros_s'], frozen=None,
                         rejected={}, queue_dropped=0)
        self.drop_start = dropped
        self.deadline = None
        self.seen_running = False
        self.rows = {k: 0 for k in FIELDS}
        self.last = {k: -1. for k in FIELDS}
        self.handles = {k: (self.folder/(k+'.csv')).open('w', newline='') for k in FIELDS}
        self.writers = {k: csv.writer(self.handles[k]) for k in FIELDS}
        for k in FIELDS:
            self.writers[k].writerow(FIELDS[k])
        self.events = (self.folder/'events.jsonl').open('w')
        self.save_metadata()

    def save_metadata(self):
        write_json(self.folder/'metadata.json', self.meta)
        if self.meta.get('frozen'):
            frozen = self.meta['frozen']
            parameters = dict(frozen['parameters'])
            parameters.update({k: frozen[k] for k in ('base_link', 'tool_link') if k in frozen})
            # JSON is also valid YAML; roslaunch <rosparam> can load this file.
            write_json(self.folder/'frozen_curve.yaml', parameters)

    def reject(self, reason):
        rejected = self.meta['rejected']
        rejected[reason] = rejected.get(reason, 0)+1

    def sample(self, kind, values, ros_s):
        if len(values) != len(FIELDS[kind]) or not all(math.isfinite(v) for v in values):
            self.reject(kind+'_invalid')
            return
        # Relative timestamps catch queued samples from a previous run arriving
        # after a new run_start on a different ROS connection.
        if values[0] < 0 or values[0] > ros_s-int(self.meta['run_id'])*1e-9+.25:
            self.reject(kind+'_wrong_run_time')
            return
        if values[0] <= self.last[kind]:
            self.reject(kind+'_nonincreasing')
            return
        if kind == 'tcp' and (values[7] < 0 or values[13] not in (0, 1)):
            self.reject('tcp_invalid_phase_or_error')
            return
        self.writers[kind].writerow(values)
        self.last[kind] = values[0]
        self.rows[kind] += 1

    def event(self, event):
        try:
            line = json.dumps(event, ensure_ascii=False, allow_nan=False)
        except ValueError:
            self.reject('nonfinite_raw_event')
            return
        self.events.write(line+'\n')

    def flush(self):
        for handle in list(self.handles.values())+[self.events]:
            handle.flush()

    def close(self, reason, dropped):
        if self.meta['state'] == 'OPEN' or not self.meta['state'].startswith(tuple(x+':' for x in TERMINAL)):
            self.meta['state'] = reason
        self.meta['close_reason'] = reason
        self.meta['queue_dropped'] = max(0, dropped-self.drop_start)
        self.meta['sample_counts'] = self.rows
        self.save_metadata()
        for handle in list(self.handles.values())+[self.events]:
            handle.close()
        return analyze_run(self.folder)


class CaptureSession:
    def __init__(self, root, snapshot, tail=2.5, label=''):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.snapshot = snapshot
        self.tail = tail
        self.label = label
        self.active = None
        self.frozen = {}
        self.closed_ids = set()
        self.closed = []

    def finish(self, reason, dropped=0):
        if self.active:
            run_id = self.active.meta['run_id']
            summary = self.active.close(reason, dropped)
            self.closed.append(summary)
            self.closed_ids.add(run_id)
            self.active = None
            return summary

    def tick(self, steady_s, dropped=0):
        if self.active and self.active.deadline is not None and steady_s >= self.active.deadline:
            return self.finish('terminal_tail_finished', dropped)

    def consume(self, event, dropped=0):
        kind, data = event['kind'], event['data']
        if kind == 'metadata':
            metadata = normalize_metadata(data)
            run_id = metadata['run_start_ns']
            self.frozen[run_id] = metadata
            while len(self.frozen) > 16:
                del self.frozen[next(iter(self.frozen))]
            if self.active and run_id == self.active.meta['run_id']:
                self.active.meta['frozen'] = metadata
                self.active.save_metadata()
        elif kind == 'start':
            run_id = str(int(data))
            if run_id == '0':
                self.finish('PREVIEW_RESET', dropped)
                return
            if run_id in self.closed_ids or self.active and run_id == self.active.meta['run_id']:
                return
            self.finish('INTERRUPTED: new run started before tail finished', dropped)
            self.active = RunWriter(self.root, run_id, event, self.snapshot(), dropped, self.label)
            if run_id in self.frozen:
                self.active.meta['frozen'] = self.frozen[run_id]
                self.active.save_metadata()
        if not self.active:
            return
        if kind in FIELDS:
            if data:  # Empty arrays reset the UI; never delete captured files.
                self.active.sample(kind, data, event['ros_s'])
        else:
            self.active.event(event)
        if kind == 'state':
            if data.startswith(('RUNNING:', 'SETTLING:')):
                self.active.seen_running = True
            if data.startswith(tuple(x+':' for x in TERMINAL)) and (
                    self.active.seen_running or self.active.rows['tcp'] or self.active.meta['joined_late']):
                if self.active.deadline is None:
                    # Later idle Stop requests belong to preparation for the
                    # next run; retain this run's first terminal outcome/cause.
                    self.active.meta['state'] = data
                    self.active.deadline = event['steady_s']+self.tail


def _context(meta):
    frozen = meta.get('frozen') or {}
    params = frozen.get('parameters', meta.get('parameter_snapshot', {}))
    return {'parameters': {k: v for k, v in params.items() if k not in TUNING},
            'base_link': frozen.get('base_link'), 'tool_link': frozen.get('tool_link'),
            'environment': meta.get('environment', {})}


def comparison_numbers(value):
    """Ignore floating point roundoff from GUI steps, retain recorded values."""
    if isinstance(value, float):
        return float(format(value, '.12g'))
    if isinstance(value, dict):
        return {k: comparison_numbers(v) for k,v in value.items()}
    if isinstance(value, list):
        return [comparison_numbers(v) for v in value]
    return value


def compare_runs(inputs, output, plots=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    folders = set()
    missing_inputs = []
    for source in inputs:
        source = Path(source)
        if not source.exists():
            missing_inputs.append(str(source)+': input path does not exist')
            continue
        if (source/'metadata.json').exists():
            folders.add(source.resolve())
        else:
            folders.update(p.parent.resolve() for p in source.rglob('metadata.json'))
    summaries, failures = [], missing_inputs
    contexts = {}
    for folder in sorted(folders):
        try:
            meta = json.loads((folder/'metadata.json').read_text())
            if meta.get('state') == 'OPEN':
                failures.append(str(folder)+': still open; not compared')
                continue
            summary = analyze_run(folder)
            # Keep differing tasks/models/backends in separate comparison groups.
            context = json.dumps(comparison_numbers(_context(meta)), sort_keys=True, allow_nan=False)
            context_id = hashlib.sha256(context.encode()).hexdigest()[:10]
            summary['context_id'] = context_id
            origin = meta.get('frozen', {}).get('origin_m') if meta.get('frozen') else None
            orientation = meta.get('frozen', {}).get('start_orientation_xyzw') if meta.get('frozen') else None
            if context_id not in contexts:
                contexts[context_id] = []
            group = None
            for i, (p, q) in enumerate(contexts[context_id]):
                if origin is not None and p is not None and orientation is not None and q is not None:
                    distance = math.sqrt(sum((a-b)**2 for a, b in zip(p, origin)))
                    dot = abs(sum(a*b for a, b in zip(q, orientation)))
                    if distance <= .002 and 2*math.acos(min(1., dot)) <= .02:
                        group = i
                        break
            if group is None:
                group = len(contexts[context_id]); contexts[context_id].append((origin, orientation))
            summary['comparison_group'] = context_id+'_'+str(group)
            summaries.append(summary)
        except (ValueError, KeyError, OSError, TypeError) as exc:
            failures.append(str(folder)+': '+str(exc))
    fields = ['run_id', 'label', 'comparison_group', 'state', 'parameter_source']+list(TUNING)+[
        'samples', 'running_peak_mm', 'running_rms_mm', 'all_peak_mm', 'all_rms_mm',
        'radial_rms_mm', 'peak_time_s', 'final_error_mm', 'eligible_for_aggregate', 'quality', 'folder']
    with (output/'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for s in summaries:
            run, all_rows = s['tcp']['running'], s['tcp']['all']
            row = {k: s.get(k) for k in fields}
            row.update({k: s['parameters'].get(k) for k in TUNING})
            row.update(samples=run['total_mm']['n'], running_peak_mm=run['total_mm']['peak_abs'],
                       running_rms_mm=run['total_mm']['rms'], all_peak_mm=all_rows['total_mm']['peak_abs'],
                       all_rms_mm=all_rows['total_mm']['rms'], radial_rms_mm=run['radial_mm']['rms'],
                       peak_time_s=run['peak_time_s'], quality=';'.join(s['quality']))
            writer.writerow(row)
    groups = {}
    for s in summaries:
        if s['eligible_for_aggregate']:
            key = (s['comparison_group'],)+tuple(comparison_numbers(s['parameters'].get(k)) for k in TUNING)
            groups.setdefault(key, []).append(s)
    aggregates = []
    for key, members in groups.items():
        peaks = [s['tcp']['running']['total_mm']['peak_abs'] for s in members]
        rms = [s['tcp']['running']['total_mm']['rms'] for s in members]
        aggregates.append(dict(comparison_group=key[0], **dict(zip(TUNING, key[1:])),
            repeats=len(members), peak_mean_mm=statistics.mean(peaks), peak_worst_mm=max(peaks),
            peak_std_mm=statistics.stdev(peaks) if len(peaks)>1 else None,
            rms_mean_mm=statistics.mean(rms), rms_std_mm=statistics.stdev(rms) if len(rms)>1 else None))
    write_json(output/'comparison.json', {'runs': summaries, 'aggregates': aggregates, 'issues': failures})
    if aggregates:
        with (output/'groups.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(aggregates[0])); writer.writeheader(); writer.writerows(aggregates)
    elif (output/'groups.csv').exists():
        (output/'groups.csv').unlink()
    report = ['# Tracking experiment comparison', '',
              'RMS is sample-weighted. Running and settling are reported separately; all-sample RMS matches the panel scope.',
              'Only completed, fully captured runs with frozen parameters and all three streams enter repeat aggregates.',
              'Separate groups preserve task/model/backend/limits and start poses (2 mm / 0.02 rad tolerance).',
              'SDK generated-minus-measured joints include queue lead; they are not motor servo errors.',
              'TCP uses model FK, not external metrology. Nearby logs show correlation, not causation.', '',
              '| Run | P / I / D / FF(s) | State | Run peak / RMS (mm) | Quality |',
              '|---|---|---|---|---|']
    for s in summaries:
        metric = s['tcp']['running']['total_mm']
        values = ' / '.join(str(s['parameters'].get(k, '?')) for k in TUNING)
        numeric = '%.3f / %.3f'%(metric['peak_abs'], metric['rms']) if metric['n'] else 'unavailable'
        report.append('| %s | %s | %s | %s | %s |'%(s['run_id'], values, s['state'].replace('|', '/'),
                      numeric, ', '.join(s['quality']) or 'complete'))
    report += ['', '## Repeat aggregates', '', '```json', json.dumps(aggregates, indent=2), '```', '',
               '## Capture/import issues', '']+failures
    if plots and summaries:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        for group in sorted(set(s['comparison_group'] for s in summaries)):
            fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
            for s in (s for s in summaries if s['comparison_group'] == group):
                rows = [r for r in read_rows(s['folder'], 'tcp') if r[13] == 0]
                if not rows:
                    continue
                label = 'P=%g FF=%g %s%s'%(float(s['parameters'].get('kp', 0)),
                    float(s['parameters'].get('acceleration_ff_time', 0)),
                    s['run_id'][-8:], '' if s['eligible_for_aggregate'] else ' [partial/quality]')
                axes[0, 0].plot([r[0] for r in rows], [r[7]*1000 for r in rows], label=label)
                axes[0, 1].plot([r[0] for r in rows], [r[14]*1000 for r in rows], label=label)
                meta = json.loads((Path(s['folder'])/'metadata.json').read_text())
                radial = radial_errors(rows, meta)
                axes[1, 0].plot([r[0] for r in rows], [v*1000 for v in radial] if radial else
                                [r[15]*1000 for r in rows], label=label)
                axes[1, 1].plot([r[0] for r in rows], [r[12]*1000 for r in rows], label=label)
            for ax, title, unit in zip(axes.flat, ['Total error', 'Signed tangential error',
                    'Radial error (circle) / local normal error', 'Feedback age'], ['mm']*3+['ms']):
                ax.set(xlabel='Run time (s)', ylabel=unit, title=title); ax.grid(alpha=.3)
                if ax.lines:
                    ax.legend(fontsize=7)
            filename = 'comparison_'+group+'.png'
            fig.savefig(output/filename, dpi=150); plt.close(fig)
            report += ['', '!['+group+']('+filename+')']
    (output/'report.md').write_text('\n'.join(report)+'\n')
    return {'runs': len(summaries), 'aggregate_groups': len(aggregates), 'issues': failures}
