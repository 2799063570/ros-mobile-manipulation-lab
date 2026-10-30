#!/usr/bin/env python3
"""Analyze queued 5ms commands and MAC/feedback RPC timing; never sends commands."""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def read_capture(path, version):
    with path.open() as stream:
        metadata = stream.readline().strip()
        if not metadata.startswith('# ' + version + ';'):
            raise ValueError('Incomplete or unsupported capture: ' + str(path))
        dropped = int(metadata.rsplit('dropped=', 1)[1])
        return list(csv.DictReader(stream)), dropped


def latency_stats(values):
    values = sorted(values)
    if not values:
        return {'n': 0}
    def percentile(fraction):
        index = fraction * (len(values)-1)
        low = int(index)
        return values[low] + (values[min(low+1, len(values)-1)]-values[low])*(index-low)
    return dict(n=len(values), p50_ms=statistics.median(values), p95_ms=percentile(.95),
                p99_ms=percentile(.99), max_ms=values[-1])


def analyze(prefix, slow_ms=50.0):
    points, point_drops = read_capture(Path(str(prefix)+'.commands.csv'), 'hardware_commands_v1')
    rows, rpc_drops = read_capture(Path(str(prefix)+'.rpc.csv'), 'hardware_rpc_v1')
    peaks = dict(velocity_rad_s=0., acceleration_rad_s2=0., jerk_rad_s3=0., jerk_time_ros_s=None)
    previous_q = previous_v = previous_a = None
    last_id, last_sample = 0, None
    point_by_id = {}
    grid_outliers = 0
    for row in points:
        current_id = int(row['id'])
        if current_id != last_id + 1:
            raise ValueError('Queued command sequence is not contiguous')
        sample = int(row['sample_ns'])
        if last_sample is not None and abs(sample-last_sample-5000000) > 1000:
            grid_outliers += 1
        last_sample, last_id = sample, current_id
        point_by_id[current_id] = row
        q = [float(row['q'+str(j)]) for j in range(6)]
        if not all(math.isfinite(value) for value in q):
            raise ValueError('Nonfinite queued command')
        if previous_q is None:
            previous_q, previous_v, previous_a = q, [0.]*6, [0.]*6
            continue
        v = [(a-b)/.005 for a,b in zip(q,previous_q)]
        a = [(a-b)/.005 for a,b in zip(v,previous_v)]
        j = [(a-b)/.005 for a,b in zip(a,previous_a)]
        peaks['velocity_rad_s'] = max(peaks['velocity_rad_s'], *map(abs,v))
        peaks['acceleration_rad_s2'] = max(peaks['acceleration_rad_s2'], *map(abs,a))
        if max(map(abs,j)) > peaks['jerk_rad_s3']:
            peaks['jerk_rad_s3'] = max(map(abs,j))
            peaks['jerk_time_ros_s'] = int(row['ros_ns'])*1e-9
        previous_q, previous_v, previous_a = q,v,a

    calls = {}
    for row in rows:
        key = row['kind'], int(row['id'])
        call = calls.setdefault(key, {'metadata': row, 'before': {}, 'after': {}})
        reference = call['metadata']
        for field in ['begin_ns','end_ns','count','mac_scalars','result','port_hint','first_point_id']:
            if row[field] != reference[field]:
                raise ValueError('Inconsistent RPC records: ' + str(key))
        port = int(row['local_port'])
        phase = row['phase']
        if phase not in ('before','after') or port in call[phase]:
            raise ValueError('Duplicate/invalid socket snapshot')
        call[phase][port] = row
    latencies = {'query': [], 'send': [], 'feedback_query': []}
    queue_waits = []
    slow = []
    for (kind, ident), call in sorted(calls.items(), key=lambda item: int(item[1]['metadata']['begin_ns'])):
        row = call['metadata']
        elapsed = (int(row['end_ns'])-int(row['begin_ns']))*1e-6
        if kind not in latencies or elapsed < 0:
            raise ValueError('Invalid RPC kind/time')
        latencies[kind].append(elapsed)
        queue_wait = {}
        if kind == 'send':
            first_id, count = int(row['first_point_id']), int(row['count'])
            if first_id <= 0 or count <= 0:
                raise ValueError('Invalid send -> queued-point association')
            for label, point_id in [('first', first_id), ('last', first_id+count-1)]:
                point = point_by_id.get(point_id)
                if point is None and point_drops == 0:
                    raise ValueError('Send references a missing queued command')
                if point is not None:
                    wait = (int(row['begin_ns'])-int(point['steady_ns']))*1e-6
                    if wait < 0:
                        raise ValueError('Send began before a referenced command was prepared')
                    queue_wait[label+'_point_queue_wait_ms'] = wait
            if 'first_point_queue_wait_ms' in queue_wait:
                queue_waits.append(queue_wait['first_point_queue_wait_ms'])
        if elapsed < slow_ms:
            continue
        event = dict(kind=kind, id=ident, duration_ms=elapsed, ros_time_s=int(row['ros_ns'])*1e-9,
                     result=int(row['result']), port_hint=int(row['port_hint']),
                     first_point_id=int(row['first_point_id']), points=int(row['count']),
                     queue_wait=queue_wait, socket_deltas=[])
        for port, before in call['before'].items():
            after = call['after'].get(port)
            if not after or before['fd'] != after['fd'] or not all(int(s[field]) for s in [before,after] for field in ['tcp_valid','bytes_valid']):
                continue
            delta = dict(local_port=port, fd=int(after['fd']),
                         is_rpc_login_hint=port==int(row['port_hint']),
                         is_mac_login_hint=kind in ('query', 'send') and port==int(row['port_hint']))
            for field in ['bytes_sent','bytes_acked','bytes_received','retransmissions']:
                value = int(after[field])-int(before[field])
                if value < 0:
                    delta['counter_reset'] = True
                delta[field+'_delta'] = value
            for field in ['last_data_sent_ms','last_data_received_ms','unacked','notsent_bytes','rtt_us','rto_us']:
                delta[field+'_after'] = int(after[field])
            event['socket_deltas'].append(delta)
        hint = next((s for s in event['socket_deltas'] if s['is_rpc_login_hint']), None)
        if hint is None or hint.get('counter_reset'):
            event['observation'] = 'No comparable RPC login socket; inspect per-port rows.'
        elif hint['retransmissions_delta'] > 0:
            event['observation'] = 'Outbound retransmission observed during RPC; not proof it caused all latency.'
        elif hint['bytes_received_delta'] > 0 and hint['last_data_received_ms_after'] >= .75*elapsed:
            event['observation'] = 'Payload arrived well before SDK return; investigate SDK/host wakeup or processing.'
        elif hint['bytes_received_delta'] > 0 and hint['last_data_received_ms_after'] <= .25*elapsed:
            event['observation'] = 'Payload arrived near SDK return; cabinet wait or inbound transport remain candidates.'
        else:
            event['observation'] = 'Socket timing does not isolate the waiting stage.'
        slow.append(event)
    return dict(prefix=str(prefix), queued_commands=len(points), command_records_dropped=point_drops,
                rpc_records_dropped=rpc_drops, command_grid_outliers=grid_outliers, queued_sample_peaks=peaks,
                latency={kind:latency_stats(values) for kind,values in latencies.items()}, slow_calls=slow,
                batch_first_point_queue_wait=latency_stats(queue_waits),
                limitations=['Queued commands are not cabinet execution or motor vibration measurements.',
                             'Port hint is inferred at the corresponding SDK login; reconnection may invalidate it.',
                             'TCP_INFO contains socket counters/last-payload age, not RPC payload identification.',
                             'No outbound retransmission does not exclude loss or retries in the other direction.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path)
    parser.add_argument('--slow-ms', type=float, default=50.)
    parser.add_argument('--json', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.slow_ms) or args.slow_ms <= 0:
        parser.error('--slow-ms must be positive and finite')
    report = analyze(args.prefix, args.slow_ms)
    output = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    print(output)
    if args.json:
        args.json.write_text(output)


if __name__ == '__main__':
    main()
