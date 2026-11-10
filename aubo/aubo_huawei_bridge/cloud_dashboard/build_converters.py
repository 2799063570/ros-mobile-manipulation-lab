"""Build DLV converter bodies for mapped single-device OBS CSV rows.

These are converter scripts, not a DLV dashboard-import archive.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
COMMON = r'''
var AUBO_DEVICE_ID = "YOUR_DEVICE_ID"; // Replace with the full registered device ID.
function objectValue(value) {
    if (typeof value === 'string') { try { value = JSON.parse(value); } catch (_) { return null; } }
    return value && typeof value === 'object' && !Array.isArray(value) ? value : null;
}
function latestSnapshot(input) {
    if (AUBO_DEVICE_ID === 'YOUR_DEVICE_ID') throw new Error('请先填写 AUBO_DEVICE_ID');
    var rows = Array.isArray(input) ? input : [input], latest = null, stamp = -1;
    rows.forEach(function (row) {
        if (!row || String(row.device_id) !== AUBO_DEVICE_ID || row.service_id !== 'Sorting') return;
        var at = Number(row.bridge_wall_ms);
        if (isFinite(at) && at > 0 && at >= stamp) { latest = row; stamp = at; }
    });
    return latest;
}
function fresh(row) {
    var age = Date.now() - Number(row.bridge_wall_ms);
    return isFinite(age) && age >= 0 && age <= 90000;
}
var row = latestSnapshot(data);
if (!row) return [];
'''.strip() + '\n'

BODIES = {
    'projection.js': r'''
if (!fresh(row)) return []; // Never show an old position as a current target.
var cache = objectValue(row.target_cache), targets = cache && cache.targets;
if (!Array.isArray(targets)) return [];
return targets.filter(function (target) {
    return target && Array.isArray(target.position) && target.position.length === 3 &&
        target.position.every(function (n) { return typeof n === 'number' && isFinite(n); });
}).map(function (target) {
    return {x: target.position[0], y: target.position[1], s: target.category, value: 1,
            name: '#' + target.id, id: target.id, category: target.category, status: target.status};
});
''',
    'completed_by_class.js': r'''
if (Number(row.statistics_available) !== 1) return [];
var statistics = objectValue(row.completed_counts);
if (!statistics || !objectValue(statistics.counts)) return [];
return Object.keys(statistics.counts).map(function (category) {
    return {name: category, value: Number(statistics.counts[category]), session: statistics.session,
            truncated: Number(row.completed_counts_truncated) === 1};
}); // Use only this snapshot. Do not sum historical cumulative values.
''',
    'completed_total.js': r'''
if (Number(row.statistics_available) !== 1) return [];
var value = Number(row.completed_total);
if (!isFinite(value) || value < 0) return [];
return [{name: '本次运行累计完成', value: value}];
''',
    'queue_counts.js': r'''
return ['total','candidate','ready','reserved','done','review'].map(function (kind) {
    return {name: kind, value: Number(row['queue_' + kind]) || 0};
});
''',
    'status.js': r'''
var isFresh = fresh(row), cell = objectValue(row.workcell) || {};
return [{name: '设备状态', value: isFresh ? row.state : '快照过期：' + row.state,
    detail: row.state_detail || '', age_s: Math.round((Date.now() - Number(row.bridge_wall_ms))/1000),
    fresh: isFresh, frame: cell.frame || '', workcell: cell,
    target_cache_truncated: Number(row.target_cache_truncated) === 1,
    completed_counts_truncated: Number(row.completed_counts_truncated) === 1}];
'''
}


def write_files():
    directory = ROOT / 'converters'
    directory.mkdir(exist_ok=True)
    for name, body in BODIES.items():
        (directory / name).write_text(COMMON + body.lstrip(), encoding='utf-8')
    fields = ['bridge_wall_ms', 'bridge_session', 'state', 'state_detail', 'phase', 'description',
        'failure', 'detection_summary', 'target_cache', 'target_cache_truncated', 'target_cache_age_s',
        'queue_total', 'queue_candidate', 'queue_ready', 'queue_reserved', 'queue_done', 'queue_review',
        'completed_counts', 'completed_total', 'statistics_available', 'completed_counts_truncated', 'workcell']
    mapping = [{'source': 'notify_data.header.device_id', 'target': 'device_id'},
               {'source': 'notify_data.body.services[0].service_id', 'target': 'service_id'}]
    mapping += [{'source': 'notify_data.body.services[0].properties.' + name, 'target': name} for name in fields]
    (ROOT / 'obs_field_mapping.json').write_text(json.dumps(mapping, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    write_files()
