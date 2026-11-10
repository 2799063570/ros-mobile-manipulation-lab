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
