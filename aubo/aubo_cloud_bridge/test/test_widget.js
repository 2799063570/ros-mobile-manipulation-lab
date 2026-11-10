/* Run with node; exercises the actual widget controller without a cloud account. */
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const path = require('path');
const source = fs.readFileSync(path.join(__dirname, '../dashboards/console.js'), 'utf8');
let sent = [], success, failure;
const scope = {$injector: {get: () => ({sendTwoWayRpcCommand: (id, body) => {
    sent.push({id, body});
    return {subscribe: (ok, bad) => {success = ok; failure = bad; return {unsubscribe(){}};}};
}})}};
const self = {ctx: {$scope: scope, servicesMap: new Map([['deviceService', 'service']]), detectChanges(){}, data: []}};
const sandbox = {self, window: {confirm: () => true, crypto: require('crypto').webcrypto},
    Date, Uint8Array, setInterval: () => 1, clearInterval(){}};
vm.runInNewContext(source, sandbox);
self.onInit();
assert.strictEqual(scope.vm.canStart, false);
function feed(values, timestamp = Date.now()) {
    self.ctx.data = Object.entries(values).map(([name, value]) => ({dataKey: {name},
        datasource: {entityId: 'device-id'}, data: [[timestamp, value]]}));
    self.onDataUpdated();
}
feed({bridge_wall_ms: Date.now(), bridge_session: 'test-session', commands_enabled: true,
    sorting_available: true, state:'READY', target_cache:'{"targets":[{"id":1,"category":"罐","status":"ready","detection_confidence":0.9}]}'});
assert.strictEqual(scope.vm.canStart, true);
assert.strictEqual(scope.vm.rows[0].confidence, '90.0%');
feed({workcell: JSON.stringify({table_center:[.7,0],table_size:[.7,1],frame:'base_link'}),
    target_cache_age_s:0, target_cache:JSON.stringify({targets:[
        {id:6,category:'bottle',status:'ready',position:[.7,0,.12]},
        {id:7,category:'can',status:'ready',position:[4,0,.12]},
        {id:8,category:'box',status:'candidate',position:null}]}),
    statistics_available:true, completed_counts:JSON.stringify({scope:'node_session', session:'first',
        counts:{bottle:1,can:2,box:3},total:6})});
assert.strictEqual(scope.vm.map.points.length, 1);
assert.strictEqual(scope.vm.map.outside, 1);
assert.ok(Math.abs(scope.vm.map.points[0].x - 360) < 1e-9);
assert.ok(Math.abs(scope.vm.map.points[0].y - 150) < 1e-9);
assert.strictEqual(scope.vm.rows[0].y, '0.000');
assert.strictEqual(scope.vm.completedTotal, 6);
feed({target_cache:'{"targets":[]}'});
assert.strictEqual(scope.vm.completedTotal, 6);
feed({completed_counts:JSON.stringify({scope:'node_session',session:'restart',counts:{bottle:0,can:0,box:0},total:0})});
assert.strictEqual(scope.vm.completedTotal, 0);
feed({statistics_available:false});
assert.strictEqual(scope.vm.completedTotal, '—');
scope.send('start');
assert.strictEqual(sent[0].id, 'device-id');
assert.strictEqual(sent[0].body.persistent, false);
assert.strictEqual(sent[0].body.params.session, 'test-session');
assert.strictEqual(sent[0].body.params.expires_at_ms - sent[0].body.params.issued_at_ms, 10000);
assert.strictEqual(scope.vm.canStart, false);
assert.strictEqual(scope.vm.canStop, true);
success({status:'accepted'});
assert.match(scope.vm.rpcMessage, /请求已接收/);
scope.send('home'); failure();
assert.match(scope.vm.rpcMessage, /结果未知/);
feed({bridge_wall_ms:Date.now() - 15000}, Date.now() + 1);
assert.strictEqual(scope.vm.canStop, false);
assert.strictEqual(scope.vm.map.points.length, 0);
scope.send('stop');
assert.strictEqual(sent.length, 2);
self.onDestroy();
console.log('Widget controller: offline, live data, command payload, pending stop, feedback and expiry passed.');
