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
scope.send('stop');
assert.strictEqual(sent.length, 2);
self.onDestroy();
console.log('Widget controller: offline, live data, command payload, pending stop, feedback and expiry passed.');
