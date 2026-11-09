const assert = require('node:assert/strict');
const {createHandler} = require('../wechat/cloudfunctions/auboApi/core');

async function main() {
    let now = 1794191363000, role = 'operator', enabled = true, session = 'a'.repeat(32);
    let stale = false, caller = {OPENID: 'trusted-user', APPID: 'wx-test'}, calls = [], timeout = false;
    const client = {
        async DescribeDeviceData(device) {
            assert.deepEqual(device, {ProductId: 'PRODUCT', DeviceName: 'robot'});
            return {Data: JSON.stringify({bridge_context: {Value: JSON.stringify({session,
                wall_ms: stale ? now - 15000 : now, enabled: true, available: true, state: 'READY'}), LastUpdate: now}})};
        },
        async CallDeviceActionSync(request) {
            calls.push(request);
            if (timeout) throw new Error('SDK error that must never leak');
            const document = JSON.parse(JSON.parse(request.InputParams).request);
            return {OutputParams: JSON.stringify({result: JSON.stringify({status: 'accepted', command_id: document.params.command_id})})};
        }
    };
    const handler = () => createHandler({identity: () => caller, acl: async openid => {
        assert.equal(openid, 'trusted-user'); return {devices: {'sorter-01': role}};
    }, client, devices: {'sorter-01': {ProductId: 'PRODUCT', DeviceName: 'robot'}},
        appId: 'wx-test', commandsEnabled: enabled, now: () => now});
    const invoke = (action, extra = {}) => handler()({action, device: 'sorter-01', ...extra});
    assert.equal((await invoke('identity', {OPENID: 'spoofed'})).openid, 'trusted-user');
    caller = {APPID: 'wrong-app', OPENID: 'trusted-user'};
    assert.equal((await invoke('read')).error, 'login_required');
    caller = {OPENID: 'trusted-user', APPID: 'wx-test'};
    role = undefined;
    assert.deepEqual((await invoke('list')).devices, []);
    assert.equal((await invoke('read')).error, 'access_denied');
    role = 'viewer';
    assert.equal((await invoke('read')).ok, true);
    assert.equal((await invoke('prepare', {method: 'start'})).error, 'operator_required');
    role = 'operator'; enabled = false;
    assert.equal((await invoke('prepare', {method: 'start'})).error, 'server_read_only');
    enabled = true; stale = true;
    assert.equal((await invoke('prepare', {method: 'start'})).error, 'device_offline');
    stale = false;
    assert.equal((await invoke('read', {device: '__proto__'})).error, 'device_not_configured');
    assert.equal((await invoke('prepare', {method: 'open_gripper'})).error, 'invalid_command');
    assert.equal(calls.length, 0);
    const prepared = await invoke('prepare', {method: 'start'});
    const document = prepared.document;
    assert.equal(document.params.expires_at_ms - document.params.issued_at_ms, 10000);
    assert.equal(document.params.session, session);
    assert.match(document.params.command_id, /^[a-f0-9]{32}$/);
    assert.equal((await invoke('command', {document})).result.status, 'accepted');
    assert.deepEqual(JSON.parse(JSON.parse(calls[0].InputParams).request), document);
    assert.equal(calls[0].ActionId, 'task_command');
    session = 'b'.repeat(32);
    assert.equal((await invoke('command', {document})).error, 'session_changed');
    assert.equal(calls.length, 1);
    session = 'a'.repeat(32);
    timeout = true;
    const unknown = await invoke('command', {document});
    assert.equal(unknown.result.status, 'unknown');
    assert.equal(calls.length, 2); // One cloud request only: no automatic application retry.
    assert.equal(JSON.stringify(unknown).includes('SDK error'), false);
    now += 10001;
    assert.equal((await invoke('command', {document})).error, 'command_expired');
    assert.equal(calls.length, 2);
    console.log('Cloud API: trusted identity, ACL, read-only gates, freshness, session, expiry, exact envelopes and timeout outcome passed.');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
