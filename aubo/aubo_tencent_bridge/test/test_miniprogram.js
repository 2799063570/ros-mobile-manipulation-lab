const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const view = require('../wechat/miniprogram/utils/view');
const source = fs.readFileSync(path.join(__dirname, '../wechat/miniprogram/pages/console/index.js'), 'utf8');

async function main() {
    let page, config = {demoMode: true, functionName: 'auboApi'}, calls = [], timers = new Set();
    let snapshot = view.demoSnapshot(Date.now());
    snapshot.context.enabled = true;
    snapshot.commandsEnabled = true;
    snapshot.role = 'operator';
    let resolveMotion, holdMotion = false, failRead = false;
    const sandbox = {require: () => view, Date, Page: definition => page = definition,
        getApp: () => ({globalData: {config}}),
        setInterval: fn => {timers.add(fn); return fn;}, clearInterval: fn => timers.delete(fn),
        wx: {showModal: ({success}) => success({confirm: true}), cloud: {callFunction: async ({data}) => {
            calls.push(data);
            if (data.action === 'identity') return {result: {ok: true, openid: 'openid'}};
            if (data.action === 'list') return {result: {ok: true, devices: [{id: 'sorter-01', role: 'operator'}]}};
            if (data.action === 'read') {
                if (failRead) throw new Error('offline');
                return {result: {ok: true, ...snapshot}};
            }
            if (data.action === 'prepare') return {result: {ok: true, document: {method: data.method, params: {command_id: 'test'}}}};
            if (holdMotion && data.document.method === 'start') return await new Promise(resolve => {resolveMotion = resolve;});
            return {result: {ok: true, result: {status: 'accepted'}}};
        }}}
    };
    vm.runInNewContext(source, sandbox);
    page.data = {...page.data};
    page.setData = update => Object.assign(page.data, update);
    page.onLoad();
    await page.onShow();
    assert.equal(page.data.vm.canMove, false);
    assert.equal(page.data.vm.canStop, false);
    assert.equal(page.data.vm.rows.length, 3);
    await page.send({currentTarget: {dataset: {method: 'start'}}});
    assert.equal(calls.length, 0); // Demo never contacts a cloud API, even for a direct handler call.
    page.onHide();
    assert.equal(timers.size, 0);
    config = {demoMode: false, functionName: 'auboApi'};
    await page.onShow();
    assert.equal(page.data.vm.canStart, true);
    holdMotion = true;
    const motion = page.send({currentTarget: {dataset: {method: 'start'}}});
    for (let i = 0; i < 10 && !resolveMotion; i++) await Promise.resolve();
    assert.ok(resolveMotion);
    assert.equal(page.data.vm.canStart, false);
    assert.equal(page.data.vm.canStop, true);
    await page.send({currentTarget: {dataset: {method: 'stop'}}});
    assert.equal(calls.filter(x => x.action === 'command' && x.document.method === 'stop').length, 1);
    resolveMotion({result: {ok: true, result: {status: 'accepted'}}});
    await motion;
    const preparedCalls = calls.filter(x => x.action === 'prepare').length;
    const oldApi = page._api;
    page._api = async data => {
        const response = await oldApi.call(page, data);
        if (data.action === 'prepare') page.onHide();
        return response;
    };
    const before = calls.filter(x => x.action === 'command').length;
    await page.send({currentTarget: {dataset: {method: 'home'}}});
    assert.equal(calls.filter(x => x.action === 'prepare').length, preparedCalls + 1);
    assert.equal(calls.filter(x => x.action === 'command').length, before);
    page._api = oldApi;
    await page.onShow();
    failRead = true;
    await page.refresh();
    assert.equal(page.data.vm.canStop, false);
    page.onUnload();
    assert.equal(timers.size, 0);
    assert.equal(view.derive(snapshot, 15000, false, false, false).canMove, false);
    assert.equal(view.derive({...snapshot, role: 'viewer'}, 0, false, false, false).canMove, false);
    console.log('Mini program: demo isolation, telemetry, permissions, independent stop, navigation cancellation, expiry and timer cleanup passed.');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
