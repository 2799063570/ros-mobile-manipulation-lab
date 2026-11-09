'use strict';
const crypto = require('crypto');
const METHODS = new Set(['move_to_observation', 'start', 'stop', 'home']);
const ID = /^[A-Za-z0-9_-]{16,80}$/;
const MOVE_STATES = new Set(['IDLE', 'READY', 'STOPPED', 'ERROR']);
class ApiError extends Error {}
function check(condition, code) { if (!condition) throw new ApiError(code); }

function telemetry(response, now) {
    const source = JSON.parse(response.Data || '{}');
    const values = Object.fromEntries(Object.entries(source).map(([key, entry]) => [key, entry.Value]));
    let context = {};
    try { context = JSON.parse(values.bridge_context || '{}'); } catch (_) {}
    const updated = Number((source.bridge_context || {}).LastUpdate);
    const fresh = Number.isFinite(updated) && updated > 0 && now - updated < 10000 && updated - now < 5000 &&
        Number.isSafeInteger(context.wall_ms) && Math.abs(now - context.wall_ms) < 10000 &&
        typeof context.session === 'string' && /^[a-f0-9]{32}$/.test(context.session);
    return {values, context, fresh, serverNowMs: now};
}

function eligible(snapshot, method) {
    check(snapshot.fresh, 'device_offline');
    check(snapshot.context.enabled === true, 'bridge_read_only');
    check(snapshot.context.available === true, 'sorting_unavailable');
    if (method === 'start') check(snapshot.context.state === 'READY', 'sorting_not_ready');
    else if (method !== 'stop') check(MOVE_STATES.has(snapshot.context.state), 'sorting_busy');
}

function validate(document, now) {
    check(document && typeof document === 'object' && !Array.isArray(document) && METHODS.has(document.method), 'invalid_command');
    const p = document.params;
    check(p && typeof p === 'object' && !Array.isArray(p) &&
        Object.keys(p).sort().join(',') === 'command_id,expires_at_ms,issued_at_ms,session', 'invalid_command');
    check(typeof p.command_id === 'string' && ID.test(p.command_id) &&
        typeof p.session === 'string' && /^[a-f0-9]{32}$/.test(p.session), 'invalid_command');
    check(Number.isSafeInteger(p.issued_at_ms) && Number.isSafeInteger(p.expires_at_ms) &&
        p.expires_at_ms > now && p.issued_at_ms <= now + 5000 &&
        p.expires_at_ms > p.issued_at_ms && p.expires_at_ms - p.issued_at_ms <= 10000, 'command_expired');
    check(Object.keys(document).sort().join(',') === 'method,params', 'invalid_command');
    check(Buffer.byteLength(JSON.stringify(document)) <= 1024, 'invalid_command');
}

// SDK and trusted caller context are injected so tests cannot accidentally call a real device.
function createHandler({identity, acl, client, devices, commandsEnabled, appId, now = Date.now}) {
    async function handle(event) {
        const caller = identity();
        check(caller && caller.APPID === appId && appId && caller.OPENID, 'login_required');
        if (event.action === 'identity') return {openid: caller.OPENID};
        const permissions = (await acl(caller.OPENID)).devices || {};
        if (event.action === 'list') return {devices: Object.keys(devices)
            .filter(key => ['viewer', 'operator'].includes(permissions[key]))
            .map(key => ({id: key, role: permissions[key]}))};
        check(Object.hasOwn(devices, event.device), 'device_not_configured');
        const role = permissions[event.device];
        check(role === 'viewer' || role === 'operator', 'access_denied');
        const device = devices[event.device];
        check(['read', 'prepare', 'command'].includes(event.action), 'invalid_action');
        if (event.action !== 'read') {
            check(role === 'operator', 'operator_required');
            check(commandsEnabled === true, 'server_read_only');
            if (event.action === 'command') validate(event.document, now());
            else check(METHODS.has(event.method), 'invalid_command');
        }
        const snapshot = telemetry(await client.DescribeDeviceData(device), now());
        if (event.action === 'read') return {...snapshot, role, commandsEnabled: commandsEnabled === true};
        const method = event.action === 'prepare' ? event.method : event.document.method;
        eligible(snapshot, method);
        if (event.action === 'prepare') {
            const issued = now();
            return {document: {method, params: {command_id: crypto.randomBytes(16).toString('hex'),
                session: snapshot.context.session, issued_at_ms: issued, expires_at_ms: issued + 10000}}};
        }
        validate(event.document, now());
        check(snapshot.context.session === event.document.params.session, 'session_changed');
        // Preserve the complete envelope on every retry; never renew a command or its expiry.
        try {
            const response = await client.CallDeviceActionSync({...device, ActionId: 'task_command',
                InputParams: JSON.stringify({request: JSON.stringify(event.document)})});
            const result = JSON.parse(JSON.parse(response.OutputParams).result);
            check(['accepted', 'rejected', 'unknown'].includes(result.status), 'invalid_device_reply');
            check(!result.command_id || result.command_id === event.document.params.command_id, 'invalid_device_reply');
            return {result, commandId: event.document.params.command_id};
        } catch (_) {
            // A cloud/API timeout can happen after ROS received the command. Do not resend it here.
            return {commandId: event.document.params.command_id,
                result: {status: 'unknown', reason: 'check_local_state_before_any_further_motion'}};
        }
    }
    return async event => {
        try { return {ok: true, ...await handle(event || {})}; }
        catch (error) { return {ok: false, error: error instanceof ApiError ? error.message : 'cloud_service_unavailable'}; }
    };
}
module.exports = {createHandler, telemetry, validate};
