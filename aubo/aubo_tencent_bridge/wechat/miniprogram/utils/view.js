const states = {INITIALIZING: '初始化', IDLE: '等待操作', READY: '已就绪', OBSERVING: '前往观察位',
    DETECTING: '识别与队列', SORTING: '分拣中', PICKING: '抓放执行', OPENING: '夹爪全开',
    PREPARING: '前往工作位', HOMING: '返回结束位', STOPPED: '已停止', ERROR: '任务失败'};
const labels = {candidate: '待确认', ready: '可领取', reserved: '执行中', done: '流程完成', review: '待复核'};
const errors = {login_required: '登录信息不可用，请检查小程序 AppID 与云函数配置。',
    server_not_configured: '云函数尚未配置，请按 README 填写环境变量。',
    access_denied: '当前账号尚未获得设备权限。', operator_required: '当前账号只有查看权限。',
    server_read_only: '云端处于只读模式。', bridge_read_only: '机器人云桥处于只读模式。',
    device_offline: '设备离线或心跳过期。', sorting_unavailable: '本地分拣接口尚未就绪。',
    sorting_not_ready: '请先观察目标，等待分拣状态为 READY。', sorting_busy: '分拣正在执行任务。',
    session_changed: '设备连接已变化，请刷新状态后重新确认操作。', command_expired: '指令已过期。',
    cloud_service_unavailable: '云服务暂不可用，请检查连接和配置。',
    device_not_configured: '设备未在云函数中配置。'};
function parse(value, fallback) { try { return JSON.parse(value); } catch (_) { return fallback; } }
function derive(snapshot, elapsed, pendingMotion, pendingStop, demo) {
    const values = snapshot ? snapshot.values : {};
    const context = snapshot ? snapshot.context : {};
    const fresh = !!snapshot && snapshot.fresh === true && elapsed >= 0 &&
        elapsed < 10000 && Math.abs(snapshot.serverNowMs + elapsed - context.wall_ms) < 10000;
    const enabled = !demo && fresh && context.available === true && context.enabled === true &&
        snapshot.commandsEnabled === true && snapshot.role === 'operator';
    const canMove = enabled && !pendingMotion && ['IDLE', 'READY', 'STOPPED', 'ERROR'].includes(context.state);
    const targets = parse(values.target_cache || '{}', {}).targets;
    const last = parse(values.last_command || '{}', {});
    return {fresh, link: demo ? '演示数据' : fresh ? '云链路在线' : '离线 / 等待心跳',
        state: states[values.state] || values.state || '等待数据', detail: values.state_detail || '',
        phase: states[values.phase] || values.phase || '等待数据', description: values.description || '',
        mode: demo ? '演示模式，未连接机器人' : enabled ? '远程任务已启用' : '只读 / 控制条件未满足',
        total: values.queue_total || 0, ready: values.queue_ready || 0,
        reserved: values.queue_reserved || 0, done: values.queue_done || 0,
        summary: values.detection_summary || '', failure: values.failure || '暂无故障报告',
        truncated: values.target_cache_truncated === 1,
        updated: values.bridge_wall_ms ? new Date(Number(values.bridge_wall_ms)).toLocaleString() : '等待机器人上报',
        rows: (Array.isArray(targets) ? targets : []).map(row => ({...row,
            label: labels[row.status] || row.status,
            confidence: typeof row.detection_confidence === 'number' ? (row.detection_confidence * 100).toFixed(1) + '%' : '—'})),
        lastCommand: last.status ? (last.method || '') + ' / ' + last.status + ' / ' + (last.message || last.reason || '') : '暂无',
        canMove, canStart: canMove && context.state === 'READY', canStop: enabled && !pendingStop};
}
function demoSnapshot(now) {
    return {serverNowMs: now, fresh: true, commandsEnabled: false, role: 'viewer',
        context: {wall_ms: now, session: '0'.repeat(32), enabled: false, available: true, state: 'READY'},
        values: {bridge_wall_ms: String(now), state: 'READY', state_detail: '等待用户开始任务（演示）',
            phase: 'DETECTING', queue_total: 3, queue_ready: 2, queue_reserved: 0, queue_done: 1,
            target_cache: JSON.stringify({targets: [
                {id: 1, category: '饮料罐', status: 'ready', detection_confidence: .95},
                {id: 2, category: '饮料罐', status: 'ready', detection_confidence: .91},
                {id: 3, category: '饮料罐', status: 'done', detection_confidence: .93}
            ]})}};
}
module.exports = {derive, demoSnapshot, errorText: code => errors[code] || '请求未完成，请检查配置或稍后刷新。'};
