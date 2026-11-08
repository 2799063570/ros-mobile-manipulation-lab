/* ThingsBoard latest-values custom widget; device credentials never reach this UI. */
var values = {}, received = {}, timer, pending = false, deviceId = '', subscriptions = [];
var states = {INITIALIZING:'初始化', IDLE:'等待操作', READY:'已就绪', OBSERVING:'前往观察位',
    DETECTING:'识别与队列', SORTING:'分拣中', PICKING:'抓放执行', OPENING:'夹爪全开',
    PREPARING:'前往工作位', HOMING:'返回结束位', STOPPED:'已停止', ERROR:'任务失败'};
var labels = {candidate:'待确认', ready:'可领取', reserved:'执行中', done:'流程完成', review:'待复核'};
function parse(value, fallback) { try { return JSON.parse(value); } catch (e) { return fallback; } }
function yes(value) { return value === true || value === 'true'; }
function refresh() {
    var vm = self.ctx.$scope.vm, now = Date.now();
    vm.fresh = received.bridge_wall_ms > 0 && now - received.bridge_wall_ms < 10000 &&
        Math.abs(now - Number(values.bridge_wall_ms)) < 10000;
    vm.link = vm.fresh ? '云链路在线' : '离线 / 数据过期';
    vm.updated = received.bridge_wall_ms ? '最近上报 ' + new Date(received.bridge_wall_ms).toLocaleString() : '等待机器人接入';
    vm.controlMode = yes(values.commands_enabled) ? '远程任务已启用' : '只读监控';
    vm.state = states[values.state] || values.state || '等待数据';
    vm.detail = values.state_detail || '';
    vm.phase = states[values.phase] || values.phase;
    vm.description = values.description || '';
    vm.total = values.queue_total || 0; vm.ready = values.queue_ready || 0;
    vm.reserved = values.queue_reserved || 0; vm.done = values.queue_done || 0;
    vm.summary = values.detection_summary || '';
    vm.failure = values.failure || '';
    vm.queueAge = Number(values.target_cache_age_s) >= 0 ? '桥接收到此快照后 ' + values.target_cache_age_s + ' 秒' : '未收到快照';
    var latest = parse(values.last_command, {});
    vm.lastCommand = latest.status ? (latest.method || '') + ' / ' + latest.status + ' / ' + (latest.message || latest.reason || '') : '暂无';
    var cache = parse(values.target_cache, {targets:[]});
    vm.rows = (Array.isArray(cache.targets) ? cache.targets : []).map(function(row) {
        return {id:row.id, category:row.category, label:labels[row.status] || row.status,
            confidence:typeof row.detection_confidence === 'number' ? (100 * row.detection_confidence).toFixed(1) + '%' : '—'};
    });
    var enabled = vm.fresh && yes(values.commands_enabled) && yes(values.sorting_available) && !!deviceId && !!values.bridge_session;
    vm.canStop = enabled; // Stop stays available while a motion RPC is pending.
    vm.canMove = enabled && !pending && ['IDLE','READY','STOPPED','ERROR'].indexOf(values.state) >= 0;
    vm.canStart = vm.canMove && values.state === 'READY';
    self.ctx.detectChanges();
}
self.onInit = function() {
    self.ctx.$scope.vm = {};
    self.ctx.$scope.send = function(method) {
        refresh();
        var vm = self.ctx.$scope.vm;
        if (method === 'stop' ? !vm.canStop : method === 'start' ? !vm.canStart : !vm.canMove) { return; }
        if (method !== 'stop' && !window.confirm('确认发送“' + ({start:'开始分拣', home:'返回结束位', move_to_observation:'前往观察位'}[method]) + '”？请确认现场允许运动。')) { return; }
        var commandId = window.crypto.randomUUID ? window.crypto.randomUUID() : Array.from(window.crypto.getRandomValues(new Uint8Array(16))).map(function(x){return x.toString(16).padStart(2,'0');}).join('');
        var now = Date.now(), body = {method:method, params:{command_id:commandId,
            session:values.bridge_session, issued_at_ms:now, expires_at_ms:now + 10000}, timeout:8000, persistent:false};
        if (method !== 'stop') { pending = true; }
        vm.rpcMessage = '正在发送 ' + method + '…'; refresh();
        var service = self.ctx.$scope.$injector.get(self.ctx.servicesMap.get('deviceService'));
        subscriptions = subscriptions.filter(function(s){return !s.closed;});
        subscriptions.push(service.sendTwoWayRpcCommand(deviceId, body).subscribe(function(result) {
            if (method !== 'stop') { pending = false; }
            vm.rpcMessage = result.status === 'accepted' ? '请求已接收，请查看任务状态确认执行结果。' :
                '指令反馈：' + result.status + ' / ' + (result.message || result.reason || '请检查机器人');
            refresh();
        }, function() {
            if (method !== 'stop') { pending = false; }
            vm.rpcMessage = '请求结果未知，请核对机器人状态；不要直接重复发起运动。'; refresh();
        }));
    };
    timer = setInterval(refresh, 1000);
    self.onDataUpdated();
};
self.onDataUpdated = function() {
    (self.ctx.data || []).forEach(function(item) {
        var data = item.data || [], point = data[data.length - 1];
        if (point && (!received[item.dataKey.name] || point[0] >= received[item.dataKey.name])) {
            values[item.dataKey.name] = point[1]; received[item.dataKey.name] = point[0];
        }
        if (item.datasource && item.datasource.entityId) {
            deviceId = typeof item.datasource.entityId === 'string' ? item.datasource.entityId : item.datasource.entityId.id;
        }
    });
    if (self.ctx.$scope.vm) { refresh(); }
};
self.onDestroy = function() { clearInterval(timer); subscriptions.forEach(function(s){s.unsubscribe();}); };
