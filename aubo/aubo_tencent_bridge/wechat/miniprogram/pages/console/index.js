const {derive, demoSnapshot, errorText} = require('../../utils/view');
const names = {move_to_observation: '前往观察位', start: '开始分拣', home: '返回结束位', stop: '停止任务'};
Page({
    data: {vm: derive(null, 0, false, false, false), demo: true, devices: [], deviceIndex: 0,
        device: '', message: '', openid: '', loading: false, switchingDisabled: false},
    onLoad() {
        this._visible = false;
        this._snapshot = null;
        this._pendingMotion = false;
        this._pendingStop = false;
    },
    async onShow() {
        const epoch = this._epoch = (this._epoch || 0) + 1;
        clearInterval(this._timer); clearInterval(this._poll);
        this._visible = true;
        this._config = getApp().globalData.config;
        this.setData({demo: this._config.demoMode === true});
        this._timer = setInterval(() => this._render(), 1000);
        if (this._config.demoMode) {
            this.setData({devices: [{id: '演示机器人', role: 'viewer'}], device: '演示机器人',
                message: '演示模式：展示样例数据，任务按钮保持禁用。'});
            this._poll = setInterval(() => this.refresh(), 3000);
            return this.refresh();
        }
        const error = getApp().globalData.configurationError;
        if (error) { this.setData({message: error}); return; }
        try {
            const who = await this._api({action: 'identity'});
            if (!this._visible || epoch !== this._epoch) return;
            this.setData({openid: who.openid});
            const list = await this._api({action: 'list'});
            if (!this._visible || epoch !== this._epoch) return;
            const selected = list.devices.find(x => x.id === this.data.device) || list.devices[0];
            this.setData({devices: list.devices, device: selected ? selected.id : '',
                deviceIndex: selected ? list.devices.indexOf(selected) : 0,
                message: selected ? '' : '尚无设备权限。请让管理员按下方 OpenID 在 aubo_acl 中授权。'});
            this._poll = setInterval(() => this.refresh(), 3000);
            await this.refresh();
        } catch (error) {
            if (this._visible && epoch === this._epoch) this.setData({message: errorText(error.message)});
        }
    },
    onHide() { this._visible = false; this._epoch++; clearInterval(this._timer); clearInterval(this._poll); },
    onUnload() { this.onHide(); },
    async _api(data) {
        const response = await wx.cloud.callFunction({name: this._config.functionName, data});
        if (!response.result || response.result.ok !== true) throw new Error(response.result ? response.result.error : 'cloud_service_unavailable');
        return response.result;
    },
    _render() {
        if (!this._visible) return;
        this.setData({switchingDisabled: this._pendingMotion || this._pendingStop,
            vm: derive(this._snapshot, Date.now() - (this._receivedAt || 0),
                this._pendingMotion, this._pendingStop, this._config.demoMode === true)});
    },
    async refresh() {
        if (!this._visible || this._fetching || (!this._config.demoMode && !this.data.device)) return;
        this._fetching = true;
        const device = this.data.device;
        const epoch = this._epoch;
        this.setData({loading: true});
        try {
            const snapshot = this._config.demoMode ? demoSnapshot(Date.now()) : await this._api({action: 'read', device});
            if (!this._visible || epoch !== this._epoch || device !== this.data.device) return;
            this._snapshot = snapshot;
            this._receivedAt = Date.now();
        } catch (error) {
            if (epoch === this._epoch && device === this.data.device) {
                if (this._snapshot) this._snapshot.fresh = false;
                if (this._visible) this.setData({message: errorText(error.message)});
            }
        } finally {
            this._fetching = false;
            if (this._visible) this.setData({loading: false});
            this._render();
        }
    },
    selectDevice(event) {
        if (this._pendingMotion || this._pendingStop) return;
        const index = Number(event.detail.value);
        this.setData({deviceIndex: index, device: this.data.devices[index].id, message: ''});
        this._snapshot = null;
        this._render();
        this.refresh();
    },
    copyOpenid() { if (this.data.openid) wx.setClipboardData({data: this.data.openid}); },
    async send(event) {
        if (this._config.demoMode) return;
        const method = event.currentTarget.dataset.method;
        const epoch = this._epoch;
        if (!Object.prototype.hasOwnProperty.call(names, method)) return;
        this._render();
        const allowed = () => method === 'stop' ? this.data.vm.canStop : method === 'start' ? this.data.vm.canStart : this.data.vm.canMove;
        if (!allowed()) return;
        if (method !== 'stop') {
            const answer = await new Promise(resolve => wx.showModal({title: names[method],
                content: '确认现场允许运动，并发送此任务？', success: resolve, fail: () => resolve({confirm: false})}));
            this._render();
            if (!answer.confirm || !this._visible || epoch !== this._epoch || !allowed()) return;
        }
        const device = this.data.device;
        if (method === 'stop') this._pendingStop = true;
        else this._pendingMotion = true;
        this.setData({message: '正在发送：' + names[method]}); this._render();
        let sent = false;
        try {
            const prepared = await this._api({action: 'prepare', device, method});
            if (!this._visible || epoch !== this._epoch) return;
            sent = true;
            const response = await this._api({action: 'command', device, document: prepared.document});
            if (this._visible && epoch === this._epoch) this.setData({message: response.result.status === 'accepted' ?
                '请求已接收，请查看任务状态确认执行结果。' : response.result.status === 'unknown' ?
                '请求结果未知，请核对本地状态，不要直接重复发起运动。' :
                '请求被拒绝：' + (response.result.message || response.result.reason || '')});
        } catch (error) {
            if (this._visible && epoch === this._epoch) this.setData({message: sent ?
                '请求结果未确认，请核对本地状态，不要直接重复发起运动。' : errorText(error.message)});
        } finally {
            if (method === 'stop') this._pendingStop = false;
            else this._pendingMotion = false;
            this._render();
            await this.refresh();
        }
    }
});
