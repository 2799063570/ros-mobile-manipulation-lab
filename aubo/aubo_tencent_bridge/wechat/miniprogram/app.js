const config = require('./config');
App({
    globalData: {config, configurationError: ''},
    onLaunch() {
        if (config.demoMode) return;
        if (!config.cloudEnvId || !wx.cloud) {
            this.globalData.configurationError = '请配置云开发环境 ID，并使用支持云开发的微信版本。';
            return;
        }
        wx.cloud.init({env: config.cloudEnvId, traceUser: false});
    }
});
