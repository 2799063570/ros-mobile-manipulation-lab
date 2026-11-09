'use strict';
const cloud = require('wx-server-sdk');
const {iotexplorer} = require('tencentcloud-sdk-nodejs-iotexplorer');
const {createHandler} = require('./core');
cloud.init({env: cloud.DYNAMIC_CURRENT_ENV});

exports.main = async event => {
    try {
        const env = process.env;
        const devices = JSON.parse(env.IOT_DEVICES || '{}');
        if (!env.WX_APP_ID || !env.IOT_REGION || !Object.keys(devices).length) {
            return {ok: false, error: 'server_not_configured'};
        }
        // Each request only receives server-configured product/device IDs.
        for (const value of Object.values(devices)) {
            if (!value || Object.keys(value).sort().join(',') !== 'DeviceName,ProductId' ||
                ![value.ProductId, value.DeviceName].every(x => typeof x === 'string' && /^[A-Za-z0-9_-]{1,64}$/.test(x))) {
                return {ok: false, error: 'server_not_configured'};
            }
        }
        const client = new iotexplorer.v20190423.Client({
            credential: {
                secretId: env.IOT_SECRET_ID || env.TENCENTCLOUD_SECRETID,
                secretKey: env.IOT_SECRET_KEY || env.TENCENTCLOUD_SECRETKEY,
                token: env.IOT_SESSION_TOKEN || env.TENCENTCLOUD_SESSIONTOKEN
            },
            region: env.IOT_REGION,
            profile: {httpProfile: {endpoint: 'iotexplorer.tencentcloudapi.com', reqTimeout: 8}}
        });
        return await createHandler({identity: () => cloud.getWXContext(),
            acl: async openid => {
                const result = await cloud.database().collection('aubo_acl').where({_id: openid}).limit(1).get();
                return result.data[0] || {};
            },
            client, devices, appId: env.WX_APP_ID, commandsEnabled: env.ENABLE_COMMANDS === 'true'
        })(event);
    } catch (_) {
        return {ok: false, error: 'server_not_configured'};
    }
};
