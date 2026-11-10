const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=__dirname,now=1794200000000;
const sample={device_id:'test-device',service_id:'Sorting',bridge_wall_ms:String(now),
 state:'READY',statistics_available:'1',completed_total:'6',
 completed_counts:JSON.stringify({scope:'node_session',session:'first-run',counts:{bottle:1,can:2,box:3},total:6}),
 target_cache:JSON.stringify({targets:[{id:1,category:'bottle',status:'ready',position:[.54,.12,.1]}]}),
 workcell:JSON.stringify({table_center:[.7,0],table_size:[.7,1],frame:'base_link'})};
function run(name,rows){const code=fs.readFileSync(path.join(root,'converters',name),'utf8').replace('YOUR_DEVICE_ID','test-device');return new Function('data','Date',code)(rows,{now:()=>now});}
const history=[{...sample,bridge_wall_ms:String(now-30000),completed_total:'100'},sample,{...sample,device_id:'other-device',bridge_wall_ms:String(now+1),completed_total:'999'}];
assert.equal(run('completed_total.js',history)[0].value,6);
assert.deepEqual(run('completed_by_class.js',history).map(r=>r.value),[1,2,3]);
assert.equal(run('projection.js',history)[0].x,.54);
assert.deepEqual(run('projection.js',[{...sample,bridge_wall_ms:String(now-90001)}]),[]);
assert.equal(run('status.js',[{...sample,bridge_wall_ms:String(now-90001)}])[0].fresh,false);
assert.deepEqual(run('completed_total.js',[{...sample,statistics_available:'0'}]),[]);
const reset={...sample,bridge_wall_ms:String(now+0),completed_total:'0',completed_counts:JSON.stringify({session:'new-run',counts:{bottle:0,can:0,box:0},total:0})};
assert.equal(run('completed_total.js',[sample,reset])[0].value,0);
assert.deepEqual(run('projection.js',[{...sample,target_cache:'invalid JSON'}]),[]);
assert.equal(run('queue_counts.js',[sample]).length,6);
console.log('PASS DLV converter examples: latest same-device snapshot, no history sum, session reset, stale projection and invalid data');
