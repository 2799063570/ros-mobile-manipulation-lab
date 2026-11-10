'use strict';
const $ = id => document.getElementById(id);
const stateNames = {INITIALIZING:'初始化',IDLE:'等待操作',READY:'已就绪',OBSERVING:'前往观察位',DETECTING:'识别与队列',SORTING:'分拣中',PICKING:'抓放执行',HOMING:'返回结束位',STOPPED:'已停止',ERROR:'任务失败',UNKNOWN:'未知状态'};
const phaseNames = {IDLE:'等待操作',DETECTING:'识别入队',OPEN_GRIPPER:'夹爪全开',PREGRASP:'接近目标',DESCEND:'下降抓取',GRASP:'夹紧目标',LIFT:'抬升目标',PREPLACE:'前往容器',PLACE:'下降放置',RELEASE:'释放目标',RETREAT:'退离容器',PICK_DONE:'本次抓放完成'};
const statusNames = {candidate:'待确认',ready:'可领取',reserved:'执行中',done:'已完成',review:'待复核'};
const categories = {bottle:'瓶',can:'罐',box:'盒',red:'红色',green:'绿色',blue:'蓝色'};
const actions = {move_to_observation:'前往观察位',start:'开始队列分拣',stop:'停止任务',home:'返回结束位'};
const colors = {bottle:'#059669',can:'#0284c7',box:'#b45309',red:'#cf5555',green:'#059669',blue:'#0284c7'};
let token = '', snapshot = null, receivedAt = 0, online = false, pending = false, stopping = false, selected = null, lastSignature = '', serial = 0, refreshing = false;
function text(id, value) { $(id).textContent = value ?? '—'; }
function parse(value, fallback) { try { return typeof value === 'string' ? JSON.parse(value) : value ?? fallback; } catch { return fallback; } }
function num(value, digits=3) { return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—'; }
function log(message) {
  const li = document.createElement('li'), t = document.createElement('time'), body = document.createElement('span');
  t.textContent = new Date().toLocaleTimeString('zh-CN', {hour12:false}); body.textContent = message; li.append(t, body); $('logs').prepend(li);
  while ($('logs').children.length > 200) $('logs').lastChild.remove();
}
function showLogin() {
  token = ''; serial++; snapshot = null; online = false; text('source', '未连接'); text('state','等待连接'); text('state-code','—');
  text('detail','登录后查看设备状态'); text('fresh','● 等待数据'); text('update','尚未同步'); text('phase','等待设备数据'); text('description','');
  text('detection','识别摘要：—'); text('session','—'); text('available','等待数据'); text('cache-age','—'); text('settings-source','未连接');
  document.querySelectorAll('[data-count]').forEach(el => el.textContent = '—');
  text('completed-total','—');$('completed-counts').replaceChildren(document.createTextNode('等待读取完成统计'));
  $('rows').replaceChildren(); $('markers').replaceChildren(); $('queue-empty').hidden=false; text('queue-empty','等待队列数据'); $('map-empty').style.display='';
  $('failure').hidden=true; $('logout').hidden=true; $('logs').replaceChildren(); lastSignature=''; selected=null; text('feedback','');
  updateControls(); if (!$('login').open) $('login').showModal();
}
async function api(path, body) {
  const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), 20000);
  try {
    const r = await fetch('./api/' + path, {method:body === undefined ? 'GET' : 'POST', credentials:'omit', cache:'no-store',
      headers:{'Content-Type':'application/json', ...(token ? {'Authorization':'Bearer '+token} : {})},
      body:body === undefined ? undefined : JSON.stringify(body), signal:controller.signal});
    const data = await r.json();
    if (!r.ok) { if (r.status === 401 && path !== 'login') showLogin(); throw new Error(data.error || '请求失败'); }
    return data;
  } finally { clearTimeout(timeout); }
}
function currentFresh() {
  if (!snapshot || !online || !navigator.onLine || !snapshot.fresh) return false;
  const age = snapshot.age_s + (performance.now()-receivedAt)/1000;
  return age >= 0 && age <= (snapshot.mode === 'ros' ? 5 : 90);
}
function snapshotAgeLabel() {
  if (!snapshot || !(Number(snapshot.properties.bridge_wall_ms) > 0) || typeof snapshot.age_s !== 'number') return '尚无设备快照';
  return `快照 ${Math.max(0,Math.round(snapshot.age_s+(performance.now()-receivedAt)/1000))} 秒前`;
}
function updateControls() {
  const p = snapshot?.properties || {};
  const enabled = currentFresh() && snapshot?.controls_enabled && p.commands_enabled === 1 && p.sorting_available === 1;
  document.querySelectorAll('[data-method]').forEach(button => {
    const method = button.dataset.method;
    button.disabled = !enabled || (method === 'stop' ? stopping : pending || !['IDLE','READY','STOPPED','ERROR'].includes(p.state) || (method === 'start' && p.state !== 'READY'));
  });
  text('control-mode', enabled ? '任务控制已开放' : '只读监控');
  text('control-note', !token ? '登录后查看设备状态' : !currentFresh() ? '数据未更新或网络断开，操作已停用' : snapshot?.mode === 'demo' ? '演示数据 · 所有操作已禁用' : !enabled ? (snapshot?.mode === 'ros' ? '当前只读；启动工作台时可启用任务控制' : '当前只读；控制需在后台和 ROS 端分别启用') : pending ? '任务请求中；仍可发送停止' : '请先观察，等待 READY 后开始分拣');
  $('fresh').classList.toggle('good', currentFresh());
  if (snapshot) text('fresh', currentFresh() ? '● 快照有效' : '● 数据过期 / 连接中断');
}
function renderQueue() {
  const p = snapshot?.properties || {}, cache = parse(p.target_cache, {});
  const targets = Array.isArray(cache.targets) ? cache.targets.filter(r=>r && typeof r === 'object').slice(0,200) : [];
  const filter = $('filter').value, query = $('search').value.trim().toLowerCase();
  const shown = targets.filter(r => (!filter || r.status === filter) && (!query || (String(r.id)+' '+r.category+' '+(categories[r.category]||'')).toLowerCase().includes(query)));
  $('rows').replaceChildren();
  shown.forEach(r => {
    const row = document.createElement('tr'); row.tabIndex = 0; row.classList.toggle('selected', String(r.id) === selected);
    const pos = Array.isArray(r.position) ? r.position : [];
    const geometry = r.grasp_geometry_valid === true && r.grasp_width > 0;
    const age = r.age >= 0 && typeof p.target_cache_age_s === 'number' && p.target_cache_age_s >= 0 ? r.age + p.target_cache_age_s : null;
    const vals = [r.id, categories[r.category] || r.category, statusNames[r.status] || r.status,
      num(pos[0]),num(pos[1]),num(pos[2]),geometry ? num(r.grasp_angle*180/Math.PI,1) : '—',
      geometry ? num(r.grasp_width*1000,1) : '—', typeof r.detection_confidence === 'number' ? num(r.detection_confidence*100,1)+'%' : '—',r.observations,num(age,1)];
    vals.forEach((val,index) => { const td = document.createElement('td'); if(index === 2) { const badge = document.createElement('span'); badge.className='badge '+(Object.hasOwn(statusNames,r.status) ? r.status : ''); badge.textContent=val ?? '—'; td.append(badge); } else td.textContent=val ?? '—'; row.append(td); });
    const select = () => { selected=String(r.id); renderQueue(); };
    row.addEventListener('click',select); row.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();select();}}); $('rows').append(row);
  });
  $('queue-empty').hidden = shown.length > 0; text('queue-empty',targets.length ? '没有匹配的目标' : '当前没有排队目标');
  text('queue-note',p.target_cache_truncated ? `云端快照已裁剪，仅展示 ${targets.length} 个目标；队列总数以统计为准。` : '点击目标可在平面图中定位；缺失字段显示“—”。');
  const center=snapshot?.workcell?.table_center,size=snapshot?.workcell?.table_size;
  const points=targets.map(r=>r.position).filter(p=>Array.isArray(p)&&p.length===3&&p.every(v=>typeof v==='number'&&Number.isFinite(v)));
  let xmin=-.5,xmax=.4,ymin=-1.2,ymax=-.1;
  if(Array.isArray(center)&&Array.isArray(size)&&center.length>=2&&size.length>=2&&[...center.slice(0,2),...size.slice(0,2)].every(Number.isFinite)&&size[0]>0&&size[1]>0){
    [xmin,xmax,ymin,ymax]=[center[0]-size[0]/2,center[0]+size[0]/2,center[1]-size[1]/2,center[1]+size[1]/2];
  }else if(points.length){
    const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
    const xpad=Math.max(.1,(Math.max(...xs)-Math.min(...xs))*.15),ypad=Math.max(.1,(Math.max(...ys)-Math.min(...ys))*.15);
    [xmin,xmax,ymin,ymax]=[Math.min(...xs)-xpad,Math.max(...xs)+xpad,Math.min(...ys)-ypad,Math.max(...ys)+ypad];
  }
  text('map-xmin',num(xmin,2));text('map-xmid',num((xmin+xmax)/2,2));text('map-xmax',num(xmax,2));text('map-ymin',num(ymin,2));text('map-ymax',num(ymax,2));
  text('map-frame','XY / m'+(snapshot?.workcell?.frame ? ' · '+snapshot.workcell.frame : ''));
  $('map-legend').replaceChildren();
  [...new Set(targets.map(r=>r.category))].slice(0,6).forEach(category=>{const item=document.createElement('span'),dot=document.createElement('i');dot.style.background=colors[category]||'#087e8b';item.append(dot,document.createTextNode(categories[category]||category||'其他'));$('map-legend').append(item);});
  $('markers').replaceChildren(); let valid=0, outside=0;
  targets.forEach(r=>{
    const pos=r.position;
    if(!Array.isArray(pos)||pos.length!==3||!pos.every(v=>typeof v==='number'&&Number.isFinite(v))) return;
    if(pos[0]<xmin||pos[0]>xmax||pos[1]<ymin||pos[1]>ymax){outside++;return;} valid++;
    const ns='http://www.w3.org/2000/svg', g=document.createElementNS(ns,'g'), circle=document.createElementNS(ns,'circle'), label=document.createElementNS(ns,'text'), title=document.createElementNS(ns,'title');
    const x=45+(pos[0]-xmin)/(xmax-xmin)*430, y=205-(pos[1]-ymin)/(ymax-ymin)*190;
    circle.setAttribute('cx',x);circle.setAttribute('cy',y);circle.setAttribute('r',String(r.id)===selected ? 9 : 6);
    circle.setAttribute('fill', ['done','review'].includes(r.status) ? '#93a5b0' : colors[r.category] || '#087e8b');
    circle.setAttribute('stroke', '#fff');circle.setAttribute('stroke-width','2');label.setAttribute('x',x+10);label.setAttribute('y',y+4);label.setAttribute('fill','#546f80');label.setAttribute('font-size','11');label.textContent='#'+r.id;
    title.textContent=`#${r.id} ${r.category} (${num(pos[0])}, ${num(pos[1])})`;g.append(title,circle,label);$('markers').append(g);
  });
  $('map-empty').style.display=valid ? 'none' : ''; text('map-count',`${valid} 个有坐标目标${outside ? ' · '+outside+' 个超出图示范围' : ''}`);
}
function renderCompleted(statistics) {
  $('completed-counts').replaceChildren();
  if(!statistics || typeof statistics.counts!=='object' || statistics.counts===null){text('completed-total','—');text('completed-counts','等待读取完成统计');return;}
  text('completed-total','合计 '+statistics.total+' 件');
  const counts=Object.entries(statistics.counts);
  if(!counts.length)text('completed-counts','暂无类别');
  counts.forEach(([category,count])=>{const item=document.createElement('span'),value=document.createElement('b');value.textContent=String(count);item.append(document.createTextNode(category+'：'),value);$('completed-counts').append(item);});
}
function render(data) {
  snapshot=data;receivedAt=performance.now();online=true;const p=data.properties;
  text('poll-note',`快照约 ${data.poll_seconds || 30} 秒更新`);
  text('source', data.mode==='demo' ? '演示数据 · 只读' : data.mode==='ros' ? 'ROS · 本机设备' : '华为云 · 设备影子');
  text('settings-source',data.mode==='demo' ? '本地演示数据（非真实设备）' : data.mode==='ros' ? '本机 ROS / Sorting' : '华为云 IoTDA / Sorting');
  text('state',stateNames[p.state]||p.state||'未知状态');text('state-code',p.state);
  text('detail',p.state_detail);text('phase',phaseNames[p.phase]||p.phase||'等待操作');text('description',p.description);
  text('update',snapshotAgeLabel());
  $('failure').hidden=!p.failure;text('failure',p.failure);text('detection','识别摘要：'+(p.detection_summary||'—'));
  text('available',p.sorting_available===1 ? '接口可用' : '接口未就绪');text('session',p.bridge_session);text('cache-age',p.target_cache_age_s>=0 ? num(p.target_cache_age_s,1)+' s' : '尚未收到队列');
  document.querySelectorAll('[data-count]').forEach(el=>el.textContent=p['queue_'+el.dataset.count]??'—');
  renderCompleted(p.completed_counts);
  const step=p.state==='OBSERVING' ? 0 : ['DETECTING','SORTING'].includes(p.state) ? 1 : p.state==='PICKING' ? (['PREPLACE','PLACE','RELEASE','RETREAT','PICK_DONE'].includes(p.phase) ? 3 : 2) : p.state==='HOMING' ? 4 : -1;
  document.querySelectorAll('.steps li').forEach((li,i)=>{li.classList.toggle('active',i===step);if(i===step)li.setAttribute('aria-current','step');else li.removeAttribute('aria-current');});
  const sig=JSON.stringify([p.state,p.state_detail,p.phase,p.description,p.failure,p.last_command]);
  if(sig!==lastSignature){log(`${stateNames[p.state]||p.state} · ${p.state_detail||''}\n${phaseNames[p.phase]||p.phase||''} ${p.description||''}${p.failure ? '\n故障：'+p.failure : ''}`); const command=parse(p.last_command,{});if(command.status)log('云桥最近命令：'+JSON.stringify(command));lastSignature=sig;}
  updateControls();renderQueue();
}
async function refresh() {
  if(!token || refreshing) return; refreshing=true;const requestSerial=serial;
  try{const data=await api('snapshot');if(requestSerial===serial && token)render(data);}
  catch(e){online=false;updateControls();text('feedback',e.message==='Failed to fetch' ? '连接中断，等待重新同步' : e.message);}
  finally{refreshing=false;}
}
$('login-form').addEventListener('submit',async e=>{e.preventDefault();
  if(location.protocol!=='https:' && !['localhost','127.0.0.1','[::1]'].includes(location.hostname)){text('login-error','请使用 HTTPS 公网地址登录');return;}const button=e.target.querySelector('button');button.disabled=true;
  try{const data=await api('login',{password:$('password').value});token=data.token;serial++;$('password').value='';$('login-error').textContent='';$('login').close();$('logout').hidden=false;log('已连接工作台');await refresh();}
  catch(error){text('login-error',error.message);}finally{button.disabled=false;}});
$('login').addEventListener('cancel',e=>e.preventDefault());
$('refresh').addEventListener('click',refresh);$('logout').addEventListener('click',showLogin);
$('filter').addEventListener('change',renderQueue);$('search').addEventListener('input',renderQueue);$('clear-log').addEventListener('click',()=>$('logs').replaceChildren());
// Tab labels retain native button keyboard behavior.
document.querySelectorAll('[data-tab]').forEach(button=>button.addEventListener('click',()=>{
  document.querySelectorAll('[data-tab]').forEach(b=>{b.classList.toggle('selected',b===button);b.setAttribute('aria-selected',String(b===button));});
  document.querySelectorAll('.tab-content').forEach(panel=>panel.hidden=panel.id!==button.dataset.tab);
}));
async function send(method) {
  if(method==='stop' ? stopping : pending) return;
  if(method!=='stop') {
    text('confirm-text',`向设备发送“${actions[method]}”？`);$('confirm').returnValue='';$('confirm').showModal();
    await new Promise(resolve=>$('confirm').addEventListener('close',resolve,{once:true}));
    if($('confirm').returnValue!=='ok')return;
  }
  if(!currentFresh()){text('feedback','快照已过期，请先刷新');return;}
  if(method==='stop')stopping=true;else pending=true;updateControls();
  try{
    const prepared=await api('prepare',{method});const result=await api('command',prepared);
    const reply=result.response?.response;
    const local=parse(reply?.paras?.result, {});
    const outcome=local.status || (reply?.result_code===0 ? 'accepted' : reply?.result_code===1 ? 'rejected' : 'unknown');
    const message=result.status==='unknown' ? result.message : ({accepted:'任务请求已接收',rejected:'任务被拒绝',unknown:'任务结果未知，请核对设备状态'}[outcome] || '已收到应答')+'。'+(local.message || local.reason || '请查看设备后续状态');
    text('feedback',message);log(actions[method]+'：'+message);
  }catch(e){text('feedback','操作未确认：'+e.message+'。请核对设备状态；不要重复发起同一动作。');log(actions[method]+'：'+$('feedback').textContent);}
  finally{if(method==='stop')stopping=false;else pending=false;updateControls();refresh();}
}
document.querySelectorAll('[data-method]').forEach(button=>button.addEventListener('click',()=>send(button.dataset.method)));
window.addEventListener('offline',()=>{online=false;updateControls();text('feedback','网络已断开，任务操作已停用');});
window.addEventListener('online',refresh);document.addEventListener('visibilitychange',()=>{if(!document.hidden)refresh();});
setInterval(()=>{updateControls();if(snapshot)text('update',snapshotAgeLabel());},1000);
let nextPoll=0;
setInterval(()=>{if(!document.hidden && performance.now() >= nextPoll){nextPoll=performance.now()+(snapshot?.poll_seconds || 30)*1000;refresh();}},1000);
showLogin();
