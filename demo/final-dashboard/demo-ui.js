const $=s=>document.querySelector(s);
if(new URLSearchParams(location.search).has('monitor'))document.body.classList.add('monitor-only');
let sceneBusy=false,mapEntries=[],sceneState=null,selectedMap=null,mapSelectionTouched=false;
let latest=null,sessionUrl=null,mapImage=null,mapWalls=null,mapHash=null,zoom=1,trail=[],lastSession=null,lastFrameAt=0,oldBlob=null,camBusy=false,camToken=0;
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const time=v=>v?new Date(v).toLocaleTimeString('zh-CN',{hour12:false}):'—';
const base=n=>String(n||'').replace(/^mcp_robot_mcp_[^_]+_/,'').replace(/^mcp_robot_mcp_/,'');
const ctx=$('#live-map').getContext('2d');
function canvasSize(canvas){const r=canvas.getBoundingClientRect(),d=window.devicePixelRatio||1;canvas.width=Math.max(1,Math.round(r.width*d));canvas.height=Math.max(1,Math.round(r.height*d));const c=canvas.getContext('2d');c.setTransform(d,0,0,d,0,0);return {c,w:r.width,h:r.height};}
function viewBounds(data){const m=data?.ros?.map;if(!m)return {x:-3,y:-7,w:15,h:11};const o=m.origin,ls=data.registry?.locations||[],bs=ls.map(x=>x.bounds).filter(Boolean);if(bs.length){const x=Math.min(...bs.map(b=>b.min_x),data.ros?.pose?.x??0)-.35,y=Math.min(...bs.map(b=>b.min_y),data.ros?.pose?.y??0)-.35;return{x,y,w:Math.max(...bs.map(b=>b.max_x),data.ros?.pose?.x??0)+.35-x,h:Math.max(...bs.map(b=>b.max_y),data.ros?.pose?.y??0)+.35-y};}return{x:o.x,y:o.y,w:m.width*m.resolution,h:m.height*m.resolution};}
// Display-only occupied-cell mask; never changes ROS occupancy or collision geometry.
function wallMask(img){
 const mask=document.createElement('canvas');mask.width=img.naturalWidth;mask.height=img.naturalHeight;
 const c=mask.getContext('2d');c.drawImage(img,0,0);
 const d=c.getImageData(0,0,mask.width,mask.height).data,path=new Path2D();
 // Contiguous occupied-cell runs form one solid outline, not shifted thin copies.
 for(let y=0;y<mask.height;y++){
  let start=-1;
  for(let x=0;x<=mask.width;x++){
   const i=(y*mask.width+x)*4,occupied=x<mask.width&&d[i]<128&&d[i+1]<128&&d[i+2]<128&&d[i+3]>0;
   if(occupied&&start<0)start=x;
   if(!occupied&&start>=0){path.rect(start,y,x-start,1);start=-1;}
  }
 }
 return path;
}
function renderMap(){
 const {c,w,h}=canvasSize($('#live-map'));c.fillStyle='#f8f9fa';c.fillRect(0,0,w,h);if(sceneBusy){c.fillStyle='#73777e';c.font='15px Inter,system-ui';c.textAlign='center';c.fillText('正在切换物理场景 · 等待新 ROS2 /map 与 TF',w/2,h/2);return;}if(!latest?.ros?.map)return;
 const b=viewBounds(latest),s=Math.min((w-60)/b.w,(h-60)/b.h)*zoom,cx=b.x+b.w/2,cy=b.y+b.h/2;
 const p=(x,y)=>({x:w/2+(x-cx)*s,y:(h-22)/2-(y-cy)*s});
 const m=latest.ros.map,o=m.origin;
 if(mapImage){const top=p(o.x,o.y+m.height*m.resolution);c.save();c.translate(top.x,top.y);c.rotate(-o.yaw);c.imageSmoothingEnabled=false;c.drawImage(mapImage,0,0,m.width*m.resolution*s,m.height*m.resolution*s);
  // At least 6 CSS px at every zoom, regardless of the imported map's cell size.
  if(mapWalls){
   const cell=m.resolution*s;c.save();c.scale(cell,cell);
   c.fillStyle=c.strokeStyle='#1C1F23';c.lineJoin='round';
   c.lineWidth=Math.max(0,6/cell-1);
   if(c.lineWidth>0)c.stroke(mapWalls);
   c.fill(mapWalls);c.restore();
  }
  c.restore();}
 c.font='13px Inter,"Noto Sans SC",system-ui,sans-serif';c.textAlign='center';
 for(const loc of latest.registry?.locations||[]){
  if(!loc.pose)continue;
  if((latest.registry?.locations?.length||0)>10){
   if(loc.type==='corridor'&&zoom<1.4)continue;
   if(loc.id!=='Start'&&Math.hypot(loc.pose.x-(latest.registry.locations.find(x=>x.id==='Start')?.pose?.x??999),loc.pose.y-(latest.registry.locations.find(x=>x.id==='Start')?.pose?.y??999))<.05)continue;
  }
  const a=p(loc.pose.x,loc.pose.y),offsets={Start:[-8,14],Room_A:[-14,-22],Corridor_A:[20,25],Room_B:[18,-16],Room_C:[-8,5],Side_Corridor:[5,5]},off=offsets[loc.id]||[0,-12],x=a.x+off[0],y=a.y+off[1];
  const label=({A_Site:'A 点',B_Site:'B 点',C_Site:'C 点',Start:'起点',Defender_Spawn:'防守出生点',Attacker_Spawn:'进攻出生点',A_Tree:'A 树屋'})[loc.id]||loc.id.replaceAll('_',' '),t=c.measureText(label).width;c.fillStyle='#ffffffef';c.fillRect(x-t/2-4,y-14,t+8,19);c.fillStyle='#646970';c.fillText(label,x,y);
 }
 if(latest.ros.pose_fresh){
  c.strokeStyle='#F4772D';c.lineWidth=2.5;c.lineCap='round';c.beginPath();trail.forEach((v,i)=>{const a=p(v.x,v.y);i?c.lineTo(a.x,a.y):c.moveTo(a.x,a.y)});c.stroke();
  const pose=latest.ros.pose,a=p(pose.x,pose.y),r=10;
  c.save();c.translate(a.x,a.y);c.rotate(-pose.yaw);c.fillStyle='#F4772D';c.strokeStyle='#fff';c.lineWidth=2.5;c.beginPath();c.moveTo(r+5,0);c.lineTo(-r,-r*.8);c.lineTo(-r*.65,0);c.lineTo(-r,r*.8);c.closePath();c.fill();c.stroke();c.restore();
 }
 const call=(latest.session?.events||[]).findLast(e=>e.type==='tool_call'&&base(e.tool)==='navigate_to'),target=(latest.registry?.locations||[]).find(l=>l.id===call?.args?.location);
 if(target&&latest.session?.status==='RUNNING'){const a=p(target.pose.x,target.pose.y);c.setLineDash([4,4]);c.strokeStyle='#1C1F23';c.lineWidth=2;c.beginPath();c.arc(a.x,a.y,16,0,Math.PI*2);c.stroke();c.setLineDash([]);}
 if(latest.ros.navigation?.status==='ACTIVE'&&latest.ros.plan_received_at&&Date.now()-latest.ros.plan_received_at*1000<3000){c.setLineDash([5,5]);c.strokeStyle='#1C1F23';c.lineWidth=1.5;c.beginPath();latest.ros.plan.forEach((v,i)=>{const a=p(v.x,v.y);i?c.lineTo(a.x,a.y):c.moveTo(a.x,a.y)});c.stroke();c.setLineDash([]);}
 const scale=Math.max(1,Math.round(60/s));c.fillStyle='#1C1F23';c.fillRect(16,h-47,scale*s,2);c.textAlign='left';c.font='10px Inter,system-ui';c.fillText(scale+' m',16,h-53);
}
function render(data){
 latest=data;document.body.classList.remove('offline');
 const native=data.native,session=data.session,ros=data.ros,ready=native?.daemon_ready&&native?.observer_fresh&&native?.robot_mcp?.state==='ready'&&native?.robot_mcp?.tools?.length===8&&native?.plugin?.actual==='running'&&ros?.pose_fresh;
 $('#system-ready').textContent=ready?'组件 READY':'组件 DEGRADED';$('#system-ready').className='pill'+(ready?' live':'');
 if(native?.web_url&&native.web_url!==sessionUrl){sessionUrl=native.web_url;$('#native-chat').src=sessionUrl;$('#open-native').href=sessionUrl;$('#trace-link').href=sessionUrl;$('#session-id').textContent='Session '+native.session_id;}
 updateChatConnection();
 $('#last-sync').textContent='同步 '+time(data.server_time)+' · ROS_DOMAIN_ID=106';
 $('#map-status').textContent=ros?.pose_fresh?'LIVE · TF':'无实时位姿';$('#map-status').className='pill'+(ros?.pose_fresh?' live':'');
 $('#map-warning').hidden=!!ros?.pose_fresh;$('#map-warning').textContent='无实时位姿 · 仅显示静态 /map 先验';
 const p=ros?.pose_fresh?ros.pose:null;
 $('#pose-value').textContent=p?p.x.toFixed(3)+' / '+p.y.toFixed(3):'无实时位姿';
 $('#nav-value').textContent=(p?(p.yaw*180/Math.PI).toFixed(1)+'° · ':'')+(ros?.fresh?ros.navigation.status:'UNKNOWN');
 const goal=(session?.events||[]).findLast(e=>e.type==='tool_call'&&base(e.tool)==='navigate_to');
 $('#goal-value').textContent=session?.status==='RUNNING'?(goal?.args?.location||'尚无导航目标'):'暂无活动目标';
 $('#map-source').textContent='静态先验：ROS2 /map · 实时：map → base_link'+(ros?.pose_fresh?' · TF age '+Math.round(ros.pose_age_ms)+' ms':' · TF 不可用')+' · seed '+(data.scenario?.seed??'—');
 if(session?.session_id!==lastSession){trail=[];lastSession=session?.session_id;}
 if(p){const prev=trail.at(-1);if(!prev||Math.hypot(p.x-prev.x,p.y-prev.y)>.015)trail.push({...p});if(trail.length>800)trail.shift();}
 if(ros?.map?.sha256&&ros.map.sha256!==mapHash){mapHash=ros.map.sha256;const img=new Image();img.onload=()=>{mapImage=img;mapWalls=wallMask(img);renderMap();};img.onerror=()=>{mapImage=null;mapWalls=null;mapHash=null;};img.src='/api/demo-ui/map.png?hash='+encodeURIComponent(mapHash);}
 renderMap();
 const fresh=native?.observer_fresh, status=fresh?(session?.status||'IDLE'):'UNKNOWN';
 $('#mission-state').textContent=status;$('#mission-stage').textContent=fresh?(session?.outcome?.reasons?.includes('READONLY_ROBOT_STATE_NOT_OBSERVED')?'缺少本轮真实 Robot MCP 证据':session?.stage||'等待原生聊天任务'):'Session 事件流已失联';
 $('#mission-task').textContent=session?.task||'在左侧输入“现在机器人在哪里？”开始只读检查。';$('#mission-task').title=session?.task||'';
 $('#progress-track').className='progress-track'+(['RUNNING','QUEUED'].includes(status)?' running':['SUCCEEDED','PARTIAL','FAILED','CANCELED'].includes(status)?' terminal'+(status==='SUCCEEDED'?' succeeded':''):'');
 const prior=data.previous_mission;$('#previous-mission').textContent=prior?.mission_id?'HISTORICAL · '+prior.mission_id+' / '+prior.status+' · 不是本次任务结果':'无历史任务';
 if($('#evidence-dialog').open)renderEvidence();
 if(sceneBusy){$('#map-status').textContent='切换中 · 非 LIVE';$('#map-warning').hidden=false;$('#map-warning').textContent='正在重建场景，不将旧地图标为实时';$('#pose-value').textContent='切换中';$('#nav-value').textContent='等待新场景';$('#system-ready').textContent='场景切换中';}
 updateSwitchButton();
 if(!data.camera?.live||sceneBusy)noCamera(sceneBusy?'物理场景切换中 · 等待新 Camera 帧':'最新 Camera 帧已缺失或过期');
}
function noCamera(reason){lastFrameAt=0;$('#camera-image').hidden=true;$('#camera-empty').hidden=false;$('#camera-empty p').textContent=reason;$('#camera-status').textContent='无实时画面';$('#camera-status').className='pill';$('#camera-time').textContent='最新帧时间 —';}
async function poll(){try{const r=await fetch('/api/demo-ui/state',{cache:'no-store',signal:AbortSignal.timeout(3500)});if(!r.ok)throw Error('HTTP '+r.status);render(await r.json());}catch(e){document.body.classList.add('offline');$('#system-ready').textContent='连接中断';$('#last-sync').textContent='网络断开 · 监控自动重连';$('#map-status').textContent='无实时位姿';$('#map-warning').hidden=false;$('#map-warning').textContent='连接中断 · 仅显示静态地图';$('#pose-value').textContent='无实时位姿';$('#nav-value').textContent='UNKNOWN';$('#mission-state').textContent='UNKNOWN';$('#progress-track').className='progress-track';if(latest?.ros)latest.ros.pose_fresh=false;renderMap();noCamera('网络断开 · 无实时画面');}finally{setTimeout(poll,600);}}
async function camera(){
 if(sceneBusy){noCamera('物理场景切换中 · 等待新 Camera 帧');setTimeout(camera,500);return;}if(camBusy)return;camBusy=true;const token=++camToken;
 try{
  const r=await fetch('/api/demo-ui/camera.png?t='+Date.now(),{cache:'no-store',signal:AbortSignal.timeout(2300)});if(!r.ok||!r.headers.get('content-type')?.includes('image/png'))throw Error('NO_LIVE_FRAME');
  const captured=r.headers.get('x-captured-at'),at=Date.parse(captured),frame=r.headers.get('x-frame-id');
  const blob=await r.blob();if(!Number.isFinite(at)||Date.now()-at>2500)throw Error('STALE_FRAME');
  const url=URL.createObjectURL(blob),img=$('#camera-image');
  img.onload=()=>{if(sceneBusy||token!==camToken||Date.now()-at>2500){URL.revokeObjectURL(url);return;}if(oldBlob)URL.revokeObjectURL(oldBlob);oldBlob=url;lastFrameAt=at;img.hidden=false;$('#camera-empty').hidden=true;$('#camera-status').textContent='LIVE · PNG';$('#camera-status').className='pill live';$('#camera-time').textContent='最新帧 '+time(captured)+' · '+Math.max(0,Date.now()-at)+' ms';$('#camera-frame').textContent=frame;};
  img.onerror=()=>{URL.revokeObjectURL(url);noCamera('PNG 图像呈现失败');};img.src=url;
 }catch(e){noCamera('当前未收到新鲜 Camera 帧 · 自动重试');}finally{camBusy=false;setTimeout(camera,500);}
}
function renderEvidence(){
 const s=latest?.session;$('#evidence-id').textContent='Session '+(s?.session_id||'—')+' · Mission '+(s?.mission_id||'—')+' · Turn '+(s?.prompt_turn_id||'—');
 $('#evidence-report').textContent=(s?.report||'尚无真实最终报告')+'\n\n'+(s?.outcome?JSON.stringify({status:s.outcome.status,reasons:s.outcome.reasons},null,2):'任务尚未终结；没有可信完成百分比。');
 $('#evidence-events').innerHTML=(s?.events||[]).slice(-24).reverse().map(e=>'<div class="event-row"><b>'+esc(e.tool)+'</b><small>'+esc(e.type)+' · '+time(e.timestamp)+' · seq '+e.sequence+'</small><details><summary>'+esc(e.result?.status||e.result?.navigation_status||e.type)+'</summary><pre>'+esc(JSON.stringify(e.result??e.args,null,2))+'</pre></details></div>').join('');
}
// Same-origin native document is observed, never rewritten or imitated.
function updateChatConnection(){
 const native=latest?.native;
 let doc=null,loaded=false,disconnected=false;
 try{doc=$('#native-chat').contentDocument;loaded=!!doc?.querySelector('textarea');disconnected=!!doc?.querySelector('#reconnect-notice')&&!doc.querySelector('#reconnect-notice').hidden&&doc.querySelector('textarea')?.disabled===true;}catch{}
 $('#frame-wait').hidden=loaded;
 const recovery=$('#chat-recovery');
 recovery.hidden=!disconnected;
 if(disconnected){
  let port=null;try{port=new URL(native?.ws_url).port;}catch{}
  recovery.textContent=port&&/^\d+$/.test(port)?'原生聊天未连接。页面和监控可用不代表聊天已连接。请在 Windows PowerShell 保持以下转发运行：ssh -N -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -L 127.0.0.1:'+port+':127.0.0.1:'+port+' guoyu@192.168.97.11；然后点击原生 Retry connection 或“演示会话”。':'原生聊天未连接；请检查 WebSocket 转发，再点击 Retry connection。';
  $('#chat-connection').textContent='聊天断连 · 原生输入已禁用';
  $('#system-ready').textContent='组件 DEGRADED · 聊天断连';
 }else{
  $('#chat-connection').textContent=native?.observer_fresh?(loaded?'原生聊天已载入 · ':'原生 Session 已绑定 · ')+'Robot MCP '+(native.robot_mcp?.tools?.length??'—')+' 工具':'会话监控断开 · 请检查连接';
 }
}
setInterval(updateChatConnection,500);
$('#restore-session').onclick=()=>{if(sessionUrl)$('#native-chat').src=sessionUrl;};
$('#zoom-in').onclick=()=>{zoom=Math.min(3,zoom*1.2);renderMap();};$('#zoom-out').onclick=()=>{zoom=Math.max(.5,zoom/1.2);renderMap();};$('#zoom-reset').onclick=()=>{zoom=1;renderMap();};
new ResizeObserver(renderMap).observe($('.map-wrap'));
$('#open-import').onclick=()=>{$('#import-dialog').showModal();loadMapCatalog();};
$('#choose-map').onclick=$('#open-import').onclick;$('#close-import').onclick=()=>$('#import-dialog').close();
$('#show-evidence').onclick=()=>{renderEvidence();$('#evidence-dialog').showModal();};$('#close-evidence').onclick=()=>$('#evidence-dialog').close();
function previewGeometry(g){
 const canvas=$('#import-map');canvas.hidden=false;const {c,w,h}=canvasSize(canvas),b=g.boundary,s=Math.min((w-40)/(b.max_x-b.min_x),(h-40)/(b.max_y-b.min_y)),cx=(b.min_x+b.max_x)/2,cy=(b.min_y+b.max_y)/2,p=(x,y)=>({x:w/2+(x-cx)*s,y:h/2-(y-cy)*s});
 c.fillStyle='#f8f9fa';c.fillRect(0,0,w,h);
 const areas=[...(g.rooms||[]),...(g.corridors||[])];
 for(const r of areas){const a=p(r.bounds.min_x,r.bounds.max_y);c.fillStyle=/^[ABC]_Site$/.test(r.id)?'#fff0e5':'#e8eaec';c.fillRect(a.x,a.y,(r.bounds.max_x-r.bounds.min_x)*s,(r.bounds.max_y-r.bounds.min_y)*s);}
 c.strokeStyle='#1C1F23';for(const wall of g.walls||[]){const a=p(...wall.a),b=p(...wall.b);c.lineWidth=Math.max(6,wall.thickness*s);c.beginPath();c.moveTo(a.x,a.y);c.lineTo(b.x,b.y);c.stroke();}
 c.fillStyle='#F4772D';for(const door of g.doors||[]){const a=p(...door.at);c.beginPath();c.arc(a.x,a.y,4,0,Math.PI*2);c.fill();}
 c.textAlign='center';c.textBaseline='middle';c.fillStyle='#1C1F23';
 for(const r of areas){
  const b=r.bounds,a=p((b.min_x+b.max_x)/2,(b.min_y+b.max_y)/2),label=r.name||r.id,width=Math.max(12,(b.max_x-b.min_x)*s-8);
  c.font='12px Inter,"Noto Sans SC",system-ui';const size=Math.min(12,Math.max(9,12*width/c.measureText(label).width));c.font=size+'px Inter,"Noto Sans SC",system-ui';c.fillText(label,a.x,a.y,width);
 }
}
$('#preview-lotus').onclick=async()=>{
 const button=$('#preview-lotus');button.disabled=true;$('#import-result').textContent='解析莲华古城静态示例中…';
 try{
  const source=await fetch('/api/demo-ui/skeletons/lotus_skeleton.json',{cache:'no-store',signal:AbortSignal.timeout(5000)});
  if(!source.ok)throw Error('示例文件不可用');
  const content=await source.text();JSON.parse(content);
  const r=await fetch('/api/map/upload',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({filename:'lotus_skeleton.json',content}),signal:AbortSignal.timeout(65000)});
  const data=await r.json();$('#import-data').textContent=JSON.stringify(data,null,2);
  if(!r.ok||!data.validation?.pass)throw Error(data.message||'骨架校验失败');
  $('#import-result').textContent='莲华古城 · 非官方简化版 · STATIC PREVIEW / NOT_ACTIVATED';previewGeometry(data.preview.geometry);
 }catch(e){$('#import-result').textContent='预览失败：'+e.message;$('#import-map').hidden=true;}finally{button.disabled=false;}
};
async function mapRequest(url,options={}){
 const r=await fetch(url,{cache:'no-store',signal:AbortSignal.timeout(7000),...options}),data=await r.json();
 if(!r.ok)throw Error(data.message||data.error||'HTTP '+r.status);return data;
}
function selectionChanged(){
 const id=$('#map-select').value;selectedMap=mapEntries.find(x=>x.id===id);
 $('#selected-description').textContent=selectedMap?.description||'';
 $('#confirm-scene').checked=false;updateSwitchButton();
}
function updateSwitchButton(){
 const safe=latest?.native?.observer_fresh&&['IDLE','SUCCEEDED','PARTIAL','FAILED','CANCELED','COMPLETED'].includes(latest?.session?.status)&&latest?.ros?.pose_fresh&&latest?.ros?.navigation?.status==='IDLE'&&!latest?.ros?.navigation?.motion_lease?.held;
 $('#activate-map').disabled=sceneBusy||!safe||!selectedMap||!$('#confirm-scene').checked||selectedMap.id===sceneState?.active?.map_id;
 $('#map-select').disabled=sceneBusy;
}
async function loadMapCatalog(preferred=null){
 try{
  const data=await mapRequest('/api/demo-ui/maps');mapEntries=data.entries;
  if(preferred)mapSelectionTouched=true;
  const id=preferred||$('#map-select').value||sceneState?.active?.map_id||'classic';
  $('#map-select').replaceChildren(...mapEntries.map(entry=>{const option=document.createElement('option');option.value=entry.id;option.textContent=entry.name;return option;}));
  $('#map-select').value=mapEntries.some(x=>x.id===id)?id:'classic';selectionChanged();
 }catch(e){$('#import-result').textContent='地图列表不可用：'+e.message;}
}
async function previewSelected(){
 try{
  const data=await mapRequest('/api/demo-ui/maps/'+encodeURIComponent($('#map-select').value));
  $('#import-data').textContent=JSON.stringify(data,null,2);
  if(data.geometry){previewGeometry(data.geometry);$('#import-result').textContent=data.name+' · 静态骨架预览（未切换物理场景）';}
  else{$('#import-map').hidden=true;$('#import-result').textContent='经典地图沿用现有救援拓扑；当前活动场景的真实 ROS2 地图显示在右侧。';}
 }catch(e){$('#import-result').textContent='骨架预览失败：'+e.message;}
}
async function pollScene(){
 try{
  const runtime=await mapRequest('/api/demo-ui/maps/runtime');sceneState=runtime;
  if(!mapSelectionTouched&&runtime.active?.map_id&&mapEntries.length){$('#map-select').value=runtime.active.map_id;selectionChanged();}
  const job=runtime.job;sceneBusy=['RUNNING','INTERRUPTED'].includes(job?.status);
  $('#scene-chat-block').hidden=!sceneBusy;
  $('#active-map-name').textContent=sceneBusy?'切换中':runtime.active?.map_name||'未登记场景';
  $('#scene-switch-status').textContent=job?.status==='RUNNING'?job.phase:job?.status==='INTERRUPTED'?job.phase:job?.status==='FAILED'?'切换失败：'+job.error+' · 恢复：'+job.rollback:'当前物理场景：'+(runtime.active?.map_name||'未登记')+' · seed '+(runtime.active?.seed??'—')+(job?.status==='SUCCEEDED'?' · 已真实切换':'');
  $('#scene-progress').className='progress-track'+(job?.status==='RUNNING'?' running':job?.status==='SUCCEEDED'?' terminal succeeded':'');
  updateSwitchButton();
 }catch(e){$('#scene-switch-status').textContent='场景状态连接中断；不会自动重发切换请求';}
 finally{setTimeout(pollScene,1200);}
}
$('#map-select').onchange=()=>{mapSelectionTouched=true;selectionChanged();};
$('#confirm-scene').onchange=updateSwitchButton;
$('#preview-selected').onclick=previewSelected;
$('#activate-map').onclick=async()=>{
 if($('#activate-map').disabled||!selectedMap)return;
 const id=selectedMap.id,request_id=crypto.randomUUID();
 sceneBusy=true;updateSwitchButton();$('#scene-chat-block').hidden=false;
 $('#scene-switch-status').textContent='已提交安全检查；等待真实切换状态…';
 try{await mapRequest('/api/demo-ui/maps/activate',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({id,request_id})});$('#confirm-scene').checked=false;}
 catch(e){$('#scene-switch-status').textContent='请求未确认：'+e.message+'。请等待状态刷新，不要重复点击。';}
};
$('#upload-skeleton').onclick=async()=>{
 const file=$('#skeleton-file').files[0],button=$('#upload-skeleton'),name=$('#skeleton-name').value.normalize('NFKC').trim();
 if(!name||[...name].length>40){$('#import-result').textContent='请先填写地图名称（1–40 字）。';return;}
 if(mapEntries.some(x=>x.name.toLocaleLowerCase('zh-CN')===name.toLocaleLowerCase('zh-CN'))){$('#import-result').textContent='这个地图名称已存在，请使用其他名称。';return;}
 if(!file){$('#import-result').textContent='请选择 JSON 文件。';return;}if(file.size>2*1024*1024){$('#import-result').textContent='文件不能超过 2 MiB。';return;}
 button.disabled=true;$('#import-result').textContent='上传并解析静态骨架中…';
 try{
  const content=await file.text();JSON.parse(content);
  const r=await fetch('/api/map/upload',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({filename:file.name,content}),signal:AbortSignal.timeout(65000)});
  const data=await r.json();$('#import-data').textContent=JSON.stringify(data,null,2);
  if(!r.ok||!data.validation?.pass)throw Error(data.message||data.validation?.errors?.join('\n')||'骨架校验失败');
  const entry=await mapRequest('/api/demo-ui/maps',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({name,upload_id:data.upload_id})});
  await loadMapCatalog(entry.id);previewGeometry(data.preview.geometry);
  $('#import-result').textContent=entry.name+' · 已保存 / STATIC PREVIEW / NOT_ACTIVATED。勾选确认并点击“确认切换物理场景”后才改变 Gazebo。';
 }catch(e){$('#import-result').textContent='上传/保存失败：'+e.message;$('#import-map').hidden=true;}finally{button.disabled=false;}
};
setInterval(()=>{if(lastFrameAt&&Date.now()-lastFrameAt>2500)noCamera('Camera 最新帧已过期 · 无实时画面');},300);
document.addEventListener('visibilitychange',()=>{if(!document.hidden){if(lastFrameAt&&Date.now()-lastFrameAt>2500)noCamera('等待恢复新鲜帧');}});
loadMapCatalog();pollScene();poll();camera();
