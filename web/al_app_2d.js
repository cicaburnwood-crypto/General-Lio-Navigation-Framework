import {Map2D} from './map_2d.js?v=20261028rb1';
import {createPatrolEditor} from './door_patrol.js?v=20261028rb1';

const $=id=>document.getElementById(id);
let state=null,activeTab='patrol',pointTab='patrol',busy=false,refreshing=false,mapKey='',loadingMap=false,navigating=false;
const events=[];
let recordingBusy=false;
async function record(action){
  if(recordingBusy)return;recordingBusy=true;
  try{
    const response=await fetch(`/api/recording/${action}`,{method:'POST',headers:{'X-Scan-Nav':'operator'}});
    const result=await response.json();if(!response.ok||!result.ok)throw Error(result.error||'录像操作失败');
    message(action==='start'?'正在录制原生 RGB':'正在封装录像，完成后自动传回本机');
    await refresh();
  }catch(e){message(String(e),true)}finally{recordingBusy=false}
}
$('rgb-record-start').onclick=()=>record('start');
$('rgb-record-stop').onclick=()=>record('stop');
function showRecording(r={}){
  $('rgb-record-start').disabled=recordingBusy||!r.online||['recording','finalizing'].includes(r.state);
  $('rgb-record-stop').disabled=recordingBusy||!r.online||r.state!=='recording';
  let text={idle:'未录制',offline:'相机录像模块离线',recording:'录制中',finalizing:'正在封装 MP4',complete:'录制完成，等待传回本机',error:'录像失败'}[r.state]||'等待相机';
  if(r.state==='recording'||r.state==='finalizing')text+=` · ${r.width}×${r.height} · ${(r.duration_s||0).toFixed(1)} 秒 · ${r.frames||0} 帧`;
  if(r.dropped)text+=` · 编码丢帧 ${r.dropped}`;
  if(r.error)text+=` · ${r.error}`;
  if(r.transfer==='received')text=`已传回本机：${r.local_path}`;
  $('rgb-record-status').textContent=text;
  const link=$('rgb-record-download');link.hidden=r.state!=='complete';
  if(r.state==='complete'){link.href=`/api/recordings/${encodeURIComponent(r.name)}`;link.download=r.name;link.textContent=`下载 ${r.name}`}
}
function message(text,bad=false){$('message').textContent=text;$('message').classList.toggle('error-text',bad)}
function log(text){events.unshift(`${new Date().toLocaleTimeString()} · ${text}`);events.splice(20);$('event-log').replaceChildren(...events.map(t=>{const e=document.createElement('div');e.textContent=t;return e}))}
function mode(tool,selected=0,append=false){
  map.setTool(tool);
  const names={pan:'平移 / 选择 · 不发送运动指令',pick:`${append?'连续新增路径点':`设置第 ${selected+1} 个路径点`} · 点击选位置 / 拖动设朝向 · 仅编辑草稿`,initial:'定位参考草稿 · 点击选位置 / 拖动设朝向 · 需手动开始匹配',measure:'测距 · 依次点击两个位置 · 单位 m'};
  $('map-mode').textContent=names[tool];
  for(const [id,t] of [['map-pan','pan'],['map-pick','pick'],['map-initial','initial'],['map-measure','measure']])$(id).setAttribute('aria-pressed',String(t===tool));
}
const map=new Map2D($('map-canvas'),{
  onMessage:message,
  onFollow:on=>{$('follow-view').textContent=`跟随：${on?'开':'关'}`;$('follow-view').setAttribute('aria-pressed',String(on))},
  onPick:(position,yaw)=>currentEditor().click(position,yaw),
  onSelect:index=>{tab(pointTab);currentEditor().select(index)},
  onInitial:(position,yaw)=>{
    ['x','y','z'].forEach((axis,i)=>$('seed-'+axis).value=position[i]);
    if(Number.isFinite(yaw))$('seed-yaw').value=yaw.toFixed(3);
    showSeed();mode('pan');message('参考位姿已填入草稿；点击“以参考位姿开始定位”后由雷达地图匹配求最终位姿。');
  },
});
const editorOptions={getMap:()=>state?.map,message,action,
  draw:points=>map.setPoints(points),onPickingChange:(picking,selected,append)=>{
    if(picking)mode('pick',selected,append);else if(map.tool==='pick')mode('pan');
  }};
const editor=createPatrolEditor(editorOptions);
const liveEditor=createPatrolEditor({...editorOptions,realtime:true});
const currentEditor=()=>pointTab==='realtime'?liveEditor:editor;
function tab(name){
  if(!['overview','localization','patrol','realtime'].includes(name))name='patrol';
  if(map.tool==='pick')mode('pan');
  activeTab=name;if(['patrol','realtime'].includes(name))pointTab=name;
  editor.show(name==='patrol');liveEditor.show(name==='realtime');
  document.querySelectorAll('.tab').forEach(e=>e.classList.toggle('active',e.dataset.tab===name));
  document.querySelectorAll('.tab-page').forEach(e=>e.classList.toggle('active',e.id===`tab-${name}`));
  history.replaceState(null,'',`#${name}`);
  if(name!=='localization'&&map.tool==='initial')mode('pan');
}
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>tab(b.dataset.tab));
$('map-pan').onclick=()=>{currentEditor().cancelPick();mode('pan')};
$('map-pick').onclick=()=>{tab(pointTab);currentEditor().beginPick()};
$('map-initial').onclick=()=>{tab('localization');mode('initial')};
$('map-measure').onclick=()=>{currentEditor().cancelPick();mode(map.tool==='measure'?'pan':'measure')};
$('fit-map').onclick=()=>map.fit();
$('reset-view').onclick=()=>map.focus(true);
$('follow-view').onclick=()=>map.setFollow(!map.follow);
$('zoom-in').onclick=()=>map.zoom(1.3);$('zoom-out').onclick=()=>map.zoom(1/1.3);
for(const [id,name] of [['show-map','map'],['show-live','live'],['show-global','global'],['show-local','local'],['show-grid','grid']]){
  $(id).onchange=()=>{map.layers[name]=$(id).checked;map.render()};
}
window.addEventListener('keydown',e=>{if(e.key==='Escape'){currentEditor().cancelPick();mode('pan')}});
function showSeed(){
  const values=['x','y','z','yaw'].map(axis=>Number($('seed-'+axis).value));
  if(values.every(Number.isFinite))map.seed={position:values.slice(0,3),yaw:values[3]};map.render();
}
for(const axis of ['x','y','z','yaw'])$('seed-'+axis).oninput=showSeed;
$('seed-current').onclick=()=>{
  if(!map.validPose()){message('尚未收到机器人位姿。',true);return}
  ['x','y','z','yaw'].forEach((axis,i)=>$('seed-'+axis).value=state.pose[i].toFixed(3));showSeed();
};
$('relocalize').onclick=()=>action('relocalize',Object.fromEntries(['x','y','z','yaw'].map(k=>[k,Number($('seed-'+k).value)])));
$('stop').onclick=()=>action('cancel_patrol');

async function action(name,more={}){
  const interrupt=name==='cancel_patrol';if(busy&&!interrupt)return null;if(!interrupt)busy=true;
  message(`${name} 执行中…`);
  try{
    const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Scan-Nav':'operator'},body:JSON.stringify({action:name,...more})});
    const result=await response.json();if(!response.ok||!result.ok)throw Error(result.error||`HTTP ${response.status}`);
    const text=result.result?.message||`${name} 完成`;
    message(text);log(text);await refresh();return result;
  }catch(e){message(String(e),true);log(String(e));return null}finally{if(!interrupt)busy=false}
}
function check(label,ok){const e=document.createElement('div');e.className=`check ${ok?'ok':''}`;e.textContent=`${ok?'✓':'×'} ${label}`;return e}
function update(d){
  showRecording(d.recording);
  for(const item of [editor,liveEditor]){item.setMap(d.map);item.updateRuntime(d)}map.update(d);
  $('map-badge').textContent=`地图 ${d.map}`;
  $('sensor-badge').textContent=d.lidar_age_s!=null&&d.lidar_age_s<1?'雷达在线':'等待雷达';
  $('state-pill').textContent=d.preview?'离线预览':d.patrol_status?.state||'等待 manager';
  $('pose').textContent=map.validPose()?`X ${d.pose[0].toFixed(3)} m · Y ${d.pose[1].toFixed(3)} m · Yaw ${(d.pose[3]*180/Math.PI).toFixed(1)}°`:'等待 map → base_link';
  $('ages').textContent=`RGB ${d.rgb_age_s??'—'} s · 定位 ${d.pose_age_s??'—'} s`;
  $('checks').replaceChildren(check('Nav2 Action',d.nav_ready),check('Patrol manager',!!d.patrol_status),check('RGB 门检测服务',d.door_ready),check('2D 地图',!!map.grid));
  $('next-step').textContent='蓝点到达即继续，红点停稳并检测门后才继续。巡检任务可保存 YAML；实时模式选点后确认执行，不保存文件。两种模式均由同一个 manager 顺序发送目标。';
  const caps=d.navigation_caps||{};
  $('caps-summary').textContent=`AL / BL 同款 MPPI：vx ${caps.max_vx_mps??'—'} m/s · vy ${caps.max_vy_mps??'—'} m/s · wz ${caps.max_wz_radps??'—'} rad/s。下游底盘限制保持原样。`;
  $('localization-state').textContent=map.validPose()?`${$('pose').textContent} · Z ${d.pose[2].toFixed(3)} m · 数据年龄 ${d.pose_age_s??'—'} s`:'等待 map → base_link';
  const loc=d.localization||{},align=loc.alignment;
  const labels={CHECKING:'正在读取原定位模块状态',RESTARTING:'正在重启 FAST-LIO，清除上一会话状态',WAITING_LIO:'等待 FAST-LIO 新里程计与点云',SUBMITTING:'正在提交参考位姿',MATCHING:'等待粗配准 / 精配准成功；参考点不是最终位置',LOCALIZED:'原定位模块已建立定位',ERROR:'定位请求失败',UNKNOWN:'定位请求结果未知，请检查原服务'};
  $('localization-progress').textContent=(labels[loc.state]||'等待定位状态')+
    (loc.native_valid===true&&!loc.pose_fresh?'；当前无新鲜定位结果，不显示旧位姿':'')+
    (loc.error?` · ${loc.error}`:'')+(loc.check_error?` · ${loc.check_error}`:'');
  const decisions={accepted:'本次匹配已采用',jump_override:'原模块已采用重定位修正',score_override:'原模块已采用匹配修正',not_converged_rough:'粗配准未收敛',not_converged_refine:'精配准未收敛',reject_rough_score:'粗配准误差未通过',reject_score_ceiling:'精配准误差未通过',reject_score_outlier:'精配准误差异常',reject_jump:'本次修正未通过原模块跳变检查'};
  $('localization-alignment').textContent=align?`最近配准：${decisions[align.decision]||align.decision} · 粗 RMS ${align.rough_rms>=0?align.rough_rms.toFixed(3)+' m':'—'} · 精 RMS ${align.refine_rms>=0?align.refine_rms.toFixed(3)+' m':'—'} · ${d.alignment_age_s??'—'} s 前`:'等待配准日志；请求已接受不等于定位成功。';
  const active=d.armed&&d.control==='manager';if(active&&!navigating)map.setFollow(true);navigating=active;
}
async function loadMap(d){
  const key=`${d.map}:${d.map_mtime_ns}`;if(key===mapKey||loadingMap)return;
  loadingMap=true;
  try{
    const response=await fetch(`/api/maps/${encodeURIComponent(d.map)}/grid`,{cache:'no-store'});if(!response.ok)throw Error(`地图 HTTP ${response.status}`);
    const grid=await response.json();await map.load(grid);mapKey=key;
    $('map-title').textContent=`2D 地图 · ${grid.resolution} m / 格 · map`;
  }catch(e){message(`2D 地图加载失败：${e}`,true)}finally{loadingMap=false}
}
async function refresh(){
  if(refreshing)return;refreshing=true;const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),5000);
  try{
    const response=await fetch('/api/status',{cache:'no-store',signal:controller.signal});if(!response.ok)throw Error(`HTTP ${response.status}`);
    const d=await response.json(),first=!state;state=d;update(d);loadMap(d);
    if(first)message('AL 在线。地图取点仅编辑任务；当前未由本页面启动运动。');
  }catch(e){$('state-pill').textContent='连接中断';message(`状态读取失败：${e}`,true)}finally{clearTimeout(timeout);refreshing=false}
}
async function poll(){await refresh();setTimeout(poll,250)}
tab(location.hash.slice(1)||'patrol');poll();
