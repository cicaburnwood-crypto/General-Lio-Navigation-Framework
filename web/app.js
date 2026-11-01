import * as THREE from 'three';
import {OrbitControls} from '/static/vendor/OrbitControls.js';
import {guideSteps} from '/static/tutorial_steps.js';
import {createPatrolEditor} from '/static/door_patrol.js?v=1';

const $ = id => document.getElementById(id);
const viewer = $('viewer');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b1620);
const camera = new THREE.PerspectiveCamera(55, 1, .01, 300);
camera.position.set(4, 5, 4);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
viewer.appendChild(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.target.set(0, 0, 0);
const grid = new THREE.GridHelper(30, 60, 0x486371, 0x273f4c);
scene.add(grid, new THREE.AxesHelper(.55));
const liveMaterial = new THREE.PointsMaterial({size:.035,vertexColors:true,sizeAttenuation:true});
const mapMaterial = new THREE.PointsMaterial({size:.026,color:0x728994,sizeAttenuation:true});
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
const ground = new THREE.Plane(new THREE.Vector3(0,1,0),0);
const robotArrow = new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),.35,0x61e3c1,.12,.08);
robotArrow.visible = false;
scene.add(robotArrow);

let state = null, liveCloud = null, mapCloud = null, globalLine = null, localLine = null;
let lastCloudKey = '', lastMapKey = '', lastGlobalKey = '', lastLocalKey = '';
let waypoints = [], markers = [], groundZ = 0, clickMode = false, loadedWaypointMap = null;
let loadedWaypointRevision = null, waypointsDirty = false, recordingWaypoint = false;
let settings = null, yamlDrafts = {}, yamlSelected = 'mapping';
let busy = false, activeTab = 'overview', lastError = '', lastDrive = [0,0,0];
let recordBusy = false, currentCapture = null;
let firstStatusReceived = false;
let followRobot = false, navigationWasActive = false, followNeedsFocus = false;
let lastHighlightedWaypointIndex = null;
let guideVisible = false, guideIndex = 0, guideNavigationSeen = false;
let lastPoseReady = false;
const guideConfirmed = new Set();
let events = [];

function logEvent(message, bad=false) {
  events.unshift({when:new Date().toLocaleTimeString(),message,bad});
  events = events.slice(0,20);
  $('event-log').replaceChildren(...events.map(e => {
    const div=document.createElement('div');
    div.textContent=`${e.when} · ${e.message}`;
    if(e.bad) div.className='error-text';
    return div;
  }));
}
function message(text,bad=false) {
  $('message').textContent=text;
  $('message').classList.toggle('error-text',bad);
}
const patrolGroup = new THREE.Group();
scene.add(patrolGroup);
const patrolEditor = createPatrolEditor({getMap:()=>state?.map,getGroundZ:()=>groundZ,message,action,
  getSeed:()=>Object.fromEntries(['x','y','z','yaw'].map(k=>[k,Number($('seed-'+k).value)])),draw:points=>{
  for(const child of [...patrolGroup.children]){
    patrolGroup.remove(child);child.geometry?.dispose();child.material?.map?.dispose();child.material?.dispose();
  }
  for(const p of points){
    if(!p.position.every(Number.isFinite))continue;
    const color=p.isHome?0xc690ff:0x52dbbb;
    const marker=new THREE.Mesh(new THREE.SphereGeometry(.09,12,8),new THREE.MeshBasicMaterial({color,depthTest:false}));
    marker.position.set(p.position[0],p.position[2]+.05,p.position[1]);marker.renderOrder=10;patrolGroup.add(marker);
    const canvas=document.createElement('canvas');canvas.width=512;canvas.height=64;
    const ctx=canvas.getContext('2d');ctx.fillStyle='#102430ee';ctx.fillRect(0,0,512,64);ctx.font='28px sans-serif';ctx.fillStyle=p.isHome?'#c690ff':'#52dbbb';ctx.fillText(p.label,10,43,492);
    const label=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(canvas),depthTest:false}));
    label.position.copy(marker.position);label.position.y+=.28;label.scale.set(1.5,.1875,1);label.renderOrder=11;patrolGroup.add(label);
  }
}});
function tab(name) {
  if(name!=='mapping')releaseMotion();
  activeTab=name;
  patrolEditor.show(name==='patrol');
  document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('active',b.dataset.tab===name));
  document.querySelectorAll('.tab-page').forEach(p=>p.classList.toggle('active',p.id===`tab-${name}`));
  if(name==='settings'&&!settings) loadSettings();
  syncRgbViews();
  if(name==='navigation')updateWaypointTracking(true);
}
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>tab(b.dataset.tab));
function syncRgbViews(){
  const mapping=activeTab==='mapping';
  $('mapping-rgb').hidden=!mapping;
  document.querySelector('.viewer-card').classList.toggle('mapping-view',mapping);
  for(const [id,visible] of [['mapping-rgb-image',mapping],['navigation-rgb-image',activeTab==='navigation']]){
    const image=$(id);
    if(visible&&!image.getAttribute('src'))image.src=`/api/rgb.mjpeg?t=${Date.now()}`;
    if(!visible)image.removeAttribute('src');
  }
  requestAnimationFrame(resize);
}
for(const [imageId,tabName,statusId] of [
  ['mapping-rgb-image','mapping','rgb-state'],
  ['navigation-rgb-image','navigation','nav-rgb-state']]){
  $(imageId).onerror=()=>{
    $(statusId).textContent='RGB 连接中断，正在重连';
    $(imageId).removeAttribute('src');
    setTimeout(()=>{if(activeTab===tabName)syncRgbViews()},1500);
  };
}

function replaceCloud(old,points,material,colored=false) {
  if(old){scene.remove(old);old.geometry.dispose()}
  if(!points?.length) return null;
  const xyz=new Float32Array(points.length*3);
  const colors=colored?new Float32Array(points.length*3):null;
  for(let i=0;i<points.length;i++){
    const [x,y,z]=points[i];
    xyz.set([x,z,y],i*3);
    if(colors){const h=Math.max(0,Math.min(1,(z+.5)/2));colors.set([.18+.7*h,.77-.19*h,.93-.52*h],i*3)}
  }
  const geometry=new THREE.BufferGeometry();
  geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));
  if(colors) geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));
  const cloud=new THREE.Points(geometry,material);scene.add(cloud);return cloud;
}
function replaceLine(old,points,color) {
  if(old){scene.remove(old);old.geometry.dispose();old.material.dispose()}
  if(!points?.length)return null;
  const xyz=[];for(const p of points)xyz.push(p[0],p[2]+.04,p[1]);
  const geometry=new THREE.BufferGeometry();
  geometry.setAttribute('position',new THREE.Float32BufferAttribute(xyz,3));
  const line=new THREE.Line(geometry,new THREE.LineBasicMaterial({color}));scene.add(line);return line;
}
function pointMarkers() {
  markers.forEach(m=>{scene.remove(m);m.geometry.dispose();m.material.dispose()});markers=[];
  waypoints.forEach((p,i)=>{
    const marker=new THREE.Mesh(new THREE.SphereGeometry(.085,12,8),
      new THREE.MeshBasicMaterial({color:i===0?0xffcf72:0xffa66a}));
    marker.position.set(p[0],p[2]+.10,p[1]);scene.add(marker);markers.push(marker);
  });
}
function renderWaypoints() {
  const list=$('waypoints');list.replaceChildren();
  waypoints.forEach((p,i)=>{
    const row=document.createElement('div');row.className='wp';row.draggable=true;row.dataset.index=i;
    const handle=document.createElement('span');handle.className='handle';handle.textContent=`${i+1} ⋮`;
    handle.title='拖拽调整顺序';row.appendChild(handle);
    ['X','Y','Z'].forEach((axis,j)=>{
      const input=document.createElement('input');input.type='number';input.step='.01';input.title=axis;input.value=p[j];
      input.onchange=()=>{const n=Number(input.value);if(Number.isFinite(n)){p[j]=n;waypointsDirty=true;pointMarkers();renderMappingWaypoints()}else input.value=p[j]};
      row.appendChild(input);
    });
    const remove=document.createElement('button');remove.type='button';remove.title='删除';remove.textContent='×';
    remove.onclick=()=>{waypoints.splice(i,1);waypointsDirty=true;renderWaypoints()};row.appendChild(remove);
    row.ondragstart=e=>{e.dataTransfer.setData('text/plain',String(i));row.classList.add('dragging')};
    row.ondragend=()=>row.classList.remove('dragging');
    row.ondragover=e=>{e.preventDefault();row.classList.add('drag-over')};
    row.ondragleave=()=>row.classList.remove('drag-over');
    row.ondrop=e=>{e.preventDefault();row.classList.remove('drag-over');const from=Number(e.dataTransfer.getData('text/plain'));
      if(Number.isInteger(from)&&from>=0&&from<waypoints.length&&from!==i){waypoints.splice(i,0,waypoints.splice(from,1)[0]);waypointsDirty=true;renderWaypoints()}};
    list.appendChild(row);
  });
  pointMarkers();
  renderMappingWaypoints();
  updateWaypointTracking(true);
}
function trackedWaypointRowIndex(d){
  if(!d?.armed||d.control!=='navigation'||!Number.isInteger(d.active_waypoint_index)||
      !Array.isArray(d.active_waypoint)||d.active_waypoint.length!==3)return null;
  const matches=p=>Array.isArray(p)&&p.length===3&&p.every((v,j)=>Math.abs(v-d.active_waypoint[j])<.005);
  const index=d.active_waypoint_index;
  if(matches(waypoints[index]))return index;
  const alternatives=waypoints.map((p,i)=>matches(p)?i:-1).filter(i=>i>=0);
  return alternatives.length===1?alternatives[0]:null;
}
function updateWaypointTracking(forceScroll=false){
  const list=$('waypoints');
  const index=trackedWaypointRowIndex(state);
  for(const row of list.children){
    const current=Number(row.dataset.index)===index;
    row.classList.toggle('current-waypoint',current);
    if(current)row.setAttribute('aria-current','step');
    else row.removeAttribute('aria-current');
  }
  if(index!==null&&activeTab==='navigation'&&(forceScroll||index!==lastHighlightedWaypointIndex)){
    const row=list.children[index];
    if(row){
      const bounds=list.getBoundingClientRect(),target=row.getBoundingClientRect();
      if(target.top<bounds.top||target.bottom>bounds.bottom){
        const top=list.scrollTop+target.top-bounds.top-(list.clientHeight-row.clientHeight)/2;
        list.scrollTo({top:Math.max(0,top),behavior:'smooth'});
      }
    }
  }
  lastHighlightedWaypointIndex=index;
}
function renderMappingWaypoints(){
  $('mapping-waypoint-state').textContent=waypoints.length?
    `当前地图 ${waypoints.length} 个路径点${waypointsDirty?' · 有未保存的编辑':' · 已保存，可在导航页使用'}`:
    '当前地图尚无已记录的路径点。';
  const rows=waypoints.map((p,i)=>{const row=document.createElement('div');
    row.textContent=`${i+1}. X ${p[0].toFixed(3)} · Y ${p[1].toFixed(3)} · Z ${p[2].toFixed(3)} m`;
    return row});
  $('mapping-waypoint-list').replaceChildren(...rows);
}
function download(name,content,type='application/json') {
  const url=URL.createObjectURL(new Blob([content],{type}));
  const link=document.createElement('a');link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),5000);
}

function resize(){const w=Math.max(1,viewer.clientWidth),h=Math.max(1,viewer.clientHeight);renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix()}
new ResizeObserver(resize).observe(viewer);
const viewAxes={x:new THREE.Vector3(1,0,0),y:new THREE.Vector3(0,0,1),z:new THREE.Vector3(0,1,0)};
const gizmoParts=Object.fromEntries(['x','y','z'].map(axis=>{
  const group=document.querySelector(`[data-view-axis="${axis}"]`);
  return [axis,{group,line:group.querySelector('line'),dot:group.querySelector('circle'),label:group.querySelector('text')}];
}));
let lastViewStep='';
function viewTranslationStep(){return Math.max(.1,Math.min(2,camera.position.distanceTo(orbit.target)*.12))}
function updateViewGizmo(){
  const inverse=camera.quaternion.clone().invert();
  for(const [axis,part] of Object.entries(gizmoParts)){
    const vector=viewAxes[axis].clone().applyQuaternion(inverse);
    const x=48+vector.x*32,y=48-vector.y*32;
    part.line.setAttribute('x2',x.toFixed(1));part.line.setAttribute('y2',y.toFixed(1));
    part.dot.setAttribute('cx',x.toFixed(1));part.dot.setAttribute('cy',y.toFixed(1));
    part.label.setAttribute('x',x.toFixed(1));part.label.setAttribute('y',y.toFixed(1));
    part.group.style.opacity=vector.z<-.1?'.55':'1';
  }
  const label=`拖动 XYZ 连续调整 · 步进 ±${viewTranslationStep().toFixed(1)} m / 15°`;
  if(label!==lastViewStep){$('view-step').textContent=label;lastViewStep=label}
  const viewTarget=orbit.target.toArray().map(value=>value.toFixed(3)).join(',');
  if(viewer.dataset.viewTarget!==viewTarget)viewer.dataset.viewTarget=viewTarget;
}
function animate(){requestAnimationFrame(animate);orbit.update();updateViewGizmo();renderer.render(scene,camera)}animate();
function freshRobotPose(d){
  return d?.pose_frame==='world'&&d.pose_age_s!==null&&d.pose_age_s<2&&
    Array.isArray(d.pose)&&d.pose.length>=3&&d.pose.slice(0,3).every(Number.isFinite);
}
function focusRobot(d,resetOrientation=false){
  if(!freshRobotPose(d))return false;
  const center=new THREE.Vector3(d.pose[0],d.pose[2],d.pose[1]);
  if(resetOrientation){
    camera.up.set(0,1,0);
    camera.position.copy(center).add(new THREE.Vector3(4,5,4));
  }else{
    camera.position.add(center.clone().sub(orbit.target));
  }
  orbit.target.copy(center);
  orbit.update();
  return true;
}
function showFollowState(){
  $('follow-view').textContent=`跟随机器人：${followRobot?'开':'关'}`;
  $('follow-view').setAttribute('aria-pressed',String(followRobot));
}
function updateRobotFollow(d){
  const navigating=d.armed&&['navigation','manager'].includes(d.control);
  if(navigating&&!navigationWasActive){
    followRobot=true;
    followNeedsFocus=true;
    showFollowState();
  }else if(!navigating&&navigationWasActive){
    followRobot=false;
    followNeedsFocus=false;
    showFollowState();
  }
  navigationWasActive=navigating;
  if(followRobot&&focusRobot(d,followNeedsFocus))followNeedsFocus=false;
}
function resetView(){
  const pose=state?.pose;
  const valid=Array.isArray(pose)&&pose.length>=3&&pose.slice(0,3).every(Number.isFinite);
  const center=valid?new THREE.Vector3(pose[0],pose[2],pose[1]):new THREE.Vector3();
  camera.up.set(0,1,0);orbit.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(4,5,4));orbit.update();
  if(navigationWasActive&&freshRobotPose(state)){
    followRobot=true;
    followNeedsFocus=false;
    showFollowState();
  }
  message(!valid?'暂无机器人位姿：已暂时以地图原点为中心；定位后再复位即可。':
    state.pose_age_s!==null&&state.pose_age_s<2?'已以实时机器人位姿为中心复位视角。':
    '已以最后已知机器人位姿为中心；当前定位尚未更新。');
}
$('reset-view').onclick=resetView;
$('follow-view').onclick=()=>{
  if(followRobot){
    followRobot=false;
    followNeedsFocus=false;
    showFollowState();
    message('已暂停点云视角跟随，可自由查看地图。');
  }else if(freshRobotPose(state)){
    followRobot=true;
    followNeedsFocus=false;
    focusRobot(state,true);
    showFollowState();
    message('已以机器人为中心并开始跟随。');
  }else{
    message('等待新鲜的 world 位姿后才能跟随机器人。',true);
  }
};
function moveViewAlong(axis,distance){
  const delta=viewAxes[axis].clone().multiplyScalar(distance);
  camera.position.add(delta);orbit.target.add(delta);orbit.update();
}
function turnView(axis,angle){
  const forward=orbit.target.clone().sub(camera.position).normalize();
  if(axis==='roll')camera.up.applyAxisAngle(forward,angle);
  else{
    const rotationAxis=axis==='yaw'?viewAxes.z.clone():forward.clone().cross(camera.up).normalize();
    if(rotationAxis.lengthSq()<.5)return;
    camera.position.sub(orbit.target).applyAxisAngle(rotationAxis,angle).add(orbit.target);
    camera.up.applyAxisAngle(rotationAxis,angle);
  }
  orbit.update();
}
for(const [axis,part] of Object.entries(gizmoParts)){
  const activate=()=>{
    const distance=Math.max(1,camera.position.distanceTo(orbit.target));
    camera.up.copy(axis==='z'?new THREE.Vector3(0,0,-1):viewAxes.z);
    camera.position.copy(orbit.target).addScaledVector(viewAxes[axis],distance);
    orbit.update();
  };
  let drag=null,suppressClickUntil=0;
  part.group.addEventListener('pointerdown',event=>{
    if(event.button!==0)return;
    drag={startX:event.clientX,startY:event.clientY,lastX:event.clientX,lastY:event.clientY,moved:false};
    part.group.setPointerCapture(event.pointerId);
    event.preventDefault();
  });
  part.group.addEventListener('pointermove',event=>{
    if(!drag)return;
    const dx=event.clientX-drag.lastX,dy=event.clientY-drag.lastY;
    if(Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)>3)drag.moved=true;
    if(drag.moved){turnView('yaw',-dx*.007);turnView('pitch',-dy*.007)}
    drag.lastX=event.clientX;drag.lastY=event.clientY;
  });
  part.group.addEventListener('pointerup',event=>{
    if(drag?.moved)suppressClickUntil=performance.now()+350;
    drag=null;
    if(part.group.hasPointerCapture(event.pointerId))part.group.releasePointerCapture(event.pointerId);
  });
  part.group.addEventListener('pointercancel',()=>{drag=null});
  part.group.addEventListener('click',event=>{
    if(performance.now()<suppressClickUntil){event.preventDefault();return}
    activate();
  });
  part.group.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();activate()}});
}
document.querySelectorAll('[data-view-shift]').forEach(button=>button.onclick=()=>{
  const [axis,direction]=button.dataset.viewShift.split(':');
  moveViewAlong(axis,Number(direction)*viewTranslationStep());
});
document.querySelectorAll('[data-view-drag-axis]').forEach(label=>{
  let drag=null;
  label.addEventListener('pointerdown',event=>{
    if(event.button!==0)return;
    drag={x:event.clientX,y:event.clientY};label.setPointerCapture(event.pointerId);event.preventDefault();
  });
  label.addEventListener('pointermove',event=>{
    if(!drag)return;
    const axis=label.dataset.viewDragAxis;
    const pixels=axis==='z'?drag.y-event.clientY:event.clientX-drag.x;
    moveViewAlong(axis,pixels*viewTranslationStep()/80);
    drag.x=event.clientX;drag.y=event.clientY;
  });
  label.addEventListener('pointerup',event=>{
    drag=null;if(label.hasPointerCapture(event.pointerId))label.releasePointerCapture(event.pointerId);
  });
  label.addEventListener('pointercancel',()=>{drag=null});
});
document.querySelectorAll('[data-view-turn]').forEach(button=>button.onclick=()=>{
  const [axis,direction]=button.dataset.viewTurn.split(':');
  turnView(axis,Number(direction)*Math.PI/12);
});
let down=null;
renderer.domElement.addEventListener('pointerdown',e=>down={x:e.clientX,y:e.clientY});
renderer.domElement.addEventListener('pointerup',e=>{
  const patrolPick=patrolEditor.picking;
  if((!patrolPick&&!(clickMode&&activeTab==='navigation'))||e.button!==0||!down||!state?.map||Math.hypot(e.clientX-down.x,e.clientY-down.y)>6)return;
  const rect=renderer.domElement.getBoundingClientRect();
  mouse.set((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1);
  raycaster.setFromCamera(mouse,camera);
  const hit=new THREE.Vector3();
  if(raycaster.ray.intersectPlane(ground,hit)){
    if(patrolPick){patrolEditor.click([Number(hit.x.toFixed(3)),Number(hit.z.toFixed(3)),groundZ]);return}
    waypoints.push([Number(hit.x.toFixed(2)),Number(hit.z.toFixed(2)),groundZ]);
    waypointsDirty=true;
    renderWaypoints();message(`已添加第 ${waypoints.length} 个地图路径点`);
  }
});

async function action(name,more={}) {
  const interrupt=['stop','estop','cancel_patrol'].includes(name);
  if(busy&&!interrupt)return null;
  if(!interrupt)busy=true;
  const working=(name==='preview_route'||name==='start_nav'||name==='apply_route')?'等待原生 PCT 输出可通行路径…':
                name==='save_map'?'保存地图和瓦片中…':`${name} 执行中…`;
  message(working);
  try {
    const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Scan-Nav':'operator'},
      body:JSON.stringify({action:name,...more})});
    const result=await response.json();
    if(!response.ok||!result.ok)throw Error(result.error||`HTTP ${response.status}`);
    // Adopt the acknowledged state before the next poll so existing heartbeats
    // continue immediately after navigation startup.
    if(result.state)state=result.state;
    if(!state?.nav2_door&&['start_nav','apply_route','start_patrol'].includes(name)) await fetch('/api/heartbeat',{method:'POST',headers:{'X-Scan-Nav':'operator'}});
    if(['save_waypoints','preview_route','start_nav','apply_route'].includes(name)){
      waypointsDirty=false;loadedWaypointRevision=result.state?.waypoint_mtime_ns??null;
    }
    const resultMessage=name==='relocalize'?(state?.nav2_door?'已发布 /initialpose；请观察实时定位。':'重定位已启动，等待本次新 pose.txt。'):(result.result?.message||`${name} 完成`);
    message(resultMessage);logEvent(resultMessage);
    await refresh();return result;
  } catch(error) {
    message(String(error),true);logEvent(String(error),true);return null;
  } finally {if(!interrupt)busy=false}
}

async function recordingPost(path, body={}) {
  const response=await fetch(`/api/recording/${path}`,{
    method:'POST',headers:{'Content-Type':'application/json','X-Scan-Nav':'operator'},
    body:JSON.stringify(body)});
  const result=await response.json();
  if(!response.ok||!result.ok)throw Error(result.error||`录制请求 HTTP ${response.status}`);
  return result;
}

function updateRecordingUI(d){
  const serverActive=!!d?.recording?.active;
  const active=serverActive||!!currentCapture;
  const button=$('record-toggle'),label=$('record-state');
  button.disabled=recordBusy;
  button.classList.toggle('recording',active);
  button.textContent=active?'■ 停止录制':'● 开始录制';
  label.classList.toggle('recording',active);
  label.textContent=serverActive?`录制中 ${Math.floor(d.recording.elapsed_s||0)} s`:
    currentCapture?'正在启动…':'未录制';
  if(serverActive)button.title=`再次点击停止；文件保存至 ${d.recording.directory}`;
  else button.title='只录制 MID-360S 点云与 IMU 的 ROS bag；再次点击停止';
}

async function startUiRecording(){
  const started=await recordingPost('start');
  const capture={id:started.recording.id,stopping:false,heartbeat:null,stopPromise:null};
  currentCapture=capture;
  capture.heartbeat=setInterval(async()=>{
    if(capture.stopping)return;
    try{await recordingPost('heartbeat',{id:capture.id})}
    catch(error){message(`录制连接中断：${error}`,true);stopUiRecording().catch(e=>message(String(e),true))}
  },3000);
  message('录制中：MID-360S 原始点云与 IMU 正写入 ROS bag。');
  logEvent(`开始 ROS bag 录制 ${capture.id}`);
}

async function stopUiRecording(){
  const capture=currentCapture;
  if(!capture){
    const id=state?.recording?.id;
    if(!id)return;
    const result=await recordingPost('stop',{id});
    message(`录制已停止：${result.result?.directory||id}`);
    logEvent(`停止录制 ${id}`);
    await refresh();
    return;
  }
  if(capture.stopPromise)return capture.stopPromise;
  capture.stopping=true;
  clearInterval(capture.heartbeat);
  capture.stopPromise=(async()=>{
    try{
      const result=await recordingPost('stop',{id:capture.id});
      if(result.result?.bag_returncode!==0)throw Error(`ROS bag 异常退出：${result.result?.directory}`);
      message(`ROS bag 录制完成：${result.result.directory}`);
      logEvent(`录制完成 ${capture.id}`);
    }finally{
      if(currentCapture===capture)currentCapture=null;
      await refresh();
    }
  })();
  return capture.stopPromise;
}

$('record-toggle').onclick=async()=>{
  if(recordBusy)return;
  recordBusy=true;
  try{
    if(currentCapture||state?.recording?.active)await stopUiRecording();
    else await startUiRecording();
  }catch(error){message(`录制失败：${error}`,true);logEvent(`录制失败：${error}`,true)}
  finally{recordBusy=false;updateRecordingUI(state)}
};
async function loadMapPoints(name,key) {
  try{
    const response=await fetch(`/api/maps/${encodeURIComponent(name)}/points`);
    if(!response.ok)throw Error(`地图 HTTP ${response.status}`);
    const result=await response.json();if(lastMapKey!==key)return;
    groundZ=result.ground_z;ground.constant=-groundZ;grid.position.y=groundZ;
    mapCloud=replaceCloud(mapCloud,result.points,mapMaterial);
    $('cloud-label').textContent=`已保存地图 · ${result.points.length} 点`;
  }catch(error){message(`地图显示失败：${error}`,true)}
}
async function loadWaypoints(name) {
  try {const response=await fetch(`/api/maps/${encodeURIComponent(name)}/waypoints`);
    if(!response.ok)throw Error(`HTTP ${response.status}`);
    const result=await response.json();if(state?.map!==name||waypointsDirty)return;
    waypoints=(result.waypoints||[]).filter(p=>Array.isArray(p)&&p.length===3).map(p=>p.map(Number));
    loadedWaypointRevision=result.mtime_ns??null;
    renderWaypoints();
  }catch(error){message(`路径点读取失败：${error}`,true)}
}
function check(label,ok,warning=false){const box=document.createElement('div');box.className=`check ${ok?'ok':warning?'warn':'bad'}`;box.textContent=`${ok?'✓':warning?'!':'×'} ${label}`;return box}
function updateStatusUI(d) {
  if(d.nav2_door){
    patrolEditor.setMap(d.map);patrolEditor.updateRuntime(d);
    $('map-badge').textContent=`地图 ${d.map}`;
    $('sensor-badge').textContent=d.lidar_age_s!==null&&d.lidar_age_s<1?'雷达在线':'等待雷达';
    $('state-pill').textContent=d.preview?'离线配置预览（不连接机器人）':(d.patrol_status?.state||'等待 manager');
    $('pose').textContent=d.pose?`X ${d.pose[0].toFixed(2)} · Y ${d.pose[1].toFixed(2)} · Yaw ${(d.pose[3]*180/Math.PI).toFixed(1)}°`:'等待 map → base_link';
    $('ages').textContent=`RGB ${d.rgb_age_s??'--'} s · 定位 ${d.pose_age_s??'--'} s`;
    $('checks').replaceChildren(check('Nav2 Action',d.nav_ready),check('Patrol manager',!!d.patrol_status),check('RGB 门检测服务',d.door_ready),check('地图',d.map_saved));
    $('next-step').textContent='巡检任务页配置门前 XYZ / 正对门 yaw 和过门目标；只有 manager 发布路径点。';
    $('caps-summary').textContent='副本 Nav2 配置：vx 0.4 m/s、vy 0.2 m/s、wz 0.3 rad/s；未连接旧 scan-nav 控制桥。';
    $('pose-file-status').textContent='使用 Nav2/VGICP 的 /initialpose；不使用 SCAN pose.txt';
    $('pose-file-detail').textContent='坐标系 map，Yaw 单位 rad；初值不是定位结果。';
    $('localization-state').textContent=d.pose?`实时定位 ${d.pose_age_s??'--'} s`:'等待外部启动的定位链路';
    robotArrow.visible=!!d.pose&&d.pose_age_s<2;
    if(robotArrow.visible){robotArrow.position.set(d.pose[0],d.pose[2],d.pose[1]);robotArrow.setDirection(new THREE.Vector3(Math.cos(d.pose[3]),0,Math.sin(d.pose[3])))}
    return;
  }
  patrolEditor.setMap(d.map);
  patrolEditor.updateRuntime(d);
  updateRecordingUI(d);
  $('map-badge').textContent=d.map?`地图 ${d.map}`:'未选择地图';
  const sensorOkay=d.lidar_age_s!==null&&d.lidar_age_s<1&&d.imu_age_s!==null&&d.imu_age_s<1;
  const sensorStatus={restarting:'雷达自动重启中',waiting_for_idle:'传感器断流 · 等待空闲恢复',
    cooldown:'传感器断流 · 等待重试',driver_stopped:'雷达服务已停止',restart_failed:'雷达自动恢复失败'};
  $('sensor-badge').textContent=sensorOkay?'MID-360S 在线':
    sensorStatus[d.sensor_recovery]||'雷达 / IMU 连接中';
  $('state-pill').textContent=`${d.mode} · ${d.armed?'运动已启用':'运动未启用'}`;
  $('state-pill').className=d.armed?'running':sensorOkay?'':'alert';
  $('pose').textContent=d.pose?`X ${d.pose[0].toFixed(2)} m · Y ${d.pose[1].toFixed(2)} m · Z ${d.pose[2].toFixed(2)} m · Yaw ${(d.pose[3]*180/Math.PI).toFixed(1)}°`:'位姿 --';
  $('ages').textContent=`雷达 ${d.lidar_age_s??'--'} s · IMU ${d.imu_age_s??'--'} s · 定位 ${d.pose_age_s??'--'} s`;
  if(d.error&&d.error!==lastError){message(d.error,true);logEvent(d.error,true)}lastError=d.error;
  const workerOnline=d.terrain_worker_age_s!==null&&d.terrain_worker_age_s<8;
  const located=d.mode==='localization'&&d.pose_age_s!==null&&d.pose_age_s<1;
  const hasRoute=d.global_path?.length>=2;
  const checks=$('checks');checks.replaceChildren(
    check('雷达与 IMU',sensorOkay),check('PCD 地图已保存',d.map_saved),
    check('5070 工作进程',workerOnline,!workerOnline),check('Tomogram 最新',d.tomogram_current),
    check('实时定位',located),check('原生路径',hasRoute,!hasRoute));
  const stages={map:d.map_saved,terrain:d.tomogram_current,loc:located,nav:hasRoute};
  for(const stage of document.querySelectorAll('[data-stage]')){
    const name=stage.dataset.stage;stage.classList.toggle('done',!!stages[name]);
    stage.classList.toggle('current',!stages[name]&&(name==='map'||(name==='terrain'&&stages.map)||(name==='loc'&&stages.terrain)||(name==='nav'&&stages.loc)));
  }
  let next='先创建或选择地图并开始建图。';
  if(d.map_saved&&!d.tomogram_current)next='PCD 已保存。停止建图后，在“地形图”用本机 5070 生成 Tomogram。';
  else if(d.tomogram_current&&!located)next='地形图就绪。设置初始位姿，依次执行重定位与实时定位。';
  else if(located&&!hasRoute)next='定位在线。添加路径点并预览原生 PCT 路线。';
  else if(hasRoute)next='路线已显示。核对环境后可启动或更新导航。';
  $('next-step').textContent=next;
  $('caps-summary').textContent=`自动导航：前后 ±${d.navigation_caps.max_vx_mps} m/s · 横移 ±${d.navigation_caps.max_vy_mps} m/s · 转向 ±${d.navigation_caps.max_wz_radps} rad/s。手操滑块另行控制，上限 1 / 1。`;
  const job=d.terrain_job;
  $('terrain-state').textContent=`地图：${d.map||'未选择'} · PCD：${d.map_saved?'已保存':'未保存'} · Tomogram：${d.tomogram_current?'最新':d.tomogram_available?'需要重建':'尚无'} · 开发本 5070：${workerOnline?'在线':'未连接'} · 任务：${job?.state||'无'}`;
  $('terrain-error').textContent=job?.error||'无错误；任务日志在开发本 scan_nav/logs/ 下。';
  const poseStatus=$('pose-file-status'),poseDetail=$('pose-file-detail');
  if(d.relocalization_ready){
    poseStatus.textContent='✓ 本次重定位已生成 pose.txt，可启动实时定位';
    poseStatus.className='metric';
  }else if(d.processes.init){
    poseStatus.textContent='⏳ 重定位运行中，等待本次新 pose.txt';
    poseStatus.className='';
  }else if(d.pose_file_path){
    poseStatus.textContent='已有 pose.txt，但未确认为本次重定位的新结果';
    poseStatus.className='muted';
  }else{
    poseStatus.textContent=d.relocalization_started?'本次重定位尚未生成 pose.txt':'尚未生成 pose.txt';
    poseStatus.className='muted';
  }
  poseDetail.textContent=d.pose_file_path?
    `${d.pose_file_path} · 写入于 ${new Date(d.pose_file_updated_ms).toLocaleString()}`:'';
  $('localization-state').textContent=located?`定位正常：${d.pose.map(x=>Number(x).toFixed(3)).join('，')}`:
    `模式 ${d.mode}；实时定位进程 ${d.processes.localization?'运行中':'未运行'}。`;
  const tracking=Number.isInteger(d.active_waypoint_index)?
    ` · 当前追踪路径点 ${d.active_waypoint_index+1}/${d.active_waypoint_total}`:'';
  $('route-state').textContent=`全局 ${d.global_path?.length||0} 点 · 局部 ${d.local_path?.length||0} 点 · ${d.armed?'底盘已启用':'底盘未启用'}${tracking}`;
  const rgbState=d.rgb_age_s!==null&&d.rgb_age_s<2?'RGB 在线':d.rgb_error||'等待彩色相机';
  $('rgb-state').textContent=rgbState;
  $('nav-rgb-state').textContent=rgbState;
  $('record-waypoint').disabled=recordingWaypoint||d.mode!=='mapping'||!d.processes?.mapping||
    d.pose_frame!=='world'||d.pose_age_s===null||d.pose_age_s>.5;
  $('manual-arm').textContent=d.control==='manual'&&d.armed?'手操已启用':'启用手操';
  $('drive-readout').textContent=`${d.control==='manual'&&d.armed?'手操已启用':'未使能'} · vx ${lastDrive[0].toFixed(2)} · vy ${lastDrive[1].toFixed(2)} · wz ${lastDrive[2].toFixed(2)}`;
  $('build-terrain').disabled=!d.map_saved||d.mode!=='idle'||!workerOnline||['pending','running'].includes(job?.state);
  robotArrow.visible=!!d.pose&&d.pose_age_s<2;
  if(robotArrow.visible){robotArrow.position.set(d.pose[0],d.pose[2],d.pose[1]);robotArrow.setDirection(new THREE.Vector3(Math.cos(d.pose[3]),0,Math.sin(d.pose[3])))}
}
async function refresh() {
  try {
    const response=await fetch('/api/status',{cache:'no-store'});
    if(!response.ok)throw Error(`HTTP ${response.status}`);
    const d=await response.json();state=d;
    updateStatusUI(d);
    updateWaypointTracking();
    updateRobotFollow(d);
    renderGuideStatus();
    if(!firstStatusReceived){firstStatusReceived=true;if(!d.error)message('系统在线；底盘默认未启用。')}
    if(d.relocalization_ready&&!lastPoseReady){
      message('本次重定位已生成 pose.txt；现在可启动实时定位。');
      logEvent('本次重定位已生成 pose.txt');
    }
    lastPoseReady=!!d.relocalization_ready;
    const select=$('map-select'),old=select.value;
    select.replaceChildren(new Option('选择地图',''));
    d.maps.forEach(n=>select.add(new Option(n,n)));
    select.value=d.map||old;
    if(d.map!==loadedWaypointMap){loadedWaypointMap=d.map;loadedWaypointRevision=null;waypointsDirty=false;
      waypoints=[];renderWaypoints();if(d.map)loadWaypoints(d.map)}
    else if(d.map&&!waypointsDirty&&d.waypoint_mtime_ns!==loadedWaypointRevision){
      loadedWaypointRevision=d.waypoint_mtime_ns;loadWaypoints(d.map);
    }
    const mapKey=`${d.map}:${d.map_mtime_ns}`;
    if(mapKey!==lastMapKey){lastMapKey=mapKey;mapCloud=replaceCloud(mapCloud,[],mapMaterial);if(d.map_mtime_ns)loadMapPoints(d.map,mapKey)}
    const world=d.world_points?.length&&d.cloud_age_s!==null&&d.cloud_age_s<2;
    const points=world?d.world_points:(d.map_mtime_ns?[]:d.raw_points);
    const frame=world?'world':'sensor';
    const cloudKey=frame+':'+(world?d.world_revision:d.raw_revision);
    if(cloudKey!==lastCloudKey){lastCloudKey=cloudKey;liveCloud=replaceCloud(liveCloud,points,liveMaterial,true);if(points?.length)$('cloud-label').textContent=`${frame==='world'?'实时世界点云':'雷达点云'} · ${points.length} 点`}
    if(liveCloud)liveCloud.visible=$('show-live').checked;
    if(mapCloud)mapCloud.visible=$('show-map').checked;
    const globalKey=String(d.global_revision);
    if(globalKey!==lastGlobalKey){lastGlobalKey=globalKey;globalLine=replaceLine(globalLine,d.global_path,0xffc869)}
    const localKey=String(d.local_revision);
    if(localKey!==lastLocalKey){lastLocalKey=localKey;localLine=replaceLine(localLine,d.local_path,0x69e6bf)}
    if(globalLine)globalLine.visible=$('show-global').checked;
    if(localLine)localLine.visible=$('show-local').checked;
    if(d.mode==='mapping'&&!mapCloud&&d.pose){groundZ=d.pose[2]-.3;ground.constant=-groundZ;grid.position.y=groundZ}
  } catch(error) { $('state-pill').textContent='页面连接失败';$('state-pill').className='alert';message(String(error),true) }
}
setInterval(refresh,500);refresh();
for(const id of ['show-map','show-live','show-global','show-local'])$(id).onchange=refresh;

$('create-map').onclick=()=>action('new_map',{name:$('new-map').value.trim()}).then(r=>{if(r)tab('mapping')});
$('select-map').onclick=()=>action('select_map',{name:$('map-select').value});
$('start-mapping').onclick=()=>action('start_mapping');
$('save-map').onclick=()=>action('save_map');
async function recordCurrentWaypoint(){
  if(busy||recordingWaypoint||waypointsDirty){
    if(waypointsDirty)message('导航页有未保存的路径点编辑，请先保存再继续记录。',true);
    return;
  }
  recordingWaypoint=true;$('record-waypoint').disabled=true;
  try{
    const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Scan-Nav':'operator'},
      body:JSON.stringify({action:'record_waypoint'})});
    const result=await response.json();if(!response.ok||!result.ok)throw Error(result.error||`HTTP ${response.status}`);
    waypoints=result.waypoints.map(p=>p.map(Number));
    loadedWaypointRevision=result.state?.waypoint_mtime_ns??null;
    waypointsDirty=false;renderWaypoints();
    const p=result.waypoint;message(`路径点 ${waypoints.length} 已保存：X ${p[0]} · Y ${p[1]} · 地面 Z ${p[2]} m`);
    logEvent(`建图记录路径点 ${waypoints.length}`);
  }catch(error){message(`路径点保存失败：${error}`,true);logEvent(`路径点保存失败：${error}`,true)}
  finally{recordingWaypoint=false;await refresh()}
}
$('record-waypoint').onclick=recordCurrentWaypoint;
$('manual-arm').onclick=()=>action('arm_manual');
$('manual-disarm').onclick=()=>{releaseMotion();action('disarm')};
$('build-terrain').onclick=()=>action('build_terrain');
$('restart-lidar').onclick=()=>action('restart_lidar');
$('relocalize').onclick=()=>action('relocalize',Object.fromEntries(['x','y','z','yaw'].map(k=>[k,Number($('seed-'+k).value)])));
$('localization').onclick=()=>action('start_localization');
$('stop').onclick=()=>{releaseMotion();action('stop')};
$('estop').onclick=()=>{releaseMotion();action('estop')};
$('wp-click-mode').onclick=()=>{clickMode=!clickMode;$('wp-click-mode').textContent=clickMode?'地图取点：开启':'开启地图取点';$('wp-click-mode').classList.toggle('amber',clickMode)};
$('add-waypoint').onclick=()=>{waypoints.push([state?.pose?.[0]||0,state?.pose?.[1]||0,groundZ]);waypointsDirty=true;renderWaypoints()};
$('clear-waypoints').onclick=()=>{waypoints=[];waypointsDirty=true;renderWaypoints()};
$('save-waypoints').onclick=()=>action('save_waypoints',{waypoints});
$('preview-route').onclick=()=>action('preview_route',{waypoints});
$('start-nav').onclick=()=>action('start_nav',{waypoints});
$('apply-route').onclick=()=>action('apply_route',{waypoints});
$('export-waypoints').onclick=()=>download(`${state?.map||'scan-nav'}-waypoints.json`,JSON.stringify({waypoints},null,2));
$('import-waypoints').onclick=()=>$('waypoint-file').click();
$('waypoint-file').onchange=async e=>{const file=e.target.files?.[0];if(!file)return;try{
  const parsed=JSON.parse(await file.text()),points=Array.isArray(parsed)?parsed:parsed.waypoints;
  if(!Array.isArray(points)||!points.length||points.length>100||points.some(p=>!Array.isArray(p)||p.length!==3||p.some(x=>!Number.isFinite(Number(x)))))throw Error('路径点 JSON 格式错误');
  waypoints=points.map(p=>p.map(Number));waypointsDirty=true;renderWaypoints();message(`已导入 ${waypoints.length} 个路径点`);
}catch(error){message(String(error),true)}e.target.value=''};

// Manual controls have their own 1 m/s and 1 rad/s limits. Navigation caps
// remain separately configurable in the robot settings and on the server.
const activeKeys=new Set();let desired=[0,0,0],inFlight=false;
const sliders=[['manual-linear','linear-value','m/s'],['manual-lateral','lateral-value','m/s'],['manual-yaw','yaw-value','rad/s']];
sliders.forEach(([id,out,unit])=>{const saved=localStorage.getItem(`scan-nav-${id}`);if(saved!==null)$(id).value=saved;
  const update=()=>{$(out).textContent=`${Number($(id).value).toFixed(2)} ${unit}`;localStorage.setItem(`scan-nav-${id}`,$(id).value)};
  $(id).oninput=update;update()});
function manualVector() {
  const x=(activeKeys.has('w')?1:0)-(activeKeys.has('s')?1:0);
  const y=(activeKeys.has('q')?1:0)-(activeKeys.has('e')?1:0);
  const yaw=(activeKeys.has('a')?1:0)-(activeKeys.has('d')?1:0);
  let vx=x*Number($('manual-linear').value),vy=y*Number($('manual-lateral').value);
  const magnitude=Math.hypot(vx,vy);if(magnitude>1){vx/=magnitude;vy/=magnitude}
  return [vx,vy,yaw*Number($('manual-yaw').value)];
}
async function flushMotion() {
  if(inFlight)return;inFlight=true;
  try{while(desired){const value=desired;desired=null;
    await fetch('/api/drive',{method:'POST',headers:{'Content-Type':'application/json','X-Scan-Nav':'operator'},
      body:JSON.stringify({vx:value[0],vy:value[1],wz:value[2]})});
  }}catch(_){}finally{inFlight=false}
}
function sendMotion() {
  if(!(state?.armed&&state.control==='manual'))return;
  lastDrive=manualVector();desired=lastDrive;flushMotion();
  $('drive-readout').textContent=`手操已启用 · vx ${lastDrive[0].toFixed(2)} · vy ${lastDrive[1].toFixed(2)} · wz ${lastDrive[2].toFixed(2)}`;
}
function releaseMotion() {
  const wasDriving=!!(state?.armed&&state.control==='manual');
  activeKeys.clear();document.querySelectorAll('[data-key]').forEach(b=>b.classList.remove('active'));
  lastDrive=[0,0,0];
  if(wasDriving){desired=[0,0,0];flushMotion()}else desired=null;
}
setInterval(()=>{if(activeKeys.size)sendMotion()},100);
document.addEventListener('keydown',e=>{
  if(e.altKey||e.ctrlKey||e.metaKey||e.target.closest('input,textarea,select,[contenteditable=true]'))return;
  const key=e.key.toLowerCase();if(!'wasdqe'.includes(key)||key.length!==1)return;
  if(!(state?.armed&&state.control==='manual'&&activeTab==='mapping'))return;
  e.preventDefault();activeKeys.add(key);document.querySelector(`[data-key="${key}"]`)?.classList.add('active');sendMotion();
});
document.addEventListener('keyup',e=>{const key=e.key.toLowerCase();if(activeKeys.delete(key)){e.preventDefault();document.querySelector(`[data-key="${key}"]`)?.classList.remove('active');sendMotion()}});
document.querySelectorAll('[data-key]').forEach(button=>{
  button.onpointerdown=e=>{if(!(state?.armed&&state.control==='manual'))return;e.preventDefault();button.setPointerCapture(e.pointerId);activeKeys.add(button.dataset.key);button.classList.add('active');sendMotion()};
  button.onpointerup=()=>{activeKeys.delete(button.dataset.key);button.classList.remove('active');sendMotion()};
  button.onpointercancel=releaseMotion;
});
window.addEventListener('blur',releaseMotion);
document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseMotion()});
setInterval(()=>{if(state?.armed&&state.control==='navigation')fetch('/api/heartbeat',{method:'POST',headers:{'X-Scan-Nav':'operator'}}).catch(()=>{})},200);

const robotSpec=[
  ['body_length_m','车长 / m',.1,2,.01],['body_width_m','车宽 / m',.1,2,.01],['body_height_m','车高 / m',.1,2,.01],
  ['mount_bottom_in_base_m.0','雷达安装 X / m',-2,2,.01],['mount_bottom_in_base_m.1','雷达安装 Y / m',-2,2,.01],['mount_bottom_in_base_m.2','雷达安装 Z / m',-2,2,.01],
  ['mount_rpy_deg.0','雷达 Roll / °',-180,180,.1],['mount_rpy_deg.1','雷达 Pitch / °',-180,180,.1],['mount_rpy_deg.2','雷达 Yaw / °',-180,180,.1],
  ['lidar_origin_above_mount_m','雷达光学中心离底座 / m',0,.3,.001],
  ['max_vx_mps','自动前后上限 / m/s',.05,1,.01],['max_vy_mps','自动横移上限 / m/s',.05,1,.01],['max_wz_radps','自动角速度上限 / rad/s',.05,1,.01],
  ['map_step_max_m','最大台阶 / m',0,.5,.01],['map_slope_max_rad','最大坡度 / °',0,45,.1,'degrees'],
  ['additional_clearance_m','侧向额外间距 / m',0,1,.005],['target_min_passage_m','标称最窄直通道 / m',.1,2,.01],
  ['local_map_resolution_m','实时碰撞栅格 / m',.03,.5,.01],
  ['lidar_ip','雷达 IPv4'],['lidar_host_ip','主控雷达网口 IPv4（需先配置 enp1s0）']
];
const terrainSpec=[
  ['resolution_m','Tomogram 分辨率 / m',.03,.5,.01],['ground_quantile','地面分位数',0,.5,.01],
  ['slice_height_m','切片高度 / m',.1,2,.05],['kernel_size','可通行核（奇数）',3,21,2],
  ['sensor_protrusion_m','雷达高出车体 / m',0,.5,.01],['free_clearance_margin_m','净空附加量 / m',0,.5,.01],
  ['standable_ratio','可站立像素比例',0,1,.01],['barrier_cost','硬障碍代价',1,1000,1],
  ['safe_margin_m','安全过渡带 / m',0,1,.01],['safe_margin_gamma','安全代价指数',.01,10,.01]
];
function getPath(object,path){return path.split('.').reduce((v,k)=>v?.[k],object)}
function setPath(object,path,value){const parts=path.split('.');let cursor=object;for(const p of parts.slice(0,-1))cursor=cursor[p];cursor[parts.at(-1)]=value}
function buildFields(container,spec,data){const target=$(container);target.replaceChildren();for(const [path,label,min,max,step,unit] of spec){
  const wrap=document.createElement('label');wrap.textContent=label;wrap.title=label;const input=document.createElement('input');input.dataset.path=path;
  if(typeof getPath(data,path)==='number'){input.type='number';input.min=min;input.max=max;input.step=step;const value=getPath(data,path);input.value=unit==='degrees'?value*180/Math.PI:value}
  else {input.type='text';input.value=getPath(data,path)||''}wrap.appendChild(input);target.appendChild(wrap)
}}
function collectFields(container,spec,original){const result=structuredClone(original);for(const [path,,min,max,,unit] of spec){const input=$(container).querySelector(`[data-path="${path}"]`);
  let value=input.type==='number'?Number(input.value):input.value.trim();if(input.type==='number'){
    if(!Number.isFinite(value)||value<min||value>max)throw Error(`${path} 超出范围 ${min}–${max}`);
    if(unit==='degrees')value=value*Math.PI/180;
    if(path==='kernel_size'&&(!Number.isInteger(value)||value%2!==1))throw Error('核大小必须为奇数');
  }setPath(result,path,value)}return result}
async function loadSettings(){try{const response=await fetch('/api/settings');if(!response.ok)throw Error(`HTTP ${response.status}`);
  settings=await response.json();buildFields('robot-fields',robotSpec,settings.robot);buildFields('terrain-fields',terrainSpec,settings.terrain);
  const pct=$('pct-fields');pct.replaceChildren();const toggle=document.createElement('label');toggle.textContent='五次 B-spline';const check=document.createElement('input');check.type='checkbox';check.id='pct-quintic';check.checked=settings.pct.use_quintic;toggle.appendChild(check);pct.appendChild(toggle);
  const heading=document.createElement('label');heading.textContent='最大航向变化率';const input=document.createElement('input');input.type='number';input.min='.01';input.max='100';input.step='.1';input.id='pct-heading';input.value=settings.pct.max_heading_rate;heading.appendChild(input);pct.appendChild(heading);
  yamlDrafts={...settings.yaml};yamlSelected=$('yaml-select').value;$('yaml-editor').value=yamlDrafts[yamlSelected];
}catch(error){message(`读取参数失败：${error}`,true)}}
$('yaml-select').onchange=()=>{yamlDrafts[yamlSelected]=$('yaml-editor').value;yamlSelected=$('yaml-select').value;$('yaml-editor').value=yamlDrafts[yamlSelected]||''};
if(matchMedia('(min-width: 1400px) and (min-height: 900px), (min-width: 1200px) and (min-height: 600px) and (min-resolution: 1.5dppx)').matches)
  document.querySelectorAll('#tab-settings details').forEach(section=>{section.open=true});
$('save-robot').onclick=async()=>{try{if(!settings)await loadSettings();const value=collectFields('robot-fields',robotSpec,settings.robot);await saveSettings('robot',{value});}catch(error){message(String(error),true)}};
$('save-terrain').onclick=async()=>{try{if(!settings)await loadSettings();const value=collectFields('terrain-fields',terrainSpec,settings.terrain);await saveSettings('terrain',{value});}catch(error){message(String(error),true)}};
$('save-pct').onclick=()=>saveSettings('pct',{value:{use_quintic:$('pct-quintic').checked,max_heading_rate:Number($('pct-heading').value)}});
$('save-yaml').onclick=()=>saveSettings('yaml',{name:yamlSelected,content:$('yaml-editor').value});
$('export-settings').onclick=()=>{if(settings)download('scan-nav-settings.json',JSON.stringify(settings,null,2))};
async function saveSettings(category,body){try{message(`保存 ${category} 中…`);const response=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json','X-Scan-Nav':'operator'},body:JSON.stringify({category,...body})});
  const result=await response.json();if(!response.ok||!result.ok)throw Error(result.error||`HTTP ${response.status}`);
  message(`${category} 参数已保存。新启动的组件将读取新参数。`);logEvent(`${category} 参数已保存`);await loadSettings();await refresh();
}catch(error){message(`参数保存失败：${error}`,true);logEvent(`参数保存失败：${error}`,true)}}

// A fixed tutorial script supplies content; these read-only checks bind it to
// the current UI and robot state without invoking any robot action endpoint.
const guideChecks={
  sensors:d=>d.lidar_age_s!==null&&d.lidar_age_s<1&&d.imu_age_s!==null&&d.imu_age_s<1,
  realMap:d=>!!d.map&&d.map!=='scan_20260923',
  mappingBase:d=>d.mode==='mapping'&&d.processes?.mapping&&d.processes?.base,
  rgb:d=>d.rgb_age_s!==null&&d.rgb_age_s<2,
  manualArmed:d=>d.mode==='mapping'&&d.armed&&d.control==='manual',
  mapping:d=>d.mode==='mapping'&&d.processes?.mapping,
  waypointSaved:d=>!!d.waypoint_mtime_ns&&waypoints.length>0&&!waypointsDirty,
  mapSaved:d=>d.map_saved&&d.tiles_saved,
  mappingStopped:d=>d.mode==='idle'&&!d.armed&&!d.processes?.base&&!d.processes?.mapping,
  tomogram:d=>d.tomogram_current,
  relocalized:d=>d.relocalization_ready,
  localization:d=>d.mode==='localization'&&d.processes?.localization&&d.pose_age_s!==null&&d.pose_age_s<1,
  waypoints:()=>waypoints.length>0,
  globalPath:d=>(d.global_path?.length||0)>=2,
  realGlobalPath:d=>(d.global_path?.length||0)>=2&&d.map!=='scan_20260923',
  navigation:d=>d.armed&&d.control==='navigation'&&d.processes?.planner,
  navigationStopped:d=>guideNavigationSeen&&d.mode==='idle'&&!d.armed
};

function guideTarget(){return document.querySelector(guideSteps[guideIndex].target)}
function clearGuideHighlight(){document.querySelectorAll('.tutorial-highlight').forEach(node=>node.classList.remove('tutorial-highlight'))}
function locateGuideTarget(){
  if(!guideVisible)return;
  const step=guideSteps[guideIndex];
  if(activeTab!==step.tab)tab(step.tab);
  clearGuideHighlight();
  const target=guideTarget();
  if(target){target.classList.add('tutorial-highlight');target.scrollIntoView({behavior:'smooth',block:'center',inline:'nearest'})}
}
function renderGuideStatus(){
  if(!guideVisible)return;
  if(state?.armed&&state.control==='navigation'){
    guideNavigationSeen=true;sessionStorage.setItem('scan-nav-guide-navigation-seen','1');
  }
  const step=guideSteps[guideIndex];
  const baseReady=!!(state&&guideChecks[step.check](state));
  const confirmed=guideConfirmed.has(guideIndex);
  const ready=baseReady&&(!step.confirm||confirmed);
  const status=$('guide-status');
  status.className=`guide-status ${ready?'ok':'wait'}`;
  status.textContent=!state?'正在读取系统状态…':ready?`✓ ${step.done}`:
    baseReady&&step.confirm?'请完成现场检查并勾选确认。':`待完成：${step.wait}`;
  $('guide-confirm-row').hidden=!step.confirm;
  $('guide-confirm').checked=confirmed;
  $('guide-next').disabled=guideIndex===guideSteps.length-1&&!ready;
}
function renderGuide(){
  if(!guideVisible)return;
  const step=guideSteps[guideIndex];
  $('guide-progress').textContent=`步骤 ${guideIndex+1} / ${guideSteps.length}`;
  $('guide-title').textContent=step.title;
  $('guide-instructions').textContent=step.instructions;
  $('guide-prev').disabled=guideIndex===0;
  $('guide-next').textContent=guideIndex===guideSteps.length-1?'完成教程':'下一步';
  $('guide-meter-fill').style.width=`${(guideIndex+1)/guideSteps.length*100}%`;
  renderGuideStatus();
}
function goToGuideStep(index){
  guideIndex=Math.max(0,Math.min(guideSteps.length-1,index));
  sessionStorage.setItem('scan-nav-guide-step',String(guideIndex));
  renderGuide();
  requestAnimationFrame(locateGuideTarget);
}
function closeGuide(){
  guideVisible=false;$('guide').hidden=true;$('guide-launch').setAttribute('aria-expanded','false');
  clearGuideHighlight();
}
function openGuide(restart=false){
  if(restart){guideConfirmed.clear();guideNavigationSeen=false;sessionStorage.removeItem('scan-nav-guide-navigation-seen');
    sessionStorage.removeItem('scan-nav-guide-confirmed');guideIndex=0}
  guideVisible=true;$('guide').hidden=false;$('guide-launch').setAttribute('aria-expanded','true');
  goToGuideStep(guideIndex);
}
$('guide-launch').onclick=()=>openGuide(true);
$('guide-close').onclick=closeGuide;
$('guide-restart').onclick=()=>openGuide(true);
$('guide-locate').onclick=locateGuideTarget;
$('guide-prev').onclick=()=>goToGuideStep(guideIndex-1);
$('guide-next').onclick=()=>guideIndex===guideSteps.length-1?closeGuide():goToGuideStep(guideIndex+1);
$('guide-confirm').onchange=event=>{
  if(event.target.checked)guideConfirmed.add(guideIndex);else guideConfirmed.delete(guideIndex);
  sessionStorage.setItem('scan-nav-guide-confirmed',JSON.stringify([...guideConfirmed]));
  renderGuideStatus();
};
try{
  const saved=Number(sessionStorage.getItem('scan-nav-guide-step'));
  if(Number.isInteger(saved)&&saved>=0&&saved<guideSteps.length)guideIndex=saved;
  for(const index of JSON.parse(sessionStorage.getItem('scan-nav-guide-confirmed')||'[]'))
    if(Number.isInteger(index)&&index>=0&&index<guideSteps.length)guideConfirmed.add(index);
  guideNavigationSeen=sessionStorage.getItem('scan-nav-guide-navigation-seen')==='1';
}catch(_){guideIndex=0;guideConfirmed.clear()}
if(new URLSearchParams(location.search).get('guide')==='1')openGuide(true);
if(location.hash==='#patrol')tab('patrol');
