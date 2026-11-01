// File editor only. No velocity publishing, task scheduling, or automatic navigation.
export function createPatrolEditor({getMap, getGroundZ, message, draw, action, getSeed}) {
  const $=id=>document.getElementById(id);
  $('tab-patrol').innerHTML=`
    <div class="patrol-head"><h2>原生巡检任务</h2><span id="patrol-map" class="badge">地图 —</span></div>
    <p class="hint">使用当前 LAB 地图；绿点为巡检顺序点，紫点为 Home。坐标直接写入原生 YAML，不添加高度偏移。</p>
    <p id="patrol-file"></p>
    <div class="info-card">可随时编辑并保存。原生管理器在 <code>/patrol/start</code> 时读取文件；运行中的巡检不改变，不自动取消或重启。保存不会让机器人运动。</div>
    <div class="info-card">
      <div class="row"><button id="patrol-start" class="amber">保存并启动巡检任务</button><button id="patrol-cancel" class="secondary">取消巡检 / 启动</button></div>
      <p class="hint">启动会实际运动：保存当前任务 → 重定位/定位 → 原生托管 PCT、SCAN、manager → 现有底盘 → /patrol/start。未运行定位时使用「定位」页初始位姿；已运行则复用。</p>
      <p id="patrol-startup-state" role="status">等待系统状态</p><p id="patrol-native-state" role="status"></p>
    </div>
    <div class="patrol-fields">
      <label>坐标系 frame_id<input id="patrol-frame" value="world"></label>
      <label>导航超时 / s<input id="patrol-timeout" type="number" step="1" min="0.001"></label>
      <label>XY 到达容差 / m<input id="patrol-xy" type="number" step="0.01" min="0.001"></label>
      <label>Z 到达容差 / m<input id="patrol-z" type="number" step="0.01" min="0.001"></label>
    </div>
    <h3>顺序巡检点</h3>
    <label>读取当前地图已保存路径点<select id="patrol-map-point"></select></label>
    <div class="row"><button id="patrol-add-saved">添加所选地图点</button><button id="patrol-refresh-points" class="ghost">刷新地图点</button></div>
    <div class="row"><button id="patrol-manual" class="secondary">＋ 手输坐标</button><button id="patrol-pick" class="secondary">地图点击添加</button><button id="patrol-replace" class="ghost">地图替换选中点</button></div>
    <p class="hint">地图点击使用当前地图地面 Z；每点 XYZ 均可手动改。等待时间仅占位，不执行真实检查。添加多个点的次序就是巡检次序。</p>
    <div id="patrol-points"></div>
    <label class="patrol-check"><input id="patrol-home-enabled" type="checkbox">巡检结束后返回 Home</label>
    <div id="patrol-home"></div>
    <div class="row" id="patrol-home-actions"><button id="patrol-home-saved" class="secondary">所选地图点设为 Home</button><button id="patrol-home-pick" class="ghost">地图选 Home</button></div>
    <div class="patrol-file-actions"><div id="patrol-feedback" role="status">尚未读取任务文件</div>
      <div class="row"><button id="patrol-save">保存并应用任务文件</button><button id="patrol-reload" class="secondary">重新读取</button><button id="patrol-export" class="ghost">导出 YAML</button></div>
    </div>
    <details><summary>原生 YAML 预览 / 手动命令参考</summary><pre id="patrol-preview"></pre><p class="hint">按钮只启动现有原生组件、调用原有服务。自动切点、占位等待与到达判断由 patrol_manager / navigation_manager 负责。不要与按钮启动的进程重复启动。</p><pre id="patrol-command"></pre></details>`;
  let mission=null,revision=null,dirty=false,visible=false,pick=null,selected=0,map=null,savedPoints=[],loading=false,saving=false,starting=false,runtime=null;
  let homeDraft={name:'Home',position:[0,0,0]};
  const feedback=(text,bad=false)=>{$('patrol-feedback').textContent=text;$('patrol-feedback').classList.toggle('error-text',bad)};
  function redraw(){
    draw(visible&&mission ? [...mission.points.map((p,i)=>({...p,label:`${i+1}: ${p.name}`})),
      ...(mission.home?[{...mission.home,label:`Home: ${mission.home.name}`,isHome:true}]:[])] : []);
    // JSON is also valid YAML 1.2 and is accepted by the native PyYAML parser.
    $('patrol-preview').textContent=mission?JSON.stringify(mission,null,2):'';
    for(const [id,mode] of [['patrol-pick','add'],['patrol-replace','replace'],['patrol-home-pick','home']])$(id).classList.toggle('amber',pick===mode);
    $('patrol-pick').textContent=pick==='add'?'地图取点中 · 再点关闭':'地图点击添加';
  }
  function changed(){dirty=true;feedback('有未保存修改 · 保存后下次 /patrol/start 读取');redraw()}
  const num=value=>value===''?NaN:Number(value);
  function inputField(parent,labelText,value,onChange,{numeric=false,className=''}={}){
    const label=document.createElement('label');label.textContent=labelText;label.className=className;
    const input=document.createElement('input');input.type=numeric?'number':'text';input.value=value;if(numeric)input.step='any';
    input.oninput=()=>{onChange(numeric?num(input.value):input.value);changed()};label.append(input);parent.append(label);return input;
  }
  function pointCard(point,i,isHome=false){
    const card=document.createElement('div');card.className=`patrol-point${!isHome&&selected===i?' selected':''}`;
    const head=document.createElement('div');head.className='patrol-point-head';const title=document.createElement('strong');title.textContent=isHome?'Home':`巡检点 ${i+1}`;head.append(title);
    const button=(label,fn,disabled=false)=>{const b=document.createElement('button');b.textContent=label;b.className='ghost';b.disabled=disabled;b.onclick=fn;head.append(b)};
    if(!isHome){
      button(selected===i?'已选中':'选中',()=>{selected=i;renderPoints()});
      for(const [text,delta] of [['↑',-1],['↓',1]])button(text,()=>{const j=i+delta;[mission.points[i],mission.points[j]]=[mission.points[j],mission.points[i]];selected=j;renderPoints();changed()},i+delta<0||i+delta>=mission.points.length);
      button('删除',()=>{mission.points.splice(i,1);selected=Math.min(selected,mission.points.length-1);renderPoints();changed()});
    }
    const fields=document.createElement('div');fields.className='patrol-point-fields';
    inputField(fields,'名称',point.name,v=>{point.name=v},{className:'patrol-name'});
    if(!isHome)inputField(fields,'占位等待 / s',point.inspection_duration_sec,v=>{point.inspection_duration_sec=v},{numeric:true});
    ['X / m','Y / m','Z / m'].forEach((label,j)=>inputField(fields,label,point.position[j],v=>{point.position[j]=v},{numeric:true}));
    card.append(head,fields);return card;
  }
  function renderPoints(){
    $('patrol-points').replaceChildren(...mission.points.map((p,i)=>pointCard(p,i)));
    $('patrol-home').replaceChildren(...(mission.home?[pointCard(mission.home,0,true)]:[]));
    $('patrol-home-enabled').checked=!!mission.home;$('patrol-home-actions').hidden=!mission.home;redraw();
  }
  async function read(){
    if(loading||saving)return;
    if(dirty&&!confirm('放弃未保存修改，重新读取巡检任务文件？'))return;
    loading=true;
    try{
      const response=await fetch('/api/settings');if(!response.ok)throw Error(`HTTP ${response.status}`);
      const data=(await response.json()).patrol;if(!data||data.error)throw Error(data?.error||'服务端未提供巡检任务');
      mission=data.value;revision=data.revision;dirty=false;pick=null;selected=0;
      if(mission.home)homeDraft=structuredClone(mission.home);
      $('patrol-file').textContent=data.file;
      $('patrol-command').textContent=`# 在小车终端执行；定位和原生托管导航组件需按原流程启动\nsource /home/yiyang/scan_nav/deployment/env.sh\nros2 run patrol_manager patrol_manager --ros-args -p mission_file:=${data.file}\n# 另一个加载了相同环境的终端：\nros2 service call /patrol/start std_srvs/srv/Trigger '{}'\nros2 service call /patrol/cancel std_srvs/srv/Trigger '{}'`;
      for(const [id,key] of [['patrol-xy','xy_tolerance'],['patrol-z','z_tolerance'],['patrol-timeout','timeout_sec']])$(id).value=mission.navigation[key];
      $('patrol-frame').value=mission.frame_id;renderPoints();feedback('已读取任务文件 · 非正在运行任务的实时状态');await loadMapPoints();
    }catch(e){feedback(`读取失败：${e.message}`,true)}finally{loading=false}
  }
  async function loadMapPoints(){
    const requested=getMap();map=requested;$('patrol-map').textContent=`地图 ${requested||'未选择'}`;
    savedPoints=[];$('patrol-map-point').replaceChildren();if(!requested)return;
    try{
      const r=await fetch(`/api/maps/${encodeURIComponent(requested)}/waypoints`);if(!r.ok)throw Error(`HTTP ${r.status}`);
      const data=await r.json();if(getMap()!==requested)return;
      savedPoints=(data.waypoints||[]).filter(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite));
      $('patrol-map-point').replaceChildren(...savedPoints.map((p,i)=>new Option(`${requested} · 路径点 ${i+1} (${p.map(v=>v.toFixed(3)).join(', ')})`,String(i))));
      if(!savedPoints.length)$('patrol-map-point').append(new Option('此地图尚无已保存路径点，请手动选点',''));
    }catch(e){feedback(`地图路径点读取失败：${e.message}`,true)}
  }
  function add(position){
    let n=1;while(mission.points.some(p=>p.name===`P${n}`))n++;
    mission.points.push({name:`P${n}`,position:[...position],inspection_duration_sec:1});selected=mission.points.length-1;renderPoints();changed();
  }
  function chosen(){const index=$('patrol-map-point').value;return index===''?null:savedPoints[Number(index)]}
  function guard(fn){return ()=>{if(!mission||saving)return;fn()}}
  $('patrol-add-saved').onclick=guard(()=>{const p=chosen();if(p)add(p);else feedback('请先选择一个已保存地图点',true)});
  $('patrol-manual').onclick=guard(()=>add([0,0,getGroundZ()]));
  $('patrol-refresh-points').onclick=loadMapPoints;
  for(const [id,mode] of [['patrol-pick','add'],['patrol-replace','replace'],['patrol-home-pick','home']])$(id).onclick=guard(()=>{pick=pick===mode?null:mode;redraw();feedback(pick?'请在左侧地图上点击地面（拖动视角不会取点）':'已退出地图取点')});
  $('patrol-home-enabled').onchange=guard(()=>{if($('patrol-home-enabled').checked)mission.home=structuredClone(homeDraft);else{homeDraft=structuredClone(mission.home);delete mission.home;if(pick==='home')pick=null}renderPoints();changed()});
  $('patrol-home-saved').onclick=guard(()=>{const p=chosen();if(p&&mission.home){mission.home.position=[...p];renderPoints();changed()}});
  $('patrol-frame').oninput=guard(()=>{mission.frame_id=$('patrol-frame').value;changed()});
  for(const [id,key] of [['patrol-xy','xy_tolerance'],['patrol-z','z_tolerance'],['patrol-timeout','timeout_sec']])$(id).oninput=guard(()=>{mission.navigation[key]=num($(id).value);changed()});
  $('patrol-reload').onclick=read;
  async function saveMission(){
    if(!mission||saving)return;
    // Reject empty/nonfinite fields before JSON converts NaN into null.
    if(!mission.frame_id.trim()||!mission.points.length||!Object.values(mission.navigation).every(v=>Number.isFinite(v)&&v>0)||
      [...mission.points,...(mission.home?[mission.home]:[])].some(p=>!p.name.trim()||!p.position.every(Number.isFinite))||
      mission.points.some(p=>!Number.isFinite(p.inspection_duration_sec)||p.inspection_duration_sec<0)){
      feedback('请填写完整名称、有限 XYZ、正数容差/超时和非负等待时间；至少一个巡检点。',true);return;
    }
    saving=true;$('patrol-save').disabled=true;
    const snapshot=JSON.stringify(mission);
    try{
      const r=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json','X-Scan-Nav':'operator'},body:JSON.stringify({category:'patrol',value:JSON.parse(snapshot),revision})});
      const data=await r.json();if(!r.ok||!data.ok)throw Error(data.error||`HTTP ${r.status}`);
      revision=data.patrol.revision;dirty=JSON.stringify(mission)!==snapshot;
      feedback(`已保存到原生任务文件；下次 /patrol/start 生效。${dirty?'仍有新的未保存修改。':''}\n旧文件已备份：${data.patrol.backup}`);
      message('巡检任务已保存；未启动、取消或改变当前导航任务。');
      return !dirty;
    }catch(e){feedback(`保存失败：${e.message}`,true)}finally{saving=false;$('patrol-save').disabled=false}
  }
  $('patrol-save').onclick=saveMission;
  $('patrol-start').onclick=async()=>{
    if(starting||saving||!mission)return;
    starting=true;$('patrol-start').disabled=true;pick=null;redraw();
    try{
      if(!await saveMission())return;
      feedback('正在启动原生巡检流程；可随时点击取消巡检或急停。');
      const result=await action('start_patrol',{map:getMap(),revision,seed:getSeed()});
      if(result)feedback('原生巡检已启动。运行状态见上方；编辑仅在下次启动时生效。');
      else feedback('巡检未成功启动，请查看启动状态和页面底部错误。',true);
    }finally{starting=false;updateRuntime(runtime)}
  };
  $('patrol-cancel').onclick=()=>action('cancel_patrol');
  function updateRuntime(value){
    runtime=value;
    const boot=value?.patrol_startup,native=value?.patrol_status;
    $('patrol-startup-state').textContent=boot?`${boot.state} · ${boot.stage}`:'等待系统状态';
    $('patrol-startup-state').classList.toggle('error-text',boot?.state==='FAILED');
    const index=Number.isInteger(native?.point_index)?` · 当前顺序索引 ${native.point_index+1}`:'';
    $('patrol-native-state').textContent=native?`原生巡检：${native.state}${index} · ${native.reason||''}`:'原生巡检尚未运行';
    const active=native&&!['IDLE','SUCCEEDED','FAILED','CANCELED'].includes(native.state);
    $('patrol-start').disabled=starting||boot?.state==='STARTING'||!!active;
  }
  $('patrol-export').onclick=guard(()=>{const url=URL.createObjectURL(new Blob([JSON.stringify(mission,null,2)+'\n'],{type:'application/yaml'}));const a=document.createElement('a');a.href=url;a.download='patrol.yaml';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)});
  window.addEventListener('beforeunload',event=>{if(dirty){event.preventDefault();event.returnValue=''}});
  return {
    updateRuntime,
    show(on){visible=on;pick=null;if(on&&!mission)read();else if(on)loadMapPoints();redraw()},
    setMap(name){if(name!==map){map=name;$('patrol-map').textContent=`地图 ${name||'未选择'}`;if(visible)loadMapPoints()}},
    get picking(){return visible&&!!pick&&!!mission&&!saving},
    click(position){if(!this.picking)return;
      if(pick==='add')add(position);
      else if(pick==='home'&&mission.home){mission.home.position=[...position];renderPoints();changed();pick=null}
      else if(pick==='replace'&&mission.points[selected]){mission.points[selected].position=[...position];renderPoints();changed();pick=null}
      else feedback('请先选择一个巡检点',true);
      redraw();
    },
  };
}
