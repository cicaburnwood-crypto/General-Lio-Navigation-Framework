// Both tabs submit one immutable queue to manager. No sequencing/cmd_vel here.
import {normalizeMission, newPoint, mapPoints} from './waypoint_model.js?v=20261028rb1';

export function createPatrolEditor({getMap,message,draw,action,onPickingChange=()=>{},realtime=false}){
  const prefix=realtime?'live':'door';
  const $=key=>document.getElementById(`${prefix}-${key}`);
  const root=document.getElementById(realtime?'tab-realtime':'tab-patrol');
  let mission=null,revision='',selected=0,picking=false,visible=false,dirty=false,loading=false,runtime=null;
  let detecting=false,starting=false;
  root.innerHTML=`
    <h2>${realtime?'实时模式 · 临时顺序任务':'巡检任务 · 红点 / 蓝点'}</h2>
    <p id="${prefix}-map" class="hint"></p>
    <p class="info-card"><span class="point-blue">● 蓝点</span>：Nav2 到达后直接发布下一点，不额外停留或检测。
    <span class="point-red">● 红点</span>：按所设 Yaw 正对门，连续停稳后每秒检测；门全开才继续。
    ${realtime?'已定位后，在地图连续选点，点击确认即可执行。不保存 YAML，刷新页面未提交的草稿会丢失。':'保存任务不会执行；点击“保存并启动巡检”才运行。'}
    当前任务由 manager 持有快照，编辑草稿不改变已启动的队列。</p>
    <div class="row">${realtime?`<button id="${prefix}-clear" class="secondary">清空临时草稿</button>`:`<button id="${prefix}-read">重新读取</button><button id="${prefix}-save">保存任务</button>`}</div>
    <div class="row"><button id="${prefix}-start" class="amber">${realtime?'确认并立即顺序执行':'保存并启动巡检'}</button><button id="${prefix}-cancel" class="secondary">取消任务</button></div>
    <p id="${prefix}-runtime" role="status"></p><p id="${prefix}-message" role="status"></p>
    <div class="row"><button id="${prefix}-add-blue" class="point-blue">＋ 蓝点</button><button id="${prefix}-add-red" class="point-red">＋ 红点</button></div>
    <div class="row"><label>连续新增点类型<select id="${prefix}-new-type"><option value="blue">蓝点 · 普通路径点</option><option value="red">红点 · 停稳 / 门检测</option></select></label><button id="${prefix}-append">连续地图选点</button></div>
    <button id="${prefix}-pick" class="secondary wide">在地图替换选中点</button>
    <p class="hint">连续选点可每次添加一个点；点击设位置，拖动同时设朝向。点的类型、坐标、Yaw 和顺序均可编辑。红点请设置正对门的朝向。Yaw=0 朝 +X，+π/2 朝 +Y。选点本身不会启动。</p>
    <div id="${prefix}-points"></div>
    <details><summary>停稳与导航参数</summary><div class="door-fields" id="${prefix}-params"></div>
    <p class="hint">停稳与门前位置 / 朝向检查只作用于红点。蓝点沿用 Nav2 原生到达判断，无额外等待。Nav2 / MPPI 配置不变。</p></details>
    <h3>D435i RGB 门检测</h3><p class="hint">每次采集 1 秒，对 1–5 帧计算平均概率；超过 47% 返回全开。调用间隔至少 1 秒，不重叠请求。</p>
    <img id="${prefix}-rgb" class="door-camera" alt="门检测 RGB">
    <button id="${prefix}-detect">静止诊断：检测一次（不发布下一点）</button>
    <p id="${prefix}-detection"></p><div id="${prefix}-probs" class="door-probs"></div>
    <details><summary>${realtime?'临时任务快照（不保存文件）':'任务 YAML / JSON'}</summary><pre id="${prefix}-json"></pre></details>`;
  const info=(text,bad=false)=>{$('message').textContent=text;$('message').classList.toggle('error-text',bad)};
  const number=input=>input.value.trim()===''?null:Number(input.value);
  function changed(){dirty=true;$('json').textContent=JSON.stringify(mission,null,2);redraw();updateRuntime(runtime)}
  function redraw(){
    $('pick').textContent=picking==='replace'?'取消替换选点':`在地图替换第 ${selected+1} 点`;
    $('pick').disabled=!mission?.points.length;
    $('append').textContent=picking==='append'?'结束连续选点':'连续地图选点';
    if(visible){draw(mapPoints(mission,selected));onPickingChange(!!picking,selected,picking==='append')}
  }
  function render(){
    $('points').replaceChildren();
    if(!mission.points.length){const hint=document.createElement('p');hint.className='hint';hint.textContent='暂无路径点。点击“连续地图选点”，按顺序选择蓝点或红点。';$('points').append(hint)}
    mission.points.forEach((p,i)=>{
      const card=document.createElement('div');card.className='door-point'+(selected===i?' selected':'');
      card.onclick=()=>{selected=i;[...$('points').children].forEach((e,j)=>e.classList.toggle('selected',i===j));redraw()};
      const title=document.createElement('strong');
      const titleText=()=>{title.textContent=`${i+1} · ${p.type==='red'?'红点：停稳并检测门':'蓝点：普通路径点'}`;title.className=`point-${p.type}`};titleText();card.append(title);
      const fields=document.createElement('div');fields.className='door-fields';card.append(fields);
      const typeLabel=document.createElement('label');typeLabel.textContent='点类型';
      const type=document.createElement('select');type.setAttribute('aria-label',`第 ${i+1} 点类型`);
      type.innerHTML='<option value="blue">蓝点 · 普通路径点</option><option value="red">红点 · 停稳 / 门检测</option>';type.value=p.type;
      type.onchange=()=>{p.type=type.value;titleText();changed()};typeLabel.append(type);fields.append(typeLabel);
      for(const [label,value,update] of [['名称',p.name,v=>p.name=v],...['X / m','Y / m','Z / m'].map((s,j)=>[s,p.position[j],v=>p.position[j]=v]),['Yaw / rad',p.yaw,v=>p.yaw=v]]){
        const wrap=document.createElement('label');wrap.textContent=label;
        const input=document.createElement('input');input.type=label==='名称'?'text':'number';input.step='.01';input.value=value??'';
        input.oninput=()=>{update(input.type==='text'?input.value:number(input));changed()};wrap.append(input);fields.append(wrap);
      }
      const row=document.createElement('div');row.className='row';
      for(const [label,delta] of [['上移',-1],['下移',1],['删除',0]]){
        const button=document.createElement('button');button.textContent=label;button.className='secondary';
        button.disabled=delta!==0&&(i+delta<0||i+delta>=mission.points.length);
        button.onclick=e=>{e.stopPropagation();if(delta){[mission.points[i],mission.points[i+delta]]=[mission.points[i+delta],mission.points[i]];selected=i+delta}else{mission.points.splice(i,1);selected=Math.max(0,Math.min(i,mission.points.length-1));picking=false}render();changed()};row.append(button);
      }
      card.append(row);$('points').append(card);
    });
    $('params').replaceChildren();
    for(const [group,key,label] of [['navigation','timeout_sec','单点导航超时 / s'],['navigation','xy_tolerance','红点位置确认容差 / m'],['navigation','yaw_tolerance','正对门容差 / rad'],['door','settle_seconds','持续停稳 / s'],['door','linear_stopped_mps','静止线速度阈值 / m/s'],['door','angular_stopped_radps','静止角速度阈值 / rad/s'],['door','observation_max_age_sec','里程计最大年龄 / s']]){
      const wrap=document.createElement('label');wrap.textContent=label;const input=document.createElement('input');input.type='number';input.min='.001';input.step='.01';input.value=mission[group][key];
      input.oninput=()=>{mission[group][key]=number(input);changed()};wrap.append(input);$('params').append(wrap);
    }
    $('json').textContent=JSON.stringify(mission,null,2);redraw();updateRuntime(runtime);
  }
  async function read(){
    if(loading)return;loading=true;
    try{
      const response=await fetch('/api/settings',{cache:'no-store'});if(!response.ok)throw Error(`HTTP ${response.status}`);
      const d=(await response.json()).patrol;mission=normalizeMission(d.value);revision=d.revision;
      if(realtime){
        // Recover the submitted in-memory queue on a page reload; never replay it.
        mission.points=runtime?.patrol_status?.task_mode==='realtime'?structuredClone(runtime.patrol_status.points||[]):[];
      }
      selected=0;picking=false;dirty=false;render();info(realtime?'临时模式就绪：选点后确认执行，不保存 YAML。':'已读取任务；红点检测门，蓝点到达即继续。');
    }catch(e){info(String(e),true)}finally{loading=false}
  }
  async function save(){
    if(realtime||!mission)return false;
    try{
      const snapshot=JSON.stringify(mission);
      const response=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json','X-Scan-Nav':'operator'},body:JSON.stringify({category:'patrol',value:mission,revision})});
      const result=await response.json();if(!response.ok||!result.ok)throw Error(result.error);
      revision=result.patrol.revision;dirty=snapshot!==JSON.stringify(mission);info('任务已保存；不会改变正在运行的任务。');return !dirty;
    }catch(e){info(String(e),true);return false}
  }
  if(realtime)$('clear').onclick=()=>{if(!mission)return;mission.points=[];selected=0;picking=false;dirty=false;render();info('已清空临时草稿；不会取消正在运行的任务。')};
  else{$('read').onclick=read;$('save').onclick=save;window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue=''}})}
  for(const kind of ['blue','red'])$('add-'+kind).onclick=()=>{if(!mission)return;mission.points.push(newPoint(kind,mission.points.length));selected=mission.points.length-1;picking='replace';render();changed()};
  $('append').onclick=()=>{if(!mission)return;picking=picking==='append'?false:'append';redraw()};
  $('pick').onclick=()=>{picking=picking==='replace'?false:'replace';redraw()};
  $('start').onclick=async()=>{
    if(starting||!mission)return;starting=true;updateRuntime(runtime);
    try{
      if(realtime){const result=await action('start_realtime',{mission:structuredClone(mission)});if(result){picking=false;redraw();info('临时任务已交给 manager 顺序执行；没有保存 YAML。')}}
      else if(await save())await action('start_patrol',{map:getMap(),revision});
    }finally{starting=false;updateRuntime(runtime)}
  };
  $('cancel').onclick=()=>action('cancel_patrol');
  $('detect').onclick=async()=>{detecting=true;updateRuntime(runtime);try{const r=await action('detect_door');if(r)info(`服务结果：${r.result.opening_percent}；没有发布目标。`)}finally{detecting=false;updateRuntime(runtime)}};
  function updateRuntime(d){
    runtime=d;const p=d?.patrol_status,active=p&&!['IDLE','SUCCEEDED','FAILED','CANCELED'].includes(p.state);
    const localized=Array.isArray(d?.pose)&&d.pose.every(Number.isFinite)&&d.pose_age_s!=null&&d.pose_age_s<=1;
    const current=p?.points?.[p.point_index];
    $('runtime').textContent=d?.preview?'离线预览：只编辑，不调用机器人。':p?`${p.task_mode==='realtime'?'实时任务':'巡检任务'} · ${p.state} · ${Math.min(p.point_index+1,p.point_count)}/${p.point_count} ${current?(current.type==='red'?'红点':'蓝点'):''} · ${p.reason}${realtime&&!localized?' · 等待有效定位':''}`:'等待 manager';
    $('start').disabled=starting||!!active||!d||d.preview||!p||!d.nav_ready||!mission?.points.length||(realtime&&!localized);
    $('detect').disabled=detecting||!!active||!d||d.preview||!d.door_ready;
    const details=d?.door_details;$('detection').textContent=details?`结果 ${details.opening_percent} · ${details.sample_count??details.probabilities?.length??0} 帧 · 平均 ${Number.isFinite(details.mean_probability)?(details.mean_probability*100).toFixed(2)+'%':'—'} · ${details.reason||'检测完成'} · manager 调用 ${p?.door?.calls??0} 次`:'尚无检测结果';
    $('probs').replaceChildren(...(details?.probabilities||[]).map((v,i)=>{const e=document.createElement('span');e.textContent=`${i+1}: ${(v*100).toFixed(1)}%`;e.className=v>=details.threshold?'pass':'';return e}));
    if(visible&&d&&!d.preview&&!$('rgb').getAttribute('src'))$('rgb').src='/api/rgb.mjpeg';
  }
  return {
    show(on){visible=on;picking=false;if(on&&!mission)read();if(!on)$('rgb').removeAttribute('src');updateRuntime(runtime);redraw()},
    updateRuntime,setMap(name){$('map').textContent=`地图：${name||'未加载'} · map · 当前显示编辑草稿`},
    beginPick(){if(!mission){message('任务正在读取，请稍后。');return}picking=realtime||!mission.points.length?'append':'replace';redraw()},
    cancelPick(){picking=false;redraw()},
    select(index){if(!mission?.points[index])return;selected=index;render();$('points').children[index]?.scrollIntoView({block:'nearest'})},
    get picking(){return visible&&!!picking&&!!mission},
    click(position,yaw){
      if(!this.picking)return;
      if(picking==='append'){mission.points.push(newPoint($('new-type').value,mission.points.length,position,yaw));selected=mission.points.length-1}
      else{const point=mission.points[selected];point.position=[...position];point.yaw=Number.isFinite(yaw)?yaw:point.yaw??0;picking=false}
      render();changed();message(realtime?'临时点已更新；点击“确认并立即顺序执行”才启动，不保存 YAML。':'红/蓝点草稿已更新；保存并启动才执行。');
    }
  };
}
