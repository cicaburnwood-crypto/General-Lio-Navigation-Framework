// Display-only geometry: ROS map +X right, +Y up. Never sends ROS commands.
export function worldToScreen(point, view) {
  return [view.width/2+(point[0]-view.x)*view.scale,
          view.height/2-(point[1]-view.y)*view.scale];
}
export function screenToWorld(point, view) {
  return [view.x+(point[0]-view.width/2)/view.scale,
          view.y-(point[1]-view.height/2)/view.scale];
}
export function pixelToWorld(point, grid) {
  const x=point[0]*grid.resolution,y=(grid.height-point[1])*grid.resolution;
  const [ox,oy,a]=grid.origin,c=Math.cos(a),s=Math.sin(a);
  return [ox+c*x-s*y,oy+s*x+c*y];
}
export function mapBounds(grid) {
  const points=[[0,0],[grid.width,0],[0,grid.height],[grid.width,grid.height]].map(p=>pixelToWorld(p,grid));
  return [Math.min(...points.map(p=>p[0])),Math.min(...points.map(p=>p[1])),
          Math.max(...points.map(p=>p[0])),Math.max(...points.map(p=>p[1]))];
}
export function headingBetween(a,b) {return Math.atan2(b[1]-a[1],b[0]-a[0]);}
export function bodyToWorld(point,pose) {
  const c=Math.cos(pose[3]),s=Math.sin(pose[3]);
  return [pose[0]+c*point[0]-s*point[1],pose[1]+s*point[0]+c*point[1]];
}

export class Map2D {
  constructor(canvas,{onPick,onSelect,onInitial,onMessage,onFollow}={}) {
    this.canvas=canvas;this.ctx=canvas.getContext('2d');
    this.callbacks={onPick,onSelect,onInitial,onMessage,onFollow};
    this.view={width:1,height:1,x:0,y:0,scale:55};
    this.layers={map:true,live:false,global:true,local:true,grid:false};
    this.tool='pan';this.points=[];this.data={};this.measure=[];this.follow=false;
    this.resizeObserver=new ResizeObserver(()=>this.resize());this.resizeObserver.observe(canvas.parentElement);
    canvas.addEventListener('contextmenu',e=>e.preventDefault());
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom(Math.exp(-Math.max(-100,Math.min(100,e.deltaY))*.003),this.local(e));},{passive:false});
    canvas.addEventListener('pointerdown',e=>this.down(e));
    canvas.addEventListener('pointermove',e=>this.move(e));
    canvas.addEventListener('pointerup',e=>this.up(e));
    canvas.addEventListener('pointercancel',()=>{this.drag=null;this.render()});
    this.resize();
  }
  resize() {
    const rect=this.canvas.parentElement.getBoundingClientRect();
    this.view.width=Math.max(1,rect.width);this.view.height=Math.max(1,rect.height);
    this.dpr=Math.min(devicePixelRatio||1,2);
    this.canvas.width=Math.round(this.view.width*this.dpr);this.canvas.height=Math.round(this.view.height*this.dpr);
    this.render();
  }
  async load(grid) {
    const image=new Image();image.src=grid.image_url;await image.decode();
    this.grid=grid;this.image=image;
    if(!this.framed){this.fit();if(this.validPose())this.focus(true);this.framed=true}
    this.render();
  }
  validPose() {return Array.isArray(this.data.pose)&&this.data.pose.length>=4&&this.data.pose.every(Number.isFinite)}
  update(data) {
    this.data=data;
    if(this.validPose()&&!this.firstPose){this.firstPose=true;if(!this.userView)this.focus(true)}
    if(this.follow&&this.validPose()){[this.view.x,this.view.y]=data.pose}
    this.render();
  }
  setPoints(points) {this.points=points;this.render()}
  setTool(tool) {this.tool=tool;this.drag=null;this.canvas.dataset.tool=tool;if(tool==='measure')this.measure=[];this.render()}
  setFollow(on) {this.follow=on;this.callbacks.onFollow?.(on);if(on)this.focus(false)}
  focus(resetScale=false) {
    if(!this.validPose()){this.callbacks.onMessage?.('尚未收到定位；地图仍可查看。');return}
    [this.view.x,this.view.y]=this.data.pose;
    if(resetScale)this.view.scale=Math.min(this.view.width,this.view.height)/10;
    this.render();
  }
  fit() {
    if(!this.grid)return;
    this.setFollow(false);const [x0,y0,x1,y1]=mapBounds(this.grid);
    this.view.x=(x0+x1)/2;this.view.y=(y0+y1)/2;
    this.view.scale=Math.min(this.view.width/Math.max(.1,x1-x0),this.view.height/Math.max(.1,y1-y0))*.9;
    this.userView=true;this.render();
  }
  zoom(factor,anchor=[this.view.width/2,this.view.height/2]) {
    const before=screenToWorld(anchor,this.view);
    this.view.scale=Math.max(.5,Math.min(1000,this.view.scale*factor));
    const after=screenToWorld(anchor,this.view);
    this.view.x+=before[0]-after[0];this.view.y+=before[1]-after[1];
    this.userView=true;this.setFollow(false);this.render();
  }
  local(e) {const r=this.canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top]}
  down(e) {
    if(e.button!==0&&e.button!==2)return;
    const screen=this.local(e),pan=e.button===2||this.tool==='pan';
    this.drag={id:e.pointerId,screen,start:screenToWorld(screen,this.view),last:screen,pan,tool:this.tool,distance:0};
    this.canvas.setPointerCapture(e.pointerId);this.canvas.focus();e.preventDefault();
    if(pan){this.setFollow(false);this.userView=true}
  }
  move(e) {
    const screen=this.local(e),world=screenToWorld(screen,this.view);
    document.getElementById('map-coordinates').textContent=`map · X ${world[0].toFixed(3)} · Y ${world[1].toFixed(3)} m`;
    const d=this.drag;if(!d||d.id!==e.pointerId)return;
    d.distance=Math.max(d.distance,Math.hypot(screen[0]-d.screen[0],screen[1]-d.screen[1]));
    if(d.pan){this.view.x-=(screen[0]-d.last[0])/this.view.scale;this.view.y+=(screen[1]-d.last[1])/this.view.scale}
    d.last=screen;d.end=screenToWorld(screen,this.view);this.render();
  }
  up(e) {
    const d=this.drag;if(!d||d.id!==e.pointerId)return;
    this.drag=null;if(this.canvas.hasPointerCapture(e.pointerId))this.canvas.releasePointerCapture(e.pointerId);
    const screen=this.local(e),end=screenToWorld(screen,this.view);
    if(!d.pan&&this.grid){
      if(d.tool==='pick'||d.tool==='initial'){
        const yaw=Math.hypot(screen[0]-d.screen[0],screen[1]-d.screen[1])>=6?headingBetween(d.start,end):undefined;
        const position=[...d.start.map(v=>Number(v.toFixed(3))),this.grid.ground_z];
        (d.tool==='pick'?this.callbacks.onPick:this.callbacks.onInitial)?.(position,yaw);
      }else if(d.tool==='measure'&&d.distance<6){
        if(this.measure.length===2)this.measure=[];
        this.measure.push(end);
        if(this.measure.length===2)this.callbacks.onMessage?.(`两点距离 ${Math.hypot(this.measure[1][0]-this.measure[0][0],this.measure[1][1]-this.measure[0][1]).toFixed(3)} m（地图米制坐标）`);
      }
    }else if(d.pan&&d.distance<6&&e.button===0){
      const nearest=this.points.map(p=>({p,dist:Math.hypot(...worldToScreen(p.position,this.view).map((v,i)=>v-screen[i]))})).sort((a,b)=>a.dist-b.dist)[0];
      if(nearest?.dist<18)this.callbacks.onSelect?.(nearest.p.index);
    }
    this.render();
  }
  line(points,color,width=2,closed=false,fill=null) {
    if(!points?.length)return;const c=this.ctx;c.beginPath();
    points.forEach((p,i)=>{const [x,y]=worldToScreen(p,this.view);i?c.lineTo(x,y):c.moveTo(x,y)});
    if(closed)c.closePath();if(fill){c.fillStyle=fill;c.fill()}
    c.strokeStyle=color;c.lineWidth=width;c.stroke();
  }
  label(text,x,y,color='#fff') {
    const c=this.ctx;c.font='12px system-ui';const width=c.measureText(text).width;
    c.fillStyle='#112131df';c.fillRect(x-4,y-13,width+8,18);c.fillStyle=color;c.fillText(text,x,y);
  }
  arrow(position,yaw,color,length=30) {
    if(!Number.isFinite(yaw))return;const c=this.ctx,[x,y]=worldToScreen(position,this.view);
    c.save();c.translate(x,y);c.rotate(-yaw);c.strokeStyle='#122532';c.fillStyle=color;c.lineWidth=1.5;
    c.beginPath();c.moveTo(0,-3);c.lineTo(length-9,-3);c.lineTo(length-9,-8);c.lineTo(length+2,0);c.lineTo(length-9,8);c.lineTo(length-9,3);c.lineTo(0,3);c.closePath();c.fill();c.stroke();c.restore();
  }
  render() {
    const c=this.ctx,v=this.view;if(!c)return;
    c.setTransform(this.dpr||1,0,0,this.dpr||1,0,0);c.clearRect(0,0,v.width,v.height);c.fillStyle='#272a2e';c.fillRect(0,0,v.width,v.height);
    if(this.grid&&this.image&&this.layers.map){
      const g=this.grid,[x,y]=worldToScreen(g.origin,v);
      c.save();c.translate(x,y);c.scale(v.scale,-v.scale);c.rotate(g.origin[2]);c.translate(0,g.height*g.resolution);c.scale(g.resolution,-g.resolution);c.imageSmoothingEnabled=false;c.drawImage(this.image,0,0);c.restore();
    }
    if(this.layers.grid){
      const [xmin,ymax]=screenToWorld([0,0],v),[xmax,ymin]=screenToWorld([v.width,v.height],v);
      const step=v.scale>=25?1:v.scale>=5?5:10;
      for(let x=Math.ceil(xmin/step)*step;x<xmax;x+=step)this.line([[x,ymin],[x,ymax]],'#71809750',1);
      for(let y=Math.ceil(ymin/step)*step;y<ymax;y+=step)this.line([[xmin,y],[xmax,y]],'#71809750',1);
    }
    if(this.layers.live){c.fillStyle='#d52d69';for(const point of this.data.world_points||[]){const [x,y]=worldToScreen(point,v);if(x>=0&&y>=0&&x<v.width&&y<v.height)c.fillRect(x-1,y-1,2,2)}}
    if(this.layers.global)this.line(this.data.global_path,'#1a5fb4',3);
    if(this.layers.local)this.line(this.data.local_path,'#19d900',3);
    for(const p of this.points){
      const [x,y]=worldToScreen(p.position,v),color=p.type==='red'?'#f64d5c':'#368fff';
      this.arrow(p.position,p.yaw,color);
      c.beginPath();c.arc(x,y,p.selected?11:9,0,Math.PI*2);c.fillStyle=color;c.fill();c.lineWidth=p.selected?3:2;c.strokeStyle=p.selected?'#fff3ad':'#102333';c.stroke();
      c.font='bold 11px system-ui';c.fillStyle='#fff';c.textAlign='center';c.textBaseline='middle';c.fillText(String(p.index+1),x,y);c.textAlign='start';c.textBaseline='alphabetic';
      this.label(p.label,x+15,y-17,color);
    }
    if(this.validPose()){
      const pose=this.data.pose,[x,y]=worldToScreen(pose,v),fresh=this.data.pose_age_s!=null&&this.data.pose_age_s<2;
      c.save();c.globalAlpha=fresh?1:.45;
      this.line((this.data.robot_footprint||[]).map(p=>bodyToWorld(p,pose)),'#29e55f',2,true,'#2196ff40');
      this.arrow(pose,pose[3],'#50adff',Math.max(27,.4*v.scale));
      c.beginPath();c.arc(x,y,5,0,Math.PI*2);c.fillStyle='#087be6';c.fill();c.restore();
      this.label(fresh?'机器人':'机器人（最后位姿）',x+12,y+27,'#6fc3ff');
    }
    if(this.drag&&!this.drag.pan&&['pick','initial'].includes(this.drag.tool)&&this.drag.end){
      this.line([this.drag.start,this.drag.end],'#e543e5',2);this.arrow(this.drag.start,headingBetween(this.drag.start,this.drag.end),'#f16cec');
    }
    if(this.seed){this.arrow(this.seed.position,this.seed.yaw,'#ed8bff');const [x,y]=worldToScreen(this.seed.position,v);this.label('初始位姿草稿',x+12,y-12,'#ed8bff')}
    if(this.measure.length){
      this.line(this.measure,'#eeda38',2);for(const p of this.measure){const [x,y]=worldToScreen(p,v);c.beginPath();c.arc(x,y,5,0,Math.PI*2);c.fillStyle='#eeda38';c.fill()}
      if(this.measure.length===2){const [a,b]=this.measure,[x,y]=worldToScreen([(a[0]+b[0])/2,(a[1]+b[1])/2],v);this.label(`${Math.hypot(b[0]-a[0],b[1]-a[1]).toFixed(3)} m`,x+8,y-8,'#eeda38')}
    }
    const desired=100/v.scale,power=10**Math.floor(Math.log10(desired));
    const metres=[1,2,5,10].map(n=>n*power).find(n=>n>=desired)||10*power;
    const length=metres*v.scale;c.strokeStyle='#fff';c.lineWidth=2;c.beginPath();c.moveTo(18,v.height-34);c.lineTo(18+length,v.height-34);c.stroke();this.label(`${metres.toPrecision(2)*1} m`,18,v.height-43);
    if(!this.grid)this.label('正在读取 AL 的 2D 地图…',20,v.height/2);
    this.canvas.dataset.viewCenter=`${v.x.toFixed(3)},${v.y.toFixed(3)}`;
  }
}
