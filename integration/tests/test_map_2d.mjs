import test from 'node:test';
import assert from 'node:assert/strict';
import {worldToScreen,screenToWorld,pixelToWorld,mapBounds,headingBetween,bodyToWorld} from '../../web/map_2d.js';

const close=(a,b)=>a.forEach((n,i)=>assert.ok(Math.abs(n-b[i])<1e-9,`${a} != ${b}`));
test('map XY screen conversion round trips at negative coordinates and varying scales',()=>{
  for(const scale of [.5,8,50,1000]){
    const view={width:950,height:700,x:-7.098,y:-37.638,scale};
    for(const p of [[0,0],[-7.098,-37.638],[-13.6,-41.5],[25,39]])close(screenToWorld(worldToScreen(p,view),view),p);
    close(worldToScreen([view.x,view.y],view),[475,350]);
    assert.ok(worldToScreen([view.x,view.y+1],view)[1]<350);
  }
});
test('ROS image uses bottom-left origin and flipped raster rows',()=>{
  const g={width:10,height:20,resolution:.05,origin:[-13.6,-41.5,0]};
  close(pixelToWorld([0,20],g),[-13.6,-41.5]);
  close(pixelToWorld([10,0],g),[-13.1,-40.5]);
  close(mapBounds(g),[-13.6,-41.5,-13.1,-40.5]);
});
test('rotated map origin is respected',()=>{
  const g={width:10,height:20,resolution:.05,origin:[1,2,Math.PI/2]};
  close(pixelToWorld([10,0],g),[0,2.5]);close(mapBounds(g),[0,2,1,2.5]);
});
test('dragging upward gives +90 degree yaw, not mirrored',()=>{
  const v={width:800,height:600,x:0,y:0,scale:40};
  close([headingBetween(screenToWorld([400,300],v),screenToWorld([400,200],v))],[Math.PI/2]);
});
test('footprint transforms using robot yaw and not camera axes',()=>{
  close(bodyToWorld([.28,.18],[-7,-37,0,Math.PI/2]),[-7.18,-36.72]);
});
