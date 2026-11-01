import test from 'node:test';
import assert from 'node:assert/strict';
import {normalizeMission,newPoint,mapPoints} from '../../web/waypoint_model.js';

test('legacy missions keep first red; explicit blue first is never changed',()=>{
  const original={frame_id:'map',points:[newPoint('blue',0),{name:'P2',position:[1,2,0],yaw:0}]};
  const normalized=normalizeMission(original);
  assert.deepEqual(normalized.points.map(p=>p.type),['blue','blue']);
  normalized.points[0].position[0]=10;assert.equal(original.points[0].position[0],null);
  delete original.points[0].type;assert.equal(normalizeMission(original).points[0].type,'red');
});
test('continuous red/blue clicks and replacements preserve independent coordinates and yaw',()=>{
  const a=[1,2,0];const blue=newPoint('blue',0,a);const red=newPoint('red',1,[3,4,0],Math.PI/2);
  a[0]=90;assert.equal(blue.position[0],1);assert.equal(blue.yaw,0);
  const draft={points:[blue,newPoint('blue',1),red]};
  const points=mapPoints(draft,2);
  assert.deepEqual(points.map(p=>[p.index,p.type,p.selected]),[[0,'blue',false],[2,'red',true]]);
  assert.equal(points[1].yaw,Math.PI/2);
  draft.points.reverse();assert.equal(mapPoints(draft,0)[0].type,'red');
});
