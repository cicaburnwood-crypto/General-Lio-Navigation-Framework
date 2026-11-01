// Shared, pure draft helpers. No storage and no ROS/HTTP side effects.
export function normalizeMission(value){
  const result=structuredClone(value);
  result.navigation={xy_tolerance:.05,yaw_tolerance:.087,timeout_sec:120,...result.navigation};
  result.door={settle_seconds:1,linear_stopped_mps:.02,angular_stopped_radps:.03,observation_max_age_sec:.5,...result.door};
  result.points=(result.points||[]).map((p,i)=>({...p,type:p.type??(i===0?'red':'blue')}));
  return result;
}
export function newPoint(type,index,position=[null,null,0],yaw=0){
  return {name:`P${index+1}`,type,position:[...position],yaw:Number.isFinite(yaw)?yaw:0,inspection_duration_sec:0};
}
export function mapPoints(mission,selected){
  return (mission?.points||[]).map((p,i)=>({position:p.position,yaw:p.yaw,type:p.type,index:i,
    label:`${i+1} ${p.name}`,selected:i===selected})).filter(p=>p.position.length===3&&p.position.every(Number.isFinite));
}
