/* PAI-VIN Map V1 envelope validation: no pose or point cloud synthesis.
 * This validator is deliberately not a SLAM runtime or localization API.
 */
(function(root,make){const api=make();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIV4MapContract=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const SCHEMA='pai-vin-metric-visual-map-v1';
function positiveInteger(v){return Number.isSafeInteger(v)&&v>0;}
function finite3(p){return Array.isArray(p)&&p.length===3&&p.every(Number.isFinite);}
function finite16(m){return Array.isArray(m)&&m.length===16&&m.every(Number.isFinite)&&Math.abs(m[15]-1)<1e-4;}
function validate(map){
 const errors=[];
 if(!map||typeof map!=='object'||Array.isArray(map))return {ok:false,errors:['not_object']};
 if(map.schema!==SCHEMA)errors.push('schema_version_mismatch');
 if(typeof map.mapId!=='string'||!map.mapId.trim())errors.push('map_id_required');
 if(!map.coordinateSystem||map.coordinateSystem.units!=='meters'||map.coordinateSystem.poseConvention!=='camera_to_map')errors.push('metric_coordinate_convention_missing');
 if(!map.scale||map.scale.verified!==true||typeof map.scale.method!=='string'||!map.scale.method.trim()||!(map.scale.relativeUncertainty>=0&&Number.isFinite(map.scale.relativeUncertainty)))errors.push('verified_scale_provenance_required');
 if(!Array.isArray(map.cameras)||!map.cameras.length)errors.push('camera_models_missing');
 else for(const c of map.cameras){if(!c||typeof c.id!=='string'||!positiveInteger(c.width)||!positiveInteger(c.height)||![c.fx,c.fy,c.cx,c.cy].every(Number.isFinite)||c.fx<=0||c.fy<=0)errors.push('invalid_camera_model');}
 if(!Array.isArray(map.keyframes)||map.keyframes.length<2)errors.push('too_few_keyframes');
 if(!Array.isArray(map.landmarks)||map.landmarks.length<4)errors.push('too_few_landmarks');
 if(errors.length)return {ok:false,errors:[...new Set(errors)]};
 const camIDs=new Set(map.cameras.map(c=>c.id)),kfIDs=new Set(),lmIDs=new Set();
 for(const f of map.keyframes){
  if(!f||typeof f.id!=='string'||kfIDs.has(f.id)||!camIDs.has(f.cameraId)||!finite16(f.poseCameraToMap))errors.push('invalid_keyframe');
  else kfIDs.add(f.id);
 }
 for(const l of map.landmarks){
  if(!l||typeof l.id!=='string'||lmIDs.has(l.id)||!finite3(l.xyzMeters)||typeof l.descriptorType!=='string'||!l.descriptorType||
    !Array.isArray(l.descriptor)||l.descriptor.length===0||!l.descriptor.every(Number.isFinite))errors.push('invalid_landmark');
  else lmIDs.add(l.id);
 }
 if(!Array.isArray(map.observations)||map.observations.length<8)errors.push('insufficient_observation_graph');
 else{
  const seen=new Set();
  for(const o of map.observations){
   const id=(o?.keyframeId||'')+':'+(o?.landmarkId||'');
   if(!o||!kfIDs.has(o.keyframeId)||!lmIDs.has(o.landmarkId)||!Number.isFinite(o.x)||!Number.isFinite(o.y)||seen.has(id))errors.push('invalid_observation_edge');
   seen.add(id);
  }
 }
 if(!map.relocalization||map.relocalization.indexReady!==true||typeof map.relocalization.descriptorType!=='string')errors.push('relocalization_index_unavailable');
 else if(map.landmarks.some(l=>l.descriptorType!==map.relocalization.descriptorType))errors.push('descriptor_type_mismatch');
 return {ok:errors.length===0,errors:[...new Set(errors)]};
}
return {SCHEMA,validate};
});