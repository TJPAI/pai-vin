const assert=require('node:assert/strict');const {SCHEMA,validate}=require('../v4-map-contract.js');
const I=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
const map={schema:SCHEMA,mapId:'floor-1',coordinateSystem:{units:'meters',poseConvention:'camera_to_map'},
scale:{verified:true,method:'a4-linked-constrained-BA',relativeUncertainty:.01},
cameras:[{id:'camera1',width:360,height:480,fx:320,fy:320,cx:180,cy:240}],
keyframes:[{id:'f0',cameraId:'camera1',poseCameraToMap:I},{id:'f1',cameraId:'camera1',poseCameraToMap:I}],
landmarks:Array.from({length:4},(_,i)=>({id:'p'+i,xyzMeters:[i,0,2],descriptorType:'orb',descriptor:[i,2,3]})),
observations:Array.from({length:8},(_,i)=>({keyframeId:'f'+Math.floor(i/4),landmarkId:'p'+i%4,x:20+i,y:30+i})),
relocalization:{indexReady:true,descriptorType:'orb'}};
assert.equal(validate(map).ok,true);
for(const [f,k] of [[m=>m.scale.verified=false,'verified_scale_provenance_required'],
[m=>m.coordinateSystem.units='arbitrary','metric_coordinate_convention_missing'],
[m=>m.relocalization.indexReady=false,'relocalization_index_unavailable'],
[m=>m.landmarks[0].descriptorType='alike','descriptor_type_mismatch'],
[m=>m.observations[0].keyframeId='missing','invalid_observation_edge']]){
 const m=structuredClone(map);f(m);assert.ok(validate(m).errors.includes(k),k);
}
assert.equal(validate({schema:SCHEMA}).ok,false);
console.log('PAI-VIN Map V1 conservative protocol contract tests PASS');