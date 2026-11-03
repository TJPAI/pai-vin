const assert=require('node:assert/strict');
const O=require('../vio-observation.js'),V=require('../vio-rotation.js');
const K=O.nominalIntrinsics(256,192);
assert.equal(K.verified,false);assert.equal(K.fx,256*.94);
const R=V.expSO3([0,.035,0]),m=(A,x)=>A.map(row=>row.reduce((s,z,i)=>s+z*x[i],0));
const matches=[];
for(let y=20;y<175;y+=17)for(let x=15;x<240;x+=17){
 const ray=[(x-K.cx)/K.fx,(y-K.cy)/K.fy,1],p=m(R,ray);
 const xx=K.cx+K.fx*p[0]/p[2],yy=K.cy+K.fy*p[1]/p[2];
 matches.push({id:matches.length+1,age:7,x:xx,y:yy,dx:xx-x,dy:yy-y});
}
const fakeBuf={between(t0,t1){return {ok:true,samples:Array.from({length:10},(_,i)=>({t:t0+10*i,gx:0,gy:.2,gz:0}))}}};
const r=O.assess({tracks:{matches},previousTime:200,currentTime:300,sensorBuffer:fakeBuf,rotationEstimator:V});
assert.equal(r.ok,true,JSON.stringify(r));assert.ok(r.visualInliers>40);
assert.ok(r.cameraRelativeAngleDeg>1&&r.cameraRelativeAngleDeg<3);
assert.equal(r.attitudeFused,false);
assert.equal(r.cameraIntrinsicsVerified,false);
assert.equal(r.metricPosition,null);
assert.equal(O.assess({tracks:{matches},previousTime:300,currentTime:200,sensorBuffer:fakeBuf,rotationEstimator:V}).reason,'invalid_frame_timing');
const no=O.assess({tracks:{matches:[]},previousTime:200,currentTime:300,sensorBuffer:fakeBuf,rotationEstimator:V});
assert.equal(no.reason,'insufficient_tracks');
const missing={between(){return {ok:false,reason:'missing_imu'}}};
const gy=O.assess({tracks:{matches},previousTime:200,currentTime:300,sensorBuffer:missing,rotationEstimator:V});
assert.equal(gy.ok,true);assert.equal(gy.imuWindowOk,false);
const rejected=O.assess({tracks:{matches:[]},previousTime:200,currentTime:300,
 sensorBuffer:fakeBuf,rotationEstimator:V,
 visualOverride:{ok:false,reason:'rotation_geometry_unreliable',inliers:25,ratio:.6,total:42}});
assert.equal(rejected.visualMatchCount,42,'rejected Worker rotation must report actual matches');
assert.equal(rejected.visualInliers,25);
const rejectedLegacy=O.assess({tracks:{matches:[]},previousTime:200,currentTime:300,
 sensorBuffer:fakeBuf,rotationEstimator:V,
 visualOverride:{ok:false,reason:'insufficient_tracks',count:7}});
assert.equal(rejectedLegacy.visualMatchCount,7);
const coherent=O.flowStatistics(Array.from({length:32},(_,i)=>({px:16+(i%8)*24,py:20+Math.floor(i/8)*40,x:22+(i%8)*24,y:24+Math.floor(i/8)*40})));
assert.equal(coherent.count,32);assert.equal(coherent.translationLike,true);assert.equal(coherent.parallaxConfirmed,false);
assert.equal(coherent.metricTranslation,null);
const scattered=O.flowStatistics(Array.from({length:32},(_,i)=>({px:16+(i%8)*24,py:20+Math.floor(i/8)*40,
 x:16+(i%8)*24+(i%2?14:-14),y:20+Math.floor(i/8)*40+(i%3?8:-8)})));
assert.equal(scattered.spatiallyVariable,true);
const poor=O.flowStatistics([{px:1,py:2,x:2,y:3}]);
assert.equal(poor.translationLike,false);
assert.equal(O.flowStatistics([]).count,0);
const classified=V.estimate(matches.slice(0,24).map((p,i)=>i<16?{...p,x:p.x+60,y:p.y-50}:p).map(p=>({id:p.id,px:p.x-p.dx,py:p.y-p.dy,x:p.x,y:p.y})),K);
assert.equal(classified.ok,false);
assert.ok(['rotation_geometry_unreliable','insufficient_tracks'].includes(classified.reason));
console.log('VIO image-track/IMU observation diagnostic tests PASS');
