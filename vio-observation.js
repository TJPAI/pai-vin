/* PAI-VIN V3.6 visual / inertial observation consistency diagnostic.
 * Real optical tracks enter an estimated camera relative rotation solver.
 * Intrinsics below are only nominal, so output is NOT an extrinsic/time
 * calibrated camera-IMU fusion measurement or a map pose. */
(function(root,factory){
 const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;
 else root.PAIVioObserve=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
function tracksToPairs(trackReport){
 const matches=trackReport?.matches;
 if(!Array.isArray(matches))return [];
 return matches.filter(p=>p.age>=2&&[p.x,p.y,p.dx,p.dy].every(Number.isFinite))
  .map(p=>({id:p.id,px:p.x-p.dx,py:p.y-p.dy,x:p.x,y:p.y}));
}
function nominalIntrinsics(width,height){
 if(!Number.isFinite(width)||!Number.isFinite(height)||width<=0||height<=0)return null;
 // Diagnostic ONLY, not measured camera intrinsics.
 return {fx:.94*width,fy:.94*width,cx:width/2,cy:height/2,verified:false,source:'nominal_diagnostic_only'};
}
function assess({tracks,previousTime,currentTime,sensorBuffer,rotationEstimator,intrinsics,width=256,height=192,visualOverride}){
 if(!Number.isFinite(previousTime)||!Number.isFinite(currentTime)||currentTime<=previousTime)
  return {ok:false,reason:'invalid_frame_timing'};
 const pairs=tracksToPairs(tracks);
 const intr=intrinsics||nominalIntrinsics(width,height);
 const visual=visualOverride||rotationEstimator.estimate(pairs,intr);
 const window=sensorBuffer.between(previousTime,currentTime);
 const seconds=(currentTime-previousTime)/1000;
 const avgRate=window.ok?window.samples.reduce((s,x)=>s+Math.hypot(x.gx,x.gy,x.gz),0)/window.samples.length:null;
 const result={ok:Boolean(visual.ok),reason:visual.ok?'visual_rotation_observed':visual.reason,
  visualMatchCount:visualOverride?(visual.total??visual.count??pairs.length):pairs.length,visualInliers:visual.inliers||0,
  visualInlierRatio:visual.ratio??null,
  cameraRelativeAngleDeg:visual.ok?visual.theta*180/Math.PI:null,
  visualMedianResidualDeg:visual.ok?visual.medianErrorRad*180/Math.PI:null,
  imuWindowOk:window.ok,imuWindowReason:window.ok?null:window.reason,
  imuAngularMagnitudeDeg:window.ok?avgRate*seconds*180/Math.PI:null,
  dtMs:currentTime-previousTime,
  cameraIntrinsicsVerified:intr.verified===true,
  cameraIMUExtrinsicsVerified:false,timeOffsetVerified:false,
  attitudeFused:false,metricPosition:null,source:'uncalibrated_visual_imu_diagnostic'};
 return result;
}
return {tracksToPairs,nominalIntrinsics,assess};
});