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
function flowStatistics(matches,width=256,height=192){
 if(!Array.isArray(matches)||!matches.length)return {count:0,flowMedianPx:null,flowResidualMedianPx:null,occupiedQuadrants:0,translationLike:false,spatiallyVariable:false};
 const clean=matches.filter(m=>[m.px,m.py,m.x,m.y].every(Number.isFinite));
 if(!clean.length)return {count:0,flowMedianPx:null,flowResidualMedianPx:null,occupiedQuadrants:0,translationLike:false,spatiallyVariable:false};
 const med=values=>{const a=values.slice().sort((x,y)=>x-y),n=a.length;return n%2?a[n>>1]:(a[n/2-1]+a[n/2])/2};
 const dx=med(clean.map(p=>p.x-p.px)),dy=med(clean.map(p=>p.y-p.py));
 const residual=med(clean.map(p=>Math.hypot(p.x-p.px-dx,p.y-p.py-dy)));
 const q=new Set(clean.map(p=>(p.px>=width/2?1:0)+(p.py>=height/2?2:0)));
 const amplitude=Math.hypot(dx,dy);
 // This is image-plane coherence, NOT a 3D translation or parallax estimate.
 return {count:clean.length,flowMedianPx:Math.round(amplitude*100)/100,
  flowResidualMedianPx:Math.round(residual*100)/100,occupiedQuadrants:q.size,
  translationLike:clean.length>=18&&q.size>=3&&amplitude>2&&residual<Math.max(1.5,amplitude*.3),
  spatiallyVariable:clean.length>=18&&q.size>=3&&residual>Math.max(2.5,amplitude*.55),
  metricTranslation:null,parallaxConfirmed:false};
}
function assess({tracks,previousTime,currentTime,sensorBuffer,rotationEstimator,intrinsics,width=256,height=192,visualOverride,flowOverride}){
 if(!Number.isFinite(previousTime)||!Number.isFinite(currentTime)||currentTime<=previousTime)
  return {ok:false,reason:'invalid_frame_timing'};
 const pairs=tracksToPairs(tracks);
 const intr=intrinsics||nominalIntrinsics(width,height);
 const visual=visualOverride||rotationEstimator.estimate(pairs,intr);
 const flow=flowOverride||flowStatistics(pairs,width,height);
 const window=sensorBuffer.between(previousTime,currentTime);
 const seconds=(currentTime-previousTime)/1000;
 const avgRate=window.ok?window.samples.reduce((s,x)=>s+Math.hypot(x.gx,x.gy,x.gz),0)/window.samples.length:null;
 const result={ok:Boolean(visual.ok),reason:visual.ok?'visual_rotation_observed':visual.reason,
  visualMatchCount:visualOverride?(visual.total??visual.count??pairs.length):pairs.length,visualInliers:visual.inliers||0,
  geometryFailureDetail:visual.geometryFailureDetail??null,
  flowDiagnostic:flow,
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
return {tracksToPairs,nominalIntrinsics,flowStatistics,assess};
});