/* PAI-VIN: calibrated visual orientation correction contract.
 * A visual rotation R_currentCamera_from_prevCamera can constrain integrated
 * IMU orientation ONLY with verified camera intrinsics, body/camera extrinsics
 * and time alignment. No position, velocity, scale or pose-graph inference. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioFusion=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const unit=a=>{const n=Math.hypot(...a);return n?a.map(x=>x/n):null};
const mul=(a,b)=>[
 a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
 a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
 a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
 a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];
const conj=q=>[q[0],-q[1],-q[2],-q[3]];
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
function matrixQ(R){
 if(!Array.isArray(R)||R.length!==3||R.some(r=>!Array.isArray(r)||r.length!==3||r.some(x=>!Number.isFinite(x))))return null;
 const tr=R[0][0]+R[1][1]+R[2][2];let w,x,y,z;
 if(tr>0){const s=Math.sqrt(tr+1)*2;w=s/4;x=(R[2][1]-R[1][2])/s;y=(R[0][2]-R[2][0])/s;z=(R[1][0]-R[0][1])/s}
 else if(R[0][0]>R[1][1]&&R[0][0]>R[2][2]){const s=Math.sqrt(1+R[0][0]-R[1][1]-R[2][2])*2;w=(R[2][1]-R[1][2])/s;x=s/4;y=(R[0][1]+R[1][0])/s;z=(R[0][2]+R[2][0])/s}
 else if(R[1][1]>R[2][2]){const s=Math.sqrt(1+R[1][1]-R[0][0]-R[2][2])*2;w=(R[0][2]-R[2][0])/s;x=(R[0][1]+R[1][0])/s;y=s/4;z=(R[1][2]+R[2][1])/s}
 else {const s=Math.sqrt(1+R[2][2]-R[0][0]-R[1][1])*2;w=(R[1][0]-R[0][1])/s;x=(R[0][2]+R[2][0])/s;y=(R[1][2]+R[2][1])/s;z=s/4}
 return unit([w,x,y,z]);
}
function angular(a,b){return 2*Math.acos(Math.min(1,Math.abs(dot(a,b))))}
function slerp(a,b,t){
 if(dot(a,b)<0)b=b.map(x=>-x);
 let theta=Math.acos(Math.min(1,Math.max(-1,dot(a,b))));
 if(theta<1e-6)return unit(a.map((v,i)=>v*(1-t)+b[i]*t));
 const s=Math.sin(theta);return unit(a.map((v,i)=>(v*Math.sin((1-t)*theta)+b[i]*Math.sin(t*theta))/s));
}
function correct({previousQuaternion,predictedQuaternion,vision,calibration,gain=.15}){
 if(!calibration?.verified||!calibration.cameraIntrinsicsVerified||!calibration.cameraIMUExtrinsicsVerified||!calibration.timeOffsetVerified)
  return {ok:false,reason:'uncalibrated_camera_imu'};
 if(!vision?.ok||vision.inliers<18||vision.ratio<.72)return {ok:false,reason:'visual_rotation_unreliable'};
 const qPrev=unit(previousQuaternion||[]),qPred=unit(predictedQuaternion||[]);
 const qBC=unit(calibration.bodyFromCameraQuaternion||[]);
 const qVisual=matrixQ(vision.rotationCameraCurrentFromPrevious);
 if(!qPrev||!qPred||qPrev.length!==4||qPred.length!==4||!qBC||qBC.length!==4||!qVisual)
  return {ok:false,reason:'invalid_rotation'};
 if(gain<0||gain>.35)return {ok:false,reason:'invalid_gain'};
 // camera current<-previous -> body current<-previous -> world<-current.
 const qBody=mul(mul(qBC,qVisual),conj(qBC));
 const visualWorldCurrent=unit(mul(qPrev,conj(qBody)));
 const residual=angular(qPred,visualWorldCurrent);
 if(residual>.16)return {ok:false,reason:'visual_imu_disagreement',residualRad:residual};
 const corrected=slerp(qPred,visualWorldCurrent,gain);
 return {ok:true,quaternion:corrected,residualRad:residual,
  source:'calibrated_visual_rotation_correction',visualInliers:vision.inliers,
  metricPosition:null,metricVelocity:null,scaleObservable:false};
}
return {correct,matrixQ,angular,slerp};
});