/* PAI-VIN V3.15: IMU preintegration factors, NOT camera position.
 * Gyro-corrected delta rotation + integrated specific force in initial device
 * frame. Gravity, accelerometer bias, extrinsics and exposure offset are
 * uncalibrated: deltaV/deltaP are NOT world displacement or metric pose. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioPreintegrate=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const norm=v=>Math.hypot(...v),add=(a,b)=>a.map((v,i)=>v+b[i]),scale=(a,k)=>a.map(x=>x*k);
const mul=(a,b)=>[
 a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
 a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
 a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
 a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];
function rotate(q,v){
 const t=mul(mul(q,[0,...v]),[q[0],-q[1],-q[2],-q[3]]);
 return t.slice(1);
}
function integrate(samples,options={}){
 if(!Array.isArray(samples)||samples.length<2)return {ok:false,reason:'missing_imu'};
 const bg=options.gyroBias||[0,0,0],ba=options.accelBias||[0,0,0];
 if(![...bg,...ba].every(Number.isFinite)||bg.length!==3||ba.length!==3)return {ok:false,reason:'invalid_bias'};
 let q=[1,0,0,0],dv=[0,0,0],dp=[0,0,0],dtTotal=0,maxGapMs=0;
 for(let i=1;i<samples.length;i++){
  const a=samples[i-1],b=samples[i];
  if(!a||!b||!['t','gx','gy','gz','ax','ay','az'].every(k=>Number.isFinite(a[k])&&Number.isFinite(b[k])))
   return {ok:false,reason:'invalid_imu'};
  const ms=b.t-a.t,dt=ms/1000;
  if(dt<=0||ms>100)return {ok:false,reason:'invalid_imu_gap',gapMs:ms};
  maxGapMs=Math.max(maxGapMs,ms);
  const omega=[(a.gx+b.gx)/2-bg[0],(a.gy+b.gy)/2-bg[1],(a.gz+b.gz)/2-bg[2]];
  const mag=norm(omega),half=mag*dt*.5,dq=mag>1e-12?[Math.cos(half),...scale(omega,Math.sin(half)/mag)]:[1,0,0,0];
  const qMid=mul(q,mag>1e-12?[Math.cos(half*.5),...scale(omega,Math.sin(half*.5)/mag)]:[1,0,0,0]);
  const accel=[(a.ax+b.ax)/2-ba[0],(a.ay+b.ay)/2-ba[1],(a.az+b.az)/2-ba[2]];
  const accelInitial=rotate(qMid,accel);
  dp=add(dp,add(scale(dv,dt),scale(accelInitial,dt*dt*.5)));
  dv=add(dv,scale(accelInitial,dt));
  q=mul(q,dq);q=scale(q,1/norm(q));
  dtTotal+=dt;
 }
 return {ok:true,dt:dtTotal,samples:samples.length,maxGapMs,
  deltaQuaternion:q,deltaVelocitySpecificForce:dv,deltaPositionSpecificForce:dp,
  coordinateFrame:'initial_device_axes',gravityCompensated:false,
  accelBiasCalibrated:false,cameraIMUExtrinsicsVerified:false,
  metricPoseAvailable:false,worldPosition:null,translationVerified:false};
}
return {integrate};
});