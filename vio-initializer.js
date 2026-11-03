/* PAI-VIN V3.3: stationary IMU initialization + bias-corrected attitude.
 * DeviceMotion samples are browser-coordinate, not camera calibrated.
 * This is the inertial orientation component of a future VIO state:
 * no metric translation, no visual update and no absolute heading. */
(function(root,factory){
 const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;
 else root.PAIVioInit=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const finite=Number.isFinite;
 const norm=a=>Math.hypot(...a);
 const scale=(a,k)=>a.map(v=>v*k);
 const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
 const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
 const unit=a=>scale(a,1/(norm(a)||1));
 const qMul=(a,b)=>[
  a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
  a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
  a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
  a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]
 ];
 function qRotate(q,v){
  const p=qMul(q,[0,...v]),r=qMul(p,[q[0],-q[1],-q[2],-q[3]]);
  return r.slice(1);
 }
 function qUnit(q){return scale(q,1/(norm(q)||1))}
 function qBetween(a,b){
  const u=unit(a),v=unit(b),d=dot(u,v);
  if(d<-.99999){
   const axis=unit(cross(u,Math.abs(u[0])<.9?[1,0,0]:[0,1,0]));
   return [0,...axis];
  }
  return qUnit([1+d,...cross(u,v)]);
 }
 function mean(v){return v.reduce((a,b)=>a+b,0)/v.length}
 function variance(v,m){return mean(v.map(x=>(x-m)*(x-m)))}
 function initialize(samples,opts={}){
  const minSamples=opts.minSamples??80;
  const durationMs=opts.durationMs??1500;
  const maxGyroStd=opts.maxGyroStd??.035; // rad/s
  const maxAccelStd=opts.maxAccelStd??.24; // m/s²
  if(!Array.isArray(samples)||samples.length<minSamples)return {ok:false,reason:'not_enough_samples',count:samples?.length||0};
  const first=samples[0],last=samples[samples.length-1];
  if(!first||!last||!finite(first.t)||!finite(last.t)||last.t-first.t<durationMs)
   return {ok:false,reason:'not_enough_time',durationMs:last?.t-first?.t};
  let maxGap=0;
  for(let i=0;i<samples.length;i++){
   const s=samples[i];
   if(!s||![s.t,s.gx,s.gy,s.gz,s.ax,s.ay,s.az].every(finite))return {ok:false,reason:'invalid_imu'};
   if(i){const delta=s.t-samples[i-1].t;if(delta<=0)return {ok:false,reason:'time_order'};maxGap=Math.max(maxGap,delta)}
  }
  if(maxGap>100)return {ok:false,reason:'imu_gap',maxGapMs:maxGap};
  const gyro=[mean(samples.map(s=>s.gx)),mean(samples.map(s=>s.gy)),mean(samples.map(s=>s.gz))];
  const acceleration=[mean(samples.map(s=>s.ax)),mean(samples.map(s=>s.ay)),mean(samples.map(s=>s.az))];
  const gyroStd=Math.sqrt([0,1,2].map((i)=>{
   const v=samples.map(s=>[s.gx,s.gy,s.gz][i]);return variance(v,gyro[i])
  }).reduce((a,b)=>a+b,0));
  const accelStd=Math.sqrt([0,1,2].map(i=>{
   const v=samples.map(s=>[s.ax,s.ay,s.az][i]);return variance(v,acceleration[i])
  }).reduce((a,b)=>a+b,0));
  if(gyroStd>maxGyroStd||accelStd>maxAccelStd)return {ok:false,reason:'device_moving',gyroStd,accelStd};
  if(norm(gyro)>.12)return {ok:false,reason:'high_mean_gyro',gyroMagnitude:norm(gyro)};
  const g=norm(acceleration);
  if(g<8.5||g>11)return {ok:false,reason:'invalid_gravity',gravityMagnitude:g};
  // Align measured gravity to +Z. This defines only tilt; yaw is arbitrary.
  const q=qBetween(acceleration,[0,0,g]);
  return {ok:true,gyroBias:gyro,gravityDevice:acceleration,gravityMagnitude:g,
    initialAttitude:q,gyroStd,accelStd,maxGapMs:maxGap,
    yawObservable:false,accelerometerBiasObservable:false,
    cameraIMUExtrinsicsKnown:false,metricPositionKnown:false};
 }
 class Attitude{
  constructor(initial){if(!initial?.ok)throw Error('stationary initialization required');
   this.bias=initial.gyroBias.slice();this.q=initial.initialAttitude.slice();
   this.last=null;this.samples=0;this.status='initialized';
  }
  push(s){
   if(!s||![s.t,s.gx,s.gy,s.gz].every(finite))return {ok:false,reason:'invalid_imu'};
   if(!this.last){this.last={...s};return {ok:true,reason:'first_sample',q:this.q.slice()}}
   const delta=s.t-this.last.t;
   if(delta<=0||delta>100){this.status='gap';this.last={...s};return {ok:false,reason:'imu_gap_or_reordering'}}
   const dt=delta/1000;
   const w=[(s.gx+this.last.gx)/2-this.bias[0],(s.gy+this.last.gy)/2-this.bias[1],(s.gz+this.last.gz)/2-this.bias[2]];
   const mag=norm(w),angle=mag*dt,half=angle/2;
   const dq=mag>1e-10?[Math.cos(half),...scale(w,Math.sin(half)/mag)]:[1,0,0,0];
   this.q=qUnit(qMul(this.q,dq));this.last={...s};this.samples++;this.status='propagating';
   return {ok:true,q:this.q.slice(),dt,angularRate:mag,source:'imu_only'};
  }
  snapshot(){return {initialized:true,quaternion:this.q.slice(),samples:this.samples,status:this.status,
   gyroBias:this.bias.slice(),visualCorrectionApplied:false,
   yawObservable:false,metricPosition:null,metricVelocity:null,covarianceEstimated:false}}
 }
 return {initialize,Attitude,qMul,qRotate,qBetween};
});