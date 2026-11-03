/* PAI-VIN VIO sensor transport — browser main-thread capture, no fake SLAM.
 * Camera frames + motion events share performance.now() monotonic clock.
 * Actual camera sensor exposure timestamps are not available via getUserMedia;
 * t is capture-observation time and needs calibrated camera–IMU offset.
 * API ready for a future WebAssembly VIO estimator. */
(function(root,factory){const m=factory();if(typeof module==='object'&&module.exports)module.exports=m;else root.PAIVioSensor=m;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const finite=x=>typeof x==='number'&&Number.isFinite(x);
const validateImu=(x)=>Boolean(x&&finite(x.t)&&finite(x.gx)&&finite(x.gy)&&finite(x.gz)&&finite(x.ax)&&finite(x.ay)&&finite(x.az));
class SensorBuffer{
 constructor(max=4096){this.max=max;this.imu=[];this.frames=[];this.dropped=0;this.duplicates=0;this.reordered=0;}
 addImu(s){
  if(!validateImu(s))return false;
  if(this.imu.length&&s.t<=this.imu[this.imu.length-1].t){if(s.t===this.imu[this.imu.length-1].t)this.duplicates++;else this.reordered++;return false}
  this.imu.push({...s});
  if(this.imu.length>this.max){this.imu.shift();this.dropped++}
  return true;
 }
 addFrame(f){
  if(!f||!finite(f.t)||!Number.isInteger(f.width)||!Number.isInteger(f.height)||f.width<1||f.height<1)return false;
  if(this.frames.length&&f.t<=this.frames[this.frames.length-1].t)return false;
  // Store no camera image here — this is the frame transport metadata.
  this.frames.push({t:f.t,width:f.width,height:f.height,features:f.features||0});
  if(this.frames.length>300)this.frames.shift();return true;
 }
 between(t0,t1){
  // Strict bracketing: never call nearest sample 'synchronized' and never
  // silently extrapolate a rotation through a main-thread sensor stall.
  if(!finite(t0)||!finite(t1)||t1<=t0)return {ok:false,reason:'invalid_window'};
  const imu=this.imu;
  if(imu.length<2)return {ok:false,reason:'missing_imu'};
  let lo=0,hi=imu.length;
  const lower=t=>{let l=0,h=imu.length;while(l<h){const m=(l+h)>>1;if(imu[m].t<t)l=m+1;else h=m;}return l};
  const begin=lower(t0),end=lower(t1);
  if(begin===imu.length||end===imu.length)
   return {ok:false,reason:'awaiting_imu_bracket',firstTime:imu[0].t,lastTime:imu[imu.length-1].t};
  if((begin===0&&imu[begin].t>t0)||(end===0&&imu[end].t>t1))
   return {ok:false,reason:'no_imu_before_frame',firstTime:imu[0].t};
  const l0=begin>0?imu[begin-1]:imu[begin],r0=imu[begin];
  const l1=end>0?imu[end-1]:imu[end],r1=imu[end];
  const ageBefore=t0-l0.t,ageAfter=r1.t-t1;
  const endpointGap=Math.max(r0.t-l0.t,r1.t-l1.t);
  if(endpointGap>100)return {ok:false,reason:'imu_boundary_gap',maxGapMs:endpointGap,ageBeforeMs:ageBefore,ageAfterMs:ageAfter};
  const inner=imu.slice(begin,end).filter(v=>v.t>t0&&v.t<t1);
  const lerp=(x,y,t)=>{
   if(x.t===y.t)return {...x,t};
   const f=(t-x.t)/(y.t-x.t),result={t};
   for(const k of ['gx','gy','gz','ax','ay','az'])result[k]=x[k]+f*(y[k]-x[k]);
   return result;
  };
  const samples=[lerp(l0,r0,t0),...inner,lerp(l1,r1,t1)];
  let maxGap=0;
  for(let i=1;i<samples.length;i++)maxGap=Math.max(maxGap,samples[i].t-samples[i-1].t);
  if(maxGap>100)return {ok:false,reason:'imu_internal_gap',maxGapMs:maxGap,
   ageBeforeMs:ageBefore,ageAfterMs:ageAfter};
  return {ok:true,samples,maxGapMs:maxGap,dt:(t1-t0)/1000,
   boundaryInterpolation:true,ageBeforeMs:ageBefore,ageAfterMs:ageAfter,
   samplingClock:'browser_receipt_not_exposure'};
 }
 reset(){this.imu=[];this.frames=[];this.dropped=0;this.duplicates=0;this.reordered=0;}
 report(){
  const rate=a=>a.length<2?0:1000*(a.length-1)/(a[a.length-1].t-a[0].t);
  return {imuSamples:this.imu.length,cameraFrames:this.frames.length,imuRateHz:rate(this.imu),cameraRateHz:rate(this.frames),dropped:this.dropped,duplicates:this.duplicates,reordered:this.reordered};
 }
}
function fromDeviceMotion(e,t){
 // DeviceMotionEvent rotationRate alpha/beta/gamma are reported in deg/s;
 // accelIncludingGravity is m/s² in device axes, not calibrated IMU axes.
 const r=e?.rotationRate,a=e?.accelerationIncludingGravity;
 if(!r||!a||![r.alpha,r.beta,r.gamma,a.x,a.y,a.z].every(finite))return null;
 const k=Math.PI/180;
 return {t,gx:r.beta*k,gy:r.gamma*k,gz:r.alpha*k,ax:a.x,ay:a.y,az:a.z,
  frame:'device_web',axesCalibrated:false,exposureTimeKnown:false};
}
function integrateGyro(samples,bias=[0,0,0]){
 if(!Array.isArray(samples)||samples.length<2)return {ok:false,reason:'missing_imu'};
 let q=[1,0,0,0],dtTotal=0;
 const mul=(a,b)=>[a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
 a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
 a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
 a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];
 for(let i=1;i<samples.length;i++){
  const a=samples[i-1],b=samples[i],dt=(b.t-a.t)/1000;
  if(!validateImu(a)||!validateImu(b)||dt<=0||dt>.1)return {ok:false,reason:'bad_timing'};
  const omega=[(a.gx+b.gx)/2-bias[0],(a.gy+b.gy)/2-bias[1],(a.gz+b.gz)/2-bias[2]];
  const mag=Math.hypot(...omega),angle=mag*dt,half=angle/2;
  const axis=mag>1e-10?omega.map(v=>v/mag):[0,0,0];
  q=mul(q,[Math.cos(half),...axis.map(v=>v*Math.sin(half))]);
  const len=Math.hypot(...q);q=q.map(v=>v/len);dtTotal+=dt;
 }
 return {ok:true,deltaQuaternion:q,dt:dtTotal,
  translationMeters:null,poseWorld:null,gyroBiasEstimated:false,gravityCompensated:false};
}
return {SensorBuffer,validateImu,fromDeviceMotion,integrateGyro};
});