/* PAI-VIN local gyro/vision consistency diagnostics.
 * DeviceMotion rotationRate is degrees/second in device axes, not camera axes.
 * Magnitude integrates angular travel without claiming calibrated extrinsics.
 * No linear acceleration integration; no metric scale or 6DoF fusion. */
(function(root,factory){
 const m=factory();if(typeof module==='object'&&module.exports)module.exports=m;
 else root.VINInertial=m;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const MAX_AGE=600,MAX_GAP=180;
 class GyroBuffer {
  constructor(){this.samples=[];this.lastT=-Infinity}
  push(t,rate){
   if(!Number.isFinite(t)||t<=this.lastT||!rate)return false;
   const v=[rate.alpha,rate.beta,rate.gamma];
   if(!v.every(Number.isFinite))return false;
   const magnitude=Math.hypot(...v);
   if(magnitude>1500)return false;
   this.samples.push({t,magnitude});this.lastT=t;
   while(this.samples.length&&this.samples[0].t<t-10000)this.samples.shift();
   return true;
  }
  interval(start,end){
   if(!Number.isFinite(start)||!Number.isFinite(end)||end<=start)return {ok:false,reason:'invalid_interval'};
   const points=this.samples.filter(s=>s.t>=start-MAX_GAP&&s.t<=end+MAX_GAP);
   if(points.length<2)return {ok:false,reason:'no_gyro_samples'};
   if(points[0].t>start+MAX_GAP||points[points.length-1].t<end-MAX_GAP)return {ok:false,reason:'incomplete_gyro_window'};
   let angle=0,covered=0;
   for(let i=1;i<points.length;i++){
    const left=Math.max(start,points[i-1].t),right=Math.min(end,points[i].t);
    if(right<=left)continue;
    if(points[i].t-points[i-1].t>MAX_GAP)return {ok:false,reason:'sensor_gap'};
    angle+=(points[i-1].magnitude+points[i].magnitude)*.5*(right-left)/1000;
    covered+=right-left;
   }
   if(covered<Math.max(0,end-start-2*MAX_GAP))return {ok:false,reason:'insufficient_coverage'};
   return {ok:true,angleDeg:angle,coverage:covered/(end-start),sampleCount:points.length};
  }
  fresh(now){return Number.isFinite(this.lastT)&&now>=this.lastT&&now-this.lastT<MAX_AGE}
  reset(){this.samples=[];this.lastT=-Infinity}
 }
 function compare(visualDeg,gyro){
  if(!gyro?.ok||!Number.isFinite(visualDeg)||visualDeg<0)return {status:'unknown',reason:gyro?.reason||'invalid_visual'};
  const diff=Math.abs(visualDeg-gyro.angleDeg);
  // Very broad check: magnitude-only; camera-device calibration is unavailable.
  const tolerance=Math.max(12,gyro.angleDeg*.85);
  return {status:diff<=tolerance?'consistent':'disagree',diffDeg:diff,toleranceDeg:tolerance,gyroAngleDeg:gyro.angleDeg,visualDeg,
   note:'magnitude_only_not_metric_fusion'};
 }
 return {GyroBuffer,compare};
});