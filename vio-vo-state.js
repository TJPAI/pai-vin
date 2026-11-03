/* PAI-VIN V3.28 camera orientation-only visual odometry state.
 * Inputs are geometrically validated relative camera rotations from
 * UNVERIFIED nominal intrinsics. Does NOT infer translation or world pose.
 * Discontinuities end a segment; never bridge a tracking outage.
 */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioVOState=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const I=()=>[[1,0,0],[0,1,0],[0,0,1]];
const transpose=A=>A[0].map((_,j)=>A.map(r=>r[j]));
const mul=(A,B)=>A.map(row=>B[0].map((_,j)=>row.reduce((s,x,k)=>s+x*B[k][j],0)));
const angle=R=>Math.acos(Math.max(-1,Math.min(1,(R[0][0]+R[1][1]+R[2][2]-1)/2)));
function validRotation(R){
 if(!Array.isArray(R)||R.length!==3||R.some(row=>!Array.isArray(row)||row.length!==3||row.some(x=>!Number.isFinite(x))))return false;
 const norm=mul(transpose(R),R),det=
 R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1])-
 R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0])+
 R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0]);
 return Math.abs(det-1)<.025&&norm.every((r,i)=>r.every((v,j)=>Math.abs(v-(i===j?1:0))<.025));
}
class State{
 constructor({minInliers=18,minRatio=.72,maxRelativeAngleRad=.55}={}){this.minInliers=minInliers;this.minRatio=minRatio;this.maxRelativeAngleRad=maxRelativeAngleRad;this.reset();}
 reset(){this.orientation=I();this.segment=0;this.segmentFrames=0;this.accepted=0;this.rejected=0;this.totalRotationRad=0;this.lastT=null;this.lastStatus='uninitialized';}
 advance(visual,epipolar,t){
  if(!Number.isFinite(t)||this.lastT!==null&&t<=this.lastT)throw Error('non_monotonic_vo_frame');
  this.lastT=t;
  const good=visual?.ok===true&&validRotation(visual.rotationCameraCurrentFromPrevious)&&
   Number.isFinite(visual.inliers)&&visual.inliers>=this.minInliers&&
   Number.isFinite(visual.ratio)&&visual.ratio>=this.minRatio&&
   Number.isFinite(visual.theta)&&visual.theta<=this.maxRelativeAngleRad;
  // A two-view epipolar test does not establish metric translation;
  // "low parallax" is not a reason to discard a valid pure rotation.
  if(!good){
   this.rejected++;this.segmentFrames=0;this.lastStatus='segment_broken';
   return this.snapshot(false,visual?.reason||'unreliable_visual_rotation');
  }
  if(this.segmentFrames===0){this.orientation=I();this.segment++;this.segmentFrames=0;}
  this.orientation=mul(visual.rotationCameraCurrentFromPrevious,this.orientation);
  if(!validRotation(this.orientation)){this.orientation=I();this.segmentFrames=0;this.rejected++;this.lastStatus='numerical_drift';return this.snapshot(false,'numerical_drift');}
  this.segmentFrames++;this.accepted++;this.totalRotationRad+=visual.theta;this.lastStatus='orientation_only';
  return this.snapshot(true,null);
 }
 snapshot(accepted=null,reason=null){return {accepted,reason,segment:this.segment,segmentFrames:this.segmentFrames,
  orientationCurrentFromSegmentStart:this.segmentFrames?this.orientation.map(r=>r.slice()):null,
  netAngleRad:this.segmentFrames?angle(this.orientation):null,totalRotationRad:this.totalRotationRad,
  acceptedSteps:this.accepted,rejectedSteps:this.rejected,lastStatus:this.lastStatus,
  intrinsicsVerified:false,rotationOnly:true,visualInertialFused:false,
  translationDirectionVerified:false,metricPositionAvailable:false,
  relativeTranslationMeters:null,worldPosition:null,loopClosureVerified:false};}
}
return {State,validRotation,mul,angle};
});