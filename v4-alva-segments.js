/* PAI-VIN V4.0.3: segment raw SLAM poses by reset/loss; never join
 * unrelated arbitrary-scale coordinate frames or claim metric distance. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIV4Segments=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
class Segments{
 constructor(){this.reset();}
 reset(){this.count=0;this.active=null;this.completed=[];this.maxConsecutive=0;this.maxJump=0;this.totalPoseFrames=0;}
 update(rawStatus,pose,t,frame){
  const ok=rawStatus===1&&pose&&pose.length===16&&Array.from(pose).every(Number.isFinite);
  if(!ok){
   if(this.active){this.active.endReason=rawStatus===2?'native_reset':'pose_unavailable';this.completed.push(this.active);this.active=null;}
   return {segmentId:null,jump:null,segmentFrames:0,coordinateContinuity:false};
  }
  if(!this.active){this.active={id:++this.count,startFrame:frame,startMs:t,frames:0,largestJump:0,lastPosition:null,endReason:null};}
  const seg=this.active,pos=Array.from(pose).slice(12,15);
  const jump=seg.lastPosition?Math.hypot(...pos.map((v,i)=>v-seg.lastPosition[i])):null;
  if(jump!==null){seg.largestJump=Math.max(seg.largestJump,jump);this.maxJump=Math.max(this.maxJump,jump);}
  seg.lastPosition=pos;seg.frames++;seg.endFrame=frame;seg.endMs=t;
  this.totalPoseFrames++;this.maxConsecutive=Math.max(this.maxConsecutive,seg.frames);
  return {segmentId:seg.id,jump,segmentFrames:seg.frames,coordinateContinuity:seg.frames>1};
 }
 snapshot(){
  const all=[...this.completed,...(this.active?[this.active]:[])];
  return {segments:this.count,completed:this.completed.length,activeSegmentId:this.active?.id??null,
   longestFrames:this.maxConsecutive,maxAdjacentStepArbitraryUnits:this.maxJump,
   segmentsSummary:all.map(s=>({id:s.id,frames:s.frames,startFrame:s.startFrame,endFrame:s.endFrame,
     startMs:s.startMs,endMs:s.endMs,largestJumpArbitraryUnits:+s.largestJump.toFixed(3),endReason:s.endReason})),
   metricScaleVerified:false,worldMapContinuityVerified:false};
 }
}
return {Segments};
});