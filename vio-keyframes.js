/* V3.24: bounded keyframe image-plane observations. No 3D pose. */
(function(root,factory){const api=factory(root.PAIVioTracks||(typeof require==='function'?require('./vio-tracker.js'):null));if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioKeyframes=api;})(typeof globalThis!=='undefined'?globalThis:this,function(T){
'use strict';
const median=a=>{if(!a.length)return null;const b=a.slice().sort((x,y)=>x-y);return b[b.length>>1];};
class KeyframeBank{
 constructor({maxFrames=3,interval=12,minShared=8,maxTrials=12}={}){this.maxFrames=maxFrames;this.interval=interval;this.minShared=minShared;this.maxTrials=maxTrials;this.reset();}
 reset(){this.frames=[];this.nextId=1;this.attempts=0;this.verified2D=0;this.rejected=0;}
 update(gray,tracks,index,t){
  if(!(gray instanceof Uint8Array)||!Array.isArray(tracks)||!Number.isFinite(index))throw Error('invalid keyframe observation');
  const live=tracks.filter(p=>Number.isInteger(p.id)&&p.id>0&&[p.x,p.y].every(Number.isFinite));
  const byId=new Map(live.map(p=>[p.id,p]));const previous=this.frames.at(-1);
  const shared=previous?previous.points.filter(p=>byId.has(p.id)).length:0;
  let inserted=false;
  if(live.length>=12&&(!previous||index-previous.index>=this.interval&&shared<Math.max(this.minShared,Math.floor(previous.points.length*.7)))){
   this.frames.push({id:this.nextId++,index,t,gray:gray.slice(),points:live.map(({id,x,y})=>({id,x,y}))});
   if(this.frames.length>this.maxFrames)this.frames.shift();inserted=true;
  }
  const result={inserted,keyframeCount:this.frames.length,latestKeyframeId:this.frames.at(-1)?.id||null,
   sharedIds:shared,attempts:0,verified2D:0,rejected:0,observations:[],metricPoseAvailable:false};
  if(inserted||index%5!==0||!this.frames.length||live.length<8)return result;
  const ref=this.frames.at(-1),joined=ref.points.filter(p=>byId.has(p.id));
  if(joined.length<this.minShared)return result;
  const dx=median(joined.map(p=>byId.get(p.id).x-p.x)),dy=median(joined.map(p=>byId.get(p.id).y-p.y));
  if(Math.hypot(dx,dy)>24)return result;
  const residual=median(joined.map(p=>Math.hypot(byId.get(p.id).x-p.x-dx,byId.get(p.id).y-p.y-dy)));
  if(residual>2.5)return result;
  for(const p of ref.points.filter(p=>!byId.has(p.id)).slice(0,this.maxTrials)){
   result.attempts++;this.attempts++;
   const q=T.trackOne(ref.gray,gray,p,{predictionOffset:[dx,dy],subpixel:false});
   const valid=q&&Math.hypot(q.x-p.x-dx,q.y-p.y-dy)<=2.5&&
    live.every(v=>Math.hypot(v.x-q.x,v.y-q.y)>=6)&&
    result.observations.every(v=>Math.hypot(v.x-q.x,v.y-q.y)>=6);
   if(valid){result.verified2D++;this.verified2D++;
    result.observations.push({keyframeId:ref.id,trackId:p.id,sourceX:p.x,sourceY:p.y,x:q.x,y:q.y,ssd:q.error,fbError:q.fbError});
   }else{result.rejected++;this.rejected++;}
  }
  return result;
 }
 snapshot(){return {keyframes:this.frames.length,attempts:this.attempts,verified2D:this.verified2D,rejected:this.rejected,observationsOnly:true,metricPoseAvailable:false};}
}
return {KeyframeBank};
});