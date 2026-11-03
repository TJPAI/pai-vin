/* V3.29: conservative keyframe-assisted SAME-INTERVAL track reattachment.
 * An older keyframe candidate MUST be independently verified against the
 * immediately previous image. No track is revived after a multi-frame gap.
 * This is image-plane track repair, not relocalization or 3D mapping.
 */
(function(root,factory){const api=factory(root.PAIVioTracks||(typeof require==='function'?require('./vio-tracker.js'):null));if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioRepair=api;})(typeof globalThis!=='undefined'?globalThis:this,function(T){
'use strict';
function repair({previousGray,previousTracks,currentGray,tracker,report,observations,maxTrials=8}){
 const result={attempts:0,validated:0,attached:0,rejected:0,reason:null,
  sameIntervalOnly:true,metricPoseAvailable:false};
 if(!previousGray||!currentGray||!Array.isArray(previousTracks)||!tracker||!report||!Array.isArray(observations)){
  result.reason='missing_inputs';return result;
 }
 const old=new Map(previousTracks.filter(p=>p.age>=3).map(p=>[p.id,p]));
 const live=new Set(tracker.tracks.map(p=>p.id));
 const attempts=observations.filter(p=>old.has(p.trackId)&&!live.has(p.trackId)).slice(0,maxTrials);
 // Reduce singleton coincidences: require two independent keyframe proposals.
 if(attempts.length<2){result.reason='insufficient_candidates';return result;}
 const accepted=[];
 for(const obs of attempts){
  result.attempts++;
  if(!Number.isFinite(obs.x)||!Number.isFinite(obs.y)||obs.fbError>1.5){result.rejected++;continue;}
  const p=old.get(obs.trackId);
  const q=T.trackOne(previousGray,currentGray,p,{predictionOffset:[obs.x-p.x,obs.y-p.y],subpixel:false,
   maxError:260,backwardSearch:6});
  if(!q||q.fbError>1.5||Math.hypot(q.x-obs.x,q.y-obs.y)>1.6||
   accepted.some(v=>Math.hypot(v.x-q.x,v.y-q.y)<8)){
   result.rejected++;continue;
  }
  const close=tracker.tracks.find(v=>Math.hypot(v.x-q.x,v.y-q.y)<6);
  if(close&&close.age>1){result.rejected++;continue;}
  accepted.push({p,q,replaceId:close?.id??null});
  result.validated++;
 }
 if(accepted.length<2){result.reason='insufficient_independent_verification';return result;}
 for(const {p,q,replaceId} of accepted){
  if(replaceId!==null){const at=tracker.tracks.findIndex(x=>x.id===replaceId);
   if(at<0||tracker.tracks[at].age>1){result.rejected++;continue;}
   tracker.tracks.splice(at,1);report.added=Math.max(0,report.added-1);
  }else if(tracker.tracks.length>=tracker.maxTracks){result.rejected++;continue;}
  const recovered={...p,x:q.x,y:q.y,age:p.age+1,dx:q.x-p.x,dy:q.y-p.y};
  tracker.tracks.push(recovered);
  report.matches.push({id:p.id,x:q.x,y:q.y,dx:recovered.dx,dy:recovered.dy,age:recovered.age});
  report.retained++;report.lost=Math.max(0,report.lost-1);
  result.attached++;
 }
 report.total=tracker.tracks.length;
 report.retention=report.initial?report.retained/report.initial:null;
 report.avgAge=tracker.tracks.length?tracker.tracks.reduce((s,p)=>s+p.age,0)/tracker.tracks.length:0;
 report.longLived=tracker.tracks.filter(p=>p.age>=5).length;
 tracker.last=report;
 return result;
}
return {repair};
});