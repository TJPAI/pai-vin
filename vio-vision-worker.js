/* PAI-VIN V3.8 background vision computation.
 * Worker keeps persistent sparse tracks and estimates relative CAMERA
 * rotation. The main thread only samples pixels and receives IMU events.
 * All frames must arrive sequentially; the producer bounds in-flight work.
 * Browser reception time is NOT the exposure timestamp. */
importScripts('./vio-tracker.js','./vio-keyframes.js','./vio-rotation.js','./vio-geometry.js','./vio-vo-state.js','./vio-observation.js');
let tracker=new PAIVioTracks.Tracker({maxTracks:90});
let keyframes=new PAIVioKeyframes.KeyframeBank({maxFrames:3,interval:12,maxTrials:12});
let voState=new PAIVioVOState.State();
let lastT=null;
self.onmessage=(e)=>{
 const m=e.data;
 if(m?.type==='reset'){
  tracker.reset();keyframes.reset();voState.reset();lastT=null;postMessage({type:'reset-ack',session:m.session});return;
 }
 if(m?.type!=='frame')return;
 try{
  const begun=performance.now();
  const gray=new Uint8Array(m.buffer);
  const current=tracker.update(gray,m.t);
  const kf=keyframes.update(gray,tracker.tracks,current.frame,m.t);
  const trackingMs=performance.now()-begun;
  let vision=null,flow=null,epipolar=null,rotationMs=0,geometryMs=0;
  let candidateAB=null,abCostMs=0;
  if(lastT!==null){
   const matches=PAIVioObserve.tracksToPairs(current);
   flow=PAIVioObserve.flowStatistics(matches);
   const intr=PAIVioObserve.nominalIntrinsics(256,192); 
   const rt=performance.now();
   vision=PAIVioRotation.estimate(matches,intr);
   rotationMs=performance.now()-rt;
   const gt=performance.now();
   epipolar=PAIVioGeometry.estimate(matches,intr);
   geometryMs=performance.now()-gt;
   // Matched-frame independent A/B: same exact 2D tracks, no estimator switch.
   // Throttle expensive parallel epipolar fit to one frame per five.
   const ci=m.candidateIntrinsics;
   if(m.seq%5===0&&ci&&[ci.fx,ci.fy,ci.cx,ci.cy].every(Number.isFinite)&&
      ci.fx>0&&ci.fy>0&&ci.width===256&&ci.height===192&&
      ci.verified!==true){
    const tAB=performance.now();
    const candidateR=PAIVioRotation.estimate(matches,ci);
    const candidateE=PAIVioGeometry.estimate(matches,ci);
    const slim=o=>({ok:!!o.ok,reason:o.reason||null,inliers:o.inliers||0,
      total:o.total||0,ratio:o.ratio??null,medianErrorRad:o.medianErrorRad??null,
      medianSampson:o.medianSampson??null,theta:o.theta??null});
    candidateAB={nominal:{rotation:slim(vision),epipolar:slim(epipolar)},
      candidate:{rotation:slim(candidateR),epipolar:slim(candidateE)},
      matchedInput:true,candidateVerified:false,metricPoseAvailable:false};
    abCostMs=performance.now()-tAB;
   }
  }
  // A failed visual solve breaks the orientation segment rather than
  // pretending an IMU-calibrated world pose exists.
  const vo=lastT===null?voState.snapshot(false,'first_frame'):
   voState.advance(vision,epipolar,m.t);
  const trackPoints=tracker.tracks.map(({x,y,age})=>({x,y,age}));
  const start=lastT;lastT=m.t;
  postMessage({type:'result',session:m.session,seq:m.seq,start,end:m.t,
   processingMs:performance.now()-begun,
   stageTiming:{trackingMs,rotationMs,geometryMs,abCostMs},candidateAB,vo,keyframe:{...kf,summary:keyframes.snapshot()},
   tracks:{total:current.total,initial:current.initial,retained:current.retained,added:current.added,retention:current.retention,longLived:current.longLived,
    lost:current.lost,avgAge:current.avgAge,medianError:current.medianError,
    recoveryAttempts:current.recoveryAttempts,recovered:current.recovered,recoveryPrior:current.recoveryPrior,
    matchCount:current.matches.length},trackPoints,vision,flow,epipolar});
 }catch(error){
  postMessage({type:'error',session:m.session,seq:m.seq,message:String(error?.message||error)});
 }
};
