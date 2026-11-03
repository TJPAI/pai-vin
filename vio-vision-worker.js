/* PAI-VIN V3.8 background vision computation.
 * Worker keeps persistent sparse tracks and estimates relative CAMERA
 * rotation. The main thread only samples pixels and receives IMU events.
 * All frames must arrive sequentially; the producer bounds in-flight work.
 * Browser reception time is NOT the exposure timestamp. */
importScripts('./vio-tracker.js','./vio-rotation.js','./vio-geometry.js','./vio-observation.js');
let tracker=new PAIVioTracks.Tracker({maxTracks:90});
let lastT=null;
self.onmessage=(e)=>{
 const m=e.data;
 if(m?.type==='reset'){
  tracker.reset();lastT=null;postMessage({type:'reset-ack',session:m.session});return;
 }
 if(m?.type!=='frame')return;
 try{
  const begun=performance.now();
  const gray=new Uint8Array(m.buffer);
  const current=tracker.update(gray,m.t);
  let vision=null,flow=null,epipolar=null;
  if(lastT!==null){
   const matches=PAIVioObserve.tracksToPairs(current);
   flow=PAIVioObserve.flowStatistics(matches);
   const intr=PAIVioObserve.nominalIntrinsics(256,192);\n   vision=PAIVioRotation.estimate(matches,intr);\n   epipolar=PAIVioGeometry.estimate(matches,intr);
  }
  const trackPoints=tracker.tracks.map(({x,y,age})=>({x,y,age}));
  const start=lastT;lastT=m.t;
  postMessage({type:'result',session:m.session,seq:m.seq,start,end:m.t,
   processingMs:performance.now()-begun,
   tracks:{total:current.total,initial:current.initial,retained:current.retained,added:current.added,retention:current.retention,longLived:current.longLived,
    lost:current.lost,avgAge:current.avgAge,medianError:current.medianError,
    matchCount:current.matches.length},trackPoints,vision,flow,epipolar});
 }catch(error){
  postMessage({type:'error',session:m.session,seq:m.seq,message:String(error?.message||error)});
 }
};
