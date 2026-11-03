/* PAI-VIN V3.8 background vision computation.
 * Worker keeps persistent sparse tracks and estimates relative CAMERA
 * rotation. The main thread only samples pixels and receives IMU events.
 * All frames must arrive sequentially; the producer bounds in-flight work.
 * Browser reception time is NOT the exposure timestamp. */
importScripts('./vio-tracker.js','./vio-rotation.js','./vio-observation.js');
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
  let vision=null;
  if(lastT!==null){
   const matches=PAIVioObserve.tracksToPairs(current);
   vision=PAIVioRotation.estimate(matches,PAIVioObserve.nominalIntrinsics(256,192));
  }
  const trackPoints=tracker.tracks.map(({x,y,age})=>({x,y,age}));
  const start=lastT;lastT=m.t;
  postMessage({type:'result',session:m.session,seq:m.seq,start,end:m.t,
   processingMs:performance.now()-begun,
   tracks:{total:current.total,retained:current.retained,longLived:current.longLived,
    lost:current.lost,avgAge:current.avgAge,medianError:current.medianError,
    matchCount:current.matches.length},trackPoints,vision});
 }catch(error){
  postMessage({type:'error',session:m.session,seq:m.seq,message:String(error?.message||error)});
 }
};
