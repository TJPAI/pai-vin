/* PAI-VIN VIO acquisition probe. Browser data are observations, not pose.
 * Produces a privacy-conscious sensor quality report and an engine status.
 * Future WASM estimator consumes the same monotonic sensor transport.
 */
(function(){
 'use strict';
 const $=id=>document.getElementById(id);
 const buffer=new PAIVioSensor.SensorBuffer(5000);
 const trackCanvas=document.createElement('canvas');trackCanvas.width=256;trackCanvas.height=192;
 const trackContext=trackCanvas.getContext('2d',{willReadFrequently:true});
 let lastTrackingTime=-Infinity,lastTrackingReport=null;
 let visionWorker=null,workerBusy=false,workerSession=0,workerSequence=0,workerProcessed=0,workerSkipped=0,workerErrors=0,workerCostMs=null;
 let visualTime=null,lastVisual=null;
 const trackHistory=[];let captureCostMs=null,captureIntervalMs=null,frameTimerTicks=0,frameTooSoon=0;
 const visualHistory=[];const visualReasons={};const pendingObservations=[];
 let syncPassed=0,syncFailed=0,pairedPassed=0;
 let visualAccepted=0,visualRejected=0;
 let stream=null,timer=null,listener=null,framesSeen=0;
 let initializer=null,attitude=null,initAttemptAt=-Infinity,initReason='等待静止初始化';
 const show=s=>{$('vioStatus').textContent=s};
 function storeObservation(obs,t){
  lastVisual=obs;
  if(obs.ok)visualAccepted++;else{visualRejected++;visualReasons[obs.reason]=(visualReasons[obs.reason]||0)+1}
  if(obs.imuWindowOk)syncPassed++;else syncFailed++;
  if(obs.ok&&obs.imuWindowOk)pairedPassed++;
  visualHistory.push({t,reason:obs.reason,matchCount:obs.visualMatchCount,inliers:obs.visualInliers,
   ratio:obs.visualInlierRatio,cameraAngleDeg:obs.cameraRelativeAngleDeg,
   imuAngleDeg:obs.imuAngularMagnitudeDeg,imuOk:obs.imuWindowOk,
   imuReason:obs.imuWindowReason,dtMs:obs.dtMs});
  if(visualHistory.length>200)visualHistory.shift();
 }
 function flushObservations(force=false){
  // Delay visual/IMU comparison until a sensor event occurs AFTER the
  // captured frame, so the frame time can be interpolated, not extrapolated.
  // Scheduling delay does not imply accurate physical exposure synchronization.
  while(pendingObservations.length){
   const p=pendingObservations[0];
   const last=buffer.imu.at(-1);
   if(!force&&(!last||last.t<p.end)&&performance.now()-p.receivedAt<400)break;
   pendingObservations.shift();
   try{
    const obs=PAIVioObserve.assess({tracks:p.tracks,
     previousTime:p.start,currentTime:p.end,sensorBuffer:buffer,rotationEstimator:PAIVioRotation,
     visualOverride:p.vision});
    storeObservation(obs,p.end);
   }catch(e){
    storeObservation({ok:false,reason:'visual_observation_exception',visualMatchCount:0,
     imuWindowOk:false,imuWindowReason:e.message},p.end);
   }
  }
 }
 function report(){
  const r=buffer.report(),last=buffer.frames.at(-1),first=buffer.frames[0];
  const win=(last&&first&&last.t>first.t)?buffer.between(Math.max(first.t,last.t-1000),last.t):{ok:false,reason:'need_frames'};
  $('vioNumbers').textContent='处理帧 '+r.cameraFrames+' · IMU '+r.imuSamples+' · 视觉帧率 '+r.cameraRateHz.toFixed(1)+' Hz · IMU '+r.imuRateHz.toFixed(1)+' Hz · Worker 处理 '+workerProcessed+' 帧 / 跳过 '+workerSkipped+' 帧 · 视觉耗时 '+(workerCostMs===null?'—':workerCostMs.toFixed(0)+' ms')+' · IMU窗口 '+(win.ok?'可用':'不可用 '+win.reason);
  const q=lastTrackingReport;
  $('vioTrackNumbers').textContent=q?'图像跟踪点 '+q.total+' · 连续继承 '+q.retained+' · 5帧以上 '+q.longLived+' · 丢失 '+q.lost+' · 平均寿命 '+q.avgAge.toFixed(1)+' 帧':'尚未获得连续视觉特征';
  const fusionStatus=$('vioFusionStatus');
  if(fusionStatus){
   const last=lastVisual;
   const names={insufficient_tracks:'连续特征不足',rotation_geometry_unreliable:'旋转几何不一致',
    invalid_frame_timing:'帧时间异常',intrinsics_missing:'相机参数缺失',
    rotation_jump:'旋转估计跳变',visual_observation_exception:'观测模块异常'};
   const recent=last?.ok?'视觉旋转 '+last.cameraRelativeAngleDeg.toFixed(2)+'°，内点 '+last.visualInliers+'/'+last.visualMatchCount:
    (last?'视觉观测暂不可用：'+(names[last.reason]||last.reason)+'，匹配 '+last.visualMatchCount:'等待连续视觉帧');
   fusionStatus.textContent='实时视觉观测：'+recent+'；本次通过 '+visualAccepted+' 次，未通过 '+visualRejected+' 次。'+
    ' 已完成 IMU 时间配对 '+syncPassed+' 次，真正双有效 '+pairedPassed+' 次。'+
    ' IMU 数据窗口：'+(last?last.imuWindowOk?'可用':'不可用 ('+last.imuWindowReason+')':'等待')+
    '。采用未标定名义焦距，仅作诊断，不进行姿态融合或位置输出。';
  }
  const sensor=$('vioInitStatus');
  if(sensor){
   if(attitude){const state=attitude.snapshot();sensor.textContent='静止初始化成功 · 陀螺仪偏置 '+state.gyroBias.map(x=>x.toFixed(4)).join(', ')+' rad/s · IMU 姿态更新 '+state.samples+' 次 · 无视觉融合及米制位姿';}
   else sensor.textContent='保持手机静止约 2 秒以初始化 IMU：'+initReason;
  }
  return {r,win};
 }
 function receiveWorkerMessage(event){
  const m=event.data;
  if(m?.session!==workerSession)return;
  if(m.type==='reset-ack')return;
  workerBusy=false;
  if(m.type==='error'){workerErrors++;$('vioTrackNumbers').textContent='视觉 Worker 异常：'+m.message;return}
  if(m.type!=='result')return;
  workerProcessed++;workerCostMs=m.processingMs;
  lastTrackingReport=m.tracks;
  trackHistory.push({t:m.end,frameGapMs:m.start===null?null:m.end-m.start,
   tracks:m.tracks.total,retained:m.tracks.retained,added:m.tracks.added,
   lost:m.tracks.lost,retention:m.tracks.retention,longLived:m.tracks.longLived,
   visualMatches:m.tracks.matchCount,medianPatchError:m.tracks.medianError,
   workerProcessingMs:m.processingMs});
  if(trackHistory.length>200)trackHistory.shift();
  if(m.start!==null){
   pendingObservations.push({start:m.start,end:m.end,receivedAt:performance.now(),
    tracks:{matches:[]},vision:m.vision});
  }
  flushObservations();
  const ctx=$('vioTrackCanvas').getContext('2d');
  ctx.clearRect(0,0,256,192);
  ctx.strokeStyle='#67ddb5';ctx.lineWidth=1.1;
  for(const p of m.trackPoints){
   ctx.beginPath();ctx.arc(p.x,p.y,p.age>=5?3:1.5,0,Math.PI*2);ctx.stroke();
  }
  report();
 }
 function sampleFrame(){
  const v=$('vioVideo');if(!stream||v.readyState<2)return;
  const t=performance.now();frameTimerTicks++;
  // Never queue multiple expensive image jobs: prioritize real IMU events.
  if(workerBusy){workerSkipped++;flushObservations();return}
  if(t-lastTrackingTime<120){frameTooSoon++;return}
  captureIntervalMs=Number.isFinite(lastTrackingTime)?t-lastTrackingTime:null;
  lastTrackingTime=t;
  try{
   const started=performance.now();
   trackContext.drawImage(v,0,0,256,192);
   const data=trackContext.getImageData(0,0,256,192).data;
   const gray=VINFeatures.grayRGBA(data);
   captureCostMs=performance.now()-started;
   if(!buffer.addFrame({t,width:v.videoWidth,height:v.videoHeight}))return;
   framesSeen++;
   workerBusy=true;
   visionWorker.postMessage({type:'frame',session:workerSession,seq:++workerSequence,t,buffer:gray.buffer},[gray.buffer]);
   flushObservations();
  }catch(e){workerBusy=false;workerErrors++;$('vioTrackNumbers').textContent='视觉处理异常：'+e.message}
  report();
 }
 async function start(){
  if(stream)return;
  buffer.reset();framesSeen=0;lastTrackingTime=-Infinity;lastTrackingReport=null;
  workerSession++;workerSequence=0;workerBusy=false;workerProcessed=0;workerSkipped=0;workerErrors=0;workerCostMs=null;
  trackHistory.length=0;captureCostMs=null;captureIntervalMs=null;frameTimerTicks=0;frameTooSoon=0;
  initializer=null;attitude=null;initAttemptAt=-Infinity;initReason='等待静止初始化';
  visualTime=null;lastVisual=null;visualAccepted=0;visualRejected=0;syncPassed=0;syncFailed=0;pairedPassed=0;visualHistory.length=0;pendingObservations.length=0;
  Object.keys(visualReasons).forEach(k=>delete visualReasons[k]);
  if(window.PAIVIN?.stopBoardCamera)window.PAIVIN.stopBoardCamera();
  try{
   if(typeof DeviceMotionEvent!=='undefined'&&typeof DeviceMotionEvent.requestPermission==='function'){
    const p=await DeviceMotionEvent.requestPermission();
    if(p!=='granted')throw Error('IMU 权限未授权');
   }
   listener=e=>{
    const s=PAIVioSensor.fromDeviceMotion(e,performance.now());
    if(!s||!buffer.addImu(s))return;
    // No computation here: data collection must remain lightweight.
    if(attitude){const updated=attitude.push(s);if(!updated.ok)initReason='姿态传播暂停：'+updated.reason;return}
    if(s.t-initAttemptAt<500)return;
    initAttemptAt=s.t;
    const recent=buffer.imu.filter(x=>x.t>=s.t-2500);
    initializer=PAIVioInit.initialize(recent,{minSamples:80,durationMs:1500});
    if(initializer.ok){attitude=new PAIVioInit.Attitude(initializer);attitude.push(s);initReason='完成'}
    else initReason=initializer.reason;
   };
   window.addEventListener('devicemotion',listener);
   if(typeof Worker!=='function')throw Error('此浏览器不支持 Web Worker 视觉处理');
   visionWorker=new Worker('./vio-vision-worker.js');
   visionWorker.onmessage=receiveWorkerMessage;
   visionWorker.onerror=e=>{workerErrors++;workerBusy=false;show('视觉 Worker 加载或运行失败：'+(e.message||'unknown'))};
   visionWorker.postMessage({type:'reset',session:workerSession});
   stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480},frameRate:{ideal:20}}});
   const video=$('vioVideo');video.srcObject=stream;await video.play();
   timer=setInterval(sampleFrame,100);
   $('vioStart').disabled=true;$('vioStop').disabled=false;
   show('正在采集：仅验证摄像头与 IMU 的数据质量。相机曝光时间戳、相机—IMU 外参及完整 VIO 尚未校准。');
  }catch(e){stop();show('启动失败：'+e.message)}
 }
 function stop(){
  flushObservations(true);
  clearInterval(timer);timer=null;
  if(visionWorker){visionWorker.terminate();visionWorker=null;workerBusy=false}
  if(stream)stream.getTracks().forEach(t=>t.stop());stream=null;
  if(listener)window.removeEventListener('devicemotion',listener);listener=null;
  $('vioVideo').srcObject=null;$('vioStart').disabled=false;$('vioStop').disabled=true;
  report();
 }
 function download(){
  const {r,win}=report();
  const data={schema:'pai-vin-vio-acquisition-v1',capturedAt:new Date().toISOString(),browserClock:'performance.now',
   cameraExposureTimestampAvailable:false,deviceIMUExtrinsicsCalibrated:false,wasmVioEstimatorConnected:false,
   visualRotationModuleAvailable:true,visualAttitudeCorrectionEnabled:false,
   visualRotationDiagnostic:{accepted:visualAccepted,rejected:visualRejected,
    imuWindowsPassed:syncPassed,imuWindowsFailed:syncFailed,bothPassed:pairedPassed,
    deferredWindowMatching:true,reasons:{...visualReasons},
    intrinsics:'unverified_nominal',recent:visualHistory},
   visionProcessing:{mode:'web_worker',processed:workerProcessed,skipped:workerSkipped,
    errors:workerErrors,lastWorkerProcessingMs:workerCostMs,
    lastMainThreadCaptureMs:captureCostMs,lastFrameIntervalMs:captureIntervalMs,
    timerTicks:frameTimerTicks,intervalFiltered:frameTooSoon,
    trackHistory,rawCameraExposureTimestamp:false},
   metricPoseAvailable:false,containsPhotos:false,containsFrames:false,
   transport:r,window:win.ok?{ok:true,dt:win.dt,maxGapMs:win.maxGapMs}:win,
   visualTracking:lastTrackingReport?{total:lastTrackingReport.total,retained:lastTrackingReport.retained,longLived:lastTrackingReport.longLived,avgAge:lastTrackingReport.avgAge,medianError:lastTrackingReport.medianError}:null,
   inertialInitialization:initializer?.ok?{ok:true,gyroBias:initializer.gyroBias,gravityDevice:initializer.gravityDevice,gravityMagnitude:initializer.gravityMagnitude,gyroStd:initializer.gyroStd,accelStd:initializer.accelStd}: {ok:false,reason:initReason},
   inertialAttitude:attitude?attitude.snapshot():null,
   samples:buffer.imu.slice(-300).map(({t,gx,gy,gz,ax,ay,az})=>({t,gx,gy,gz,ax,ay,az}))};
  const url=URL.createObjectURL(new Blob([JSON.stringify(data)],{type:'application/json'}));
  const a=document.createElement('a');a.href=url;a.download='pai-vin-vio-sensors-'+Date.now()+'.json';
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),30000);
 }
 $('vioStart').onclick=start;$('vioStop').onclick=stop;$('vioExport').onclick=download;
 window.addEventListener('pagehide',stop);
 report();
})();