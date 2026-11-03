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
 let visualTime=null,lastVisual=null,lastEpipolar=null,epipolarAccepted=0,epipolarRejected=0;
 const trackHistory=[];let lastKeyframe=null,keyframeHistory=[];let captureCostMs=null,captureIntervalMs=null,frameTimerTicks=0,frameTooSoon=0;
 const visualHistory=[];const visualReasons={};const pendingObservations=[];
 let syncPassed=0,syncFailed=0,pairedPassed=0;
 let preintPassed=0,preintFailed=0,lastPreintegration=null;
 let visualAccepted=0,visualRejected=0;
 const calibrationStatus=()=>{
  let plane=null;try{plane=JSON.parse(localStorage.getItem('pai-vin-a4-plane-v1')||'null');}catch(e){}
  return PAIVioCalibration.fromBrowser({a4:plane,exposureTimestampAvailable:false});
 };
 let stream=null,timer=null,listener=null,framesSeen=0;
 let trackSourceGeometry=null,sharedCameraHandoff=null;
 let candidateABChecked=0,candidateABRotation={nominal:0,candidate:0},candidateABEpipolar={nominal:0,candidate:0},candidateABHistory=[];
 let initializer=null,attitude=null,initAttemptAt=-Infinity,initReason='等待静止初始化';
 const show=s=>{$('vioStatus').textContent=s};
 function storeObservation(obs,t){
  lastVisual=obs;
  if(obs.ok)visualAccepted++;else{visualRejected++;visualReasons[obs.reason]=(visualReasons[obs.reason]||0)+1}
  if(obs.imuWindowOk)syncPassed++;else syncFailed++;
  if(obs.ok&&obs.imuWindowOk)pairedPassed++;
  visualHistory.push({t,reason:obs.reason,detail:obs.geometryFailureDetail,flow:obs.flowDiagnostic,matchCount:obs.visualMatchCount,inliers:obs.visualInliers,
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
     visualOverride:p.vision,flowOverride:p.flow});
    const bracket=buffer.between(p.start,p.end);
    const integ=bracket.ok?PAIVioPreintegrate.integrate(bracket.samples,{gyroBias:initializer?.ok?initializer.gyroBias:[0,0,0]}):{ok:false,reason:bracket.reason};
    if(integ.ok){preintPassed++;lastPreintegration={ok:true,dt:integ.dt,samples:integ.samples,maxGapMs:integ.maxGapMs,
     deltaQuaternion:integ.deltaQuaternion,gravityCompensated:false,metricPoseAvailable:false};}
    else{preintFailed++;lastPreintegration={ok:false,reason:integ.reason};}
    storeObservation(obs,p.end);
   }catch(e){
    storeObservation({ok:false,reason:'visual_observation_exception',visualMatchCount:0,
     imuWindowOk:false,imuWindowReason:e.message},p.end);
   }
  }
 }
 function report(){
  const readiness=calibrationStatus();
  let focal=null;try{focal=JSON.parse(localStorage.getItem('pai-vin-focal-candidate-v1')||'null')}catch(e){}
  const geometryCheck=sharedCameraHandoff?
   PAIVioViewport.compatibility(sharedCameraHandoff.boardGeometry,trackSourceGeometry,
    {sameStream:true,intrinsics:sharedCameraHandoff.focal}):
   PAIVioViewport.compatibility(focal?.boardSourceGeometry,trackSourceGeometry);
  const geometryEl=$('vioGeometryStatus');
  if(geometryEl)geometryEl.textContent='相机图像几何：'+(trackSourceGeometry?'VIO源 '+trackSourceGeometry.sourceWidth+'×'+trackSourceGeometry.sourceHeight+'→256×192；':'未取得VIO源画幅；')+
   (focal?.boardSourceGeometry?'标定源 '+focal.boardSourceGeometry.sourceWidth+'×'+focal.boardSourceGeometry.sourceHeight+'；':'标定源画幅未知；')+
   (geometryCheck.ok?'同一摄像头会话：图像变换可用；':'焦距不能直接套用（'+geometryCheck.reason+'）；')+
   (geometryCheck.candidateIntrinsics?'跟踪图像候选焦距 fx='+geometryCheck.candidateIntrinsics.fx.toFixed(1)+'px；':'')+
   '跟踪采用等比居中裁剪'+(trackSourceGeometry?.visibleFraction!==undefined?'（保留 '+(100*trackSourceGeometry.visibleFraction).toFixed(1)+'% 图像面积）':'')+'；候选未完成完整标定，视觉仍用名义焦距诊断。';
  const el=$('vioCalibrationStatus');
  if(el)el.textContent='联合估计标定检查：'+(readiness.a4PlaneAvailable?'A4 平面参照已保存；':'A4 平面参照未提供；')+
   '相机内参、相机—IMU 外参与真实曝光时间偏移均未验证，视觉惯性位置融合保持关闭。';
  const r=buffer.report(),last=buffer.frames.at(-1),first=buffer.frames[0];
  const win=(last&&first&&last.t>first.t)?buffer.between(Math.max(first.t,last.t-1000),last.t):{ok:false,reason:'need_frames'};
  $('vioNumbers').textContent='处理帧 '+r.cameraFrames+' · IMU '+r.imuSamples+' · 视觉帧率 '+r.cameraRateHz.toFixed(1)+' Hz · IMU '+r.imuRateHz.toFixed(1)+' Hz · Worker 处理 '+workerProcessed+' 帧 / 跳过 '+workerSkipped+' 帧 · 视觉耗时 '+(workerCostMs===null?'—':workerCostMs.toFixed(0)+' ms')+' · IMU窗口 '+(win.ok?'可用':'不可用 '+win.reason);
  const q=lastTrackingReport;
  $('vioTrackNumbers').textContent=q?'图像跟踪点 '+q.total+' · 连续继承 '+q.retained+' · 5帧以上 '+q.longLived+' · 丢失 '+q.lost+' · 平均寿命 '+q.avgAge.toFixed(1)+' 帧 · 运动辅助恢复 '+(q.recovered||0)+'/'+(q.recoveryAttempts||0):'尚未获得连续视觉特征';
  const fusionStatus=$('vioFusionStatus');
  if(fusionStatus){
   const last=lastVisual;
   const epi=lastEpipolar;
   const names={insufficient_tracks:'连续特征不足',rotation_geometry_unreliable:'旋转几何不一致',
    invalid_frame_timing:'帧时间异常',intrinsics_missing:'相机参数缺失',
    rotation_jump:'旋转估计跳变',visual_observation_exception:'观测模块异常'};
   const details={too_few_rotation_inliers:'纯旋转模型内点不足',low_rotation_inlier_ratio:'纯旋转模型内点比例低',high_rotation_residual:'纯旋转模型残差偏大'};
   const flowInfo=last?.flowDiagnostic?.count>0?' · 2D光流中值 '+last.flowDiagnostic.flowMedianPx+' px、离散度 '+last.flowDiagnostic.flowResidualMedianPx+' px':'';
   const recent=last?.ok?'视觉旋转 '+last.cameraRelativeAngleDeg.toFixed(2)+'°，内点 '+last.visualInliers+'/'+last.visualMatchCount:
    (last?'视觉观测暂不可用：'+(details[last.geometryFailureDetail]||names[last.reason]||last.reason)+'，匹配 '+last.visualMatchCount:'等待连续视觉帧');
   fusionStatus.textContent='实时视觉观测：'+recent+'；本次通过 '+visualAccepted+' 次，未通过 '+visualRejected+' 次。'+
    ' 已完成 IMU 时间配对 '+syncPassed+' 次，真正双有效 '+pairedPassed+' 次。'+
    flowInfo+(epi?' · 双视图极线 '+(epi.ok?'一致':'暂不可用 '+epi.reason)+' '+epi.inliers+'/'+epi.total+'，中位图像位移 '+(epi.medianFlowPx?.toFixed(1)??'—')+' px':'')+' IMU 数据窗口：'+(last?last.imuWindowOk?'可用':'不可用 ('+last.imuWindowReason+')':'等待')+
    '。采用未标定名义焦距，仅作诊断，不进行姿态融合或位置输出。';
  }
  const abEl=$('vioCandidateABStatus');
  if(abEl)abEl.textContent=candidateABChecked?
   '相同视觉匹配点候选内参 A/B（'+candidateABChecked+'帧，每5帧评测）：旋转名义 '+
    candidateABRotation.nominal+'/'+candidateABChecked+'、候选 '+
    candidateABRotation.candidate+'/'+candidateABChecked+'；极线名义 '+
    candidateABEpipolar.nominal+'/'+candidateABChecked+'、候选 '+
    candidateABEpipolar.candidate+'/'+candidateABChecked+'。仅独立诊断，未改变实时观测与融合。':
   '候选内参 A/B 尚无有效同帧样本；需本次摄像头会话完成多视角 A4 候选。';
  const keyEl=$('vioKeyframeStatus');
  if(keyEl)keyEl.textContent=lastKeyframe?'关键帧 '+lastKeyframe.keyframeCount+' · 历史二维重匹配 '+lastKeyframe.summary.verified2D+'/'+lastKeyframe.summary.attempts+' · 未建立3D地图':'等待关键帧二维观测';
  const pre=$('vioPreintegrationStatus');
  if(pre)pre.textContent='IMU 预积分窗口 '+preintPassed+' 次成功 / '+preintFailed+' 次失败 · '+
    (lastPreintegration?.ok?'最新 '+(lastPreintegration.dt*1000).toFixed(0)+'ms、'+lastPreintegration.samples+' 样本':lastPreintegration?'暂不可用 '+lastPreintegration.reason:'等待数据')+
    ' · 仅为惯性因子，未经重力/外参/时间标定，不输出位置。';
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
  if(m.keyframe){lastKeyframe=m.keyframe;if(m.keyframe.inserted||m.keyframe.attempts){keyframeHistory.push({t:m.end,...m.keyframe});if(keyframeHistory.length>100)keyframeHistory.shift();}}
  if(m.candidateAB?.matchedInput){
   const ab=m.candidateAB;candidateABChecked++;
   if(ab.nominal.rotation.ok)candidateABRotation.nominal++;
   if(ab.candidate.rotation.ok)candidateABRotation.candidate++;
   if(ab.nominal.epipolar.ok)candidateABEpipolar.nominal++;
   if(ab.candidate.epipolar.ok)candidateABEpipolar.candidate++;
   candidateABHistory.push({t:m.end,nominal:ab.nominal,candidate:ab.candidate,abCostMs:m.stageTiming?.abCostMs||0});
   if(candidateABHistory.length>200)candidateABHistory.shift();
  }
  lastTrackingReport=m.tracks;
  lastEpipolar=m.epipolar||null;
  if(lastEpipolar){if(lastEpipolar.ok)epipolarAccepted++;else epipolarRejected++;}
  trackHistory.push({t:m.end,frameGapMs:m.start===null?null:m.end-m.start,
   tracks:m.tracks.total,retained:m.tracks.retained,added:m.tracks.added,
   lost:m.tracks.lost,retention:m.tracks.retention,longLived:m.tracks.longLived,
   recoveryAttempts:m.tracks.recoveryAttempts,recovered:m.tracks.recovered,recoveryPrior:m.tracks.recoveryPrior,
   visualMatches:m.tracks.matchCount,medianPatchError:m.tracks.medianError,
   workerProcessingMs:m.processingMs,
   stageTiming:m.stageTiming||null});
  if(trackHistory.length>200)trackHistory.shift();
  if(m.start!==null){
   pendingObservations.push({start:m.start,end:m.end,receivedAt:performance.now(),
    tracks:{matches:[]},vision:m.vision,flow:m.flow});
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
  // The timer fires roughly every 100 ms. A 120 ms gate previously
  // discarded every other tick and forced ~5 Hz despite ~52 ms Worker cost.
  if(t-lastTrackingTime<90){frameTooSoon++;return}
  captureIntervalMs=Number.isFinite(lastTrackingTime)?t-lastTrackingTime:null;
  lastTrackingTime=t;
  try{
   const started=performance.now();
   trackSourceGeometry=PAIVioViewport.rect(v.videoWidth,v.videoHeight,256,192,'cover');
   const crop=PAIVioViewport.sourceCrop(trackSourceGeometry);
   if(!crop.ok)throw Error('video_crop_'+crop.reason);
   trackContext.drawImage(v,crop.x,crop.y,crop.width,crop.height,0,0,256,192);
   const data=trackContext.getImageData(0,0,256,192).data;
   const gray=VINFeatures.grayRGBA(data);
   captureCostMs=performance.now()-started;
   if(!buffer.addFrame({t,width:v.videoWidth,height:v.videoHeight}))return;
   framesSeen++;
   workerBusy=true;
   const sourceCheck=sharedCameraHandoff?PAIVioViewport.compatibility(
    sharedCameraHandoff.boardGeometry,trackSourceGeometry,
    {sameStream:true,intrinsics:sharedCameraHandoff.focal}):null;
   const candidate=sourceCheck?.candidateIntrinsics||null;
   visionWorker.postMessage({type:'frame',session:workerSession,seq:++workerSequence,t,
    candidateIntrinsics:candidate?{fx:candidate.fx,fy:candidate.fy,
     cx:candidate.cx,cy:candidate.cy,width:256,height:192,verified:false}:null,
    buffer:gray.buffer},[gray.buffer]);
   flushObservations();
  }catch(e){workerBusy=false;workerErrors++;$('vioTrackNumbers').textContent='视觉处理异常：'+e.message}
  report();
 }
 async function start(){
  if(stream)return;
  buffer.reset();framesSeen=0;trackSourceGeometry=null;sharedCameraHandoff=null;lastTrackingTime=-Infinity;lastKeyframe=null;keyframeHistory=[];
  candidateABChecked=0;candidateABRotation={nominal:0,candidate:0};candidateABEpipolar={nominal:0,candidate:0};candidateABHistory=[];lastTrackingReport=null;
  workerSession++;workerSequence=0;workerBusy=false;workerProcessed=0;workerSkipped=0;workerErrors=0;workerCostMs=null;
  trackHistory.length=0;captureCostMs=null;captureIntervalMs=null;frameTimerTicks=0;frameTooSoon=0;
  initializer=null;attitude=null;initAttemptAt=-Infinity;initReason='等待静止初始化';
  visualTime=null;lastVisual=null;lastEpipolar=null;epipolarAccepted=0;epipolarRejected=0;visualAccepted=0;visualRejected=0;syncPassed=0;syncFailed=0;pairedPassed=0;preintPassed=0;preintFailed=0;lastPreintegration=null;visualHistory.length=0;pendingObservations.length=0;
  Object.keys(visualReasons).forEach(k=>delete visualReasons[k]);
  // Do not stop the calibration video before transferring its live stream.
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
   sharedCameraHandoff=window.PAIVIN?.handoffCamera?.()||null;
   stream=sharedCameraHandoff?.stream||await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480},frameRate:{ideal:20}}});
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
  const data={schema:'pai-vin-vio-acquisition-v1',appVersion:document.getElementById('appVersion')?.dataset.version||'unknown',buildId:document.getElementById('appVersion')?.dataset.build||'unknown',capturedAt:new Date().toISOString(),browserClock:'performance.now',
   cameraExposureTimestampAvailable:false,deviceIMUExtrinsicsCalibrated:false,wasmVioEstimatorConnected:false,
   visualRotationModuleAvailable:true,visualAttitudeCorrectionEnabled:false,
   calibrationReadiness:calibrationStatus(),
   cameraFrameGeometry:{tracking:trackSourceGeometry,reusedCalibrationStream:!!sharedCameraHandoff,
    compatibility:(()=>{
    let focal=null;try{focal=JSON.parse(localStorage.getItem('pai-vin-focal-candidate-v1')||'null')}catch(e){}
    return sharedCameraHandoff?PAIVioViewport.compatibility(sharedCameraHandoff.boardGeometry,trackSourceGeometry,
     {sameStream:true,intrinsics:sharedCameraHandoff.focal}):
     PAIVioViewport.compatibility(focal?.boardSourceGeometry,trackSourceGeometry);
   })(),activeIntrinsics:'unverified_nominal',calibrationCandidateUsedForTracking:false},
   imuPreintegrationDiagnostic:{passed:preintPassed,failed:preintFailed,last:lastPreintegration,
    gravityCompensated:false,metricPoseAvailable:false,usesBrowserReceiptTimestamps:true},
   twoViewEpipolarDiagnostic:{accepted:epipolarAccepted,rejected:epipolarRejected,last:lastEpipolar,metricPoseAvailable:false},
   visualRotationDiagnostic:{accepted:visualAccepted,rejected:visualRejected,
    imuWindowsPassed:syncPassed,imuWindowsFailed:syncFailed,bothPassed:pairedPassed,
    deferredWindowMatching:true,reasons:{...visualReasons},
    intrinsics:'unverified_nominal',recent:visualHistory},
   keyframeObservations:{last:lastKeyframe,history:keyframeHistory,metricPoseAvailable:false},
   candidateIntrinsicsAB:{checkedFrames:candidateABChecked,
    rotationPassed:{...candidateABRotation},epipolarPassed:{...candidateABEpipolar},
    sampledEveryNFrames:5,matchedImageTracks:true,parallelDiagnosticOnly:true,
    activeIntrinsicsChanged:false,history:candidateABHistory},
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