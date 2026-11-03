/* PAI-VIN VIO acquisition probe. Browser data are observations, not pose.
 * Produces a privacy-conscious sensor quality report and an engine status.
 * Future WASM estimator consumes the same monotonic sensor transport.
 */
(function(){
 'use strict';
 const $=id=>document.getElementById(id);
 const buffer=new PAIVioSensor.SensorBuffer(5000);
 const tracker=new PAIVioTracks.Tracker({maxTracks:90});
 const trackCanvas=document.createElement('canvas');trackCanvas.width=256;trackCanvas.height=192;
 const trackContext=trackCanvas.getContext('2d',{willReadFrequently:true});
 let lastTrackingTime=-Infinity,lastTrackingReport=null;
 let stream=null,timer=null,listener=null,framesSeen=0;
 const show=s=>{$('vioStatus').textContent=s};
 function report(){
  const r=buffer.report(),last=buffer.frames.at(-1),first=buffer.frames[0];
  const win=(last&&first&&last.t>first.t)?buffer.between(Math.max(first.t,last.t-1000),last.t):{ok:false,reason:'need_frames'};
  $('vioNumbers').textContent='视频帧 '+r.cameraFrames+' · IMU '+r.imuSamples+' · 帧率 '+r.cameraRateHz.toFixed(1)+' Hz · IMU '+r.imuRateHz.toFixed(1)+' Hz · 同步窗 '+(win.ok?'可用':'不可用 '+win.reason);
  const q=lastTrackingReport;
  $('vioTrackNumbers').textContent=q?'图像跟踪点 '+q.total+' · 连续继承 '+q.retained+' · 5帧以上 '+q.longLived+' · 丢失 '+q.lost+' · 平均寿命 '+q.avgAge.toFixed(1)+' 帧':'尚未获得连续视觉特征';
  return {r,win};
 }
 function sampleFrame(){
  const v=$('vioVideo');if(!stream||v.readyState<2)return;
  // No image retention or synthesized track.
  const t=performance.now();buffer.addFrame({t,width:v.videoWidth,height:v.videoHeight});
  framesSeen++;
  if(t-lastTrackingTime>=200){
   lastTrackingTime=t;
   try{
    trackContext.drawImage(v,0,0,256,192);
    const data=trackContext.getImageData(0,0,256,192).data;
    const gray=VINFeatures.grayRGBA(data);
    lastTrackingReport=tracker.update(gray,t);
    const output=$('vioTrackCanvas'),ctx=output.getContext('2d');
    ctx.clearRect(0,0,256,192);
    ctx.strokeStyle='#67ddb5';ctx.fillStyle='#67ddb5';ctx.lineWidth=1.1;
    for(const p of tracker.tracks){
     ctx.beginPath();ctx.arc(p.x,p.y,p.age>=5?3:1.5,0,Math.PI*2);ctx.stroke();
    }
   }catch(e){$('vioTrackNumbers').textContent='视觉跟踪异常：'+e.message}
  }
  report();
 }
 async function start(){
  if(stream)return;
  buffer.reset();tracker.reset();framesSeen=0;lastTrackingTime=-Infinity;lastTrackingReport=null;
  try{
   if(typeof DeviceMotionEvent!=='undefined'&&typeof DeviceMotionEvent.requestPermission==='function'){
    const p=await DeviceMotionEvent.requestPermission();
    if(p!=='granted')throw Error('IMU 权限未授权');
   }
   listener=e=>{const s=PAIVioSensor.fromDeviceMotion(e,performance.now());if(s)buffer.addImu(s)};
   window.addEventListener('devicemotion',listener);
   stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480},frameRate:{ideal:20}}});
   const video=$('vioVideo');video.srcObject=stream;await video.play();
   timer=setInterval(sampleFrame,100);
   $('vioStart').disabled=true;$('vioStop').disabled=false;
   show('正在采集：仅验证摄像头与 IMU 的数据质量。相机曝光时间戳、相机—IMU 外参及完整 VIO 尚未校准。');
  }catch(e){stop();show('启动失败：'+e.message)}
 }
 function stop(){
  clearInterval(timer);timer=null;if(stream)stream.getTracks().forEach(t=>t.stop());stream=null;
  if(listener)window.removeEventListener('devicemotion',listener);listener=null;
  $('vioVideo').srcObject=null;$('vioStart').disabled=false;$('vioStop').disabled=true;
  report();
 }
 function download(){
  const {r,win}=report();
  const data={schema:'pai-vin-vio-acquisition-v1',capturedAt:new Date().toISOString(),browserClock:'performance.now',
   cameraExposureTimestampAvailable:false,deviceIMUExtrinsicsCalibrated:false,wasmVioEstimatorConnected:false,
   metricPoseAvailable:false,containsPhotos:false,containsFrames:false,
   transport:r,window:win.ok?{ok:true,dt:win.dt,maxGapMs:win.maxGapMs}:win,
   visualTracking:lastTrackingReport?{total:lastTrackingReport.total,retained:lastTrackingReport.retained,longLived:lastTrackingReport.longLived,avgAge:lastTrackingReport.avgAge,medianError:lastTrackingReport.medianError}:null,
   samples:buffer.imu.slice(-300).map(({t,gx,gy,gz,ax,ay,az})=>({t,gx,gy,gz,ax,ay,az}))};
  const url=URL.createObjectURL(new Blob([JSON.stringify(data)],{type:'application/json'}));
  const a=document.createElement('a');a.href=url;a.download='pai-vin-vio-sensors-'+Date.now()+'.json';
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),30000);
 }
 $('vioStart').onclick=start;$('vioStop').onclick=stop;$('vioExport').onclick=download;
 window.addEventListener('pagehide',stop);
 report();
})();