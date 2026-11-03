/* PAI-VIN v4.0 isolated live AlvaAR 6DoF candidate test.
 * Exact upstream GPLv3 artifact is fetched at CI build time; no JS VO fallback.
 * Raw pose is arbitrary-scale, not metric map position or relocalization.
 */
import {AlvaAR} from './vendor/alva_ar.js';
const $=id=>document.getElementById(id);
const W=320,H=240,canvas=$('view'),ctx=canvas.getContext('2d',{alpha:false}),video=$('camera');
let stream=null,alva=null,running=false,raf=0,frameNumber=0,attempts=0,valid=0,errors=0,lastRun=0,costs=[],lastPose=null,history=[],startMs=0;
const set=t=>$('status').textContent=t;
function stop(){
 running=false;cancelAnimationFrame(raf);raf=0;
 if(stream)stream.getTracks().forEach(t=>t.stop());stream=null;video.srcObject=null;
 alva=null;$('start').disabled=false;$('stop').disabled=true;$('export').disabled=history.length===0;
 set('已停止：请导出诊断JSON进行评估。');
}
$('stop').onclick=stop;
function quantile(a,q){if(!a.length)return null;const b=a.slice().sort((x,y)=>x-y);return +b[Math.min(b.length-1,Math.floor((b.length-1)*q))].toFixed(2)}
function poseValid(p){return p&&p.length===16&&Array.from(p).every(Number.isFinite)&&Math.abs(p[15]-1)<.02}
function tick(t){
 if(!running)return;
 raf=requestAnimationFrame(tick);
 if(document.hidden||video.readyState<2||t-lastRun<100)return;
 lastRun=t;frameNumber++;
 try{
  const sw=video.videoWidth,sh=video.videoHeight;if(!sw||!sh)return;
  // Equal-aspect source crop is essential; never stretch portrait video.
  const scale=Math.max(W/sw,H/sh),cw=W/scale,ch=H/scale,sx=(sw-cw)/2,sy=(sh-ch)/2;
  ctx.drawImage(video,sx,sy,cw,ch,0,0,W,H);
  const frame=ctx.getImageData(0,0,W,H),begin=performance.now();
  const p=alva.findCameraPose(frame),elapsed=performance.now()-begin;
  attempts++;costs.push(elapsed);if(costs.length>250)costs.shift();
  const ok=poseValid(p);
  if(ok){valid++;lastPose=Array.from(p);$('pose').textContent=JSON.stringify({matrixColumnMajor:lastPose,
    translationArbitraryUnits:lastPose.slice(12,15),metricScaleVerified:false,worldMapReidentified:false},null,2);}
  else{$('pose').textContent='初始化中 / 跟踪暂不可用；不会显示伪位姿';}
  if(!ok){
   const pts=alva.getFramePoints();ctx.fillStyle='#76e8aa';
   for(const point of pts){ctx.beginPath();ctx.arc(point.x,point.y,2,0,Math.PI*2);ctx.fill();}
  }
  history.push({elapsedMs:Math.round(t-startMs),frame:frameNumber,poseAvailable:!!ok,processingMs:+elapsed.toFixed(2),
   positionArbitraryUnits:ok?lastPose.slice(12,15):null});
  if(history.length>1200)history.shift();
  $('metrics').textContent='WASM尝试 '+attempts+'帧 · 有效位姿 '+valid+'帧 ('+(100*valid/attempts).toFixed(1)+'%) · P50 '+quantile(costs,.5)+'ms / P95 '+quantile(costs,.95)+'ms · 源 '+sw+'×'+sh+' → '+W+'×'+H;
  set(ok?'视觉6DoF候选可用（非米制，非地图重定位）':'引擎已运行：正在初始化/跟踪恢复');
 }catch(e){errors++;set('引擎帧处理错误：'+String(e?.message||e));stop();}
}
$('start').onclick=async()=>{
 $('start').disabled=true;set('正在请求摄像头权限…');
 try{
  stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480},frameRate:{ideal:20}}});
  video.srcObject=stream;await video.play();
  set('正在初始化AlvaAR嵌入式WASM，请避免切换页面…');
  alva=await AlvaAR.Initialize(W,H);
  running=true;lastRun=0;attempts=0;valid=0;errors=0;frameNumber=0;costs=[];history=[];lastPose=null;startMs=performance.now();
  $('stop').disabled=false;$('export').disabled=false;set('WASM已初始化；请缓慢移动手机提供视差');raf=requestAnimationFrame(tick);
 }catch(e){set('启动失败：'+String(e?.message||e));stop();$('status').textContent='启动失败：'+String(e?.message||e);}
};
$('export').onclick=()=>{
 const data={schema:'pai-vin-v4-wasm-alva-diagnostic-v1',engine:'AlvaAR',upstreamCommit:'7796af500ee92001ac2a9888363ff64d7a3bee75',
  build:'v400-alva-isolated-20261104',createdAt:new Date().toISOString(),frames:attempts,
  validPoseFrames:valid,failedFrames:attempts-valid,errors,
  processingP50Ms:quantile(costs,.5),processingP95Ms:quantile(costs,.95),
  inputResolution:[W,H],cameraFrameTimestampsHardwareVerified:false,metricScaleVerified:false,mapSaved:false,
  persistentRelocalizationVerified:false,imuUsed:false,history};
 const url=URL.createObjectURL(new Blob([JSON.stringify(data)],{type:'application/json'}));const a=document.createElement('a');
 a.href=url;a.download='pai-vin-v4-alva-'+Date.now()+'.json';document.body.appendChild(a);a.click();a.remove();
 setTimeout(()=>URL.revokeObjectURL(url),30000);
};
document.addEventListener('visibilitychange',()=>{if(document.hidden&&running)set('后台暂停取帧，返回后继续（不保证地图状态连续）')});
