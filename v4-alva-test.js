/* PAI-VIN v4.0 isolated live AlvaAR 6DoF candidate test.
 * Exact upstream GPLv3 artifact is fetched at CI build time; no JS VO fallback.
 * Raw pose is arbitrary-scale, not metric map position or relocalization.
 */
import {AlvaAR} from './vendor/alva_ar.js';
const {Segments}=globalThis.PAIV4Segments;
const {Trace,render:renderTrace}=globalThis.PAIV4Trace;
const $=id=>document.getElementById(id);
const W=360,H=480,canvas=$('view'),ctx=canvas.getContext('2d',{alpha:false}),video=$('camera');
let stream=null,alva=null,running=false,raf=0,frameNumber=0,attempts=0,valid=0,errors=0,lastRun=0,costs=[],lastPose=null,history=[],startMs=0,featureCountHistory=[],frameLumaHistory=[],frameChangeHistory=[],prevSignature=null,sourceGeometry=null,rawStatuses={ready:0,reset:0,initializing:0,other:0},lastRawStatus=null,poseRawBypassAvailable=false;
const set=t=>$('status').textContent=t;
const segments=new Segments();
const trajectory=new Trace();
const trailCanvas=$('trajectory'),trailCtx=trailCanvas.getContext('2d');
renderTrace(trailCtx,trajectory,trailCanvas.width,trailCanvas.height);
function stop(){
 running=false;cancelAnimationFrame(raf);raf=0;
 if(stream)stream.getTracks().forEach(t=>t.stop());stream=null;video.srcObject=null;
 alva=null;$('start').disabled=false;$('stop').disabled=true;$('export').disabled=history.length===0;
 set('已停止：请导出诊断JSON进行评估。');
}
$('stop').onclick=stop;
function quantile(a,q){if(!a.length)return null;const b=a.slice().sort((x,y)=>x-y);return +b[Math.min(b.length-1,Math.floor((b.length-1)*q))].toFixed(2)}
function poseValid(p){return p&&p.length===16&&Array.from(p).every(Number.isFinite)&&Math.abs(p[15]-1)<.02}
function frameStats(frame){
 const d=frame.data,samples=[],w=frame.width,h=frame.height;
 for(let y=8;y<h-8;y+=16)for(let x=8;x<w-8;x+=16){
  const i=(y*w+x)*4,v=(d[i]*77+d[i+1]*150+d[i+2]*29)>>8;
  samples.push(v);
 }
 const mean=samples.reduce((a,b)=>a+b,0)/Math.max(1,samples.length);
 let change=null;if(prevSignature?.length===samples.length){
  change=samples.reduce((a,v,i)=>a+Math.abs(v-prevSignature[i]),0)/samples.length;
 }
 prevSignature=samples;return {mean:+mean.toFixed(1),change:change===null?null:+change.toFixed(1)};
}
function installStatusProbe(engine){
 // Observe the real native C++ return status without replacing or faking SLAM.
 // Upstream 1=pose ready, 2=reset, 3=initializing.
 if(!engine?.system||typeof engine.system.findCameraPose!=='function')return false;
 const original=engine.system.findCameraPose.bind(engine.system);
 try{
  engine.system.findCameraPose=(...args)=>{
   const status=original(...args);
   lastRawStatus=status;
   const k=status===1?'ready':status===2?'reset':status===3?'initializing':'other';
   rawStatuses[k]++;return status;
  };
  return true;
 }catch(e){return false;}
}
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
  sourceGeometry={sourceWidth:sw,sourceHeight:sh,crop:[+sx.toFixed(1),+sy.toFixed(1),+cw.toFixed(1),+ch.toFixed(1)],visibleFraction:+((cw*ch)/(sw*sh)).toFixed(3)};
  const frame=ctx.getImageData(0,0,W,H),luma=frameStats(frame);
  frameLumaHistory.push(luma.mean);if(luma.change!==null)frameChangeHistory.push(luma.change);
  const begin=performance.now();
  const p=alva.findCameraPose(frame),elapsed=performance.now()-begin;
  attempts++;costs.push(elapsed);if(costs.length>250)costs.shift();
  const ok=poseValid(p);
  if(ok){valid++;lastPose=Array.from(p);$('pose').textContent=JSON.stringify({matrixColumnMajor:lastPose,
    translationArbitraryUnits:lastPose.slice(12,15),metricScaleVerified:false,worldMapReidentified:false},null,2);}
  else{$('pose').textContent='初始化中 / 跟踪暂不可用；不会显示伪位姿';}
  const segment=segments.update(lastRawStatus,ok?p:null,Math.round(t-startMs),frameNumber);
  const tracePoint=trajectory.ingest(lastRawStatus,ok?p:null,Math.round(t-startMs),frameNumber);
  renderTrace(trailCtx,trajectory,trailCanvas.width,trailCanvas.height);
  $('trailState').textContent=tracePoint.valid
    ?'跟踪中 · 轨迹片段 #'+tracePoint.segmentId+' · 本段 '+segment.segmentFrames+' 帧'
    :'位姿不可用 · 已断线，不连接旧轨迹；'+trajectory.lastState;
  const pts=alva.getFramePoints();featureCountHistory.push(pts.length);
  if(!ok){ctx.fillStyle='#76e8aa';
   for(const point of pts){ctx.beginPath();ctx.arc(point.x,point.y,2,0,Math.PI*2);ctx.fill();}
  }
  history.push({elapsedMs:Math.round(t-startMs),frame:frameNumber,poseAvailable:!!ok,
   featureCount:pts.length,rawStatus:lastRawStatus,segmentId:segment.segmentId,
   adjacentStepArbitraryUnits:segment.jump,
   traceRelativeXZ:tracePoint.valid?tracePoint.relativeXZ:null,
   traceHeadingXZ:tracePoint.valid?tracePoint.headingXZ:null,
   imageLumaMean:luma.mean,imageChangeMean:luma.change,processingMs:+elapsed.toFixed(2),
   positionArbitraryUnits:ok?lastPose.slice(12,15):null});
  if(history.length>1200)history.shift();
  $('metrics').textContent='WASM尝试 '+attempts+'帧 · 有效位姿 '+valid+'帧 ('+(100*valid/attempts).toFixed(1)+'%) · P50 '+quantile(costs,.5)+'ms / P95 '+quantile(costs,.95)+'ms · 特征点 '+pts.length+' · 灰度 '+luma.mean+' · 画面变化 '+(luma.change??'—')+' · 保留视野 '+Math.round(sourceGeometry.visibleFraction*100)+'% · 底层状态 '+(lastRawStatus??'未知')+
  ' · 重置 '+rawStatuses.reset+' · 片段 '+segments.count+' · 最长连续 '+segments.maxConsecutive+'帧 · 源 '+sw+'×'+sh+' → '+W+'×'+H;
  const stage=lastRawStatus===2?'引擎内部已重置（检查跟踪点和帧率）':
   lastRawStatus===3?'引擎正在初始化（检查特征数量和实际视差）':
   lastRawStatus===1?'引擎已输出位姿状态':'引擎执行中（原始状态未获取）';
  set(ok?'视觉6DoF候选可用（非米制，非地图重定位）':stage);
 }catch(e){errors++;set('引擎帧处理错误：'+String(e?.message||e));stop();}
}
$('start').onclick=async()=>{
 $('start').disabled=true;set('正在请求摄像头权限…');
 try{
  stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480},frameRate:{ideal:20}}});
  video.srcObject=stream;await video.play();
  set('正在初始化AlvaAR嵌入式WASM，请避免切换页面…');
  alva=await AlvaAR.Initialize(W,H);
  lastRawStatus=null;rawStatuses={ready:0,reset:0,initializing:0,other:0};
  poseRawBypassAvailable=installStatusProbe(alva);
  running=true;lastRun=0;attempts=0;valid=0;errors=0;frameNumber=0;costs=[];history=[];featureCountHistory=[];frameLumaHistory=[];frameChangeHistory=[];prevSignature=null;sourceGeometry=null;lastPose=null;segments.reset();trajectory.reset();renderTrace(trailCtx,trajectory,trailCanvas.width,trailCanvas.height);startMs=performance.now();
  $('stop').disabled=false;$('export').disabled=false;set('WASM已初始化；请缓慢移动手机提供视差');raf=requestAnimationFrame(tick);
 }catch(e){set('启动失败：'+String(e?.message||e));stop();$('status').textContent='启动失败：'+String(e?.message||e);}
};
$('export').onclick=()=>{
 const data={schema:'pai-vin-v4-wasm-alva-diagnostic-v1',engine:'AlvaAR',upstreamCommit:'7796af500ee92001ac2a9888363ff64d7a3bee75',
  build:'v404-alva-live-trace-20261104',createdAt:new Date().toISOString(),frames:attempts,
  validPoseFrames:valid,failedFrames:attempts-valid,errors,
  processingP50Ms:quantile(costs,.5),processingP95Ms:quantile(costs,.95),
  inputResolution:[W,H],sourceGeometry,defaultFovDeg:45,
  upstreamMinTrackedForInit:50,upstreamFrameMaxCellSizePx:40,
  upstreamMaxGridCells:Math.ceil(W/40)*Math.ceil(H/40),
  rawStatuses,rawStatusProbeInstalled:poseRawBypassAvailable,
  continuousPoseSegments:segments.snapshot(),
  trajectoryDisplay:trajectory.snapshot(),
  featureCount:{last:featureCountHistory.at(-1)??null,p50:quantile(featureCountHistory,.5),p95:quantile(featureCountHistory,.95),max:featureCountHistory.length?Math.max(...featureCountHistory):null},
  imageLuma:{p50:quantile(frameLumaHistory,.5)},imageChange:{p50:quantile(frameChangeHistory,.5)},
  cameraFrameTimestampsHardwareVerified:false,metricScaleVerified:false,mapSaved:false,
  persistentRelocalizationVerified:false,imuUsed:false,history};
 const url=URL.createObjectURL(new Blob([JSON.stringify(data)],{type:'application/json'}));const a=document.createElement('a');
 a.href=url;a.download='pai-vin-v4-alva-'+Date.now()+'.json';document.body.appendChild(a);a.click();a.remove();
 setTimeout(()=>URL.revokeObjectURL(url),30000);
};
document.addEventListener('visibilitychange',()=>{if(document.hidden&&running)set('后台暂停取帧，返回后继续（不保证地图状态连续）')});
