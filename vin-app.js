/* PAI-VIN main UI: A4 physical-plane reference only.
 * No demo pose, no old keyframe scene-map, no arbitrary VO trajectory. */
(()=>{
'use strict';
const $=id=>document.getElementById(id);
let boardStream=null,snapshot=false,corners=[],result=null;
 let calibrationStreamOwned=false;
 const focalSamples=[];
 const focalStatus=$('boardFocalStatus');
 const focalSay=message=>{if(focalStatus)focalStatus.textContent=message};
 let focalLastResult=null;
 let boardSourceGeometry=null;
 function exportFocal(){
  const data={schema:'pai-vin-focal-capture-v1',appVersion:$('appVersion')?.dataset.version||'unknown',
   recordedAt:new Date().toISOString(),imageWidth:snap.width,imageHeight:snap.height,
   boardMarkerSpacingM:[.16,.247],boardSourceGeometry,samples:focalSamples.map(x=>x.corners.map(p=>p.slice())),
   estimate:focalLastResult,containsPhotos:false,containsFrames:false,
   warning:'Unverified constrained focal candidate; not full calibration or metric SLAM'};
  const url=URL.createObjectURL(new Blob([JSON.stringify(data)],{type:'application/json'}));
  const a=document.createElement('a');a.href=url;a.download='pai-vin-focal-'+Date.now()+'.json';
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),30000);
 }
 $('boardExport').onclick=exportFocal;
 focalSay('本次会话已有0个视角；请从不同角度拍摄，同一视角不会重复计数。');
const canvas=$('boardCanvas'),ctx=canvas.getContext('2d');
const snap=document.createElement('canvas');snap.width=canvas.width;snap.height=canvas.height;
const snapCtx=snap.getContext('2d');
const say=t=>{$('boardStatus').textContent=t};
function draw(){
 const w=canvas.width,h=canvas.height;
 ctx.fillStyle='#102139';ctx.fillRect(0,0,w,h);
 if(snapshot)ctx.drawImage(snap,0,0);
 else{ctx.fillStyle='#c5d9e9';ctx.font='16px -apple-system,sans-serif';ctx.fillText('请拍摄 A4 标定纸',15,33)}
 ctx.fillStyle='#ffce77';ctx.strokeStyle='#72e4b3';ctx.lineWidth=2;
 for(let i=0;i<corners.length;i++){
  const [x,y]=corners[i];ctx.beginPath();ctx.arc(x,y,6,0,Math.PI*2);ctx.fill();
  ctx.font='17px sans-serif';ctx.fillText(String(i+1),x+9,y-5);
 }
 if(corners.length===4){ctx.beginPath();corners.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.stroke()}
}
function closeCamera(){
 if(boardStream&&calibrationStreamOwned)boardStream.getTracks().forEach(t=>t.stop());
 calibrationStreamOwned=false;
 boardStream=null;$('boardCamera').srcObject=null;$('boardSnap').disabled=true;
}
function handoffCamera(){
 if(!boardStream||!boardStream.getVideoTracks().some(t=>t.readyState==='live'))return null;
 const transfer={stream:boardStream,boardGeometry:boardSourceGeometry,
  focal:focalLastResult?.ok?{...focalLastResult}:null};
 boardStream=null;calibrationStreamOwned=false;
 $('boardCamera').srcObject=null;$('boardSnap').disabled=true;
 return transfer;
}
$('boardOpen').onclick=async()=>{
 if(boardStream)return;
 // Avoid competing rear-camera streams with the VIO sensor probe.
 if(!$('vioStart').disabled){/* VIO is not running */}else return say('请先停止 VIO 采集，再开启标定摄像头');
 try{
  boardStream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480},frameRate:{ideal:20}}});
  calibrationStreamOwned=true;
  $('boardCamera').srcObject=boardStream;await $('boardCamera').play();
  $('boardSnap').disabled=false;say('标定摄像头已开启。可连续拍摄多视角；结束后直接点击开始 VIO 采集，复用同一摄像头会话。');
 }catch(e){closeCamera();say('相机启动失败：'+e.message)}
};
$('boardSnap').onclick=()=>{
 const v=$('boardCamera');if(!boardStream||v.readyState<2)return;
 const w=snap.width,h=snap.height,dw=v.videoWidth,dh=v.videoHeight;
 if(!dw||!dh)return say('等待摄像头画面');
 const geometry=PAIVioViewport.rect(dw,dh,w,h,'contain');
 if(!geometry.ok)return say('视频图像几何无效：'+geometry.reason);
 boardSourceGeometry=geometry;
 const factor=geometry.scaleX,ww=dw*factor,hh=dh*factor;
 snapCtx.fillStyle='#07101d';snapCtx.fillRect(0,0,w,h);
 snapCtx.drawImage(v,(w-ww)/2,(h-hh)/2,ww,hh);
 snapshot=true;corners=[];result=null;
 // Preserve live camera stream across calibration snapshots.
 try{
  const pixels=snapCtx.getImageData(0,0,w,h).data;
  const detected=VINBoardDetect.detect(pixels,w,h);
  if(!detected.ok){
   draw();say('自动识别失败（'+detected.reason+'），请调整角度、距离或光线，再次拍摄。');
   return;
  }
  const candidate=VINBoard.calibrate(detected.corners,w,h);
  if(!candidate.ok){draw();say('标定几何校验未通过（'+candidate.reason+'），请重拍');return}
  corners=detected.corners;result=candidate;draw();
  localStorage.setItem('pai-vin-a4-plane-v1',JSON.stringify({
   ...result,recordedAt:new Date().toISOString(),autoDetected:true,
   detectedMarkerCount:4,detector:detected.method
  }));
  // Multi-view candidate: no image is stored. Require varied perspectives.
  const existing=focalSamples.some(p=>Math.sqrt(p.corners.reduce((a,v,i)=>a+(v[0]-corners[i][0])**2+(v[1]-corners[i][1])**2,0)/4)<12);
  if(!existing&&focalSamples.length<12)focalSamples.push({corners:corners.map(p=>p.slice())});
  const estimate=PAIVioFocal.estimate(focalSamples.map(p=>p.corners),w,h);
  focalLastResult=estimate;
  // Do not let an old candidate silently survive a new, invalid session.
  localStorage.removeItem('pai-vin-focal-candidate-v1');
  if(estimate.ok){
   localStorage.setItem('pai-vin-focal-candidate-v1',JSON.stringify({...estimate,boardSourceGeometry,recordedAt:new Date().toISOString()}));
   const q=estimate.quality;
   const label=q?.status==='review_capture'?'视角稳定性待复核':
    q?.status==='model_limited'?'焦距稳定，受限模型拟合偏差':'候选一致性较好';
   const reasons={limited_perspective_diversity:'倾斜角差异不足',
    leave_one_out_focal_variation_gt_1pct:'留一法焦距波动超过1%',
    in_sample_reprojection_gt_1px:'模型内RMS超过1px',
    individual_view_fit_gt_1_8px:'有单视角拟合超过1.8px'};
   const radial=estimate.radialDistortionCandidate;
   const radialInfo=radial?.status==='cross_view_candidate_improved'?
    ' · 径向k1候选 '+radial.candidateK1.toFixed(3)+
    '，留一视角几何约束改善 '+(100*radial.relativeImprovement).toFixed(1)+'%（仍未验证）':
    ' · 径向畸变候选尚无可靠的≥10%跨视角改善证据';
   focalSay('标定画布640×480；候选焦距 fx='+estimate.fx.toFixed(1)+'px · 质量提示：'+label+
    (q?.reasons?.length?'（'+q.reasons.map(x=>reasons[x]||x).join('、')+'）':'')+
    ' · 模型内重投影 RMS '+estimate.reprojectionRmsPx.toFixed(2)+'px · 逐视角最大 '+estimate.maxViewReprojectionRmsPx.toFixed(2)+'px · 留一法波动 '+(100*estimate.leaveOneOutRelativeSpread).toFixed(1)+'%'+radialInfo+'。当前畸变仅诊断未用于VIO；四角约束并非独立像素重投影验证，仍需完整内参与外参标定。');
   say('A4平面参照已保存；多视角焦距候选已产生。'+
    (q?.status==='review_capture'?'留一法焦距变化偏大，请补充不同倾斜方向的视角。':
     q?.status==='model_limited'?'留一法焦距较稳定，当前主要是针孔模型拟合误差，继续增加照片未必改善；不需要反复拍摄。':
     '候选一致性较好，无需额外拍摄；内参仍待独立验证。')+
     '尚不能启用正式视觉融合。');
  }else{
   focalSay('多视角 '+focalSamples.length+'/5（最多12组）：'+estimate.reason+(existing?'。重复视角未计数。':'。请改变A4倾斜方向。')+'；可导出本次角点供分析。');
   say('A4平面参照已保存；单张照片已完成平面参照，不等于内参标定。');
  }
 }catch(e){draw();say('自动标定未完成：'+e.message+'。请重新拍摄。')}
};
try{
 const saved=JSON.parse(localStorage.getItem('pai-vin-a4-plane-v1')||'null');
 if(saved?.schema==='pai-vin-a4-plane-v1'&&saved.metricPlaneOnly===true)say('已读取之前保存的 A4 平面参照；可重新拍摄');
}catch(e){}
window.addEventListener('pagehide',closeCamera);
window.PAIVIN={stopBoardCamera:closeCamera,handoffCamera};
draw();
})();