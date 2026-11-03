/* PAI-VIN main UI: A4 physical-plane reference only.
 * No demo pose, no old keyframe scene-map, no arbitrary VO trajectory. */
(()=>{
'use strict';
const $=id=>document.getElementById(id);
let boardStream=null,snapshot=false,corners=[],result=null;
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
 if(boardStream)boardStream.getTracks().forEach(t=>t.stop());
 boardStream=null;$('boardCamera').srcObject=null;$('boardSnap').disabled=true;
}
$('boardOpen').onclick=async()=>{
 if(boardStream)return;
 // Avoid competing rear-camera streams with the VIO sensor probe.
 if(!$('vioStart').disabled){/* VIO is not running */}else return say('请先停止 VIO 采集，再开启标定摄像头');
 try{
  boardStream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:720},height:{ideal:960}}});
  $('boardCamera').srcObject=boardStream;await $('boardCamera').play();
  $('boardSnap').disabled=false;say('标定摄像头已开启。请使标记纸完整进入画面');
 }catch(e){closeCamera();say('相机启动失败：'+e.message)}
};
$('boardSnap').onclick=()=>{
 const v=$('boardCamera');if(!boardStream||v.readyState<2)return;
 const w=snap.width,h=snap.height,dw=v.videoWidth,dh=v.videoHeight;
 if(!dw||!dh)return say('等待摄像头画面');
 const factor=Math.min(w/dw,h/dh),ww=dw*factor,hh=dh*factor;
 snapCtx.fillStyle='#07101d';snapCtx.fillRect(0,0,w,h);
 snapCtx.drawImage(v,(w-ww)/2,(h-hh)/2,ww,hh);
 snapshot=true;corners=[];result=null;
 closeCamera();
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
  say('自动识别四角并保存 A4 平面参照成功；无需手动点选。此结果不是 SLAM 全局尺度标定。');
 }catch(e){draw();say('自动标定未完成：'+e.message+'。请重新拍摄。')}
};
try{
 const saved=JSON.parse(localStorage.getItem('pai-vin-a4-plane-v1')||'null');
 if(saved?.schema==='pai-vin-a4-plane-v1'&&saved.metricPlaneOnly===true)say('已读取之前保存的 A4 平面参照；可重新拍摄');
}catch(e){}
window.addEventListener('pagehide',closeCamera);
window.PAIVIN={stopBoardCamera:closeCamera};
draw();
})();