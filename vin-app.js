/* PAI-VIN V2 alpha product shell. A visual map without a verified localization
 * engine MUST NOT output an invented position or path.
 * Local imported JSON is a developer test format, not an authenticity proof. */
(()=>{
'use strict';
const $=id=>document.getElementById(id);
const tabSetup=$('tabSetup'),tabUse=$('tabUse'),setup=$('setup'),use=$('use');
let stream=null,map=null,track=null;
const say=(id,message)=>{$(id).textContent=message};
const selectTab=name=>{
 const showSetup=name==='setup';setup.hidden=!showSetup;use.hidden=showSetup;
 tabSetup.setAttribute('aria-selected',String(showSetup));
 tabUse.setAttribute('aria-selected',String(!showSetup));
 tabSetup.className=showSetup?'':'alt';tabUse.className=showSetup?'alt':'';
 if(!showSetup)draw();
};
tabSetup.onclick=()=>selectTab('setup');tabUse.onclick=()=>selectTab('use');
function emptyCanvas(){
 const c=$('mapCanvas'),x=c.getContext('2d');
 x.fillStyle='#102139';x.fillRect(0,0,c.width,c.height);
 x.strokeStyle='#2c415e';x.lineWidth=1;
 for(let a=0;a<c.width;a+=40){x.beginPath();x.moveTo(a,0);x.lineTo(a,c.height);x.stroke()}
 for(let b=0;b<c.height;b+=40){x.beginPath();x.moveTo(0,b);x.lineTo(c.width,b);x.stroke()}
 x.fillStyle='#a9c2dc';x.font='17px -apple-system,sans-serif';
 x.fillText('尚无可用的米制地图 / 定位轨迹',20,50);
}
function draw(){
 if(map&&track)VINMap.render($('mapCanvas'),map,track.poses);
 else emptyCanvas();
}
function explain(){
 if(!map){say('locNotice','需要先建立经校验的米制地图，才允许绘制实际轨迹。');return}
 say('locNotice','已加载开发格式地图，但尚未连接真实米制相机定位引擎；没有可靠位姿时不画轨迹。');
}
$('mapFile').addEventListener('change',async e=>{
 const f=e.target.files?.[0];if(!f)return;
 try{
  if(f.size>4*1024*1024)throw Error('地图文件大于 4 MB');
  const candidate=JSON.parse(await f.text());
  if(!VINMap.validMap(candidate))throw Error('未通过米制地图结构和尺度标志检查');
  map=candidate;track=new VINMap.Track(map);
  say('mapStatus','已导入开发测试地图：'+map.mapId+' · 楼层 '+map.floor+' · '+map.landmarks.length+' 个地图点。文件声明的米制状态尚需外部独立验证。');
  say('trackCount','0');say('position','尚未定位');explain();draw();
 }catch(err){
  map=null;track=null;draw();explain();
  say('mapStatus','地图拒绝导入：'+err.message);
 }
});
$('startCamera').onclick=async()=>{
 if(stream)return;
 try{
  stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480}}});
  $('camera').srcObject=stream;await $('camera').play();
  $('startCamera').disabled=true;$('stopCamera').disabled=false;
  say('cameraStatus','摄像头已开启，等待真实定位引擎');
 }catch(err){
  if(stream){stream.getTracks().forEach(t=>t.stop());stream=null}
  say('cameraStatus','摄像头权限/启动失败：'+err.message);
 }
};
$('stopCamera').onclick=()=>{
 if(stream)stream.getTracks().forEach(t=>t.stop());stream=null;
 $('camera').srcObject=null;$('startCamera').disabled=false;$('stopCamera').disabled=true;
 say('cameraStatus','摄像头已关闭');
};
window.addEventListener('pagehide',()=>{if(stream)stream.getTracks().forEach(t=>t.stop())});
/* Engine integration contract: local trusted estimator directly calls this
 * with a metric map-aligned position; no UI-generated pose or fake map path. */
window.PAIVIN={acceptVerifiedPose(p){
 if(!track||!stream)return false;
 if(!track.add(p))return false;
 const last=track.poses[track.poses.length-1];
 say('position',last.x.toFixed(2)+' m，'+last.y.toFixed(2)+' m');
 say('trackCount',String(track.poses.length));draw();return true;
},getState:()=>({mapLoaded:!!map,cameraOn:!!stream,trackPoints:track?.poses.length||0})};
// A4 plane reference: explicit user-picked corners; NOT full camera calibration.
let boardStream=null,boardSnapshot=false,boardCorners=[],boardResult=null;
const boardCanvas=$('boardCanvas'),boardCtx=boardCanvas.getContext('2d');
const boardImage=document.createElement('canvas');
boardImage.width=boardCanvas.width;boardImage.height=boardCanvas.height;
const boardImageCtx=boardImage.getContext('2d',{willReadFrequently:true});
function paintBoard(){
 const w=boardCanvas.width,h=boardCanvas.height;
 boardCtx.fillStyle='#102139';boardCtx.fillRect(0,0,w,h);
 if(boardSnapshot)boardCtx.drawImage(boardImage,0,0);
 else{boardCtx.fillStyle='#acc6dd';boardCtx.font='16px -apple-system';boardCtx.fillText('请拍摄 A4 标定纸',16,36)}
 boardCtx.lineWidth=2;boardCtx.strokeStyle='#6ce0b9';boardCtx.fillStyle='#ffcc6b';
 boardCorners.forEach(([x,y],i)=>{
  boardCtx.beginPath();boardCtx.arc(x,y,6,0,Math.PI*2);boardCtx.fill();
  boardCtx.font='18px -apple-system';boardCtx.fillText(String(i+1),x+9,y-8);
 });
 if(boardCorners.length===4){
  boardCtx.beginPath();boardCorners.forEach(([x,y],i)=>i?boardCtx.lineTo(x,y):boardCtx.moveTo(x,y));
  boardCtx.closePath();boardCtx.stroke();
 }
}
$('boardOpen').onclick=async()=>{
 if(boardStream)return;
 try{
  boardStream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:720},height:{ideal:960}}});
  $('boardCamera').srcObject=boardStream;await $('boardCamera').play();
  $('boardSnap').disabled=false;say('boardStatus','相机已开启。请让纸张四角都在画面里，再点击拍摄标定帧。');
 }catch(e){say('boardStatus','标定摄像头启动失败：'+e.message)}
};
$('boardSnap').onclick=()=>{
 const video=$('boardCamera');if(!boardStream||video.readyState<2)return;
 const w=boardImage.width,h=boardImage.height;
 boardImageCtx.fillStyle='#000';boardImageCtx.fillRect(0,0,w,h);
 const vw=video.videoWidth,vh=video.videoHeight;
 // Show full frame by contain, avoiding hidden crop and displaced coordinates.
 const scale=Math.min(w/vw,h/vh),dw=vw*scale,dh=vh*scale;
 boardImageCtx.drawImage(video,(w-dw)/2,(h-dh)/2,dw,dh);
 boardSnapshot=true;boardCorners=[];boardResult=null;$('boardSave').disabled=true;
 boardStream.getTracks().forEach(t=>t.stop());boardStream=null;video.srcObject=null;$('boardSnap').disabled=true;
 paintBoard();say('boardStatus','依次点击标记中心：左上 → 右上 → 右下 → 左下（需要四个）');
};
boardCanvas.addEventListener('click',e=>{
 if(!boardSnapshot||boardCorners.length>=4)return;
 const rect=boardCanvas.getBoundingClientRect();
 const x=(e.clientX-rect.left)*boardCanvas.width/rect.width;
 const y=(e.clientY-rect.top)*boardCanvas.height/rect.height;
 if(!Number.isFinite(x)||!Number.isFinite(y))return;
 boardCorners.push([x,y]);paintBoard();
 if(boardCorners.length===4){
  boardResult=VINBoard.calibrate(boardCorners,boardCanvas.width,boardCanvas.height);
  $('boardSave').disabled=!boardResult.ok;
  say('boardStatus',boardResult.ok?'四角平面几何检测通过，重投影残差 '+boardResult.reprojectionErrorPx.toFixed(2)+' px；可保存平面参照（非 SLAM 尺度）。':
   '标记检测未通过：'+boardResult.reason+'。请重新点选。');
 }
});
$('boardReset').onclick=()=>{boardCorners=[];boardResult=null;$('boardSave').disabled=true;paintBoard();say('boardStatus','已清除角点，请重新依序点选四个中心。')};
$('boardSave').onclick=()=>{
 if(!boardResult?.ok)return;
 try{
  const result={...boardResult,recordedAt:new Date().toISOString()};
  localStorage.setItem('pai-vin-a4-plane-v1',JSON.stringify(result));
  say('boardStatus','A4 平面参照已保存到此浏览器。下一步可扫描环境；尚未完成 SLAM 全局米制标定。');
 }catch(e){say('boardStatus','保存失败：'+e.message)}
};
try{
 const saved=JSON.parse(localStorage.getItem('pai-vin-a4-plane-v1')||'null');
 if(saved?.schema==='pai-vin-a4-plane-v1'&&saved.metricPlaneOnly===true)
  say('boardStatus','检测到此前保存的 A4 平面参照（不等于已完成 SLAM 尺度标定）；可以重新拍摄。');
}catch(e){/* storage may be unavailable */}
window.addEventListener('pagehide',()=>{if(boardStream)boardStream.getTracks().forEach(t=>t.stop())});
paintBoard();

emptyCanvas();explain();
})();