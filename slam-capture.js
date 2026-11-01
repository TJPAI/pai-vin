/* PAI-VIN SLAM dataset capture; does not run SLAM or claim point-cloud output. */
(function(global){
'use strict';
let recorder=null,stream=null,chunks=[],bytes=0,started=0,timer=null,stopReason='',recordCount=0;
const MAX_MS=120000, MAX_BYTES=140*1024*1024;
const $=id=>document.getElementById(id);
function status(t){const e=$('slamCaptureStatus');if(e)e.textContent=t}
function chooseMime(){
 if(!global.MediaRecorder)return '';
 for(const m of ['video/mp4;codecs=avc1.42E01E','video/mp4','video/webm;codecs=vp8','video/webm']){
  if(MediaRecorder.isTypeSupported(m))return m
 }
 return '';
}
function file(blob,name){
 const url=URL.createObjectURL(blob),a=document.createElement('a');
 a.href=url;a.download=name;document.body.appendChild(a);a.click();a.remove();
 setTimeout(()=>URL.revokeObjectURL(url),15000)
}
function finalize(){
 clearInterval(timer);timer=null;
 if(!chunks.length){status('录制无有效视频数据。建议使用下方系统相机拍摄视频。');return}
 const type=recorder.mimeType||chunks[0].type||'video/mp4';
 const ext=type.includes('webm')?'webm':'mp4';
 const blob=new Blob(chunks,{type});
 const stamp=new Date(started).toISOString().replace(/[:.]/g,'-');
 const base='pai-vin-slam-'+stamp;
 file(blob,base+'.'+ext);
 const manifest={schema:'pai-vin-slam-capture-v1',createdAt:new Date(started).toISOString(),
 durationMs:Date.now()-started,videoFile:base+'.'+ext,videoBytes:blob.size,
 mediaType:type,source:'Safari MediaRecorder',routeType:$('slamCaptureMode')?.value||'mapping',
 metricScaleReference:'a4-board-v1',boardCornersMm:[[25,25],[185,25],[185,272],[25,272]],
 metadataNotice:'Video alone does not contain surveyed camera intrinsics, 3D map, SLAM trajectory, or verified metric scale.'};
 file(new Blob([JSON.stringify(manifest,null,2)],{type:'application/json'}),base+'.json');
 status('录制结束并请求保存视频及元数据：'+Math.round(blob.size/1048576)+' MB。'+(stopReason||'')+' 请检查 iPhone 下载列表；文件不会上传服务器。');
 chunks=[];bytes=0;recorder=null;recordCount++;
}
async function start(){
 if(recorder&&recorder.state!=='inactive')return;
 if(!global.MediaRecorder){status('当前 Safari 不支持网页录制，请使用下方系统相机录视频。');return}
 const mime=chooseMime();
 if(!mime){status('浏览器没有可用的视频编码器，请使用系统相机录视频。');return}
 try{
  stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:1280},height:{ideal:720},frameRate:{ideal:30,max:30}}});
  chunks=[];bytes=0;stopReason='';started=Date.now();
  recorder=new MediaRecorder(stream,{mimeType:mime,videoBitsPerSecond:2500000});
  recorder.ondataavailable=e=>{
   if(e.data&&e.data.size){chunks.push(e.data);bytes+=e.data.size}
   if(bytes>=MAX_BYTES){stopReason='达到文件大小保护上限';stop()}
  };
  recorder.onerror=e=>{status('录制出错：'+(e.error?.message||'未知错误'));stop()};
  recorder.onstop=()=>{stream?.getTracks().forEach(t=>t.stop());stream=null;finalize()};
  recorder.start(1000);
  $('slamStart').disabled=true;$('slamStop').disabled=false;
  status('正在采集真实建图视频。请先对准 A4 标定板多角度缓慢拍摄，再沿路线行走并回到起点。');
  timer=setInterval(()=>{
   const sec=Math.round((Date.now()-started)/1000);
   status('正在录制 '+sec+' 秒 / 120 秒；约 '+(bytes/1048576).toFixed(1)+' MB。请勿锁屏或切换 Safari。');
   if(Date.now()-started>=MAX_MS){stopReason='达到两分钟保护上限';stop()}
  },1000)
 }catch(e){stream?.getTracks().forEach(t=>t.stop());stream=null;recorder=null;status('摄像头录制失败：'+e.message+'。可使用系统相机拍摄。')}
}
function stop(){
 if($('slamStart'))$('slamStart').disabled=false;
 if($('slamStop'))$('slamStop').disabled=true;
 if(recorder&&recorder.state!=='inactive'){status('结束录制，正在准备下载…');recorder.stop()}
}
function selected(){
 const input=$('slamSystemVideo');
 if(!input?.files?.length)return;
 const f=input.files[0];
 status('已选择系统相机视频：'+f.name+'（'+(f.size/1048576).toFixed(1)+' MB）。视频保留在 iPhone 本地，稍后可传给 SLAM 处理端。此页面未上传视频。');
}
function init(){
 if(!$('slamStart'))return;
 $('slamStart').onclick=start;$('slamStop').onclick=stop;
 $('slamSystemVideo').onchange=selected;
 $('slamStop').disabled=true;
 if(!global.MediaRecorder)status('网页录像不可用；请使用系统相机视频入口。');
}
if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
global.VINCapture={start,stop};
})(window);