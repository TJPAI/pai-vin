/* PAI-VIN V0.6.1: recording requires an explicit, user-initiated video save.
   Recordings stay in memory until replaced or the tab closes. */
(function(global){
'use strict';
const $=id=>document.getElementById(id);
const MAX_MS=120000,MAX_BYTES=140*1024*1024;
let recorder=null,stream=null,chunks=[],bytes=0,started=0,timer=null,stopReason='';
let clip=null,clipUrl=null,manifest=null,saving=false,routeType='mapping';
function status(t){const e=$('slamCaptureStatus');if(e)e.textContent=t}
function mime(){if(!global.MediaRecorder)return '';return ['video/mp4;codecs=avc1.42E01E','video/mp4','video/webm;codecs=vp8','video/webm'].find(m=>MediaRecorder.isTypeSupported(m))||''}
function trigger(blob,name){
 const u=URL.createObjectURL(blob),a=document.createElement('a');
 a.href=u;a.download=name;document.body.appendChild(a);a.click();a.remove();
 setTimeout(()=>URL.revokeObjectURL(u),60000);
}
function updateUI(){
 $('slamStart').disabled=!!recorder;
 $('slamStop').disabled=!recorder;
 $('slamSaveVideo').disabled=!clip;
 $('slamDownloadVideo').disabled=!clip;
 $('slamSaveJson').disabled=!manifest;
}
function finalize(){
 clearInterval(timer);timer=null;
 if(!chunks.length){recorder=null;status('录制失败：没有生成视频数据。请使用系统相机。');updateUI();return}
 const type=recorder?.mimeType||chunks[0].type||'video/mp4';
 const ext=type.includes('webm')?'webm':'mp4';
 const blob=new Blob(chunks,{type});
 const stamp=new Date(started).toISOString().replace(/[:.]/g,'-');
 const base='pai-vin-'+routeType+'-'+stamp;
 if(clipUrl)URL.revokeObjectURL(clipUrl);
 clip={blob,name:base+'.'+ext};clipUrl=URL.createObjectURL(blob);
 manifest={schema:'pai-vin-slam-capture-v2',createdAt:new Date(started).toISOString(),
 durationMs:Date.now()-started,videoFile:clip.name,videoBytes:blob.size,mediaType:type,
 source:'Safari MediaRecorder',routeType,
 metricScaleReference:routeType==='mapping'?'a4-board-v1':null,
 boardCornersMm:routeType==='mapping'?[[25,25],[185,25],[185,272],[25,272]]:null,
 metadataNotice:'This manifest does not prove the video was saved, metric scale estimated, or 3D SLAM performed.'};
 const v=$('slamPreview');v.src=clipUrl;v.style.display='block';v.load();
 $('slamSavedFile').textContent='已生成 '+clip.name+' · '+(blob.size/1048576).toFixed(1)+' MiB。先播放确认，再点击“保存视频到文件”。';
 $('slamSavePanel').hidden=false;
 chunks=[];bytes=0;recorder=null;updateUI();
 status('录像已结束，但视频尚未保存到文件！先播放预览，然后点击“保存视频到文件”。'+(stopReason?'（'+stopReason+'）':''));
}
async function start(){
 if(recorder)return;
 if(clip&&!confirm('上一段视频可能尚未存入「文件」App。确定放弃并开始新录制吗？'))return;
 if(!global.MediaRecorder){status('不支持网页录像，请用系统相机拍摄。');return}
 const type=mime();if(!type){status('无可用编码器，请用系统相机拍摄。');return}
 try{
  stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:1280},height:{ideal:720},frameRate:{ideal:30,max:30}}});
  if(clipUrl){URL.revokeObjectURL(clipUrl);clipUrl=null}
  clip=null;manifest=null;$('slamSavePanel').hidden=true;
  routeType=$('slamCaptureMode').value;
  chunks=[];bytes=0;stopReason='';started=Date.now();
  recorder=new MediaRecorder(stream,{mimeType:type,videoBitsPerSecond:2500000});
  recorder.ondataavailable=e=>{
   if(e.data?.size){chunks.push(e.data);bytes+=e.data.size}
   if(bytes>=MAX_BYTES){stopReason='文件达到容量上限';stop()}
  };
  recorder.onerror=e=>{status('录制发生错误：'+(e.error?.message||'未知错误'));stop()};
  recorder.onstop=()=>{stream?.getTracks().forEach(t=>t.stop());stream=null;finalize()};
  recorder.start(1000);updateUI();
  status('正在录像：完成闭环后点击“停止录制”。录制结束后还必须单独保存视频！');
  timer=setInterval(()=>{
   const n=Math.round((Date.now()-started)/1000);
   status('录像中 '+n+' / 120 秒，约 '+(bytes/1048576).toFixed(1)+' MiB；请保持浏览器前台。');
   if(Date.now()-started>=MAX_MS){stopReason='达到两分钟保护上限';stop()}
  },1000)
 }catch(e){stream?.getTracks().forEach(t=>t.stop());stream=null;recorder=null;updateUI();status('录像启动失败：'+e.message+'；请使用系统相机。')}
}
function stop(){
 if(recorder?.state==='recording'){status('结束录制，正在合成视频…');recorder.stop();updateUI()}
}
async function saveVideo(){
 if(!clip||saving)return;saving=true;
 try{
  const snapshot=clip;
  const file=new File([snapshot.blob],snapshot.name,{type:snapshot.blob.type||'video/mp4'});
  if(navigator.canShare?.({files:[file]})&&navigator.share){
   status('已调出系统分享菜单：请选择“存储到文件”，然后确认存储位置。');
   await navigator.share({files:[file],title:snapshot.name});
   status('系统分享已返回。请打开「文件」App 核对 MP4 是否真实存在且可播放；页面不会自动认定已保存。');
  }else{
   trigger(snapshot.blob,snapshot.name);
   status('已请求 MP4 下载。请到 Safari 下载项和「文件」App 验证文件，不要只看 JSON。');
  }
 }catch(e){
  if(e.name==='AbortError')status('您取消了保存；视频还在当前页面，请重新点击保存。');
  else status('系统分享失败：'+e.message+'。可使用下方“直接下载 MP4”备用入口。');
 }finally{saving=false}
}
function downloadVideo(){
 if(!clip)return;
 trigger(clip.blob,clip.name);
 status('已单独请求下载 '+clip.name+'。请检查 Safari 下载项。');
}
function saveJson(){
 if(!manifest)return;
 const name=clip.name.replace(/\.(mp4|webm)$/i,'.json');
 trigger(new Blob([JSON.stringify(manifest,null,2)],{type:'application/json'}),name);
 status('JSON 已请求下载。注意它不是视频，不能据此认为 MP4 已保存。');
}
function selected(){
 const f=$('slamSystemVideo')?.files?.[0];if(!f)return;
 status('已选择系统相机文件：'+f.name+'（'+(f.size/1048576).toFixed(1)+' MiB）。请到“照片”或“文件”确认原视频保留。');
}
function init(){
 if(!$('slamStart'))return;
 $('slamStart').onclick=start;$('slamStop').onclick=stop;
 $('slamSaveVideo').onclick=saveVideo;$('slamDownloadVideo').onclick=downloadVideo;$('slamSaveJson').onclick=saveJson;
 $('slamSystemVideo').onchange=selected;
 updateUI();
 if(!global.MediaRecorder)status('当前 Safari 不支持网页录像，请使用系统相机拍摄。');
}
if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
global.VINCapture={start,stop};
})(window);