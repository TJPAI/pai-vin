/* PAI-VIN browser-only visual mapping and second-visit place recognition.
 * No metric position, 6DoF claim, or camera frame upload.
 * Camera frames are processed at 256x192 and immediately discarded. */
(function(root){
 'use strict';
 const $=id=>document.getElementById(id);
 const canvas=document.createElement('canvas');canvas.width=VINFeatures.W;canvas.height=VINFeatures.H;
 const context=canvas.getContext('2d',{willReadFrequently:true});
 let graph=new VINLandmarks.Graph(),sketch=new VINPath.Sketch(),scanStream=null,scanTimer=null,recTimer=null,db=null;
 let scanBusy=false,recBusy=false,lastPlace=-1,confirmed=0,recognitions=[];
 const say=(id,t)=>{if($(id))$(id).textContent=t};
 const buttons=()=>{if($('scanStart'))$('scanStart').disabled=!!scanStream;
  if($('scanStop'))$('scanStop').disabled=!scanStream};
 function frame(video){
  if(!video||video.readyState<2)return null;
  context.drawImage(video,0,0,canvas.width,canvas.height);
  const bytes=context.getImageData(0,0,canvas.width,canvas.height).data;
  return VINFeatures.detect(VINFeatures.grayRGBA(bytes));
 }
 function openDatabase(){
  return new Promise((resolve,reject)=>{
   const req=indexedDB.open('pai-vin-visual-graph-v1',1);
   req.onupgradeneeded=()=>req.result.createObjectStore('graphs');
   req.onsuccess=()=>{db=req.result;resolve()};
   req.onerror=()=>reject(req.error||Error('IndexedDB not available'));
  });
 }
 function transact(method,...args){
  if(!db)return Promise.reject(Error('IndexedDB not ready'));
  return new Promise((resolve,reject)=>{
   const tx=db.transaction('graphs',method==='get'?'readonly':'readwrite');
   const req=tx.objectStore('graphs')[method](...args);
   req.onsuccess=()=>resolve(req.result);
   req.onerror=()=>reject(req.error||Error('Storage failure'));
  });
 }
 async function save(){await transact('put',{...graph.serialize(),voSketch:sketch.serialize()},'current')}
 async function load(){
  const result=await transact('get','current');
  if(result){graph=new VINLandmarks.Graph(result);try{sketch=new VINPath.Sketch(result.voSketch)}catch(e){sketch=new VINPath.Sketch()}showGraph();say('scanStatus','已恢复本机视觉地图：'+graph.frames.length+' 关键帧。此图尚无米制坐标。')}
 }
 function showGraph(active=-1){
  VINLandmarks.render($('visualGraph'),graph,active);
  VINPath.render($('voCanvas'),sketch);
  say('voScanStatus','可信相对运动段：'+(sketch.nodes.length-1)+'；不表示米制距离，丢失段不推算。');
  if($('recognitionQuality')&&graph.frames.length===0)say('recognitionQuality','没有保存的视觉关键帧');
 }
 async function scanTick(){
  if(scanBusy||!scanStream)return;
  scanBusy=true;
  try{
   const pts=frame($('scanCamera'));
   if(!pts)return;
   const r=graph.add(pts,Date.now());
   if(r.added){
    if(graph.frames.length>=2){
      const prev=graph.frames[graph.frames.length-2];
      const step=VINPath.parseStep(prev.points,pts,VINGeometry,VINVO);
      if(step.ok)sketch.append(step);else sketch.rejected++;
    }
    showGraph(graph.frames.length-1);
    await save();
   }
   let m=r.added?'已保存第 '+r.count+' 张视觉关键帧':
    r.reason==='same_view'?'画面与前一关键帧相似，请继续缓慢移动':
    r.reason==='low_texture'?'纹理不足，请对准桌椅、门框等':'地图已达容量限制，请结束扫描';
   if(graph.closed)m+='；检测到首批画面回访候选（非几何回环优化）';
   say('scanStatus',m);
   if(r.reason==='map_full')stopScan();
  }catch(e){say('scanStatus','扫描错误：'+e.message)}
  finally{scanBusy=false}
 }
 async function startScan(){
  if(scanStream)return;
  let board=null;
  try{board=JSON.parse(localStorage.getItem('pai-vin-a4-plane-v1')||'null')}catch(e){}
  if(board?.schema!=='pai-vin-a4-plane-v1'||board?.metricPlaneOnly!==true){
   say('scanStatus','请先完成上方 A4 四角平面标定并保存，再进行环境扫描。');return;
  }
  stopRecognize();
  try{
   scanStream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480}}});
   $('scanCamera').srcObject=scanStream;await $('scanCamera').play();
   buttons();say('scanStatus','摄像头已启动。扫描区域并回到起点，特征仅保存在本机。');
   scanTick();scanTimer=setInterval(scanTick,1200);
  }catch(e){
   if(scanStream)scanStream.getTracks().forEach(t=>t.stop());
   scanStream=null;buttons();say('scanStatus','摄像头启动失败：'+e.message);
  }
 }
 async function stopScan(){
  clearInterval(scanTimer);scanTimer=null;
  if(scanStream)scanStream.getTracks().forEach(t=>t.stop());
  scanStream=null;$('scanCamera').srcObject=null;buttons();
  try{if(db)await save()}catch(e){say('scanStatus','保存失败：'+e.message);return}
  say('scanStatus','扫描停止；本机保存 '+graph.frames.length+' 张视觉关键帧。'+(graph.closed?'检测到候选回访。':'尚无明确回访首批画面证据。')+' 未计算米制二维轨迹。');
 }
 async function clear(){
  if(scanStream)return say('scanStatus','请先停止扫描');
  if(!confirm('确定清空已保存的视觉关键帧地图？'))return;
  graph=new VINLandmarks.Graph();sketch=new VINPath.Sketch();recognitions=[];lastPlace=-1;confirmed=0;
  try{await transact('delete','current');showGraph();say('scanStatus','视觉地图已清空。')}
  catch(e){say('scanStatus','清空失败：'+e.message)}
 }
 function exportMap(){
  if(!graph.frames.length)return say('scanStatus','目前没有可导出的视觉地图');
  const data=new Blob([JSON.stringify({...graph.serialize(),voSketch:sketch.serialize()})],{type:'application/json'});
  const url=URL.createObjectURL(data),a=document.createElement('a');
  a.href=url;a.download='pai-vin-visual-graph-'+new Date().toISOString().slice(0,10)+'.json';
  document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),30000);
  say('scanStatus','已请求下载视觉关键帧 JSON；文件非米制二维地图，不包含原始图像。');
 }
 function recognize(video){
  if(recBusy||!graph.frames.length)return;
  recBusy=true;
  try{
   const pts=frame(video);
   if(!pts||pts.length<18){lastPlace=-1;confirmed=0;say('recognitionStatus','纹理不足');return}
   const result=graph.match(pts),best=result.best;
   const accepted=result.accepted&&best&&best.inliers>=11;
   if(accepted){
    if(lastPlace===best.index)confirmed++;else{lastPlace=best.index;confirmed=1}
   }else{lastPlace=-1;confirmed=0}
   if(accepted&&confirmed>=2){
    const place=best.index+1;
    say('recognitionStatus','识别到保存的关键帧 #'+place+'（仅场景识别）');
    if(recognitions.at(-1)!==place){
     recognitions.push(place);recognitions=recognitions.slice(-10);
     say('recognitionSequence',recognitions.map(x=>'#'+x).join(' → '));
    }
    showGraph(best.index);
   }else{
    say('recognitionStatus',result.ambiguous?'场景歧义，请换个视角':accepted?'找到候选，等待连续确认':'没有可信视觉匹配');
   }
   say('recognitionQuality',(best?.inliers||0)+' 内点，'+Math.round((best?.ratio||0)*100)+'% 比率；不代表米制定位');
  }catch(e){say('recognitionStatus','识别失败：'+e.message)}
  finally{recBusy=false}
 }
 function startRecognize(video){
  stopRecognize();lastPlace=-1;confirmed=0;recognitions=[];
  say('recognitionSequence','—');
  if(!graph.frames.length){say('recognitionStatus','尚无保存的视觉地图，先完成首次扫描');return}
  say('recognitionStatus','开始本机视觉场景重识别');
  recognize(video);recTimer=setInterval(()=>recognize(video),1600);
 }
 function stopRecognize(){clearInterval(recTimer);recTimer=null;lastPlace=-1;confirmed=0}
 function init(){
  $('scanStart').onclick=startScan;$('scanStop').onclick=stopScan;
  $('scanExport').onclick=exportMap;$('scanClear').onclick=clear;
  showGraph();buttons();
  openDatabase().then(load).catch(e=>say('scanStatus','浏览器本地地图不可用：'+e.message));
  window.addEventListener('pagehide',()=>{stopRecognize();if(scanStream)scanStream.getTracks().forEach(t=>t.stop())});
 }
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
 root.VINScan={startRecognize,stopRecognize,getFrameCount:()=>graph.frames.length};
})(window);