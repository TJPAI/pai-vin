/* PAI-VIN Safari-only experimental keyframe recognition.
   Gradient corners + deterministic binary intensity descriptors + geometric voting.
   Not 6DoF SLAM, no actual metric position. No remote transmission. */
(()=>{
'use strict';
const $=id=>document.getElementById(id);
const W=256,H=192,MAX=60,INTERVAL=1050,TRACK_INTERVAL=240;
const cv=document.createElement('canvas');cv.width=W;cv.height=H;
const ctx=cv.getContext('2d',{willReadFrequently:true});
let stream=null,mode='',timer=null,trackTimer=null,busy=false,frames=[],db=null,imu=null;
let lastTrackPoints=null,trackFrames=0,trackGood=0,lastTrackTime=0,visualMotion=null,voAttempt=0;
let queries=0,hits=0,attempts=0,lastCandidate=-1,candidateStreak=0,lastConfirmed=-1,route=[];
const set=(id,v)=>{const el=$(id);if(el)el.textContent=v};
const status=v=>set('status',v);
function dbOpen(){
 return new Promise((ok,fail)=>{
  const req=indexedDB.open('pai-vin-web-vio-v1',1);
  req.onupgradeneeded=()=>{if(!req.result.objectStoreNames.contains('map'))req.result.createObjectStore('map')};
  req.onsuccess=()=>{db=req.result;ok(db)};
  req.onerror=()=>fail(req.error);
 });
}
function transaction(kind,method,...args){
 return new Promise((ok,fail)=>{
  const tx=db.transaction('map',kind),request=tx.objectStore('map')[method](...args);
  request.onsuccess=()=>ok(request.result);request.onerror=()=>fail(request.error);
 });
}
function features(){
 if(!stream||$('camera').readyState<2)return [];
 ctx.drawImage($('camera'),0,0,W,H);
 const raw=ctx.getImageData(0,0,W,H).data;
 return VINFeatures.detect(VINFeatures.grayRGBA(raw));
}
function showMotion(v){
 if(!v){set('trackQuality','等待画面');set('visualDelta','—');return}
 set('trackQuality',v.accepted?'稳定跟踪 · '+v.inliers+' 内点':'跟踪不足 · '+v.inliers+' 内点');
 set('visualDelta',v.accepted?('视觉帧间旋转 '+v.angle.toFixed(1)+'° · 图像尺度 ×'+v.scale.toFixed(3)):'旋转/尺度不可置信');
 set('trackRate',trackGood+'/'+trackFrames);
}
function track(){
 if(!stream||!mode||busy||$('camera').readyState<2)return;
 try{
  const now=performance.now(),pts=features();
  if(lastTrackPoints&&lastTrackPoints.length>=8&&pts.length>=8){
   const v=VINGeometry.verify(lastTrackPoints,pts);
   if(v.accepted)trackGood++;
   trackFrames++;visualMotion=v;showMotion(v);
   // Relative 3D pose is intentionally evaluated at a slower rate on iPhone.
   if(++voAttempt%4===0){
    const pairs=VINGeometry.correspond(lastTrackPoints,pts);
    const pose=VINVO.estimate(lastTrackPoints,pts,{pairs,focal:220,iterations:36});
    if(pose.ok){
     set('voPose','旋转 '+pose.rotationDeg.toFixed(1)+'° · 平移方向 ['+pose.t.map(n=>n.toFixed(2)).join(', ')+']');
     set('voQuality',pose.inliers+'/'+pose.matches+' 极线内点；未知真实尺度');
    }else{
     set('voPose','相对位姿不可可靠确定');
     set('voQuality',(pose.inliers||0)+'/'+(pose.matches||0)+' 极线内点 · '+pose.reason);
    }
   }
  }else{set('trackQuality','纹理不足，请寻找有边缘的区域')}
  lastTrackPoints=pts;lastTrackTime=now;
 }catch(e){set('trackQuality','跟踪错误：'+e.message)}
}
function drawMap(active=-1){
 set('count',frames.length);
 const holder=$('timeline');holder.replaceChildren();
 for(let i=0;i<frames.length;i++){
  const div=document.createElement('div');div.className='mark'+(i===active?' active':'');div.textContent=i+1;
  holder.appendChild(div);
 }
 if(active>=0&&holder.children[active])holder.children[active].scrollIntoView({block:'nearest',inline:'nearest'});
}
function toggleButtons(){
 $('startMap').disabled=!!mode||!stream;
 $('startLocalize').disabled=!!mode||!stream||frames.length===0;
 $('stopMap').disabled=mode!=='mapping';
 $('stopLocalize').disabled=mode!=='localization';
 $('openCamera').disabled=!!mode;
 $('clearMap').disabled=!!mode;
}
async function camera(){
 try{
  if(stream)stream.getTracks().forEach(t=>t.stop());
  stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480},frameRate:{ideal:20}}});
  $('camera').srcObject=stream;await $('camera').play();
  set('sensorStatus','摄像头已开启 · '+(imu?'IMU 已响应':'IMU 未授权/无数据'));status('相机已就绪');toggleButtons();
 }catch(e){status('摄像头失败：'+e.message)}
}
async function motion(){
 try{
  const promises=[];
  if(typeof DeviceOrientationEvent!=='undefined'&&typeof DeviceOrientationEvent.requestPermission==='function')promises.push(DeviceOrientationEvent.requestPermission());
  if(typeof DeviceMotionEvent!=='undefined'&&typeof DeviceMotionEvent.requestPermission==='function')promises.push(DeviceMotionEvent.requestPermission());
  const granted=await Promise.all(promises);
  if(granted.some(v=>v!=='granted'))throw Error('用户未授权运动传感器');
  window.addEventListener('deviceorientation',e=>{
   if(e.alpha!=null){imu={alpha:e.alpha,heading:Number.isFinite(e.webkitCompassHeading)?e.webkitCompassHeading:null,t:Date.now()};
    set('heading',imu.heading!=null?imu.heading.toFixed(0)+'°（指南针）':imu.alpha.toFixed(0)+'°（相对方向）');
   }
  });
  set('sensorStatus','摄像头 '+(stream?'已开启':'关闭')+' · IMU 权限已请求（等待数据）');
 }catch(e){set('sensorStatus','IMU 授权失败：'+e.message)}
}
async function store(){
 try{await transaction('readwrite','put',{schema:'pai-vin-web-vio-map-v1',createdAt:new Date().toISOString(),frames},'current')}
 catch(e){status('本地地图保存失败：'+e.message)}
}
async function load(){
 try{
  const saved=await transaction('readonly','get','current');
  if(saved?.schema==='pai-vin-web-vio-map-v1'&&Array.isArray(saved.frames)){
   frames=saved.frames.filter(f=>Array.isArray(f.points)&&f.points.length>0).slice(0,MAX);drawMap();
   status('已恢复 '+frames.length+' 张本机关键帧地图');
  }
 }catch(e){status('本地地图加载失败：'+e.message)}
 toggleButtons();
}
async function tick(){
 if(busy||!stream||!mode)return;
 busy=true;
 try{
  const current=features();
  if(mode==='mapping'){
   if(current.length<18){status('纹理不足，寻找门框、家具或书架');return}
   const previous=frames[frames.length-1]?.points;
   const comparison=previous?VINGeometry.verify(current,previous):null;
   // Avoid essentially duplicate snapshots; allow a keyframe after a few checks.
   attempts++;
   if(previous&&comparison?.inliers>=24&&comparison.ratio>.55&&attempts<4){
    status('正在扫描；当前画面与上一关键帧相似');return
   }
   attempts=0;
   frames.push({id:frames.length+1,points:current,orientation:imu?.alpha??null,time:Date.now()});
   drawMap(frames.length-1);status('已存入关键帧 '+frames.length+'，特征点 '+current.length);
   await store();
   if(frames.length>=MAX)stop();
  }else{
   queries++;
   if(current.length<14){set('matchLabel','纹理不足');set('matchDetails','当前特征点 '+current.length);set('hits',hits+'/'+queries);return}
   const decision=VINGeometry.rank(current,frames);
   const best=decision.best||{inliers:0,matches:0,ratio:0,rmse:null,spread:0};
   const index=best.index??-1;
   // Require repeatable results on consecutive samples before declaring a stable match.
   if(decision.accepted&&index>=0){
    if(lastCandidate===index)candidateStreak++;else{lastCandidate=index;candidateStreak=1}
   }else{candidateStreak=0;lastCandidate=-1}
   const accepted=decision.accepted&&candidateStreak>=2;
   if(accepted){
    hits++;set('matchLabel','稳定匹配关键帧 #'+(index+1));drawMap(index);
    if(lastConfirmed!==index){
     route.push(index+1);route=route.slice(-16);lastConfirmed=index;
     set('route',route.map(x=>'#'+x).join(' → '));
    }
   }else{
    set('matchLabel',decision.ambiguous?'场景歧义，移动到更有特征的位置':decision.accepted?'候选关键帧 #'+(index+1)+'（等待连续确认）':'未确认重定位');
    drawMap();
   }
   set('matchDetails',best.inliers+' 内点 / '+best.matches+' 匹配；空间区 '+best.spread+'/4；重投影残差 '+(best.rmse===null?'—':best.rmse.toFixed(1))+' px');
   set('hits',hits+'/'+queries);
   status(accepted?'识别到已存场景（非米制坐标）':'正在搜索已建图场景');
  }
 }catch(e){status('检测异常：'+e.message)}
 finally{busy=false}
}
function start(which){
 if(!stream)return status('请先开启摄像头');
 if(typeof VINFeatures==='undefined'||typeof VINVO==='undefined')return status('视觉模块未加载，请刷新页面');
 if(which==='localization'&&!frames.length)return status('请先建立关键帧地图');
 if(mode)return;
 mode=which;lastTrackPoints=null;trackFrames=0;trackGood=0;visualMotion=null;voAttempt=0;set('voPose','等待数据');set('voQuality','—');set('trackRate','0/0');showMotion(null);queries=0;hits=0;attempts=0;lastCandidate=-1;candidateStreak=0;lastConfirmed=-1;route=[];set('route','—');set('hits','0/0');
 status(which==='mapping'?'开始采集视觉关键帧':'开始检索已有视觉地图');
 toggleButtons();tick();timer=setInterval(tick,INTERVAL);trackTimer=setInterval(track,TRACK_INTERVAL);
}
function stop(){clearInterval(timer);timer=null;clearInterval(trackTimer);trackTimer=null;lastTrackPoints=null;mode='';toggleButtons();status('已停止。地图关键帧：'+frames.length)}
async function clear(){
 if(!confirm('确定清空本机视觉地图？此操作不能恢复。'))return;
 frames=[];await transaction('readwrite','delete','current');drawMap();toggleButtons();status('地图已清空');
}
function init(){
 $('openCamera').onclick=camera;$('motionBtn').onclick=motion;
 $('startMap').onclick=()=>start('mapping');$('stopMap').onclick=stop;
 $('startLocalize').onclick=()=>start('localization');$('stopLocalize').onclick=stop;
 $('saveMap').onclick=store;$('clearMap').onclick=clear;
 dbOpen().then(load).catch(e=>status('IndexedDB 不可用：'+e.message));
 toggleButtons();
 window.addEventListener('pagehide',()=>{if(stream)stream.getTracks().forEach(t=>t.stop())});
}
if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
