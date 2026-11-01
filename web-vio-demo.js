/* PAI-VIN Safari-only experimental keyframe recognition.
   Gradient corners + deterministic binary intensity descriptors + geometric voting.
   Not 6DoF SLAM, no actual metric position. No remote transmission. */
(()=>{
'use strict';
const $=id=>document.getElementById(id);
const W=256,H=192,MAX=60,INTERVAL=1050;
const cv=document.createElement('canvas');cv.width=W;cv.height=H;
const ctx=cv.getContext('2d',{willReadFrequently:true});
let stream=null,mode='',timer=null,busy=false,frames=[],db=null,imu=null;
let queries=0,hits=0,lastSaved=0,lastFeatures=null,attempts=0;
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
const pattern=(()=>{
 const pairs=[];let seed=0x327a41b9;
 const rand=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296};
 for(let i=0;i<64;i++){
  const p=[];
  for(let j=0;j<2;j++)p.push([Math.floor(rand()*17)-8,Math.floor(rand()*17)-8]);
  pairs.push(p);
 }
 return pairs;
})();
function pop32(x){x=x-((x>>>1)&0x55555555);x=(x&0x33333333)+((x>>>2)&0x33333333);return Math.imul((x+(x>>>4))&0x0f0f0f0f,0x01010101)>>>24}
function hamming(a,b){return pop32((a.a^b.a)>>>0)+pop32((a.b^b.b)>>>0)}
function descriptor(gray,x,y){
 let a=0,b=0;
 for(let n=0;n<64;n++){
  const p=pattern[n],on=gray[(y+p[0][1])*W+x+p[0][0]]<gray[(y+p[1][1])*W+x+p[1][0]];
  if(on){if(n<32)a=(a|(1<<n))>>>0;else b=(b|(1<<(n-32)))>>>0}
 }
 return {x,y,a,b};
}
function features(){
 if(!stream||$('camera').readyState<2)return [];
 ctx.drawImage($('camera'),0,0,W,H);
 const raw=ctx.getImageData(0,0,W,H).data,gray=new Uint8Array(W*H);
 for(let i=0,j=0;i<raw.length;i+=4,j++)gray[j]=(raw[i]*77+raw[i+1]*150+raw[i+2]*29)>>8;
 // Approximate cornerness: strong gradients in both axes with a diagonal check.
 const candidates=[];
 for(let y=11;y<H-11;y+=3)for(let x=11;x<W-11;x+=3){
  const i=y*W+x,dx=Math.abs(gray[i+W+1]-gray[i+W-1])+Math.abs(gray[i-W+1]-gray[i-W-1]);
  const dy=Math.abs(gray[i+W+1]-gray[i-W+1])+Math.abs(gray[i+W-1]-gray[i-W-1]);
  const d1=Math.abs(gray[i+W+1]-gray[i-W-1]),d2=Math.abs(gray[i+W-1]-gray[i-W+1]);
  const score=Math.min(dx,dy)+Math.min(d1,d2);
  if(score>86)candidates.push({x,y,score});
 }
 candidates.sort((a,b)=>b.score-a.score);
 const selected=[],cells=new Set();
 for(const c of candidates){
  const cx=Math.floor(c.x/12),cy=Math.floor(c.y/12),key=cy*100+cx;
  if(cells.has(key))continue;
  cells.add(key);selected.push(descriptor(gray,c.x,c.y));if(selected.length>=115)break;
 }
 return selected;
}
function correspond(a,b){
 let forward=[];
 for(let i=0;i<a.length;i++){
  let best=99,second=99,bi=-1;
  for(let j=0;j<b.length;j++){
   const d=hamming(a[i],b[j]);
   if(d<best){second=best;best=d;bi=j}else if(d<second)second=d;
  }
  if(best<=18&&best<second*.82)forward.push({i,j:bi,d:best,dx:b[bi].x-a[i].x,dy:b[bi].y-a[i].y});
 }
 const unique=new Map();
 for(const m of forward)if(!unique.has(m.j)||unique.get(m.j).d>m.d)unique.set(m.j,m);
 return [...unique.values()];
}
function geometric(a,b){
 if(a.length<8||b.length<8)return {matches:0,inliers:0,ratio:0,dx:0,dy:0};
 const m=correspond(a,b);if(m.length<6)return {matches:m.length,inliers:0,ratio:0,dx:0,dy:0};
 // Translation consensus is intentionally conservative for this demo.
 let best=[],dx=0,dy=0;
 for(let k=0;k<m.length;k++){
  const candidate=m[k],inliers=m.filter(v=>Math.hypot(v.dx-candidate.dx,v.dy-candidate.dy)<13);
  if(inliers.length>best.length){best=inliers;dx=candidate.dx;dy=candidate.dy}
 }
 return {matches:m.length,inliers:best.length,ratio:best.length/Math.max(1,m.length),dx,dy};
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
   const comparison=previous?geometric(current,previous):null;
   // Avoid essentially duplicate snapshots; allow a keyframe after a few checks.
   attempts++;
   if(previous&&comparison?.inliers>=24&&comparison.ratio>.55&&attempts<4){
    status('正在扫描；当前画面与上一关键帧相似');return
   }
   attempts=0;
   frames.push({id:frames.length+1,points:current,orientation:imu?.alpha??null,time:Date.now()});
   lastSaved=Date.now();drawMap(frames.length-1);status('已存入关键帧 '+frames.length+'，特征点 '+current.length);
   await store();
   if(frames.length>=MAX)stop();
  }else{
   queries++;
   if(current.length<14){set('matchLabel','纹理不足');set('matchDetails','当前特征点 '+current.length);set('hits',hits+'/'+queries);return}
   let best={inliers:0,matches:0,ratio:0},index=-1,runner=0;
   for(let i=0;i<frames.length;i++){
    const g=geometric(current,frames[i].points);
    const score=g.inliers*Math.min(g.ratio,1);
    if(score>best.inliers*Math.min(best.ratio,1)){runner=best.inliers;best=g;index=i}
    else runner=Math.max(runner,g.inliers);
   }
   const accepted=best.inliers>=9&&best.ratio>=.38&&best.inliers>=runner*1.08;
   if(accepted){hits++;set('matchLabel','匹配关键帧 #'+(index+1));drawMap(index)}
   else{set('matchLabel','未确认重定位');drawMap()}
   set('matchDetails',best.inliers+' 几何内点 / '+best.matches+' 特征匹配；比率 '+Math.round(best.ratio*100)+'%');
   set('hits',hits+'/'+queries);
   status(accepted?'识别到已存场景（非米制坐标）':'正在搜索已建图场景');
  }
 }catch(e){status('检测异常：'+e.message)}
 finally{busy=false}
}
function start(which){
 if(!stream)return status('请先开启摄像头');
 if(which==='localization'&&!frames.length)return status('请先建立关键帧地图');
 if(mode)return;
 mode=which;queries=0;hits=0;attempts=0;set('hits','0/0');
 status(which==='mapping'?'开始采集视觉关键帧':'开始检索已有视觉地图');
 toggleButtons();tick();timer=setInterval(tick,INTERVAL);
}
function stop(){clearInterval(timer);timer=null;mode='';toggleButtons();status('已停止。地图关键帧：'+frames.length)}
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
