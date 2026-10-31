'use strict';
const $=id=>document.getElementById(id);
let stream=null,imu=false,steps=0,x=0,y=0,yaw=0,lastMag=9.8,lastStep=0,lastTime=0,motionSamples=0,tracks=[[0,0]],logs=[],anchors=[];
let storageReady=false,storageError=null,saveBusy=false,lastCorrection=null;
function note(t){$('status').textContent=t;$('activity').textContent=new Date().toLocaleTimeString()+' · '+t}
function log(type,detail={}){logs.push({time:Date.now(),type,...detail});if(logs.length>12000)logs.shift()}
function render(){
 $('steps').textContent=steps;
 $('distance').textContent=(steps*.68).toFixed(1);
 $('coordinates').textContent='相对坐标：'+x.toFixed(1)+'，'+y.toFixed(1)+' m · 航向仅粗估';
 VINMap.render({x,y,tracks,anchors,correction:lastCorrection});
}
function step(source){steps++;x+=.68*Math.sin(yaw);y+=.68*Math.cos(yaw);tracks.push([x,y]);if(tracks.length>1500)tracks.shift();render();log('step',{source,x,y,steps})}
function handleMotion(e){
 if(!imu)return;
 const a=e.accelerationIncludingGravity;if(!a||a.x==null)return;
 const m=Math.hypot(a.x||0,a.y||0,a.z||0),t=performance.now();
 if(m>11.4&&lastMag<=11.4&&t-lastStep>380){lastStep=t;step('imu')}
 lastMag=m;
 const r=e.rotationRate;if(r&&Number.isFinite(r.alpha)){let dt=lastTime?Math.min(.08,(t-lastTime)/1000):.02;yaw+=r.alpha*Math.PI/180*dt}
 lastTime=t;
 if(++motionSamples%50===0){log('imu_sample',{mag:+m.toFixed(2),interval:e.interval||null});$('sensor').textContent='IMU 采样 '+motionSamples+' 次 · '+Math.round(1000/(e.interval||20))+' Hz（设备报告）'}
}
async function startMotion(){
 try{
  if(!window.DeviceMotionEvent)throw Error('当前浏览器不支持 DeviceMotionEvent');
  if(typeof DeviceMotionEvent.requestPermission==='function'){
   const p=await DeviceMotionEvent.requestPermission();if(p!=='granted')throw Error('未授予运动传感器权限');
  }
  if(!imu)window.addEventListener('devicemotion',handleMotion);
  imu=true;$('sensor').textContent='IMU 已启动（步长固定0.68m，未校准）';$('mode').textContent='IMU测试';
  note('已请求 IMU。请走动观察步数是否更新。');log('imu_started')
 }catch(e){note('IMU 开启失败：'+e.message);log('imu_error',{message:e.message})}
}
async function openCamera(){
 try{
  if(!navigator.mediaDevices?.getUserMedia)throw Error('需要 HTTPS 和摄像头权限');
  if(stream)return;
  stream=await navigator.mediaDevices.getUserMedia({video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480}},audio:false});
  $('video').srcObject=stream;$('cameraBox').style.display='block';$('mode').textContent='视觉采集';note('摄像头已打开，可保存锚点或匹配。');log('camera_on')
 }catch(e){note('摄像头失败：'+e.message);log('camera_error',{message:e.message})}
}
function closeCamera(){
 if(stream){stream.getTracks().forEach(t=>t.stop());stream=null}
 $('video').srcObject=null;$('cameraBox').style.display='none';$('mode').textContent='口袋模式';note('摄像头已关闭；请保持 Safari 前台与屏幕常亮。');log('camera_off')
}
function descriptor(){
 if(!stream||$('video').readyState<2)throw Error('摄像头画面尚未就绪');
 const c=$('canvas'),ctx=c.getContext('2d',{willReadFrequently:true});
 ctx.drawImage($('video'),0,0,320,240);
 return VINVision.extract(ctx.getImageData(0,0,320,240).data);
}
async function save(){
 if(saveBusy)return;
 saveBusy=true;
 try{
  if(!storageReady)throw Error('锚点数据库尚未就绪：'+(storageError||'请稍后重试'));
  const points=descriptor();
  if(points.length<12)throw Error('当前画面特征过少，请面对门牌或纹理丰富的墙面');
  const label=($('anchorName').value||'').trim()||('位置 '+(anchors.length+1));
  const a={id:Date.now(),label,x,y,points};
  await VINStore.put(a);anchors.push(a);
  render();$('scanStatus').textContent='已保存「'+label+'」：'+points.length+' 个局部特征点（坐标仍为未经校准PDR估计）';$('anchorName').value='';
  log('anchor_saved',{id:a.id,features:points.length,x,y});
 }catch(e){$('scanStatus').textContent='保存失败，未计入锚点：'+e.message;log('anchor_save_failed',{error:e.message})}finally{saveBusy=false}
}
function match(){
 try{
  const start=performance.now(),points=descriptor();
  if(!anchors.length)throw Error('请先在不同位置保存图像锚点');
  const ranked=anchors.filter(a=>Array.isArray(a.points))
    .map(a=>({a,m:VINVision.match(points,a.points)}))
    .sort((u,v)=>Number(v.m.accepted)-Number(u.m.accepted)||v.m.inliers-u.m.inliers||v.m.ratio-u.m.ratio);
  if(!ranked.length)throw Error('旧版锚点不兼容，请重新保存');
  const best=ranked[0],second=ranked[1],elapsed=Math.round(performance.now()-start);
  const ambiguous=!!(second&&second.m.accepted&&
    second.m.inliers>=best.m.inliers*.85 &&
    Math.hypot(second.a.x-best.a.x,second.a.y-best.a.y)>2);
  $('quality').textContent=best.m.inliers+' 点';
  $('matchTime').textContent=elapsed+' ms';
  const rows=ranked.slice(0,4).map((r,i)=>(i+1)+'. '+(r.a.label||('锚点 '+r.a.id))+
    '：'+r.m.inliers+' 匹配 / '+(r.m.accepted?'几何通过':'拒绝')).join('；');
  $('matchCandidates').textContent=rows;
  log('vision_matching',{id:best.a.id,elapsed,ambiguous,secondId:second?.a.id||null,...best.m,features:points.length});
  if(!best.m.accepted||ambiguous){
   const reason=ambiguous?'多个不同位置的候选相近，拒绝自动纠偏':best.m.reason;
   $('scanStatus').textContent='匹配未通过：'+reason+'，请转动手机补扫。';
   $('matchState').textContent='不可靠（未纠偏）';
   log('relocalization_rejected',{reason});return;
  }
  const old=[x,y],jump=Math.hypot(best.a.x-x,best.a.y-y);
  x=best.a.x;y=best.a.y;tracks.push([x,y]);lastCorrection={old,at:[x,y],jump,anchorId:best.a.id,when:Date.now()};render();
  $('correctionSummary').textContent='最近纠偏：'+(best.a.label||'锚点')+' · 相对坐标跳变 '+jump.toFixed(1)+' m（非真实定位误差）';
  $('matchState').textContent='锚点匹配成功（示意纠偏）';
  $('scanStatus').textContent='匹配「'+(best.a.label||best.a.id)+'」：'+best.m.inliers+
   ' 点，耗时 '+elapsed+' ms；相对轨迹示意修正 '+jump.toFixed(1)+' m。非三维绝对定位。';
  log('illustrative_relocation',{old,new:[x,y],jump,elapsed,inliers:best.m.inliers,anchorId:best.a.id});
 }catch(e){$('scanStatus').textContent=e.message;$('matchState').textContent='匹配失败'}
}
function simulate(){yaw=Math.PI/2;for(let i=0;i<8;i++)step('SIMULATED');yaw=0;for(let i=0;i<5;i++)step('SIMULATED');$('mode').textContent='模拟轨迹';note('当前轨迹明确为模拟数据，非IMU测量。');log('simulation')}
function exportLog(){let data={version:'0.4',steps,relativePosition:[x,y],tracks,logs,lastCorrection,anchors:anchors.map(({points,...a})=>({...a,featureCount:points?.length||0}))};let url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));let a=document.createElement('a');a.href=url;a.download='pai-vin-measurements.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),2000)}
async function reset(){
 if(!confirm('清除本机轨迹和视觉锚点？此操作不可撤销。'))return;
 try{
  await VINStore.clear();
  try{localStorage.removeItem('pai-vin-anchors')}catch{}
  anchors=[];x=y=yaw=steps=0;tracks=[[0,0]];logs=[];lastCorrection=null;$('correctionSummary').textContent='尚无视觉纠偏记录';
  $('matchCandidates').textContent='';$('matchTime').textContent='—';$('matchState').textContent='未匹配';
  render();note('数据库与实验轨迹已清空');
 }catch(e){note('清除未完成：'+e.message)}
}
async function initStorage(){
 try{
  const result=await VINStore.load();anchors=result.anchors;storageReady=true;storageError=null;render();
  $('storageStatus').textContent='IndexedDB 可用 · '+anchors.length+' 个锚点'+
    (result.migrated?' · 已迁移旧数据 '+result.migrated+' 个':'')+
    (result.warning?' · 迁移警告：'+result.warning:'');
  log('storage_ready',{anchors:anchors.length,migrated:result.migrated});
 }catch(e){
  storageReady=false;storageError=e.message;
  anchors=VINStore.legacy();render();
  $('storageStatus').textContent='存储异常：'+e.message+'。旧锚点仅供当前会话查询，禁止假保存。';
  log('storage_error',{message:e.message});
 }
}

$('motion').onclick=startMotion;$('camera').onclick=openCamera;$('pocket').onclick=closeCamera;$('save').onclick=save;$('match').onclick=match;$('simulate').onclick=simulate;$('export').onclick=exportLog;$('reset').onclick=reset;
if('serviceWorker' in navigator){
 navigator.serviceWorker.register('./sw.js',{updateViaCache:'none'})
  .then(reg=>reg.update().catch(e=>log('sw_update_error',{message:e.message})))
  .catch(e=>log('sw_error',{message:e.message}));
 navigator.serviceWorker.addEventListener('controllerchange',()=>{
  if(!sessionStorage.getItem('pai-vin-sw-refresh-040')){
   sessionStorage.setItem('pai-vin-sw-refresh-032','1');
   window.location.reload();
  }
 });
}
render();initStorage();