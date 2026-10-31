'use strict';
const $=id=>document.getElementById(id);
let stream=null,imu=false,steps=0,x=0,y=0,yaw=0,lastMag=9.8,lastStep=0,lastTime=0,motionSamples=0,tracks=[[0,0]],logs=[],anchors=[];
try{anchors=JSON.parse(localStorage.getItem('pai-vin-anchors')||'[]')}catch{}
function note(t){$('status').textContent=t;$('activity').textContent=new Date().toLocaleTimeString()+' · '+t}
function log(type,detail={}){logs.push({time:Date.now(),type,...detail});if(logs.length>12000)logs.shift()}
function render(){
 $('steps').textContent=steps;$('distance').textContent=(steps*.68).toFixed(1);
 $('coordinates').textContent='相对坐标：'+x.toFixed(1)+'，'+y.toFixed(1)+' m · 航向仅粗估';
 $('position').setAttribute('cx',180+x*6);$('position').setAttribute('cy',125-y*6);
 $('trail').setAttribute('points',tracks.map(([a,b])=>(180+a*6)+','+(125-b*6)).join(' '));
 $('anchorDots').replaceChildren(...anchors.map(a=>{let n=document.createElementNS('http://www.w3.org/2000/svg','circle');n.setAttribute('cx',180+a.x*6);n.setAttribute('cy',125-a.y*6);n.setAttribute('r','4');n.setAttribute('fill','#ffda72');return n}))
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
function save(){
 try{
  const points=descriptor();
  if(points.length<12)throw Error('当前画面特征过少，请面对门牌或纹理丰富的墙面');
  const a={id:Date.now(),x,y,points};
  anchors.push(a);localStorage.setItem('pai-vin-anchors',JSON.stringify(anchors));
  render();$('scanStatus').textContent='已保存局部特征锚点 '+anchors.length+'：'+points.length+' 个特征点（非3D地图）';
  log('anchor_saved',{id:a.id,features:points.length,x,y});
 }catch(e){$('scanStatus').textContent=e.message}
}
function match(){
 try{
  const points=descriptor();
  if(!anchors.length)throw Error('请先保存至少一个锚点');
  const results=anchors.filter(a=>Array.isArray(a.points)).map(a=>({a,m:VINVision.match(points,a.points)}))
    .sort((u,v)=>v.m.inliers-u.m.inliers);
  if(!results.length)throw Error('旧版锚点需要重新保存，以使用新版特征匹配');
  const best=results[0];$('quality').textContent=best.m.inliers+' 点';
  const summary=best.m.inliers+'/'+best.m.raw+' 个匹配；覆盖率 '+Math.round(best.m.spread*100||0)+'%';
  log('vision_matching',{id:best.a.id,...best.m,features:points.length});
  if(!best.m.accepted){$('scanStatus').textContent='拒绝位置修正：'+best.m.reason+'（'+summary+'）。请转动手机补扫。';return}
  const old=[x,y];x=best.a.x;y=best.a.y;tracks.push([x,y]);render();
  $('scanStatus').textContent='二维局部特征几何验证通过：'+summary+'。已示意回到锚点位置，非三维绝对定位。';
  log('illustrative_relocation',{old,new:[x,y],matches:best.m.inliers});
 }catch(e){$('scanStatus').textContent=e.message}
}
function simulate(){yaw=Math.PI/2;for(let i=0;i<8;i++)step('SIMULATED');yaw=0;for(let i=0;i<5;i++)step('SIMULATED');$('mode').textContent='模拟轨迹';note('当前轨迹明确为模拟数据，非IMU测量。');log('simulation')}
function exportLog(){let data={version:'0.2',steps,relativePosition:[x,y],tracks,logs,anchors:anchors.map(({points,...a})=>({...a,featureCount:points?.length||0}))};let url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));let a=document.createElement('a');a.href=url;a.download='pai-vin-measurements.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),2000)}
function reset(){if(!confirm('清除本机轨迹和图像锚点？'))return;localStorage.removeItem('pai-vin-anchors');anchors=[];x=y=yaw=steps=0;tracks=[[0,0]];logs=[];render();note('已重置实验数据')}
$('motion').onclick=startMotion;$('camera').onclick=openCamera;$('pocket').onclick=closeCamera;$('save').onclick=save;$('match').onclick=match;$('simulate').onclick=simulate;$('export').onclick=exportLog;$('reset').onclick=reset;
if('serviceWorker'in navigator)navigator.serviceWorker.register('./sw.js').catch(e=>log('sw_error',{message:e.message}));render();