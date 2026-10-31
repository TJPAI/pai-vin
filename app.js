'use strict';
const $=id=>document.getElementById(id);
let stream=null,imu=false,steps=0,x=0,y=0,yaw=0,lastMag=9.8,lastStep=0,lastTime=0,tracks=[[0,0]],logs=[],anchors=[];
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
 if(logs.length%30===0)log('imu_sample',{mag:m,interval:e.interval||null});
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
 if(!stream||$('video').readyState<2)throw Error('等待摄像头画面稳定');
 const c=$('canvas'),ctx=c.getContext('2d',{willReadFrequently:true});ctx.drawImage($('video'),0,0,320,240);
 const raw=ctx.getImageData(0,0,320,240).data,v=[];
 for(let yy=0;yy<8;yy++)for(let xx=0;xx<8;xx++){
 let sum=0,count=0;
 for(let j=5;j<27;j+=4)for(let i=5;i<37;i+=5){const q=((yy*30+j)*320+xx*40+i)*4;sum+=.299*raw[q]+.587*raw[q+1]+.114*raw[q+2];count++}
 v.push(sum/count)
 }
 let mean=v.reduce((a,b)=>a+b)/64,sd=Math.sqrt(v.reduce((a,b)=>a+(b-mean)**2,0)/64)||1;
 return v.map(z=>Math.max(-2,Math.min(2,(z-mean)/sd)))
}
function sim(a,b){let d=0;for(let i=0;i<a.length;i++)d+=(a[i]-b[i])**2;return Math.exp(-1.3*d/a.length)}
function save(){
 try{let a={id:Date.now(),x,y,desc:descriptor()};anchors.push(a);localStorage.setItem('pai-vin-anchors',JSON.stringify(anchors));render();$('scanStatus').textContent='已保存第 '+anchors.length+' 个二维图像锚点；其坐标未经测量。';log('anchor_saved',{id:a.id,x,y})}
 catch(e){$('scanStatus').textContent=e.message}
}
function match(){
 try{
 const d=descriptor();if(!anchors.length)throw Error('请先保存至少一个锚点');
 let ranked=anchors.map(a=>({a,score:sim(d,a.desc)})).sort((a,b)=>b.score-a.score);
 let result=ranked[0];$('quality').textContent=(100*result.score).toFixed(0)+'%';
 log('image_match',{score:result.score,id:result.a.id});
 if(result.score<.78){$('scanStatus').textContent='相似度偏低，已拒绝纠偏；请重新扫描。';return}
 let old=[x,y];x=result.a.x;y=result.a.y;tracks.push([x,y]);render();
 $('scanStatus').textContent='图像与保存锚点相似，示意坐标已回到锚点；不是三维定位验证。';
 log('illustrative_relocation',{old,new:[x,y],score:result.score})
 }catch(e){$('scanStatus').textContent=e.message}
}
function simulate(){yaw=Math.PI/2;for(let i=0;i<8;i++)step('SIMULATED');yaw=0;for(let i=0;i<5;i++)step('SIMULATED');$('mode').textContent='模拟轨迹';note('当前轨迹明确为模拟数据，非IMU测量。');log('simulation')}
function exportLog(){let data={version:'0.1',steps,relativePosition:[x,y],tracks,logs,anchors:anchors.map(({desc,...a})=>a)};let url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));let a=document.createElement('a');a.href=url;a.download='pai-vin-measurements.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),2000)}
function reset(){if(!confirm('清除本机轨迹和图像锚点？'))return;localStorage.removeItem('pai-vin-anchors');anchors=[];x=y=yaw=steps=0;tracks=[[0,0]];logs=[];render();note('已重置实验数据')}
$('motion').onclick=startMotion;$('camera').onclick=openCamera;$('pocket').onclick=closeCamera;$('save').onclick=save;$('match').onclick=match;$('simulate').onclick=simulate;$('export').onclick=exportLog;$('reset').onclick=reset;
if('serviceWorker'in navigator)navigator.serviceWorker.register('./sw.js').catch(e=>log('sw_error',{message:e.message}));render();