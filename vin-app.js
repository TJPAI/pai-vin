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
emptyCanvas();explain();
})();