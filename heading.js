/* V0.4.1: browser heading adapter. Orientation readings are phone attitude,
   NOT guaranteed pedestrian walking direction, particularly in a pocket. */
(function(global){
'use strict';
let heading=null,source='未获得',updated=0,permission='unknown';
function wrap(v){return ((v%360)+360)%360}
function onOrientation(e){
 let deg=null,src='';
 if(Number.isFinite(e.webkitCompassHeading)){deg=e.webkitCompassHeading;src='iOS 罗盘'}
 else if(e.absolute===true&&Number.isFinite(e.alpha)){deg=360-e.alpha;src='绝对设备朝向'}
 if(deg===null)return;
 heading=wrap(deg);source=src;updated=Date.now();
}
async function enable(){
 if(typeof DeviceOrientationEvent==='undefined')return {ok:false,reason:'浏览器不支持方向事件'};
 try{
  if(typeof DeviceOrientationEvent.requestPermission==='function'){
   const result=await DeviceOrientationEvent.requestPermission();
   if(result!=='granted')return {ok:false,reason:'未授予方向传感器权限'};
  }
  if(permission!=='granted')global.addEventListener('deviceorientation',onOrientation);
  permission='granted';return {ok:true};
 }catch(e){return {ok:false,reason:e.message}}
}
function read(){return {heading,source,ageMs:updated?Date.now()-updated:null,valid:updated>0&&Date.now()-updated<2500}}
global.VINHeading={enable,read};
})(window);