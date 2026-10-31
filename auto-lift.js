/* Experimental raise-to-scan detector. Runs only while document stays visible.
   Device orientation is a heuristic, not a guaranteed pocket-mode classifier. */
(function(){
'use strict';
let enabled=false,base=null,lastFire=0,stableStart=0,lastBeta=null,events=0;
const button=document.getElementById('autoLift');
const info=document.getElementById('autoStatus');
function message(s){info.textContent=s}
function orientation(e){
 if(!enabled||document.hidden||stream)return;
 if(!Number.isFinite(e.beta))return;
 events++;
 const beta=e.beta;
 if(base===null){base=beta;message('已记录初始姿态 '+Math.round(base)+'°，把手机放入口袋后举起测试');return}
 const delta=Math.abs(beta-base);
 // Candidate movement from the reference pocket angle, require 400ms dwell and rate limit.
 if(delta>38){
  if(!stableStart)stableStart=performance.now();
  if(performance.now()-stableStart>420 && performance.now()-lastFire>8000){
   lastFire=performance.now();stableStart=0;
   enabled=false;button.textContent='自动举起检测：关闭';
   message('检测到姿态改变 '+Math.round(delta)+'°，尝试自动打开摄像头');
   log('raise_detected',{delta,events});
   openCamera().then(()=>{if(!stream)message('自动启动摄像头失败，请手动点击“打开摄像头”')});
  }
 }else stableStart=0;
 if(events%80===0)message('举起监测中 · 当前角度变化 '+Math.round(delta)+'°');
}
button.onclick=async()=>{
 if(!enabled && typeof DeviceOrientationEvent!=='undefined' && typeof DeviceOrientationEvent.requestPermission==='function'){
 try{let p=await DeviceOrientationEvent.requestPermission();if(p!=='granted'){message('姿态权限被拒绝，请在Safari设置中允许后重试');return}}
 catch(e){message('姿态权限请求失败：'+e.message);return}
 }
 enabled=!enabled;base=null;stableStart=0;events=0;
 button.textContent=enabled?'自动举起检测：已开启':'自动举起检测：关闭';
 message(enabled?'请在目前姿态停留片刻以校准，然后举起手机。需先授权摄像头。':'自动检测已关闭。');
 log('auto_lift_toggled',{enabled});
};
window.addEventListener('deviceorientation',orientation);
document.addEventListener('visibilitychange',()=>{if(document.hidden&&enabled){message('Safari 进入后台，自动举起检测可能暂停');log('visibility_hidden')}});
})();