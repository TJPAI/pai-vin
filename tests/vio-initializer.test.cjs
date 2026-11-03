const assert=require('node:assert/strict');
const V=require('../vio-initializer.js');
function samples({gyro=[.012,-.008,.018],accel=[0,0,9.81],noise=0}={}){
 const out=[];
 for(let i=0;i<220;i++){
  const j=noise*Math.sin(i*.91);
  out.push({t:i*10,gx:gyro[0]+j,gy:gyro[1]-j/2,gz:gyro[2]+j/3,
   ax:accel[0]+j,ay:accel[1]-j,az:accel[2]+j});
 }return out;
}
const s=samples({noise:.002}),i=V.initialize(s);
assert.equal(i.ok,true,JSON.stringify(i));
assert.ok(Math.abs(i.gyroBias[0]-.012)<.001);
assert.ok(Math.abs(i.gravityMagnitude-9.81)<.01);
assert.equal(i.yawObservable,false);
assert.equal(i.metricPositionKnown,false);
const x=V.initialize(samples({gyro:[0,0,.4]}));
assert.equal(x.ok,false);assert.equal(x.reason,'high_mean_gyro');
const moving=samples({noise:.22});
assert.equal(V.initialize(moving).reason,'device_moving');
assert.equal(V.initialize(s.slice(0,30)).reason,'not_enough_samples');
const gap=s.map(v=>({...v}));gap[60].t+=300;
assert.equal(V.initialize(gap).ok,false);
const tilted=V.initialize(samples({accel:[0,9.81,0]}));
assert.equal(tilted.ok,true,JSON.stringify(tilted));
const rotated=V.qRotate(tilted.initialAttitude,[0,9.81,0]);
assert.ok(Math.abs(rotated[2]-9.81)<.00001,JSON.stringify(rotated));
const a=new V.Attitude(i);const init=a.push({...s[0],t:3000});
assert.equal(init.ok,true);
for(let k=1;k<=100;k++){
 const t=3000+k*10;
 const r=a.push({t,gx:.012,gy:-.008,gz:.018+Math.PI/2,ax:0,ay:0,az:9.81});
 assert.equal(r.ok,true);
}
const q=a.snapshot();
const angle=2*Math.atan2(q.quaternion[3],q.quaternion[0]);
assert.ok(Math.abs(angle-Math.PI/2)<.025,JSON.stringify({angle,q}));
assert.equal(q.metricPosition,null);
assert.equal(q.visualCorrectionApplied,false);
assert.equal(a.push({t:4000,gx:0,gy:0,gz:0}).ok,false);
assert.equal(a.push({t:4300,gx:0,gy:0,gz:0}).ok,false);
console.log('VIO stationary initialization and bias-corrected attitude tests PASS');
