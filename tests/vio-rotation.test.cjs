const assert=require('node:assert/strict'),V=require('../vio-rotation.js');
const K={fx:240,fy:246,cx:128,cy:96};
const R=V.expSO3([.035,-.055,.07]);
const mv=(A,x)=>A.map(row=>row.reduce((s,a,i)=>s+a*x[i],0));
const samples=[];
for(let y=25;y<170;y+=18)for(let x=20;x<236;x+=20){
 const p=[(x-K.cx)/K.fx,(y-K.cy)/K.fy,1],q=mv(R,p);
 const newx=K.fx*q[0]/q[2]+K.cx,newy=K.fy*q[1]/q[2]+K.cy;
 samples.push({id:samples.length+1,px:x,py:y,x:newx,y:newy});
}
const z=V.estimate(samples,K);
assert.equal(z.ok,true,JSON.stringify(z));
assert.ok(z.inliers>=50,JSON.stringify(z));
assert.ok(z.medianErrorRad<.0005);
const matrixDist=Math.sqrt(R.flat().reduce((v,a,i)=>v+(a-z.rotationCameraCurrentFromPrevious.flat()[i])**2,0));
assert.ok(matrixDist<.002,'rotation must be recovered, got '+matrixDist);
assert.equal(z.metricTranslation,null);
assert.equal(z.cameraIMUExtrinsicsApplied,false);
assert.equal(V.estimate(samples,null).reason,'intrinsics_missing');
assert.equal(V.estimate(samples.slice(0,10),K).reason,'insufficient_tracks');
// Reject inconsistent scene flow instead of creating orientation correction.
const corrupted=samples.map((p,i)=>i%3===0?{...p,x:p.x+30*(i%2?1:-1),y:p.y+40}:p);
const bad=V.estimate(corrupted,K);
assert.equal(bad.ok,false,JSON.stringify(bad));
const none=V.estimate(samples.map(p=>({...p,x:p.px,y:p.py})),K);
assert.equal(none.ok,true,JSON.stringify(none));
assert.ok(none.theta<.002);
console.log('VIO visual rotation geometry and corruption rejection PASS');
