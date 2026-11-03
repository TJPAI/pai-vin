const assert=require('node:assert/strict');
const P=require('../web-vio-pose.js');
const E=P.eigSym([[2,0,0],[0,5,0],[0,0,1]]);
assert.ok(Math.abs(E[0].val-1)<1e-8);
assert.ok(Math.abs(E[2].val-5)<1e-8);
const A=[],B=[],pairs=[];
let seed=12345;const rand=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296};
const f=220,cx=128,cy=96;
const th=.09,c=Math.cos(th),s=Math.sin(th);
for(let i=0;i<70;i++){
 const X=(rand()-.5)*3,Y=(rand()-.5)*2,Z=3+rand()*5;
 const a={x:cx+f*X/Z,y:cy+f*Y/Z};
 const xx=c*X+s*Z+.28,yy=Y+.03,zz=-s*X+c*Z+.16;
 const b={x:cx+f*xx/zz,y:cy+f*yy/zz};
 A.push(a);B.push(b);pairs.push({i,j:i});
}
const q=P.estimate(A,B,{pairs,focal:f,iterations:100});
assert.equal(q.ok,true,JSON.stringify(q));
assert.ok(q.inliers>=45,JSON.stringify(q));
assert.ok(Number.isFinite(q.rotationDeg)&&q.rotationDeg<20,JSON.stringify(q));
assert.ok(q.t.length===3&&q.R.length===3);
assert.equal(q.metricScaleKnown,false);
assert.equal(P.estimate(A,B,{pairs:pairs.slice(0,5)}).ok,false);
console.log('Two-view experimental VO geometry tests PASS',JSON.stringify({inliers:q.inliers,rotationDeg:q.rotationDeg,cheirality:q.cheirality}));
