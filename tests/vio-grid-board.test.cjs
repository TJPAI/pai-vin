const assert=require('node:assert/strict');
const B=require('../vin-a4-board.js'),G=require('../vio-grid-board.js');
const W=640,H=480,g=new Uint8Array(W*H).fill(245);
const corners=[[200,50],[420,50],[420,395],[200,395]];
const Hm=B.homography(B.WORLD,corners);
const circle=(x,y,r)=>{for(let v=Math.floor(y-r);v<=Math.ceil(y+r);v++)for(let u=Math.floor(x-r);u<=Math.ceil(x+r);u++)if(u>=0&&v>=0&&u<W&&v<H&&Math.hypot(u-x,v-y)<=r)g[v*W+u]=15;};
for(const p of G.WORLD){const [x,y]=B.project(Hm,[p.x,p.y]);circle(x,y,4);}
let res=G.detect(g,W,H,corners);
assert.equal(G.WORLD.length,24);assert.equal(res.ok,true,JSON.stringify(res));
assert.equal(res.detected,24,JSON.stringify(res));
assert.ok(res.rmsFourCornerHomographyPx<1.3,JSON.stringify(res));
assert.equal(res.independentInteriorGeometry,true);assert.equal(res.independentCameraCalibration,false);
assert.equal(G.detect(new Uint8Array(W*H).fill(245),W,H,corners).ok,false);
assert.equal(G.detect(g,W,H,null).ok,false);
console.log('V3.27 A4 grid independent interior point diagnostics PASS');