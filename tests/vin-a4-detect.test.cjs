const assert=require('node:assert/strict');
const D=require('../vin-a4-detect.js'),B=require('../vin-a4-board.js');
const W=360,H=480;
function frame(missing=-1){
 const gray=new Uint8Array(W*H).fill(45);
 const box=(x0,y0,x1,y1,v)=>{
  for(let y=Math.max(0,y0);y<Math.min(H,y1);y++)for(let x=Math.max(0,x0);x<Math.min(W,x1);x++)gray[y*W+x]=v;
 };
 box(48,25,312,454,236);
 const centres=[[88,78],[272,78],[272,404],[88,404]];
 // Board border runs through four fiducial centres, as in the real printable template.
 box(88,78,273,79,12);box(88,404,273,405,12);
 box(88,78,89,405,12);box(272,78,273,405,12);
 centres.forEach(([x,y],i)=>{
  if(i===missing)return;
  box(x-11,y-11,x+12,y+12,12);
  box(x-5,y-5,x+6,y+6,252);
  box(x-2,y-2,x+3,y+3,11);
 });
 return {gray,centres};
}
const a=frame(),res=D.detect(a.gray,W,H);
assert.equal(res.ok,true,JSON.stringify(res));
for(let i=0;i<4;i++)assert.ok(Math.hypot(res.corners[i][0]-a.centres[i][0],res.corners[i][1]-a.centres[i][1])<2,JSON.stringify(res.corners));
const plane=B.calibrate(res.corners,W,H);assert.equal(plane.ok,true,JSON.stringify(plane));
assert.equal(plane.metricPlaneOnly,true);assert.equal(plane.globalSLAMScaleKnown,false);
const rgba=new Uint8Array(W*H*4);
for(let i=0;i<W*H;i++)rgba.set([a.gray[i],a.gray[i],a.gray[i],255],4*i);
assert.equal(D.detect(rgba,W,H).ok,true);
const no=frame(2),notFound=D.detect(no.gray,W,H);
assert.equal(notFound.ok,false,'should not validate image with missing marker');
assert.equal(D.detect(new Uint8Array(W*H).fill(255),W,H).ok,false,'plain A4 should not produce false corners');
assert.equal(D.detect(new Uint8Array(W*H).fill(0),W,H).ok,false);
console.log('Automatic A4 four-marker detection synthetic tests PASS',JSON.stringify({corners:res.corners,threshold:res.threshold}));
