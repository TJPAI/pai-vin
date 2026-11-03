const assert=require('node:assert/strict');
const V=require('../vio-tracker.js');
const W=256,H=192;
const image=new Uint8Array(W*H);
let seed=1594;function rnd(){seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed>>>24}
for(let y=0;y<H;y++)for(let x=0;x<W;x++)image[y*W+x]=rnd();
function shift(a,dx,dy){
 const b=new Uint8Array(W*H);
 for(let y=0;y<H;y++)for(let x=0;x<W;x++){
  const px=x-dx,py=y-dy;
  b[y*W+x]=(px>=0&&py>=0&&px<W&&py<H)?a[py*W+px]:0;
 }return b;
}
const tracker=new V.Tracker({maxTracks:85});
const first=tracker.update(image,100);
assert.ok(first.total>=35,JSON.stringify(first));
let f=image;
for(let i=1;i<=5;i++){
 f=shift(f,2,1);
 const r=tracker.update(f,100+100*i);
 assert.ok(r.retained>=12,JSON.stringify(r));
 assert.ok(r.retention>.2,JSON.stringify(r));
 assert.equal(r.metricPoseKnown,false);
 assert.equal(r.slamInitialized,false);
 const matches=r.matches.filter(p=>p.age>=2);
 assert.ok(matches.length>=12,JSON.stringify(r));
 const exact=matches.filter(p=>Math.abs(p.dx-2)<=1&&Math.abs(p.dy-1)<=1);
 assert.ok(exact.length>=matches.length*.6,JSON.stringify({matches:matches.length,exact:exact.length}));
}
assert.ok(tracker.last.longLived>=10,'stable IDs survive five successive frames');
const same=tracker.update(f,800);
assert.ok(same.retained>=12,JSON.stringify(same));
const stable=same.matches.filter(p=>p.dx===0&&p.dy===0);
assert.ok(stable.length>=same.matches.length*.65,'no false movement for static image');
assert.throws(()=>tracker.update(f,700),/non-monotonic/);
assert.throws(()=>tracker.update(new Uint8Array(10),900),/invalid grayscale/);
const flat=new Uint8Array(W*H).fill(127),t2=new V.Tracker();
assert.equal(t2.update(flat,100).total,0);
assert.equal(t2.update(flat,200).total,0);
// Larger movement must be handled by the quarter-resolution search,
 // without fabricating 2D/3D spatial position.
const wide=new V.Tracker({maxTracks:80});
const base=wide.update(image,100);
const moved=wide.update(shift(image,14,6),300);
assert.ok(moved.retained>=8,JSON.stringify({base:base.total,retained:moved.retained}));
const correct=moved.matches.filter(p=>Math.abs(p.dx-14)<=1&&Math.abs(p.dy-6)<=1);
assert.ok(correct.length>=Math.max(5,moved.matches.length*.45),JSON.stringify({matches:moved.matches.length,correct:correct.length}));
tracker.reset();assert.equal(tracker.tracks.length,0);
console.log('VIO persistent feature tracking tests PASS');
