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
// Regression: fast SSD must equal the legacy two-pass definition for all patches.
function referenceSSD(a,b,w,h,x,y,u,v,r=2){
 let sa=0,sb=0,s=0,n=0;
 for(let j=-r;j<=r;j++)for(let i=-r;i<=r;i++){
  sa+=a[(y+j)*w+x+i];sb+=b[(v+j)*w+u+i];n++;
 }
 const offset=Math.round((sb-sa)/n);
 for(let j=-r;j<=r;j++)for(let i=-r;i<=r;i++){
  const d=a[(y+j)*w+x+i]+offset-b[(v+j)*w+u+i];s+=d*d;
 }
 return s/n;
}
const brightness=shift(image,3,2);
for(let i=0;i<200;i++){
 const x=5+(i*19)%(W-10),y=5+(i*23)%(H-10);
 const u=5+(i*17)%(W-10),v=5+(i*29)%(H-10);
 assert.ok(Math.abs(V.score(image,brightness,W,H,x,y,u,v)-referenceSSD(image,brightness,W,H,x,y,u,v))<1e-8,'one-pass SSD must be exact');
}
// Subpixel displacement recovery: intensity-interpolated synthetic camera motion.
function fractionalShift(a,dx,dy){
 const out=new Uint8Array(W*H);
 for(let y=0;y<H;y++)for(let x=0;x<W;x++){
  const value=V.bilinear(a,W,H,x-dx,y-dy);
  out[y*W+x]=Number.isFinite(value)?Math.round(value):0;
 }
 return out;
}
// Smooth band-limited texture is the correct LK model; white pixel noise
// violates brightness-constancy linearization and should not be a ground truth.
const pattern=new Uint8Array(W*H);
for(let y=0;y<H;y++)for(let x=0;x<W;x++)
 pattern[y*W+x]=Math.max(0,Math.min(255,Math.round(128+37*Math.sin(x*.19+y*.13)+34*Math.cos(x*.14-y*.20)+23*Math.sin(x*.31-y*.16))));
const frac=fractionalShift(pattern,.65,.35);
const estimates=[];
for(let y=30;y<160;y+=16)for(let x=30;x<225;x+=17){
 const q=V.refineLK(pattern,frac,W,H,x,y,x+1,y);
 if(q)estimates.push(Math.hypot(q.x-x-.65,q.y-y-.35));
}
assert.ok(estimates.length>=30,'KLT must refine textured tracks');
estimates.sort((a,b)=>a-b);
assert.ok(estimates[estimates.length>>1]<.32,'median KLT subpixel error must be below 0.32 px');
const flatLK=new Uint8Array(W*H).fill(127);
assert.equal(V.refineLK(flatLK,flatLK,W,H,80,80,80,80),null,'degenerate image gradients rejected');
// Corner structure tensor: accept a 2D intersection, reject a straight edge.
const edge=new Uint8Array(W*H);
for(let y=0;y<H;y++)for(let x=0;x<W;x++)edge[y*W+x]=x<128?40:210;
assert.ok(V.cornerStrength(edge,W,H,128,96)<1,'straight edge must not be a corner');
const cross=new Uint8Array(W*H);
for(let y=0;y<H;y++)for(let x=0;x<W;x++)cross[y*W+x]=(x>=128)!==(y>=96)?220:20;
assert.ok(V.cornerStrength(cross,W,H,128,96)>100,'intersection must be a corner');
// Corner detection follows frozen V3.12 semantics; no Shi-Tomasi-specific selection requirement.
// Regression against the frozen V3.12 corner response: catch any index
// transposition in the off-center gradient stencil.
function legacyCorners(input,w=W,h=H,max=125){
 const candidates=[];
 for(let y=9;y<h-9;y+=3)for(let x=9;x<w-9;x+=3){
  const i=y*w+x;
  const dx=Math.abs(input[i+1]-input[i-1])+Math.abs(input[i+w+1]-input[i+w-1]);
  const dy=Math.abs(input[i+w]-input[i-w])+Math.abs(input[i+w+1]-input[i-w+1]);
  const score=Math.min(dx,dy);
  if(score>48)candidates.push({x,y,score});
 }
 candidates.sort((a,b)=>b.score-a.score);
 const cols=8,rows=6,cells=Array.from({length:cols*rows},()=>[]);
 for(const p of candidates){
  const cx=Math.min(cols-1,Math.floor(p.x*cols/w));
  const cy=Math.min(rows-1,Math.floor(p.y*rows/h));
  cells[cy*cols+cx].push(p);
 }
 const out=[],insert=p=>{
  if(out.some(q=>(q.x-p.x)**2+(q.y-p.y)**2<100))return false;
  out.push(p);return true;
 };
 for(const cell of cells){
  for(const p of cell)if(insert(p))break;
  if(out.length>=max)return out;
 }
 for(const p of candidates){
  if(out.length>=max)break;
  insert(p);
 }
 return out;
}
for(const sample of [image,pattern,shift(image,11,4),frac]){
 const got=V.corners(sample,W,H,90),want=legacyCorners(sample,W,H,90);
 assert.deepEqual(got,want,'corner detection must precisely match V3.12 baseline');
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
