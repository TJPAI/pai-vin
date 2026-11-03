const assert=require('node:assert/strict');const K=require('../vio-keyframes.js');const V=require('../vio-tracker.js');
const W=256,H=192;let seed=213;const a=new Uint8Array(W*H);
for(let i=0;i<a.length;i++){seed=(Math.imul(seed,1664525)+1013904223)>>>0;a[i]=seed>>>24;}
const shift=(src,dx,dy)=>{const out=new Uint8Array(src.length);for(let y=0;y<H;y++)for(let x=0;x<W;x++){const u=x-dx,v=y-dy;out[y*W+x]=u>=0&&v>=0&&u<W&&v<H?src[v*W+u]:0;}return out;};
const tracker=new V.Tracker({maxTracks:80}),bank=new K.KeyframeBank({interval:2});
tracker.update(a,100);let r=bank.update(a,tracker.tracks,1,100);
assert.equal(r.inserted,true);assert.equal(r.keyframeCount,1);
const b=shift(a,2,1);tracker.update(b,200);const live=tracker.tracks.filter(p=>p.age>1);
assert.ok(live.length>=25);
r=bank.update(b,live.slice(14),5,500);
assert.ok(r.attempts>0,JSON.stringify(r));assert.ok(r.verified2D>0,JSON.stringify(r));
for(const p of r.observations)assert.ok(live.slice(0,14).some(h=>h.id===p.trackId));
assert.equal(r.metricPoseAvailable,false);
assert.equal(bank.update(new Uint8Array(W*H),live.slice(14),10,1000).verified2D,0);
bank.reset();assert.equal(bank.snapshot().keyframes,0);
const bounded=new K.KeyframeBank({maxFrames:2,interval:1});
for(let i=0;i<5;i++)bounded.update(a,tracker.tracks.map(p=>({...p,id:p.id+1000*i})),i+1,100+i);
assert.ok(bounded.frames.length<=2);
console.log('V3.24 keyframe observations PASS');