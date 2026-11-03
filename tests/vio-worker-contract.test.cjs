const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const answers=[],sandbox={Uint8Array,Math,Number,Array,Map,Set,Float64Array,performance:{now:()=>20},
 postMessage:message=>answers.push(message)};
sandbox.self=sandbox;
const ctx=vm.createContext(sandbox);
sandbox.importScripts=(...files)=>{for(const f of files)vm.runInContext(fs.readFileSync(f.replace(/^\.\//,''),'utf8'),ctx,{filename:f})};
vm.runInContext(fs.readFileSync('vio-vision-worker.js','utf8'),ctx,{filename:'vio-vision-worker.js'});
assert.equal(typeof sandbox.onmessage,'function');
sandbox.onmessage({data:{type:'reset',session:7}});
assert.equal(answers.at(-1).type,'reset-ack');
const W=256,H=192;
let seed=234;function rand(){seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed>>>24}
const gray=new Uint8Array(W*H);
for(let i=0;i<gray.length;i++)gray[i]=rand();
function shift(src,dx,dy){const a=new Uint8Array(src.length);for(let y=dy;y<H;y++)for(let x=dx;x<W;x++)a[y*W+x]=src[(y-dy)*W+(x-dx)];return a}
sandbox.onmessage({data:{type:'frame',session:7,seq:1,t:100,buffer:gray.buffer}});
const one=answers.at(-1);
assert.equal(one.type,'result',JSON.stringify(one));
assert.equal(one.start,null);
assert.ok(one.tracks.total>20);
assert.equal(one.trackPoints.length,one.tracks.total);
sandbox.onmessage({data:{type:'frame',session:7,seq:2,t:300,buffer:shift(gray,2,1).buffer}});
const two=answers.at(-1);
assert.equal(two.type,'result',JSON.stringify(two));
assert.equal(two.start,100);assert.equal(two.end,300);
assert.ok(two.tracks.retained>=10,JSON.stringify(two));
assert.ok(Number.isInteger(two.tracks.recoveryAttempts)&&Number.isInteger(two.tracks.recovered));
assert.ok(two.vision&&typeof two.vision.ok==='boolean');
assert.ok(two.vo&&two.vo.rotationOnly===true);
assert.equal(two.vo.metricPositionAvailable,false);
assert.equal(two.vo.visualInertialFused,false);
assert.ok(two.keyframe&&two.keyframe.metricPoseAvailable===false);
assert.equal(two.keyframe.summary.observationsOnly,true);
assert.ok(Number.isFinite(two.processingMs));
assert.ok(two.stageTiming&&['trackingMs','rotationMs','geometryMs'].every(x=>Number.isFinite(two.stageTiming[x])),'worker must report stage timings');
assert.equal(two.candidateAB,null,'no candidate means no A/B');
for(let i=3;i<=5;i++)sandbox.onmessage({data:{type:'frame',session:7,seq:i,t:100+i*100,
 buffer:shift(gray,2*(i-1),i-1).buffer,candidateIntrinsics:{
 fx:261.85,fy:261.85,cx:128,cy:96,width:256,height:192,verified:false}}});
const ab=answers.at(-1);
assert.equal(ab.type,'result',JSON.stringify(ab));
assert.ok(ab.candidateAB&&ab.candidateAB.matchedInput);
assert.equal(ab.candidateAB.candidateVerified,false);
assert.equal(ab.candidateAB.metricPoseAvailable,false);
assert.ok(typeof ab.candidateAB.nominal.rotation.ok==='boolean');
assert.ok(typeof ab.candidateAB.candidate.epipolar.ok==='boolean');
assert.ok(ab.stageTiming.abCostMs>=0);
assert.ok(!Object.hasOwn(two,'metricPosition')&&!Object.hasOwn(two,'poseWorld'));
console.log('VIO Worker integration protocol tests PASS');
