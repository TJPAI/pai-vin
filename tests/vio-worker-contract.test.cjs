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
assert.ok(two.vision&&typeof two.vision.ok==='boolean');
assert.ok(Number.isFinite(two.processingMs));
assert.ok(two.stageTiming&&['trackingMs','rotationMs','geometryMs'].every(x=>Number.isFinite(two.stageTiming[x])),'worker must report stage timings');
assert.ok(!Object.hasOwn(two,'metricPosition')&&!Object.hasOwn(two,'poseWorld'));
console.log('VIO Worker integration protocol tests PASS');
