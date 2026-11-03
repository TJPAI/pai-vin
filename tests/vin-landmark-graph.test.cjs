const assert=require('node:assert/strict');
global.VINGeometry=require('../web-vio-geometry.js');
const {Graph,isPoints,render}=require('../vin-landmark-graph.js');
let x=1234567;function rand(){x=(Math.imul(x,1664525)+1013904223)>>>0;return x}
function points(offset){return Array.from({length:24},(_,i)=>({x:16+(i%6)*40,y:15+Math.floor(i/6)*40,a:(rand()+offset)>>>0,b:(rand()+offset)>>>0}))}
const p=points(0);assert.equal(isPoints(p),true);
const g=new Graph(),a=g.add(p,1000);assert.equal(a.added,true);
assert.equal(g.add(p,2000).reason,'same_view');
assert.equal(g.match(p).accepted,true);
assert.equal(g.add([],3000).reason,'low_texture');
for(let i=0;i<14;i++){const next=points(i*100);const a=g.add(next,3000+i*1000);assert.equal(a.added,true)}
assert.equal(g.frames.length,15);
const restored=new Graph(g.serialize());assert.equal(restored.frames.length,15);
assert.equal(restored.serialize().metricScaleVerified,false);
assert.equal(restored.match(p).accepted,true);
assert.throws(()=>new Graph({schema:'wrong',frames:[]}));
let drew=0;const ctx={fillRect(){drew++},fillText(){},beginPath(){},arc(){},moveTo(){},lineTo(){},stroke(){},fill(){},set fillStyle(v){},set font(v){},set strokeStyle(v){},set lineWidth(v){}};
render({width:360,height:300,getContext:()=>ctx},restored,1);assert.ok(drew>0);
console.log('Device landmark graph tests PASS');
