const assert=require('node:assert/strict');
const F=require('../web-vio-features.js');
const W=F.W,H=F.H;
const raw=new Uint8Array(W*H*4);
for(let y=0;y<H;y++)for(let x=0;x<W;x++){
 const z=((Math.imul(x+1,73856093)^Math.imul(y+1,19349663))>>>0)%256,i=(y*W+x)*4;
 raw[i]=z;raw[i+1]=z;raw[i+2]=z;raw[i+3]=255;
}
const gray=F.grayRGBA(raw);
assert.equal(gray.length,W*H);
const one=F.detect(gray),two=F.detect(gray);
assert.ok(one.length>=30,'checkerboard has enough image corners');
assert.deepEqual(one,two,'corner descriptors reproducible');
assert.ok(one.every(p=>p.x>=11&&p.x<W-11&&p.y>=11&&p.y<H-11));
assert.ok(one.every(p=>Number.isInteger(p.a)&&Number.isInteger(p.b)));
const blank=F.detect(new Uint8Array(W*H));
assert.equal(blank.length,0,'blank wall contains no detected corners');
assert.throws(()=>F.grayRGBA(new Uint8Array(4)),/short/);
assert.ok(one.length>10,'active frontend emits usable corners');
console.log('Safari camera feature extraction tests passed:',one.length,'features');
