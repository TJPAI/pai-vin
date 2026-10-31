const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const context={window:{}};vm.runInNewContext(fs.readFileSync('vision.js','utf8'),context);
const {extract,match}=context.window.VINVision;
const image=new Uint8ClampedArray(320*240*4);
for(let y=0;y<240;y++)for(let x=0;x<320;x++){
 const i=(y*320+x)*4;const v=((x*37+y*17+(x*y)%127)>>>0)%256;
 image[i]=v;image[i+1]=(v*3)%256;image[i+2]=(v*7)%256;image[i+3]=255;
}
const pts=extract(image);
assert.ok(pts.length>=12,'synthetic high texture yields features');
const same=match(pts,pts);
assert.equal(same.accepted,true,'identical frames pass geometric verification');
assert.ok(same.inliers>=12);
const black=extract(new Uint8ClampedArray(320*240*4));
assert.equal(match(black,pts).accepted,false,'blank scenes are rejected');
console.log('vision matching smoke tests passed',pts.length,same.inliers);
