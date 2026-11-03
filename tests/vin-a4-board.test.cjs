const assert=require('node:assert/strict');
const B=require('../vin-a4-board.js');
const img=[[92,70],[248,63],[258,300],[83,320]];
const c=B.calibrate(img,360,480);
assert.equal(c.ok,true,JSON.stringify(c));
assert.ok(c.reprojectionErrorPx<1e-5);
assert.equal(c.globalSLAMScaleKnown,false);
assert.equal(c.metricPlaneOnly,true);
for(let i=0;i<4;i++){
 const p=B.project(c.planeImageToMeters,img[i]);
 assert.ok(Math.hypot(p[0]-B.WORLD[i][0],p[1]-B.WORLD[i][1])<1e-8);
}
assert.equal(B.calibrate([[0,0],[0,0],[0,0],[0,0]],360,480).ok,false);
assert.equal(B.calibrate([[10,10],[200,150],[200,10],[10,150]],360,480).ok,false);
assert.equal(B.calibrate([[0,0],[12,0],[12,20],[0,20]],360,480).ok,false);
console.log('A4 plane geometry + invalid corner tests PASS');
