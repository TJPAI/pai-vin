const assert=require('node:assert/strict'),G=require('../vio-geometry.js');
const K={fx:240,fy:240,cx:128,cy:96,verified:false};
const a=[];let id=0;
for(let iy=0;iy<9;iy++)for(let ix=0;ix<11;ix++){
 const x=(ix-5)*.08,y=(iy-4)*.07,z=1.7+((ix*3+iy*7)%9)*.35;
 const X=x*z,Y=y*z,Z=z;
 const x1=K.cx+K.fx*X/Z,y1=K.cy+K.fy*Y/Z;
 const x2=K.cx+K.fx*(X-.15)/(Z+.025),y2=K.cy+K.fy*(Y+.025)/(Z+.025);
 a.push({id:++id,px:x1,py:y1,x:x2,y:y2});
}
const good=G.estimate(a,K);
assert.equal(good.ok,true,JSON.stringify(good));
assert.ok(good.inliers>70,JSON.stringify(good));
assert.equal(good.metricTranslation,null);
assert.equal(good.translationDirectionVerified,false);
assert.equal(G.estimate(a.slice(0,7),K).reason,'insufficient_tracks');
assert.equal(G.estimate(a,null).reason,'intrinsics_missing');
const still=G.estimate(a.map(p=>({...p,x:p.px,y:p.py})),K);
assert.equal(still.ok,false,JSON.stringify(still));
assert.equal(still.reason,'low_parallax_or_spatial_degeneracy');
console.log('V3.11 epipolar geometry diagnostic PASS');