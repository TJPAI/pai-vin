const assert=require('node:assert/strict'),F=require('../vio-focal.js');
const W=360,H=480,fx=390,fy=390,cx=W/2,cy=H/2;
function view(ax,ay,az,tx,ty,tz){
 const c=Math.cos,s=Math.sin;
 const Rx=[[1,0,0],[0,c(ax),-s(ax)],[0,s(ax),c(ax)]];
 const Ry=[[c(ay),0,s(ay)],[0,1,0],[-s(ay),0,c(ay)]];
 const Rz=[[c(az),-s(az),0],[s(az),c(az),0],[0,0,1]];
 const mm=(A,B)=>A.map(row=>B[0].map((_,j)=>row.reduce((z,q,k)=>z+q*B[k][j],0)));
 const R=mm(Rz,mm(Ry,Rx));
 return [[0,0],[.16,0],[.16,.247],[0,.247]].map(([x,y])=>{
  const X=R[0][0]*x+R[0][1]*y+tx,Y=R[1][0]*x+R[1][1]*y+ty,Z=R[2][0]*x+R[2][1]*y+tz;
  return [cx+fx*X/Z,cy+fy*Y/Z];
 });
}
const angles=[[.3,.35,0],[-.34,.2,.2],[.25,-.35,-.1],[-.28,-.22,.08],[.37,.12,-.1],[.15,.4,.15]];
const views=angles.map((a,i)=>view(...a,-.08+(i%3)*.02,-.12+(i%2)*.01,.43+(i%3)*.05));
const k=F.estimate(views,W,H);
assert.equal(k.ok,true,JSON.stringify(k));
assert.ok(Math.abs(k.fx-fx)<1e-4,JSON.stringify(k));
assert.ok(k.reprojectionRmsPx<1e-4,JSON.stringify(k));
assert.ok(k.maxViewReprojectionRmsPx<1e-4,JSON.stringify(k));
assert.equal(k.perViewReprojectionRmsPx.length,views.length);
assert.equal(k.reprojectionMeasurement,'same_four_markers_constrained_model_fit_not_independent');
assert.equal(k.verified,false);assert.equal(k.distortionCalibrated,false);assert.equal(k.metricPoseAvailable,false);
assert.equal(F.estimate(views.slice(0,1),W,H).reason,'need_five_views');
assert.equal(F.estimate(views,0,H).reason,'invalid_resolution');
assert.equal(F.estimate(Array(6).fill(views[0]),W,H).ok,false,'repeated view must be rejected');
const bad=views.map(v=>v.map(p=>p.slice()));bad[2][0][0]+=32;
assert.equal(F.estimate(bad,W,H).ok,false,'corrupt corner must be rejected');
console.log('Constrained multi-view A4 focal estimate and degeneracy gates PASS');