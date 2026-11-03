const assert=require('node:assert/strict');
const M=require('../vio-vo-state.js'),R=require('../vio-rotation.js');
const st=new M.State();
assert.equal(st.snapshot().metricPositionAvailable,false);
const step=R.expSO3([0,0,.07]),v={ok:true,rotationCameraCurrentFromPrevious:step,theta:.07,inliers:42,ratio:.9};
for(let i=0;i<10;i++){const s=st.advance(v,null,100+i*100);assert.equal(s.accepted,true);assert.equal(s.segment,1);}
const s=st.snapshot();
assert.equal(s.segmentFrames,10);assert.ok(Math.abs(s.netAngleRad-.7)<1e-5,JSON.stringify(s));
assert.equal(s.relativeTranslationMeters,null);assert.equal(s.visualInertialFused,false);
let broken=st.advance({ok:false,reason:'insufficient_tracks'},null,1200);
assert.equal(broken.accepted,false);assert.equal(broken.orientationCurrentFromSegmentStart,null);
const recovered=st.advance(v,{ok:false,reason:'low_parallax_or_spatial_degeneracy'},1300);
assert.equal(recovered.segment,2);assert.equal(recovered.segmentFrames,1);
assert.ok(Math.abs(recovered.netAngleRad-.07)<1e-5);
assert.equal(st.advance({...v,rotationCameraCurrentFromPrevious:[[2,0,0],[0,1,0],[0,0,1]]},null,1400).accepted,false);
assert.throws(()=>st.advance(v,null,1400),/non_monotonic/);
st.reset();assert.equal(st.snapshot().acceptedSteps,0);
console.log('V3.28 orientation-only VO state safety and dropout segmentation PASS');