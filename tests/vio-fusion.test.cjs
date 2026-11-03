const assert=require('node:assert/strict');
const V=require('../vio-rotation.js'),F=require('../vio-fusion.js');
const angle=.09,R=V.expSO3([0,0,angle]);
const camera={ok:true,rotationCameraCurrentFromPrevious:R,inliers:52,ratio:.9};
const previous=[1,0,0,0];
const calibrated={verified:true,cameraIntrinsicsVerified:true,cameraIMUExtrinsicsVerified:true,
 timeOffsetVerified:true,bodyFromCameraQuaternion:[1,0,0,0]};
const pred=[Math.cos(-angle*.5+.019),0,0,Math.sin(-angle*.5+.019)];
let result=F.correct({previousQuaternion:previous,predictedQuaternion:pred,vision:camera,calibration:calibrated,gain:.35});
assert.equal(result.ok,true,JSON.stringify(result));
const target=[Math.cos(-angle/2),0,0,Math.sin(-angle/2)];
assert.ok(F.angular(result.quaternion,target)<F.angular(pred,target));
assert.equal(result.metricPosition,null);
assert.equal(F.correct({previousQuaternion:previous,predictedQuaternion:pred,vision:camera,calibration:{}}).reason,'uncalibrated_camera_imu');
assert.equal(F.correct({previousQuaternion:previous,predictedQuaternion:pred,vision:{...camera,inliers:4},calibration:calibrated}).reason,'visual_rotation_unreliable');
const opposing=[Math.cos(angle/2),0,0,Math.sin(angle/2)];
assert.equal(F.correct({previousQuaternion:previous,predictedQuaternion:opposing,vision:camera,calibration:calibrated}).reason,'visual_imu_disagreement');
assert.equal(F.correct({previousQuaternion:previous,predictedQuaternion:pred,vision:camera,calibration:calibrated,gain:.7}).reason,'invalid_gain');
console.log('Calibrated visual inertial orientation correction tests PASS');
