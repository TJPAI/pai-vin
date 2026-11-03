/* PAI-VIN V3.16 calibration readiness gate.
 * A4 homography is ONLY a plane measurement. This module never invents
 * intrinsics, body-camera pose, exposure timestamp or time offset.
 * A user-supplied "verified" flag alone is insufficient as provenance.
 */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioCalibration=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const finite=Number.isFinite;
const allFinite=(a,n)=>Array.isArray(a)&&a.length===n&&a.every(finite);
const goodEvidence=(e,kind)=>e&&e.method===kind&&typeof e.sessionId==='string'&&e.sessionId.length>=8&&
 typeof e.deviceId==='string'&&e.deviceId.length>=6&&
 typeof e.calibratedAt==='string'&&Number.isFinite(Date.parse(e.calibratedAt))&&
 allFinite(e.reprojectionErrorPx===undefined?[]:[e.reprojectionErrorPx],1)&&e.reprojectionErrorPx>=0;
function check(input={}){
 const intr=input.intrinsics,ext=input.extrinsics,time=input.timing;
 const intrOk=!!(intr&&intr.width>=1&&intr.height>=1&&[intr.fx,intr.fy,intr.cx,intr.cy].every(finite)&&
  intr.fx>0&&intr.fy>0&&intr.cx>=0&&intr.cy>=0&&intr.cx<intr.width&&intr.cy<intr.height&&
  goodEvidence(intr.evidence,'multi_view_camera_intrinsics')&&intr.evidence.reprojectionErrorPx<=2&&
  intr.verified===true);
 const q=ext?.bodyFromCameraQuaternion,qn=allFinite(q,4)?Math.hypot(...q):NaN;
 const extOk=!!(ext&&allFinite(q,4)&&Math.abs(qn-1)<.001&&
  allFinite(ext.bodyFromCameraTranslationM,3)&&
  goodEvidence(ext.evidence,'camera_imu_extrinsics')&&ext.evidence.reprojectionErrorPx<=2&&ext.verified===true);
 const timeOk=!!(time&&finite(time.cameraToIMUOffsetMs)&&Math.abs(time.cameraToIMUOffsetMs)<200&&
  finite(time.offsetUncertaintyMs)&&time.offsetUncertaintyMs>=0&&time.offsetUncertaintyMs<=5&&
  time.cameraExposureTimestampsAvailable===true&&
  goodEvidence(time.evidence,'camera_imu_time_alignment')&&time.verified===true);
 const reasons=[];
 if(!intrOk)reasons.push('camera_intrinsics_unverified');
 if(!extOk)reasons.push('camera_imu_extrinsics_unverified');
 if(!timeOk)reasons.push('camera_imu_timing_unverified');
 const a4=input.a4;
 const a4Valid=!!(a4?.schema==='pai-vin-a4-plane-v1'&&a4.ok===true&&a4.metricPlaneOnly===true&&
  a4.cameraIntrinsicsKnown===false&&a4.globalSLAMScaleKnown===false);
 return {ready:intrOk&&extOk&&timeOk,reasons,cameraIntrinsicsVerified:intrOk,
  cameraIMUExtrinsicsVerified:extOk,timeOffsetVerified:timeOk,a4PlaneAvailable:a4Valid,
  a4OnlyNotSufficient:true,metricPoseAvailable:false,visualFusionEnabled:false,
  cameraExposureTimestampAvailable:time?.cameraExposureTimestampsAvailable===true};
}
function fromBrowser({a4=null,exposureTimestampAvailable=false}={}){
 return check({a4,timing:{cameraExposureTimestampsAvailable:exposureTimestampAvailable}});
}
return {check,fromBrowser};
});