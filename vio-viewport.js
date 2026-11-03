/* PAI-VIN V3.19: camera viewport geometry and cross-stream safety gate.
 * Transforms a pinhole K only between measured canvas/source rectangles.
 * No claim that two getUserMedia streams share optics, crop, orientation or FOV.
 */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioViewport=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
function rect(sourceWidth,sourceHeight,canvasWidth,canvasHeight,fit){
 if(![sourceWidth,sourceHeight,canvasWidth,canvasHeight].every(v=>Number.isFinite(v)&&v>0))
  return {ok:false,reason:'invalid_dimensions'};
 if(!['contain','cover','stretch'].includes(fit))return {ok:false,reason:'invalid_fit'};
 const sx=canvasWidth/sourceWidth,sy=canvasHeight/sourceHeight;
 const scale=fit==='contain'?Math.min(sx,sy):fit==='cover'?Math.max(sx,sy):null;
 const scaleX=scale??sx,scaleY=scale??sy;
 return {ok:true,sourceWidth,sourceHeight,canvasWidth,canvasHeight,fit,
  visibleFraction:fit==='cover'?Math.min(1,(canvasWidth*canvasHeight)/(sourceWidth*sourceHeight*scaleX*scaleY)):1,
  scaleX,scaleY,offsetX:(canvasWidth-sourceWidth*scaleX)/2,
  offsetY:(canvasHeight-sourceHeight*scaleY)/2,
  aspectDistorted:Math.abs(scaleX-scaleY)>1e-8,
  sourceFrameRotated:false};
}
function sourceCrop(g){
 if(!g?.ok||g.fit!=='cover'||g.aspectDistorted)
  return {ok:false,reason:'requires_undistorted_cover'};
 const x=Math.max(0,-g.offsetX/g.scaleX),y=Math.max(0,-g.offsetY/g.scaleY);
 const width=g.canvasWidth/g.scaleX,height=g.canvasHeight/g.scaleY;
 if(x+width>g.sourceWidth+1e-6||y+height>g.sourceHeight+1e-6)
  return {ok:false,reason:'crop_outside_video'};
 return {ok:true,x,y,width,height};
}
function transform(K,from,to){
 if(!from?.ok||!to?.ok||![K?.fx,K?.fy,K?.cx,K?.cy].every(Number.isFinite))
  return {ok:false,reason:'invalid_intrinsics_or_geometry'};
 if(from.sourceWidth!==to.sourceWidth||from.sourceHeight!==to.sourceHeight)
  return {ok:false,reason:'source_frame_size_mismatch'};
 // Map canvas A -> common original video pixels -> canvas B.
 const sX=to.scaleX/from.scaleX,sY=to.scaleY/from.scaleY;
 return {ok:true,fx:K.fx*sX,fy:K.fy*sY,
  cx:(K.cx-from.offsetX)*sX+to.offsetX,
  cy:(K.cy-from.offsetY)*sY+to.offsetY,
  width:to.canvasWidth,height:to.canvasHeight,
  verified:false,source:'same_source_frame_coordinate_transform_only'};
}
function compatibility(calibration,tracking,opts={}){
 if(!calibration?.ok||!tracking?.ok)return {ok:false,reason:'missing_frame_geometry'};
 if(calibration.sourceWidth!==tracking.sourceWidth||calibration.sourceHeight!==tracking.sourceHeight)
  return {ok:false,reason:'source_resolution_mismatch',calibrationSource:[calibration.sourceWidth,calibration.sourceHeight],
   trackingSource:[tracking.sourceWidth,tracking.sourceHeight]};
 if(opts.sameStream===true){
  const transformed=opts.intrinsics?transform(opts.intrinsics,calibration,tracking):null;
  return {ok:true,reason:'same_live_camera_stream',sameSourceDimensions:true,
   candidateCoordinateTransformAvailable:!!(transformed?.ok),
   candidateIntrinsics:transformed?.ok?transformed:null,
   candidateVerified:false,metricPoseAvailable:false,
   exposureTimeKnown:false,externalCalibrationVerified:false};
 }
 // Separate media streams may select a different active camera/lens/FOV even
 // when the image dimensions and deviceId happen to match.
 return {ok:false,reason:'independent_stream_fov_unverified',
  sameSourceDimensions:true,candidateCoordinateTransformAvailable:true,
  metricPoseAvailable:false};
}
return {rect,sourceCrop,transform,compatibility};
});