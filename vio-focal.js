/* PAI-VIN V3.17: constrained planar multi-view focal estimator.
 * Assumes zero skew, fx=fy and fixed center principal point. This is a
 * candidate only, NOT a verified intrinsic calibration or distortion fit.
 * Four automatically detected A4 corners from each distinct view are used.
 */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioFocal=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const BW=.16,BH=.247,world=[[0,0],[BW,0],[BW,BH],[0,BH]];
function solve(A,b){
 const n=b.length,M=A.map((r,i)=>[...r,b[i]]);
 for(let i=0;i<n;i++){
  let p=i;for(let j=i+1;j<n;j++)if(Math.abs(M[j][i])>Math.abs(M[p][i]))p=j;
  if(Math.abs(M[p][i])<1e-10)return null;
  [M[i],M[p]]=[M[p],M[i]];
  const d=M[i][i];for(let k=i;k<=n;k++)M[i][k]/=d;
  for(let j=0;j<n;j++)if(j!==i){const a=M[j][i];for(let k=i;k<=n;k++)M[j][k]-=a*M[i][k]}
 }
 return M.map(r=>r[n]);
}
function homography(pts){
 const A=[],b=[];
 for(let i=0;i<4;i++){
  const [x,y]=world[i],[u,v]=pts[i];
  A.push([x,y,1,0,0,0,-u*x,-u*y]);b.push(u);
  A.push([0,0,0,x,y,1,-v*x,-v*y]);b.push(v);
 }
 const s=solve(A,b);return s&&[...s,1];
}
function candidate(view,width,height){
 if(!Array.isArray(view)||view.length!==4||view.some(p=>!Array.isArray(p)||p.length!==2||p.some(x=>!Number.isFinite(x))))return null;
 const H=homography(view);if(!H)return null;
 const cx=width/2,cy=height/2;
 const a=[H[0]-cx*H[6],H[3]-cy*H[6],H[6]];
 const b=[H[1]-cx*H[7],H[4]-cy*H[7],H[7]];
 const dot=(u,v)=>u[0]*v[0]+u[1]*v[1];
 const n1=dot(a,a),n2=dot(b,b);
 const rows=[
  {x:a[2]*b[2],y:-dot(a,b),scale:Math.sqrt(n1*n2)},
  {x:a[2]*a[2]-b[2]*b[2],y:n2-n1,scale:n1+n2}
 ];
 return {H,rows:rows.filter(r=>r.scale>1e-14).map(r=>({x:r.x/r.scale,y:r.y/r.scale})),
  tilt:Math.hypot(a[2],b[2])/(Math.sqrt(n1+n2)||1)};
}
// Project the same four markers through a constrained pinhole camera pose.
// This is an in-sample model-fit residual, NOT independent reprojection QA:
// four corners also determine each homography.
function constrainedReprojection(view,H,f,cx,cy){
 const vdot=(a,b)=>a.reduce((sum,x,i)=>sum+x*b[i],0);
 const vn=a=>Math.hypot(...a);
 const unit=a=>a.map(x=>x/vn(a));
 const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
 const a=[(H[0]-cx*H[6])/f,(H[3]-cy*H[6])/f,H[6]];
 const b=[(H[1]-cx*H[7])/f,(H[4]-cy*H[7])/f,H[7]];
 const c=[(H[2]-cx*H[8])/f,(H[5]-cy*H[8])/f,H[8]];
 const n1=vn(a),n2=vn(b),factor=(n1+n2)/2;
 if(factor<1e-12)return null;
 const r1=unit(a),parallel=vdot(r1,b);
 const perp=b.map((x,i)=>x-parallel*r1[i]);
 if(vn(perp)<1e-12)return null;
 const r2=unit(perp),r3=cross(r1,r2),t=c.map(x=>x/factor);
 if(r3[2]===undefined)return null;
 let sum=0;
 for(let i=0;i<4;i++){
  const [x,y]=world[i],X=r1[0]*x+r2[0]*y+t[0],Y=r1[1]*x+r2[1]*y+t[1],Z=r1[2]*x+r2[2]*y+t[2];
  if(Z<=0)return null;
  const px=cx+f*X/Z,py=cy+f*Y/Z;
  sum+=(px-view[i][0])**2+(py-view[i][1])**2;
 }
 return Math.sqrt(sum/4);
}
// Repeatability/identifiability QA on the SAME corner set, not accuracy certification.
function quality(views,width,height,estimateResult){
 if(!estimateResult?.ok)return {status:'unavailable',reasons:['no_valid_focal_candidate'],independentValidation:false};
 const computed=views.map(v=>candidate(v,width,height));
 const tilts=computed.map(x=>x?.tilt??0);
 const spread=Math.max(...tilts)-Math.min(...tilts);
 const leaveOneOut=estimateResult.leaveOneOutRelativeSpread;
 const perView=estimateResult.perViewReprojectionRmsPx||[];
 const reasons=[];
 if(spread<.015)reasons.push('limited_perspective_diversity');
 if(leaveOneOut>.01)reasons.push('leave_one_out_focal_variation_gt_1pct');
 if(estimateResult.reprojectionRmsPx>1)reasons.push('in_sample_reprojection_gt_1px');
 if(perView.some(x=>x>1.8))reasons.push('individual_view_fit_gt_1_8px');
 // Distinguish unstable corner data from pinhole-model limitations.
 // Do not instruct repeated shooting when leave-one-out focal is already stable.
 const captureReview=leaveOneOut>.01;
 const modelLimit=estimateResult.reprojectionRmsPx>1||perView.some(x=>x>1.8);
 const status=captureReview?'review_capture':modelLimit?'model_limited':'candidate_consistent';
 return {status,needsAdditionalViews:captureReview,
  modelLimitDetected:modelLimit,
  guidance:captureReview?'capture_more_diverse_views':modelLimit?'improve_intrinsic_distortion_model':'no_additional_capture_needed',
  reasons,tiltSpread:spread,tilts,leaveOneOutRelativeSpread:leaveOneOut,
  thresholdRelative:0.01,inSampleReprojectionRmsPx:estimateResult.reprojectionRmsPx,
  independentValidation:false,distortionEstimated:false,
  crossSessionRepeatabilityVerified:false,
  certifiedIntrinsics:false};
}
function estimate(views,width,height){
 if(!Number.isInteger(width)||!Number.isInteger(height)||width<100||height<100)
  return {ok:false,reason:'invalid_resolution'};
 if(!Array.isArray(views)||views.length<5)return {ok:false,reason:'need_five_views',collected:views?.length||0};
 // Repeating the same perspective cannot count as independent calibration evidence.
 for(let i=0;i<views.length;i++)for(let j=0;j<i;j++){
  if(Array.isArray(views[i])&&Array.isArray(views[j])&&
   views[i].length===4&&views[j].length===4&&
   views[i].every((p,k)=>Array.isArray(p)&&Array.isArray(views[j][k])&&
    Math.hypot(p[0]-views[j][k][0],p[1]-views[j][k][1])<8))
   return {ok:false,reason:'duplicate_view',first:j,second:i};
 }
 const good=views.map(v=>candidate(v,width,height));
 if(good.some(v=>!v))return {ok:false,reason:'invalid_view'};
 const rows=good.flatMap(v=>v.rows),den=rows.reduce((s,r)=>s+r.x*r.x,0);
 if(den<1e-16)return {ok:false,reason:'insufficient_perspective_diversity'};
 const f2=rows.reduce((s,r)=>s+r.x*r.y,0)/den;
 const f=Math.sqrt(f2);
 if(!Number.isFinite(f)||f<.35*Math.max(width,height)||f>5*Math.max(width,height))
  return {ok:false,reason:'implausible_focal_length'};
 const residual=Math.sqrt(rows.reduce((s,r)=>s+(r.x*f2-r.y)**2,0)/rows.length);
 // Diversity and leave-one-out stability reject fronto-parallel / repeated data.
 const loo=[];
 for(let skip=0;skip<good.length;skip++){
  const rr=good.filter((_,i)=>i!==skip).flatMap(v=>v.rows);
  const d=rr.reduce((s,r)=>s+r.x*r.x,0);
  if(d<1e-16)return {ok:false,reason:'insufficient_perspective_diversity'};
  const z=rr.reduce((s,r)=>s+r.x*r.y,0)/d;
  if(z<=0)return {ok:false,reason:'unstable_focal_estimate'};
  loo.push(Math.sqrt(z));
 }
 const stability=Math.max(...loo.map(v=>Math.abs(v-f)/f));
 const perViewReprojectionRmsPx=good.map((v,i)=>constrainedReprojection(views[i],v.H,f,width/2,height/2));
 if(perViewReprojectionRmsPx.some(x=>x===null||!Number.isFinite(x)))
  return {ok:false,reason:'invalid_constrained_pose'};
 const reprojectionRmsPx=Math.sqrt(perViewReprojectionRmsPx.reduce((sum,x)=>sum+x*x,0)/good.length);
 const maxViewReprojectionRmsPx=Math.max(...perViewReprojectionRmsPx);
 if(residual>.025||stability>.15||reprojectionRmsPx>2||maxViewReprojectionRmsPx>3)return {ok:false,reason:'multi_view_inconsistent',residual,leaveOneOutRelativeSpread:stability,reprojectionRmsPx,maxViewReprojectionRmsPx};
 const qualitySummary=quality(views,width,height,{ok:true,leaveOneOutRelativeSpread:stability,
  reprojectionRmsPx,perViewReprojectionRmsPx});
 return {ok:true,fx:f,fy:f,cx:width/2,cy:height/2,width,height,views:good.length,
  quality:qualitySummary,
  constraintRms:residual,leaveOneOutRelativeSpread:stability,
  reprojectionRmsPx,maxViewReprojectionRmsPx,perViewReprojectionRmsPx,
  reprojectionMeasurement:'same_four_markers_constrained_model_fit_not_independent',
  model:'pinhole_square_pixels_zero_skew_centered_principal_point',
  verified:false,distortionCalibrated:false,metricPoseAvailable:false,
  source:'multi_view_a4_constrained_candidate_only'};
}
return {estimate,homography,candidate,constrainedReprojection,quality};
});