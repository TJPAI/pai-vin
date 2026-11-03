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
 if(den<1e-8)return {ok:false,reason:'insufficient_perspective_diversity'};
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
  if(d<1e-8)return {ok:false,reason:'insufficient_perspective_diversity'};
  const z=rr.reduce((s,r)=>s+r.x*r.y,0)/d;
  if(z<=0)return {ok:false,reason:'unstable_focal_estimate'};
  loo.push(Math.sqrt(z));
 }
 const stability=Math.max(...loo.map(v=>Math.abs(v-f)/f));
 if(residual>.025||stability>.15)return {ok:false,reason:'multi_view_inconsistent',residual,leaveOneOutRelativeSpread:stability};
 return {ok:true,fx:f,fy:f,cx:width/2,cy:height/2,width,height,views:good.length,
  constraintRms:residual,leaveOneOutRelativeSpread:stability,
  model:'pinhole_square_pixels_zero_skew_centered_principal_point',
  verified:false,distortionCalibrated:false,metricPoseAvailable:false,
  source:'multi_view_a4_constrained_candidate_only'};
}
return {estimate,homography,candidate};
});