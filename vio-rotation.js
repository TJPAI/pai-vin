/* PAI-VIN V3.5: geometrically checked relative visual rotation.
 * Uses stable-ID 2D tracks and caller-supplied camera intrinsics; returns
 * CAMERA-frame relative orientation only, never metric translation.
 * Do not fuse into IMU body attitude without measured camera–IMU extrinsics
 * and timing offset. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioRotation=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const norm=a=>Math.hypot(...a),dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit=a=>{const n=norm(a);return n?a.map(v=>v/n):null};
const skew=(a)=>[[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]];
const mv=(M,v)=>M.map(row=>dot(row,v));
const mm=(A,B)=>A.map(row=>B[0].map((_,j)=>dot(row,B.map(r=>r[j]))));
const I=[[1,0,0],[0,1,0],[0,0,1]];
const add=(A,B)=>A.map((r,i)=>r.map((v,j)=>v+B[i][j]));
const mul=(A,c)=>A.map(r=>r.map(v=>v*c));
const transpose=A=>A[0].map((_,j)=>A.map(r=>r[j]));
function expSO3(w){
 const angle=norm(w),K=skew(w);
 if(angle<1e-10)return add(I,K);
 const k=mul(K,Math.sin(angle)/angle),k2=mul(mm(K,K),(1-Math.cos(angle))/(angle*angle));
 return add(add(I,k),k2);
}
function solve3(A,b){
 const a=A.map((r,i)=>[...r,b[i]]);
 for(let j=0;j<3;j++){
  let p=j;for(let i=j+1;i<3;i++)if(Math.abs(a[i][j])>Math.abs(a[p][j]))p=i;
  if(Math.abs(a[p][j])<1e-10)return null;
  [a[j],a[p]]=[a[p],a[j]];
  const d=a[j][j];for(let k=j;k<4;k++)a[j][k]/=d;
  for(let i=0;i<3;i++)if(i!==j){const f=a[i][j];for(let k=j;k<4;k++)a[i][k]-=f*a[j][k]}
 }return a.map(r=>r[3]);
}
function bearings(matches,intr){
 if(!intr||![intr.fx,intr.fy,intr.cx,intr.cy].every(Number.isFinite)||intr.fx<=0||intr.fy<=0)return null;
 return matches.filter(m=>[m.px,m.py,m.x,m.y].every(Number.isFinite)).map(m=>({
  a:unit([(m.px-intr.cx)/intr.fx,(m.py-intr.cy)/intr.fy,1]),
  b:unit([(m.x-intr.cx)/intr.fx,(m.y-intr.cy)/intr.fy,1]),id:m.id
 })).filter(p=>p.a&&p.b);
}
function fitRotation(pairs,iterations=7){
 let R=I;
 for(let step=0;step<iterations;step++){
  let H=[[0,0,0],[0,0,0],[0,0,0]],g=[0,0,0];
  for(const p of pairs){
   const u=mv(R,p.a),v=p.b,err=v.map((x,i)=>x-u[i]);
   const J=mul(skew(u),-1),JT=transpose(J);
   H=add(H,mm(JT,J));g=g.map((x,i)=>x+mv(JT,err)[i]);
  }
  const delta=solve3(H,g);
  if(!delta||!delta.every(Number.isFinite))return null;
  R=mm(expSO3(delta),R);
  if(norm(delta)<1e-7)break;
 }
 return R;
}
const angular=(a,b)=>Math.acos(Math.max(-1,Math.min(1,dot(a,b))));
function estimate(matches,intr,options={}){
 const pairs=bearings(matches,intr),min=options.minInliers??18;
 if(!pairs||pairs.length<min)return {ok:false,reason:!pairs?'intrinsics_missing':'insufficient_tracks',count:pairs?.length||0,total:pairs?.length||0,inliers:0};
 // deterministic triplet sampling guards against depth-dependent parallax.
 const attempts=Math.min(90,Math.max(30,pairs.length*2));
 const threshold=options.thresholdRad??.012;
 let best=[],bestR=null;
 for(let n=0;n<attempts;n++){
  const i=n%pairs.length,j=(i+1+7*n)%pairs.length,k=(i+2+13*n)%pairs.length;
  if(i===j||i===k||j===k)continue;
  const R=fitRotation([pairs[i],pairs[j],pairs[k]],5);if(!R)continue;
  const kept=pairs.filter(p=>angular(mv(R,p.a),p.b)<threshold);
  if(kept.length>best.length){best=kept;bestR=R}
 }
 if(best.length<min)return {ok:false,reason:'rotation_geometry_unreliable',geometryFailureDetail:'too_few_rotation_inliers',inliers:best.length,total:pairs.length};
 let R=fitRotation(best,8)||bestR;
 const errors=best.map(p=>angular(mv(R,p.a),p.b)).sort((a,b)=>a-b);
 const inliers=pairs.filter(p=>angular(mv(R,p.a),p.b)<threshold);
 const ratio=inliers.length/pairs.length;
 const med=errors[errors.length>>1];
 // Rotation may be predicted but never accepted as camera/body calibration.
 if(ratio<(options.minRatio??.72)||med>(options.maxMedianError??.007))
  return {ok:false,reason:'rotation_geometry_unreliable',geometryFailureDetail:ratio<(options.minRatio??.72)?'low_rotation_inlier_ratio':'high_rotation_residual',total:pairs.length,ratio,inliers:inliers.length,medianErrorRad:med};
 const theta=Math.acos(Math.max(-1,Math.min(1,(R[0][0]+R[1][1]+R[2][2]-1)/2)));
 if(theta>.55)return {ok:false,reason:'rotation_jump',total:pairs.length,inliers:inliers.length,theta};
 return {ok:true,rotationCameraCurrentFromPrevious:R,inliers:inliers.length,total:pairs.length,
  ratio,medianErrorRad:med,theta,metricTranslation:null,absoluteHeadingKnown:false,
  cameraIMUExtrinsicsApplied:false,assumesProvidedIntrinsics:true};
}
return {estimate,expSO3,fitRotation,angular};
});