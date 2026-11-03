/* V3.11: calibrated-normalized two-view epipolar diagnostic.
 * Estimates a projective epipolar constraint only; not relative metric pose,
 * camera-IMU calibration, triangulation, or proof of translation.
 */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioGeometry=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const median=a=>{if(!a.length)return null;const b=a.slice().sort((x,y)=>x-y);return b[b.length>>1]};
function eigenMin(A){
 const n=A.length,a=A.map(r=>r.slice()),v=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>+(i===j)));
 for(let k=0;k<150;k++){
  let p=0,q=1,max=0;
  for(let i=0;i<n;i++)for(let j=i+1;j<n;j++)if(Math.abs(a[i][j])>max){max=Math.abs(a[i][j]);p=i;q=j}
  if(max<1e-11)break;
  const phi=.5*Math.atan2(2*a[p][q],a[q][q]-a[p][p]),c=Math.cos(phi),s=Math.sin(phi);
  const app=a[p][p],aqq=a[q][q],apq=a[p][q];
  a[p][p]=c*c*app-2*s*c*apq+s*s*aqq;a[q][q]=s*s*app+2*s*c*apq+c*c*aqq;
  a[p][q]=a[q][p]=0;
  for(let i=0;i<n;i++)if(i!==p&&i!==q){const x=a[i][p],y=a[i][q];a[i][p]=a[p][i]=c*x-s*y;a[i][q]=a[q][i]=s*x+c*y}
  for(let i=0;i<n;i++){const x=v[i][p],y=v[i][q];v[i][p]=c*x-s*y;v[i][q]=s*x+c*y}
 }
 let i=0;for(let j=1;j<n;j++)if(a[j][j]<a[i][i])i=j;
 return v.map(r=>r[i]);
}
function fit(pairs){
 const a=Array.from({length:9},()=>Array(9).fill(0));
 for(const p of pairs){
  const {x,y,u,v}=p,r=[u*x,u*y,u,v*x,v*y,v,x,y,1];
  for(let i=0;i<9;i++)for(let j=i;j<9;j++)a[i][j]+=r[i]*r[j];
 }
 for(let i=0;i<9;i++)for(let j=0;j<i;j++)a[i][j]=a[j][i];
 const f=eigenMin(a),norm=Math.hypot(...f);
 return norm>1e-10?f.map(x=>x/norm):null;
}
function error(f,p){
 const {x,y,u,v}=p;
 const ex=f[0]*x+f[1]*y+f[2],ey=f[3]*x+f[4]*y+f[5];
 const tx=f[0]*u+f[3]*v+f[6],ty=f[1]*u+f[4]*v+f[7];
 const z=u*ex+v*ey+f[6]*x+f[7]*y+f[8];
 const denom=ex*ex+ey*ey+tx*tx+ty*ty;
 return denom>1e-14?Math.abs(z)/Math.sqrt(denom):Infinity;
}
function estimate(matches,intr,opts={}){
 const min=opts.minInliers??20;
 if(!intr||![intr.fx,intr.fy,intr.cx,intr.cy].every(Number.isFinite)||intr.fx<=0||intr.fy<=0)
  return {ok:false,reason:'intrinsics_missing',total:0,inliers:0};
 const pairs=(matches||[]).filter(p=>[p.px,p.py,p.x,p.y].every(Number.isFinite))
  .map(p=>({x:(p.px-intr.cx)/intr.fx,y:(p.py-intr.cy)/intr.fy,u:(p.x-intr.cx)/intr.fx,v:(p.y-intr.cy)/intr.fy,px:p.px,py:p.py,dx:p.x-p.px,dy:p.y-p.py}));
 const total=pairs.length;
 if(total<Math.max(8,min))return {ok:false,reason:'insufficient_tracks',total,inliers:0};
 const threshold=opts.threshold??.005,attempts=opts.attempts??75;
 let best=[],bestF=null,state=123456789;
 for(let k=0;k<attempts;k++){
  const selected=[],used=new Set();
  while(selected.length<8){state=(Math.imul(state,1664525)+1013904223)>>>0;const i=state%total;if(!used.has(i)){used.add(i);selected.push(pairs[i])}}
  const f=fit(selected);if(!f)continue;
  const inliers=pairs.filter(p=>error(f,p)<threshold);
  if(inliers.length>best.length){best=inliers;bestF=f}
 }
 if(best.length<min)return {ok:false,reason:'epipolar_geometry_unreliable',total,inliers:best.length,ratio:best.length/total};
 const f=fit(best)||bestF;
 const inliers=pairs.filter(p=>error(f,p)<threshold),ratio=inliers.length/total;
 const errors=inliers.map(p=>error(f,p));
 const quadrants=new Set(inliers.map(p=>(p.px>=intr.cx?1:0)+(p.py>=intr.cy?2:0))).size;
 const flow=median(inliers.map(p=>Math.hypot(p.dx,p.dy)));
 // Low motion and weak spatial support make epipolar constraints ambiguous.
 const degenerate=flow<(opts.minFlowPx??1.5)||quadrants<3;
 return {ok:ratio>=(opts.minRatio??.65)&&inliers.length>=min&&!degenerate,
  reason:degenerate?'low_parallax_or_spatial_degeneracy':ratio<(opts.minRatio??.65)?'epipolar_geometry_unreliable':'epipolar_consistent',
  total,inliers:inliers.length,ratio,medianSampson:median(errors),medianFlowPx:flow,occupiedQuadrants:quadrants,
  epipolarMatrix:f,translationDirectionVerified:false,metricTranslation:null,
  calibrationVerified:intr.verified===true};
}
return {estimate,fit,error};
});