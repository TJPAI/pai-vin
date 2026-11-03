/* Experimental calibrated two-view geometry in pure JS.
 * Assumes APPROXIMATE focal length, not factory calibration. Translation is
 * direction-only and unscaled. Not a metric pose estimator or full SLAM. */
(function(root,factory){
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 else root.VINVO=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
 const norm=a=>Math.sqrt(dot(a,a));
 const unit=a=>{const n=norm(a)||1;return a.map(v=>v/n)};
 const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
 const tr=A=>A[0].map((_,i)=>A.map(r=>r[i]));
 const mv=(A,v)=>A.map(r=>dot(r,v));
 const mm=(A,B)=>A.map(r=>tr(B).map(c=>dot(r,c)));
 const det=A=>dot(A[0],cross(A[1],A[2]));
 function eigSym(A){
  const n=A.length,a=A.map(r=>r.slice()),v=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>+(i===j)));
  for(let k=0;k<n*n*30;k++){
   let p=0,q=1,off=0;
   for(let i=0;i<n;i++)for(let j=i+1;j<n;j++){const ab=Math.abs(a[i][j]);if(ab>off){off=ab;p=i;q=j}}
   if(off<1e-11)break;
   const phi=.5*Math.atan2(2*a[p][q],a[q][q]-a[p][p]);
   const c=Math.cos(phi),s=Math.sin(phi);
   // Rotation columns p=(c,-s), q=(s,c).
   const ap=a[p][p],aq=a[q][q],b=a[p][q];
   a[p][p]=c*c*ap-2*s*c*b+s*s*aq;
   a[q][q]=s*s*ap+2*s*c*b+c*c*aq;
   a[p][q]=a[q][p]=0;
   for(let i=0;i<n;i++)if(i!==p&&i!==q){
    const ip=a[i][p],iq=a[i][q];
    a[i][p]=a[p][i]=c*ip-s*iq;
    a[i][q]=a[q][i]=s*ip+c*iq;
   }
   for(let i=0;i<n;i++){const ip=v[i][p],iq=v[i][q];v[i][p]=c*ip-s*iq;v[i][q]=s*ip+c*iq}
  }
  const ordered=Array.from({length:n},(_,i)=>({val:a[i][i],vec:v.map(r=>r[i])})).sort((x,y)=>x.val-y.val);
  return ordered;
 }
 function svd3(A){
  const vals=eigSym(mm(tr(A),A)).reverse();
  let v0=unit(vals[0].vec),v1=unit(vals[1].vec);
  v1=unit(v1.map((x,i)=>x-dot(v1,v0)*v0[i]));
  const v2=unit(cross(v0,v1));
  const ss=vals.map(x=>Math.sqrt(Math.max(0,x.val)));
  const u0=unit(mv(A,v0));let u1=mv(A,v1);
  u1=unit(u1.map((x,i)=>x-dot(u1,u0)*u0[i]));
  let u2=unit(cross(u0,u1));
  if(dot(u2,mv(A,v2))<0)u2=u2.map(x=>-x);
  return {U:tr([u0,u1,u2]),S:ss,V:tr([v0,v1,v2])};
 }
 function eight(matches){
  const ata=Array.from({length:9},()=>Array(9).fill(0));
  for(const m of matches){
   const {x,y,u,v}=m,row=[u*x,u*y,u,v*x,v*y,v,x,y,1];
   for(let i=0;i<9;i++)for(let j=i;j<9;j++)ata[i][j]+=row[i]*row[j];
  }
  for(let i=0;i<9;i++)for(let j=0;j<i;j++)ata[i][j]=ata[j][i];
  const f=eigSym(ata)[0].vec;
  if(!f||f.some(v=>!Number.isFinite(v)))return null;
  const E=[f.slice(0,3),f.slice(3,6),f.slice(6,9)];
  const {U,S,V}=svd3(E);
  if(S[1]<1e-9)return null;
  const sig=(S[0]+S[1])/2;
  const clean=mm(mm(U,[[sig,0,0],[0,sig,0],[0,0,0]]),tr(V));
  return clean;
 }
 function sampson(E,m){
  const p=[m.x,m.y,1],q=[m.u,m.v,1],e=mv(E,p),et=mv(tr(E),q);
  const v=dot(q,e),den=e[0]*e[0]+e[1]*e[1]+et[0]*et[0]+et[1]*et[1];
  return den<1e-14?Infinity:v*v/den;
 }
 function decompose(E){
  const {U,V}=svd3(E);
  let u=U.map(r=>r.slice()),v=V.map(r=>r.slice());
  if(det(u)<0)for(let i=0;i<3;i++)u[i][2]*=-1;
  if(det(v)<0)for(let i=0;i<3;i++)v[i][2]*=-1;
  const W=[[0,-1,0],[1,0,0],[0,0,1]];
  const candidates=[W,tr(W)].map(w=>mm(mm(u,w),tr(v)));
  const t=unit(u.map(r=>r[2]));
  return candidates.flatMap(R=>[t,t.map(x=>-x)].map(tv=>({R,t:tv})));
 }
 function positive(candidate,m){
  const a=mv(candidate.R,[m.x,m.y,1]),b=[m.u,m.v,1],t=candidate.t;
  const aa=dot(a,a),bb=dot(b,b),ab=dot(a,b),at=dot(a,t),bt=dot(b,t);
  const den=aa*bb-ab*ab;
  if(den<1e-9)return false;
  const l1=(-at*bb+ab*bt)/den,l2=(aa*bt-ab*at)/den;
  return l1>0&&l2>0;
 }
 function estimate(pointsA,pointsB,options={}){
  const focal=options.focal||220,cx=options.cx??128,cy=options.cy??96;
  const pairs=options.pairs||[];
  if(pairs.length<12)return {ok:false,reason:'too_few_correspondences',matches:pairs.length};
  const data=pairs.map(p=>({
   x:(pointsA[p.i].x-cx)/focal,y:(pointsA[p.i].y-cy)/focal,
   u:(pointsB[p.j].x-cx)/focal,v:(pointsB[p.j].y-cy)/focal
  }));
  let seed=0x44de81,top=[],bestE=null;
  const next=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed};
  const threshold=(options.thresholdPx||2.5)**2/(focal*focal);
  const iterations=options.iterations||56;
  for(let k=0;k<iterations;k++){
   const picks=new Set();while(picks.size<8)picks.add(next()%data.length);
   const E=eight([...picks].map(i=>data[i]));if(!E)continue;
   const inliers=data.filter(m=>sampson(E,m)<threshold);
   if(inliers.length>top.length){top=inliers;bestE=E}
  }
  if(top.length<12||top.length/data.length<.35)return {ok:false,reason:'insufficient_epipolar_support',matches:data.length,inliers:top.length};
  const refined=eight(top)||bestE;
  const poses=decompose(refined),scores=poses.map(p=>top.slice(0,32).filter(m=>positive(p,m)).length);
  const ix=scores.indexOf(Math.max(...scores)),pose=poses[ix];
  const cheirality=scores[ix],polar=Math.acos(Math.max(-1,Math.min(1,(pose.R[0][0]+pose.R[1][1]+pose.R[2][2]-1)/2)))*180/Math.PI;
  const reliable=cheirality>=Math.min(8,Math.ceil(top.length*.35));
  return {ok:reliable,reason:reliable?'relative_direction_only':'cheirality_ambiguous',matches:data.length,inliers:top.length,
   ratio:top.length/data.length,rotationDeg:polar,R:pose.R,t:pose.t,cheirality,focalAssumedPx:focal,metricScaleKnown:false};
 }
 return {estimate,eigSym,svd3,eight,sampson,decompose,positive};
});