/* PAI-VIN PWA similarity geometry: camera viewpoint recognition diagnostic,
 * NOT six-degree-of-freedom pose estimation or SLAM.
 * UMD: node tests and Safari without bundlers. */
(function(root,factory){
 const API=factory(); if(typeof module==='object'&&module.exports)module.exports=API;
 else root.VINGeometry=API;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 function pop32(x){x=(x-((x>>>1)&0x55555555))>>>0;x=((x&0x33333333)+((x>>>2)&0x33333333))>>>0;return Math.imul((x+(x>>>4))&0x0f0f0f0f,0x01010101)>>>24}
 function distance(a,b){return pop32((a.a^b.a)>>>0)+pop32((a.b^b.b)>>>0)}
 function correspond(a,b){
  if(!a?.length||!b?.length)return [];
  const proposals=[];
  for(let i=0;i<a.length;i++){
   let best=99,second=99,jBest=-1;
   for(let j=0;j<b.length;j++){
    const d=distance(a[i],b[j]);
    if(d<best){second=best;best=d;jBest=j}
    else if(d<second)second=d;
   }
   if(jBest>=0&&best<=18&&best<second*0.82)proposals.push({i,j:jBest,d:best});
  }
  const reciprocal=new Map();
  for(const m of proposals)if(!reciprocal.has(m.j)||reciprocal.get(m.j).d>m.d)reciprocal.set(m.j,m);
  return [...reciprocal.values()];
 }
 function fitPair(p,q,r,s){
  const ax=q.x-p.x,ay=q.y-p.y,bx=s.x-r.x,by=s.y-r.y,den=ax*ax+ay*ay;
  if(den<225)return null;
  const c=(ax*bx+ay*by)/den,z=(ax*by-ay*bx)/den,scale=Math.hypot(c,z);
  if(scale<0.65||scale>1.6)return null;
  return {c,z,tx:r.x-c*p.x+z*p.y,ty:r.y-z*p.x-c*p.y,scale};
 }
 function residual(T,a,b){
  return Math.hypot(T.c*a.x-T.z*a.y+T.tx-b.x,T.z*a.x+T.c*a.y+T.ty-b.y);
 }
 function verify(a,b){
  const ms=correspond(a,b);
  const fail={matches:ms.length,inliers:0,ratio:0,spread:0,rmse:null,angle:null,scale:null,confidence:0,accepted:false};
  if(ms.length<8)return fail;
  let best=[],bestT=null;
  // Deterministic 2-point RANSAC over spatially separated candidate correspondences.
  const limit=Math.min(ms.length,48);
  for(let i=0;i<limit;i++)for(let j=i+1;j<limit;j++){
   const p=a[ms[i].i],q=a[ms[j].i],r=b[ms[i].j],s=b[ms[j].j];
   const T=fitPair(p,q,r,s);if(!T)continue;
   const inliers=ms.filter(m=>residual(T,a[m.i],b[m.j])<5);
   if(inliers.length>best.length){best=inliers;bestT=T}
  }
  if(!bestT)return fail;
  const buckets=new Set(),errors=[];
  for(const m of best){
   const p=a[m.i];buckets.add((p.x<128?0:1)+(p.y<96?0:2));
   errors.push(residual(bestT,p,b[m.j]));
  }
  const spread=buckets.size;
  const rmse=Math.sqrt(errors.reduce((acc,e)=>acc+e*e,0)/errors.length);
  const ratio=best.length/ms.length;
  const accepted=best.length>=10&&ratio>=0.45&&spread>=3&&rmse<4;
  return {matches:ms.length,inliers:best.length,ratio,spread,rmse,
          angle:Math.atan2(bestT.z,bestT.c)*180/Math.PI,scale:bestT.scale,
          confidence:best.length*ratio*(spread/4),accepted};
 }
 function rank(query,map){
  const all=map.map((f,index)=>({index,...verify(query,f.points)}))
    .sort((a,b)=>b.confidence-a.confidence);
  const first=all[0]||null,second=all[1]||null;
  const distinct=!second||first?.confidence>=second.confidence*1.13||first?.inliers>=second.inliers+5;
  return {best:first,runner:second,accepted:Boolean(first?.accepted&&distinct),ambiguous:Boolean(first?.accepted&&!distinct)};
 }
 return {distance,correspond,verify,rank};
});
