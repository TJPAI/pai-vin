/* Lightweight 2D image keypoints for offline feasibility experiments.
 * Not SLAM, not PnP, and no calibrated location confidence. */
(function(global){
'use strict';
const W=320,H=240;
const pairs=Array.from({length:64},(_,i)=>{
 const angle=(i*2.399963229728653)% (2*Math.PI);
 const r=3+(i*7%8),r2=2+(i*11%9);
 return [Math.round(Math.cos(angle)*r),Math.round(Math.sin(angle)*r),Math.round(Math.cos(angle+1.77)*r2),Math.round(Math.sin(angle+1.77)*r2)];
});
function extract(raw){
 const gray=new Uint8Array(W*H);
 for(let i=0,j=0;i<raw.length;i+=4,j++)gray[j]=(77*raw[i]+150*raw[i+1]+29*raw[i+2])>>8;
 const points=[];
 // Choose locally distinctive locations on a coarse grid, avoiding image edges.
 for(let ty=16;ty<H-16;ty+=16)for(let tx=16;tx<W-16;tx+=16){
   let best=0,bx=0,by=0;
   for(let y=ty;y<Math.min(ty+16,H-12);y+=3)for(let x=tx;x<Math.min(tx+16,W-12);x+=3){
     const at=y*W+x, gx=gray[at+2]-gray[at-2], gy=gray[at+2*W]-gray[at-2*W];
     const d1=gray[at+W+2]-gray[at-W-2],d2=gray[at-W+2]-gray[at+W-2];
     const score=Math.min(gx*gx+gy*gy, d1*d1+d2*d2);
     if(score>best){best=score;bx=x;by=y}
   }
   if(best<650)continue;
   let lo=0,hi=0;
   for(let k=0;k<64;k++){
     const p=pairs[k],a=gray[(by+p[1])*W+bx+p[0]],b=gray[(by+p[3])*W+bx+p[2]];
     if(a<b){if(k<32)lo=(lo|(1<<k));else hi=(hi|(1<<(k-32)))}
   }
   points.push({x:bx,y:by,lo:lo>>>0,hi:hi>>>0});
 }
 return points.slice(0,240);
}
function popcnt(v){v-=((v>>>1)&0x55555555);v=(v&0x33333333)+((v>>>2)&0x33333333);return (((v+(v>>>4))&0x0F0F0F0F)*0x01010101)>>>24}
function dist(a,b){return popcnt(a.lo^b.lo)+popcnt(a.hi^b.hi)}
function match(query,reference){
 if(!Array.isArray(query)||!Array.isArray(reference)||query.length<12||reference.length<12)return {accepted:false,inliers:0,raw:0,ratio:0,reason:'纹理特征太少'};
 const matches=[];
 for(const p of query){
   let best=65,second=65,idx=-1;
   for(let i=0;i<reference.length;i++){const d=dist(p,reference[i]);if(d<best){second=best;best=d;idx=i}else if(d<second)second=d}
   if(best<=19 && best<second*.76)matches.push({p,q:reference[idx],d:best});
 }
 if(matches.length<10)return {accepted:false,inliers:0,raw:matches.length,ratio:0,reason:'有效局部特征匹配不足'};
 let bestSet=[];
 // Deterministic translation consensus; only suitable for very similar viewpoints.
 for(let i=0;i<matches.length;i++){
   const dx=matches[i].q.x-matches[i].p.x,dy=matches[i].q.y-matches[i].p.y;
   let set=matches.filter(m=>Math.hypot((m.q.x-m.p.x)-dx,(m.q.y-m.p.y)-dy)<12);
   if(set.length>bestSet.length)bestSet=set;
 }
 const ratio=bestSet.length/matches.length;
 const xs=bestSet.map(m=>m.p.x),ys=bestSet.map(m=>m.p.y);
 const spread=(Math.max(...xs)-Math.min(...xs))*(Math.max(...ys)-Math.min(...ys))/(W*H);
 const accepted=bestSet.length>=12&&ratio>=.58&&spread>=.065;
 return {accepted,inliers:bestSet.length,raw:matches.length,ratio:+ratio.toFixed(3),spread:+spread.toFixed(3),reason:accepted?'局部特征匹配通过':'匹配几何一致性或分布不足'};
}
global.VINVision={extract,match};
})(window);