/* PAI-VIN M1: persistent sparse optical tracks in 256x192 gray frames.
 * Pure JS reference frontend; replaceable by WASM KLT. Stable IDs, patch
 * SSD, coarse-to-fine search, forward-backward validation, re-detection.
 * This outputs 2D image tracks ONLY. It does not estimate world positions. */
(function(root,factory){const m=factory();if(typeof module==='object'&&module.exports)module.exports=m;else root.PAIVioTracks=m;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const W=256,H=192;
function down(g,w,h){
 const tw=w>>1,th=h>>1,o=new Uint8Array(tw*th);
 for(let y=0;y<th;y++)for(let x=0;x<tw;x++){
  const i=(2*y)*w+2*x;o[y*tw+x]=(g[i]+g[i+1]+g[i+w]+g[i+w+1]+2)>>2;
 }return o;
}
function pixels(g,w,h,x,y){
 if(x<3||y<3||x>=w-3||y>=h-3)return false;
 const i=y*w+x;
 const vx=Math.abs(g[i+1]-g[i-1])+Math.abs(g[i+w+1]-g[i+w-1]);
 const vy=Math.abs(g[i+w]-g[i-w])+Math.abs(g[i+w+1]-g[i-w+1]);
 return Math.min(vx,vy)>22;
}
function score(a,b,w,h,x,y,u,v,r=2){
 if(x<r||y<r||u<r||v<r||x>=w-r||u>=w-r||y>=h-r||v>=h-r)return Infinity;
 let s=0,sa=0,sb=0,n=0;
 for(let j=-r;j<=r;j++)for(let i=-r;i<=r;i++){
  const va=a[(y+j)*w+x+i],vb=b[(v+j)*w+u+i];
  sa+=va;sb+=vb;n++;
 }
 const offset=Math.round((sb-sa)/n);
 for(let j=-r;j<=r;j++)for(let i=-r;i<=r;i++){
  const d=a[(y+j)*w+x+i]+offset-b[(v+j)*w+u+i];s+=d*d;
 }
 return s/n;
}
function nearest(a,b,w,h,x,y,px,py,search){
 let best=Infinity,bx=-1,by=-1;
 const sx=Math.round(px),sy=Math.round(py);
 for(let v=sy-search;v<=sy+search;v++)for(let u=sx-search;u<=sx+search;u++){
  const s=score(a,b,w,h,x,y,u,v);
  if(s<best){best=s;bx=u;by=v}
 }
 return {x:bx,y:by,error:best};
}
function trackOne(prev,now,p,opts={}){
 const w=opts.w||W,h=opts.h||H;
 if(!pixels(prev,w,h,p.x,p.y))return null;
 const aSmall=opts.prevSmall||down(prev,w,h),bSmall=opts.nowSmall||down(now,w,h);
 const x=p.x>>1,y=p.y>>1;
 const rough=nearest(aSmall,bSmall,w>>1,h>>1,x,y,x,y,opts.coarseSearch??6);
 if(!Number.isFinite(rough.error)||rough.error>1000)return null;
 // A second full-resolution local search handles subpixel/odd-pixel moves
 // that become ambiguous after 2x box downsampling.
 const coarse=nearest(prev,now,w,h,p.x,p.y,2*rough.x,2*rough.y,opts.fineSearch??3);
 const local=nearest(prev,now,w,h,p.x,p.y,p.x,p.y,opts.localSearch??6);
 const fwd=local.error<=coarse.error?local:coarse;
 if(!Number.isFinite(fwd.error)||fwd.error>(opts.maxError??360))return null;
 const back=nearest(now,prev,w,h,fwd.x,fwd.y,p.x,p.y,opts.backwardSearch??3);
 if(!Number.isFinite(back.error)||Math.hypot(back.x-p.x,back.y-p.y)>1.5)return null;
 return {x:fwd.x,y:fwd.y,error:fwd.error,fbError:Math.hypot(back.x-p.x,back.y-p.y)};
}
function corners(gray,w=W,h=H,max=125){
 const arr=[];
 for(let y=9;y<h-9;y+=3)for(let x=9;x<w-9;x+=3){
  const i=y*w+x;
  const dx=Math.abs(gray[i+1]-gray[i-1])+Math.abs(gray[i+w+1]-gray[i+w-1]);
  const dy=Math.abs(gray[i+w]-gray[i-w])+Math.abs(gray[i+w+1]-gray[i-w+1]);
  const c=Math.min(dx,dy);
  if(c>48)arr.push({x,y,score:c});
 }
 arr.sort((a,b)=>b.score-a.score);
 const out=[];
 for(const p of arr){
  if(out.some(q=>(q.x-p.x)**2+(q.y-p.y)**2<100))continue;
  out.push(p);if(out.length>=max)break;
 }return out;
}
class Tracker{
 constructor({maxTracks=100,w=W,h=H}={}){
  this.w=w;this.h=h;this.maxTracks=maxTracks;this.frame=null;this.tracks=[];
  this.nextId=1;this.frameCount=0;this.trackTotal=0;this.last=null;
 }
 update(gray,t){
  if(!(gray instanceof Uint8Array)||gray.length!==this.w*this.h||!Number.isFinite(t))
   throw Error('invalid grayscale frame or timestamp');
  if(this.last&&t<=this.last.t)throw Error('non-monotonic frame timestamp');
  const prev=this.frame,old=this.tracks,initial=old.length;
  let current=[],failed=0,errs=[];
  if(prev){
   const opts={w:this.w,h:this.h,prevSmall:down(prev,this.w,this.h),nowSmall:down(gray,this.w,this.h)};
   for(const p of old){
    const q=trackOne(prev,gray,p,opts);
    if(q&&current.every(k=>(k.x-q.x)**2+(k.y-q.y)**2>=36)){
     current.push({...p,x:q.x,y:q.y,age:p.age+1,dx:q.x-p.x,dy:q.y-p.y});
     errs.push(q.error);
    }else failed++;
   }
  }
  const retained=current.length,newCountBefore=current.length;
  if(current.length<this.maxTracks*.75){
   const pts=corners(gray,this.w,this.h,Math.max(80,this.maxTracks*2));
   for(const p of pts){
    if(current.length>=this.maxTracks)break;
    if(current.some(q=>(q.x-p.x)**2+(q.y-p.y)**2<100))continue;
    current.push({id:this.nextId++,x:p.x,y:p.y,age:1,dx:0,dy:0});this.trackTotal++;
   }
  }
  this.tracks=current;this.frame=gray.slice();this.frameCount++;
  const result={timestamp:t,frame:this.frameCount,total:current.length,retained,initial,added:current.length-newCountBefore,
    lost:failed,retention:initial?retained/initial:null,
    avgAge:current.length?current.reduce((s,p)=>s+p.age,0)/current.length:0,
    longLived:current.filter(p=>p.age>=5).length,
    medianError:errs.length?errs.sort((a,b)=>a-b)[errs.length>>1]:null,
    matches:current.filter(p=>p.age>1).map(p=>({id:p.id,x:p.x,y:p.y,dx:p.dx,dy:p.dy,age:p.age})),
    metricPoseKnown:false,slamInitialized:false};
  this.last=result;return result;
 }
 reset(){this.frame=null;this.tracks=[];this.nextId=1;this.frameCount=0;this.trackTotal=0;this.last=null}
}
return {Tracker,corners,trackOne,down,score};
});