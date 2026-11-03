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
// One-pass brightness-offset-corrected SSD, mathematically equivalent to
// the original two-pass search. Keep the integer offset for exact behaviour.
function score(a,b,w,h,x,y,u,v,r=2){
 if(x<r||y<r||u<r||v<r||x>=w-r||u>=w-r||y>=h-r||v>=h-r)return Infinity;
 let sum=0,sum2=0;
 for(let j=-r;j<=r;j++)for(let i=-r;i<=r;i++){
  const d=a[(y+j)*w+x+i]-b[(v+j)*w+u+i];
  sum+=d;sum2+=d*d;
 }
 const n=(2*r+1)*(2*r+1),offset=Math.round(-sum/n);
 return (sum2+2*offset*sum+n*offset*offset)/n;
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
// Image-gradient Lucas-Kanade subpixel refinement around a validated
// integer-pixel coarse-to-fine correspondence. No pose/scale is inferred.
function bilinear(g,w,h,x,y){
 if(x<0||y<0||x>=w-1||y>=h-1)return NaN;
 const ix=Math.floor(x),iy=Math.floor(y),dx=x-ix,dy=y-iy,k=iy*w+ix;
 return g[k]*(1-dx)*(1-dy)+g[k+1]*dx*(1-dy)+g[k+w]*(1-dx)*dy+g[k+w+1]*dx*dy;
}
function refineLK(prev,now,w,h,px,py,seedX,seedY){
 if(Math.min(px,py,seedX,seedY)<5||px>=w-5||py>=h-5||seedX>=w-5||seedY>=h-5)return null;
 const radius=2,patch=[];
 for(let dy=-radius;dy<=radius;dy++)for(let dx=-radius;dx<=radius;dx++)
  patch.push({dx,dy,value:bilinear(prev,w,h,px+dx,py+dy)});
 let x=seedX,y=seedY;
 for(let iter=0;iter<4;iter++){
  if(x<5||y<5||x>=w-5||y>=h-5)return null;
  let hxx=0,hxy=0,hyy=0,bx=0,by=0,mean=0;
  // Compensate constant brightness shifts for consistency with SSD search.
  for(const p of patch)mean+=bilinear(now,w,h,x+p.dx,y+p.dy)-p.value;
  mean/=patch.length;
  for(const p of patch){
   const tx=x+p.dx,ty=y+p.dy;
   const gx=(bilinear(now,w,h,tx+1,ty)-bilinear(now,w,h,tx-1,ty))*.5;
   const gy=(bilinear(now,w,h,tx,ty+1)-bilinear(now,w,h,tx,ty-1))*.5;
   const residual=p.value+mean-bilinear(now,w,h,tx,ty);
   hxx+=gx*gx;hxy+=gx*gy;hyy+=gy*gy;
   bx+=gx*residual;by+=gy*residual;
  }
  const det=hxx*hyy-hxy*hxy;
  if(det<100||!Number.isFinite(det))return null;
  const dx=(hyy*bx-hxy*by)/det,dy=(hxx*by-hxy*bx)/det;
  if(!Number.isFinite(dx)||!Number.isFinite(dy)||Math.abs(dx)>1.5||Math.abs(dy)>1.5)return null;
  x+=dx;y+=dy;
  if(dx*dx+dy*dy<.0004)break;
 }
 if(Math.hypot(x-seedX,y-seedY)>.85)return null;
 return {x,y};
}
function trackOne(prev,now,p,opts={}){
 const w=opts.w||W,h=opts.h||H;
 if(!pixels(prev,w,h,Math.round(p.x),Math.round(p.y)))return null;
 const aSmall=opts.prevSmall||down(prev,w,h),bSmall=opts.nowSmall||down(now,w,h);
 const ax=Math.round(p.x),ay=Math.round(p.y),x=ax>>1,y=ay>>1;
 // Three-level tracking: accommodate larger genuine inter-frame motion
 // while retaining full-resolution forward-backward validation.
 const aQuarter=opts.prevQuarter||down(aSmall,w>>1,h>>1);
 const bQuarter=opts.nowQuarter||down(bSmall,w>>1,h>>1);
 const q=nearest(aQuarter,bQuarter,w>>2,h>>2,ax>>2,ay>>2,ax>>2,ay>>2,opts.quarterSearch??5);
 const halfFromQuarter=Number.isFinite(q.error)?
  nearest(aSmall,bSmall,w>>1,h>>1,x,y,2*q.x,2*q.y,opts.halfSearch??3):
  {error:Infinity};
 const halfDirect=nearest(aSmall,bSmall,w>>1,h>>1,x,y,x,y,opts.coarseSearch??6);
 const rough=halfFromQuarter.error<=halfDirect.error?halfFromQuarter:halfDirect;
 const coarse=nearest(prev,now,w,h,ax,ay,2*rough.x,2*rough.y,opts.fineSearch??3);
 const local=nearest(prev,now,w,h,ax,ay,ax,ay,opts.localSearch??6);
 const fwd=local.error<=coarse.error?local:coarse;
 if(!Number.isFinite(fwd.error)||fwd.error>(opts.maxError??360))return null;
 const back=nearest(now,prev,w,h,fwd.x,fwd.y,ax,ay,opts.backwardSearch??5);
 if(!Number.isFinite(back.error)||Math.hypot(back.x-ax,back.y-ay)>1.5)return null;
 const subpixel=opts.subpixel===false?null:refineLK(prev,now,w,h,p.x,p.y,fwd.x,fwd.y);
 return {x:subpixel?.x??fwd.x,y:subpixel?.y??fwd.y,error:fwd.error,fbError:Math.hypot(back.x-ax,back.y-ay)};
}
// Shi-Tomasi minimum eigenvalue of the 3x3 structure tensor:
// unlike 1D gradient scores, it rejects single-direction edges.
function cornerStrength(gray,w,h,x,y){
 if(x<3||y<3||x>=w-3||y>=h-3)return 0;
 let xx=0,xy=0,yy=0;
 for(let j=-1;j<=1;j++)for(let i=-1;i<=1;i++){
  const k=(y+j)*w+x+i;
  const gx=(gray[k+1]-gray[k-1])*.5,gy=(gray[k+w]-gray[k-w])*.5;
  xx+=gx*gx;xy+=gx*gy;yy+=gy*gy;
 }
 xx/=9;xy/=9;yy/=9;
 return (xx+yy-Math.sqrt((xx-yy)*(xx-yy)+4*xy*xy))*.5;
}
function corners(gray,w=W,h=H,max=125){
 const arr=[];
 for(let y=9;y<h-9;y+=3)for(let x=9;x<w-9;x+=3){
  const i=y*w+x;
  const dx=Math.abs(gray[i+1]-gray[i-1])+Math.abs(gray[i+w+1]-gray[i-w+1]);
  const dy=Math.abs(gray[i+w]-gray[i-w])+Math.abs(gray[i+w+1]-gray[i-w+1]);
  const c=Math.min(dx,dy);
  if(c>48)arr.push({x,y,score:c});
 }
 arr.sort((a,b)=>b.score-a.score);
 // Preserve coverage across the image instead of allowing a textured wall
 // or poster to occupy nearly all new feature slots.
 const cols=8,rows=6,cells=Array.from({length:cols*rows},()=>[]);
 for(const p of arr){
  const cx=Math.min(cols-1,Math.floor(p.x*cols/w));
  const cy=Math.min(rows-1,Math.floor(p.y*rows/h));
  cells[cy*cols+cx].push(p);
 }
 const out=[];
 const insert=p=>{
  if(out.some(q=>(q.x-p.x)**2+(q.y-p.y)**2<100))return false;
  out.push(p);return true;
 };
 for(const cell of cells){
  for(const p of cell)if(insert(p))break;
  if(out.length>=max)return out;
 }
 for(const p of arr){
  if(out.length>=max)break;
  insert(p);
 }
 return out;
}
class Tracker{
 constructor({maxTracks=100,w=W,h=H}={}){
  this.w=w;this.h=h;this.maxTracks=maxTracks;this.frame=null;this.tracks=[];
  this.nextId=1;this.frameCount=0;this.trackTotal=0;this.last=null;
 }
 update(gray,t){
  if(!(gray instanceof Uint8Array)||gray.length!==this.w*this.h||!Number.isFinite(t))
   throw Error('invalid grayscale frame or timestamp');
  if(this.last&&t<=this.last.timestamp)throw Error('non-monotonic frame timestamp');
  const prev=this.frame,old=this.tracks,initial=old.length;
  let current=[],failed=0,errs=[];
  if(prev){
   const prevSmall=down(prev,this.w,this.h),nowSmall=down(gray,this.w,this.h);
   const opts={w:this.w,h:this.h,prevSmall,nowSmall,
    prevQuarter:down(prevSmall,this.w>>1,this.h>>1),nowQuarter:down(nowSmall,this.w>>1,this.h>>1)};
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
return {Tracker,corners,cornerStrength,trackOne,down,score,refineLK,bilinear};
});