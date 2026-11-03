/* V3.27 A4 V2 dot-grid observations. Fiducial corners anchor the mapping.
 * Interior printed dots are separate image evidence; no 3D camera pose.
 */
(function(root,factory){const api=factory(root.VINBoard||(typeof require==='function'?require('./vin-a4-board.js'):null));if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIVioGrid=api;})(typeof globalThis!=='undefined'?globalThis:this,function(B){
'use strict';
const X=[.050,.085,.120,.155],Y=[.050,.090,.130,.170,.210,.250];
const WORLD=Y.flatMap((y,row)=>X.map((x,col)=>({row,col,x,y})));
const grayAt=(g,w,h,x,y)=>x<0||y<0||x>=w||y>=h?255:g[y*w+x];
function detect(gray,w,h,corners){
 if(!(gray instanceof Uint8Array)||gray.length!==w*h||!B.quadValid(corners,w,h))
  return {ok:false,reason:'invalid_input',points:[]};
 const H=B.homography(B.WORLD,corners);
 if(!H)return {ok:false,reason:'invalid_homography',points:[]};
 const spacing=Math.min(...[
  Math.hypot(...[0,1].map((_,i)=>0))||Infinity, // replaced below
  ...WORLD.slice(1).filter(p=>p.row===0).map(p=>{const a=B.project(H,[p.x,p.y]);const b=B.project(H,[p.x-.035,p.y]);return a&&b?Math.hypot(a[0]-b[0],a[1]-b[1]):Infinity;})
 ]);
 const step=Number.isFinite(spacing)?spacing:20;
 const search=Math.max(3,Math.min(8,Math.floor(step*.20)));
 const radius=Math.max(2,Math.min(5,Math.round(step*.085)));
 const ring=Math.max(radius+3,Math.min(12,Math.round(step*.23)));
 const points=[];
 for(const p of WORLD){
  const q=B.project(H,[p.x,p.y]);if(!q)continue;
  const px=Math.round(q[0]),py=Math.round(q[1]);
  if(px-search-ring<0||py-search-ring<0||px+search+ring>=w||py+search+ring>=h)continue;
  let best=null,bestValue=Infinity;
  for(let dy=-search;dy<=search;dy++)for(let dx=-search;dx<=search;dx++){
   const x=px+dx,y=py+dy;
   const value=(grayAt(gray,w,h,x,y)+grayAt(gray,w,h,x+1,y)+grayAt(gray,w,h,x,y+1)+grayAt(gray,w,h,x-1,y)+grayAt(gray,w,h,x,y-1))/5+
     .16*(dx*dx+dy*dy);
   if(value<bestValue){bestValue=value;best=[x,y];}
  }
  if(!best)continue;
  const [bx,by]=best;
  const center=grayAt(gray,w,h,bx,by);
  const directions=[[1,0],[-1,0],[0,1],[0,-1],[.707,.707],[-.707,.707],[.707,-.707],[-.707,-.707]];
  const surround=directions.map(([dx,dy])=>grayAt(gray,w,h,Math.round(bx+ring*dx),Math.round(by+ring*dy)));
  const white=surround.filter(v=>v>150).length;
  const mean=surround.reduce((s,v)=>s+v,0)/surround.length;
  if(center>105||mean-center<90||white<6)continue;
  let sum=0,cx=0,cy=0,n=0;
  for(let y=by-radius;y<=by+radius;y++)for(let x=bx-radius;x<=bx+radius;x++){
   const val=grayAt(gray,w,h,x,y),weight=Math.max(0,155-val);
   if(weight>0){sum+=weight;cx+=x*weight;cy+=y*weight;n++;}
  }
  if(!sum||n<3)continue;
  const x=cx/sum,y=cy/sum;
  const err=Math.hypot(x-q[0],y-q[1]);
  if(err>search+1)continue;
  points.push({row:p.row,col:p.col,world:[p.x,p.y],image:[x,y],predicted:q,residualPx:err});
 }
 const errors=points.map(p=>p.residualPx);
 const rms=errors.length?Math.sqrt(errors.reduce((s,v)=>s+v*v,0)/errors.length):null;
 return {ok:points.length>=16,reason:points.length>=16?'grid_detected':'insufficient_grid_dots',
  board:'pai-vin-a4-grid-v2',detected:points.length,expected:WORLD.length,
  points,rmsFourCornerHomographyPx:rms,
  independentInteriorGeometry:true,independentCameraCalibration:false,
  undistortionEnabled:false};
}
return {WORLD,detect};
});