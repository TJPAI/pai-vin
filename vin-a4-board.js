/* PAI-VIN A4 reference geometry, pure JS.
 * Calibrates a plane ONLY from four user-selected fiducials (160 x 247 mm).
 * Does NOT estimate camera intrinsics, camera/IMU extrinsics, or SLAM scale. */
(function(root,factory){
 const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;
 else root.VINBoard=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const WORLD=[[0,0],[.16,0],[.16,.247],[0,.247]];
 function area(pts){return pts.reduce((s,p,i)=>{const q=pts[(i+1)%pts.length];return s+p[0]*q[1]-p[1]*q[0]},0)/2}
 function quadValid(p,w,h){
  if(!Array.isArray(p)||p.length!==4||!p.every(v=>Array.isArray(v)&&v.length===2&&v.every(Number.isFinite)))return false;
  if(!p.every(([x,y])=>x>=0&&x<w&&y>=0&&y<h))return false;
  if(Math.abs(area(p))<w*h*.005)return false;
  const cross=(a,b,c)=>(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]);
  const signs=p.map((x,i)=>Math.sign(cross(x,p[(i+1)%4],p[(i+2)%4])));
  return signs.every(s=>s===signs[0]&&s!==0);
 }
 function solve(A,b){
  const n=b.length,m=A.map((r,i)=>[...r,b[i]]);
  for(let j=0;j<n;j++){
   let pivot=j;for(let i=j+1;i<n;i++)if(Math.abs(m[i][j])>Math.abs(m[pivot][j]))pivot=i;
   if(Math.abs(m[pivot][j])<1e-12)return null;
   [m[j],m[pivot]]=[m[pivot],m[j]];
   const d=m[j][j];for(let k=j;k<=n;k++)m[j][k]/=d;
   for(let i=0;i<n;i++)if(i!==j){const d=m[i][j];for(let k=j;k<=n;k++)m[i][k]-=d*m[j][k]}
  }
  return m.map(r=>r[n]);
 }
 function homography(src,dst){
  if(src.length!==4||dst.length!==4)return null;
  const A=[],b=[];
  for(let i=0;i<4;i++){
   const [x,y]=src[i],[u,v]=dst[i];
   A.push([x,y,1,0,0,0,-u*x,-u*y]);b.push(u);
   A.push([0,0,0,x,y,1,-v*x,-v*y]);b.push(v);
  }
  const p=solve(A,b);return p&&[p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],1];
 }
 function project(H,p){
  const [x,y]=p,den=H[6]*x+H[7]*y+1;
  if(Math.abs(den)<1e-10)return null;
  return [(H[0]*x+H[1]*y+H[2])/den,(H[3]*x+H[4]*y+H[5])/den];
 }
 function calibrate(imageCorners,width,height){
  if(!quadValid(imageCorners,width,height))return {ok:false,reason:'invalid_corners'};
  const H=homography(imageCorners,WORLD),inverse=homography(WORLD,imageCorners);
  if(!H||!inverse)return {ok:false,reason:'singular_homography'};
  const err=Math.sqrt(WORLD.reduce((s,p,i)=>{
   const xy=project(inverse,p);if(!xy)return s+1e12;
   return s+(xy[0]-imageCorners[i][0])**2+(xy[1]-imageCorners[i][1])**2;
  },0)/4);
  const edge=(a,b)=>Math.hypot(imageCorners[a][0]-imageCorners[b][0],imageCorners[a][1]-imageCorners[b][1]);
  const shortest=Math.min(edge(0,1),edge(1,2),edge(2,3),edge(3,0));
  if(shortest<25)return {ok:false,reason:'board_too_small'};
  if(err>1)return {ok:false,reason:'reprojection_error'};
  return {ok:true,schema:'pai-vin-a4-plane-v1',boardWidthM:.16,boardHeightM:.247,
   imageWidth:width,imageHeight:height,corners:imageCorners.map(p=>p.slice()),
   planeImageToMeters:H,planeMetersToImage:inverse,reprojectionErrorPx:err,
   metricPlaneOnly:true,cameraIntrinsicsKnown:false,globalSLAMScaleKnown:false,
   warning:'Plane homography is not camera calibration or verified metric SLAM'};
 }
 return {WORLD,area,quadValid,homography,project,calibrate};
});