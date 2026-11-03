/* Experimental browser-local 2D visual odometry sketch.
 * Inputs are actual matched features + two-view R,t. Output is arbitrary
 * step-normalized CAMERA MOTION directions, not meters or floor coordinates.
 * Assumes a roughly level rear camera; yaw and translation are unreliable
 * under pure rotation, planar scene, poor calibration and low parallax. */
(function(root,factory){
 const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.VINPath=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const finite=v=>Number.isFinite(v);
 function median(v){const a=v.slice().sort((x,y)=>x-y);return a.length?a[a.length>>1]:0}
 // Photometric stationary gate: exposure-robust local image difference.
 // A stationary camera may receive auto-exposure shifts; remove median luminance
 // offset before measuring textured pixel change.
 function frameChange(a,b){
  if(!a||!b||a.length!==b.length||a.length<1024)return {ok:false,reason:'missing_frames'};
  const increments=[],diff=[];
  for(let i=0;i<a.length;i+=8)increments.push(b[i]-a[i]);
  const offset=median(increments);
  for(let i=0;i<a.length;i+=8)diff.push(Math.abs((b[i]-a[i])-offset));
  const sorted=diff.sort((x,y)=>x-y),p75=sorted[Math.floor(sorted.length*.75)],p90=sorted[Math.floor(sorted.length*.9)];
  const frac=sorted.filter(x=>x>=16).length/sorted.length;
  const changed=p75>=13&&p90>=27&&frac>=.18;
  return {ok:changed,reason:changed?'photometric_change':'stationary_frame',p75,p90,frac,exposureOffset:offset};
 }
 // Estimate translation evidence from spatially coherent feature movements.
 // This intentionally rejects ambiguous pure rotation/zoom and drifting features.
 function flowEvidence(a,b,pairs){
  const flows=pairs.map(m=>({dx:b[m.j].x-a[m.i].x,dy:b[m.j].y-a[m.i].y,x:a[m.i].x,y:a[m.i].y}));
  const dx=median(flows.map(f=>f.dx)),dy=median(flows.map(f=>f.dy));
  const medianFlow=Math.hypot(dx,dy);
  const agreement=flows.filter(f=>Math.hypot(f.dx-dx,f.dy-dy)<3.8).length/Math.max(1,flows.length);
  const grid=new Set(flows.filter(f=>Math.hypot(f.dx-dx,f.dy-dy)<3.8).map(f=>(f.x<128?0:1)+(f.y<96?0:2)));
  return {medianFlow,agreement,spread:grid.size};
 }
  function parseStep(a,b,geometry,vo,options={}){
  const photo=options.frameChange;
  if(!photo?.ok)return {ok:false,reason:photo?.reason||'no_frame_motion_evidence'};
  const pairs=geometry.correspond(a,b);
  if(pairs.length<16)return {ok:false,reason:'few_correspondences',pairs:pairs.length};
  const evidence=flowEvidence(a,b,pairs);
  // Real camera translation produces parallax across different scene depths.
  // Requiring a common 2D flow vector incorrectly rejects it and turns.
  const displacement=median(pairs.map(m=>Math.hypot(a[m.i].x-b[m.j].x,a[m.i].y-b[m.j].y)));
  if(displacement<2.5)return {ok:false,reason:'insufficient_baseline',pairs:pairs.length,displacement};
  const coverage=new Set(pairs.map(m=>(a[m.i].x<128?0:1)+(a[m.i].y<96?0:2)));
  if(coverage.size<3)return {ok:false,reason:'poor_image_coverage'};
  const p=vo.estimate(a,b,{pairs,focal:220,iterations:64,thresholdPx:2.5});
  if(!p.ok||p.inliers<14||p.ratio<.45||p.cheirality<8||!p.R||!p.t)
   return {ok:false,reason:'epipolar_unreliable',pairs:pairs.length,inliers:p.inliers||0};
  const focal=220;
  const rotationResidual=[];
  for(const m of pairs){
   const u=a[m.i],v=b[m.j],ray=[(u.x-128)/focal,(u.y-96)/focal,1];
   const rot=p.R.map(row=>row[0]*ray[0]+row[1]*ray[1]+row[2]*ray[2]);
   if(rot[2]<.1)continue;
   rotationResidual.push(Math.hypot(128+focal*rot[0]/rot[2]-v.x,96+focal*rot[1]/rot[2]-v.y));
  }
  const parallaxPx=median(rotationResidual);
  if(parallaxPx<2.2)return {ok:false,reason:'pure_rotation_or_no_parallax',parallaxPx};
  // Camera center displacement in previous camera coordinates: -R^T * t
  const R=p.R,t=p.t;
  const dx=-(R[0][0]*t[0]+R[1][0]*t[1]+R[2][0]*t[2]);
  const dz=-(R[0][2]*t[0]+R[1][2]*t[1]+R[2][2]*t[2]);
  const horizontal=Math.hypot(dx,dz);
  if(horizontal<.35)return {ok:false,reason:'vertical_or_unobservable_motion'};
  const yaw=Math.atan2(R[0][2],R[2][2]);
  if(![dx,dz,yaw].every(finite)||Math.abs(yaw)>1.3)return {ok:false,reason:'unstable_rotation'};
  return {ok:true,dx:dx/horizontal,dz:dz/horizontal,yaw,displacement,inliers:p.inliers,pairs:p.matches,
   units:'arbitrary_step',metric:false,evidence,parallaxPx};
 }
 class Sketch{
  constructor(saved){
   this.nodes=[{x:0,y:0,heading:0,segment:0}];this.rejected=0;
   if(saved){
    if(saved.schema!=='pai-vin-vo-sketch-v1'||saved.metric!==false||
      !Array.isArray(saved.nodes)||saved.nodes.length<1||saved.nodes.length>200||
      !saved.nodes.every(n=>[n.x,n.y,n.heading,n.segment].every(finite)))throw Error('bad VO sketch');
    if(saved.gateVersion!==3)throw Error('legacy untrusted VO path');
    this.nodes=saved.nodes.map(n=>({...n}));this.rejected=Number.isInteger(saved.rejected)?saved.rejected:0;
   }
  }
  append(step){
   if(!step?.ok||![step.dx,step.dz,step.yaw].every(finite)){
    this.rejected++;return false;
   }
   const last=this.nodes[this.nodes.length-1];
   if(this.nodes.length>=200)return false;
   // Rotate local camera-frame direction into the cumulative approximate orientation.
   const c=Math.cos(last.heading),s=Math.sin(last.heading);
   const x=last.x+c*step.dx-s*step.dz;
   const y=last.y+s*step.dx+c*step.dz;
   this.nodes.push({x,y,heading:last.heading+step.yaw,segment:last.segment+1});
   return true;
  }
  serialize(){return {schema:'pai-vin-vo-sketch-v1',units:'arbitrary_step',metric:false,
    physicalDistanceKnown:false,headingCalibrated:false,gateVersion:3,rejected:this.rejected,nodes:this.nodes};}
 }
 function render(canvas,sketch){
  const c=canvas.getContext('2d'),w=canvas.width,h=canvas.height,n=sketch?.nodes||[];
  c.fillStyle='#102139';c.fillRect(0,0,w,h);
  c.fillStyle='#c7dced';c.font='14px -apple-system,sans-serif';
  c.fillText('视觉运动草图：步长归一化 / 非米制',12,22);
  if(n.length<2){c.fillText('请持手机缓慢平移，等待可信双视图估计',12,56);return}
  const x=n.map(p=>p.x),y=n.map(p=>p.y),minX=Math.min(...x),maxX=Math.max(...x),minY=Math.min(...y),maxY=Math.max(...y);
  const sc=Math.min((w-44)/Math.max(2,maxX-minX),(h-80)/Math.max(2,maxY-minY));
  const project=p=>[w/2+(p.x-(minX+maxX)/2)*sc,h/2+(p.y-(minY+maxY)/2)*sc];
  c.strokeStyle='#60d2ac';c.lineWidth=2.5;c.beginPath();
  n.forEach((p,i)=>{const [xx,yy]=project(p);if(!i)c.moveTo(xx,yy);else c.lineTo(xx,yy)});c.stroke();
  const [xx,yy]=project(n[n.length-1]);c.fillStyle='#ffce6a';c.beginPath();c.arc(xx,yy,5,0,Math.PI*2);c.fill();
  c.fillStyle='#bad3e8';c.font='12px -apple-system,sans-serif';
  c.fillText('累计相对方向，仅算法诊断；不能用于导航',12,h-13);
 }
 return {parseStep,Sketch,render,frameChange,flowEvidence};
});