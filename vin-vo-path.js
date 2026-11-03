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
 function parseStep(a,b,geometry,vo){
  const pairs=geometry.correspond(a,b);
  if(pairs.length<16)return {ok:false,reason:'few_correspondences',pairs:pairs.length};
  const displacement=median(pairs.map(m=>Math.hypot(a[m.i].x-b[m.j].x,a[m.i].y-b[m.j].y)));
  // Reject nearly fixed image, especially pure stationary camera.
  if(displacement<2.8)return {ok:false,reason:'low_image_motion',pairs:pairs.length,displacement};
  const p=vo.estimate(a,b,{pairs,focal:220,iterations:46,thresholdPx:2.0});
  if(!p.ok||p.inliers<14||p.ratio<.42||p.cheirality<8||!p.R||!p.t)
   return {ok:false,reason:'epipolar_unreliable',pairs:pairs.length,inliers:p.inliers||0};
  // Camera center displacement in previous camera coordinates: -R^T * t
  const R=p.R,t=p.t;
  const dx=-(R[0][0]*t[0]+R[1][0]*t[1]+R[2][0]*t[2]);
  const dz=-(R[0][2]*t[0]+R[1][2]*t[1]+R[2][2]*t[2]);
  const horizontal=Math.hypot(dx,dz);
  if(horizontal<.35)return {ok:false,reason:'vertical_or_unobservable_motion'};
  const yaw=Math.atan2(R[0][2],R[2][2]);
  if(![dx,dz,yaw].every(finite)||Math.abs(yaw)>1.3)return {ok:false,reason:'unstable_rotation'};
  return {ok:true,dx:dx/horizontal,dz:dz/horizontal,yaw,displacement,inliers:p.inliers,pairs:p.matches,
   units:'arbitrary_step',metric:false};
 }
 class Sketch{
  constructor(saved){
   this.nodes=[{x:0,y:0,heading:0,segment:0}];this.rejected=0;
   if(saved){
    if(saved.schema!=='pai-vin-vo-sketch-v1'||saved.metric!==false||
      !Array.isArray(saved.nodes)||saved.nodes.length<1||saved.nodes.length>200||
      !saved.nodes.every(n=>[n.x,n.y,n.heading,n.segment].every(finite)))throw Error('bad VO sketch');
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
    physicalDistanceKnown:false,headingCalibrated:false,rejected:this.rejected,nodes:this.nodes};}
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
 return {parseStep,Sketch,render};
});