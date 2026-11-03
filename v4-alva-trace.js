/* V4.0.4 visual trajectory: each native SLAM continuity segment has its
 * own arbitrary-scale origin and canvas panel. NO metric or map alignment.
 * AlvaAR Twc column-major: position indices 12..14, camera +Z heading 8,10.
 */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PAIV4Trace=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const palette=['#58e5b2','#ffbf69','#8dbbff','#f88dca','#b7a0ff','#76dbe8'];
function numericPose(p){return p&&p.length===16&&Array.from(p).every(Number.isFinite)&&Math.abs(p[15]-1)<.02;}
class Trace{
 constructor(){this.reset();}
 reset(){this.groups=[];this.active=null;this.lastStatus=null;this.totalPoints=0;this.lastState='未初始化';}
 ingest(status,pose,t,frame){
  if(status!==1||!numericPose(pose)){
   this.active=null;this.lastStatus=status;
   this.lastState=status===2?'引擎重置':status===3?'初始化/失跟':'位姿不可用';
   return {valid:false,segmentId:null};
  }
  if(!this.active){
   const g={id:this.groups.length?this.groups[this.groups.length-1].id+1:1,color:palette[(this.groups.length)%palette.length],
    startFrame:frame,startMs:t,endFrame:frame,endMs:t,origin:[pose[12],pose[14]],points:[]};
   this.groups.push(g);this.active=g;
   // Keep only 24 distinct segments on device; no join or geometry transform.
   if(this.groups.length>24)this.groups.shift();
  }
  const g=this.active;
  const x=pose[12]-g.origin[0],z=pose[14]-g.origin[1];
  // Camera +Z axis projected onto horizontal XZ plane; not a compass heading.
  const dx=pose[8],dz=pose[10],norm=Math.hypot(dx,dz);
  const point={x,z,dx:norm>1e-8?dx/norm:0,dz:norm>1e-8?dz/norm:1,t,frame};
  if(!g.points.length||g.points[g.points.length-1].frame!==frame){
   g.points.push(point);this.totalPoints++;
  }
  if(g.points.length>1500)g.points.shift();
  g.endFrame=frame;g.endMs=t;this.lastStatus=status;this.lastState='视觉位姿可用';
  return {valid:true,segmentId:g.id,relativeXZ:[x,z],headingXZ:[point.dx,point.dz]};
 }
 snapshot(){
  return {units:'arbitrary',metricScaleVerified:false,coordinateFramesAligned:false,
   definition:'Each SLAM segment is recentered to its own origin; x,z from Twc; no reset-frame alignment',
   activeSegmentId:this.active?.id??null,totalPoints:this.totalPoints,
   groups:this.groups.map(g=>({id:g.id,startFrame:g.startFrame,endFrame:g.endFrame,
    startMs:g.startMs,endMs:g.endMs,originArbitraryXZ:[...g.origin],
    points:g.points.map(p=>({frame:p.frame,t:p.t,x:p.x,z:p.z,headingXZ:[p.dx,p.dz]}))}))};
 }
}
function render(ctx,trace,w,h){
 if(!ctx||!trace)return;
 ctx.clearRect(0,0,w,h);ctx.fillStyle='#0a1a2b';ctx.fillRect(0,0,w,h);
 const current=trace.active,prior=trace.groups.filter(x=>x!==current).slice(-2).reverse();
 const mainH=Math.floor(h*.73);
 drawPanel(ctx,current,6,6,w-12,mainH-10,'当前连续段（独立坐标）',true);
 const gap=6,subY=mainH+4,subH=h-subY-6,subW=(w-18)/2;
 drawPanel(ctx,prior[0],6,subY,subW,subH,'历史段 1',false);
 drawPanel(ctx,prior[1],12+subW,subY,subW,subH,'历史段 2',false);
}
function drawPanel(ctx,seg,x,y,w,h,label,main){
 ctx.fillStyle='#13263c';ctx.fillRect(x,y,w,h);
 ctx.strokeStyle='#334960';ctx.lineWidth=1;ctx.strokeRect(x+.5,y+.5,w-1,h-1);
 ctx.fillStyle='#b9cde0';ctx.font=(main?'13':'11')+'px sans-serif';ctx.fillText(label+(seg?' · #'+seg.id:''),x+10,y+19);
 if(!seg||!seg.points.length){ctx.fillStyle='#819bb5';ctx.fillText('无连续位姿',x+10,y+40);return;}
 const pts=seg.points;
 let minX=0,maxX=0,minZ=0,maxZ=0;
 for(const p of pts){minX=Math.min(minX,p.x);maxX=Math.max(maxX,p.x);minZ=Math.min(minZ,p.z);maxZ=Math.max(maxZ,p.z);}
 const usableW=Math.max(1,w-32),usableH=Math.max(1,h-61),rangeX=Math.max(1e-6,maxX-minX),rangeZ=Math.max(1e-6,maxZ-minZ);
 const scale=Math.min(usableW/Math.max(.4,rangeX),usableH/Math.max(.4,rangeZ),150);
 const cx=x+w/2,cy=y+37+(h-51)/2,midX=(minX+maxX)/2,midZ=(minZ+maxZ)/2;
 const at=p=>[cx+(p.x-midX)*scale,cy-(p.z-midZ)*scale];
 ctx.save();ctx.beginPath();ctx.rect(x+6,y+28,w-12,h-34);ctx.clip();
 ctx.strokeStyle='#263f56';ctx.lineWidth=1;
 for(let k=-3;k<=3;k++){let gx=cx+k*30,gy=cy+k*30;ctx.beginPath();ctx.moveTo(gx,y+28);ctx.lineTo(gx,y+h-6);ctx.stroke();ctx.beginPath();ctx.moveTo(x+6,gy);ctx.lineTo(x+w-6,gy);ctx.stroke();}
 // A single path is constructed ONLY from points in this one segment.
 ctx.strokeStyle=seg.color;ctx.lineWidth=main?2.8:2;ctx.beginPath();
 pts.forEach((p,i)=>{const q=at(p);if(i===0)ctx.moveTo(...q);else ctx.lineTo(...q);});ctx.stroke();
 const first=at(pts[0]),last=at(pts[pts.length-1]);
 ctx.fillStyle='#ffffff';ctx.beginPath();ctx.arc(...first,main?4:3,0,Math.PI*2);ctx.fill();
 ctx.fillStyle=seg.color;ctx.beginPath();ctx.arc(...last,main?6:4,0,Math.PI*2);ctx.fill();
 if(main){const p=pts[pts.length-1],arrow=16;
  ctx.strokeStyle='#ffffff';ctx.lineWidth=2.3;ctx.beginPath();ctx.moveTo(...last);ctx.lineTo(last[0]+p.dx*arrow,last[1]-p.dz*arrow);ctx.stroke();
 }
 ctx.restore();
 ctx.fillStyle='#b9cde0';ctx.font='11px sans-serif';ctx.fillText(pts.length+'帧 · 相对单位（非米）',x+10,y+h-10);
}
return {Trace,render};
});