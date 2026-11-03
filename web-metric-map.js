/* PAI-VIN map contract + validated 2D path rendering.
 * Draw ONLY meter-scale positions that passed explicit validation.
 * Not an estimator: camera pose must originate in a calibrated, aligned map. */
(function(root,factory){
 const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.VINMap=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const validPoint=p=>p&&Number.isFinite(p.x)&&Number.isFinite(p.y);
 const validMap=m=>Boolean(m&&m.schema==='pai-vin-metric-map-v1'&&m.units==='m'&&m.metricScaleVerified===true&&
   m.coordinateFrame==='building-floor-xy'&&typeof m.mapId==='string'&&m.mapId.length>0&&
   Array.isArray(m.landmarks)&&m.landmarks.every(validPoint)&&m.landmarks.length>=4&&
   Number.isFinite(m.floor));
 class Track {
  constructor(map){if(!validMap(map))throw Error('Metric map is not verified');this.map=map;this.poses=[];this.rejected=0}
  add(p){
   if(!p||p.mapId!==this.map.mapId||!validPoint(p)||!Number.isFinite(p.timestamp)||!Number.isFinite(p.confidence)||p.confidence<.5||p.confidence>1||p.verified!==true||p.floor!==this.map.floor){this.rejected++;return false}
   if(this.poses.length){
    const last=this.poses[this.poses.length-1],dt=(p.timestamp-last.timestamp)/1000;
    if(dt<=0||dt>5||Math.hypot(p.x-last.x,p.y-last.y)>Math.max(1.5,dt*3.5)){this.rejected++;return false}
   }
   this.poses.push({x:p.x,y:p.y,timestamp:p.timestamp,confidence:p.confidence});
   if(this.poses.length>3000)this.poses.shift();
   return true;
  }
 }
 function viewport(map,poses,width,height){
  const all=[...map.landmarks,...poses].filter(validPoint);
  const xs=all.map(p=>p.x),ys=all.map(p=>p.y);
  const minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
  const rangeX=Math.max(2,maxX-minX),rangeY=Math.max(2,maxY-minY);
  const scale=Math.min((width-42)/rangeX,(height-42)/rangeY);
  const cx=(minX+maxX)/2,cy=(minY+maxY)/2;
  return p=>[width/2+(p.x-cx)*scale,height/2-(p.y-cy)*scale];
 }
 function render(canvas,map,poses=[]){
  if(!validMap(map))throw Error('Cannot render unverified metric map');
  const ctx=canvas.getContext('2d'),width=canvas.width,height=canvas.height;
  if(!ctx)throw Error('Canvas unavailable');
  ctx.fillStyle='#102139';ctx.fillRect(0,0,width,height);
  const proj=viewport(map,poses,width,height);
  ctx.strokeStyle='#28455f';ctx.lineWidth=1;
  for(let i=0;i<=width;i+=40){ctx.beginPath();ctx.moveTo(i,0);ctx.lineTo(i,height);ctx.stroke()}
  for(let i=0;i<=height;i+=40){ctx.beginPath();ctx.moveTo(0,i);ctx.lineTo(width,i);ctx.stroke()}
  ctx.fillStyle='#8fb9dd';
  for(const p of map.landmarks){const [x,y]=proj(p);ctx.beginPath();ctx.arc(x,y,2,0,Math.PI*2);ctx.fill()}
  if(poses.length){
   ctx.strokeStyle='#56e1a0';ctx.lineWidth=3;ctx.beginPath();
   poses.forEach((p,i)=>{const [x,y]=proj(p);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});
   ctx.stroke();
   const [x,y]=proj(poses[poses.length-1]);ctx.fillStyle='#ffcf68';ctx.beginPath();ctx.arc(x,y,6,0,Math.PI*2);ctx.fill();
  }
  ctx.fillStyle='#c9ddf0';ctx.font='14px -apple-system,sans-serif';
  ctx.fillText('Floor '+map.floor+' · metric map',12,22);
  return true;
 }
 return {validMap,Track,render};
});