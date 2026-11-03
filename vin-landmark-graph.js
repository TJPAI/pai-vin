/* PAI-VIN local landmark graph: true visual keyframes, NOT geometric SLAM.
   Device-only feature descriptors (no photos); IndexedDB, no network calls.
   Node order is temporal, not a measured 2D floor-plan coordinate. */
(function(root,factory){
 const v=factory();if(typeof module==='object'&&module.exports)module.exports=v;else root.VINLandmarks=v;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const MAX=160,MIN_FEAT=18;
 const isPoints=p=>Array.isArray(p)&&p.length>=MIN_FEAT&&p.length<=130&&p.every(v=>
  Number.isFinite(v.x)&&Number.isFinite(v.y)&&Number.isInteger(v.a)&&Number.isInteger(v.b));
 function similarity(A,B){return VINGeometry.verify(A,B)}
 class Graph {
  constructor(saved){
   this.frames=[];this.closed=false;this.revisit=null;
   if(saved){
    if(saved.schema!=='pai-vin-visual-landmark-graph-v1'||!Array.isArray(saved.frames)||saved.frames.length>MAX)throw Error('invalid landmark graph');
    for(const f of saved.frames){
     if(!isPoints(f.points)||!Number.isFinite(f.t)||typeof f.id!=='number')throw Error('invalid frame');
     this.frames.push({id:f.id,t:f.t,points:f.points});
    }
    this.closed=Boolean(saved.closed&&this.frames.length>=12);
   }
  }
  add(points,t){
   if(!isPoints(points))return {added:false,reason:'low_texture',count:this.frames.length};
   if(!Number.isFinite(t))return {added:false,reason:'time_invalid',count:this.frames.length};
   if(this.frames.length>=MAX)return {added:false,reason:'map_full',count:this.frames.length};
   const prev=this.frames.at(-1);
   if(prev){
    if(t<=prev.t)return {added:false,reason:'time_invalid',count:this.frames.length};
    const v=similarity(points,prev.points);
    if(v.accepted&&v.inliers>=18&&v.ratio>=.55)return {added:false,reason:'same_view',count:this.frames.length};
   }
   const f={id:this.frames.length+1,t,points};this.frames.push(f);
   // A revisit of the first few frames after >=12 new keyframes is a loop candidate,
   // not a metric loop closure or graph-optimized trajectory.
   if(this.frames.length>=12){
    const r=this.match(points,this.frames.slice(0,Math.min(4,this.frames.length-8)));
    if(r.accepted&&r.best?.inliers>=12){this.closed=true;this.revisit=r.best.index+1}
   }
   return {added:true,count:this.frames.length,closed:this.closed,revisit:this.revisit};
  }
  match(points,subset){
   if(!isPoints(points)||!this.frames.length)return {accepted:false,reason:'no_visual_match'};
   const all=subset||this.frames;
   if(!all.length)return {accepted:false,reason:'empty_subset'};
   const r=VINGeometry.rank(points,all);
   return {...r,index:r.best?.index??-1};
  }
  serialize(){return {schema:'pai-vin-visual-landmark-graph-v1',createdAt:new Date().toISOString(),
   units:'none',metricScaleVerified:false,globalPositionKnown:false,closed:this.closed,
   note:'Temporal visual-keyframe topology only; NOT a metric indoor map',
   frames:this.frames.map(f=>({id:f.id,t:f.t,points:f.points}))}}
 }
 function render(canvas,graph,active=-1){
  const c=canvas.getContext('2d'),w=canvas.width,h=canvas.height;
  c.fillStyle='#102139';c.fillRect(0,0,w,h);
  c.fillStyle='#c8d8e8';c.font='14px -apple-system,sans-serif';
  c.fillText('视觉关键帧顺序图（非二维米制地图）',14,24);
  if(!graph?.frames.length){c.fillText('尚未扫描环境',14,58);return}
  const n=graph.frames.length,cols=Math.min(12,Math.ceil(Math.sqrt(n*1.8))),rows=Math.ceil(n/cols);
  const dx=(w-38)/Math.max(1,cols-1),dy=Math.min(35,(h-73)/Math.max(1,rows-1));
  const xy=i=>[19+(i%cols)*dx,49+Math.floor(i/cols)*dy];
  // A snake traversal reduces long diagram lines; no physical coordinate meaning.
  const loc=i=>{const r=Math.floor(i/cols),j=i%cols;return xy(r%2===0?i:r*cols+(cols-1-j))};
  c.strokeStyle='#6585a6';c.lineWidth=1.5;c.beginPath();
  for(let i=0;i<n;i++){const [x,y]=loc(i);if(i===0)c.moveTo(x,y);else c.lineTo(x,y)}c.stroke();
  graph.frames.forEach((f,i)=>{const [x,y]=loc(i);c.beginPath();c.arc(x,y,i===active?6:3.5,0,Math.PI*2);
    c.fillStyle=i===active?'#ffcd5a':'#71c4ed';c.fill()});
  if(graph.closed&&n>=12){c.fillStyle='#9ff4c7';c.fillText('检测到回访首批画面（候选闭环）',14,h-12)}
 }
 return {Graph,render,isPoints,MAX};
});