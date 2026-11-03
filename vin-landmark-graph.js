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
  // Every node below is a captured viewpoint in time, not a physical XY position.
  // Do not draw fake serpentine lines suggesting a walking route.
  const n=graph.frames.length,cols=10,spacing=28;
  c.fillStyle='#a5bdd5';c.font='13px -apple-system,sans-serif';
  c.fillText('已采集 '+n+' 帧（只按时间编号）',14,45);
  const visible=graph.frames.slice(-70),first=n-visible.length;
  visible.forEach((f,j)=>{
   const i=first+j,x=22+(j%cols)*Math.min(31,(w-35)/cols),y=76+Math.floor(j/cols)*spacing;
   c.beginPath();c.arc(x,y,i===active?6:4,0,Math.PI*2);
   c.fillStyle=i===active?'#ffce6b':'#71c4ed';c.fill();
   if((j%cols)===0){c.fillStyle='#90a9c4';c.font='10px -apple-system';c.fillText('#'+f.id,Math.max(4,x-17),y-11)}
  });
  if(graph.closed){c.fillStyle='#9ff4c7';c.fillText('候选回访，不等于真实轨迹闭环',14,h-12)}

 }
 return {Graph,render,isPoints,MAX};
});