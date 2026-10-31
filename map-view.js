/* PAI-VIN V0.4: relative-map visualization only.
   No floorplan, no surveyed coordinates and no claims of metric accuracy. */
(function(global){
'use strict';
const NS='http://www.w3.org/2000/svg';
function el(tag,attrs,text){const n=document.createElementNS(NS,tag);for(const [k,v] of Object.entries(attrs||{}))n.setAttribute(k,String(v));if(text!==undefined)n.textContent=text;return n}
function render({x,y,tracks,anchors,correction}){
 const svg=document.getElementById('map');
 if(!svg)return;
 const pts=[{x:0,y:0},{x,y},...tracks.map(p=>({x:p[0],y:p[1]})),...anchors];
 if(correction){pts.push({x:correction.old[0],y:correction.old[1]},{x:correction.at[0],y:correction.at[1]})}
 const xs=pts.map(p=>p.x),ys=pts.map(p=>p.y);
 const minx=Math.min(...xs),maxx=Math.max(...xs),miny=Math.min(...ys),maxy=Math.max(...ys);
 const span=Math.max(14,maxx-minx+8,(maxy-miny+8)*360/250),height=span*250/360;
 const cx=(minx+maxx)/2,cy=(miny+maxy)/2;
 svg.setAttribute('viewBox',[(cx-span/2).toFixed(2),(-cy-height/2).toFixed(2),span.toFixed(2),height.toFixed(2)].join(' '));
 // Screen Y grows down while indoor Y grows up.
 const group=document.getElementById('mapDynamic');
 if(!group)return;
 group.replaceChildren();
 const path=tracks.map((p,i)=>(i?'L':'M')+p[0].toFixed(2)+' '+(-p[1]).toFixed(2)).join(' ');
 if(path)group.appendChild(el('path',{d:path,fill:'none',stroke:'#66d4af','stroke-width':Math.max(.08,span/200),'stroke-linecap':'round','stroke-linejoin':'round'}));
 group.appendChild(el('circle',{cx:0,cy:0,r:span/150,fill:'#c4d5e8'}));
 for(const [i,a] of anchors.entries()){
  group.appendChild(el('circle',{cx:a.x,cy:-a.y,r:span/120,fill:'#ffcf74',stroke:'#192a3f','stroke-width':span/700}));
  const text=el('text',{x:a.x+span/90,y:-a.y-span/100,fill:'#ffe4a5','font-size':span/47,'font-weight':600},a.label||('点'+(i+1)));
  group.appendChild(text);
 }
 if(correction){
  group.appendChild(el('path',{d:'M'+correction.old[0]+' '+(-correction.old[1])+' L'+correction.at[0]+' '+(-correction.at[1]),stroke:'#ff9f87','stroke-width':span/130,'stroke-dasharray':(span/50)+' '+(span/75),fill:'none'}));
 }
 group.appendChild(el('circle',{cx:x,cy:-y,r:span/85,fill:'#4eaaff',stroke:'white','stroke-width':span/300}));
 document.getElementById('mapLegend').textContent='锚点 '+anchors.length+' · 轨迹点 '+tracks.length+
  (correction?' · 最近一次纠偏 '+correction.jump.toFixed(1)+' m（未标定）':'')+
  ' · 自动缩放';
}
global.VINMap={render};
})(window);
