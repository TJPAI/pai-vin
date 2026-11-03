/* PAI-VIN A4 four-target automatic one-photo detector.
 * Targets are black 18mm outer squares, white 8mm inset squares, black
 * 4mm center blocks. Bright enclosed white-ring components are detected
 * and validated against the black-white-black concentric pattern.
 * Browser-local, no media upload. A4-plane homography, not full calibration.
 */
(function(root,factory){
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;else root.VINBoardDetect=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const at=(g,w,h,x,y)=>{x=Math.round(x);y=Math.round(y);return x<0||x>=w||y<0||y>=h?null:g[y*w+x]};
 function toGray(rgba,w,h){
  if(!rgba||rgba.length!==w*h*4)throw Error('invalid frame');
  const gray=new Uint8Array(w*h);
  for(let i=0;i<gray.length;i++)gray[i]=(77*rgba[4*i]+150*rgba[4*i+1]+29*rgba[4*i+2])>>8;
  return gray;
 }
 function threshold(gray){
  const hist=new Uint32Array(256);for(let i=0;i<gray.length;i++)hist[gray[i]]++;
  const total=gray.length;let sum=0;for(let i=0;i<256;i++)sum+=i*hist[i];
  let below=0,lower=0,best=-1,t=135;
  for(let i=0;i<256;i++){
   below+=hist[i];if(!below||below===total)continue;
   lower+=i*hist[i];
   const a=lower/below,b=(sum-lower)/(total-below),score=below*(total-below)*(a-b)*(a-b);
   if(score>best){best=score;t=i}
  }
  return Math.min(210,Math.max(85,t+12));
 }
 function rings(gray,w,h,cut){
  const visited=new Uint8Array(w*h),found=[],queue=new Int32Array(w*h);
  const minSize=Math.max(4,Math.floor(Math.min(w,h)*.012));
  const maxSize=Math.min(w,h)*.11;
  for(let y=2;y<h-2;y++)for(let x=2;x<w-2;x++){
   const start=y*w+x;if(visited[start]||gray[start]<=cut)continue;
   let head=0,tail=1;queue[0]=start;visited[start]=1;
   let minX=x,maxX=x,minY=y,maxY=y,count=0,edge=false;
   while(head<tail){
    const p=queue[head++],px=p%w,py=(p/w)|0;count++;
    if(px===0||py===0||px===w-1||py===h-1)edge=true;
    if(px<minX)minX=px;if(px>maxX)maxX=px;
    if(py<minY)minY=py;if(py>maxY)maxY=py;
    for(const n of [p-1,p+1,p-w,p+w]){
     if(n<0||n>=w*h)continue;
     const nx=n%w,ny=(n/w)|0;
     if(Math.abs(px-nx)+Math.abs(py-ny)!==1||visited[n]||gray[n]<=cut)continue;
     visited[n]=1;queue[tail++]=n;
    }
   }
   if(edge)continue;
   const bw=maxX-minX+1,bh=maxY-minY+1;
   if(bw<minSize||bh<minSize||bw>maxSize||bh>maxSize)continue;
   if(bw/bh<.55||bw/bh>1.8)continue;
   const fill=count/(bw*bh);
   // Ring area is ~75% of white square after its black centre is removed.
   if(fill<.32||fill>.95)continue;
   const cx=(minX+maxX)/2,cy=(minY+maxY)/2;
   const core=at(gray,w,h,cx,cy);
   if(core===null||core>cut-12)continue;
   const test=(fraction,bright)=>{
    let passed=0,valid=0;
    for(const [dx,dy] of [[-1,0],[1,0],[0,-1],[0,1]]){
     const v=at(gray,w,h,cx+dx*bw*fraction,cy+dy*bh*fraction);
     if(v===null)continue;valid++;
     if(bright?v>cut+10:v<cut-8)passed++;
    }
    return valid===4?passed:0;
   };
   const inner=test(.34,true),outer=test(.74,false);
   if(inner<3||outer<3)continue;
   found.push({x:cx,y:cy,score:inner+outer+Math.min(1,fill/.72),bw,bh});
  }
  return found.sort((a,b)=>b.score-a.score).slice(0,18);
 }
 function order(pts){
  if(pts.length!==4)return null;
  const used=new Set();
  const pick=rank=>{for(const p of rank)if(!used.has(p)){used.add(p);return p}return null};
  const tl=pick([...pts].sort((a,b)=>(a.x+a.y)-(b.x+b.y)));
  const tr=pick([...pts].sort((a,b)=>(b.x-b.y)-(a.x-a.y)));
  const br=pick([...pts].sort((a,b)=>(b.x+b.y)-(a.x+a.y)));
  const bl=pick([...pts].sort((a,b)=>(a.x-a.y)-(b.x-b.y)));
  return [tl,tr,br,bl].map(p=>[p.x,p.y]);
 }
 function geometry(p,w,h){
  const cross=(a,b,c)=>(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]);
  if(!p||p.length!==4)return false;
  const z=p.map((a,i)=>cross(a,p[(i+1)%4],p[(i+2)%4]));
  if(!z.every(v=>v>150))return false;
  const d=(a,b)=>Math.hypot(p[a][0]-p[b][0],p[a][1]-p[b][1]);
  const top=d(0,1),bottom=d(3,2),left=d(0,3),right=d(1,2);
  if(Math.min(top,bottom)<w*.12||Math.min(left,right)<h*.16)return false;
  if(Math.max(top,bottom)/Math.min(top,bottom)>2.2)return false;
  if(Math.max(left,right)/Math.min(left,right)>2.2)return false;
  const aspect=(top+bottom)/(left+right);
  if(aspect<.32||aspect>1.3)return false;
  return true;
 }
 function detect(rgba,w,h){
  if(!Number.isInteger(w)||!Number.isInteger(h)||w<100||h<100)return {ok:false,reason:'invalid_image'};
  const gray=rgba.length===w*h?rgba:toGray(rgba,w,h),cut=threshold(gray);
  const candidates=rings(gray,w,h,cut);
  if(candidates.length<4)return {ok:false,reason:'markers_not_found',found:candidates.length,threshold:cut};
  let best=null,bestScore=-Infinity,next=-Infinity,combinations=0;
  // Consider small candidate combinations; reject rather than guess if ambiguous.
  for(let i=0;i<candidates.length;i++)for(let j=i+1;j<candidates.length;j++)
   for(let k=j+1;k<candidates.length;k++)for(let m=k+1;m<candidates.length;m++){
    const group=[candidates[i],candidates[j],candidates[k],candidates[m]];
    const p=order(group);if(!geometry(p,w,h))continue;
    combinations++;
    const d=(a,b)=>Math.hypot(p[a][0]-p[b][0],p[a][1]-p[b][1]);
    const balance=Math.abs(Math.log(d(0,1)/d(3,2)))+Math.abs(Math.log(d(0,3)/d(1,2)));
    const aspect=(d(0,1)+d(3,2))/(d(0,3)+d(1,2));
    const ratioPenalty=Math.abs(Math.log(aspect/(160/247)));
    const size=Math.min(...group.map(x=>Math.min(x.bw,x.bh)));
    const total=group.reduce((s,x)=>s+x.score,0)-5*balance-4*ratioPenalty+Math.min(size,20)*.03;
    if(total>bestScore){next=bestScore;bestScore=total;best=p}
    else if(total>next)next=total;
   }
  if(!best)return {ok:false,reason:'invalid_marker_layout',found:candidates.length,threshold:cut};
  if(combinations>1&&bestScore-next<1.6)return {ok:false,reason:'ambiguous_markers',found:candidates.length};
  return {ok:true,corners:best,found:candidates.length,threshold:cut,method:'concentric_square_fiducials',automatic:true};
 }
 return {detect,toGray,threshold,rings,order,geometry};
});