/* PAI-VIN iPhone Safari feature frontend: 256x192 grayscale, corners and
   deterministic 64-bit local binary descriptors. No images leave device.
   Visual front-end only, not metric SLAM, intrinsics or 6DoF pose. */
(function(root,factory){
 const m=factory();if(typeof module==='object'&&module.exports)module.exports=m;else root.VINFeatures=m;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const W=256,H=192;
 const pattern=(()=>{
  let seed=0x327a41b9;const rand=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296};
  return Array.from({length:64},()=>Array.from({length:2},()=>[Math.floor(rand()*17)-8,Math.floor(rand()*17)-8]));
 })();
 function grayRGBA(raw,width=W,height=H){
  const out=new Uint8Array(width*height);
  if(raw.length<width*height*4)throw new Error('RGBA buffer too short');
  for(let i=0,j=0;j<out.length;i+=4,j++)out[j]=(raw[i]*77+raw[i+1]*150+raw[i+2]*29)>>8;
  return out;
 }
 function detect(gray,width=W,height=H,max=115){
  if(width!==W||height!==H)throw new Error('feature image must be 256x192');
  if(!gray||gray.length<W*H)throw new Error('invalid grayscale frame');
  const candidates=[];
  for(let y=11;y<H-11;y+=3)for(let x=11;x<W-11;x+=3){
   const i=y*W+x;
   const dx=Math.abs(gray[i+W+1]-gray[i+W-1])+Math.abs(gray[i-W+1]-gray[i-W-1]);
   const dy=Math.abs(gray[i+W+1]-gray[i-W+1])+Math.abs(gray[i+W-1]-gray[i-W-1]);
   const d1=Math.abs(gray[i+W+1]-gray[i-W-1]),d2=Math.abs(gray[i+W-1]-gray[i-W+1]);
   const score=Math.min(dx,dy)+Math.min(d1,d2);
   if(score>86)candidates.push({x,y,score});
  }
  candidates.sort((a,b)=>b.score-a.score);
  const result=[],occupied=new Set();
  for(const p of candidates){
   const cell=(p.y/12|0)*100+(p.x/12|0);
   if(occupied.has(cell))continue;
   occupied.add(cell);
   let a=0,b=0;
   for(let n=0;n<64;n++){
    const pair=pattern[n],i1=(p.y+pair[0][1])*W+p.x+pair[0][0],i2=(p.y+pair[1][1])*W+p.x+pair[1][0];
    if(gray[i1]<gray[i2]){if(n<32)a=(a|(1<<n))>>>0;else b=(b|(1<<(n-32)))>>>0}
   }
   result.push({x:p.x,y:p.y,a,b});
   if(result.length>=max)break;
  }
  return result;
 }
 return {W,H,grayRGBA,detect};
});
