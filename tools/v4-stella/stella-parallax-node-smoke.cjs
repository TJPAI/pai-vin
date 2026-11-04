// Deterministic synthetic 3D-parallax camera sequence for genuine Stella WASM.
// This is an algorithmic integration fixture, NOT a claim of iPhone accuracy.
// Two physical-depth planes create real, different image motion under camera
// translation; NO fabricated map, pose, or estimated trajectory is produced.
'use strict';
const assert = require('node:assert/strict');
const path = require('node:path');

const W=320,H=240,FX=500,FY=500,CX=160,CY=120;
function hash(x,y,layer) {
  let a = (Math.imul(x, 374761393) + Math.imul(y, 668265263) + Math.imul(layer, 2246822519))|0;
  a = Math.imul(a ^ (a>>>13), 1274126177);
  return (a^(a>>>16))>>>0;
}
const textureScale = Number(process.env.PAI_TEXTURE_SCALE || 41);
function texture(x,y,layer) {
  const u = Math.floor(x*textureScale), v=Math.floor(y*textureScale);
  const tile=(hash(u,v,layer)>>>24);
  const grid=(u%5===0||v%7===0)?40:0;
  const cross=(Math.abs(x*textureScale-u-0.5)<0.09||Math.abs(y*textureScale-v-0.5)<0.09)?36:0;
  return Math.min(255,25+(tile*0.68)+grid+cross)|0;
}
function imageAt(cameraX, cameraY, yaw=0) {
  const pixels=new Uint8Array(W*H*4);
  for(let y=0;y<H;y++) for(let x=0;x<W;x++){
    const dx=(x-CX)/FX,dy=(y-CY)/FY;
    const rayX=Math.cos(yaw)*dx+Math.sin(yaw);
    const rayZ=Math.cos(yaw)-Math.sin(yaw)*dx;
    if(rayZ<=0.001){const i=(y*W+x)*4;pixels[i]=pixels[i+1]=pixels[i+2]=0;pixels[i+3]=255;continue;}
    const foreX=cameraX+rayX*2.15/rayZ,foreY=cameraY+dy*2.15/rayZ;
    const foreground=foreX>-0.55&&foreX<0.1&&foreY>-0.41&&foreY<0.41;
    const depth=foreground?2.15:4.6;
    const worldX=cameraX+rayX*depth/rayZ,worldY=cameraY+dy*depth/rayZ;
    let v=texture(worldX,worldY,foreground?1:2);
    const i=(y*W+x)*4; pixels[i]=v;pixels[i+1]=v;pixels[i+2]=v;pixels[i+3]=255;
  }
  return pixels;
}
(async()=>{
  if(!process.argv[2])throw Error('Usage: node stella-parallax-node-smoke.cjs <module.js>');
  const factory=require(path.resolve(process.argv[2]));
  const mod=await factory();
  const preset=process.env.PAI_PARALLAX_PRESET||'baseline';
  const settings={baseline:{step:0.010,minMatch:50,minTri:50,minParallax:1.0},wide:{step:0.025,minMatch:35,minTri:25,minParallax:0.8},near:{step:0.017,minMatch:25,minTri:15,minParallax:0.5}};
  const cfg=settings[preset]; if(!cfg)throw Error('Unknown preset: '+preset);
  const yaml=`Camera:
  name: PAI VIN parallax validation fixture
  setup: monocular
  model: perspective
  fx: ${FX}
  fy: ${FY}
  cx: ${CX}
  cy: ${CY}
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
  fps: 30.0
  cols: ${W}
  rows: ${H}
  color_order: RGB
Preprocessing:
  min_size: 320
Feature:
  name: PAI VIN parallax ORB
  scale_factor: 1.2
  num_levels: 6
  ini_fast_threshold: 12
  min_fast_threshold: 6
Mapping:
  enable_interruption_of_landmark_generation: false
  enable_interruption_before_local_BA: false
`;
  assert.equal(mod.ccall('pai_stella_create','number',['string'],[yaml]),0);
  const p=mod._malloc(W*H*4),pose=mod._malloc(16*8);
  assert(p&&pose);
  const turnReturn=process.env.PAI_TURN_RETURN==='1';
  const numFrames=turnReturn?64:32;
  let nativePoses=0, noPose=0, firstPose=null, posesDuringTurn=0, posesAfterReturn=0;
  const poses=[];
  for(let i=0;i<numFrames;i++){
    // Real geometrical camera translation (world frame), not a pose returned.
    const cameraX=turnReturn&&i>=32 ? (63-i)*cfg.step : i*cfg.step;
    const cameraY=Math.sin(i*0.15)*0.009;
    const yaw=turnReturn&&i>=32&&i<56 ? 0.8*Math.sin(Math.PI*(i-32)/24):0;
    const image=imageAt(cameraX,cameraY,yaw);
    mod.HEAPU8.set(image,p);
    const rc=mod.ccall('pai_stella_track_rgba','number',
      ['number','number','number','number','number'],
      [p,W,H,(i+1)/30,pose]);
    if(rc<0){
      let e='native_error';const q=mod._pai_stella_last_error();
      if(q)e=mod.UTF8ToString(q);
      throw Error(`Native Stella failed frame ${i}, rc=${rc}: ${e}`);
    }
    assert(rc===0||rc===1);
    if(rc===1){
      const poseOut=Array.from(new Float64Array(mod.HEAPU8.buffer,pose,16));
      assert(poseOut.every(Number.isFinite));
      nativePoses++;
      if(turnReturn&&i>=32&&i<56)posesDuringTurn++;
      if(turnReturn&&i>=56)posesAfterReturn++;
      if(firstPose===null)firstPose=i;
      poses.push({frame:i,translation:[poseOut[12],poseOut[13],poseOut[14]]});
    }else noPose++;
  }
  mod._free(p);mod._free(pose);mod._pai_stella_destroy();
  console.log('PAI_STELLA_GEOMETRIC_PARALLAX_MULTIFRAME_PASS', JSON.stringify({
    preset,textureScale,frames:numFrames,nativePoses,noPose,firstPose,posesDuringTurn,posesAfterReturn,poseSample:poses.slice(0,3),
    caveat:'synthetic two-plane scene; no iPhone or metric accuracy verification'
  }));
  if(turnReturn)console.log('PAI_STELLA_TURN_RETURN_MULTIFRAME_PASS',JSON.stringify({posesDuringTurn,posesAfterReturn,nativePoses}));
  if(nativePoses===0)console.log('PAI_STELLA_PARALLAX_INIT_NOT_YET_VALIDATED');
})().catch(e=>{console.error(e);process.exitCode=1;});
