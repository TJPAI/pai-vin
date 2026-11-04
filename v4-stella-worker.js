/* Experimental Stella worker: only genuine WASM outputs; never synthesize poses.
 * Hosted separately from AlvaAR; real concurrent mapping is NOT yet validated.
 */
'use strict';
let mod = null, imagePtr = 0, posePtr = 0, diagPtr = 0, frameBytes = 0, width = 0, height = 0;
function send(type, data = {}) { self.postMessage({type, ...data}); }
function lastError() {
  if (!mod) return 'module_unavailable';
  const p = mod._pai_stella_last_error();
  return p ? mod.UTF8ToString(p) : 'native_error';
}
async function init(message) {
  try {
    importScripts('./vendor/stella/pai_stella_browser.js');
    if (typeof self.createPAIStella !== 'function') throw Error('Stella WASM factory missing');
    mod = await self.createPAIStella({locateFile: file => './vendor/stella/' + file});
    if (!mod.HEAPU8 || !mod._pai_stella_create) throw Error('Stella browser ABI incomplete');
    const rc = mod.ccall('pai_stella_create', 'number', ['string'], [message.yaml]);
    if (rc !== 0) throw Error('Stella init error ' + rc + ': ' + lastError());
    width = message.width; height = message.height;
    frameBytes = width * height * 4;
    imagePtr = mod._malloc(frameBytes);
    posePtr = mod._malloc(16 * 8);
    diagPtr = mod._malloc(5 * 4);
    if (!imagePtr || !posePtr || !diagPtr || typeof mod._pai_stella_get_diagnostics !== 'function') throw Error('Stella V1.3 diagnostics ABI missing or allocation failed');
    send('ready', {message: '真实 Stella 引擎已装载。尚未确认实时定位。'});
  } catch (error) {
    send('error', {message: String(error?.message || error)});
  }
}
function frame(message) {
  if (!mod || !imagePtr) return send('error', {message:'engine_not_ready'});
  try {
    const begin = performance.now();
    const rgba = new Uint8Array(message.pixels);
    if (rgba.byteLength !== frameBytes) throw Error('image dimensions mismatch');
    mod.HEAPU8.set(rgba, imagePtr);
    const args = [imagePtr, width, height, message.timestamp];
    if (message.mode === 'features') {
      const count = mod.ccall('pai_stella_features_rgba', 'number',
        ['number','number','number','number'], args);
      if (count < 0) throw Error('native ORB: ' + lastError());
      send('frame', {sequence:message.sequence, features:count, pose:null, mode:'features', processingMs:performance.now()-begin});
      return;
    }
    const rc = mod.ccall('pai_stella_track_rgba','number',
      ['number','number','number','number','number'], [...args, posePtr]);
    if (rc < 0) throw Error('native tracker: ' + lastError());
    let pose = null;
    if (rc === 1) {
      pose = Array.from(new Float64Array(mod.HEAPU8.buffer, posePtr, 16));
      if (!pose.every(Number.isFinite)) throw Error('native pose contains non-finite values');
    }
    const diagResult=mod._pai_stella_get_diagnostics(diagPtr);
    const diag=diagResult===0?Array.from(new Int32Array(mod.HEAPU8.buffer,diagPtr,5)):null;
    send('frame', {sequence:message.sequence, pose, features:null, mode:'track',
      trackingState:diag?.[0]??null,keyframes:diag?.[1]??null,
      mapPoints:diag?.[2]??null,orbCount:diag?.[3]??null,
      associatedPoints:diag?.[4]??null,processingMs:performance.now()-begin});
  } catch (error) {
    send('error', {message: String(error?.message || error)});
  }
}
self.onmessage = async e => {
  if (e.data?.type === 'init') return init(e.data);
  if (e.data?.type === 'frame') return frame(e.data);
  if (e.data?.type === 'destroy') {
    try {
      if (mod) {
        mod._pai_stella_destroy();
        if (imagePtr) mod._free(imagePtr);
        if (posePtr) mod._free(posePtr);
        if (diagPtr) mod._free(diagPtr);
      }
    } finally { mod=null; imagePtr=posePtr=diagPtr=frameBytes=0; send('stopped'); }
  }
};
