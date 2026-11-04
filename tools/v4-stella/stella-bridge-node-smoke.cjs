// Real Stella WebAssembly C ABI smoke (no synthetic pose accepted).
const assert = require('node:assert/strict');
const path = require('node:path');

(async () => {
  const location = process.argv[2];
  if (!location) throw new Error('usage: node stella-bridge-node-smoke.cjs <module.js>');
  const create = require(path.resolve(location));
  assert.equal(typeof create, 'function');
  const mod = await create();
  for (const fn of ['_malloc','_free','_pai_stella_create','_pai_stella_features_rgba','_pai_stella_track_rgba','_pai_stella_destroy']) {
    assert.equal(typeof mod[fn], 'function', `missing ${fn}`);
  }
  const yaml = `Camera:
  name: PAI-VIN real wasm browser bridge
  setup: monocular
  model: perspective
  fx: 500.0
  fy: 500.0
  cx: 160.0
  cy: 120.0
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
  fps: 30.0
  cols: 320
  rows: 240
  color_order: RGB
Preprocessing:
  min_size: 320
Feature:
  name: PAI-VIN probe ORB
  scale_factor: 1.2
  num_levels: 4
  ini_fast_threshold: 20
  min_fast_threshold: 7
`;
  const createRc = mod.ccall('pai_stella_create','number',['string'],[yaml]);
  assert.equal(createRc, 0, 'native Stella construction failed');
  const width = 320, height = 240, bytes = width * height * 4;
  const p = mod._malloc(bytes), pose = mod._malloc(16 * 8);
  assert(p > 0 && pose > 0);
  let seed = 0x9e3779b9;
  const pixels = new Uint8Array(bytes);
  for (let y = 0; y < height; y++)
    for (let x = 0; x < width; x++) {
      seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
      const i = (y * width + x) * 4, v = seed >>> 24;
      pixels[i] = v; pixels[i + 1] = v; pixels[i + 2] = v; pixels[i + 3] = 255;
    }
  mod.HEAPU8.set(pixels, p);
  const featureCount = mod.ccall('pai_stella_features_rgba','number',['number','number','number','number'],[p,width,height,0.0]);
  assert(featureCount > 0, `real ORB extraction count must be >0, got ${featureCount}`);
  console.log(`PAI_STELLA_BROWSER_ABI_REAL_ORB_PASS features=${featureCount}`);
  // A single frame cannot generate a reliable mono pose. Return code 0 is valid
  // and NEVER interpreted as tracking success.
  const trackRc = mod.ccall('pai_stella_track_rgba','number',['number','number','number','number','number'],[p,width,height,0.033,pose]);
  assert([0,1].includes(trackRc), `Stella real tracking returned error ${trackRc}`);
  if (trackRc === 1) {
    const mat = Array.from(new Float64Array(mod.HEAPU8.buffer, pose, 16));
    assert(mat.every(Number.isFinite));
    console.log('PAI_STELLA_BROWSER_ABI_NATIVE_POSE_PRESENT');
  } else {
    console.log('PAI_STELLA_BROWSER_ABI_NO_POSE_YET (initialization, not a failure)');
  }
  mod._free(p); mod._free(pose); mod._pai_stella_destroy();
  console.log('PAI_STELLA_BROWSER_ABI_NODE_PASS');
})().catch(e => { console.error(e); process.exitCode = 1; });
