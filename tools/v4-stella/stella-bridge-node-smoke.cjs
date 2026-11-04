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
  assert.equal(typeof mod._pai_stella_get_diagnostics,'function','Missing native Stella V1.3 diagnostics ABI');
  const createRc = mod.ccall('pai_stella_create','number',['string'],[yaml]);
  assert.equal(createRc, 0, 'native Stella construction failed');
  const width = 320, height = 240, bytes = width * height * 4;
  const p = mod._malloc(bytes), pose = mod._malloc(16 * 8), stats = mod._malloc(20);
  assert(p > 0 && pose > 0 && stats > 0);
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
  // Continue with distinct, deterministically translated REAL input frames.
  // This is a stability/deadlock smoke, not a valid 3D/parallax fixture:
  // it must never claim localization when Stella reports no native pose.
  const total = 24;
  let poses = trackRc === 1 ? 1 : 0;
  let noPoses = trackRc === 0 ? 1 : 0;
  const moved = new Uint8Array(bytes);
  for (let frame = 1; frame <= total; frame++) {
    const shiftX = Math.round(9 * Math.sin(frame * 0.3));
    const shiftY = Math.round(6 * Math.cos(frame * 0.25));
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const sx = (x + shiftX + width) % width;
        const sy = (y + shiftY + height) % height;
        const from = (sy * width + sx) * 4;
        const to = (y * width + x) * 4;
        moved[to] = pixels[from];
        moved[to+1] = pixels[from+1];
        moved[to+2] = pixels[from+2];
        moved[to+3] = 255;
      }
    }
    mod.HEAPU8.set(moved, p);
    const rc = mod.ccall('pai_stella_track_rgba','number',
      ['number','number','number','number','number'],
      [p,width,height,(frame+1)/30,pose]);
    assert([0,1].includes(rc), `Native Stella failure at frame ${frame}: ${rc}`);
    if (rc === 1) {
      poses++;
      const matrix = Array.from(new Float64Array(mod.HEAPU8.buffer,pose,16));
      assert(matrix.every(Number.isFinite), 'Nonfinite native pose');
    } else noPoses++;
  }
  console.log(`PAI_STELLA_BROWSER_ABI_MULTIFRAME_PASS frames=${total+1} poses=${poses} no_pose=${noPoses} fixture=2D_translation_not_3D_parallax`);
  // Pose count is diagnostic only: not an acceptance criterion for this 2D fixture.
  assert.equal(mod._pai_stella_get_diagnostics(stats), 0, 'Stella tracking telemetry read failed');
  const telemetry=Array.from(new Int32Array(mod.HEAPU8.buffer,stats,5));
  assert([0,1,2].includes(telemetry[0]), 'Unknown tracker state');
  assert(telemetry.slice(1).every(v=>Number.isInteger(v)&&v>=0),'Invalid map/ORB telemetry');
  console.log('PAI_STELLA_NATIVE_TRACK_DIAGNOSTICS_PASS',JSON.stringify(telemetry));
  mod._free(stats);
  mod._free(p); mod._free(pose); mod._pai_stella_destroy();
  console.log('PAI_STELLA_BROWSER_ABI_NODE_PASS');
})().catch(e => { console.error(e); process.exitCode = 1; });
