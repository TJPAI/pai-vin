const assert=require('node:assert/strict'),fs=require('node:fs');
const page=fs.readFileSync('vin-app.html','utf8'),home=fs.readFileSync('index.html','utf8');
const sw=fs.readFileSync('sw.js','utf8'),manifest=JSON.parse(fs.readFileSync('manifest.json','utf8'));
const required=['boardCamera','boardCanvas','boardOpen','boardSnap','boardStatus','vioDev','vioVideo','vioStart','vioStop','vioTrackCanvas','vioTrackNumbers','vioInitStatus','vioFusionStatus','vioNumbers','vioExport','vioStatus'];
for(const id of required)assert.ok(page.includes('id="'+id+'"'),'missing '+id);
const scripts=['vin-a4-board.js','vin-a4-detect.js','vin-app.js','web-vio-features.js','vio-sensor-core.js','vio-tracker.js','vio-initializer.js','vio-rotation.js','vio-fusion.js','vio-observation.js','vio-sensor-probe.js'];
for(const name of scripts){assert.ok(page.includes('src="./'+name+'"'),'missing '+name);assert.ok(sw.includes("'./"+name+"'"),'not cached '+name)}
assert.ok(sw.includes("'./vio-vision-worker.js'"),'worker script must be offline cached');
assert.ok(fs.readFileSync('vio-sensor-probe.js','utf8').includes("new Worker('./vio-vision-worker.js')"),'worker must run in separate execution context');
const legacy=['vin-scan.js','vin-vo-path.js','vin-landmark-graph.js','web-vio-demo.js','web-vio-demo.html','web-vio-pose.js','web-vio-inertial.js','web-vio-geometry.js','web-metric-map.js','slam-capture.js','visualGraph','voCanvas','scanStart','scanStop','mapFile','startCamera','legacy VO'];
for(const name of legacy){assert.ok(!page.includes(name),'legacy in UI: '+name);assert.ok(!sw.includes(name),'legacy cached: '+name)}
assert.equal(manifest.start_url,'./vin-app.html');
const version=page.match(/id="appVersion"[^>]*data-version="(V[0-9.]+)"[^>]*>([^<]+)<[/]span>/);
assert.ok(version,'visible version must be beside PAI-VIN header');
assert.ok(page.includes('data-build="v3132-exact-corner-20261103"'),'build identifier must be visible in document metadata');
assert.ok(fs.readFileSync('vio-sensor-probe.js','utf8').includes('appVersion:document.getElementById'),'diagnostic export must identify actual running UI version');
assert.ok(version[2].startsWith(version[1]),'display version and data-version must match');
assert.ok(page.includes('<title>PAI-VIN · '+version[2]+'</title>'),'browser title must share visible version');
assert.ok(manifest.name.includes(version[1]),'PWA manifest must share visible version');
assert.ok(sw.includes('pai-vin-v3132-20261103'),'bump service worker cache with display-version release');
assert.ok(!page.includes('boardReset')&&!page.includes('boardSave'),'manual controls must be absent');
assert.ok(home.includes('vin-app.html'));
const probe=fs.readFileSync('vio-sensor-probe.js','utf8');
assert.ok(probe.includes('t-lastTrackingTime<90'),'100 ms sampler must not be throttled to 5 Hz by 120 ms gate');
console.log('Clean V3.13.2 header version/manifest/offline asset tests PASS');
