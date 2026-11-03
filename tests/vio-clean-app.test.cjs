const assert=require('node:assert/strict'),fs=require('node:fs');
const page=fs.readFileSync('vin-app.html','utf8'),home=fs.readFileSync('index.html','utf8');
const sw=fs.readFileSync('sw.js','utf8'),manifest=JSON.parse(fs.readFileSync('manifest.json','utf8'));
const required=['boardCamera','boardCanvas','boardOpen','boardSnap','boardStatus','vioDev','vioVideo','vioStart','vioStop','vioTrackCanvas','vioTrackNumbers','vioInitStatus','vioFusionStatus','vioNumbers','vioExport','vioStatus'];
for(const id of required)assert.ok(page.includes('id="'+id+'"'),'missing '+id);
const scripts=['vin-a4-board.js','vin-a4-detect.js','vin-app.js','web-vio-features.js','vio-sensor-core.js','vio-tracker.js','vio-initializer.js','vio-rotation.js','vio-fusion.js','vio-sensor-probe.js'];
for(const name of scripts){assert.ok(page.includes('src="./'+name+'"'),'missing '+name);assert.ok(sw.includes("'./"+name+"'"),'not cached '+name)}
const legacy=['vin-scan.js','vin-vo-path.js','vin-landmark-graph.js','web-vio-demo.js','web-vio-demo.html','web-vio-pose.js','web-vio-inertial.js','web-vio-geometry.js','web-metric-map.js','slam-capture.js','visualGraph','voCanvas','scanStart','scanStop','mapFile','startCamera','legacy VO'];
for(const name of legacy){assert.ok(!page.includes(name),'legacy in UI: '+name);assert.ok(!sw.includes(name),'legacy cached: '+name)}
assert.equal(manifest.start_url,'./vin-app.html');
assert.ok(page.includes('V3.5 Alpha'));
assert.ok(!page.includes('boardReset')&&!page.includes('boardSave'),'manual controls must be absent');
assert.ok(home.includes('vin-app.html'));
console.log('Clean V3.5 product/manifest/offline asset tests PASS');
