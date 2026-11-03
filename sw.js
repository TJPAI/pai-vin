/* PAI-VIN V3.2: only current product modules are cached for offline use.
 * Legacy demos are intentionally not part of this version. */
const CACHE='pai-vin-v3132-20261103';
const CORE=['./','./index.html','./vin-app.html','./vin-app.js','./manifest.json','./a4-board.html','./vin-a4-board.js','./vin-a4-detect.js','./web-vio-features.js','./vio-sensor-core.js','./vio-tracker.js','./vio-initializer.js','./vio-rotation.js','./vio-geometry.js','./vio-fusion.js','./vio-observation.js','./vio-vision-worker.js','./vio-sensor-probe.js'];
self.addEventListener('install',event=>event.waitUntil((async()=>{
 const c=await caches.open(CACHE);
 await c.addAll(CORE.map(url=>new Request(url,{cache:'reload'})));
 await self.skipWaiting();
})()));
self.addEventListener('activate',event=>event.waitUntil((async()=>{
 const all=await caches.keys();
 await Promise.all(all.filter(x=>x.startsWith('pai-vin-')&&x!==CACHE).map(x=>caches.delete(x)));
 await self.clients.claim();
})()));
self.addEventListener('fetch',event=>{
 const req=event.request,url=new URL(req.url);
 if(req.method!=='GET'||url.origin!==self.location.origin)return;
 const isCurrent=CORE.some(x=>new URL(x,self.registration.scope).pathname===url.pathname);
 if(!isCurrent)return;
 event.respondWith((async()=>{
  try{
   const response=await fetch(new Request(req,{cache:'no-store'}));
   if(response.ok){const cache=await caches.open(CACHE);await cache.put(req,response.clone()).catch(()=>{})}
   return response;
  }catch(e){
   const cached=await caches.match(req,{ignoreSearch:true});
   if(cached)return cached;
   throw e;
  }
 })());
});
