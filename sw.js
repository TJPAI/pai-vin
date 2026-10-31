/* PAI-VIN V0.3.2: network-first for HTML and application code; offline fallback.
   Avoid cache-first updates trapping Safari on an old release. */
const CACHE='pai-vin-v032-20261101';
const CORE=['./','./index.html','./app.js','./storage.js','./vision.js','./auto-lift.js','./manifest.json'];
self.addEventListener('install',event=>{
  event.waitUntil((async()=>{
    const cache=await caches.open(CACHE);
    await cache.addAll(CORE.map(url=>new Request(url,{cache:'reload'})));
    await self.skipWaiting();
  })());
});
self.addEventListener('activate',event=>{
  event.waitUntil((async()=>{
    const keys=await caches.keys();
    await Promise.all(keys.filter(k=>k.startsWith('pai-vin-')&&k!==CACHE).map(k=>caches.delete(k)));
    await self.clients.claim();
  })());
});
self.addEventListener('fetch',event=>{
  const req=event.request;
  if(req.method!=='GET'||new URL(req.url).origin!==self.location.origin)return;
  event.respondWith((async()=>{
    const cache=await caches.open(CACHE);
    const url=new URL(req.url);
    const isAppAsset=req.mode==='navigate'||/\.(?:html|js|json)$/.test(url.pathname);
    if(isAppAsset){
      try{
        const fresh=await fetch(new Request(req,{cache:'no-store'}));
        if(fresh.ok)await cache.put(req,fresh.clone()).catch(()=>{});
        return fresh;
      }catch(e){
        const hit=await caches.match(req,{ignoreSearch:true});
        if(hit)return hit;
        if(req.mode==='navigate'){const shell=await cache.match('./index.html');if(shell)return shell}
        throw e;
      }
    }
    return (await cache.match(req))||fetch(req);
  })());
});
