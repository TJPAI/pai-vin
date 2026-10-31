/* IndexedDB-backed local visual anchor store, V0.3.1.
   Keeps legacy localStorage data intact unless a successful migration is confirmed. */
(function(global){
'use strict';
const DB='pai-vin-data',STORE='anchors',LEGACY='pai-vin-anchors';
let dbPromise=null;
function open(){
 if(dbPromise)return dbPromise;
 dbPromise=new Promise((resolve,reject)=>{
  if(!global.indexedDB)return reject(new Error('浏览器不支持 IndexedDB'));
  let request=global.indexedDB.open(DB,1);
  request.onupgradeneeded=()=>{const db=request.result;if(!db.objectStoreNames.contains(STORE))db.createObjectStore(STORE,{keyPath:'id'})};
  request.onsuccess=()=>resolve(request.result);
  request.onerror=()=>reject(request.error||new Error('IndexedDB 打开失败'));
  request.onblocked=()=>reject(new Error('IndexedDB 被其他标签页阻塞'));
 }).catch(e=>{dbPromise=null;throw e});
 return dbPromise;
}
async function all(){
 const db=await open();
 return new Promise((resolve,reject)=>{
  const tx=db.transaction(STORE,'readonly'),req=tx.objectStore(STORE).getAll();
  req.onsuccess=()=>resolve(req.result||[]);
  req.onerror=()=>reject(req.error||new Error('锚点读取失败'));
 });
}
async function put(anchor){
 const db=await open();
 return new Promise((resolve,reject)=>{
  const tx=db.transaction(STORE,'readwrite');
  tx.objectStore(STORE).put(anchor);
  tx.oncomplete=()=>resolve(true);
  tx.onabort=()=>reject(tx.error||new Error('锚点写入事务中止'));
  tx.onerror=()=>reject(tx.error||new Error('锚点写入失败'));
 });
}
async function clear(){
 const db=await open();
 return new Promise((resolve,reject)=>{
  const tx=db.transaction(STORE,'readwrite');tx.objectStore(STORE).clear();
  tx.oncomplete=()=>resolve();tx.onabort=()=>reject(tx.error||new Error('清除失败'));
 });
}
function legacy(){
 try{const a=JSON.parse(global.localStorage.getItem(LEGACY)||'[]');return Array.isArray(a)?a:[]}catch{return []}
}
async function load(){
 let old=legacy(),stored=await all(),ids=new Set(stored.map(a=>String(a.id)));
 let migrated=0,errors=[];
 for(const a of old){
  if(ids.has(String(a.id)))continue;
  try{await put(a);stored.push(a);ids.add(String(a.id));migrated++}catch(e){errors.push(e.message);break}
 }
 // Deliberately do not delete legacy data; avoids silent loss if an interrupted migration occurs.
 return {anchors:stored.sort((a,b)=>a.id-b.id),migrated,warning:errors.join('; ')};
}
global.VINStore={load,put,clear,legacy};
})(window);