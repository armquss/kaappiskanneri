/* Offline-välimuisti: sovellus ja tunnistusmoottori toimivat ilman verkkoa. */
const VERSION = 'kaappi-v4';
const FILES = ['./', 'index.html', 'style.css', 'app.js', 'worker.js', 'vision.js', 'opencv.js', 'manifest.webmanifest', 'manifest.webmanifest?v=2',
  'icons/icon-192.png', 'icons/icon-512.png', 'icons/icon-maskable-512.png'];

self.addEventListener('install', e => {
  // cache: 'reload' ohittaa selaimen HTTP-välimuistin, jotta uusi versio ei täyty vanhoilla tiedostoilla
  e.waitUntil(caches.open(VERSION).then(c => c.addAll(FILES.map(f => new Request(f, { cache: 'reload' })))).then(() => self.skipWaiting()));
});
self.addEventListener('activate', e => {
  e.waitUntil(caches.keys().then(ks => Promise.all(ks.filter(k => k !== VERSION).map(k => caches.delete(k)))).then(() => self.clients.claim()));
});
self.addEventListener('fetch', e => {
  if (e.request.method !== 'GET') return;
  const url = new URL(e.request.url);
  if (url.origin !== location.origin) return;
  e.respondWith(caches.open(VERSION).then(async c => {
    const hit = await c.match(e.request, { ignoreSearch: !url.search.includes('v=') });
    if (hit) return hit;
    try { const r = await fetch(e.request); if (r.ok) c.put(e.request, r.clone()); return r; }
    catch (err) { return new Response('Offline', { status: 503 }); }
  }));
});
