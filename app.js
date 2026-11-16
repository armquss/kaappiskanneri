/* Kaappiskanneri – sovelluslogiikka */
'use strict';

// ---------------------------------------------------------------- apuvälineet
const $ = (s, el = document) => el.querySelector(s);
const uid = () => Date.now().toString(36) + Math.random().toString(36).slice(2, 7);
const esc = s => String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const hueOf = id => { let h = 0; for (const c of String(id)) h = (h * 31 + c.charCodeAt(0)) >>> 0; return h % 360; };
const colOf = (id, a = 1) => `hsla(${hueOf(id)}, 75%, 60%, ${a})`;
const sleep = ms => new Promise(r => setTimeout(r, ms));
let toastTimer = null;
function toast(msg, ms = 2800) {
  const t = $('#toast'); t.textContent = msg; t.classList.remove('hidden');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.add('hidden'), ms);
}
function vibrate(ms) { if (S.settings.vibrate && navigator.vibrate && (!navigator.userActivation || navigator.userActivation.hasBeenActive)) try { navigator.vibrate(ms); } catch (e) {} }
const norm = s => String(s || '').toLocaleLowerCase('fi').trim();

// ---------------------------------------------------------------- tietokanta
const DB = {
  db: null,
  open() {
    return new Promise((res, rej) => {
      const r = indexedDB.open('kaappiskanneri', 1);
      r.onupgradeneeded = () => {
        const d = r.result;
        d.createObjectStore('cabinets', { keyPath: 'id' });
        d.createObjectStore('views', { keyPath: 'id' });
        d.createObjectStore('kv');
      };
      r.onsuccess = () => { this.db = r.result; res(); };
      r.onerror = () => rej(r.error);
    });
  },
  _tx(store, mode, fn) {
    return new Promise((res, rej) => {
      const t = this.db.transaction(store, mode);
      const req = fn(t.objectStore(store));
      t.oncomplete = () => res(req ? req.result : undefined);
      t.onerror = () => rej(t.error); t.onabort = () => rej(t.error);
    });
  },
  all(s) { return this._tx(s, 'readonly', o => o.getAll()); },
  get(s, k) { return this._tx(s, 'readonly', o => o.get(k)); },
  put(s, v, k) { return this._tx(s, 'readwrite', o => k === undefined ? o.put(v) : o.put(v, k)); },
  del(s, k) { return this._tx(s, 'readwrite', o => o.delete(k)); },
  clear(s) { return this._tx(s, 'readwrite', o => o.clear()); }
};

// ---------------------------------------------------------------- tila
const S = {
  cabinets: new Map(), // id -> {id, name, items:[{id,text}], updated}
  views: new Map(),    // id -> {id, name, blob, thumb, w, h, regions:[{cabinetId, poly}], created, q?}
  settings: { quality: 'balanced', vibrate: true, debug: false },
  target: null,        // {cabinetId, query}
};
const saveSettings = () => DB.put('kv', S.settings, 'settings');
const cabName = id => (S.cabinets.get(id) || {}).name || 'Nimetön kaappi';
function sortedCabinets() { return [...S.cabinets.values()].sort((a, b) => a.name.localeCompare(b.name, 'fi', { numeric: true })); }
function sortedViews() { return [...S.views.values()].sort((a, b) => a.created - b.created); }
async function saveCabinet(c) { c.updated = Date.now(); S.cabinets.set(c.id, c); await DB.put('cabinets', c); }
async function saveView(v) { S.views.set(v.id, v); await DB.put('views', v); }
function viewsOfCabinet(id) { return sortedViews().filter(v => v.regions.some(r => r.cabinetId === id)); }

// ---------------------------------------------------------------- tunnistusmoottori (worker)
const APP_VERSION = '1.1';
const PRESETS = {
  fast: { refMax: 1200, refFeatures: 2500, frameFeatures: 800, frameMax: 480 },
  balanced: { refMax: 1600, refFeatures: 4000, frameFeatures: 1200, frameMax: 640 },
  precise: { refMax: 2000, refFeatures: 6000, frameFeatures: 1600, frameMax: 800 }
};
const Engine = {
  det: null, trk: null, seq: 0, pending: new Map(), ready: false, status: 'Ladataan…',
  params() { return PRESETS[S.settings.quality] || PRESETS.balanced; },
  _worker(role) {
    const w = new Worker('worker.js');
    w.onmessage = e => {
      const m = e.data;
      if (m.type === 'update') { Scan.onResult(m.res, true); return; }
      const p = this.pending.get(m.id);
      if (!p) return; this.pending.delete(m.id);
      m.error ? p.rej(new Error(m.error)) : p.res(m.result);
    };
    w.onerror = e => { this.status = 'Virhe (' + role + '): ' + (e.message || 'worker'); updateEngineInfo(); };
    return w;
  },
  _call(w, type, data = {}, transfer = []) {
    if (!w) return Promise.reject(new Error('no worker'));
    const id = ++this.seq;
    return new Promise((res, rej) => {
      this.pending.set(id, { res, rej });
      w.postMessage(Object.assign({ type, id }, data), transfer);
    });
  },
  call(type, data, transfer) { return this._call(this.det, type, data, transfer); },   // tunnistus
  tcall(type, data, transfer) { return this._call(this.trk, type, data, transfer); },  // seuranta
  async start() {
    this.ready = false; this.status = 'Ladataan tunnistusmoottoria…'; updateEngineInfo();
    this.det = this._worker('tunnistus'); this.trk = this._worker('seuranta');
    const p = this.params();
    await Promise.all([this.call('init', { role: 'detector', params: p }), this.tcall('init', { role: 'tracker', params: p })]);
    const ch = new MessageChannel();
    await Promise.all([this.call('connect', { port: ch.port1 }, [ch.port1]), this.tcall('connect', { port: ch.port2 }, [ch.port2])]);
    for (const v of sortedViews()) await this.loadView(v);
    this.ready = true; this.status = 'valmis'; updateEngineInfo();
  },
  async restart() {
    this.ready = false;
    if (this.det) this.det.terminate(); if (this.trk) this.trk.terminate();
    for (const p of this.pending.values()) p.rej(new Error('restart'));
    this.pending.clear(); this.det = this.trk = null;
    await this.start();
  },
  async loadView(v) {
    const img = await blobToImageData(v.blob, this.params().refMax);
    const r = await this.call('addView', { viewId: v.id, img, regions: v.regions }, [img.data.buffer]);
    v.ew = r.w; v.eh = r.h;
    await this.tcall('setView', { viewId: v.id, w: r.w, h: r.h, regions: v.regions });
    v.q = await this.call('quality', { viewId: v.id });
    return r;
  },
  setRegions(v) {
    if (!this.det) return;
    this.call('setRegions', { viewId: v.id, regions: v.regions }).catch(() => {});
    if (v.ew) this.tcall('setView', { viewId: v.id, w: v.ew, h: v.eh, regions: v.regions }).catch(() => {});
  },
  removeView(id) {
    if (this.det) this.call('removeView', { viewId: id }).catch(() => {});
    if (this.trk) this.tcall('removeView', { viewId: id }).catch(() => {});
  }
};
function updateEngineInfo() {
  const doors = sortedViews().reduce((n, v) => n + v.regions.length, 0);
  $('#engine-info').textContent = `Versio ${APP_VERSION} · tunnistusmoottori: ${Engine.status} · ${S.views.size} kartoituskuvaa · ${doors} merkittyä ovea · ${S.cabinets.size} kaappia`;
}

// ---------------------------------------------------------------- kuvat
async function loadBitmap(blob) {
  try { return await createImageBitmap(blob, { imageOrientation: 'from-image' }); }
  catch (e) {
    const url = URL.createObjectURL(blob);
    const img = new Image(); img.src = url; await img.decode(); URL.revokeObjectURL(url); return img;
  }
}
function canvasOf(w, h) { const c = document.createElement('canvas'); c.width = w; c.height = h; return c; }
const toBlob = (c, type = 'image/jpeg', q = 0.9) => new Promise(r => c.toBlob(r, type, q));
async function blobToImageData(blob, max) {
  const bmp = await loadBitmap(blob);
  const s = Math.min(1, max / Math.max(bmp.width, bmp.height));
  const w = Math.round(bmp.width * s), h = Math.round(bmp.height * s);
  const c = canvasOf(w, h), ctx = c.getContext('2d', { willReadFrequently: true });
  ctx.drawImage(bmp, 0, 0, w, h);
  const d = ctx.getImageData(0, 0, w, h);
  return { data: d.data, width: w, height: h };
}
async function prepPhoto(file) {
  const bmp = await loadBitmap(file);
  const s = Math.min(1, 2200 / Math.max(bmp.width, bmp.height));
  const w = Math.round(bmp.width * s), h = Math.round(bmp.height * s);
  const c = canvasOf(w, h); c.getContext('2d').drawImage(bmp, 0, 0, w, h);
  const t = canvasOf(240, Math.round(240 * h / w)); t.getContext('2d').drawImage(bmp, 0, 0, t.width, t.height);
  return { blob: await toBlob(c, 'image/jpeg', 0.9), thumb: await toBlob(t, 'image/jpeg', 0.8), w, h };
}
const thumbUrls = new Map();
function thumbUrl(v) {
  if (!thumbUrls.has(v.id)) thumbUrls.set(v.id, URL.createObjectURL(v.thumb || v.blob));
  return thumbUrls.get(v.id);
}

// ---------------------------------------------------------------- navigaatio
let currentScreen = 'scr-scan';
function showScreen(id) {
  currentScreen = id;
  document.querySelectorAll('.screen').forEach(s => s.classList.toggle('active', s.id === id));
  document.querySelectorAll('#tabs button').forEach(b => b.classList.toggle('active', b.dataset.scr === id));
  if (id === 'scr-scan') Scan.start(); else Scan.stop();
  if (id === 'scr-cabs') renderCabinets();
  if (id === 'scr-views') renderViews();
}
document.querySelectorAll('#tabs button').forEach(b => b.onclick = () => showScreen(b.dataset.scr));

// ---------------------------------------------------------------- alapaneeli
let sheetClose = null;
function openSheet(html, onClose) {
  $('#sheet').innerHTML = '<div class="grab"></div><button class="sheet-x link">Valmis</button>' + html;
  $('#sheet .sheet-x').onclick = closeSheet;
  $('#sheet').classList.remove('hidden'); $('#sheet-bg').classList.remove('hidden');
  sheetClose = onClose || null;
  return $('#sheet');
}
function closeSheet() {
  if ($('#sheet').classList.contains('hidden')) return;
  $('#sheet').classList.add('hidden'); $('#sheet-bg').classList.add('hidden');
  const f = sheetClose; sheetClose = null; if (f) f();
}
$('#sheet-bg').onclick = closeSheet;

// ---------------------------------------------------------------- SKANNERI
const Scan = {
  stream: null, running: false, busy: false, last: null, shownCab: null, shownLive: null, frameId: 0,
  pc: null, pctx: null, torchOn: false, fps: 0, lastResAt: 0, disp: new Map(), dispView: null, lastDraw: 0, bitmapOk: true,
  async start() {
    if (this.running) return;
    if (!S.views.size) { this.showMsg('<div><b>Aloita kartoittamalla kaappiseinä</b><p class="muted">Ota kuva seinästä ja merkitse ovet. Sen jälkeen skanneri tunnistaa, mitä kaappia osoitat.</p><button class="btn" id="msg-go">Kartoitukseen</button></div>'); $('#msg-go').onclick = () => showScreen('scr-views'); setStatus('lost', 'Ei kartoitusta'); return; }
    if (!window.isSecureContext || !navigator.mediaDevices) { this.showMsg('<div><b>Kamera vaatii HTTPS-yhteyden.</b><p class="muted">Avaa sovellus https-osoitteesta (esim. GitHub Pages).</p></div>'); return; }
    this.running = true;
    try {
      this.stream = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 720 }, frameRate: { ideal: 30 } }
      });
    } catch (e) {
      this.running = false;
      const denied = e && (e.name === 'NotAllowedError' || e.name === 'SecurityError');
      this.showMsg(`<div><b>${denied ? 'Kameran käyttö on estetty.' : 'Kameraa ei saatu käyttöön.'}</b><p class="muted">${denied ? 'Salli kamera selaimen sivustoasetuksista ja yritä uudelleen.' : esc(e && e.message || '')}</p><button class="btn" id="msg-retry">Yritä uudelleen</button></div>`);
      $('#msg-retry').onclick = () => { this.hideMsg(); this.start(); };
      return;
    }
    if (!this.running) { this.stream.getTracks().forEach(t => t.stop()); this.stream = null; return; }
    this.hideMsg();
    const v = $('#video'); v.srcObject = this.stream;
    try { await v.play(); } catch (e) {}
    const track = this.stream.getVideoTracks()[0];
    const caps = track.getCapabilities ? track.getCapabilities() : {};
    $('#btn-torch').classList.toggle('hidden', !caps.torch);
    if (caps.focusMode && caps.focusMode.includes('continuous')) track.applyConstraints({ advanced: [{ focusMode: 'continuous' }] }).catch(() => {});
    this.torchOn = false; $('#btn-torch').classList.remove('on');
    this.disp.clear(); this.last = null;
    if (Engine.trk) Engine.tcall('reset').catch(() => {});
    this.captureLoop(); this.drawLoop();
  },
  stop() {
    this.running = false;
    if (this.stream) { this.stream.getTracks().forEach(t => t.stop()); this.stream = null; }
    $('#video').srcObject = null;
  },
  showMsg(html) { const m = $('#cam-msg'); m.innerHTML = html; m.classList.remove('hidden'); },
  hideMsg() { $('#cam-msg').classList.add('hidden'); },
  // Kuvien syöttö seurantaan kameran omassa tahdissa; uusi kuva vasta kun edellinen on käsitelty.
  captureLoop() {
    if (!this.running) return;
    const v = $('#video');
    const next = () => this.captureLoop();
    if (!this.busy && Engine.ready && v.readyState >= 2 && v.videoWidth) this.sendFrame(v);
    if (v.requestVideoFrameCallback) v.requestVideoFrameCallback(next); else requestAnimationFrame(next);
  },
  async sendFrame(v) {
    this.busy = true;
    try {
      const max = Engine.params().frameMax;
      const s = Math.min(1, max / Math.max(v.videoWidth, v.videoHeight));
      const w = Math.round(v.videoWidth * s), h = Math.round(v.videoHeight * s);
      const id = ++this.frameId, t = performance.now(), debug = S.settings.debug;
      let r;
      if (this.bitmapOk && window.createImageBitmap && window.OffscreenCanvas) {
        // kuva siirtyy säikeelle ilman pääsäikeen pikselikopiota
        try {
          const bmp = await createImageBitmap(v, { resizeWidth: w, resizeHeight: h, resizeQuality: 'low' });
          r = await Engine.tcall('frame', { bmp, fid: id, t, debug }, [bmp]);
        } catch (e) { if (String(e.message) === 'restart') throw e; this.bitmapOk = false; }
      }
      if (!r) {
        if (!this.pc || this.pc.width !== w || this.pc.height !== h) { this.pc = canvasOf(w, h); this.pctx = this.pc.getContext('2d', { willReadFrequently: true }); }
        this.pctx.drawImage(v, 0, 0, w, h);
        const d = this.pctx.getImageData(0, 0, w, h);
        r = await Engine.tcall('frame', { img: { data: d.data, width: w, height: h }, fid: id, t, debug }, [d.data.buffer]);
      }
      const now = performance.now();
      if (this.lastResAt) this.fps = this.fps * 0.85 + 0.15 * (1000 / Math.max(1, now - this.lastResAt));
      this.lastResAt = now;
      if (this.running) this.onResult(r);
    } catch (e) {
      if (String(e.message) !== 'restart') console.warn(e);
      await sleep(200);
    } finally { this.busy = false; }
  },
  onResult(r, isAnchor) {
    if (!r) return;
    if (isAnchor && this.last) { r.debug = this.last.debug; }
    this.last = r;
    const vn = r.viewId && S.views.get(r.viewId);
    if (r.mode === 'detect') setStatus('detect', 'Tunnistettu' + (vn ? ' · ' + vn.name : ''));
    else if (r.mode === 'track') setStatus('track', 'Seuranta');
    else setStatus('lost', 'Etsitään…');
    const live = !!(r.cabinetId && r.mode !== 'lost');
    if (r.cabinetId && r.cabinetId !== this.shownCab) { this.shownCab = r.cabinetId; this.shownLive = live; vibrate(15); renderCard(); }
    else if (live !== this.shownLive) { this.shownLive = live; renderCard(); }
    if (S.settings.debug) {
      const d = r.debug || {}, a = r.det;
      $('#debug-box').textContent =
        `${r.mode}  seuranta ${d.ms ?? '-'} ms (${d.via || '-'} ${d.inliers ?? 0})  ${this.fps.toFixed(1)} fps\n` +
        `ankkuri ${d.anchorAge != null ? (d.anchorAge / 1000).toFixed(1) + ' s sitten' : '-'}  raaka ${r.rawCabinetId ? cabName(r.rawCabinetId) : '-'}${r.onEdge ? ' (reuna)' : ''}\n` +
        (a ? `tunnistus ${a.ms} ms ${a.ok ? a.via + ' ' + a.inliers : 'ei osumaa'}${a.ok ? (a.applied ? ' ✓ +' + a.steps : ' ✗ ' + a.reason) : ''}\n` +
          (a.rejects && a.rejects.length && !a.ok ? '  ' + a.rejects.join(', ').replace(/[\w-]{8,}:/g, k => ((S.views.get(k.slice(0, -1)) || {}).name || 'näkymä') + ':') : '') : 'tunnistus –');
    }
    updateTargetBar();
  },
  // video (object-fit: cover) -> näytön koordinaatit
  mapper() {
    const v = $('#video'), c = $('#overlay');
    const W = c.clientWidth, H = c.clientHeight, vw = v.videoWidth || 1, vh = v.videoHeight || 1;
    const s = Math.max(W / vw, H / vh), dx = (W - vw * s) / 2, dy = (H - vh * s) / 2;
    return ([x, y]) => [x * vw * s + dx, y * vh * s + dy];
  },
  // Pehmennys: piirretyt alueet liukuvat kohti uusinta sijaintia (ei nykimistä tulosten välillä)
  smoothed(r, dt) {
    if (!r || r.mode === 'lost') { this.disp.clear(); return []; }
    if (this.dispView !== r.viewId) { this.disp.clear(); this.dispView = r.viewId; }
    const k = 1 - Math.exp(-dt / 45), seen = new Set(), out = [];
    for (const o of r.overlay) {
      seen.add(o.regionIndex);
      let d = this.disp.get(o.regionIndex);
      const jump = d && o.poly.some((p, i) => Math.hypot(p[0] - d.poly[i][0], p[1] - d.poly[i][1]) > 0.35);
      if (!d || jump) { d = { poly: o.poly.map(p => p.slice()) }; this.disp.set(o.regionIndex, d); }
      else d.poly.forEach((p, i) => { p[0] += (o.poly[i][0] - p[0]) * k; p[1] += (o.poly[i][1] - p[1]) * k; });
      out.push({ cabinetId: o.cabinetId, poly: d.poly });
    }
    for (const key of [...this.disp.keys()]) if (!seen.has(key)) this.disp.delete(key);
    return out;
  },
  drawLoop() {
    if (!this.running) return;
    const now = performance.now(), dt = this.lastDraw ? Math.min(100, now - this.lastDraw) : 16;
    this.lastDraw = now;
    this.draw(dt);
    requestAnimationFrame(() => this.drawLoop());
  },
  draw(dt) {
    const c = $('#overlay'), dpr = window.devicePixelRatio || 1;
    const W = c.clientWidth, H = c.clientHeight;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const g = c.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    const r = this.last;
    const polys = this.smoothed(r, dt || 16);
    if (!polys.length) return;
    const m = this.mapper();
    const tgt = S.target && S.target.cabinetId;
    let tgtCentroid = null;
    for (const o of polys) {
      const pts = o.poly.map(m);
      const isCur = o.cabinetId === r.cabinetId, isTgt = o.cabinetId === tgt;
      g.beginPath(); pts.forEach(([x, y], i) => i ? g.lineTo(x, y) : g.moveTo(x, y)); g.closePath();
      if (isTgt) { g.fillStyle = 'rgba(251,191,36,.28)'; g.fill(); }
      else if (isCur) { g.fillStyle = 'rgba(45,212,191,.22)'; g.fill(); }
      g.lineWidth = isTgt ? 4 : isCur ? 3 : 1.5;
      g.strokeStyle = isTgt ? '#fbbf24' : isCur ? '#2dd4bf' : 'rgba(255,255,255,.55)';
      g.setLineDash(r.mode === 'track' && !isCur && !isTgt ? [6, 6] : []);
      g.stroke(); g.setLineDash([]);
      const cx = pts.reduce((a, p) => a + p[0], 0) / pts.length, cy = pts.reduce((a, p) => a + p[1], 0) / pts.length;
      if (isTgt) tgtCentroid = [cx, cy];
      if (cx > -50 && cx < W + 50 && cy > -20 && cy < H + 20) label(g, (isTgt ? '★ ' : '') + cabName(o.cabinetId), cx, cy, isTgt ? '#fbbf24' : isCur ? '#2dd4bf' : 'rgba(0,0,0,.55)', isTgt || isCur ? '#04201c' : '#fff');
    }
    if (tgtCentroid && r.cabinetId !== tgt) { // nuoli vain, jos tähtäin ei jo ole kohdekaapissa
      const [x, y] = tgtCentroid, mg = 40;
      if (x < mg || x > W - mg || y < mg + 60 || y > H - mg) {
        const cx = W / 2, cy = H / 2, ang = Math.atan2(y - cy, x - cx);
        const t = Math.min(Math.abs((W / 2 - mg) / Math.cos(ang)), Math.abs((H / 2 - mg - 30) / Math.sin(ang)));
        const ax = cx + Math.cos(ang) * t, ay = cy + Math.sin(ang) * t;
        g.save(); g.translate(ax, ay); g.rotate(ang);
        g.fillStyle = '#fbbf24'; g.beginPath(); g.moveTo(18, 0); g.lineTo(-12, -14); g.lineTo(-6, 0); g.lineTo(-12, 14); g.closePath(); g.fill();
        g.restore();
      }
    }
    if (S.settings.debug && r.debug && r.debug.pts) {
      g.fillStyle = r.mode === 'detect' ? '#2dd4bf' : '#fbbf24';
      for (let i = 0; i < r.debug.pts.length; i += 2) { const [x, y] = m([r.debug.pts[i], r.debug.pts[i + 1]]); g.fillRect(x - 1.5, y - 1.5, 3, 3); }
    }
  }
};
function label(g, text, x, y, bg, fg) {
  g.font = '600 13px system-ui, sans-serif';
  const w = g.measureText(text).width + 14;
  g.fillStyle = bg; roundRect(g, x - w / 2, y - 12, w, 24, 12); g.fill();
  g.fillStyle = fg; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(text, x, y + 0.5);
}
function roundRect(g, x, y, w, h, r) { g.beginPath(); g.moveTo(x + r, y); g.arcTo(x + w, y, x + w, y + h, r); g.arcTo(x + w, y + h, x, y + h, r); g.arcTo(x, y + h, x, y, r); g.arcTo(x, y, x + w, y, r); g.closePath(); }
function setStatus(cls, text) { const p = $('#status'); p.className = 'pill ' + cls; p.querySelector('span').textContent = text; }

function renderCard() {
  const card = $('#result-card'), id = Scan.shownCab, c = id && S.cabinets.get(id);
  const live = !!(Scan.last && Scan.last.cabinetId === id && Scan.last.mode !== 'lost');
  if (!c) {
    card.className = 'card idle';
    $('#rc-title').textContent = 'Osoita kaappia';
    $('#rc-sub').textContent = 'Aloita hieman kauempaa, niin että ympäristöä näkyy. Lähemmäs voi sitten siirtyä.';
    $('#rc-items').innerHTML = ''; $('#rc-edit').classList.add('hidden');
    return;
  }
  card.className = 'card' + (live ? ' hit' : '');
  $('#rc-title').textContent = c.name;
  $('#rc-sub').textContent = live ? `${c.items.length} ${c.items.length === 1 ? 'tavara' : 'tavaraa'}` : 'Viimeksi tunnistettu';
  const q = S.target && S.target.query ? norm(S.target.query) : '';
  $('#rc-items').innerHTML = c.items.length
    ? c.items.map(i => `<li class="${q && norm(i.text).includes(q) ? 'match' : ''}">${esc(i.text)}</li>`).join('')
    : '<li class="none">Kaappi on tyhjä – lisää sisältö Muokkaa-napista</li>';
  $('#rc-edit').classList.remove('hidden');
}
$('#rc-edit').onclick = () => Scan.shownCab && openCabinet(Scan.shownCab);
$('#result-card').addEventListener('click', e => { if (e.target.closest('button')) return; if (Scan.shownCab) openCabinet(Scan.shownCab); });

function updateTargetBar() {
  const bar = $('#target-bar');
  if (!S.target) { bar.classList.add('hidden'); return; }
  bar.classList.remove('hidden');
  let txt = esc(cabName(S.target.cabinetId));
  const r = Scan.last, vs = viewsOfCabinet(S.target.cabinetId);
  if (!vs.length) txt += ' <span class="muted">(ei merkitty kartoituskuvaan)</span>';
  else if (r && r.mode !== 'lost' && r.viewId && !vs.some(v => v.id === r.viewId)) txt += ` <span>– katso: ${esc(vs[0].name)}</span>`;
  $('#target-name').innerHTML = txt;
}
$('#target-clear').onclick = () => { S.target = null; updateTargetBar(); renderCard(); };
function setTarget(cabinetId, query) { S.target = { cabinetId, query: query || '' }; closeSheet(); showScreen('scr-scan'); updateTargetBar(); }

$('#btn-debug').onclick = () => {
  S.settings.debug = !S.settings.debug; saveSettings();
  $('#btn-debug').classList.toggle('on', S.settings.debug);
  $('#debug-box').classList.toggle('hidden', !S.settings.debug);
};
$('#btn-torch').onclick = async () => {
  const t = Scan.stream && Scan.stream.getVideoTracks()[0]; if (!t) return;
  Scan.torchOn = !Scan.torchOn;
  try { await t.applyConstraints({ advanced: [{ torch: Scan.torchOn }] }); } catch (e) { Scan.torchOn = false; }
  $('#btn-torch').classList.toggle('on', Scan.torchOn);
};
document.addEventListener('visibilitychange', () => {
  if (document.hidden) Scan.stop(); else if (currentScreen === 'scr-scan' && $('#editor').classList.contains('hidden')) Scan.start();
});

// ---------------------------------------------------------------- KAAPIT
function renderCabinets() {
  const list = sortedCabinets();
  $('#cabs-empty').classList.toggle('hidden', list.length > 0);
  $('#cab-list').innerHTML = list.map(c => {
    const nv = viewsOfCabinet(c.id).length;
    const prev = c.items.slice(0, 6).map(i => i.text).join(', ') || 'tyhjä';
    return `<div class="li" data-id="${c.id}"><span class="sw" style="background:${colOf(c.id)}"></span>
      <div class="grow"><div class="t">${esc(c.name)}</div><div class="s">${c.items.length} kpl · ${esc(prev)}</div>
      ${nv ? '' : '<div class="s" style="color:var(--amber)">Ei merkitty kartoituskuvaan</div>'}</div></div>`;
  }).join('');
  $('#cab-list').querySelectorAll('.li').forEach(el => el.onclick = () => openCabinet(el.dataset.id));
  renderSearch();
}
function renderSearch() {
  const q = norm($('#search').value), box = $('#search-results');
  $('#cab-list').classList.toggle('hidden', !!q);
  if (!q) { box.innerHTML = ''; return; }
  const hits = [];
  for (const c of sortedCabinets()) {
    const items = c.items.filter(i => norm(i.text).includes(q));
    if (items.length || norm(c.name).includes(q)) hits.push({ c, items });
  }
  const hl = t => { const i = norm(t).indexOf(q); return i < 0 ? esc(t) : esc(t.slice(0, i)) + '<mark>' + esc(t.slice(i, i + q.length)) + '</mark>' + esc(t.slice(i + q.length)); };
  box.innerHTML = hits.length
    ? hits.map(({ c, items }) => `<div class="li" data-id="${c.id}"><span class="sw" style="background:${colOf(c.id)}"></span>
        <div class="grow"><div class="t">${hl(c.name)}</div><div class="s">${items.length ? items.map(i => hl(i.text)).join(', ') : c.items.length + ' tavaraa'}</div></div>
        <button class="btn small" data-show="${c.id}">Näytä</button></div>`).join('')
    : `<div class="empty muted">Ei osumia haulle ”${esc($('#search').value)}”</div>`;
  box.querySelectorAll('[data-show]').forEach(b => b.onclick = e => { e.stopPropagation(); setTarget(b.dataset.show, $('#search').value); });
  box.querySelectorAll('.li').forEach(el => el.onclick = () => openCabinet(el.dataset.id));
}
$('#search').oninput = renderSearch;
$('#btn-new-cab').onclick = async () => {
  const c = { id: uid(), name: `Kaappi ${S.cabinets.size + 1}`, items: [] };
  await saveCabinet(c); openCabinet(c.id, true);
};

function openCabinet(id, focusName) {
  const c = S.cabinets.get(id); if (!c) return;
  const wasRunning = Scan.running;
  const vs = viewsOfCabinet(id);
  const el = openSheet(`
    <input id="cb-name" class="field" style="font-size:18px;font-weight:600" value="${esc(c.name)}" maxlength="50">
    <ul class="ed-items" id="cb-items"></ul>
    <div class="row"><input id="cb-add" class="field" placeholder="Lisää tavara (Enter)" enterkeyhint="done"><button id="cb-add-btn" class="btn">Lisää</button></div>
    <p class="muted small">${vs.length ? 'Merkitty kuvaan: ' + vs.map(v => esc(v.name)).join(', ') : 'Kaappia ei ole vielä merkitty kartoituskuvaan.'} Voit liittää monta tavaraa kerralla rivinvaihdoin.</p>
    <div class="row end">
      <button id="cb-show" class="btn ghost" ${vs.length ? '' : 'disabled'}>Näytä kamerassa</button>
      <button id="cb-del" class="btn small danger">Poista kaappi</button>
    </div>`, () => { renderCard(); if (currentScreen === 'scr-cabs') renderCabinets(); });
  const save = () => saveCabinet(c);
  const renderItems = () => {
    $('#cb-items', el).innerHTML = c.items.map(i => `<li data-id="${i.id}"><input value="${esc(i.text)}" maxlength="80"><button class="x" aria-label="Poista">×</button></li>`).join('')
      || '<li class="muted" style="border:0">Ei vielä tavaroita</li>';
    $('#cb-items', el).querySelectorAll('li[data-id]').forEach(li => {
      const it = c.items.find(i => i.id === li.dataset.id);
      li.querySelector('input').onchange = e => { const t = e.target.value.trim(); if (t) it.text = t; else c.items = c.items.filter(i => i !== it); save(); renderItems(); };
      li.querySelector('.x').onclick = () => { c.items = c.items.filter(i => i !== it); save(); renderItems(); };
    });
  };
  const add = () => {
    const raw = $('#cb-add', el).value; const parts = raw.split(/\r?\n|;/).map(s => s.trim()).filter(Boolean);
    if (!parts.length) return;
    parts.forEach(t => c.items.push({ id: uid(), text: t }));
    $('#cb-add', el).value = ''; save(); renderItems(); $('#cb-add', el).focus();
  };
  $('#cb-add', el).onkeydown = e => { if (e.key === 'Enter') { e.preventDefault(); add(); } };
  $('#cb-add', el).addEventListener('paste', e => {
    const t = (e.clipboardData || window.clipboardData).getData('text');
    if (/\n/.test(t)) { e.preventDefault(); $('#cb-add', el).value = t; add(); }
  });
  $('#cb-add-btn', el).onclick = add;
  $('#cb-name', el).onchange = e => { c.name = e.target.value.trim() || c.name; save(); };
  $('#cb-show', el).onclick = () => setTarget(id, '');
  $('#cb-del', el).onclick = async () => {
    if (!confirm(`Poistetaanko ”${c.name}” ja sen merkinnät kartoituskuvista?`)) return;
    S.cabinets.delete(id); await DB.del('cabinets', id);
    for (const v of S.views.values()) {
      const n = v.regions.length; v.regions = v.regions.filter(r => r.cabinetId !== id);
      if (v.regions.length !== n) { await saveView(v); Engine.setRegions(v); }
    }
    if (Scan.shownCab === id) Scan.shownCab = null;
    if (S.target && S.target.cabinetId === id) S.target = null;
    closeSheet(); updateEngineInfo();
  };
  renderItems();
  if (focusName) setTimeout(() => { $('#cb-name', el).focus(); $('#cb-name', el).select(); }, 50);
  else if (!c.items.length) setTimeout(() => $('#cb-add', el).focus(), 50);
}

// valitse kaappi oven merkintään
function pickCabinet(title) {
  return new Promise(resolve => {
    let done = false;
    const finish = v => { if (done) return; done = true; resolve(v); closeSheet(); };
    const list = sortedCabinets();
    const el = openSheet(`<h3>${esc(title)}</h3>
      <div class="row"><input id="pk-new" class="field" placeholder="Uuden kaapin nimi" value="Kaappi ${S.cabinets.size + 1}"><button id="pk-create" class="btn">Luo</button></div>
      ${list.length ? '<p class="muted small">…tai valitse olemassa oleva:</p>' : ''}
      ${list.map(c => `<button class="choice" data-id="${c.id}"><span style="color:${colOf(c.id)}">●</span> ${esc(c.name)}<small>${c.items.length} tavaraa${viewsOfCabinet(c.id).length ? '' : ' · ei vielä merkitty'}</small></button>`).join('')}`,
      () => { if (!done) { done = true; resolve(null); } });
    el.querySelectorAll('.choice').forEach(b => b.onclick = () => finish(b.dataset.id));
    const create = async () => {
      const name = $('#pk-new', el).value.trim(); if (!name) return;
      const c = { id: uid(), name, items: [] }; await saveCabinet(c); finish(c.id);
    };
    $('#pk-create', el).onclick = create;
    $('#pk-new', el).onkeydown = e => { if (e.key === 'Enter') create(); };
    setTimeout(() => { const i = $('#pk-new', el); i.focus(); i.select(); }, 50);
  });
}

// ---------------------------------------------------------------- KARTOITUS
function qualityText(q) {
  if (!q) return ['', ''];
  return { good: ['good', 'Hyvä tunnistettavuus'], ok: ['ok', 'Kohtalainen tunnistettavuus'], poor: ['poor', 'Heikko tunnistettavuus'] }[q.level];
}
function renderViews() {
  const list = sortedViews();
  $('#views-empty').classList.toggle('hidden', list.length > 0);
  $('#view-list').innerHTML = list.map(v => {
    const [cls, txt] = qualityText(v.q);
    return `<div class="li" data-id="${v.id}"><img src="${thumbUrl(v)}" alt="">
      <div class="grow"><div class="t">${esc(v.name)}</div><div class="s">${v.regions.length} ${v.regions.length === 1 ? 'ovi' : 'ovea'}</div>
      ${txt ? `<span class="badge ${cls}">${txt}</span>` : ''}</div></div>`;
  }).join('');
  $('#view-list').querySelectorAll('.li').forEach(el => el.onclick = () => Editor.open(el.dataset.id));
  updateEngineInfo();
}
async function addPhoto(file) {
  if (!file) return;
  toast('Käsitellään kuvaa…', 8000);
  try {
    const p = await prepPhoto(file);
    const v = { id: uid(), name: `Seinä ${S.views.size + 1}`, blob: p.blob, thumb: p.thumb, w: p.w, h: p.h, regions: [], created: Date.now() };
    await saveView(v);
    if (Engine.det) await Engine.loadView(v);
    updateEngineInfo();
    const q = v.q;
    if (q && q.level === 'poor') toast('Kuvassa on vähän yksityiskohtia – tunnistus voi olla epävarmaa. Kokeile kuvata kauempaa ja paremmassa valossa.', 6000);
    else toast('Kuva lisätty. Merkitse nyt ovet.');
    Editor.open(v.id);
  } catch (e) { toast('Kuvan käsittely epäonnistui: ' + e.message, 5000); }
}
$('#inp-photo').onchange = e => { addPhoto(e.target.files[0]); e.target.value = ''; };
$('#inp-gallery').onchange = e => { addPhoto(e.target.files[0]); e.target.value = ''; };

// ---------------------------------------------------------------- NÄKYMÄEDITORI
const Editor = {
  v: null, img: null, s: 1, tx: 0, ty: 0, fitS: 1, sel: -1, adding: false, drag: null, ptrs: new Map(), pinch: null,
  async open(id) {
    const v = S.views.get(id); if (!v) return;
    this.v = v; this.sel = -1; this.adding = false; this.drag = null; this.ptrs.clear(); this.pinch = null;
    Scan.stop();
    $('#editor').classList.remove('hidden');
    $('#ed-name').value = v.name;
    this.img = await loadBitmap(v.blob);
    this.fit(); this.ui(); this.draw();
  },
  close() {
    $('#editor').classList.add('hidden'); this.v = null; this.img = null;
    renderViews(); if (currentScreen === 'scr-scan') Scan.start();
  },
  canvas() { return $('#ed-canvas'); },
  fit() {
    const c = this.canvas(), dpr = window.devicePixelRatio || 1;
    c.width = Math.round(c.clientWidth * dpr); c.height = Math.round(c.clientHeight * dpr);
    const W = c.clientWidth, H = c.clientHeight, w = this.img.width, h = this.img.height;
    this.fitS = this.s = Math.min(W / w, H / h) * 0.96;
    this.tx = (W - w * this.s) / 2; this.ty = (H - h * this.s) / 2;
  },
  toScr(nx, ny) { return [nx * this.img.width * this.s + this.tx, ny * this.img.height * this.s + this.ty]; },
  toImg(x, y) { return [(x - this.tx) / this.s / this.img.width, (y - this.ty) / this.s / this.img.height]; },
  local(e) { const r = this.canvas().getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; },
  draw() {
    if (!this.img) return;
    const c = this.canvas(), dpr = window.devicePixelRatio || 1, g = c.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, c.clientWidth, c.clientHeight);
    g.imageSmoothingQuality = 'high';
    g.drawImage(this.img, this.tx, this.ty, this.img.width * this.s, this.img.height * this.s);
    this.v.regions.forEach((r, i) => {
      const pts = r.poly.map(([x, y]) => this.toScr(x, y)), sel = i === this.sel;
      g.beginPath(); pts.forEach(([x, y], k) => k ? g.lineTo(x, y) : g.moveTo(x, y)); g.closePath();
      g.fillStyle = colOf(r.cabinetId, sel ? 0.3 : 0.18); g.fill();
      g.lineWidth = sel ? 3 : 2; g.strokeStyle = colOf(r.cabinetId); g.stroke();
      const cx = pts.reduce((a, p) => a + p[0], 0) / 4, cy = pts.reduce((a, p) => a + p[1], 0) / 4;
      label(g, cabName(r.cabinetId), cx, cy, colOf(r.cabinetId, 0.92), '#0b1215');
      if (sel) pts.forEach(([x, y]) => { g.beginPath(); g.arc(x, y, 10, 0, 7); g.fillStyle = '#fff'; g.fill(); g.lineWidth = 3; g.strokeStyle = colOf(r.cabinetId); g.stroke(); });
    });
    if (this.drag && this.drag.type === 'rect') {
      const [ax, ay] = this.toScr(...this.drag.a), [bx, by] = this.toScr(...this.drag.b);
      g.setLineDash([8, 6]); g.lineWidth = 2.5; g.strokeStyle = '#fbbf24'; g.strokeRect(ax, ay, bx - ax, by - ay); g.setLineDash([]);
      g.fillStyle = 'rgba(251,191,36,.15)'; g.fillRect(ax, ay, bx - ax, by - ay);
    }
  },
  ui() {
    const v = this.v, sel = this.sel >= 0 ? v.regions[this.sel] : null;
    $('#ed-sel').classList.toggle('hidden', !sel);
    if (sel) $('#ed-sel-name').textContent = cabName(sel.cabinetId);
    $('#ed-add').classList.toggle('on', this.adding);
    $('#ed-add').textContent = this.adding ? 'Peruuta' : '+ Merkitse ovi';
    const [cls, txt] = qualityText(v.q); const q = $('#ed-quality'); q.className = 'badge ' + cls; q.textContent = txt;
    $('#ed-hint').textContent = this.adding ? 'Vedä suorakulmio oven päälle'
      : sel ? 'Raahaa kulmapisteitä, jos ovi on vinossa'
      : !v.regions.length ? 'Paina ”Merkitse ovi” ja vedä oven ympärille' : 'Nipistä zoomataksesi · napauta ovea';
  },
  async save() { await saveView(this.v); Engine.setRegions(this.v); updateEngineInfo(); },
  hitRegion(nx, ny) {
    let best = -1, ba = Infinity;
    this.v.regions.forEach((r, i) => {
      if (KaappiGeom.pointInPoly(nx, ny, r.poly)) { const a = Math.abs(KaappiGeom.area(r.poly)); if (a < ba) { ba = a; best = i; } }
    });
    return best;
  },
  down(e) {
    const c = this.canvas(); c.setPointerCapture(e.pointerId);
    const p = this.local(e); this.ptrs.set(e.pointerId, p);
    if (this.ptrs.size === 2) {
      const [a, b] = [...this.ptrs.values()];
      this.pinch = { d: Math.hypot(a[0] - b[0], a[1] - b[1]), m: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2], s: this.s, tx: this.tx, ty: this.ty };
      if (this.drag && this.drag.type === 'corner') this.restoreDrag();
      this.drag = null; this.draw(); return;
    }
    if (this.ptrs.size > 2) return;
    const [nx, ny] = this.toImg(...p);
    if (this.sel >= 0) {
      const r = this.v.regions[this.sel];
      const ci = r.poly.findIndex(([x, y]) => { const [sx, sy] = this.toScr(x, y); return Math.hypot(sx - p[0], sy - p[1]) < 26; });
      if (ci >= 0) { this.drag = { type: 'corner', i: ci, orig: r.poly.map(q => q.slice()) }; return; }
      if (KaappiGeom.pointInPoly(nx, ny, r.poly)) { this.drag = { type: 'move', start: [nx, ny], orig: r.poly.map(q => q.slice()), moved: false, p0: p }; return; }
    }
    if (this.adding) { this.drag = { type: 'rect', a: [nx, ny], b: [nx, ny] }; return; }
    this.drag = { type: 'pan', p0: p, tx: this.tx, ty: this.ty, moved: false };
  },
  restoreDrag() { if (this.drag && this.drag.orig) this.v.regions[this.sel].poly = this.drag.orig; },
  move(e) {
    if (!this.ptrs.has(e.pointerId)) return;
    const p = this.local(e); this.ptrs.set(e.pointerId, p);
    if (this.pinch && this.ptrs.size >= 2) {
      const [a, b] = [...this.ptrs.values()];
      const d = Math.hypot(a[0] - b[0], a[1] - b[1]), m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      const s = Math.max(this.fitS * 0.6, Math.min(this.fitS * 14, this.pinch.s * d / this.pinch.d)), k = s / this.pinch.s;
      this.s = s; this.tx = m[0] - (this.pinch.m[0] - this.pinch.tx) * k; this.ty = m[1] - (this.pinch.m[1] - this.pinch.ty) * k;
      this.draw(); return;
    }
    const dr = this.drag; if (!dr) return;
    const [nx, ny] = this.toImg(...p);
    if (dr.type === 'pan') {
      if (Math.hypot(p[0] - dr.p0[0], p[1] - dr.p0[1]) > 6) dr.moved = true;
      this.tx = dr.tx + p[0] - dr.p0[0]; this.ty = dr.ty + p[1] - dr.p0[1];
    } else if (dr.type === 'rect') dr.b = [nx, ny];
    else if (dr.type === 'corner') this.v.regions[this.sel].poly[dr.i] = [nx, ny];
    else if (dr.type === 'move') {
      if (Math.hypot(p[0] - dr.p0[0], p[1] - dr.p0[1]) > 6) dr.moved = true;
      const dx = nx - dr.start[0], dy = ny - dr.start[1];
      this.v.regions[this.sel].poly = dr.orig.map(([x, y]) => [x + dx, y + dy]);
    }
    this.draw();
  },
  async up(e) {
    this.ptrs.delete(e.pointerId);
    if (this.pinch) { if (this.ptrs.size < 2) this.pinch = null; this.drag = null; return; }
    const dr = this.drag; this.drag = null; if (!dr) return;
    if (dr.type === 'rect') {
      const x0 = Math.min(dr.a[0], dr.b[0]), x1 = Math.max(dr.a[0], dr.b[0]), y0 = Math.min(dr.a[1], dr.b[1]), y1 = Math.max(dr.a[1], dr.b[1]);
      this.draw();
      if ((x1 - x0) * this.img.width < 15 || (y1 - y0) * this.img.height < 15) { toast('Vedä suorakulmio oven kokoiseksi'); return; }
      const cabinetId = await pickCabinet('Mikä kaappi tämä on?');
      this.adding = false;
      if (cabinetId) {
        this.v.regions.push({ cabinetId, poly: [[x0, y0], [x1, y0], [x1, y1], [x0, y1]] });
        this.sel = this.v.regions.length - 1; await this.save();
      }
      this.ui(); this.draw(); return;
    }
    if (dr.type === 'corner' || (dr.type === 'move' && dr.moved)) { await this.save(); return; }
    if ((dr.type === 'pan' || dr.type === 'move') && !dr.moved) {
      const [nx, ny] = this.toImg(...this.local(e));
      this.sel = this.hitRegion(nx, ny); this.ui(); this.draw();
    }
  },
  wheel(e) {
    e.preventDefault();
    const p = this.local(e), k = Math.exp(-e.deltaY * 0.0015);
    const s = Math.max(this.fitS * 0.6, Math.min(this.fitS * 14, this.s * k)), kk = s / this.s;
    this.tx = p[0] - (p[0] - this.tx) * kk; this.ty = p[1] - (p[1] - this.ty) * kk; this.s = s; this.draw();
  },
  async transferFrom(fromId) {
    closeSheet(); toast('Etsitään vastaavuuksia…', 10000);
    try {
      const r = await Engine.call('transfer', { from: fromId, to: this.v.id });
      if (!r.ok) { toast('Kuvat eivät täsmää riittävästi. Varmista, että ne ovat samasta seinästä.', 5000); return; }
      const have = new Set(this.v.regions.map(x => x.cabinetId));
      const add = r.regions.filter(x => !have.has(x.cabinetId));
      this.v.regions.push(...add); await this.save(); this.ui(); this.draw();
      toast(`Kopioitu ${add.length} ovea (${r.inliers} vastaavuutta). Tarkista sijainnit.`, 4500);
    } catch (e) { toast('Kopiointi epäonnistui: ' + e.message, 4000); }
  },
  menu() {
    const others = sortedViews().filter(v => v.id !== this.v.id && v.regions.length);
    const el = openSheet(`<h3>${esc(this.v.name)}</h3>
      ${others.length ? '<p class="muted small">Kopioi ovet toisesta saman seinän kuvasta (esim. eri valaistus). Sijainnit lasketaan kuvien vastaavuuksista.</p>' + others.map(v => `<button class="choice" data-from="${v.id}">Kopioi ovet: ${esc(v.name)}<small>${v.regions.length} ovea</small></button>`).join('') : ''}
      <button class="choice" id="em-fit">Sovita kuva näytölle</button>
      <button class="choice" id="em-del" style="color:var(--red)">Poista tämä kartoituskuva</button>`);
    el.querySelectorAll('[data-from]').forEach(b => b.onclick = () => this.transferFrom(b.dataset.from));
    $('#em-fit', el).onclick = () => { closeSheet(); this.fit(); this.draw(); };
    $('#em-del', el).onclick = async () => {
      if (!confirm('Poistetaanko kartoituskuva? Kaapit ja niiden sisällöt säilyvät.')) return;
      const id = this.v.id; closeSheet();
      S.views.delete(id); await DB.del('views', id); Engine.removeView(id);
      if (thumbUrls.has(id)) { URL.revokeObjectURL(thumbUrls.get(id)); thumbUrls.delete(id); }
      this.close();
    };
  }
};
const KaappiGeom = {
  pointInPoly(x, y, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const [xi, yi] = poly[i], [xj, yj] = poly[j];
      if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) inside = !inside;
    }
    return inside;
  },
  area(p) { let s = 0; for (let i = 0; i < p.length; i++) { const [a, b] = p[i], [c, d] = p[(i + 1) % p.length]; s += a * d - c * b; } return s / 2; }
};
(() => {
  const c = $('#ed-canvas');
  c.addEventListener('pointerdown', e => Editor.down(e));
  c.addEventListener('pointermove', e => Editor.move(e));
  c.addEventListener('pointerup', e => Editor.up(e));
  c.addEventListener('pointercancel', e => { Editor.ptrs.delete(e.pointerId); Editor.pinch = null; Editor.restoreDrag(); Editor.drag = null; Editor.draw(); });
  c.addEventListener('wheel', e => Editor.wheel(e), { passive: false });
  window.addEventListener('resize', () => { if (Editor.v) { Editor.fit(); Editor.draw(); } });
  $('#ed-back').onclick = () => Editor.close();
  $('#ed-menu').onclick = () => Editor.menu();
  $('#ed-name').onchange = e => { Editor.v.name = e.target.value.trim() || Editor.v.name; Editor.save(); };
  $('#ed-add').onclick = () => { Editor.adding = !Editor.adding; Editor.sel = -1; Editor.ui(); Editor.draw(); };
  $('#ed-sel-del').onclick = async () => { Editor.v.regions.splice(Editor.sel, 1); Editor.sel = -1; await Editor.save(); Editor.ui(); Editor.draw(); };
  $('#ed-sel-change').onclick = async () => {
    const i = Editor.sel, id = await pickCabinet('Vaihda kaappi');
    if (id && Editor.v.regions[i]) { Editor.v.regions[i].cabinetId = id; await Editor.save(); }
    Editor.ui(); Editor.draw();
  };
})();

// ---------------------------------------------------------------- asetukset, vienti ja tuonti
$('#set-quality').onchange = async e => {
  S.settings.quality = e.target.value; await saveSettings();
  toast('Ladataan tunnistus uusilla asetuksilla…'); await Engine.restart(); renderViews();
};
$('#set-vibrate').onchange = e => { S.settings.vibrate = e.target.checked; saveSettings(); };
const blobToDataUrl = b => new Promise(r => { const f = new FileReader(); f.onload = () => r(f.result); f.readAsDataURL(b); });
const dataUrlToBlob = async u => (await fetch(u)).blob();
$('#btn-export').onclick = async () => {
  const views = [];
  for (const v of sortedViews()) views.push({ id: v.id, name: v.name, w: v.w, h: v.h, regions: v.regions, created: v.created, image: await blobToDataUrl(v.blob), thumb: v.thumb ? await blobToDataUrl(v.thumb) : null });
  const data = { app: 'kaappiskanneri', version: 1, exported: new Date().toISOString(), cabinets: sortedCabinets(), views };
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([JSON.stringify(data)], { type: 'application/json' }));
  a.download = `kaappiskanneri-${new Date().toISOString().slice(0, 10)}.json`; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
};
$('#inp-import').onchange = async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  try {
    const d = JSON.parse(await f.text());
    if (d.app !== 'kaappiskanneri') throw new Error('Tiedosto ei ole Kaappiskannerin varmuuskopio');
    if (!confirm(`Tuodaanko ${d.cabinets.length} kaappia ja ${d.views.length} kartoituskuvaa? Nykyiset tiedot korvataan.`)) return;
    toast('Tuodaan…', 10000);
    await DB.clear('cabinets'); await DB.clear('views'); S.cabinets.clear(); S.views.clear(); thumbUrls.clear();
    for (const c of d.cabinets) await saveCabinet(c);
    for (const v of d.views) {
      const { image, thumb, ...rest } = v;
      await saveView(Object.assign(rest, { blob: await dataUrlToBlob(image), thumb: thumb ? await dataUrlToBlob(thumb) : null }));
    }
    Scan.shownCab = null; S.target = null;
    await Engine.restart(); renderViews(); toast('Varmuuskopio tuotu');
  } catch (err) { toast('Tuonti epäonnistui: ' + err.message, 5000); }
};
$('#btn-wipe').onclick = async () => {
  if (!confirm('Poistetaanko kaikki kaapit, sisällöt ja kartoituskuvat pysyvästi?')) return;
  await DB.clear('cabinets'); await DB.clear('views'); await DB.clear('kv'); location.reload();
};

// ---------------------------------------------------------------- käynnistys
(async function init() {
  await DB.open();
  const st = await DB.get('kv', 'settings'); if (st) Object.assign(S.settings, st);
  for (const c of await DB.all('cabinets')) S.cabinets.set(c.id, c);
  for (const v of await DB.all('views')) S.views.set(v.id, v);
  $('#set-quality').value = S.settings.quality; $('#set-vibrate').checked = S.settings.vibrate;
  $('#btn-debug').classList.toggle('on', S.settings.debug); $('#debug-box').classList.toggle('hidden', !S.settings.debug);
  if (navigator.storage && navigator.storage.persist) navigator.storage.persist().catch(() => {});
  if ('serviceWorker' in navigator && (location.protocol === 'https:' || location.hostname === 'localhost')) {
    // Uusi versio otetaan käyttöön heti: kun uusi service worker ottaa ohjat, sivu ladataan kerran uudelleen.
    const hadController = !!navigator.serviceWorker.controller;
    let reloaded = false;
    navigator.serviceWorker.addEventListener('controllerchange', () => { if (hadController && !reloaded) { reloaded = true; location.reload(); } });
    navigator.serviceWorker.register('sw.js', { updateViaCache: 'none' }).then(r => r.update()).catch(() => {});
  }
  renderCard(); updateEngineInfo();
  setStatus('lost', 'Ladataan tunnistusta…');
  const startScreen = S.views.size ? 'scr-scan' : 'scr-views';
  showScreen(startScreen);
  try { await Engine.start(); } catch (e) { Engine.status = 'virhe: ' + e.message; updateEngineInfo(); toast('Tunnistusmoottorin lataus epäonnistui', 5000); }
  if (currentScreen === 'scr-views') renderViews();
  if (Scan.running) setStatus('lost', 'Etsitään…');
})();
