/*
 * Kaksi roolia samasta tiedostosta:
 *  - "tracker": kevyt seuranta jokaiselle kamerakuvalle (~10–40 ms)
 *  - "detector": raskas tunnistus (ankkurointi kartoituskuvaan), taustalla
 * Säikeet keskustelevat suoraan MessageChannelin kautta, joten pääsäie ei ole välissä.
 */
importScripts('opencv.js', 'vision.js');

let cv = null, role = null, engine = null, tracker = null, port = null, params = {};
async function ensureCv() {
  if (cv) return cv;
  let c = self.cv;
  if (c && typeof c.then === 'function' && !c.Mat) c = await c;
  else if (c && !c.Mat) await new Promise(r => (c.onRuntimeInitialized = r));
  cv = c; return cv;
}

// ---------------------------------------------------------------- tracker-tila
const T = {
  detIdle: true, lastDetT: -1e9, lastFullT: -1e9, lastDet: null, canvas: null, ctx: null,
  // seurannan aikana tarkistus harvoin (säästää suoritinta seurannalle), hukassa jatkuvasti
  detectIntervalMs: 1200, fullEveryMs: 6000
};
function bitmapToImage(bmp) {
  const w = bmp.width, h = bmp.height;
  if (!T.canvas || T.canvas.width !== w || T.canvas.height !== h) {
    T.canvas = new OffscreenCanvas(w, h); T.ctx = T.canvas.getContext('2d', { willReadFrequently: true });
  }
  T.ctx.drawImage(bmp, 0, 0); bmp.close();
  const d = T.ctx.getImageData(0, 0, w, h);
  return { data: d.data, width: w, height: h };
}
// VideoFrame -> harmaasävy. YUV-muodoissa Y-taso on suoraan harmaasävykuva (ei värimuunnosta).
async function videoFrameToGray(vf) {
  try {
    const fmt = vf.format, rect = vf.visibleRect;
    if (!fmt || !rect) throw new Error('vf-format ' + fmt);
    const w = rect.width, h = rect.height;
    const size = vf.allocationSize({ rect });
    if (!T.vbuf || T.vbuf.byteLength < size) T.vbuf = new Uint8Array(size);
    const layout = await vf.copyTo(T.vbuf, { rect });
    let data;
    if (/^(I4|NV12)/.test(fmt)) {
      const { offset, stride } = layout[0];
      if (stride === w) data = T.vbuf.slice(offset, offset + w * h);
      else { data = new Uint8Array(w * h); for (let y = 0; y < h; y++) data.set(T.vbuf.subarray(offset + y * stride, offset + y * stride + w), y * w); }
    } else if (/^(RGB|BGR)/.test(fmt)) {
      const { offset, stride } = layout[0];
      const m = new cv.Mat(h, w, cv.CV_8UC4);
      for (let y = 0; y < h; y++) m.data.set(T.vbuf.subarray(offset + y * stride, offset + y * stride + w * 4), y * w * 4);
      const g = new cv.Mat(); cv.cvtColor(m, g, fmt.startsWith('BGR') ? cv.COLOR_BGRA2GRAY : cv.COLOR_RGBA2GRAY);
      data = new Uint8Array(g.data); m.delete(); g.delete();
    } else throw new Error('vf-format ' + fmt);
    T.src = fmt;
    return { data, width: w, height: h, gray: true, rot: vf.rotation || 0 };
  } finally { vf.close(); }
}
async function trackFrame(m) {
  const tr0 = Date.now();
  const img = m.vf ? await videoFrameToGray(m.vf) : m.bmp ? bitmapToImage(m.bmp) : m.img;
  if (!m.vf) T.src = m.bmp ? 'bitmap' : 'canvas';
  const readMs = Date.now() - tr0;
  const st = tracker.state;
  // pyydetään tunnistusta, kun tunnistussäie on vapaa ja (sijainti hukassa tai on aika tarkistaa)
  const lostish = !st.H;
  const want = T.detIdle && tracker.viewCount > 0 && port &&
    (lostish || (m.t - T.lastDetT >= T.detectIntervalMs));
  const out = tracker.processFrame(img, { id: m.fid, t: m.t, wantGray: want, debug: m.debug, keepChain: !T.detIdle });
  // nopean liikkeen aikana tarkistus odottaa (tulos vanhenisi heti), paitsi jos sijainti on hukassa
  if (want && (lostish || out.res.moving < 0.04)) {
    const pr = out.prior, full = !pr || pr.loose || (m.t - T.lastFullT >= T.fullEveryMs);
    if (full) T.lastFullT = m.t;
    T.detIdle = false; T.lastDetT = m.t;
    port.postMessage({ type: 'anchor', frameId: out.id, img: out.gray, priorH: pr && pr.H, priorView: pr && pr.viewId, loose: !!(pr && pr.loose), full, t: m.t }, [out.gray.data.buffer]);
  }
  out.res.det = T.lastDet;
  if (out.res.debug) { out.res.debug.read = readMs; out.res.debug.src = T.src; }
  return out.res;
}
function onAnchor(r) {
  T.detIdle = true;
  const a = tracker.applyAnchor(r);
  T.lastDet = { ms: r.ms, via: r.via, inliers: r.inliers, ok: r.ok, applied: a.applied, reason: a.reason || '', steps: a.steps || 0, rejects: (r.rejects || []).slice(0, 4) };
  if (a.applied && a.res) { a.res.det = T.lastDet; self.postMessage({ type: 'update', res: a.res }); }
}

// ---------------------------------------------------------------- viestit
const handlers = {
  async init(m) {
    await ensureCv();
    role = m.role; params = m.params || {};
    if (role === 'tracker') tracker = KaappiVision.createTracker(cv, params);
    else { if (engine) engine.clear(); engine = KaappiVision.createEngine(cv, params); }
    return { ok: true, role };
  },
  connect(m) {
    port = m.port;
    port.onmessage = e => {
      const q = e.data;
      if (role === 'tracker') { if (q.type === 'anchorResult') onAnchor(q.result); return; }
      if (q.type === 'anchor') {
        let result;
        try { result = engine.anchor(q.img, { priorH: q.priorH, priorView: q.priorView, loose: q.loose, full: q.full, t: q.t }); }
        catch (err) { result = { ok: false, rejects: ['virhe: ' + (err && err.message || err)] }; }
        result.frameId = q.frameId;
        port.postMessage({ type: 'anchorResult', result });
      }
    };
    return true;
  },
  // tunnistus
  addView(m) { return engine.addView(m.viewId, m.img, m.regions); },
  setRegions(m) { return engine.setRegions(m.viewId, m.regions); },
  removeView(m) { if (engine) engine.removeView(m.viewId); if (tracker) tracker.removeView(m.viewId); return true; },
  quality(m) { return engine.viewQuality(m.viewId); },
  transfer(m) { return engine.transferRegions(m.from, m.to); },
  // seuranta
  setView(m) { tracker.setView(m.viewId, m.w, m.h, m.regions); return true; },
  reset() { if (tracker) tracker.reset(); if (engine) engine.resetTracking(); T.detIdle = true; return true; },
  async frame(m) { return await trackFrame(m); }
};

let queue = Promise.resolve();
self.onmessage = (e) => {
  const m = e.data;
  queue = queue.then(async () => {
    try {
      if (m.type !== 'init' && m.type !== 'connect' && !engine && !tracker) throw new Error('engine not ready');
      const result = await handlers[m.type](m);
      if (m.id != null) self.postMessage({ id: m.id, result });
    } catch (err) {
      if (m.type === 'frame') try { if (m.bmp) m.bmp.close(); if (m.vf) m.vf.close(); } catch (e) {}
      if (m.id != null) self.postMessage({ id: m.id, error: String(err && err.message || err) });
    }
  });
};
