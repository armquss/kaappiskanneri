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
async function videoFrameToGray(vf, m_max) {
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
      // Pienennys 2x2-keskiarvolla suoraan Y-tasosta, kun kuva on vähintään kaksi kertaa käsittelykokoa:
      // ei isoa kuvamatriisia eikä erillistä skaalausta (puhelimessa tämä vaihe vei jopa 40 ms)
      if (Math.max(w, h) >= 2 * (m_max || 640)) {
        const w2 = w >> 1, h2 = h >> 1, src = T.vbuf;
        data = new Uint8Array(w2 * h2);
        for (let y = 0; y < h2; y++) {
          let r0 = offset + (y * 2) * stride, r1 = r0 + stride, o = y * w2;
          for (let x = 0; x < w2; x++, r0 += 2, r1 += 2) data[o + x] = (src[r0] + src[r0 + 1] + src[r1] + src[r1 + 1] + 2) >> 2;
        }
        T.src = fmt + ' ½';
        return { data, width: w2, height: h2, gray: true, rot: vf.rotation || 0 };
      }
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
  const img = m.vf ? await videoFrameToGray(m.vf, m.max) : m.bmp ? bitmapToImage(m.bmp) : m.img;
  if (!m.vf) T.src = m.bmp ? 'bitmap' : 'canvas';
  const readMs = Date.now() - tr0;
  const st = tracker.state;
  // pyydetään tunnistusta, kun tunnistussäie on vapaa ja (sijainti hukassa tai on aika tarkistaa)
  const lostish = !st.H;
  // Mukautuva tahti: kun seuranta on vahvaa ja ovien reunat vahvistavat sijainnin, tarkistetaan harvemmin;
  // toistuvat tuloksettomat tarkistukset (esim. lähikuva) harvennetaan, jotta seuranta saa suorittimen.
  const verified = st.H && st.lastT - st.doorT < 600;
  const interval = Math.min(4000, (verified ? 2400 : T.detectIntervalMs) * Math.pow(1.5, Math.min(3, T.misses || 0)));
  const want = T.detIdle && tracker.viewCount > 0 && port &&
    (lostish || (m.t - T.lastDetT >= interval));
  const out = tracker.processFrame(img, { id: m.fid, t: m.t, wantGray: want, debug: m.debug, keepChain: !T.detIdle });
  // Hukassa: tunnistukselle valitaan terävä kuva (liike-epäterävä kuva tuhlaisi raskaan haun).
  // Vertailukohta on viime hetkien terävin kuva; liian pitkää odotusta ei sallita.
  let sharpOk = true;
  if (lostish && out.res.sharp != null) {
    T.sharpRef = Math.max(out.res.sharp, (T.sharpRef || 0) * 0.96);
    sharpOk = out.res.sharp >= 0.6 * T.sharpRef || m.t - T.lastDetT > 900;
    if (!sharpOk) T.skippedBlur = (T.skippedBlur || 0) + 1;
  }
  // nopean liikkeen aikana tarkistus odottaa (tulos vanhenisi heti), paitsi jos sijainti on hukassa
  if (want && sharpOk && (lostish || out.res.moving < 0.04)) {
    const pr = out.prior, full = !pr || pr.loose || (m.t - T.lastFullT >= T.fullEveryMs);
    if (full) T.lastFullT = m.t;
    T.detIdle = false; T.lastDetT = m.t;
    port.postMessage({ type: 'anchor', frameId: out.id, img: out.gray, priorH: pr && pr.H, priorView: pr && pr.viewId, loose: !!(pr && pr.loose), full, t: m.t }, [out.gray.data.buffer]);
  }
  for (const fb of out.kfFeedback || []) port && port.postMessage({ type: 'kfFeedback', id: fb.id, ok: fb.ok, H: fb.H });
  out.res.det = T.lastDet;
  if (out.res.debug) { out.res.debug.read = readMs; out.res.debug.src = T.src; out.res.debug.blurSkip = T.skippedBlur || 0; }
  return out.res;
}
function onAnchor(r) {
  T.detIdle = true;
  const a = tracker.applyAnchor(r);
  if (a.kfReject) port.postMessage({ type: 'kfFeedback', id: a.kfReject, ok: false });
  T.misses = a.applied || !tracker.state.H ? 0 : (T.misses || 0) + 1;
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
      if (q.type === 'kfFeedback') {
        try { engine.keyframeFeedback(q.id, q.ok, q.H); } catch (e) {}
        const ev = engine.takeKfEvents();
        if (ev.length) { const tr = []; for (const e of ev) if (e.add) tr.push(e.add.desc.buffer, e.add.pts.buffer, e.add.thumb.buffer); self.postMessage({ type: 'kf', events: ev }, tr); }
        return;
      }
      if (q.type === 'anchor') {
        let result;
        try { result = engine.anchor(q.img, { priorH: q.priorH, priorView: q.priorView, loose: q.loose, full: q.full, t: q.t }); }
        catch (err) { result = { ok: false, rejects: ['virhe: ' + (err && err.message || err)] }; }
        result.frameId = q.frameId;
        port.postMessage({ type: 'anchorResult', result });
        // oppiva kartoitus: uudet/poistuneet avainkuvat pääsäikeelle tallennettavaksi
        const ev = engine.takeKfEvents();
        if (ev.length) {
          const tr = [];
          for (const e of ev) if (e.add) tr.push(e.add.desc.buffer, e.add.pts.buffer, e.add.thumb.buffer);
          self.postMessage({ type: 'kf', events: ev }, tr);
        }
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
  addKeyframes(m) { return engine.addKeyframes(m.list); },
  forgetKeyframes(m) { engine.forgetKeyframes(m.viewId); return true; },
  snap(m) { return KaappiVision.snapQuad(cv, m.img, m.quad, m.opts); },
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
