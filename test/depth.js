// Rajojen tarkkuus syvyydellisessä (parallaksi) aineistossa: seuranta + tunnistus viiveellä kuten
// sovelluksessa; jokaisessa kuvassa verrataan piirrettyjen ovien kulmia oikeisiin kulmiin (pikseleinä,
// 360x640-kuvassa). Käyttö: python3 test/gen_depth.py <varmuuskopio> "<näkymä>" <kansio> && node test/depth.js <kansio>
const fs = require('fs');
const path = require('path');
const lib = require('../vision.js');
const dir = process.argv[2];
const EP = JSON.parse(process.env.EP || '{}'), TP = JSON.parse(process.env.TP || '{}');

const lat0 = +(process.env.LAT || 4);
(async () => {
  let cv = require('@techstark/opencv-js');
  if (cv instanceof Promise) cv = await cv; else if (!cv.Mat) await new Promise(r => (cv.onRuntimeInitialized = r));
  const meta = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json')));
  const ref = { data: new Uint8ClampedArray(fs.readFileSync(path.join(dir, 'ref.rgba'))), width: meta.W, height: meta.H };
  const eng = lib.createEngine(cv, Object.assign({ learn: true }, EP));
  const info = eng.addView('A', ref, meta.regions);
  const tr = lib.createTracker(cv, TP);
  tr.setView('A', info.w, info.h, meta.regions);
  const frameIn = (dd, f) => ({ data: new Uint8Array(fs.readFileSync(path.join(dd, f + '.gray'))), width: 360, height: 640, gray: true });
  const frame = f => frameIn(dir, f);
  // Aiempi käyttökerta (oppiminen) toisella kamerapolulla samassa keittiössä
  const KS = { ankkurit: 0, väliaikaisia: 0, vahvistettu: 0, 'vahvistettu-ei-mahtunut': 0, hylätty: 0, snapKuvia: 0 };
  for (const ld of (process.env.LEARN_FROM || '').split(',').filter(Boolean)) {
    const lm = JSON.parse(fs.readFileSync(path.join(ld, 'meta.json')));
    const tl = lib.createTracker(cv, TP); tl.setView('A', info.w, info.h, meta.regions);
    let pend = null, last = -99;
    lm.frames.forEach((fr, k) => {
      if (pend && k >= pend.readyAt) { const a = tl.applyAnchor(Object.assign({ frameId: pend.frameId }, pend.result)); if (a.kfReject) eng.keyframeFeedback(a.kfReject, false); pend = null; }
      const w = !pend && (tl.state.H == null || k - last >= 4);
      const o = tl.processFrame(frameIn(ld, fr.file), { id: k, t: k * 66, wantGray: w, keepChain: !!pend });
      for (const fb of o.kfFeedback || []) { const c = eng.keyframeFeedback(fb.id, fb.ok, fb.H); KS[fb.ok ? (c ? 'vahvistettu' : 'vahvistettu-ei-mahtunut') : 'hylätty']++; }
      if (o.res.debug.snap) KS.snapKuvia++;
      if (w) { const pr = o.prior; const rr = eng.anchor(o.gray, { priorH: pr && pr.H, priorView: pr && pr.viewId, loose: !!(pr && pr.loose), full: !pr, t: k * 66 }); if (rr.ok) KS.ankkurit++; if (rr.kfId) KS.väliaikaisia++; pend = { frameId: o.id, result: rr, readyAt: k + lat0 }; last = k; }
    });
    eng.takeKfEvents(); eng.resetTracking();
  }
  if (process.env.LEARN_FROM) eng.params.learn = process.env.KEEP_LEARNING ? true : false;
  if (process.env.SHOWKF) console.log('opittuja avainkuvia', JSON.stringify(eng.keyframeStats()), JSON.stringify(KS));
  let pending = null, lastSend = -99;
  const bySeg = {}, all = [], vias = {};
  const lat = +(process.env.LAT || 4);
  const visible = p => p.some(([x, y]) => x > -50 && x < 410 && y > -50 && y < 690);
  meta.frames.forEach((fr, k) => {
    if (pending && k >= pending.readyAt) { (__a => { if (__a.kfReject) eng.keyframeFeedback(__a.kfReject, false); return __a; })(tr.applyAnchor(Object.assign({ frameId: pending.frameId }, pending.result))); if (pending.result.ok) vias[pending.result.via] = (vias[pending.result.via] || 0) + 1; pending = null; }
    const want = !pending && (tr.state.H == null || k - lastSend >= 4);
    const out = tr.processFrame(frame(fr.file), { id: k, t: k * 66, wantGray: want, keepChain: !!pending }); for (const fb of out.kfFeedback || []) eng.keyframeFeedback(fb.id, fb.ok, fb.H);
    if (want) {
      const pr = out.prior;
      const result = eng.anchor(out.gray, { priorH: pr && pr.H, priorView: pr && pr.viewId, loose: !!(pr && pr.loose), full: !pr, t: k * 66 });
      if (process.env.GT_ANCHOR) { // koe: täydellinen ankkuri (oikeasta vastauksesta)
        const src = [], dst = [];
        for (const g of fr.gt) { const reg = meta.regions.find(r => r.cabinetId === g.cabinetId); g.poly.forEach(([x, y], i) => { src.push(x, y); dst.push(reg.poly[i][0] * info.w, reg.poly[i][1] * info.h); }); }
        const sM = cv.matFromArray(src.length / 2, 1, cv.CV_32FC2, src), dM = cv.matFromArray(dst.length / 2, 1, cv.CV_32FC2, dst);
        const Hm = cv.findHomography(sM, dM, 0); Object.assign(result, { ok: true, H: Array.from(Hm.data64F), viewId: 'A', w: info.w, h: info.h, fw: 360, fh: 640, via: 'GT', kfId: null });
        sM.delete(); dM.delete(); Hm.delete();
      }
      pending = { frameId: out.id, result, readyAt: k + lat }; lastSend = k;
    }
    const seg = bySeg[fr.seg] || (bySeg[fr.seg] = { n: 0, tracked: 0, errs: [] });
    seg.n++;
    if (out.res.mode === 'lost') return;
    seg.tracked++;
    for (const g of fr.gt) {
      if (!visible(g.poly)) continue;
      const o = out.res.overlay.find(x => x.cabinetId === g.cabinetId);
      if (!o) continue;
      const e = o.poly.reduce((s, [x, y], i) => s + Math.hypot(x * out.res.frameW * 360 / out.res.frameW - g.poly[i][0], y * 640 - g.poly[i][1]), 0) / 4;
      seg.errs.push(e); all.push(e);
    }
  });
  const q = (a, p) => { if (!a.length) return NaN; const s = a.slice().sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };
  for (const [name, s] of Object.entries(bySeg)) console.log(`${name.padEnd(22)} seurannassa ${String(s.tracked).padStart(2)}/${s.n}  kulmavirhe mediaani ${q(s.errs, 0.5).toFixed(1)} px, 90 % ${q(s.errs, 0.9).toFixed(1)} px`);
  if (process.env.JSON) console.log('JSON' + JSON.stringify({ all, tracked: Object.values(bySeg).reduce((a, s) => a + s.tracked, 0), n: meta.frames.length, close: Object.entries(bySeg).filter(([k]) => /lähellä|kaappi/.test(k)).flatMap(([, s]) => s.errs) }));
  console.log(`KAIKKI: seurannassa ${Object.values(bySeg).reduce((a, s) => a + s.tracked, 0)}/${meta.frames.length}, kulmavirhe mediaani ${q(all, 0.5).toFixed(1)} px, 90 % ${q(all, 0.9).toFixed(1)} px, max ${q(all, 1).toFixed(0)} px | ankkurit ${JSON.stringify(vias)}`);
})();
