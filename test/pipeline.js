// Kaksisäikeisen putken simulaatio: seuranta joka kuvalla, tunnistus viiveellä (kuten hitaassa puhelimessa).
// Käyttö: node test/pipeline.js [test/data]
const fs = require('fs');
const path = require('path');
const { createEngine, createTracker } = require('../vision.js');
const dir = process.argv[2] || 'test/data';

(async () => {
  let cv = require('@techstark/opencv-js');
  if (cv instanceof Promise) cv = await cv; else if (!cv.Mat) await new Promise(r => (cv.onRuntimeInitialized = r));
  const meta = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json')));
  const load = (f, w, h) => ({ data: new Uint8ClampedArray(fs.readFileSync(path.join(dir, f + '.rgba'))), width: w, height: h });
  const regions = meta.regions.map(r => ({ cabinetId: r.name, poly: r.poly }));

  const eng = createEngine(cv, JSON.parse(process.env.EP || '{}'));
  const info = eng.addView('A', load('ref', meta.W, meta.H), regions);

  let allOk = true;
  for (const stride of (process.env.ST||'1,2,3').split(',').map(Number)) for (const latency of (process.env.LAT||'2,8,20').split(',').map(Number)) {
    const tr = createTracker(cv, JSON.parse(process.env.TP || '{}'));
    tr.setView('A', info.w, info.h, regions);
    eng.resetTracking();
    let pending = null, lastSend = -99, ok = 0, wrong = 0, miss = 0, applied = 0, rejected = {}, ms = 0, n = 0;
    const log = [];
    const frames = [];
    for (let i = 0; i < meta.seq.length; i += stride) frames.push(meta.seq[i]);
    // toistetaan sarja edestakaisin, jotta nähdään myös palautuminen
    const order = frames.concat(frames.slice().reverse());
    order.forEach((fr, k) => {
      // tunnistuksen valmistuminen viiveellä
      if (pending && k >= pending.readyAt) {
        const r = (__a => { if (__a.kfReject) eng.keyframeFeedback(__a.kfReject, false); return __a; })(tr.applyAnchor(Object.assign({ frameId: pending.frameId }, pending.result)));
        if (process.env.V) log.push(`[A${pending.frameId}@${k}:${r.applied ? 'ok' : r.reason}:${pending.result.via}:${pending.result.inliers}${pending.result.ok ? '' : ':' + pending.result.rejects.join('|')}]`);
        if (r.applied) applied++; else rejected[r.reason || '-'] = (rejected[r.reason || '-'] || 0) + 1;
        pending = null;
      }
      const wantDetect = !pending && (tr.state.H == null || k - lastSend >= 3);
      const out = tr.processFrame(load(fr.file, meta.FW, meta.FH), { id: k, t: k * 40 * stride, wantGray: wantDetect, keepChain: !!pending }); for (const fb of out.kfFeedback || []) eng.keyframeFeedback(fb.id, fb.ok, fb.H);
      ms += out.res.debug.ms; n++;
      if (wantDetect) {
        const pr = out.prior;
        const result = eng.anchor(out.gray, { priorH: pr && pr.H, priorView: pr && pr.viewId, loose: !!(pr && pr.loose), full: !pr, t: k * 40 * stride });
        pending = { frameId: out.id, result, readyAt: k + latency };
        lastSend = k;
      }
      const got = out.res.mode === 'lost' ? null : out.res.cabinetId;
      if (process.env.DRIFT && out.H) { const c = require('../vision.js')._m.apply(out.H, out.res.frameW / 2, out.res.frameH / 2); const s = meta.W / info.w; log.push(`<${Math.round(c[0] * s - fr.c[0])},${Math.round(c[1] * s - fr.c[1])}>`); }
      if (fr.truth == null) return;
      if (got === fr.truth) ok++; else if (got == null) miss++; else if (out.res.rawCabinetId === fr.truth || out.res.onEdge) ok++; /* rajalla: vakautuksen viive */ else { wrong++; }
      log.push(`${k}${out.res.mode[0]}${out.res.debug.inliers}${got || '-'}${got === fr.truth ? '' : '(' + fr.truth + ')'}`);
    });
    if (wrong) allOk = false;
    console.log(`askel ${stride}, viive ${String(latency).padStart(2)} kuvaa: oikein ${ok}, väärin ${wrong}, ei tiedossa ${miss} | ankkureita ${applied}, hylätty ${JSON.stringify(rejected)} | seuranta ${(ms / n).toFixed(0)} ms/kuva`);
    if (process.env.V) console.log('   ' + log.join(' '));
  }
  console.log(allOk ? '\nEi vääriä tunnistuksia.' : '\nVÄÄRIÄ TUNNISTUKSIA!');
})();
