// Oppivan kartoituksen testi iltavalossa.
//  1. käyttökerta: seuranta + tunnistus viiveellä (kuten sovelluksessa) -> avainkuvat tallentuvat
//  2. käyttökerta: uusi moottori, ladataan tallennetut avainkuvat -> tunnistus tyhjästä laajoista kuvista
//  Turvallisuus: identtisten ovien lähikuvat eivät saa koskaan tunnistua väärin.
// Käyttö: python3 test/gen_evening.py test/data && node test/learning.js test/data
const fs = require('fs');
const path = require('path');
const lib = require('../vision.js');
const dir = process.argv[2] || 'test/data';

(async () => {
  let cv = require('@techstark/opencv-js');
  if (cv instanceof Promise) cv = await cv; else if (!cv.Mat) await new Promise(r => (cv.onRuntimeInitialized = r));
  const meta = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json')));
  const em = JSON.parse(fs.readFileSync(path.join(dir, 'evening', 'meta.json')));
  const ref = { data: new Uint8ClampedArray(fs.readFileSync(path.join(dir, 'ref.rgba'))), width: meta.W, height: meta.H };
  const regions = meta.regions.map(r => ({ cabinetId: r.name, poly: r.poly }));
  const frame = f => ({ data: new Uint8Array(fs.readFileSync(path.join(dir, 'evening', f + '.gray'))), width: 360, height: 640, gray: true });
  const cabOf = (r, eng) => {
    if (!r.ok) return null;
    const c = lib._m.apply(r.H, r.fw / 2, r.fh / 2);
    for (const reg of regions) if (lib._m.pointInPoly(c[0], c[1], reg.poly.map(([x, y]) => [x * r.w, y * r.h]))) return reg.cabinetId;
    return '(ei ovea)';
  };

  // ---- 1. käyttökerta: oppiminen
  const eng1 = lib.createEngine(cv);
  const info = eng1.addView('A', ref, regions);
  const tr = lib.createTracker(cv);
  tr.setView('A', info.w, info.h, regions);
  const store = new Map(); // "IndexedDB"
  const persist = () => { for (const e of eng1.takeKfEvents()) { if (e.add) store.set(e.add.id, structuredClone(e.add)); if (e.remove) store.delete(e.remove); } };
  let pending = null, lastSend = -99, known = 0, wrong = 0;
  em.learn.forEach((fr, k) => {
    if (pending && k >= pending.readyAt) { tr.applyAnchor(Object.assign({ frameId: pending.frameId }, pending.result)); pending = null; persist(); }
    const want = !pending && (tr.state.H == null || k - lastSend >= 3);
    const out = tr.processFrame(frame(fr.file), { id: k, t: k * 40, wantGray: want, keepChain: !!pending });
    if (want) {
      const pr = out.prior;
      const result = eng1.anchor(out.gray, { priorH: pr && pr.H, priorView: pr && pr.viewId, loose: !!(pr && pr.loose), full: !pr, t: k * 40 });
      pending = { frameId: out.id, result, readyAt: k + 4 }; lastSend = k;
    }
    const got = out.res.mode === 'lost' ? null : out.res.cabinetId;
    if (fr.truth && got) { if (got === fr.truth || out.res.rawCabinetId === fr.truth || out.res.onEdge) known++; else wrong++; }
  });
  persist();
  const stats = eng1.keyframeStats();
  const bytes = [...store.values()].reduce((a, e) => a + e.desc.byteLength + e.pts.byteLength + e.thumb.byteLength + 200, 0);
  console.log(`1. käyttökerta: kaappi tiedossa ${known}/${em.learn.filter(f => f.truth).length} kuvassa, väärin ${wrong} | opittuja avainkuvia ${store.size} (kontekstirikkaita ${stats.byView.A ? stats.byView.A.ctx : 0}), tallennuskoko ${(bytes / 1024).toFixed(0)} kt`);

  // ---- 2. käyttökerta: ilman oppimista vs. opitut avainkuvat
  let allSafe = wrong === 0;
  for (const [name, withKf, learn] of [['ilman opittuja', false, false], ['opittujen kanssa', true, false], ['opittujen kanssa + oppii lisää', true, true]]) {
    const eng = lib.createEngine(cv, { learn });
    eng.addView('A', ref, regions);
    if (withKf) { const r = eng.addKeyframes([...store.values()].map(e => structuredClone(e))); if (r.loaded !== store.size) throw new Error('lataus ' + r.loaded); }
    let ok = 0, bad = 0, ms = 0, vias = {};
    for (const fr of em.test) {
      eng.resetTracking();
      const r = eng.anchor(frame(fr.file), {}); ms += r.ms;
      const cab = cabOf(r);
      if (r.ok) { vias[r.via] = (vias[r.via] || 0) + 1; if (cab === fr.truth || fr.truth == null) ok++; else if (cab === '(ei ovea)') { /* tähtäin oven reunalla: ohilyönti, ei väärä kaappi */ } else { bad++; console.log(`   väärin ${fr.file}: ${cab} (oikea ${fr.truth}) ${r.via} ${r.inliers}/${r.good} vw ${fr.vw}`); } }
    }
    let cOk = 0, cBad = 0;
    for (const fr of em.close) {
      eng.resetTracking();
      const r = eng.anchor(frame(fr.file), {});
      const cab = cabOf(r);
      if (r.ok) { if (cab === fr.truth) cOk++; else if (cab === '(ei ovea)') { /* reunalla */ } else { cBad++; console.log(`   lähikuva väärin ${fr.file}: ${cab} (oikea ${fr.truth}) ${r.via} ${r.inliers}/${r.good}`); } }
    }
    if (bad || cBad) allSafe = false;
    console.log(`2. käyttökerta ${name}: laajat kuvat ${ok}/${em.test.length} (väärin ${bad}, keskim. ${(ms / em.test.length).toFixed(0)} ms, ${JSON.stringify(vias)}) | lähikuvat ilman ennakkotietoa: oikein ${cOk}, väärin ${cBad} / ${em.close.length}`);
    eng.clear();
  }
  console.log(allSafe ? '\nEi vääriä tunnistuksia.' : '\nVÄÄRIÄ TUNNISTUKSIA!');
})();
