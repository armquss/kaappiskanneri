// Tunnistusmoottorin testi synteettisellä keittiöllä. Käyttö: node test/run_engine.js test/data
const fs = require('fs');
const path = require('path');
const { createEngine } = require('../vision.js');
const dir = process.argv[2] || 'test/data';

(async () => {
  let cv = require('@techstark/opencv-js');
  if (cv instanceof Promise) cv = await cv; else if (!cv.Mat) await new Promise(r => (cv.onRuntimeInitialized = r));
  const meta = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json')));
  const load = (f, w, h) => ({ data: new Uint8ClampedArray(fs.readFileSync(path.join(dir, f + '.rgba'))), width: w, height: h });
  const regs = rs => rs.map(r => ({ cabinetId: r.name, poly: r.poly }));
  const eng = createEngine(cv, JSON.parse(process.env.P || "{}"));
  let t = Date.now();
  const info = eng.addView('A', load('ref', meta.W, meta.H), regs(meta.regions));
  console.log('view A', info, 'quality', eng.viewQuality('A'), (Date.now() - t) + 'ms');

  // 1) Yksittäiset kuvat (seuranta nollataan joka kuvan välissä)
  let ok = 0, wrong = 0, miss = 0; const fails = []; let ms = 0;
  for (const fr of meta.frames) {
    eng.resetTracking();
    const r = eng.processFrame(load(fr.file, meta.FW, meta.FH), { debug: true });
    ms += r.debug.ms;
    const got = r.rawCabinetId;
    if (got === fr.truth) ok++;
    else if (got == null) { miss++; fails.push(`${fr.file} truth=${fr.truth} got=- vw=${fr.view_w} ${r.debug.rejects.join(',')}`); }
    else { wrong++; fails.push(`${fr.file} truth=${fr.truth} got=${got} ${r.onEdge ? 'reuna' : 'WRONG'} vw=${fr.view_w} inl=${r.debug.inliers}`); }
  }
  console.log(`\nYksittäiskuvat: oikein ${ok}/${meta.frames.length}, väärin ${wrong}, ei tunnistusta ${miss}, keskim. ${(ms / meta.frames.length).toFixed(0)} ms`);
  fails.forEach(f => console.log('  ', f));

  // 2) Liikesarja: zoomaus lähikuvaan ja liukuminen naapurioveen pelkällä seurannalla
  eng.resetTracking();
  let sOk = 0, sWrong = 0, sMiss = 0, sMs = 0; const modes = {}; const log = [];
  for (const fr of meta.seq) {
    const r = eng.processFrame(load(fr.file, meta.FW, meta.FH), { debug: true });
    modes[r.mode] = (modes[r.mode] || 0) + 1; sMs += r.debug.ms;
    const got = r.cabinetId;
    if (fr.truth == null) continue;
    if (got === fr.truth) sOk++; else if (got == null) sMiss++; else sWrong++;
    log.push(`${fr.file}:${r.mode[0]}:${got || '-'}${got === fr.truth ? '' : '(' + fr.truth + ')'}`);
  }
  console.log(`\nLiikesarja: oikein ${sOk}, väärin ${sWrong}, ei tunnistusta ${sMiss}; keskim. ${(sMs / meta.seq.length).toFixed(0)} ms; tilat`, modes);
  console.log(log.join(' '));

  // 3) Alueiden siirto toiseen näkymään (eri valaistus ja kulma)
  eng.addView('B', load('ref2', meta.W, meta.H), []);
  const tr = eng.transferRegions('A', 'B');
  let maxErr = 0;
  if (tr.ok) for (const r of tr.regions) {
    const truth = meta.regions2.find(x => x.name === r.cabinetId);
    r.poly.forEach((p, i) => { maxErr = Math.max(maxErr, Math.hypot((p[0] - truth.poly[i][0]) * meta.W, (p[1] - truth.poly[i][1]) * meta.H)); });
  }
  console.log(`\nAlueiden siirto: ok=${tr.ok} inliers=${tr.inliers} alueita=${tr.regions ? tr.regions.length : 0} maksimivirhe=${maxErr.toFixed(1)} px`);
  eng.removeView('A');
  eng.setRegions('B', tr.regions || []);
  let bOk = 0; const sample = meta.frames.filter(f => f.truth).slice(0, 48);
  for (const fr of sample) { eng.resetTracking(); const r = eng.processFrame(load(fr.file, meta.FW, meta.FH)); if (r.rawCabinetId === fr.truth) bOk++; }
  console.log(`Pelkällä siirretyllä näkymällä B: oikein ${bOk}/${sample.length}`);
})();
