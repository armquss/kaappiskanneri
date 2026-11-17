// Koko seinän tunnistus laajoista pystykuvista ilman ennakkotietoa (kuten sovellusta avatessa).
// Käyttö: python3 test/gen_wide.py test/data && node test/wide.js test/data
const fs = require('fs');
const path = require('path');
const lib = require('../vision.js');
const dir = process.argv[2] || 'test/data';
(async () => {
  let cv = require('@techstark/opencv-js');
  if (cv instanceof Promise) cv = await cv; else if (!cv.Mat) await new Promise(r => (cv.onRuntimeInitialized = r));
  const meta = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json')));
  const wm = JSON.parse(fs.readFileSync(path.join(dir, 'wide', 'meta.json')));
  const ref = { data: new Uint8ClampedArray(fs.readFileSync(path.join(dir, 'ref.rgba'))), width: meta.W, height: meta.H };
  const regions = meta.regions.map(r => ({ cabinetId: r.name, poly: r.poly }));
  for (const [name, opt] of [['monimittakaava', {}], ['yksi mittakaava (vanha)', { globalScales: null }]]) {
    const eng = lib.createEngine(cv, opt); eng.addView('A', ref, regions);
    const by = {}; let ms = 0, wrong = 0;
    for (const fr of wm) {
      eng.resetTracking();
      const g = new Uint8Array(fs.readFileSync(path.join(dir, 'wide', fr.file + '.gray')));
      const r = eng.anchor({ data: g, width: 360, height: 640, gray: true }, {}); ms += r.ms;
      let cab = null;
      if (r.ok) { const c = lib._m.apply(r.H, r.fw / 2, r.fh / 2); for (const reg of regions) if (lib._m.pointInPoly(c[0], c[1], reg.poly.map(([x, y]) => [x * r.w, y * r.h]))) cab = reg.cabinetId; }
      const b = by[fr.vw] || (by[fr.vw] = { ok: 0, n: 0 }); b.n++;
      if (r.ok) { if (cab === fr.truth || fr.truth == null) b.ok++; else wrong++; }
    }
    console.log(`${name}: ` + Object.entries(by).map(([k, b]) => `näkyvä leveys ${k}px ${b.ok}/${b.n}`).join(', ') + ` | väärin ${wrong} | ${(ms / wm.length).toFixed(0)} ms`);
    eng.clear();
  }
})();
