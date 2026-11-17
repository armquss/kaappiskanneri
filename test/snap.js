// Ovien reunojen kohdistuksen tarkkuus: karkeasti piirretty suorakulmio -> oven todelliset kulmat.
// Käyttö: node test/snap.js test/data
const fs = require('fs');
const path = require('path');
const lib = require('../vision.js');
const dir = process.argv[2] || 'test/data';
(async () => {
  let cv = require('@techstark/opencv-js');
  if (cv instanceof Promise) cv = await cv; else if (!cv.Mat) await new Promise(r => (cv.onRuntimeInitialized = r));
  const meta = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json')));
  let rnd = 7; const rand = () => ((rnd = (rnd * 16807) % 2147483647) / 2147483647);
  const crop = (img, x0, y0, w, h) => { const d = new Uint8ClampedArray(w * h * 4); for (let y = 0; y < h; y++) d.set(img.data.subarray(((y0 + y) * img.width + x0) * 4, ((y0 + y) * img.width + x0 + w) * 4), y * w * 4); return { data: d, width: w, height: h }; };
  const err = (a, b) => a.reduce((s, p, i) => s + Math.hypot(p[0] - b[i][0], p[1] - b[i][1]), 0) / 4;
  for (const [name, file, regs] of [['suora kartoituskuva', 'ref', meta.regions], ['vino kartoituskuva', 'ref2', meta.regions2]]) {
    const img = { data: new Uint8ClampedArray(fs.readFileSync(path.join(dir, file + '.rgba'))), width: meta.W, height: meta.H };
    let before = 0, after = 0, n = 0, worse = 0, ms = 0;
    for (const reg of regs) for (let trial = 0; trial < 4; trial++) {
      const truth = reg.poly.map(([x, y]) => [x * meta.W, y * meta.H]);
      // käyttäjän piirtämä suorakulmio: oven rajaava laatikko, siirretty ja skaalattu satunnaisesti
      const xs = truth.map(p => p[0]), ys = truth.map(p => p[1]);
      let x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
      const w = x1 - x0, h = y1 - y0, sc = 1 + (rand() - 0.5) * 0.12, dx = (rand() - 0.5) * 0.12 * w, dy = (rand() - 0.5) * 0.12 * h;
      const cx = (x0 + x1) / 2 + dx, cy = (y0 + y1) / 2 + dy;
      x0 = cx - w * sc / 2; x1 = cx + w * sc / 2; y0 = cy - h * sc / 2; y1 = cy + h * sc / 2;
      const user = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
      // rajaus marginaalilla kuten sovelluksessa
      const m = 0.25 * Math.max(w, h);
      const cx0 = Math.max(0, Math.floor(x0 - m)), cy0 = Math.max(0, Math.floor(y0 - m));
      const cx1 = Math.min(meta.W, Math.ceil(x1 + m)), cy1 = Math.min(meta.H, Math.ceil(y1 + m));
      const t0 = Date.now();
      const r = lib.snapQuad(cv, crop(img, cx0, cy0, cx1 - cx0, cy1 - cy0), user.map(([x, y]) => [x - cx0, y - cy0]));
      ms += Date.now() - t0;
      const res = r.quad.map(([x, y]) => [x + cx0, y + cy0]);
      const e0 = err(user, truth), e1 = err(res, truth);
      before += e0; after += e1; n++; if (e1 > e0 + 1) worse++;
    }
    console.log(`${name}: kulmien keskivirhe ${(before / n).toFixed(1)} px -> ${(after / n).toFixed(1)} px (${n} ovea, huonompi ${worse}, ${(ms / n).toFixed(0)} ms/ovi)`);
  }
})();
