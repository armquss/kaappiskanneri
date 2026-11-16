/* Tunnistus erillisessä säikeessä, jotta kamerakuva ja käyttöliittymä pysyvät sulavina. */
importScripts('opencv.js', 'vision.js');

let cv = null, engine = null;

async function ensureCv() {
  if (cv) return cv;
  let c = self.cv;
  if (c && typeof c.then === 'function' && !c.Mat) c = await c;
  else if (c && !c.Mat) await new Promise(r => (c.onRuntimeInitialized = r));
  cv = c;
  return cv;
}

const handlers = {
  async init({ params }) {
    await ensureCv();
    if (engine) engine.clear();
    engine = KaappiVision.createEngine(cv, params || {});
    return { ok: true };
  },
  addView({ viewId, img, regions }) { return engine.addView(viewId, img, regions); },
  setRegions({ viewId, regions }) { return engine.setRegions(viewId, regions); },
  removeView({ viewId }) { engine.removeView(viewId); return true; },
  quality({ viewId }) { return engine.viewQuality(viewId); },
  transfer({ from, to }) { return engine.transferRegions(from, to); },
  reset() { engine.resetTracking(); return true; },
  frame({ img, debug, t }) { return engine.processFrame(img, { debug, t }); }
};

// Viestit käsitellään järjestyksessä (init ehtii valmiiksi ennen muita)
let queue = Promise.resolve();
self.onmessage = (e) => {
  const m = e.data;
  queue = queue.then(async () => {
    try {
      if (!engine && m.type !== 'init') throw new Error('engine not ready');
      const result = await handlers[m.type](m);
      self.postMessage({ id: m.id, result });
    } catch (err) {
      self.postMessage({ id: m.id, error: String(err && err.message || err) });
    }
  });
};
