/*
 * Kaappiskanneri – tunnistusmoottori.
 * Toimii sekä Web Workerissa (importScripts) että Nodessa (require) testejä varten.
 *
 * Periaate:
 *  - Jokainen "näkymä" on kauempaa otettu kuva kaappiseinästä, johon on merkitty ovien alueet.
 *  - Kamerakuvasta lasketaan ORB-piirteet, ne verrataan näkymien piirteisiin ja RANSAC-homografialla
 *    selvitetään, mistä kohtaa seinää kamera katsoo. Koska vertailu tehdään koko kohtaukseen
 *    (laatat, kodinkoneet, kahvat, ikkunat…), samannäköisetkin ovet erottuvat toisistaan.
 *  - Ruudun keskipiste (tähtäin) projisoidaan näkymään -> osuttu alue = kaappi.
 *  - Kun piirretunnistus ei onnistu (esim. kamera aivan oven edessä), sijaintia jatketaan
 *    optisella vuolla edellisestä kuvasta (Lucas–Kanade).
 */
(function (root) {
  'use strict';

  // ---------- pieni 3x3-matriisikirjasto (rivipääjärjestys, 9 alkiota) ----------
  function mul(a, b) {
    const r = new Array(9);
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++)
        r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    return r;
  }
  function inv(m) {
    const [a, b, c, d, e, f, g, h, i] = m;
    const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const det = a * A + b * B + c * C;
    if (!isFinite(det) || Math.abs(det) < 1e-12) return null;
    const k = 1 / det;
    return [A * k, -(b * i - c * h) * k, (b * f - c * e) * k,
            B * k, (a * i - c * g) * k, -(a * f - c * d) * k,
            C * k, -(a * h - b * g) * k, (a * e - b * d) * k];
  }
  function apply(m, x, y) {
    const w = m[6] * x + m[7] * y + m[8];
    if (Math.abs(w) < 1e-9) return null;
    return [(m[0] * x + m[1] * y + m[2]) / w, (m[3] * x + m[4] * y + m[5]) / w];
  }
  function polyArea(p) { // p = [[x,y],...]
    let s = 0;
    for (let i = 0; i < p.length; i++) {
      const [x1, y1] = p[i], [x2, y2] = p[(i + 1) % p.length];
      s += x1 * y2 - x2 * y1;
    }
    return s / 2;
  }
  function isConvex(p) {
    let sign = 0;
    for (let i = 0; i < p.length; i++) {
      const a = p[i], b = p[(i + 1) % p.length], c = p[(i + 2) % p.length];
      const cr = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
      if (Math.abs(cr) < 1e-9) continue;
      const s = cr > 0 ? 1 : -1;
      if (sign === 0) sign = s; else if (s !== sign) return false;
    }
    return true;
  }
  function pointInPoly(x, y, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
      if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) inside = !inside;
    }
    return inside;
  }
  function distToPoly(x, y, poly) {
    let best = Infinity;
    for (let i = 0; i < poly.length; i++) {
      const [ax, ay] = poly[i], [bx, by] = poly[(i + 1) % poly.length];
      const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy || 1;
      let t = ((x - ax) * dx + (y - ay) * dy) / l2; t = Math.max(0, Math.min(1, t));
      const px = ax + t * dx - x, py = ay + t * dy - y;
      best = Math.min(best, Math.hypot(px, py));
    }
    return best;
  }

  // Ovitaso: ovialueet laajennettuna marginaalilla (oven reunat, raot ja kahvat mukaan)
  function expandPoly(poly, m) {
    const cx = poly.reduce((a, p) => a + p[0], 0) / poly.length, cy = poly.reduce((a, p) => a + p[1], 0) / poly.length;
    return poly.map(([x, y]) => [cx + (x - cx) * (1 + 2 * m), cy + (y - cy) * (1 + 2 * m)]);
  }
  const inZone = (zone, x, y) => { for (const z of zone) if (pointInPoly(x, y, z)) return true; return false; };
  // Pisteiden levinneisyys (osuus kuvan leveydestä ja korkeudesta) – liian tiivis joukko antaa epävakaan homografian
  function spread(pts, w, h) {
    if (pts.length < 4) return 0;
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let i = 0; i < pts.length; i += 2) { x0 = Math.min(x0, pts[i]); x1 = Math.max(x1, pts[i]); y0 = Math.min(y0, pts[i + 1]); y1 = Math.max(y1, pts[i + 1]); }
    return Math.min((x1 - x0) / w, (y1 - y0) / h);
  }

  // Robusti 2D-similariteetti (siirto, kierto, mittakaava) p -> q: RANSAC kahden pisteen näytteillä + PNS-tarkennus
  function fitSimilarity(p, q, thr, iters) {
    const n = p.length / 2; if (n < 3) return null;
    const ls = idx => {
      let px = 0, py = 0, qx = 0, qy = 0; for (const i of idx) { px += p[i * 2]; py += p[i * 2 + 1]; qx += q[i * 2]; qy += q[i * 2 + 1]; }
      const m = idx.length; px /= m; py /= m; qx /= m; qy /= m;
      let a = 0, b = 0, nn = 0;
      for (const i of idx) { const ax = p[i * 2] - px, ay = p[i * 2 + 1] - py, bx = q[i * 2] - qx, by = q[i * 2 + 1] - qy; a += ax * bx + ay * by; b += ax * by - ay * bx; nn += ax * ax + ay * ay; }
      if (nn < 1e-6) return null;
      const c = a / nn, s = b / nn; // c = s·cosθ, s = s·sinθ
      return [c, -s, qx - (c * px - s * py), s, c, qy - (s * px + c * py), 0, 0, 1];
    };
    const inl = S => { const r = []; for (let i = 0; i < n; i++) { const x = S[0] * p[i * 2] + S[1] * p[i * 2 + 1] + S[2], y = S[3] * p[i * 2] + S[4] * p[i * 2 + 1] + S[5]; if (Math.hypot(x - q[i * 2], y - q[i * 2 + 1]) < thr) r.push(i); } return r; };
    let best = null, bestIn = [];
    let seed = 12345; const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
    for (let it = 0; it < (iters || 120); it++) {
      const i = Math.floor(rnd() * n), j = Math.floor(rnd() * n); if (i === j) continue;
      if (Math.hypot(p[i * 2] - p[j * 2], p[i * 2 + 1] - p[j * 2 + 1]) < 8) continue;
      const S = ls([i, j]); if (!S) continue;
      const I = inl(S); if (I.length > bestIn.length) { best = S; bestIn = I; }
    }
    if (!best || bestIn.length < 3) return null;
    const S2 = ls(bestIn); if (!S2) return null;
    const I2 = inl(S2);
    return { S: S2, inliers: I2.length, n };
  }

  // Näkyykö kartoituskuvasta jotain ruudulla: 5x5 pisteruudukko kartoituskuvasta ruudulle
  // Näkyykö kartoituskuvasta jotain ruudulla: ruudun ala kartoituskuvassa leikkaa kartoituskuvan suorakulmiota
  function viewVisible(H, fw, fh, v) {
    const fp = [[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(H, x, y));
    if (fp.some(p => !p)) return false;
    const inRect = ([x, y]) => x >= 0 && x <= v.w && y >= 0 && y <= v.h;
    if (fp.some(inRect)) return true;
    const vr = [[0, 0], [v.w, 0], [v.w, v.h], [0, v.h]];
    if (vr.some(([x, y]) => pointInPoly(x, y, fp))) return true;
    const cross = (a, b, c, d) => { const o = (p, q, r) => Math.sign((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])); return o(a, b, c) !== o(a, b, d) && o(c, d, a) !== o(c, d, b); };
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) if (cross(fp[i], fp[(i + 1) % 4], vr[j], vr[(j + 1) % 4])) return true;
    return false;
  }

  const POP = new Uint8Array(256);
  for (let i = 0; i < 256; i++) POP[i] = (i & 1) + POP[i >> 1];
  const CELL = 48;

  const DEFAULTS = {
    refMax: 1600,        // näkymäkuvan pidempi sivu sisäisesti (px)
    refFeatures: 4000,   // ORB-piirteitä per näkymä
    frameFeatures: 800,  // ORB-piirteitä per kamerakuva
    // Globaali haku monessa mittakaavassa: kun koko seinä näkyy, kaapit ovat kamerakuvassa
    // moninkertaisesti pienempiä kuin kartoituskuvassa. [mittakaava, piirteitä]
    globalScales: [[1, 1200], [0.6, 900], [0.35, 600], [0.2, 400]],
    earlyStopInliers: 50, // globaali haku lopetetaan, kun näin vahva osuma löytyy
    refineR: 0.02,        // globaalin osuman tarkennus ohjatulla haulla (säde, osuus leveydestä)
    frameMax: 640,       // kamerakuvan käsittelykoko (pidempi sivu)
    ratio: 0.8,          // Lowen suhdetesti
    multi: false,        // toistuvien rakenteiden kaksoisehdokkaat
    maxDist: 64,         // Hamming-etäisyyden yläraja
    fast: 12,            // FAST-kulmakynnys
    clahe: true,         // paikallinen kontrastin tasaus (valaistuserot)
    preBlur: 0,          // esisumennus (kohina)
    ransacPx: 5,         // RANSAC-kynnys näkymäpikseleinä
    minInliers: 15,      // hyväksytyn tunnistuksen vähimmäisinlierit
    minInlierRatio: 0.2,
    detectEvery: 6,      // vahvan seurannan aikana tunnistus joka n:s kuva
    detectEveryWeak: 3,  // heikon seurannan aikana (esim. lähikuva)
    skipDetectInliers: 60,
    minTrackPts: 60,     // tätä vähemmillä seurantapisteillä haetaan uudet pisteet
    trackMinPts: 10,
    maxTrackFrames: 400, // kuinka monta kuvaa pelkällä seurannalla saa jatkaa ilman tunnistusta
    fullScanEvery: 10,   // lukittunakin tarkistetaan muut näkymät joka n:s kuva
    switchFrames: 2,     // monta peräkkäistä kuvaa tarvitaan kaapin vaihtoon
    holdFrames: 6,       // kuinka monta kuvaa kaappi pidetään näkyvissä katoamisen jälkeen
    edgeTolerance: 0.015, // tähtäin saa olla näin lähellä (normalisoitu) alueen reunaa
    guideR: 0.045,       // ohjatun haun säde (osuus näkymän leveydestä)
    minGuidedInliers: 12,
    reacquireFrames: 8,  // kuinka monta kuvaa katoamisen jälkeen yritetään ohjattua hakua
    reacquireMs: 1500,   // …ja enintään näin kauan (ms)
    kfRadius: 0.06,      // avainkuvahaun säde (osuus näkymän leveydestä)
    kfRadiusLoose: 0.12, // …kun sijainti on hetken ollut hukassa
    kfTry: 3, kfMinInliers: 30,
    // Oppiva kartoitus: avainkuvat lokeroidaan paikan, mittakaavan ja valaistuksen mukaan
    kfGridX: 12, kfGridY: 8, kfMaxPerView: 100,
    kfLevel1Inliers: 80,  // avainkuvasta johdettu (taso 1) avainkuva vain vahvalla osumalla
    kfCtxFactor: 3,       // kontekstirikas = kuva-ala >= 3 x suurimman näkyvän oven ala
    kfGlobalTry: 4, kfGlobalMinInliers: 40, kfGlobalStrong: 80,
    kfOutsideMin: 12, kfOutsideFrac: 0.2, // globaalissa avainkuvahaussa näyttöä myös ovien ulkopuolelta
    strongInliers: 60, maxShift: 0.02, maxScaleShift: 0.15,
    // Tasotietoisuus: sijainti sovitetaan ensisijaisesti ovien tason piirteisiin (huonekalut, katto ja
    // syvennetty hylly ovat eri syvyydellä ja vääristäisivät rajoja lähellä ja vinosta katsottaessa)
    planeMargin: 0.08, planeMinPts: 12, planeMinSpread: 0.12, planeGain: 1.0, looseMinSpread: 0.2, suspectMisses: 2,
    simMinPts: 6, simThr: 3,  // ennusteen korjaus ovialueen pisteistä (similariteetti)
    kfTryTracking: 2,     // seurannan aikana kokeiltavat avainkuvat (kevyempi)
    kfMaxBad: 3, kfVerifyMin: 25,
    kfPersistMin: 30,     // tason 0 avainkuvan vähimmäisosumat (heikko ankkuri ei jää muistiin)
    fastMotion: 0.035,   // kuvan kulmien siirtymä (osuus leveydestä) per kuva, jonka yli kamera "liikkuu nopeasti"
  };

  function createEngine(cv, options) {
    const P = Object.assign({}, DEFAULTS, options || {});
    const orbRef = new cv.ORB(P.refFeatures);
    const orbFrame = new cv.ORB(P.frameFeatures);
    try { orbFrame.setFastThreshold(P.fast); orbRef.setFastThreshold(P.fast); } catch (e) {}
    const matcher = new cv.BFMatcher(cv.NORM_HAMMING, false);
    const clahe = new cv.CLAHE(2.0, new cv.Size(8, 8));
    const emptyMask = new cv.Mat();

    const views = new Map();
    let kfs = [];          // avainkuvat (säilyvät seurannan nollauksen yli; osa tallennetaan laitteelle)
    let kfEvents = [];     // tallennettavat muutokset pääsäikeelle: {add} / {remove}
    let kfSeq = 0;
    let st = freshState();

    function freshState() {
      return {
        frameNo: 0, viewId: null, H: null, mode: 'lost', trackAge: 0,
        prevGray: null, prevPts: null,
        stableCab: null, candCab: null, candCount: 0, lostCount: 0, lastDetectFrame: -1, lastFullScan: -99
      };
    }
    function resetTracking() {
      if (st.prevGray) st.prevGray.delete();
      if (st.lastGood) st.lastGood.desc.delete();
      st = freshState();
    }

    // RGBA (Uint8ClampedArray) -> harmaasävy Mat, skaalattuna niin että pidempi sivu <= maxSide
    function toGray(img, maxSide) {
      let gray;
      if (img.gray) { // valmiiksi harmaasävy (seurantasäikeeltä)
        gray = new cv.Mat(img.height, img.width, cv.CV_8UC1); gray.data.set(img.data);
      } else {
        const rgba = new cv.Mat(img.height, img.width, cv.CV_8UC4);
        rgba.data.set(img.data);
        gray = new cv.Mat();
        cv.cvtColor(rgba, gray, cv.COLOR_RGBA2GRAY);
        rgba.delete();
      }
      let scale = 1;
      const long = Math.max(img.width, img.height);
      if (maxSide && long > maxSide) {
        scale = maxSide / long;
        const small = new cv.Mat();
        cv.resize(gray, small, new cv.Size(Math.round(img.width * scale), Math.round(img.height * scale)), 0, 0, cv.INTER_AREA);
        gray.delete(); gray = small;
      }
      if (P.preBlur) cv.GaussianBlur(gray, gray, new cv.Size(0, 0), P.preBlur);
      if (!P.clahe) return { gray, scale };
      const eq = new cv.Mat();
      clahe.apply(gray, eq);
      gray.delete();
      return { gray: eq, scale };
    }

    function detect(orb, gray) {
      const kp = new cv.KeyPointVector();
      const desc = new cv.Mat();
      orb.detectAndCompute(gray, emptyMask, kp, desc);
      const n = kp.size();
      const pts = new Float32Array(n * 2);
      const resp = new Float32Array(n);
      for (let i = 0; i < n; i++) {
        const k = kp.get(i);
        pts[i * 2] = k.pt.x; pts[i * 2 + 1] = k.pt.y; resp[i] = k.response;
      }
      kp.delete();
      return { pts, resp, desc, n };
    }

    // Hajautuneisuus: monessako 6x4-ruudukon solussa on piirteitä
    function coverage(pts, w, h) {
      const cells = new Set();
      for (let i = 0; i < pts.length; i += 2)
        cells.add(Math.min(5, Math.floor(pts[i] / w * 6)) + ',' + Math.min(3, Math.floor(pts[i + 1] / h * 4)));
      return cells.size / 24;
    }

    function regionsToPx(v) {
      v.regionsPx = (v.regions || []).map(r => ({
        cabinetId: r.cabinetId,
        poly: r.poly.map(([x, y]) => [x * v.w, y * v.h])
      }));
      v.zone = v.regionsPx.map(r => expandPoly(r.poly, P.planeMargin));
    }

    function buildGlobalSet(gray) {
      const allPts = [], descs = [];
      for (const [sc, nf] of P.globalScales) {
        let g = gray;
        if (sc !== 1) { g = new cv.Mat(); cv.resize(gray, g, new cv.Size(Math.round(gray.cols * sc), Math.round(gray.rows * sc)), 0, 0, cv.INTER_AREA); }
        const o = new cv.ORB(nf); try { o.setFastThreshold(P.fast); } catch (e) {}
        const ff = detect(o, g); o.delete();
        for (let i = 0; i < ff.n; i++) allPts.push(ff.pts[i * 2] / sc, ff.pts[i * 2 + 1] / sc);
        descs.push(ff.desc);
        if (g !== gray) g.delete();
      }
      const n = allPts.length / 2, M = new cv.Mat(n, 32, cv.CV_8U);
      let off = 0; for (const d of descs) { if (d.rows) M.data.set(d.data, off); off += d.rows * 32; d.delete(); }
      return { pts: new Float32Array(allPts), desc: M, n };
    }

    function addView(id, img, regions) {
      removeView(id);
      const { gray, scale } = toGray(img, P.refMax);
      const f = detect(orbRef, gray);
      const v = { id, w: gray.cols, h: gray.rows, scale, pts: f.pts, desc: f.desc, n: f.n, regions: regions || [] };
      v.descArr = new Uint8Array(f.desc.data); v.dw = f.desc.cols || 32;
      v.gw = Math.ceil(v.w / CELL); v.gh = Math.ceil(v.h / CELL); v.grid = new Array(v.gw * v.gh);
      for (let i = 0; i < f.n; i++) {
        const k = Math.min(v.gh - 1, Math.floor(f.pts[i * 2 + 1] / CELL)) * v.gw + Math.min(v.gw - 1, Math.floor(f.pts[i * 2] / CELL));
        (v.grid[k] || (v.grid[k] = [])).push(i);
      }
      v.coverage = coverage(f.pts, v.w, v.h);
      regionsToPx(v);
      v.g = P.globalScales ? buildGlobalSet(gray) : null;
      gray.delete();
      views.set(id, v);
      return { id, features: v.n, coverage: v.coverage, w: v.w, h: v.h };
    }
    function setRegions(id, regions) {
      const v = views.get(id); if (!v) return false;
      v.regions = regions || []; regionsToPx(v);
      for (const k of kfs) if (k.viewId === id) k.ctx = isCtx(k, v);
      return true;
    }
    function removeView(id) {
      const v = views.get(id);
      if (v) { v.desc.delete(); if (v.g) v.g.desc.delete(); views.delete(id); }
      kfs = kfs.filter(k => { if (k.viewId === id) { k.desc.delete(); return false; } return true; });
      if (st.viewId === id) resetTracking();
    }
    function clear() { for (const id of [...views.keys()]) removeView(id); resetTracking(); for (const k of kfs) k.desc.delete(); kfs = []; for (const k of provisional) k.desc.delete(); provisional = []; }

    // Vertaa piirteitä (query -> train) ja laske homografia query-koordinaateista train-koordinaatteihin
    function matchHomography(q, t, ransacPx) {
      if (q.n < 8 || t.n < 8 || q.desc.rows === 0 || t.desc.rows === 0) return null;
      const mv = new cv.DMatchVectorVector();
      matcher.knnMatch(q.desc, t.desc, mv, 2);
      const src = [], dst = [];
      const usedTrain = new Map();
      for (let i = 0; i < mv.size(); i++) {
        const m = mv.get(i);
        if (m.size() < 2) { m.delete(); continue; }
        const a = m.get(0), b = m.get(1);
        const put = (d) => {
          const prev = usedTrain.get(d.trainIdx); // yksi train-piste vain kerran (paras)
          if (prev === undefined || prev.d > d.distance) usedTrain.set(d.trainIdx, { d: d.distance, q: d.queryIdx });
        };
        if (a.distance < P.maxDist) {
          if (a.distance < P.ratio * b.distance) put(a);
          else if (P.multi && b.distance < P.maxDist) {
            // Toistuva rakenne (identtiset ovet): molemmat ehdokkaat RANSACille,
            // ympäristön piirteet ratkaisevat kumpi on oikea.
            put(a); put(b);
          }
        }
        m.delete();
      }
      mv.delete();
      for (const [ti, o] of usedTrain) {
        src.push(q.pts[o.q * 2], q.pts[o.q * 2 + 1]);
        dst.push(t.pts[ti * 2], t.pts[ti * 2 + 1]);
      }
      return ransac(src, dst, ransacPx, P.minInliers);
    }

    function ransac(src, dst, ransacPx, minN) {
      const nGood = src.length / 2;
      if (nGood < minN) return { good: nGood, inliers: 0, H: null };
      const sM = cv.matFromArray(nGood, 1, cv.CV_32FC2, src);
      const dM = cv.matFromArray(nGood, 1, cv.CV_32FC2, dst);
      const mask = new cv.Mat();
      const Hm = cv.findHomography(sM, dM, cv.RANSAC, ransacPx, mask, 2000, 0.995);
      let H = null, inliers = 0; const inlierPts = [], ps = [], pd = [];
      if (!Hm.empty()) {
        H = Array.from(Hm.data64F);
        for (let i = 0; i < nGood; i++) if (mask.data[i]) {
          inliers++; ps.push(src[i * 2], src[i * 2 + 1]); pd.push(dst[i * 2], dst[i * 2 + 1]);
          if (inlierPts.length < 300) inlierPts.push(src[i * 2], src[i * 2 + 1]);
        }
      }
      sM.delete(); dM.delete(); mask.delete(); Hm.delete();
      return { good: nGood, inliers, H, inlierPts, pairs: { s: ps, d: pd } };
    }

    /**
     * Ohjattu haku: kun sijainti tiedetään suunnilleen (seuranta tai edellinen kuva), verrataan
     * kunkin kamerapiirteen kuvailijaa vain ennustetun paikan lähellä oleviin näkymän piirteisiin.
     * Toistuvat rakenteet (identtiset ovet) eivät tällöin sekoita, koska naapuriovi on haun ulkopuolella.
     */
    function guidedMatch(f, v, Hp, R) {
      if (!f || f.n < 8) return null;
      const fd = new Uint8Array(f.desc.data), rd = v.descArr, dw = v.dw, R2 = R * R;
      const rc = Math.ceil(R / CELL);
      const used = new Map();
      for (let i = 0; i < f.n; i++) {
        const p = apply(Hp, f.pts[i * 2], f.pts[i * 2 + 1]);
        if (!p || p[0] < -R || p[1] < -R || p[0] > v.w + R || p[1] > v.h + R) continue;
        const cx = Math.floor(p[0] / CELL), cy = Math.floor(p[1] / CELL);
        let b1 = 999, b2 = 999, bi = -1;
        for (let gy = Math.max(0, cy - rc); gy <= Math.min(v.gh - 1, cy + rc); gy++)
          for (let gx = Math.max(0, cx - rc); gx <= Math.min(v.gw - 1, cx + rc); gx++) {
            const cell = v.grid[gy * v.gw + gx]; if (!cell) continue;
            for (let c = 0; c < cell.length; c++) {
              const j = cell[c], dx = v.pts[j * 2] - p[0], dy = v.pts[j * 2 + 1] - p[1];
              if (dx * dx + dy * dy > R2) continue;
              let d = 0; const o1 = i * dw, o2 = j * dw;
              for (let k = 0; k < dw; k++) d += POP[fd[o1 + k] ^ rd[o2 + k]];
              if (d < b1) { b2 = b1; b1 = d; bi = j; } else if (d < b2) b2 = d;
            }
          }
        if (bi >= 0 && b1 < P.maxDist && b1 < 0.9 * b2) {
          const prev = used.get(bi);
          if (prev === undefined || prev.d > b1) used.set(bi, { d: b1, q: i });
        }
      }
      const src = [], dst = [];
      for (const [ti, o] of used) { src.push(f.pts[o.q * 2], f.pts[o.q * 2 + 1]); dst.push(v.pts[ti * 2], v.pts[ti * 2 + 1]); }
      return ransac(src, dst, P.ransacPx * v.w / 1000, P.minGuidedInliers);
    }

    // Onko homografia (kamerakuva -> näkymä) järkevä?
    function plausible(H, fw, fh, v) {
      if (!H) return false;
      if (H[0] * H[4] - H[1] * H[3] <= 0) return false; // peilikuva
      const corners = [[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(H, x, y));
      if (corners.some(c => !c || !isFinite(c[0]) || !isFinite(c[1]))) return false;
      if (!isConvex(corners)) return false;
      const a = Math.abs(polyArea(corners)), va = v.w * v.h;
      if (a < va * 0.0008 || a > va * 40) return false;
      // Ruudun keskipiste saa osua kartoitetun alueen ulkopuolelle (esim. kamera lattiaan, kaapit
      // ruudun yläosassa), kunhan osa kartoituskuvasta näkyy ruudulla.
      const c = apply(H, fw / 2, fh / 2);
      if (!c || c[0] < -3 * v.w || c[0] > 4 * v.w || c[1] < -3 * v.h || c[1] > 4 * v.h) return false;
      if (!viewVisible(H, fw, fh, v)) return false;
      // ei liian rajua perspektiiviä: sivujen pituussuhteet
      const L = i => Math.hypot(corners[i][0] - corners[(i + 1) % 4][0], corners[i][1] - corners[(i + 1) % 4][1]);
      const r1 = L(0) / L(2), r2 = L(1) / L(3);
      if (r1 > 4 || r1 < 0.25 || r2 > 4 || r2 < 0.25) return false;
      return true;
    }

    // Seuranta edellisestä kuvasta nykyiseen (LK). Palauttaa homografian nykyinen -> näkymä.
    function track(gray) {
      if (!st.prevGray || !st.prevPts || st.prevPts.length < P.trackMinPts * 2 || !st.H) return null;
      const n = st.prevPts.length / 2;
      const p0 = cv.matFromArray(n, 1, cv.CV_32FC2, Array.from(st.prevPts));
      // Ennuste: oletetaan että liike jatkuu kuten edellisellä askeleella (vakionopeus)
      let flags = 0, p1;
      let Mi = st.motion ? inv(st.motion) : null;
      if (Mi && st.dtPrev > 0 && st.dtNow > 0) {
        // vakionopeusennuste skaalattuna todelliseen aikaväliin (1. kertaluvun approksimaatio)
        const k = Math.max(0.3, Math.min(3, st.dtNow / st.dtPrev));
        Mi = Mi.map((x, i) => (i === 0 || i === 4 || i === 8 ? 1 : 0) + (x - (i === 0 || i === 4 || i === 8 ? 1 : 0)) * k);
      }
      if (Mi) {
        const guess = new Array(n * 2);
        for (let i = 0; i < n; i++) {
          const q = apply(Mi, st.prevPts[i * 2], st.prevPts[i * 2 + 1]) || [st.prevPts[i * 2], st.prevPts[i * 2 + 1]];
          guess[i * 2] = q[0]; guess[i * 2 + 1] = q[1];
        }
        p1 = cv.matFromArray(n, 1, cv.CV_32FC2, guess); flags = cv.OPTFLOW_USE_INITIAL_FLOW;
      } else p1 = new cv.Mat();
      const status = new cv.Mat(), err = new cv.Mat();
      const crit = new cv.TermCriteria(cv.TermCriteria_COUNT | cv.TermCriteria_EPS, 30, 0.01);
      let out = null;
      try {
        cv.calcOpticalFlowPyrLK(st.prevGray, gray, p0, p1, status, err, new cv.Size(21, 21), 4, crit, flags);
        const cur = [], prev = [];
        for (let i = 0; i < n; i++) {
          if (!status.data[i]) continue;
          const x = p1.data32F[i * 2], y = p1.data32F[i * 2 + 1];
          if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows) continue;
          cur.push(x, y); prev.push(st.prevPts[i * 2], st.prevPts[i * 2 + 1]);
        }
        const m = cur.length / 2;
        if (P.debugLog && m < P.trackMinPts) console.log('LK', st.frameNo, 'n', n, 'tracked only', m);
        if (m >= P.trackMinPts) {
          const cM = cv.matFromArray(m, 1, cv.CV_32FC2, cur);
          const pM = cv.matFromArray(m, 1, cv.CV_32FC2, prev);
          const mask = new cv.Mat();
          const Hm = cv.findHomography(cM, pM, cv.RANSAC, 3, mask, 1000, 0.99);
          if (!Hm.empty()) {
            let inl = 0; const keep = [];
            for (let i = 0; i < m; i++) if (mask.data[i]) { inl++; keep.push(cur[i * 2], cur[i * 2 + 1]); }
            if (P.debugLog) console.log('LK', st.frameNo, 'n', n, 'tracked', m, 'inl', inl, 'pred', !!Mi);
            if (inl >= P.trackMinPts && inl / m > 0.5) {
              const M = Array.from(Hm.data64F); // nykyinen -> edellinen
              out = { H: mul(st.H, M), motion: M, pts: new Float32Array(keep), inliers: inl };
            }
          }
          cM.delete(); pM.delete(); mask.delete(); Hm.delete();
        }
      } finally {
        p0.delete(); p1.delete(); status.delete(); err.delete();
      }
      return out;
    }

    function motionMag(M, fw, fh) {
      let m = 0;
      for (const [x, y] of [[0, 0], [fw, 0], [fw, fh], [0, fh]]) { const p = apply(M, x, y); if (!p) return 1; m = Math.max(m, Math.hypot(p[0] - x, p[1] - y)); }
      return m / fw;
    }

    // Liike-ennusteen järkevyys: ei liian suurta skaalaa/siirtymää yhdessä askeleessa
    function sane(M) {
      const det = M[0] * M[4] - M[1] * M[3];
      return isFinite(det) && det > 0.5 && det < 2 && Math.abs(M[2]) < 200 && Math.abs(M[5]) < 200 && Math.abs(M[6]) < 0.003 && Math.abs(M[7]) < 0.003;
    }

    function pickTrackPts(f, gray) {
      // vahvimmat ORB-pisteet, tasaisesti levitettynä (max 3 per 8x6 solu… n. 250 kpl)
      const idx = Array.from({ length: f.n }, (_, i) => i).sort((a, b) => f.resp[b] - f.resp[a]);
      const cellCount = new Map(), out = [];
      for (const i of idx) {
        const x = f.pts[i * 2], y = f.pts[i * 2 + 1];
        const key = Math.floor(x / gray.cols * 10) + ',' + Math.floor(y / gray.rows * 8);
        const c = cellCount.get(key) || 0;
        if (c >= 4) continue;
        cellCount.set(key, c + 1); out.push(x, y);
        if (out.length >= 600) break;
      }
      return new Float32Array(out);
    }

    function cabinetAt(v, x, y) {
      // x,y näkymäpikseleinä; palauttaa {cabinetId, regionIndex}
      let hit = null, bestArea = Infinity;
      v.regionsPx.forEach((r, i) => {
        if (pointInPoly(x, y, r.poly)) {
          const a = Math.abs(polyArea(r.poly));
          if (a < bestArea) { bestArea = a; hit = { cabinetId: r.cabinetId, regionIndex: i, edge: false }; }
        }
      });
      if (hit) return hit;
      const tol = P.edgeTolerance * Math.max(v.w, v.h);
      let best = null, bd = tol;
      v.regionsPx.forEach((r, i) => {
        const d = distToPoly(x, y, r.poly);
        if (d < bd) { bd = d; best = { cabinetId: r.cabinetId, regionIndex: i, edge: true }; }
      });
      return best;
    }

    /**
     * Käsittele kamerakuva. img = {data: RGBA, width, height} (jo valmiiksi pienennetty, ~640px).
     * Palauttaa tilan: mode, viewId, cabinetId (vakautettu), rawCabinetId, overlay-polygonit (0..1), debug.
     */
    function processFrame(img, opts) {
      opts = opts || {};
      const t0 = Date.now();
      st.frameNo++;
      const now = opts.t != null ? opts.t : Date.now();
      st.dtNow = st.lastT != null ? now - st.lastT : 0;
      const { gray } = toGray(img, P.frameMax);
      const fw = gray.cols, fh = gray.rows;
      // 1) seuranta edellisestä kuvasta (halpa), jos sijainti on tiedossa
      let tracked = null;
      if (st.H && st.viewId && views.has(st.viewId) && st.mode !== 'lost' && st.trackAge < P.maxTrackFrames) {
        tracked = track(gray);
        if (tracked && !plausible(tracked.H, fw, fh, views.get(st.viewId))) { if (P.debugLog) console.log('LK implausible', st.frameNo); tracked = null; }
      }
      // Seuranta on nopea (~10 ms) ja pyörii joka kuvalla. Raskas tunnistus ajetaan vain, kun
      // sijainti on hukassa, seurantapisteet vähenevät tai ajautumista pitää korjata.
      const sinceDetect = st.frameNo - st.lastDetectFrame;
      const strong = tracked && tracked.inliers >= P.skipDetectInliers;
      // Nopean liikkeen aikana raskas tunnistus jätetään väliin: sen viive katkaisisi seurannan.
      const moving = tracked && motionMag(tracked.motion, fw, fh) > P.fastMotion;
      const skipDetect = !opts.forceDetect && tracked &&
        (moving || (tracked.pts.length >= P.minTrackPts * 2 && sinceDetect < (strong ? P.detectEvery : P.detectEveryWeak)));
      const refreshOnly = skipDetect && tracked.pts.length < P.minTrackPts * 2;

      // 2) piirretunnistus
      let best = null, f = null; const rejects = [];
      if (refreshOnly) f = detect(orbFrame, gray); // vain uudet seurantapisteet
      if (!skipDetect) {
        f = detect(orbFrame, gray);
        if (views.size) {
          const lockedRecently = st.viewId && views.has(st.viewId) && st.mode !== 'lost';
          const fullScan = !lockedRecently || (st.frameNo - st.lastFullScan) >= P.fullScanEvery;
          if (fullScan) st.lastFullScan = st.frameNo;
          const cands = fullScan ? [...views.values()] : [views.get(st.viewId)];
          for (const v of cands) {
            const r = matchHomography(f, v.g || v, P.ransacPx * v.w / 1000);
            if (!r || !r.H) { rejects.push(v.id + ':few' + (r ? r.good : 0)); continue; }
            if (r.inliers < P.minInliers || r.inliers / r.good < P.minInlierRatio) { rejects.push(v.id + ':inl' + r.inliers + '/' + r.good); continue; }
            if (!plausible(r.H, fw, fh, v)) { rejects.push(v.id + ':geom' + r.inliers); continue; }
            // pieni etu nykyiselle näkymälle, ettei hyppelehditä
            const score = r.inliers * (v.id === st.viewId ? 1.15 : 1);
            if (!best || score > best.score) best = Object.assign({ v, score }, r);
          }
        }
      }

      // 3) ohjattu haku tunnetun sijainnin ympäriltä (lähikuvat, toistuvat rakenteet)
      if (!best && f) {
        let prior = null, pv = null, R = 0;
        if (tracked) { prior = tracked.H; pv = st.viewId; R = P.guideR; }
        else if (st.recentH && st.frameNo - st.recentFrame <= P.reacquireFrames) { prior = st.recentH; pv = st.recentView; R = P.guideR * 2.5; }
        const v = pv && views.get(pv);
        if (v) {
          const r = guidedMatch(f, v, prior, R * v.w);
          if (r && r.H && r.inliers >= P.minGuidedInliers && r.inliers / r.good >= 0.3 && plausible(r.H, fw, fh, v)) {
            best = Object.assign({ v, score: r.inliers, guided: true }, r);
          } else rejects.push(v.id + ':ohjattu' + (r ? r.inliers + '/' + r.good : 0));
        }
      }

      // 4) palautus viimeisimmästä hyvästä kamerakuvasta (sama mittakaava -> toimii lähikuvissakin)
      if (!best && !tracked && f && !skipDetect && st.lastGood && views.has(st.lastGood.viewId) &&
          st.frameNo - st.lastGood.frameNo <= P.reacquireFrames * 2 && now - st.lastGood.t <= P.reacquireMs) {
        const r = matchHomography(f, st.lastGood, 4);
        const v = views.get(st.lastGood.viewId);
        if (r && r.H && r.inliers >= 20 && r.inliers / r.good >= 0.4) {
          const Hn = mul(st.lastGood.H, r.H);
          if (plausible(Hn, fw, fh, v)) best = { v, score: r.inliers, H: Hn, good: r.good, inliers: r.inliers, inlierPts: r.inlierPts, reloc: true };
        } else rejects.push('palautus' + (r ? r.inliers + '/' + r.good : 0));
      }

      if (f && !refreshOnly) st.lastDetectFrame = st.frameNo;
      let mode = 'lost', H = null, viewId = null;
      if (best && best.reloc) {
        mode = 'track'; H = best.H; viewId = best.v.id; st.trackAge++;
      } else if (best) {
        mode = 'detect'; H = best.H; viewId = best.v.id; st.trackAge = 0;
      } else if (tracked) {
        mode = 'track'; H = tracked.H; viewId = st.viewId; st.trackAge++;
        if (skipDetect) mode = 'track';
      }

      // päivitä seurantatila
      if (st.prevGray) st.prevGray.delete();
      st.prevGray = gray; // säilytetään seuraavaa kuvaa varten
      if (mode === 'detect') st.prevPts = pickTrackPts(f, gray);
      else if (mode === 'track' && !tracked) st.prevPts = pickTrackPts(f, gray); // palautettu sijainti
      else if (mode === 'track') st.prevPts = (f && tracked.pts.length < 250 * 2) ? mergePts(tracked.pts, pickTrackPts(f, gray)) : tracked.pts;
      else st.prevPts = null;
      // liike-ennuste seuraavaa kuvaa varten (kehyskoordinaateissa: nykyinen -> edellinen)
      if (H && st.H && st.viewId === viewId) { const Hi = inv(H); st.motion = Hi ? mul(Hi, st.H) : null; }
      else st.motion = null;
      if (st.motion && !sane(st.motion)) st.motion = null;
      st.H = H; st.viewId = viewId; st.mode = mode;
      if (H) { st.recentH = H; st.recentView = viewId; st.recentFrame = st.frameNo; }
      const nFeat = f ? f.n : 0;
      if (f && H && f.n >= 50) {
        // talteen palautusta varten (piirteet + sijainti)
        if (st.lastGood) st.lastGood.desc.delete();
        st.lastGood = { desc: f.desc, pts: f.pts, n: f.n, H, viewId, frameNo: st.frameNo, t: now };
      } else if (f) f.desc.delete();
      if (st.lastT != null) st.dtPrev = st.dtNow;
      st.lastT = now;

      // 3) tähtäin -> kaappi
      let raw = null, overlay = [], center = null, v = viewId ? views.get(viewId) : null;
      if (v && H) {
        center = apply(H, fw / 2, fh / 2);
        if (center) raw = cabinetAt(v, center[0], center[1]);
        const Hinv = inv(H);
        if (Hinv) {
          overlay = v.regionsPx.map((r, i) => {
            const poly = r.poly.map(([x, y]) => apply(Hinv, x, y));
            if (poly.some(p => !p)) return null;
            return { cabinetId: r.cabinetId, regionIndex: i, poly: poly.map(([x, y]) => [x / fw, y / fh]) };
          }).filter(Boolean);
        }
      }

      // 4) vakautus (hystereesi)
      const rawId = raw ? raw.cabinetId : null;
      if (mode === 'lost' || !raw) {
        st.lostCount++;
        if (st.lostCount > P.holdFrames) st.stableCab = null;
        st.candCab = null; st.candCount = 0;
      } else {
        st.lostCount = 0;
        if (rawId === st.stableCab) { st.candCab = null; st.candCount = 0; }
        else {
          if (rawId === st.candCab) st.candCount++; else { st.candCab = rawId; st.candCount = 1; }
          const need = st.stableCab == null ? 1 : P.switchFrames;
          if (st.candCount >= need) { st.stableCab = rawId; st.candCab = null; st.candCount = 0; }
        }
      }

      return {
        mode, guided: !!(best && best.guided), reloc: !!(best && best.reloc), viewId, cabinetId: st.stableCab, rawCabinetId: rawId, onEdge: !!(raw && raw.edge),
        overlay, frameW: fw, frameH: fh,
        refPoint: center && v ? [center[0] / v.w, center[1] / v.h] : null,
        debug: opts.debug ? {
          features: nFeat, skipped: !!skipDetect, good: best ? best.good : 0, inliers: best ? best.inliers : (tracked ? tracked.inliers : 0), via: best ? (best.reloc ? 'palautus' : best.guided ? 'ohjattu' : 'globaali') : tracked ? 'LK' : '-',
          pts: (best ? best.inlierPts : tracked ? Array.from(tracked.pts.slice(0, 300)) : []).map((x, i) => i % 2 ? x / fh : x / fw),
          trackAge: st.trackAge, rejects, ms: Date.now() - t0
        } : { ms: Date.now() - t0 }
      };
    }

    function mergePts(a, b) {
      const out = new Float32Array(Math.min(a.length + b.length, 1200));
      out.set(a.subarray(0, Math.min(a.length, 600)));
      out.set(b.subarray(0, out.length - Math.min(a.length, 600)), Math.min(a.length, 600));
      return out;
    }

    /** Siirrä alueet näkymästä toiseen (esim. sama seinä eri valaistuksessa). */
    function transferRegions(fromId, toId) {
      const a = views.get(fromId), b = views.get(toId);
      if (!a || !b) return { ok: false, reason: 'missing' };
      const r = matchHomography(b, a, P.ransacPx * 1.5); // b -> a
      if (!r || !r.H || r.inliers < Math.max(25, P.minInliers)) return { ok: false, reason: 'nomatch', inliers: r ? r.inliers : 0 };
      const Hinv = inv(r.H); if (!Hinv) return { ok: false, reason: 'degenerate' };
      const regions = [];
      for (const reg of a.regionsPx) {
        const poly = reg.poly.map(([x, y]) => apply(Hinv, x, y));
        if (poly.some(p => !p)) continue;
        const n = poly.map(([x, y]) => [x / b.w, y / b.h]);
        // pidä vain alueet, jotka osuvat ainakin osittain kuvaan
        if (n.every(([x, y]) => x < 0 || x > 1 || y < 0 || y > 1)) continue;
        regions.push({ cabinetId: reg.cabinetId, poly: n.map(([x, y]) => [Math.max(-0.2, Math.min(1.2, x)), Math.max(-0.2, Math.min(1.2, y))]) });
      }
      return { ok: true, inliers: r.inliers, regions };
    }

    /** Laatuarvio näkymälle: piirteiden määrä, kattavuus ja ovien erottuvuus ympäristön avulla. */
    function viewQuality(id) {
      const v = views.get(id); if (!v) return null;
      let level = 'good';
      if (v.n < 400 || v.coverage < 0.35) level = 'poor';
      else if (v.n < 1000 || v.coverage < 0.55) level = 'ok';
      return { features: v.n, coverage: v.coverage, level };
    }


    /**
     * Ankkurointi (tunnistussäie): laske kamerakuvan sijainti näkymässä. Ei pidä omaa seurantatilaa,
     * vaan saa ennakkotiedon (priorH) seurantasäikeeltä. Tulos koskee juuri tätä kuvaa.
     * opts: { priorH, priorView, full, t }
     */
    // ---------------- oppiva kartoitus ----------------
    // Pieni normalisoitu pikkukuva (16x16): nopea "näyttääkö tämä samalta" -vertailu avainkuvien valintaan.
    function thumbOf(gray) {
      const t = new cv.Mat();
      cv.resize(gray, t, new cv.Size(16, 16), 0, 0, cv.INTER_AREA);
      const v = new Float32Array(256); let m = 0;
      for (let i = 0; i < 256; i++) { v[i] = t.data[i]; m += v[i]; }
      t.delete(); m /= 256;
      let sd = 0; for (let i = 0; i < 256; i++) { v[i] -= m; sd += v[i] * v[i]; }
      sd = Math.sqrt(sd / 256) || 1;
      for (let i = 0; i < 256; i++) v[i] /= sd;
      v.mean = m;
      return v;
    }
    const thumbSim = (a, b) => { let s = 0; for (let i = 0; i < 256; i++) s += a[i] * b[i]; return s / 256; };
    const brightBucket = b => b < 70 ? 0 : b < 135 ? 1 : 2;
    function footprint(H, fw, fh) { return [[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(H, x, y) || [0, 0]); }
    // Kontekstirikas: kuvassa näkyy selvästi muutakin kuin yksi ovi (identtisten ovien sekaantumisen esto)
    function isCtx(k, v) {
      if (!v.regionsPx.length) return k.area >= 0.15 * v.w * v.h;
      const fp = k.fp;
      let maxA = 0;
      for (const r of v.regionsPx) {
        const touches = r.poly.some(([x, y]) => pointInPoly(x, y, fp)) || pointInPoly(k.center[0], k.center[1], r.poly);
        if (touches) maxA = Math.max(maxA, Math.abs(polyArea(r.poly)));
      }
      if (!maxA) { const as = v.regionsPx.map(r => Math.abs(polyArea(r.poly))).sort((a, b) => a - b); maxA = as[as.length >> 1]; }
      return k.area >= P.kfCtxFactor * maxA;
    }
    function bucketOf(viewId, center, area, bright, v) {
      const gx = Math.max(0, Math.min(P.kfGridX - 1, Math.floor(center[0] / v.w * P.kfGridX)));
      const gy = Math.max(0, Math.min(P.kfGridY - 1, Math.floor(center[1] / v.h * P.kfGridY)));
      const sb = Math.round(2 * Math.log2(Math.max(1e-6, area / (v.w * v.h))));
      return `${viewId}|${gx}|${gy}|${sb}|${brightBucket(bright)}`;
    }
    function exportKf(k, v) {
      // H normalisoidaan näkymän koordinaatteihin 0..1, jotta tallennus kestää tarkkuusasetuksen vaihdon
      const Hn = mul([1 / v.w, 0, 0, 0, 1 / v.h, 0, 0, 0, 1], k.H);
      return { id: k.id, viewId: k.viewId, n: k.n, pts: new Float32Array(k.pts), desc: new Uint8Array(k.desc.data), Hn, fw: k.fw, fh: k.fh,
        thumb: new Float32Array(k.thumb), bright: k.bright, inliers: k.inliers, level: k.level, t: k.t };
    }
    // Avainkuva syntyy ensin väliaikaisena. Se otetaan käyttöön ja tallennetaan vasta, kun seuranta
    // vahvistaa sijainnin ovien reunoista (ja korjaa samalla avainkuvan H:n), tai heti, jos näkymässä
    // ei ole ovia, joista vahvistaa.
    let provisional = [];
    function storeKeyframe(best, f, fw, fh, thumb, bright, level, now) {
      const v = best.v, center = apply(best.H, fw / 2, fh / 2);
      if (!center) return null;
      const k = { id: 'kf' + Date.now().toString(36) + (kfSeq++).toString(36), viewId: v.id, desc: f.desc, pts: f.pts, n: f.n, H: best.H,
        fw, fh, thumb, bright, inliers: best.inliers, level, t: now };
      setKfGeom(k, v);
      if (!v.regionsPx.length) { commitKeyframe(k); return k.id; }
      provisional.push(k);
      while (provisional.length > 6) provisional.shift().desc.delete();
      return k.id;
    }
    function setKfGeom(k, v) {
      k.fp = footprint(k.H, k.fw, k.fh); k.center = apply(k.H, k.fw / 2, k.fh / 2) || [0, 0]; k.area = Math.abs(polyArea(k.fp));
      k.bucket = bucketOf(v.id, k.center, k.area, k.bright, v); k.ctx = isCtx(k, v);
    }
    function commitKeyframe(k) {
      const v = views.get(k.viewId); if (!v) { k.desc.delete(); return false; }
      const ex = kfs.findIndex(x => x.bucket === k.bucket);
      if (ex >= 0 && kfs[ex].inliers * (kfs[ex].level < k.level ? 1.3 : 1) * (kfs[ex].verified && !k.verified ? 1.5 : 1) > k.inliers) { k.desc.delete(); return false; }
      if (ex >= 0) { kfEvents.push({ remove: kfs[ex].id }); kfs[ex].desc.delete(); kfs[ex] = k; } else kfs.push(k);
      // enimmäismäärä per näkymä: pudota heikoin (taso 1 ensin)
      const mine = kfs.filter(x => x.viewId === v.id);
      if (mine.length > P.kfMaxPerView) {
        const drop = mine.sort((a, b) => (b.level - a.level) || (a.inliers - b.inliers))[0];
        kfs = kfs.filter(x => x !== drop); kfEvents.push({ remove: drop.id }); drop.desc.delete();
      }
      if (kfs.includes(k)) kfEvents.push({ add: exportKf(k, v) });
      return kfs.includes(k);
    }
    /** Seurannan palaute väliaikaiselle avainkuvalle: vahvistus (ja korjattu H näkymän pikseleinä) tai hylkäys. */
    function keyframeFeedback(id, ok, H) {
      const i = provisional.findIndex(k => k.id === id);
      if (i < 0) return false;
      const k = provisional.splice(i, 1)[0];
      if (!ok) { k.desc.delete(); return false; }
      if (H) { k.H = H; k.verified = true; setKfGeom(k, views.get(k.viewId)); }
      return commitKeyframe(k);
    }
    /** Lataa laitteelle tallennetut avainkuvat (näkymän pitää olla jo lisätty). */
    function addKeyframes(list) {
      let n = 0;
      for (const e of list || []) {
        const v = views.get(e.viewId);
        if (!v || !e.n || !e.desc || e.desc.length !== e.n * 32) continue;
        if (kfs.some(k => k.id === e.id)) continue;
        const desc = new cv.Mat(e.n, 32, cv.CV_8U); desc.data.set(e.desc);
        const H = mul([v.w, 0, 0, 0, v.h, 0, 0, 0, 1], e.Hn);
        const fp = footprint(H, e.fw, e.fh), center = apply(H, e.fw / 2, e.fh / 2), area = Math.abs(polyArea(fp));
        const thumb = new Float32Array(e.thumb);
        const k = { id: e.id, viewId: e.viewId, desc, pts: new Float32Array(e.pts), n: e.n, H, center, area, fp, fw: e.fw, fh: e.fh,
          thumb, bright: e.bright, inliers: e.inliers, level: e.level || 0, t: e.t, bucket: bucketOf(e.viewId, center, area, e.bright, v) };
        k.ctx = isCtx(k, v);
        kfs.push(k); n++;
      }
      return { loaded: n, total: kfs.length };
    }
    function takeKfEvents() { const e = kfEvents; kfEvents = []; return e; }
    function forgetKeyframes(viewId) {
      kfs = kfs.filter(k => { if (!viewId || k.viewId === viewId) { k.desc.delete(); return false; } return true; });
      kfEvents = [];
    }
    function keyframeStats() {
      const byView = {};
      for (const k of kfs) { const b = byView[k.viewId] || (byView[k.viewId] = { n: 0, ctx: 0 }); b.n++; if (k.ctx) b.ctx++; }
      return { total: kfs.length, byView };
    }
    /**
     * Globaali haku opituista avainkuvista (ei ennakkotietoa). Vain kontekstirikkaat avainkuvat, ja osuman
     * pitää nojata myös ovien ulkopuolisiin piirteisiin: muuten identtinen naapuriovi voisi sekoittaa.
     */
    function globalKeyframes(f, fw, fh, thumb, bright, rejects) {
      const cands = kfs.filter(k => k.ctx && views.has(k.viewId))
        .map(k => ({ k, s: thumbSim(thumb, k.thumb) - Math.abs(k.bright - bright) / 400 }))
        .sort((a, b) => b.s - a.s).slice(0, P.kfGlobalTry);
      let best = null;
      for (const { k } of cands) {
        const v = views.get(k.viewId);
        const r = matchHomography(f, k, 4);
        if (!r || !r.H || r.inliers < P.kfGlobalMinInliers || r.inliers / r.good < 0.35) { rejects.push('opittu ' + (r ? r.inliers + '/' + r.good : 0)); continue; }
        const Hn = mul(k.H, r.H);
        if (!plausible(Hn, fw, fh, v)) { rejects.push('opittu geom'); continue; }
        // näyttö ovien ulkopuolelta
        let outside = 0;
        for (let i = 0; i < r.inlierPts.length; i += 2) {
          const p = apply(Hn, r.inlierPts[i], r.inlierPts[i + 1]);
          if (p && !v.regionsPx.some(rg => pointInPoly(p[0], p[1], rg.poly))) outside++;
        }
        const tot = r.inlierPts.length / 2;
        if (v.regionsPx.length && (outside < P.kfOutsideMin || outside / tot < P.kfOutsideFrac)) { rejects.push(`opittu vain ovesta ${outside}/${tot}`); continue; }
        if (!best || r.inliers > best.inliers) best = { v, H: Hn, good: r.good, inliers: r.inliers, inlierPts: r.inlierPts, pairs: r.pairs, toView: k.H, via: 'opittu', kfLevel: (k.level || 0) + 1, ratio: r.inliers / r.good };
        if (best.inliers >= P.kfGlobalStrong) break;
      }
      return best;
    }

    /**
     * Tasotarkennus: sovitetaan homografia uudelleen vain niihin vastaavuuksiin, jotka osuvat ovialueille
     * (laajennettuna). toView muuntaa vastaavuuksien kohdekoordinaatit näkymän koordinaateiksi
     * (kartoituskuva: identiteetti; avainkuva: avainkuvan H).
     */
    function zoneCount(best) {
      const v = best.v; if (!best.pairs || !v.zone) return 0;
      const T = best.toView || null; let n = 0;
      for (let i = 0; i < best.pairs.d.length; i += 2) { const p = T ? apply(T, best.pairs.d[i], best.pairs.d[i + 1]) : [best.pairs.d[i], best.pairs.d[i + 1]]; if (p && inZone(v.zone, p[0], p[1])) n++; }
      return n;
    }
    function planeRefine(best, fw, fh) {
      const v = best.v;
      if (P.planeRefine === false || !best.pairs || !v.zone || !v.zone.length || best.pairs.s.length < P.planeMinPts * 2) return best;
      const T = best.toView || null;
      const ss = [], dd = [];
      for (let i = 0; i < best.pairs.s.length; i += 2) {
        const x = best.pairs.d[i], y = best.pairs.d[i + 1];
        const p = T ? apply(T, x, y) : [x, y];
        if (p && inZone(v.zone, p[0], p[1])) { ss.push(best.pairs.s[i], best.pairs.s[i + 1]); dd.push(x, y); }
      }
      if (P.debugPlane) console.log('taso', best.via, 'pareja', best.pairs.s.length / 2, 'ovialueella', ss.length / 2, 'levinneisyys', spread(ss, fw, fh).toFixed(2));
      if (ss.length / 2 < P.planeMinPts || spread(ss, fw, fh) < P.planeMinSpread) return best;
      const r = ransac(ss, dd, Math.max(2, P.ransacPx * 0.6 * (T ? 1 : v.w / 1000)), P.planeMinPts);
      if (!r.H || r.inliers < P.planeMinPts * 0.8) return best;
      const Hn = T ? mul(T, r.H) : r.H;
      if (!plausible(Hn, fw, fh, v)) return best;
      // Hyväksytään vain, jos ovialueen vastaavuudet todella sopivat tarkennettuun selvästi paremmin.
      // Tasaisessa näkymässä (ei syvyyseroja) koko kuvan homografia on jo oikea ja paremmin tuettu;
      // vähemmillä pisteillä sovitettu taso voisi silloin vain vääristää mittakaavaa ja perspektiiviä.
      const med = HH => {
        const e = [];
        for (let i = 0; i < ss.length; i += 2) {
          const a = apply(HH, ss[i], ss[i + 1]), b = T ? apply(T, dd[i], dd[i + 1]) : [dd[i], dd[i + 1]];
          if (a && b) e.push(Math.hypot(a[0] - b[0], a[1] - b[1]));
        }
        e.sort((x, y) => x - y); return e.length ? e[e.length >> 1] : 1e9;
      };
      const eFull = med(best.H), eRef = med(Hn);
      if (P.debugPlane) console.log('  taso: mediaanivirhe koko', eFull.toFixed(2), 'tarkennettu', eRef.toFixed(2));
      if (!(eFull - eRef > P.planeGain * v.w / 1000 && eRef < eFull * 0.7)) return best;
      return Object.assign({}, best, { H: Hn, plane: r.inliers });
    }

    // Laadunvarmistus avainkuville: jos avainkuvan antama sijainti on ristiriidassa kartoituskuvan kanssa,
    // avainkuvaa epäillään; toistuvasti epäilty avainkuva poistetaan (myös laitteen muistista).
    function blame(k) {
      k.bad = (k.bad || 0) + 1;
      if (k.bad >= P.kfMaxBad && kfs.includes(k)) {
        kfs = kfs.filter(x => x !== k); kfEvents.push({ remove: k.id }); k.desc.delete();
      }
    }
    function verifyKeyframe(best, k, f, fw, fh, rejects) {
      // ristiintarkistus kartoituskuvaa vasten avainkuvan antaman sijainnin ympäriltä (halpa)
      const r = guidedMatch(f, best.v, best.H, P.refineR * 1.5 * best.v.w);
      if (!r || !r.H || r.inliers < P.kfVerifyMin) return best; // kartoituskuva ei näe tätä kohtaa: ei näyttöä suuntaan tai toiseen
      const ref = planeRefine(Object.assign({ v: best.v }, r), fw, fh);
      const area = HH => Math.abs(polyArea([[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(HH, x, y) || [0, 0])));
      const c1 = apply(best.H, fw / 2, fh / 2), c0 = apply(ref.H, fw / 2, fh / 2);
      const dc = c1 && c0 ? Math.hypot(c1[0] - c0[0], c1[1] - c0[1]) / best.v.w : 1;
      const ds = Math.abs(Math.log(area(best.H) / (area(ref.H) || 1)));
      if (dc > 2 * P.maxShift || ds > 2 * P.maxScaleShift) {
        blame(k); rejects.push(`avainkuva ristiriidassa kartoituskuvan kanssa (${(dc * 100).toFixed(1)}%, ${(ds * 100).toFixed(0)}%)`);
        return Object.assign({}, ref, { via: 'ohjattu (tarkistettu)' });
      }
      k.bad = 0;
      return best;
    }

    /**
     * Ankkurin korjaus ennusteeseen nähden: seurannan antama perspektiivi pidetään, ja ankkurilla
     * korjataan vain siirto, kierto ja mittakaava – ja vain ovialueen vastaavuuksista. Lähellä ja
     * vinosta tämä on paljon vakaampi kuin koko homografian laskeminen eri syvyyksillä olevista pisteistä.
     */
    function priorCorrection(best, priorH, fw, fh) {
      const v = best.v;
      if (P.priorCorrect === false || !best.pairs || !v.zone || !v.zone.length) return null;
      const T = best.toView || null, Pi = inv(priorH); if (!Pi) return null;
      const pp = [], ff = [];
      for (let i = 0; i < best.pairs.s.length; i += 2) {
        const d = T ? apply(T, best.pairs.d[i], best.pairs.d[i + 1]) : [best.pairs.d[i], best.pairs.d[i + 1]];
        if (!d || !inZone(v.zone, d[0], d[1])) continue;
        const p = apply(Pi, d[0], d[1]); if (!p) continue;
        pp.push(p[0], p[1]); ff.push(best.pairs.s[i], best.pairs.s[i + 1]);
      }
      if (pp.length / 2 < P.simMinPts) return null;
      const r = fitSimilarity(pp, ff, P.simThr * fw / 360, 150);
      if (!r || r.inliers < P.simMinPts || r.inliers / r.n < 0.5) return null;
      const S = r.S, sc = Math.hypot(S[0], S[3]), rot = Math.abs(Math.atan2(S[3], S[0])) * 180 / Math.PI;
      if (sc < 0.8 || sc > 1.25 || rot > 8 || Math.hypot(S[2] + (S[0] - 1) * fw / 2 + S[1] * fh / 2, S[5] + S[3] * fw / 2 + (S[4] - 1) * fh / 2) > 0.2 * fw) return null;
      const Si = inv(S); if (!Si) return null;
      const Hn = mul(priorH, Si);
      if (!plausible(Hn, fw, fh, v)) return null;
      return Object.assign({}, best, { H: Hn, via: best.via + '~', sim: r.inliers });
    }

    function anchor(img, opts) {
      opts = opts || {};
      const t0 = Date.now(), now = opts.t != null ? opts.t : t0;
      const { gray } = toGray(img, P.frameMax);
      const fw = gray.cols, fh = gray.rows;
      const f = detect(orbFrame, gray);
      const thumb = thumbOf(gray), bright = thumb.mean;
      gray.delete();
      const rejects = [];
      let best = null;
      const pv = opts.priorView && opts.priorH ? views.get(opts.priorView) : null;
      const refine = x => x ? planeRefine(x, fw, fh) : x;
      // Johdonmukaisuus: heikko ankkuri ei saa siirtää varmaa sijaintia paljon (väärä ankkuri
      // aiheuttaisi kasvavan virheen seurannassa). Vahva ankkuri saa korjata.
      const disagree = (H) => {
        const area = HH => Math.abs(polyArea([[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(HH, x, y) || [0, 0])));
        const c1 = apply(H, fw / 2, fh / 2), c0 = apply(opts.priorH, fw / 2, fh / 2);
        const dc = c1 && c0 ? Math.hypot(c1[0] - c0[0], c1[1] - c0[1]) / pv.w : 1;
        const ds = Math.abs(Math.log(area(H) / (area(opts.priorH) || 1)));
        return { bad: dc > P.maxShift || ds > P.maxScaleShift, dc, ds };
      };
      // Epäilty sijainti: ohjattu haku on epäonnistunut useita kertoja peräkkäin, vaikka seuranta jatkuu.
      // Seuranta voi silloin olla lukittunut väärään kohtaan (esim. identtinen naapurikaappi lähikuvassa),
      // joten tehdään myös koko kuvan haku eikä sitä sidota epäiltyyn ennusteeseen.
      const suspect = !!pv && !opts.loose && (st.guidedMiss || 0) >= P.suspectMisses;
      const consistent = (x) => {
        if (!x || !pv || opts.loose || suspect || x.inliers >= P.strongInliers) return true;
        const d = disagree(x.H);
        if (d.bad) rejects.push(`ristiriita ${x.via} ${x.inliers} siirto ${(d.dc * 100).toFixed(1)}% mittak ${(d.ds * 100).toFixed(0)}%`);
        return !d.bad;
      };
      // 1) ohjattu haku ennakkotiedon ympäriltä (halpa, toimii lähikuvissakin)
      if (pv) {
        const r = guidedMatch(f, pv, opts.priorH, P.guideR * pv.w * (opts.loose ? 2.5 : 1));
        // Hukassa (vanha ennuste, laaja hakusäde) toistuva kuvio – laatat, listat – voi antaa siirtyneen
        // osuman. Silloin vaaditaan, että vastaavuudet kattavat riittävän alueen kuvasta.
        const narrow = r && r.H && opts.loose && spread(r.inlierPts, fw, fh) < P.looseMinSpread;
        if (narrow) rejects.push('ohjattu suppea ' + r.inliers);
        if (!narrow && r && r.H && r.inliers >= P.minGuidedInliers && r.inliers / r.good >= 0.3 && plausible(r.H, fw, fh, pv)) {
          const g0 = Object.assign({ v: pv, via: 'ohjattu' }, r);
          const g = (P.priorCorrect && !opts.loose && priorCorrection(g0, opts.priorH, fw, fh)) || refine(g0);
          if (consistent(g)) best = g;
        } else if (!narrow) rejects.push('ohjattu ' + (r ? r.inliers + '/' + r.good : 0));
      }
      // 2) koko kuvan vertailu (raskas): kun sijainti on hukassa tai pyydetään täysi tarkistus
      const guidedOk = !!best;
      if (!best && (!pv || opts.full || opts.loose || suspect)) {
        // viimeksi onnistunut näkymä ensin; haku loppuu heti vahvaan osumaan
        const order = [...views.values()].sort((a, b) => (b.id === st.lastViewId) - (a.id === st.lastViewId));
        for (const v of order) {
          if (best && best.inliers >= P.earlyStopInliers) break;
          const r = matchHomography(f, v.g || v, P.ransacPx * v.w / 1000);
          if (!r || !r.H) { rejects.push(v.id + ':few' + (r ? r.good : 0)); continue; }
          if (r.inliers < P.minInliers || r.inliers / r.good < P.minInlierRatio) { rejects.push(v.id + ':inl' + r.inliers + '/' + r.good); continue; }
          if (!plausible(r.H, fw, fh, v)) { rejects.push(v.id + ':geom' + r.inliers); continue; }
          const score = r.inliers * (pv && v.id === pv.id ? 1.15 : 1);
          if (!best || score > best.score) best = Object.assign({ v, score, via: 'globaali' }, r);
        }
        // tarkennus: ohjattu haku täysresoluutioisiin piirteisiin löydetyn sijainnin ympäriltä
        if (best && best.via === 'globaali') {
          const r2 = guidedMatch(f, best.v, best.H, P.refineR * best.v.w);
          if (r2 && r2.H && r2.inliers >= best.inliers && plausible(r2.H, fw, fh, best.v)) best = Object.assign({ v: best.v, score: r2.inliers, via: 'globaali+' }, r2);
        }
        best = refine(best);
        // opitut avainkuvat: samanlainen valaistus ja kamera kuin nyt -> usein paljon vahvempi osuma
        if (!best || best.inliers < P.earlyStopInliers) {
          const k = refine(globalKeyframes(f, fw, fh, thumb, bright, rejects));
          if (k && (!best || k.inliers > best.inliers)) best = k;
        }
        if (best && best.v === pv && !consistent(best)) best = null;
      }
      // 3) avainkuvat: aiemmin varmasti ankkuroidut kamerakuvat lähellä ennustettua sijaintia.
      //    Lähikuvassa kartoituskuvan tarkkuus ei riitä, mutta samalta etäisyydeltä otettu avainkuva riittää.
      //    Haku rajataan ennusteen ympärille, joten identtinen naapuriovi ei voi sekoittaa.
      //    Ajetaan myös, jos kartoituskuvan tulos hylättiin ristiriitaisena.
      if (!best && pv) {
        const pc = apply(opts.priorH, fw / 2, fh / 2);
        const pArea = Math.abs(polyArea([[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(opts.priorH, x, y) || [0, 0])));
        const R = (opts.loose ? P.kfRadiusLoose : P.kfRadius) * pv.w;
        const cands = kfs.filter(k => k.viewId === pv.id && pc && Math.hypot(k.center[0] - pc[0], k.center[1] - pc[1]) < R)
          .map(k => ({ k, score: Math.hypot(k.center[0] - pc[0], k.center[1] - pc[1]) / R + Math.abs(Math.log(k.area / (pArea || 1))) }))
          .sort((a, b) => a.score - b.score).slice(0, opts.loose ? P.kfTry : P.kfTryTracking);
        let used = null;
        for (const { k } of cands) {
          const r = matchHomography(f, k, 4);
          if (!r || !r.H || r.inliers < P.kfMinInliers || r.inliers / r.good < 0.35) { rejects.push('avainkuva ' + (r ? r.inliers + '/' + r.good : 0)); continue; }
          const Hn = mul(k.H, r.H), c = apply(Hn, fw / 2, fh / 2);
          if (!plausible(Hn, fw, fh, pv) || !c || Math.hypot(c[0] - pc[0], c[1] - pc[1]) > R) { rejects.push('avainkuva geom'); continue; }
          if (!best || r.inliers > best.inliers) { best = { v: pv, H: Hn, good: r.good, inliers: r.inliers, inlierPts: r.inlierPts, pairs: r.pairs, toView: k.H, via: 'avainkuva', kfLevel: (k.level || 0) + 1, ratio: r.inliers / r.good }; used = k; }
          if (r.inliers >= P.kfGlobalStrong) break;
        }
        if (best) best = (P.priorCorrect && !opts.loose && priorCorrection(best, opts.priorH, fw, fh)) || refine(best);
        if (best && !consistent(best)) { if (used) blame(used); best = null; }
        if (best && used) best = verifyKeyframe(best, used, f, fw, fh, rejects);
      }
      // Oppiva kartoitus: tallenna avainkuvaksi, jos ankkurointi on varma. Taso 0 = suoraan kartoituskuvaan,
      // taso 1 = vahvasti tasoon 0 sidottu. Syvempiä ketjuja ei tehdä (virhe ei pääse kertautumaan).
      let kept = false, kfId = null;
      if (best && f.n >= 80 && P.learn !== false && opts.learn !== false) {
        const level = (best.via === 'avainkuva' || best.via === 'opittu') ? best.kfLevel : 0;
        const ratio0 = best.good ? best.inliers / best.good : 0;
        const planeOk = true; // ovialueelliset avainkuvat vahvistetaan ovien reunoista ennen käyttöä (keyframeFeedback)
        const okLevel = level === 0 ? (best.inliers >= P.kfPersistMin && ratio0 >= 0.35 && planeOk) : (level === 1 && best.inliers >= P.kfLevel1Inliers && best.ratio >= 0.5);
        if (okLevel) { kfId = storeKeyframe(best, f, fw, fh, thumb, bright, level, now); kept = !!kfId; }
      }
      if (!kept) f.desc.delete();
      if (best) st.lastViewId = best.v.id;
      st.guidedMiss = !pv || opts.loose ? 0 : guidedOk ? 0 : (st.guidedMiss || 0) + 1;
      if (suspect && best) st.guidedMiss = 0;
      return {
        ok: !!best, H: best ? best.H : null, viewId: best ? best.v.id : null, w: best ? best.v.w : 0, h: best ? best.v.h : 0,
        via: best ? best.via : '-', inliers: best ? best.inliers : 0, good: best ? best.good : 0, plane: best ? best.plane || 0 : 0, kfId,
        features: f.n, rejects, fw, fh, ms: Date.now() - t0, kfCount: kfs.length, spread: best && best.inlierPts ? spread(best.inlierPts, fw, fh) : 0, zonePts: best ? zoneCount(best) : 0
      };
    }

    function viewSize(id) { const v = views.get(id); return v ? { w: v.w, h: v.h } : null; }

    return {
      addView, setRegions, removeView, clear, processFrame, anchor, viewSize, transferRegions, viewQuality, resetTracking,
      addKeyframes, takeKfEvents, forgetKeyframes, keyframeStats, keyframeFeedback,
      get viewCount() { return views.size; }, params: P
    };
  }


  /**
   * Seurantasäie: kevyt, ajetaan jokaiselle kamerakuvalle.
   *  - laskee kuvien välisen liikkeen (Lucas–Kanade + homografia) ja pitää siitä historiaa
   *  - siirtää sijainnin (H: kehys -> näkymä) kuvasta toiseen
   *  - ottaa vastaan tunnistussäikeen ankkureita vanhemmille kuville ja siirtää ne nykyhetkeen
   */
  function createTracker(cv, options) {
    const P = Object.assign({}, DEFAULTS, {
      trackPts: 200, minTrackPts: 80, trackMinPts: 15, maxTrackMs: 20000, planeTrack: false, histLen: 240, anchoredMs: 1500, lostPriorMs: 1500, trackMax: 480,
      planeTrackMin: 20, planeTrackSpread: 0.1, planePts: 120, maxPerspStep: 0.06,
      doorSnap: true, snapRmin: 6, snapRmax: 18, snapEvery: 1,
      doorVerifiedMs: 450, anchorDoorTol: 10, kfConfirmTtl: 20
    }, options || {});
    const clahe = new cv.CLAHE(2.0, new cv.Size(8, 8));
    const gftt = new cv.GFTTDetector(P.trackPts, 0.01, 10, 3, false, 0.04);
    const crit = new cv.TermCriteria(cv.TermCriteria_COUNT | cv.TermCriteria_EPS, 20, 0.02);
    const winSize = new cv.Size(15, 15);
    const orb = new cv.ORB(500); try { orb.setFastThreshold(12); } catch (e) {}
    const bf = new cv.BFMatcher(cv.NORM_HAMMING, false);
    const noMask = new cv.Mat();
    const views = new Map(); // id -> {w, h, regions, regionsPx}
    let st = fresh();

    function fresh() {
      return { prevGray: null, prevPts: null, motion: null, dtPrev: 0, dtNow: 0, lastT: null,
        H: null, viewId: null, hist: [], anchorT: -1e9, lastId: -1,
        stableCab: null, candCab: null, candCount: 0, lostCount: 0, doorT: -1e9, pendingKfs: [] };
    }
    function reset() { if (st.prevGray) st.prevGray.delete(); st = fresh(); }

    function setView(id, w, h, regions) {
      const v = { w, h, regions: regions || [] };
      v.regionsPx = v.regions.map(r => ({ cabinetId: r.cabinetId, poly: r.poly.map(([x, y]) => [x * w, y * h]) }));
      v.zone = v.regionsPx.map(r => expandPoly(r.poly, P.planeMargin));
      views.set(id, v);
      if (st.viewId === id && !w) { st.H = null; st.viewId = null; }
    }
    function removeView(id) { views.delete(id); if (st.viewId === id) { st.H = null; st.viewId = null; } }

    function toGray(img) {
      let gray;
      if (img.gray) { gray = new cv.Mat(img.height, img.width, cv.CV_8UC1); gray.data.set(img.data); }
      else {
        const rgba = new cv.Mat(img.height, img.width, cv.CV_8UC4); rgba.data.set(img.data);
        gray = new cv.Mat(); cv.cvtColor(rgba, gray, cv.COLOR_RGBA2GRAY); rgba.delete();
      }
      if (img.rot) { // kameran kehyksen kierto näytön suuntaan
        const code = { 90: cv.ROTATE_90_CLOCKWISE, 180: cv.ROTATE_180, 270: cv.ROTATE_90_COUNTERCLOCKWISE }[img.rot];
        if (code !== undefined) { const r = new cv.Mat(); cv.rotate(gray, r, code); gray.delete(); gray = r; }
      }
      const long = Math.max(gray.cols, gray.rows);
      if (P.frameMax && long > P.frameMax) {
        const s = P.frameMax / long, small = new cv.Mat();
        cv.resize(gray, small, new cv.Size(Math.round(gray.cols * s), Math.round(gray.rows * s)), 0, 0, cv.INTER_AREA);
        gray.delete(); gray = small;
      }
      const raw = gray; // tunnistukselle lähetetään täysikokoisena ja ilman tasausta (se tekee omansa)
      // seuranta pienemmällä resoluutiolla (nopeampi)
      let small = raw;
      const l2 = Math.max(raw.cols, raw.rows);
      if (P.trackMax && l2 > P.trackMax) {
        const s = P.trackMax / l2; small = new cv.Mat();
        cv.resize(raw, small, new cv.Size(Math.round(raw.cols * s), Math.round(raw.rows * s)), 0, 0, cv.INTER_AREA);
      }
      const eq = new cv.Mat(); clahe.apply(small, eq);
      if (small !== raw) small.delete();
      return { raw, eq };
    }

    // Ovitason alue kehyksessä (maski), kun sijainti tiedetään
    function zoneMask(gray) {
      const v = st.viewId && views.get(st.viewId);
      if (!st.H || !v || !v.zone || !v.zone.length) return null;
      const Hi = inv(st.H); if (!Hi) return null;
      const mask = cv.Mat.zeros(gray.rows, gray.cols, cv.CV_8UC1);
      const mv = new cv.MatVector();
      for (const z of v.zone) {
        const pts = z.map(([x, y]) => apply(Hi, x, y));
        if (pts.some(p => !p)) continue;
        const flat = []; for (const [x, y] of pts) flat.push(Math.round(Math.max(-4 * gray.cols, Math.min(5 * gray.cols, x))), Math.round(Math.max(-4 * gray.rows, Math.min(5 * gray.rows, y))));
        mv.push_back(cv.matFromArray(pts.length, 1, cv.CV_32SC2, flat));
      }
      if (mv.size()) cv.fillPoly(mask, mv, new cv.Scalar(255));
      for (let i = 0; i < mv.size(); i++) mv.get(i).delete();
      mv.delete();
      return mask;
    }
    function corners(gray, existing, planeFirst) {
      // uudet seurantapisteet tasaisesti; vältetään olemassa olevien läheisyyttä.
      // Kun ovien paikka tiedetään, haetaan ensin pisteitä ovien alueelta (ne määräävät rajojen sijainnin).
      const occ = new Set();
      const cell = (x, y) => Math.floor(x / 16) + ',' + Math.floor(y / 16);
      if (existing) for (let i = 0; i < existing.length; i += 2) occ.add(cell(existing[i], existing[i + 1]));
      const out = existing ? Array.from(existing) : [];
      const add = (kp, limit) => {
        for (let i = 0; i < kp.size() && out.length < limit; i++) {
          const p = kp.get(i).pt, c = cell(p.x, p.y);
          if (occ.has(c)) continue;
          occ.add(c); out.push(p.x, p.y);
        }
      };
      const mask = planeFirst ? zoneMask(gray) : null;
      if (mask) {
        const kp = new cv.KeyPointVector();
        gftt.detect(gray, kp, mask); add(kp, Math.min(P.trackPts, out.length / 2 + P.planePts) * 2); kp.delete(); mask.delete();
      }
      const kp = new cv.KeyPointVector();
      gftt.detect(gray, kp); add(kp, P.trackPts * 2); kp.delete();
      return new Float32Array(out);
    }

    function predict() {
      if (P.noPredict) return null;
      let Mi = st.motion ? inv(st.motion) : null;
      if (Mi && st.dtPrev > 0 && st.dtNow > 0) {
        const k = Math.max(0.3, Math.min(3, st.dtNow / st.dtPrev));
        Mi = Mi.map((x, i) => { const d = (i === 0 || i === 4 || i === 8) ? 1 : 0; return d + (x - d) * k; });
      }
      return Mi;
    }

    // Liike edellisestä kuvasta nykyiseen. Palauttaa {M: nykyinen -> edellinen, pts, inliers} tai null.
    // Liike: ensin ennusteen kanssa; jos se epäonnistuu (esim. liike kääntyi), uudestaan ilman ennustetta.
    function flow(gray) {
      const Mi = predict();
      const r = flowOnce(gray, Mi);
      if (r || !Mi) return r;
      return flowOnce(gray, null);
    }
    function flowOnce(gray, Mi) {
      if (!st.prevGray || !st.prevPts || st.prevPts.length < 16 * 2) return null;
      if (st.prevGray.cols !== gray.cols || st.prevGray.rows !== gray.rows) return null;
      const n = st.prevPts.length / 2;
      const p0 = cv.matFromArray(n, 1, cv.CV_32FC2, Array.from(st.prevPts));
      let p1, flags = 0;
      if (Mi) {
        const g = new Array(n * 2);
        for (let i = 0; i < n; i++) { const q = apply(Mi, st.prevPts[i * 2], st.prevPts[i * 2 + 1]) || [st.prevPts[i * 2], st.prevPts[i * 2 + 1]]; g[i * 2] = q[0]; g[i * 2 + 1] = q[1]; }
        p1 = cv.matFromArray(n, 1, cv.CV_32FC2, g); flags = cv.OPTFLOW_USE_INITIAL_FLOW;
      } else p1 = new cv.Mat();
      const status = new cv.Mat(), err = new cv.Mat();
      let out = null;
      try {
        cv.calcOpticalFlowPyrLK(st.prevGray, gray, p0, p1, status, err, winSize, 4, crit, flags);
        const cur = [], prev = [];
        for (let i = 0; i < n; i++) {
          if (!status.data[i]) continue;
          const x = p1.data32F[i * 2], y = p1.data32F[i * 2 + 1];
          if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows) continue;
          cur.push(x, y); prev.push(st.prevPts[i * 2], st.prevPts[i * 2 + 1]);
        }
        const m = cur.length / 2;
        const fit = (c, p, thr) => {
          const k = c.length / 2;
          const cM = cv.matFromArray(k, 1, cv.CV_32FC2, c), pM = cv.matFromArray(k, 1, cv.CV_32FC2, p), mask = new cv.Mat();
          const Hm = cv.findHomography(cM, pM, cv.RANSAC, thr, mask, 1000, 0.99);
          let r = null;
          if (!Hm.empty()) { let inl = 0; const keep = []; for (let i = 0; i < k; i++) if (mask.data[i]) { inl++; keep.push(c[i * 2], c[i * 2 + 1]); } r = { M: Array.from(Hm.data64F), inl, keep }; }
          // Kuvien välillä perspektiivi muuttuu vain vähän. Jos homografia väittää muuta (esim. vähäpiirteinen
          // valkoinen ovi lähikuvassa), käytetään vakaampaa affiinista mallia samoista pisteistä.
          if (r && (Math.abs(r.M[6]) * gray.cols > P.maxPerspStep || Math.abs(r.M[7]) * gray.rows > P.maxPerspStep)) {
            const am = new cv.Mat(), A = cv.estimateAffine2D(cM, pM, am, cv.RANSAC, thr, 1000, 0.99, 10);
            if (!A.empty()) { let inl = 0; const keep = []; for (let i = 0; i < k; i++) if (am.data[i]) { inl++; keep.push(c[i * 2], c[i * 2 + 1]); } const a = A.data64F; r = { M: [a[0], a[1], a[2], a[3], a[4], a[5], 0, 0, 1], inl, keep, affine: true }; }
            else r = null;
            am.delete(); A.delete();
          }
          cM.delete(); pM.delete(); mask.delete(); Hm.delete();
          return r;
        };
        if (m >= P.trackMinPts) {
          // 1) ovitason pisteet (edellisen kuvan sijainnin perusteella) -> liike ovien tasossa
          const v = st.viewId && views.get(st.viewId);
          let planeRes = null, pc = [], pp = [], rest = [];
          if (P.planeTrack !== false && st.H && v && v.zone && v.zone.length) {
            for (let i = 0; i < m; i++) {
              const q = apply(st.H, prev[i * 2], prev[i * 2 + 1]);
              if (q && inZone(v.zone, q[0], q[1])) { pc.push(cur[i * 2], cur[i * 2 + 1]); pp.push(prev[i * 2], prev[i * 2 + 1]); }
              else rest.push(cur[i * 2], cur[i * 2 + 1]);
            }
            if (pc.length / 2 >= P.planeTrackMin && spread(pp, gray.cols, gray.rows) >= P.planeTrackSpread) {
              const r = fit(pc, pp, 2.5);
              if (r && r.inl >= P.planeTrackMin * 0.75 && r.inl / (pc.length / 2) > 0.5 && sane(r.M)) planeRes = r;
            }
          }
          if (planeRes) {
            // muut pisteet säilytetään seurattavina (varalle), mutta ne eivät vaikuta liikkeeseen
            const keep = planeRes.keep.concat(rest.slice(0, Math.max(0, P.trackPts * 2 - planeRes.keep.length)));
            out = { M: planeRes.M, pts: new Float32Array(keep), inliers: planeRes.inl, plane: planeRes.inl };
          } else {
            const r = fit(cur, prev, 3);
            if (r && r.inl >= P.trackMinPts && r.inl / m > 0.5 && sane(r.M)) out = { M: r.M, pts: new Float32Array(r.keep), inliers: r.inl };
          }
          if (out) out.planePts = pc.length / 2;
        }
      } finally { p0.delete(); p1.delete(); status.delete(); err.delete(); }
      return out;
    }

    function orbFeat(gray) {
      const kp = new cv.KeyPointVector(), desc = new cv.Mat();
      orb.detectAndCompute(gray, noMask, kp, desc);
      const n = kp.size(), pts = new Float32Array(n * 2);
      for (let i = 0; i < n; i++) { const p = kp.get(i).pt; pts[i * 2] = p.x; pts[i * 2 + 1] = p.y; }
      kp.delete();
      return { desc, pts, n };
    }
    // Varakeino suurelle liikkeelle (nopea zoomaus/heilautus): piirrevertailu edelliseen kuvaan.
    function orbFlow(gray) {
      if (!st.prevGray) return null;
      const a = orbFeat(st.prevGray), b = orbFeat(gray);
      let out = null;
      try {
        if (a.n < 30 || b.n < 30) return null;
        const mv = new cv.DMatchVectorVector();
        bf.knnMatch(b.desc, a.desc, mv, 2);
        const src = [], dst = [];
        for (let i = 0; i < mv.size(); i++) {
          const m = mv.get(i);
          if (m.size() >= 2) { const x = m.get(0), y = m.get(1); if (x.distance < 0.8 * y.distance && x.distance < 64) { src.push(b.pts[x.queryIdx * 2], b.pts[x.queryIdx * 2 + 1]); dst.push(a.pts[x.trainIdx * 2], a.pts[x.trainIdx * 2 + 1]); } }
          m.delete();
        }
        mv.delete();
        const n = src.length / 2;
        if (n < 20) return null;
        const sM = cv.matFromArray(n, 1, cv.CV_32FC2, src), dM = cv.matFromArray(n, 1, cv.CV_32FC2, dst), mask = new cv.Mat();
        const Hm = cv.findHomography(sM, dM, cv.RANSAC, 4, mask, 1000, 0.995);
        if (!Hm.empty()) {
          let inl = 0; const keep = [];
          for (let i = 0; i < n; i++) if (mask.data[i]) { inl++; keep.push(src[i * 2], src[i * 2 + 1]); }
          const M = Array.from(Hm.data64F), det = M[0] * M[4] - M[1] * M[3];
          if (inl >= 20 && inl / n > 0.4 && det > 0.3 && det < 3.5 && Math.abs(M[6]) < 0.005 && Math.abs(M[7]) < 0.005) out = { M, pts: new Float32Array(keep), inliers: inl, orb: true };
        }
        sM.delete(); dM.delete(); mask.delete(); Hm.delete();
      } finally { a.desc.delete(); b.desc.delete(); }
      return out;
    }

    // seurantakoordinaatit <-> tunnistuskoordinaatit (kuva skaalattu kertoimella k)
    const scaleIn = (H, k) => mul(H, [k, 0, 0, 0, k, 0, 0, 0, 1]);       // H_det -> H_track
    const scaleOut = (H, k) => mul(H, [1 / k, 0, 0, 0, 1 / k, 0, 0, 0, 1]); // H_track -> H_det

    function sane(M) {
      const det = M[0] * M[4] - M[1] * M[3];
      return isFinite(det) && det > 0.5 && det < 2 && Math.abs(M[2]) < 250 && Math.abs(M[5]) < 250 && Math.abs(M[6]) < 0.004 && Math.abs(M[7]) < 0.004;
    }
    function motionMag(M, fw, fh) {
      let m = 0;
      for (const [x, y] of [[0, 0], [fw, 0], [fw, fh], [0, fh]]) { const p = apply(M, x, y); if (!p) return 1; m = Math.max(m, Math.hypot(p[0] - x, p[1] - y)); }
      return m / fw;
    }
    // Terävyys = Laplacen varianssi pienennetystä kuvasta (liike-epäterävyys pienentää sitä)
    const lap = new cv.Mat(), mean = new cv.Mat(), sd = new cv.Mat(), sharpSize = new cv.Size(0, 0);
    function sharpness(gray) {
      const small = new cv.Mat();
      cv.resize(gray, small, sharpSize, 0.5, 0.5, cv.INTER_AREA);
      cv.Laplacian(small, lap, cv.CV_16S, 1);
      cv.meanStdDev(lap, mean, sd);
      small.delete();
      return sd.data64F[0] * sd.data64F[0];
    }
    function plausible(H, fw, fh, v) {
      if (!H || !v || !v.w) return false;
      if (H[0] * H[4] - H[1] * H[3] <= 0) return false;
      const cs = [[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(H, x, y));
      if (cs.some(c => !c || !isFinite(c[0]) || !isFinite(c[1]))) return false;
      if (!isConvex(cs)) return false;
      const a = Math.abs(polyArea(cs)), va = v.w * v.h;
      if (a < va * 0.0004 || a > va * 40) return false;
      const c = apply(H, fw / 2, fh / 2);
      if (!c || c[0] < -3 * v.w || c[0] > 4 * v.w || c[1] < -3 * v.h || c[1] > 4 * v.h) return false;
      if (!viewVisible(H, fw, fh, v)) return false;
      const L = i => Math.hypot(cs[i][0] - cs[(i + 1) % 4][0], cs[i][1] - cs[(i + 1) % 4][1]);
      const r1 = L(0) / L(2), r2 = L(1) / L(3);
      return !(r1 > 5 || r1 < 0.2 || r2 > 5 || r2 < 0.2);
    }
    function cabinetAt(v, x, y) {
      let hit = null, ba = Infinity;
      v.regionsPx.forEach((r, i) => { if (pointInPoly(x, y, r.poly)) { const a = Math.abs(polyArea(r.poly)); if (a < ba) { ba = a; hit = { cabinetId: r.cabinetId, edge: false }; } } });
      if (hit) return hit;
      let best = null, bd = P.edgeTolerance * Math.max(v.w, v.h);
      v.regionsPx.forEach(r => { const d = distToPoly(x, y, r.poly); if (d < bd) { bd = d; best = { cabinetId: r.cabinetId, edge: true }; } });
      return best;
    }

    /** Käsittele kuva. Palauttaa tuloksen ja (raw) harmaasävykuvan tunnistusta varten pyydettäessä. */
    function processFrame(img, opts) {
      opts = opts || {};
      const t0 = Date.now(), now = opts.t != null ? opts.t : t0, id = opts.id != null ? opts.id : st.lastId + 1;
      st.dtNow = st.lastT != null ? now - st.lastT : 0;
      const { raw, eq } = toGray(img);
      const fw = eq.cols, fh = eq.rows, k = raw.cols / eq.cols;
      st.k = k;
      const t1 = Date.now();
      let fl = flow(eq);
      const t2 = Date.now();
      // raskas varakeino vain, kun sijainti on tiedossa ja enintään joka toisella kuvalla
      if (!fl && (st.H || opts.keepChain) && st.prevGray && st.prevGray.cols === eq.cols && id - (st.orbAt || -9) >= 2) { st.orbAt = id; fl = orbFlow(eq); }
      const t3 = Date.now();

      // liikehistoria ankkureiden siirtoa varten (null = ketju katkesi)
      st.hist.push({ id, M: fl ? fl.M : null });
      if (st.hist.length > P.histLen) st.hist.shift();

      // sijainnin siirto
      const prevH = st.H;
      if (st.H) {
        if (fl) {
          const Hn = mul(st.H, fl.M);
          st.H = plausible(Hn, fw, fh, views.get(st.viewId)) ? Hn : null;
        } else st.H = null;
        if (now - st.anchorT > P.maxTrackMs) st.H = null;
        if (!st.H) { st.lostH = prevH; st.lostView = st.viewId; st.lostT = now; st.viewId = null; }
      }
      st.motion = fl && !fl.orb ? fl.M : null;
      const moving = fl ? motionMag(fl.M, fw, fh) : 0;

      // seurantapisteet
      let pts = fl ? fl.pts : null;
      const t4 = Date.now();
      const needPlane = P.planeTrack !== false && st.H && fl && (fl.planePts || 0) < P.planeTrackMin * 1.5 && id - (st.planeAt || -9) >= 3;
      if (!pts || pts.length < P.minTrackPts * 2 || needPlane) { if (needPlane) st.planeAt = id; pts = corners(eq, pts, P.planeTrack !== false && !!st.H); }
      const t5 = Date.now();
      if (st.prevGray) st.prevGray.delete();
      st.prevGray = eq; st.prevPts = pts;
      if (st.lastT != null) st.dtPrev = st.dtNow;
      st.lastT = now; st.lastId = id;

      const t6 = Date.now();
      const snap = st.H && (id % P.snapEvery === 0) ? doorSnap(eq) : null;
      const t7 = Date.now();
      // väliaikaisen avainkuvan vahvistus: kun ovien reunat vahvistavat sijainnin, lasketaan avainkuvan
      // kuvalle korjattu H liikehistorian kautta (H_k = H_nyt · inv(M_k+1 ··· M_nyt))
      const kfFeedback = [];
      for (const pk of st.pendingKfs.slice()) {
        if (snap && snap.verified && st.H) {
          const idx = st.hist.findIndex(e => e.id === pk.frameId);
          let chain = [1, 0, 0, 0, 1, 0, 0, 0, 1], okc = idx >= 0;
          for (let i = idx + 1; okc && i < st.hist.length; i++) { if (!st.hist[i].M) okc = false; else chain = mul(chain, st.hist[i].M); }
          const ci = okc ? inv(chain) : null;
          kfFeedback.push(ci ? { id: pk.kfId, ok: true, H: scaleOut(mul(st.H, ci), pk.k) } : { id: pk.kfId, ok: false });
          st.pendingKfs.splice(st.pendingKfs.indexOf(pk), 1);
        } else if (--pk.ttl <= 0) { kfFeedback.push({ id: pk.kfId, ok: false }); st.pendingKfs.splice(st.pendingKfs.indexOf(pk), 1); }
      }
      const res = describe(fw, fh, now);
      res.moving = moving;
      if (!st.H) res.sharp = sharpness(eq); // hukassa: tunnistukselle valitaan terävät kuvat
      res.debug = { ms: Date.now() - t0, parts: { kuva: t1 - t0, LK: t2 - t1, ORB: t3 - t2, pisteet: t5 - t4, reunat: t7 - t6 }, snap: snap ? snap.regions : 0, inliers: fl ? fl.inliers : 0, via: fl ? (fl.orb ? 'ORB' : fl.plane ? 'LK-taso' : 'LK') : '-', pts: opts.debug ? Array.from(pts.slice(0, 400)).map((x, i) => i % 2 ? x / fh : x / fw) : null, anchorAge: Math.round(now - st.anchorT) };
      let gray = null;
      if (opts.wantGray) gray = { data: new Uint8Array(raw.data), width: raw.cols, height: raw.rows, gray: true };
      raw.delete();
      // ennakkotieto tunnistukselle: nykyinen sijainti tai hetki sitten kadonnut (löysempi haku)
      let prior = null;
      if (st.H) prior = { H: scaleOut(st.H, k), viewId: st.viewId, loose: false };
      else if (st.lostH && now - st.lostT < P.lostPriorMs && views.has(st.lostView)) prior = { H: scaleOut(st.lostH, k), viewId: st.lostView, loose: true };
      return { res, gray, H: st.H, viewId: st.viewId, prior, id, kfFeedback };
    }

    /**
     * Ovien reunojen seuranta: ennustetun oven sivut kohdistetaan lähimpiin oveen sopiviin reunoihin
     * (rako/varjo), ja sijainti korjataan niiden mukaan. Valkoisissa piirteettömissä ovissa reunat ovat
     * luotettavin näyttö oven tasosta – erityisesti lähellä ja vinosta, kun muut piirteet ovat eri syvyydellä.
     */
    function clipSeg(A, B, W, H, m) {
      let t0 = 0, t1 = 1; const dx = B[0] - A[0], dy = B[1] - A[1];
      const p = [-dx, dx, -dy, dy], q = [A[0] - m, W - m - A[0], A[1] - m, H - m - A[1]];
      for (let i = 0; i < 4; i++) {
        if (p[i] === 0) { if (q[i] < 0) return null; continue; }
        const r = q[i] / p[i];
        if (p[i] < 0) { if (r > t1) return null; if (r > t0) t0 = r; } else { if (r < t0) return null; if (r < t1) t1 = r; }
      }
      return [[A[0] + dx * t0, A[1] + dy * t0], [A[0] + dx * t1, A[1] + dy * t1]];
    }
    function lineX(l1, l2) {
      const [p, p2] = l1, [r, r2] = l2;
      const d1 = [p2[0] - p[0], p2[1] - p[1]], d2 = [r2[0] - r[0], r2[1] - r[1]];
      const den = d1[0] * d2[1] - d1[1] * d2[0];
      if (Math.abs(den) < 1e-9) return null;
      const t = ((r[0] - p[0]) * d2[1] - (r[1] - p[1]) * d2[0]) / den;
      return [p[0] + d1[0] * t, p[1] + d1[1] * t];
    }
    function doorSnap(gray) {
      const v = st.viewId && views.get(st.viewId);
      if (!P.doorSnap || !st.H || !v || !v.regionsPx.length) return null;
      const Hi = inv(st.H); if (!Hi) return null;
      const W = gray.cols, Hh = gray.rows;
      const projs = v.regionsPx.map(r => r.poly.map(([x, y]) => apply(Hi, x, y)));
      // rajaus: kaikki näkyvät ovet + marginaali
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity, any = false;
      const plans = projs.map(poly => {
        if (poly.some(p => !p)) return null;
        const lens = poly.map((p, i) => Math.hypot(poly[(i + 1) % 4][0] - p[0], poly[(i + 1) % 4][1] - p[1]));
        if (Math.min(...lens) < 30) return null;
        // vähintään 3 kulmaa näkyvissä: muuten kulmat pitäisi arvata jatkamalla viivoja ruudun ulkopuolelle
        if (poly.filter(([x, y]) => x > -0.08 * W && x < 1.08 * W && y > -0.08 * Hh && y < 1.08 * Hh).length < 3) return null;
        const R = Math.max(P.snapRmin, Math.min(P.snapRmax, 0.06 * (lens[0] + lens[1]) / 2));
        const sides = poly.map((p, i) => clipSeg(p, poly[(i + 1) % 4], W, Hh, 3));
        const vis = sides.map(sd => sd && Math.hypot(sd[1][0] - sd[0][0], sd[1][1] - sd[0][1]) >= 25 ? sd : null);
        if (vis.filter(Boolean).length < 2) return null;
        for (const sd of vis) if (sd) for (const [x, y] of sd) { x0 = Math.min(x0, x - R - 14); y0 = Math.min(y0, y - R - 14); x1 = Math.max(x1, x + R + 14); y1 = Math.max(y1, y + R + 14); any = true; }
        return { poly, R, vis };
      });
      if (!any) return null;
      x0 = Math.max(0, Math.floor(x0)); y0 = Math.max(0, Math.floor(y0)); x1 = Math.min(W, Math.ceil(x1)); y1 = Math.min(Hh, Math.ceil(y1));
      const cw = x1 - x0, ch = y1 - y0; if (cw < 20 || ch < 20) return null;
      const roi = gray.roi(new cv.Rect(x0, y0, cw, ch)), bl = new cv.Mat(), gx = new cv.Mat(), gy = new cv.Mat();
      cv.GaussianBlur(roi, bl, new cv.Size(0, 0), 1.0);
      cv.Sobel(bl, gx, cv.CV_32F, 1, 0, 3); cv.Sobel(bl, gy, cv.CV_32F, 0, 1, 3);
      const GX = gx.data32F, GY = gy.data32F, GR = bl.data;
      let mag = 0; for (let i = 0; i < GX.length; i += 3) mag += Math.abs(GX[i]) + Math.abs(GY[i]); mag = mag / (GX.length / 3) * 0.75 || 1;
      const g = (x, y) => { const xi = Math.round(x - x0), yi = Math.round(y - y0); return xi < 1 || yi < 1 || xi >= cw - 1 || yi >= ch - 1 ? -1 : yi * cw + xi; };
      const feat = pts => { let m = 0, t = 0, n = 0; for (const [x, y] of pts) { const i = g(x, y); if (i < 0) continue; m += GR[i]; t += Math.abs(GX[i]) + Math.abs(GY[i]); n++; } return n ? [m / n, t / n] : null; };
      const fd = (a, b) => !a || !b ? 1 : Math.abs(a[0] - b[0]) / 25 + Math.abs(a[1] - b[1]) / Math.max(4, mag);
      const src = [], dst = [];
      let snapped = 0, sidesOk = 0, strongRegions = 0;
      plans.forEach((pl, ri) => {
        if (!pl) return;
        const poly = pl.poly, R = pl.R;
        const cx = poly.reduce((a, p) => a + p[0], 0) / 4, cy = poly.reduce((a, p) => a + p[1], 0) / 4;
        const inner = []; for (let a = 0; a < 7; a++) for (let b = 0; b < 7; b++) { const u = a / 6, w = b / 6;
          const top = [poly[0][0] + (poly[1][0] - poly[0][0]) * u, poly[0][1] + (poly[1][1] - poly[0][1]) * u], bot = [poly[3][0] + (poly[2][0] - poly[3][0]) * u, poly[3][1] + (poly[2][1] - poly[3][1]) * u];
          const p = [top[0] + (bot[0] - top[0]) * w, top[1] + (bot[1] - top[1]) * w]; inner.push([cx + (p[0] - cx) * 0.6, cy + (p[1] - cy) * 0.6]); }
        const fIn = feat(inner);
        const lines = [], ok = [];
        for (let k = 0; k < 4; k++) {
          const full = [poly[k], poly[(k + 1) % 4]], sd = pl.vis[k];
          lines.push(full); ok.push(false);
          if (!sd || !fIn) continue;
          const [A, B] = sd, dx = B[0] - A[0], dy = B[1] - A[1], L = Math.hypot(dx, dy);
          let nx = -dy / L, ny = dx / L;
          if ((cx - (A[0] + B[0]) / 2) * nx + (cy - (A[1] + B[1]) / 2) * ny < 0) { nx = -nx; ny = -ny; } // n sisäänpäin
          const N = Math.max(16, Math.min(60, Math.round(L / 3)));
          const score = (s1, s2) => {
            const a = [A[0] + nx * s1, A[1] + ny * s1], b = [B[0] + nx * s2, B[1] + ny * s2];
            const ex = b[0] - a[0], ey = b[1] - a[1], LL = Math.hypot(ex, ey) || 1, mx = -ey / LL, my = ex / LL;
            let sum = 0, sup = 0, cnt = 0;
            for (let j = 0; j < N; j++) { const t = 0.05 + 0.9 * j / (N - 1); const i = g(a[0] + ex * t, a[1] + ey * t); if (i < 0) continue; const gg = Math.abs(GX[i] * mx + GY[i] * my); sum += gg; if (gg > 1.5 * mag) sup++; cnt++; }
            return cnt < N * 0.6 ? null : { mean: sum / cnt, sc: (sum / cnt) * (0.5 + 0.5 * sup / cnt), a, b };
          };
          let best = null, top = 0;
          const cands = [];
          const step = R > 12 ? 2 : 1;
          for (let s1 = -R; s1 <= R; s1 += step) for (let s2 = -R; s2 <= R; s2 += step) { const r = score(s1, s2); if (r) { r.s1 = s1; r.s2 = s2; cands.push(r); if (r.sc > top) top = r.sc; } }
          cands.sort((p, q) => q.sc - p.sc);
          for (const c of cands.slice(0, 12)) { // vahvimmat ehdokkaat; oven sisäpuoli + ulkopuoli ratkaisee
            if (c.mean < 2.2 * mag) break;
            const band = sign => { const p = []; for (let j = 0; j < 12; j++) { const t = 0.1 + 0.8 * j / 11; for (const d of [4, 7, 10]) p.push([c.a[0] + (c.b[0] - c.a[0]) * t + nx * d * sign, c.a[1] + (c.b[1] - c.a[1]) * t + ny * d * sign]); } return p; };
            const ctx = fd(feat(band(-1)), fIn) - fd(feat(band(1)), fIn);
            const tot = c.sc / top + 0.6 * Math.max(-2, Math.min(2, ctx)) - 0.35 * (Math.abs(c.s1) + Math.abs(c.s2)) / (2 * R);
            if (ctx > 0.2 && (!best || tot > best.tot)) best = Object.assign({ tot }, c);
          }
          if (best) { lines[k] = [best.a, best.b]; ok[k] = true; sidesOk++; }
        }
        // kumpaankin suuntaan vähintään yksi varma sivu (muuten ovi ei ole sidottu)
        if (!((ok[0] || ok[2]) && (ok[1] || ok[3]))) return;
        let corners = [0, 1, 2, 3].map(k => lineX(lines[(k + 3) % 4], lines[k]));
        if (corners.some(c => !c) || !isConvex(corners)) return;
        if (corners.some((c, k) => Math.hypot(c[0] - poly[k][0], c[1] - poly[k][1]) > 2.5 * R)) return;
        const areaR = Math.abs(polyArea(corners)) / Math.abs(polyArea(poly));
        if (areaR < 0.8 || areaR > 1.25) return; // muoto muuttui liikaa -> todennäköisesti väärä reuna
        // Vahvistus vain, kun reunat osuvat lähes ennustettuun kohtaan. Isompi siirto tehdään vähitellen,
        // jotta väärä reuna (esim. verhon reuna kaapin edessä) ei voi vetää sijaintia kerralla kauas.
        const shift = Math.max(...corners.map((c, k) => Math.hypot(c[0] - poly[k][0], c[1] - poly[k][1])));
        if (ok.filter(Boolean).length >= 3 && shift <= Math.max(3, 0.35 * R)) strongRegions++;
        const lim = 0.5 * R;
        corners = corners.map((c, k) => { const dx = c[0] - poly[k][0], dy = c[1] - poly[k][1], d = Math.hypot(dx, dy); const f = d > lim ? lim / d : 1; return [poly[k][0] + dx * f, poly[k][1] + dy * f]; });
        for (let k = 0; k < 4; k++) { src.push(corners[k][0], corners[k][1]); dst.push(v.regionsPx[ri].poly[k][0], v.regionsPx[ri].poly[k][1]); }
        snapped++;
      });
      roi.delete(); bl.delete(); gx.delete(); gy.delete();
      if (!snapped) return { regions: 0, sides: sidesOk };
      const sM = cv.matFromArray(src.length / 2, 1, cv.CV_32FC2, src), dM = cv.matFromArray(dst.length / 2, 1, cv.CV_32FC2, dst);
      const Hm = cv.findHomography(sM, dM, 0);
      let Hn = Hm.empty() ? null : Array.from(Hm.data64F);
      sM.delete(); dM.delete(); Hm.delete();
      if (!Hn || !plausible(Hn, W, Hh, v)) return { regions: 0, sides: sidesOk };
      st.H = Hn;
      if (strongRegions) st.doorT = st.lastT != null ? st.lastT : Date.now(); // ovien reunat vahvistivat sijainnin
      return { regions: snapped, sides: sidesOk, verified: strongRegions > 0 };
    }

    function describe(fw, fh, now) {
      const v = st.viewId ? views.get(st.viewId) : null;
      let raw = null, overlay = [];
      if (v && st.H) {
        const c = apply(st.H, fw / 2, fh / 2);
        if (c) raw = cabinetAt(v, c[0], c[1]);
        const Hi = inv(st.H);
        if (Hi) overlay = v.regionsPx.map((r, i) => {
          const poly = r.poly.map(([x, y]) => apply(Hi, x, y));
          if (poly.some(p => !p)) return null;
          return { cabinetId: r.cabinetId, regionIndex: i, poly: poly.map(([x, y]) => [x / fw, y / fh]) };
        }).filter(Boolean);
      }
      const rawId = raw ? raw.cabinetId : null;
      if (!raw) {
        st.lostCount++; if (st.lostCount > P.holdFrames) st.stableCab = null;
        st.candCab = null; st.candCount = 0;
      } else {
        st.lostCount = 0;
        if (rawId === st.stableCab) { st.candCab = null; st.candCount = 0; }
        else {
          if (rawId === st.candCab) st.candCount++; else { st.candCab = rawId; st.candCount = 1; }
          if (st.candCount >= (st.stableCab == null ? 1 : P.switchFrames)) { st.stableCab = rawId; st.candCab = null; st.candCount = 0; }
        }
      }
      const mode = !st.H ? 'lost' : (now - st.anchorT < P.anchoredMs ? 'detect' : 'track');
      return { mode, viewId: st.viewId, cabinetId: st.stableCab, rawCabinetId: rawId, onEdge: !!(raw && raw.edge), overlay, frameW: fw, frameH: fh };
    }

    /**
     * Tunnistussäikeen tulos kuvalle frameId -> siirrä nykyhetkeen kuvien välisten liikkeiden avulla.
     * H_nyt = H_ankkuri · M_(k+1) · … · M_nyt
     */
    function applyAnchor(a) {
      if (!a || !a.ok || !a.H) return { applied: false, reason: 'ei tulosta' };
      const v = views.get(a.viewId);
      if (!v) return { applied: false, reason: 'näkymä' };
      if (a.w && (v.w !== a.w || v.h !== a.h)) setView(a.viewId, a.w, a.h, v.regions);
      const idx = st.hist.findIndex(e => e.id === a.frameId);
      if (idx < 0) return { applied: false, reason: 'liian vanha' };
      let H = scaleIn(a.H, a.fw && st.prevGray ? a.fw / st.prevGray.cols : (st.k || 1));
      for (let i = idx + 1; i < st.hist.length; i++) {
        const M = st.hist[i].M;
        if (!M) return { applied: false, reason: 'ketju katkesi' };
        H = mul(H, M);
      }
      const fw = st.prevGray ? st.prevGray.cols : a.fw, fh = st.prevGray ? st.prevGray.rows : a.fh;
      if (!st.prevGray) return { applied: false, reason: 'ei kuvaa', kfReject: a.kfId };
      if (!plausible(H, fw, fh, views.get(a.viewId))) return { applied: false, reason: 'epäuskottava', kfReject: a.kfId };
      // Ovien reunat ovat juuri vahvistaneet nykyisen sijainnin: ristiriitainen ankkuri hylätään
      // (lähellä ja vinosta kartoituskuvaan perustuva ankkuri voi olla parallaksin takia pielessä).
      if (st.H && st.viewId === a.viewId && st.lastT - st.doorT < P.doorVerifiedMs) {
        const Hi0 = inv(st.H), Hi1 = inv(H);
        let sum = 0, n = 0;
        if (Hi0 && Hi1) for (const r of views.get(a.viewId).regionsPx) for (const [x, y] of r.poly) {
          const p0 = apply(Hi0, x, y), p1 = apply(Hi1, x, y);
          if (!p0 || !p1 || p0[0] < -fw * 0.2 || p0[0] > fw * 1.2 || p0[1] < -fh * 0.2 || p0[1] > fh * 1.2) continue;
          sum += Math.hypot(p0[0] - p1[0], p0[1] - p1[1]); n++;
        }
        if (n && sum / n > P.anchorDoorTol) return { applied: false, reason: `ovien reunat eri mieltä (${(sum / n).toFixed(0)} px)`, kfReject: a.kfId };
      }
      st.H = H; st.viewId = a.viewId; st.lostH = null; st.anchorT = st.lastT != null ? st.lastT : Date.now();
      if (a.kfId) { st.pendingKfs.push({ kfId: a.kfId, frameId: a.frameId, ttl: P.kfConfirmTtl, k: a.fw && st.prevGray ? a.fw / st.prevGray.cols : (st.k || 1) }); if (st.pendingKfs.length > 4) st.pendingKfs.shift(); }
      return { applied: true, steps: st.hist.length - 1 - idx, res: describe(fw, fh, st.anchorT) };
    }

    return { processFrame, applyAnchor, setView, removeView, reset, hasView: id => views.has(id), get viewCount() { return views.size; }, get state() { return st; }, params: P };
  }

  /**
   * Ovien reunojen kohdistus: siirtää käyttäjän piirtämän nelikulmion sivut lähimpiin vahvoihin
   * suoriin reunoihin (oven ja raon/seinän raja). Kukin sivu haetaan erikseen: kokeillaan sivun
   * päätepisteiden siirtoja sivun normaalin suuntaan ja pisteytetään suoran "reunatuki" eli
   * gradientin komponentti suoran normaalin suuntaan. Kulmat = vierekkäisten sivujen leikkauspisteet.
   * img: RGBA-rajaus, quad: [[x,y]x4] rajauksen pikseleinä (järjestys: vasen ylä, oikea ylä, oikea ala, vasen ala).
   */
  function snapQuad(cv, img, quad, opts) {
    opts = opts || {};
    const rgba = new cv.Mat(img.height, img.width, cv.CV_8UC4); rgba.data.set(img.data);
    let gray = new cv.Mat(); cv.cvtColor(rgba, gray, cv.COLOR_RGBA2GRAY); rgba.delete();
    let s = 1;
    const long = Math.max(gray.cols, gray.rows);
    if (long > (opts.maxSide || 900)) {
      s = (opts.maxSide || 900) / long;
      const g2 = new cv.Mat(); cv.resize(gray, g2, new cv.Size(Math.round(gray.cols * s), Math.round(gray.rows * s)), 0, 0, cv.INTER_AREA);
      gray.delete(); gray = g2;
    }
    cv.GaussianBlur(gray, gray, new cv.Size(0, 0), 1.2);
    const gx = new cv.Mat(), gy = new cv.Mat();
    cv.Sobel(gray, gx, cv.CV_32F, 1, 0, 3); cv.Sobel(gray, gy, cv.CV_32F, 0, 1, 3);
    const W = gray.cols, H = gray.rows, GX = new Float32Array(gx.data32F), GY = new Float32Array(gy.data32F), GR = new Uint8Array(gray.data);
    gray.delete(); gx.delete(); gy.delete();
    let mag = 0; for (let i = 0; i < GX.length; i++) mag += Math.hypot(GX[i], GY[i]); mag /= GX.length || 1;

    const q = quad.map(([x, y]) => [x * s, y * s]);
    const lens = q.map((p, i) => Math.hypot(q[(i + 1) % 4][0] - p[0], q[(i + 1) % 4][1] - p[1]));
    const R = Math.max(6, Math.min(60 * s + 20, 0.15 * Math.min(...lens)));
    const sigma = R * 0.8;
    const qc = [q.reduce((a, p) => a + p[0], 0) / 4, q.reduce((a, p) => a + p[1], 0) / 4];
    // Pinnan tunnusluvut (kirkkaus, tekstuuri) pisteiden joukossa
    const feat = pts => {
      let m = 0, t = 0, n = 0;
      for (const [x0, y0] of pts) {
        const x = Math.round(x0), y = Math.round(y0);
        if (x < 1 || y < 1 || x >= W - 1 || y >= H - 1) continue;
        const i = y * W + x; m += GR[i]; t += Math.hypot(GX[i], GY[i]); n++;
      }
      return n ? [m / n, t / n] : null;
    };
    // Oven sisäosa: keskialue (oven pinta)
    const inner = [];
    for (let k = 0; k < 400; k++) { const u = (k % 20) / 19 - 0.5, v = Math.floor(k / 20) / 19 - 0.5;
      const top = [q[0][0] + (q[1][0] - q[0][0]) * (u + 0.5), q[0][1] + (q[1][1] - q[0][1]) * (u + 0.5)];
      const bot = [q[3][0] + (q[2][0] - q[3][0]) * (u + 0.5), q[3][1] + (q[2][1] - q[3][1]) * (u + 0.5)];
      const w = 0.5 + 0.5 * v; inner.push([qc[0] + ((top[0] + (bot[0] - top[0]) * w) - qc[0]) * 0.5, qc[1] + ((top[1] + (bot[1] - top[1]) * w) - qc[1]) * 0.5]); }
    const fIn = feat(inner) || [128, mag];
    const fdist = (a, b) => !a || !b ? 1 : Math.abs(a[0] - b[0]) / 25 + Math.abs(a[1] - b[1]) / Math.max(4, mag);
    // Reunan puolet: oikealla reunalla sisäpuoli muistuttaa oven pintaa ja ulkopuoli ei.
    function sideContext(A, B) {
      const dx = B[0] - A[0], dy = B[1] - A[1], L = Math.hypot(dx, dy) || 1;
      let nx = -dy / L, ny = dx / L;
      const mx = (A[0] + B[0]) / 2, my = (A[1] + B[1]) / 2;
      if ((qc[0] - mx) * nx + (qc[1] - my) * ny < 0) { nx = -nx; ny = -ny; } // n osoittaa sisään
      const band = sign => { const p = []; for (let k = 0; k < 30; k++) { const t = 0.1 + 0.8 * k / 29; for (const d of [5, 8, 11]) p.push([A[0] + dx * t + nx * d * sign, A[1] + dy * t + ny * d * sign]); } return p; };
      return { dIn: fdist(feat(band(1)), fIn), dOut: fdist(feat(band(-1)), fIn) };
    }

    function lineScore(A, B) {
      const dx = B[0] - A[0], dy = B[1] - A[1], L = Math.hypot(dx, dy) || 1, nx = -dy / L, ny = dx / L;
      const N = Math.max(20, Math.min(200, Math.round(L / 2)));
      let sum = 0, sup = 0, cnt = 0;
      for (let k = 0; k < N; k++) {
        const t = 0.06 + 0.88 * k / (N - 1);
        const x = Math.round(A[0] + dx * t), y = Math.round(A[1] + dy * t);
        if (x < 1 || y < 1 || x >= W - 1 || y >= H - 1) continue;
        const i = y * W + x, g = Math.abs(GX[i] * nx + GY[i] * ny);
        sum += g; if (g > 1.5 * mag) sup++; cnt++;
      }
      if (cnt < N * 0.6) return { mean: 0, score: 0 };
      const mean = sum / cnt;
      return { mean, score: mean * (0.5 + 0.5 * sup / cnt) };
    }
    // Kullekin sivulle useita ehdokasreunoja (paikalliset maksimit); lopullinen valinta tehdään
    // yhdistelmänä, jotta esim. oven sisäpaneelin ura tai naapurioven reuna ei voita oikeaa reunaa.
    function sideCands(a, b) {
      const dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy) || 1, nx = -dy / L, ny = dx / L;
      const at = (s1, s2) => [[a[0] + nx * s1, a[1] + ny * s1], [b[0] + nx * s2, b[1] + ny * s2]];
      const evalAt = (s1, s2) => { const [A, B] = at(s1, s2); const r = lineScore(A, B); return { s1, s2, A, B, mean: r.mean, score: r.score }; };
      const step = R > 20 ? 2 : 1, grid = [];
      for (let s1 = -R; s1 <= R; s1 += step) for (let s2 = -R; s2 <= R; s2 += step) grid.push(evalAt(s1, s2));
      grid.sort((p, q2) => q2.score - p.score);
      const picked = [];
      for (const g of grid) {
        if (picked.length >= 4) break;
        if (g.mean < (opts.minEdge || 2.2) * mag) break;
        if (picked.some(p => Math.abs(p.s1 - g.s1) + Math.abs(p.s2 - g.s2) < 8)) continue; // eri suora
        // tarkennus puolen pikselin askelin
        let best = g;
        for (let s1 = g.s1 - step; s1 <= g.s1 + step; s1 += 0.5) for (let s2 = g.s2 - step; s2 <= g.s2 + step; s2 += 0.5) {
          const e = evalAt(s1, s2); if (e.score > best.score) best = e;
        }
        Object.assign(best, sideContext(best.A, best.B));
        picked.push(best);
      }
      const base = evalAt(0, 0);
      base.isBase = true;
      return { cands: picked, base, top: picked.length ? picked[0].score : 0 };
    }
    const sc4 = [0, 1, 2, 3].map(i => sideCands(q[i], q[(i + 1) % 4]));
    function intersect(l1, l2) {
      const [p, p2] = [l1.A, l1.B], [r, r2] = [l2.A, l2.B];
      const d1 = [p2[0] - p[0], p2[1] - p[1]], d2 = [r2[0] - r[0], r2[1] - r[1]];
      const den = d1[0] * d2[1] - d1[1] * d2[0];
      if (Math.abs(den) < 1e-9) return null;
      const t = ((r[0] - p[0]) * d2[1] - (r[1] - p[1]) * d2[0]) / den;
      return [p[0] + d1[0] * t, p[1] + d1[1] * t];
    }
    // Yhdistelmän pisteytys: reunojen vahvuus (suhteessa sivun vahvimpaan) + muodon ja koon samankaltaisuus
    // piirretyn alueen kanssa + pieni etäisyysrangaistus. Sivu voi myös jäädä paikalleen (ei reunaa).
    const area = p => Math.abs(polyArea(p)), qa = area(q);
    const opt4 = sc4.map(x => x.cands.length ? x.cands : [x.base]);
    let best = null;
    const idx = [0, 0, 0, 0];
    const total = opt4.reduce((n, o) => n * o.length, 1);
    for (let c = 0; c < total; c++) {
      let r = c; for (let k = 0; k < 4; k++) { idx[k] = r % opt4[k].length; r = Math.floor(r / opt4[k].length); }
      const sides = idx.map((j, k) => opt4[k][j]);
      const out = [0, 1, 2, 3].map(i => intersect(sides[(i + 3) % 4], sides[i]));
      if (!out.every(Boolean) || !isConvex(out)) continue;
      const a = area(out);
      if (a < 0.6 * qa || a > 1.6 * qa) continue;
      if (!out.every((p, i) => Math.hypot(p[0] - q[i][0], p[1] - q[i][1]) < 2.5 * R)) continue;
      let sc = 0;
      sides.forEach((sd, k) => {
        sc += sd.isBase ? 0 : sd.score / (sc4[k].top || 1);
        sc -= 0.35 * (Math.abs(sd.s1 || 0) + Math.abs(sd.s2 || 0)) / (2 * R);
        if (!sd.isBase) sc += 0.6 * Math.max(-2, Math.min(2, sd.dOut - sd.dIn));
      });
      sc -= 2.0 * Math.abs(Math.log(a / qa));
      if (!best || sc > best.sc) best = { sc, out, sides: sides.slice() };
    }
    if (!best) return { quad, changed: false, sides: 0 };
    const moved = best.sides.filter(x => !x.isBase && Math.max(Math.abs(x.s1), Math.abs(x.s2)) > 0.5).length;
    return { quad: best.out.map(([x, y]) => [x / s, y / s]), changed: moved > 0, sides: moved };
  }

  const api = { createEngine, createTracker, snapQuad, _m: { mul, inv, apply, pointInPoly, isConvex } };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.KaappiVision = api;
})(typeof self !== 'undefined' ? self : this);
