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

  const POP = new Uint8Array(256);
  for (let i = 0; i < 256; i++) POP[i] = (i & 1) + POP[i >> 1];
  const CELL = 48;

  const DEFAULTS = {
    refMax: 1600,        // näkymäkuvan pidempi sivu sisäisesti (px)
    refFeatures: 4000,   // ORB-piirteitä per näkymä
    frameFeatures: 1200, // ORB-piirteitä per kamerakuva
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
    kfTry: 3, kfMinInliers: 30, kfMax: 40,
    strongInliers: 60, maxShift: 0.02, maxScaleShift: 0.15,
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
    let st = freshState();

    function freshState() {
      return {
        frameNo: 0, viewId: null, H: null, mode: 'lost', trackAge: 0,
        prevGray: null, prevPts: null,
        stableCab: null, candCab: null, candCount: 0, lostCount: 0, lastDetectFrame: -1, lastFullScan: -99, kfs: []
      };
    }
    function resetTracking() {
      if (st.prevGray) st.prevGray.delete();
      if (st.lastGood) st.lastGood.desc.delete();
      for (const k of st.kfs) k.desc.delete();
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
      gray.delete();
      views.set(id, v);
      return { id, features: v.n, coverage: v.coverage, w: v.w, h: v.h };
    }
    function setRegions(id, regions) {
      const v = views.get(id); if (!v) return false;
      v.regions = regions || []; regionsToPx(v); return true;
    }
    function removeView(id) {
      const v = views.get(id);
      if (v) { v.desc.delete(); views.delete(id); }
      st.kfs = st.kfs.filter(k => { if (k.viewId === id) { k.desc.delete(); return false; } return true; });
      if (st.viewId === id) resetTracking();
    }
    function clear() { for (const id of [...views.keys()]) removeView(id); resetTracking(); }

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
      let H = null, inliers = 0; const inlierPts = [];
      if (!Hm.empty()) {
        H = Array.from(Hm.data64F);
        for (let i = 0; i < nGood; i++) if (mask.data[i]) { inliers++; if (inlierPts.length < 300) inlierPts.push(src[i * 2], src[i * 2 + 1]); }
      }
      sM.delete(); dM.delete(); mask.delete(); Hm.delete();
      return { good: nGood, inliers, H, inlierPts };
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
      // ruudun keskipisteen pitää osua järkevän lähelle näkymää
      const c = apply(H, fw / 2, fh / 2);
      if (!c || c[0] < -v.w || c[0] > 2 * v.w || c[1] < -v.h || c[1] > 2 * v.h) return false;
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
            const r = matchHomography(f, v, P.ransacPx * v.w / 1000);
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
    function anchor(img, opts) {
      opts = opts || {};
      const t0 = Date.now(), now = opts.t != null ? opts.t : t0;
      const { gray } = toGray(img, P.frameMax);
      const fw = gray.cols, fh = gray.rows;
      const f = detect(orbFrame, gray);
      gray.delete();
      const rejects = [];
      let best = null;
      const pv = opts.priorView && opts.priorH ? views.get(opts.priorView) : null;
      // 1) ohjattu haku ennakkotiedon ympäriltä (halpa, toimii lähikuvissakin)
      if (pv) {
        const r = guidedMatch(f, pv, opts.priorH, P.guideR * pv.w * (opts.loose ? 2.5 : 1));
        if (r && r.H && r.inliers >= P.minGuidedInliers && r.inliers / r.good >= 0.3 && plausible(r.H, fw, fh, pv)) best = Object.assign({ v: pv, via: 'ohjattu' }, r);
        else rejects.push('ohjattu ' + (r ? r.inliers + '/' + r.good : 0));
      }
      // 2) koko kuvan vertailu (raskas): kun sijainti on hukassa tai pyydetään täysi tarkistus
      if (!best && (!pv || opts.full || opts.loose)) {
        for (const v of views.values()) {
          const r = matchHomography(f, v, P.ransacPx * v.w / 1000);
          if (!r || !r.H) { rejects.push(v.id + ':few' + (r ? r.good : 0)); continue; }
          if (r.inliers < P.minInliers || r.inliers / r.good < P.minInlierRatio) { rejects.push(v.id + ':inl' + r.inliers + '/' + r.good); continue; }
          if (!plausible(r.H, fw, fh, v)) { rejects.push(v.id + ':geom' + r.inliers); continue; }
          const score = r.inliers * (pv && v.id === pv.id ? 1.15 : 1);
          if (!best || score > best.score) best = Object.assign({ v, score, via: 'globaali' }, r);
        }
      }
      // 3) avainkuvat: aiemmin varmasti ankkuroidut kamerakuvat lähellä ennustettua sijaintia.
      //    Lähikuvassa kartoituskuvan tarkkuus ei riitä, mutta samalta etäisyydeltä otettu avainkuva riittää.
      //    Haku rajataan ennusteen ympärille, joten identtinen naapuriovi ei voi sekoittaa.
      if (!best && pv) {
        const pc = apply(opts.priorH, fw / 2, fh / 2);
        const pArea = Math.abs(polyArea([[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(opts.priorH, x, y) || [0, 0])));
        const R = (opts.loose ? P.kfRadiusLoose : P.kfRadius) * pv.w;
        const cands = st.kfs.filter(k => k.viewId === pv.id && pc && Math.hypot(k.center[0] - pc[0], k.center[1] - pc[1]) < R)
          .map(k => ({ k, score: Math.hypot(k.center[0] - pc[0], k.center[1] - pc[1]) / R + Math.abs(Math.log(k.area / (pArea || 1))) }))
          .sort((a, b) => a.score - b.score).slice(0, P.kfTry);
        for (const { k } of cands) {
          const r = matchHomography(f, k, 4);
          if (!r || !r.H || r.inliers < P.kfMinInliers || r.inliers / r.good < 0.35) { rejects.push('avainkuva ' + (r ? r.inliers + '/' + r.good : 0)); continue; }
          const Hn = mul(k.H, r.H), c = apply(Hn, fw / 2, fh / 2);
          if (!plausible(Hn, fw, fh, pv) || !c || Math.hypot(c[0] - pc[0], c[1] - pc[1]) > R) { rejects.push('avainkuva geom'); continue; }
          if (!best || r.inliers > best.inliers) best = { v: pv, H: Hn, good: r.good, inliers: r.inliers, inlierPts: r.inlierPts, via: 'avainkuva' };
        }
      }
      // Johdonmukaisuus: heikko ankkuri ei saa siirtää varmaa sijaintia paljon (väärä ankkuri
      // aiheuttaisi kasvavan virheen seurannassa). Vahva ankkuri saa korjata.
      if (best && pv && !opts.loose && best.inliers < P.strongInliers) {
        const area = H => Math.abs(polyArea([[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(H, x, y) || [0, 0])));
        const c1 = apply(best.H, fw / 2, fh / 2), c0 = apply(opts.priorH, fw / 2, fh / 2);
        const dc = c1 && c0 ? Math.hypot(c1[0] - c0[0], c1[1] - c0[1]) / pv.w : 1;
        const ds = Math.abs(Math.log(area(best.H) / (area(opts.priorH) || 1)));
        if (dc > P.maxShift || ds > P.maxScaleShift) { rejects.push(`ristiriita ${best.via} ${best.inliers} siirto ${(dc * 100).toFixed(1)}% mittak ${(ds * 100).toFixed(0)}%`); best = null; }
      }
      // tallenna avainkuvaksi, jos ankkuroitu suoraan kartoituskuvaan ja kohtaa ei vielä ole
      let kept = false;
      if (best && best.via !== 'avainkuva' && best.inliers >= P.kfMinInliers && f.n >= 80) {
        const cs = [[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(best.H, x, y));
        const center = apply(best.H, fw / 2, fh / 2), area = Math.abs(polyArea(cs));
        const near = st.kfs.findIndex(k => k.viewId === best.v.id && Math.hypot(k.center[0] - center[0], k.center[1] - center[1]) < 0.025 * best.v.w && Math.abs(Math.log(k.area / area)) < 0.3);
        const kf = { desc: f.desc, pts: f.pts, n: f.n, H: best.H, viewId: best.v.id, center, area, t: now };
        if (near >= 0) { st.kfs[near].desc.delete(); st.kfs[near] = kf; } else st.kfs.push(kf);
        if (st.kfs.length > P.kfMax) st.kfs.shift().desc.delete();
        kept = true;
      }
      if (!kept) f.desc.delete();
      return {
        ok: !!best, H: best ? best.H : null, viewId: best ? best.v.id : null, w: best ? best.v.w : 0, h: best ? best.v.h : 0,
        via: best ? best.via : '-', inliers: best ? best.inliers : 0, good: best ? best.good : 0,
        features: f.n, rejects, fw, fh, ms: Date.now() - t0
      };
    }

    function viewSize(id) { const v = views.get(id); return v ? { w: v.w, h: v.h } : null; }

    return {
      addView, setRegions, removeView, clear, processFrame, anchor, viewSize, transferRegions, viewQuality, resetTracking,
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
      trackPts: 200, minTrackPts: 80, trackMinPts: 15, maxTrackMs: 20000, histLen: 240, anchoredMs: 1500, lostPriorMs: 1500, trackMax: 480
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
        stableCab: null, candCab: null, candCount: 0, lostCount: 0 };
    }
    function reset() { if (st.prevGray) st.prevGray.delete(); st = fresh(); }

    function setView(id, w, h, regions) {
      const v = { w, h, regions: regions || [] };
      v.regionsPx = v.regions.map(r => ({ cabinetId: r.cabinetId, poly: r.poly.map(([x, y]) => [x * w, y * h]) }));
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

    function corners(gray, existing) {
      // uudet seurantapisteet tasaisesti; vältetään olemassa olevien läheisyyttä
      const kp = new cv.KeyPointVector();
      gftt.detect(gray, kp);
      const occ = new Set();
      const cell = (x, y) => Math.floor(x / 16) + ',' + Math.floor(y / 16);
      if (existing) for (let i = 0; i < existing.length; i += 2) occ.add(cell(existing[i], existing[i + 1]));
      const out = existing ? Array.from(existing) : [];
      for (let i = 0; i < kp.size() && out.length < P.trackPts * 2; i++) {
        const p = kp.get(i).pt, c = cell(p.x, p.y);
        if (occ.has(c)) continue;
        occ.add(c); out.push(p.x, p.y);
      }
      kp.delete();
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
    function flow(gray) {
      if (!st.prevGray || !st.prevPts || st.prevPts.length < 16 * 2) return null;
      if (st.prevGray.cols !== gray.cols || st.prevGray.rows !== gray.rows) return null;
      const n = st.prevPts.length / 2;
      const p0 = cv.matFromArray(n, 1, cv.CV_32FC2, Array.from(st.prevPts));
      const Mi = predict();
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
        if (m >= P.trackMinPts) {
          const cM = cv.matFromArray(m, 1, cv.CV_32FC2, cur), pM = cv.matFromArray(m, 1, cv.CV_32FC2, prev), mask = new cv.Mat();
          const Hm = cv.findHomography(cM, pM, cv.RANSAC, 3, mask, 1000, 0.99);
          if (!Hm.empty()) {
            let inl = 0; const keep = [];
            for (let i = 0; i < m; i++) if (mask.data[i]) { inl++; keep.push(cur[i * 2], cur[i * 2 + 1]); }
            const M = Array.from(Hm.data64F);
            if (inl >= P.trackMinPts && inl / m > 0.5 && sane(M)) out = { M, pts: new Float32Array(keep), inliers: inl };
          }
          cM.delete(); pM.delete(); mask.delete(); Hm.delete();
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
    function plausible(H, fw, fh, v) {
      if (!H || !v || !v.w) return false;
      if (H[0] * H[4] - H[1] * H[3] <= 0) return false;
      const cs = [[0, 0], [fw, 0], [fw, fh], [0, fh]].map(([x, y]) => apply(H, x, y));
      if (cs.some(c => !c || !isFinite(c[0]) || !isFinite(c[1]))) return false;
      if (!isConvex(cs)) return false;
      const a = Math.abs(polyArea(cs)), va = v.w * v.h;
      if (a < va * 0.0004 || a > va * 40) return false;
      const c = apply(H, fw / 2, fh / 2);
      if (!c || c[0] < -v.w || c[0] > 2 * v.w || c[1] < -v.h || c[1] > 2 * v.h) return false;
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
      if (!pts || pts.length < P.minTrackPts * 2) pts = corners(eq, pts);
      const t5 = Date.now();
      if (st.prevGray) st.prevGray.delete();
      st.prevGray = eq; st.prevPts = pts;
      if (st.lastT != null) st.dtPrev = st.dtNow;
      st.lastT = now; st.lastId = id;

      const res = describe(fw, fh, now);
      res.moving = moving;
      res.debug = { ms: Date.now() - t0, parts: { kuva: t1 - t0, LK: t2 - t1, ORB: t3 - t2, pisteet: t5 - t4 }, inliers: fl ? fl.inliers : 0, via: fl ? (fl.orb ? 'ORB' : 'LK') : '-', pts: opts.debug ? Array.from(pts.slice(0, 400)).map((x, i) => i % 2 ? x / fh : x / fw) : null, anchorAge: Math.round(now - st.anchorT) };
      let gray = null;
      if (opts.wantGray) gray = { data: new Uint8Array(raw.data), width: raw.cols, height: raw.rows, gray: true };
      raw.delete();
      // ennakkotieto tunnistukselle: nykyinen sijainti tai hetki sitten kadonnut (löysempi haku)
      let prior = null;
      if (st.H) prior = { H: scaleOut(st.H, k), viewId: st.viewId, loose: false };
      else if (st.lostH && now - st.lostT < P.lostPriorMs && views.has(st.lostView)) prior = { H: scaleOut(st.lostH, k), viewId: st.lostView, loose: true };
      return { res, gray, H: st.H, viewId: st.viewId, prior, id };
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
      if (!st.prevGray) return { applied: false, reason: 'ei kuvaa' };
      if (!plausible(H, fw, fh, views.get(a.viewId))) return { applied: false, reason: 'epäuskottava' };
      st.H = H; st.viewId = a.viewId; st.lostH = null; st.anchorT = st.lastT != null ? st.lastT : Date.now();
      return { applied: true, steps: st.hist.length - 1 - idx, res: describe(fw, fh, st.anchorT) };
    }

    return { processFrame, applyAnchor, setView, removeView, reset, hasView: id => views.has(id), get viewCount() { return views.size; }, get state() { return st; }, params: P };
  }

  const api = { createEngine, createTracker, _m: { mul, inv, apply, pointInPoly, isConvex } };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.KaappiVision = api;
})(typeof self !== 'undefined' ? self : this);
