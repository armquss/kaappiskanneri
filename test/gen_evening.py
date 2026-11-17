"""Iltavalaistus-testiaineisto oppivalle kartoitukselle.
 - learn: yhtenäinen kameraliike (1. käyttökerta), jonka aikana avainkuvat opitaan
 - test:  satunnaisia laajoja kuvia samassa valaistuksessa (2. käyttökerta, ei ennakkotietoa)
 - close: pelkkiä ovien lähikuvia (identtiset ovet) -> ei saa koskaan tunnistua väärin
Kehykset ovat pystykuvia 360x640 kuten puhelimessa. Käyttö: python3 test/gen_evening.py test/data
"""
import json, math, os, sys
import numpy as np, cv2

D = sys.argv[1] if len(sys.argv) > 1 else 'test/data'
O = os.path.join(D, 'evening'); os.makedirs(O, exist_ok=True)
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gen_scene.py')).read()
src = src[:src.rindex('main()')]  # vain funktiot
g = {'__name__': 'gen'}; exec(src.replace("OUT = sys.argv[1] if len(sys.argv) > 1 else 'test/data'", "OUT = '/tmp'").replace('os.makedirs(OUT, exist_ok=True)', ''), g)
make_wall, W, H = g['make_wall'], g['W'], g['H']
meta = json.load(open(os.path.join(D, 'meta.json')))

wall, regions = make_wall(clutter_shift=1, seed_clutter=5)  # tavarat eri paikoissa kuin kartoituskuvassa
# ympäristö seinän ympärille
big = np.zeros((H * 3, W * 3, 3), np.uint8)
r0 = np.random.default_rng(4)
n = cv2.resize(r0.normal(0, 1, (H * 3 // 24 + 2, W * 3 // 24 + 2)).astype(np.float32), (W * 3, H * 3))
big[:] = np.clip(140 + n[..., None] * 25, 0, 255).astype(np.uint8)
big[H:2 * H, W:2 * W] = wall
FW, FH = 360, 640

def evening(img, rng):
    """Lämmin ja hämärä iltavalo, kattolampun valokeila ja kohina (paikallisia eroja, ei vain koko kuvan kirkkautta)."""
    f = img.astype(np.float32) / 255
    h, w = f.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    lx, ly = w * rng.uniform(0.3, 0.7), -h * 0.1
    spot = np.exp(-(((xx - lx) / (w * 0.6)) ** 2 + ((yy - ly) / (h * 0.7)) ** 2))
    light = 0.35 + 0.75 * spot
    f = np.power(np.clip(f * light[..., None], 0, 1), 1.35) * np.array([0.72, 0.86, 1.0], np.float32)
    f = f * 255 + rng.normal(0, 7, f.shape)
    f = np.clip(f, 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(f, (0, 0), 0.7)

def cam(cx, cy, vw, rot=0.0, tx=0.0, ty=0.0):
    vh = vw * FH / FW; a = math.radians(rot)
    def wp(u, v):
        x, y = u * vw / 2 * (1 + tx * v), v * vh / 2 * (1 + ty * u)
        return [cx + W + x * math.cos(a) - y * math.sin(a), cy + H + x * math.sin(a) + y * math.cos(a)]
    src = np.float32([wp(-1, -1), wp(1, -1), wp(1, 1), wp(-1, 1)])
    return cv2.getPerspectiveTransform(src, np.float32([[0, 0], [FW, 0], [FW, FH], [0, FH]]))

def door_at(x, y):
    for reg in regions:
        p = np.array(reg['poly']) * [W, H]
        if p[0][0] <= x <= p[1][0] and p[0][1] <= y <= p[2][1]: return reg['name']
    return None

def render(name, M, rng):
    fr = cv2.warpPerspective(big, M, (FW, FH), flags=cv2.INTER_AREA)
    fr = evening(fr, rng)
    open(os.path.join(O, name + '.gray'), 'wb').write(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY).tobytes())
    Mi = np.linalg.inv(M); c = Mi @ [FW / 2, FH / 2, 1]; c = c[:2] / c[2] - [W, H]
    return fr, door_at(*c)

out = {'learn': [], 'test': [], 'close': []}
rng = np.random.default_rng(21)
# 1) oppimisliike: laaja kuva -> pyyhkäisy seinän yli -> lähestyminen jokaiseen yläkaappiin ja takaisin
path = []
for i in range(25): t = i / 24; path.append((300 + 1300 * t, 600, 2200))
for d in range(4):
    cx = 250 + d * 370
    for i in range(12): t = i / 11; path.append((cx, 300 + 300 * (1 - t), 2200 - 1500 * t))
    for i in range(12): t = i / 11; path.append((cx, 300 + 300 * t, 700 + 1500 * t))
for i, (x, y, vw) in enumerate(path):
    fr, truth = render(f'l{i:03d}', cam(x, y, vw, 3 * math.sin(i / 9), 0.06 * math.sin(i / 13), 0), rng)
    out['learn'].append({'file': f'l{i:03d}', 'truth': truth})
    if i == 30: cv2.imwrite(os.path.join(O, 'sample_learn.jpg'), fr)
# 2) toinen käyttökerta: laajat satunnaiset kuvat
for k in range(40):
    reg = regions[rng.integers(0, 8)]; p = np.array(reg['poly']) * [W, H]
    x, y = rng.uniform(p[0][0] + 40, p[1][0] - 40), rng.uniform(p[0][1] + 40, p[2][1] - 40)
    vw = float(rng.choice([1000, 1400, 1900, 2500]))
    fr, truth = render(f't{k:03d}', cam(x, y, vw, rng.uniform(-8, 8), rng.uniform(-.15, .15), rng.uniform(-.1, .1)), rng)
    out['test'].append({'file': f't{k:03d}', 'truth': truth, 'vw': vw})
    if k == 0: cv2.imwrite(os.path.join(O, 'sample_test.jpg'), fr)
# 3) pelkät lähikuvat ovista (kaikki identtisiä)
for k in range(32):
    reg = regions[k % 8]; p = np.array(reg['poly']) * [W, H]
    x, y = rng.uniform(p[0][0] + 60, p[1][0] - 60), rng.uniform(p[0][1] + 80, p[2][1] - 80)
    fr, truth = render(f'c{k:03d}', cam(x, y, float(rng.uniform(260, 380)), rng.uniform(-6, 6)), rng)
    out['close'].append({'file': f'c{k:03d}', 'truth': truth})
json.dump(out, open(os.path.join(O, 'meta.json'), 'w'))
print({k: len(v) for k, v in out.items()})
