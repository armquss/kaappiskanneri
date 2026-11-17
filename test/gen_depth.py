"""Syvyydellinen (2.5D) testiaineisto OIKEASTA kartoituskuvasta: kuva jaetaan syvyyskerroksiin
(yläkaappien ovet, takaseinä/hylly, alakaapit, verhot, huonekalut) ja uudet kuvakulmat renderöidään
kerros kerrallaan omalla tasohomografiallaan. Syntyy aito parallaksi: etualan huonekalut ja syvennetty
hylly liikkuvat eri tavalla kuin ovet. Oikea vastaus = ovien kulmat ovitason homografialla.

Käyttö: python3 test/gen_depth.py <varmuuskopio.json> <näkymän nimi> <ulostulokansio>
HUOM: varmuuskopio sisältää kuvan kodistasi - aineistoa ei viedä repoon.
"""
import base64, json, math, os, sys
import numpy as np, cv2

bk, view_name, O = sys.argv[1], sys.argv[2], sys.argv[3]
VARIANT = int(sys.argv[4]) if len(sys.argv) > 4 else 0  # kamerapolun muunnelma
os.makedirs(O, exist_ok=True)
d = json.load(open(bk))
v = next(x for x in d['views'] if x['name'] == view_name)
ref = cv2.imdecode(np.frombuffer(base64.b64decode(v['image'].split(',', 1)[1]), np.uint8), cv2.IMREAD_COLOR)
RH, RW = ref.shape[:2]
regions = [{'cabinetId': r['cabinetId'], 'poly': r['poly']} for r in v['regions']]
open(os.path.join(O, 'ref.rgba'), 'wb').write(cv2.cvtColor(ref, cv2.COLOR_BGR2RGBA).tobytes())

# --- kamerat: kartoituskuvan kamera (vaaka) ja puhelimen pystykamera
f_ref = (RW / 2) / math.tan(math.radians(37.5))
K_ref = np.array([[f_ref, 0, RW / 2], [0, f_ref, RH / 2], [0, 0, 1]])
LW, LH = 720, 1280
f_live = (LW / 2) / math.tan(math.radians(25))
K_live = np.array([[f_live, 0, LW / 2], [0, f_live, LH / 2], [0, 0, 1]])
Z0 = 3.0  # yläkaappien ovien etäisyys kartoituskuvan kamerasta (m)

# --- syvyyskartta kerroksittain (normalisoidut koordinaatit, myöhempi ylikirjoittaa)
depth = np.full((RH, RW), Z0 + 0.32, np.float32)               # takaseinä, välitila, hylly, katto
def rect(x0, y0, x1, y1, z): depth[int(y0 * RH):int(y1 * RH), int(x0 * RW):int(x1 * RW)] = z
rect(0, 0, 0.26, 1, Z0 - 0.4)        # vasen ikkunaseinä verhoineen
rect(0.73, 0, 1, 1, Z0 - 0.1)        # oikea seinä (taulu)
rect(0.26, 0.585, 0.73, 1, Z0 - 0.28) # työtaso ja alakaapit
for r in regions:                     # yläkaappien ovet = mitattava taso
    cv2.fillPoly(depth, [np.int32([[x * RW, y * RH] for x, y in r['poly']])], Z0)
rect(0.08, 0.68, 0.42, 1, Z0 - 1.3)  # pöytä ja tuolit
rect(0.585, 0.74, 1, 1, Z0 - 1.6)    # sohva
layers = sorted(set(np.round(depth.ravel(), 3).tolist()), reverse=True)  # kaukaisin ensin

def look_at(pos, target):
    f = np.array(target, float) - pos; f /= np.linalg.norm(f)
    r = np.cross(f, [0, -1, 0]); r /= np.linalg.norm(r)
    dn = np.cross(f, r)
    R = np.stack([r, dn, f]); return R, -R @ pos

def plane_H(R, t, z):
    n = np.array([[0, 0, 1.0]])
    return K_live @ (R + t.reshape(3, 1) @ n / z) @ np.linalg.inv(K_ref)

def px(nx, ny, z):  # kartoituskuvan piste -> 3D (kartoituskuvan kameran koordinaatit)
    return np.array([(nx * RW - RW / 2) / f_ref * z, (ny * RH - RH / 2) / f_ref * z, z])

c1 = np.mean([px(x, y, Z0) for x, y in regions[0]['poly']], 0)
c2 = np.mean([px(x, y, Z0) for x, y in regions[1]['poly']], 0) if len(regions) > 1 else c1
mid = (c1 + c2) / 2
# --- kamerapolku kuten käyttäjän videoilla: kaukaa -> lähelle kaappia alaviistosta -> toiseen kaappiin -> pois -> pöytään päin
keys = [
    (np.array([0, 0, 0.0]), mid + [0, 0.25, 0], 'laaja'),
    (np.array([-0.3, 0.05, 0.9]), c1 + [0, 0.1, 0], 'lähestyy'),
    (np.array([-0.55, 0.15, 1.9]), c1, 'lähellä, alaviisto'),
    (np.array([-0.6, 0.1, 2.25]), c1 + [0, 0.05, 0], 'hyvin lähellä'),
    (np.array([0.35, 0.1, 2.15]), c2, 'toinen kaappi lähellä'),
    (np.array([0.5, 0.25, 1.6]), c2 + [0.1, 0.15, 0], 'sivuttain'),
    (np.array([0.1, 0.1, 0.6]), mid + [0, 0.2, 0], 'peruuttaa'),
    (np.array([0.0, 0.0, 0.6]), np.array([-0.4, 0.9, 2.2]), 'pöytään päin'),
    (np.array([0.0, 0.0, 0.5]), mid + [0, 0.2, 0], 'takaisin'),
]
if VARIANT == 1:  # oikealta: ensin kaappi 2, matalalta ja jyrkästi ylös, sitten liuku kaappiin 1
    keys = [
        (np.array([0.3, 0.1, 0.0]), mid + [0.2, 0.2, 0], 'laaja'),
        (np.array([0.7, 0.35, 1.2]), c2 + [0, 0.1, 0], 'lähestyy'),
        (np.array([0.6, 0.45, 2.0]), c2, 'lähellä, alaviisto'),
        (np.array([0.45, 0.3, 2.3]), c2 + [-0.05, 0.05, 0], 'hyvin lähellä'),
        (np.array([-0.5, 0.3, 2.2]), c1, 'toinen kaappi lähellä'),
        (np.array([-0.7, 0.2, 1.5]), c1 + [-0.1, 0.1, 0], 'sivuttain'),
        (np.array([-0.2, 0.0, 0.4]), mid + [0, 0.2, 0], 'peruuttaa'),
    ]
elif VARIANT == 2:  # suoraan edestä, silmien korkeudelta, lähelle molempia vuorotellen
    keys = [
        (np.array([0.0, -0.1, 0.2]), mid + [0, 0.1, 0], 'laaja'),
        (np.array([-0.2, -0.2, 1.4]), c1 + [0, 0.15, 0], 'lähestyy'),
        (np.array([-0.4, -0.25, 2.1]), c1 + [0, 0.1, 0], 'lähellä, alaviisto'),
        (np.array([0.1, -0.25, 2.1]), mid + [0, 0.1, 0], 'hyvin lähellä'),
        (np.array([0.45, -0.25, 2.15]), c2 + [0, 0.1, 0], 'toinen kaappi lähellä'),
        (np.array([0.2, -0.1, 1.0]), mid + [0.1, 0.2, 0], 'sivuttain'),
        (np.array([0.0, 0.0, 0.3]), mid + [0, 0.2, 0], 'peruuttaa'),
    ]
steps = 18
frames = []
rng = np.random.default_rng(5 + VARIANT)
base = cv2.GaussianBlur(ref, (0, 0), 1.5)
k = 0
for s in range(len(keys) - 1):
    p0, t0, lab = keys[s]; p1, t1, _ = keys[s + 1]
    for i in range(steps):
        a = i / steps; a = a * a * (3 - 2 * a)  # pehmeä liike
        pos = p0 + (p1 - p0) * a + rng.normal(0, 0.004, 3)
        tgt = t0 + (t1 - t0) * a
        R, t = look_at(pos, tgt)
        out = cv2.warpPerspective(base, plane_H(R, t, Z0 + 0.32), (LW, LH), flags=cv2.INTER_LINEAR, borderValue=(40, 40, 40))  # aukkojen täyte
        for z in layers:
            if pos[2] > z - 0.15: continue  # kerros on jo kameran takana (esim. ohitettu pöytä)
            m = (np.abs(depth - z) < 1e-3).astype(np.uint8) * 255
            H = plane_H(R, t, z)
            # taso kameran takana? ohita
            if np.linalg.det(H) == 0: continue
            wm = cv2.warpPerspective(m, H, (LW, LH), flags=cv2.INTER_NEAREST)
            wi = cv2.warpPerspective(ref, H, (LW, LH), flags=cv2.INTER_LINEAR)
            out[wm > 0] = wi[wm > 0]
        g = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY).astype(np.float32)
        g = g * rng.uniform(0.85, 1.1) + rng.normal(0, 3, g.shape)
        g = cv2.GaussianBlur(np.clip(g, 0, 255).astype(np.uint8), (0, 0), 0.6)
        g = cv2.resize(g, (360, 640), interpolation=cv2.INTER_AREA)
        open(os.path.join(O, f'd{k:03d}.gray'), 'wb').write(g.tobytes())
        HF = plane_H(R, t, Z0)
        gt = []
        for r in regions:
            pts = np.float32([[x * RW, y * RH] for x, y in r['poly']]).reshape(-1, 1, 2)
            gt.append({'cabinetId': r['cabinetId'], 'poly': (cv2.perspectiveTransform(pts, HF).reshape(-1, 2) * 0.5).tolist()})
        frames.append({'file': f'd{k:03d}', 'seg': lab, 'gt': gt})
        if k % 18 == 9: cv2.imwrite(os.path.join(O, f'sample_{k:03d}.jpg'), g)
        k += 1
json.dump({'W': RW, 'H': RH, 'regions': regions, 'frames': frames, 'FW': 360, 'FH': 640}, open(os.path.join(O, 'meta.json'), 'w'))
print(len(frames), 'kuvaa,', len(layers), 'syvyyskerrosta')
