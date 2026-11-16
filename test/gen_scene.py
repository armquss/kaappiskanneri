"""Synteettinen keittiöseinä testejä varten: identtiset ovet + ympäristö (laatat, jääkaappi, liesi, ikkuna, tavarat).
Tuottaa: referenssikuvan, ovien alueet, testikuvia tunnetulla totuudella ja liikesarjan seurannan testaamiseen."""
import json, os, math, sys
import numpy as np, cv2

rng = np.random.default_rng(7)
OUT = sys.argv[1] if len(sys.argv) > 1 else 'test/data'
os.makedirs(OUT, exist_ok=True)
W, H = 2400, 1400

def noise_tex(h, w, base, amp, scale=8, seed=0):
    r = np.random.default_rng(seed)
    n = r.normal(0, 1, (h // scale + 2, w // scale + 2)).astype(np.float32)
    n = cv2.resize(n, (w, h), interpolation=cv2.INTER_CUBIC)
    fine = r.normal(0, 0.35, (h, w)).astype(np.float32)
    t = np.clip(np.array(base, np.float32)[None, None, :] + (n + fine)[..., None] * amp, 0, 255)
    return t.astype(np.uint8)

def wood(h, w, seed):
    r = np.random.default_rng(seed)
    y = np.arange(h)[:, None].astype(np.float32); x = np.arange(w)[None, :].astype(np.float32)
    g = np.sin(x / 9 + 3 * np.sin(y / 70) + r.normal(0, .3)) * 0.5 + np.sin(x / 3.1 + y / 400) * 0.25
    base = np.stack([60 + 25 * g, 110 + 30 * g, 160 + 35 * g], -1)  # BGR
    base += r.normal(0, 6, (h, w, 1))
    return np.clip(base, 0, 255).astype(np.uint8)

def make_wall(clutter_shift=0, seed_clutter=1):
    img = noise_tex(H, W, (205, 215, 222), 6, 16, 1)
    # laatoitus (välitila ylä- ja alakaappien välissä)
    ty0, ty1 = 520, 760
    tiles = noise_tex(ty1 - ty0, 1500, (225, 225, 215), 5, 6, 2)
    for yy in range(0, ty1 - ty0, 48):
        off = 0 if (yy // 48) % 2 == 0 else 48
        cv2.line(tiles, (0, yy), (1500, yy), (150, 150, 145), 3)
        for xx in range(-off, 1500, 96):
            cv2.line(tiles, (xx, yy), (xx, yy + 48), (150, 150, 145), 3)
    # muutama koristelaatta
    for (cx, cy) in [(230, 70), (610, 150), (1010, 90), (1330, 170)]:
        cv2.rectangle(tiles, (cx, cy), (cx + 90, cy + 44), (90, 120, 40), -1)
        cv2.circle(tiles, (cx + 45, cy + 22), 14, (30, 200, 230), -1)
    img[ty0:ty1, 60:1560] = tiles
    # ovet: täysin identtinen tekstuuri jokaisessa
    door_w, door_h = 340, 440
    tex_up = wood(door_h, door_w, 11)
    tex_lo = wood(560, door_w, 11)[:540]
    regions = []
    def door(x, y, w, h, tex, name, handle_at):
        img[y:y + h, x:x + w] = tex[:h, :w]
        cv2.rectangle(img, (x, y), (x + w, y + h), (40, 60, 80), 4)
        cv2.rectangle(img, (x + 22, y + 22), (x + w - 22, y + h - 22), (55, 85, 110), 3)
        hx, hy = handle_at
        cv2.rectangle(img, (x + hx - 6, y + hy - 45), (x + hx + 6, y + hy + 45), (190, 190, 195), -1)
        cv2.rectangle(img, (x + hx - 6, y + hy - 45), (x + hx + 6, y + hy + 45), (90, 90, 90), 2)
        regions.append({'name': name, 'poly': [[x / W, y / H], [(x + w) / W, y / H], [(x + w) / W, (y + h) / H], [x / W, (y + h) / H]]})
    for i in range(4):
        x = 80 + i * (door_w + 30)
        door(x, 60, door_w, door_h, tex_up, f'U{i+1}', (door_w - 40 if i % 2 == 0 else 40, door_h - 70))
    # työtaso
    cv2.rectangle(img, (40, 760), (1580, 800), (70, 70, 75), -1)
    img[762:798, 40:1580] = noise_tex(36, 1540, (85, 85, 90), 14, 3, 5)
    for i in range(4):
        x = 80 + i * (door_w + 30)
        door(x, 820, door_w, 540, tex_lo, f'L{i+1}', (door_w - 40 if i % 2 == 0 else 40, 80))
    # liesi alakaappien välissä? -> liesi on L2/L3 päällä työtasolla: keittolevy ja nupit
    cv2.rectangle(img, (470, 762), (1050, 798), (25, 25, 25), -1)
    for k in range(5):
        cv2.circle(img, (520 + k * 120, 780), 12, (200, 200, 200), -1)
        cv2.line(img, (520 + k * 120, 780), (520 + k * 120, 770), (20, 20, 20), 3)
    # jääkaappi
    fx, fy, fw, fh = 1640, 60, 420, 1300
    img[fy:fy + fh, fx:fx + fw] = noise_tex(fh, fw, (228, 230, 232), 4, 10, 6)
    cv2.rectangle(img, (fx, fy), (fx + fw, fy + fh), (120, 120, 125), 5)
    cv2.line(img, (fx, fy + 470), (fx + fw, fy + 470), (120, 120, 125), 5)
    cv2.rectangle(img, (fx + 30, fy + 300), (fx + 46, fy + 440), (150, 150, 155), -1)
    cv2.rectangle(img, (fx + 30, fy + 500), (fx + 46, fy + 700), (150, 150, 155), -1)
    # magneetit ja lappuset jääkaapissa
    for k, (mx, my) in enumerate([(1800, 600), (1900, 720), (1760, 860), (1950, 900), (1850, 1050)]):
        cv2.rectangle(img, (mx, my), (mx + 90, my + 110), (255, 250, 235) if k % 2 else (200, 240, 255), -1)
        for l in range(4): cv2.line(img, (mx + 10, my + 20 + l * 20), (mx + 75 - (l * 13) % 30, my + 20 + l * 20), (60, 60, 60), 2)
        cv2.circle(img, (mx + 45, my + 5), 9, (40, 40, 200) if k % 2 else (200, 80, 40), -1)
    cv2.putText(img, 'KAUPPALISTA', (1772, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (60, 60, 140), 2)
    # ikkuna
    wx, wy = 2120, 120
    cv2.rectangle(img, (wx, wy), (wx + 250, wy + 420), (250, 250, 250), -1)
    sky = noise_tex(390, 220, (230, 190, 140), 10, 20, 9)
    img[wy + 15:wy + 405, wx + 15:wx + 235] = sky
    cv2.line(img, (wx + 125, wy), (wx + 125, wy + 420), (245, 245, 245), 10)
    cv2.line(img, (wx, wy + 210), (wx + 250, wy + 210), (245, 245, 245), 10)
    for k in range(6):  # puun oksat ikkunassa
        cv2.line(img, (wx + 20 + k * 30, wy + 400), (wx + 60 + k * 25, wy + 250 - k * 20), (40, 90, 50), 4)
    # taulu seinällä
    cv2.rectangle(img, (2110, 700), (2370, 900), (30, 30, 30), -1)
    img[712:888, 2122:2358] = noise_tex(176, 236, (120, 150, 190), 40, 12, 10)
    # siirrettävät tavarat työtasolla
    r = np.random.default_rng(seed_clutter)
    for k in range(6):
        cx = int(150 + k * 230 + clutter_shift * (k % 3 - 1) * 60)
        hh = int(r.integers(80, 170)); ww = int(r.integers(50, 90))
        col = tuple(int(c) for c in r.integers(30, 230, 3))
        cv2.rectangle(img, (cx, 760 - hh), (cx + ww, 760), col, -1)
        cv2.rectangle(img, (cx + 8, 760 - hh + 20), (cx + ww - 8, 760 - hh + 50), (240, 240, 240), -1)
    return img, regions

def lighting(img, gain=1.0, gamma=1.0, tint=(1, 1, 1), noise=0.0, blur=0, vignette=0.0, seed=0):
    f = img.astype(np.float32) / 255
    f = np.power(np.clip(f * gain, 0, 1), gamma) * np.array(tint, np.float32)
    if vignette:
        h, w = f.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        d = ((xx - w / 2) ** 2 / (w / 2) ** 2 + (yy - h / 2) ** 2 / (h / 2) ** 2)
        f *= (1 - vignette * d)[..., None]
    f = f * 255
    if noise: f += np.random.default_rng(seed).normal(0, noise, f.shape)
    f = np.clip(f, 0, 255).astype(np.uint8)
    if blur: f = cv2.GaussianBlur(f, (0, 0), blur)
    return f

FW, FH = 640, 480

def cam_homography(cx, cy, view_w, rot_deg=0, tilt_x=0.0, tilt_y=0.0):
    """Homografia seinä -> kehys. Kehyksen keskipiste osuu seinän pisteeseen (cx,cy)."""
    view_h = view_w * FH / FW
    a = math.radians(rot_deg)
    def wall_pt(u, v):  # u,v in [-1,1]
        x, y = u * view_w / 2, v * view_h / 2
        # perspektiivi: kauempana oleva reuna näyttää kapeammalta -> seinällä leveämpi
        sx = 1 + tilt_x * v; sy = 1 + tilt_y * u
        x *= sx; y *= sy
        xr = x * math.cos(a) - y * math.sin(a); yr = x * math.sin(a) + y * math.cos(a)
        return [cx + xr, cy + yr]
    src = np.float32([wall_pt(-1, -1), wall_pt(1, -1), wall_pt(1, 1), wall_pt(-1, 1)])
    dst = np.float32([[0, 0], [FW, 0], [FW, FH], [0, FH]])
    M = cv2.getPerspectiveTransform(src, dst)
    # varmistetaan että keskipiste osuu (cx,cy): projisoi keskipiste
    Minv = np.linalg.inv(M)
    c = Minv @ np.array([FW / 2, FH / 2, 1.0]); c = c[:2] / c[2]
    return M, c

def door_at(regions, x, y):
    for r in regions:
        p = np.array(r['poly']) * [W, H]
        if p[0][0] <= x <= p[1][0] and p[0][1] <= y <= p[2][1]: return r['name']
    return None

def save_rgba(path, bgr):
    rgba = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGBA)
    with open(path, 'wb') as fh: fh.write(rgba.tobytes())

def main():
    wall, regions = make_wall()
    ref = lighting(wall, 1.0, 1.0, noise=2, seed=1)
    cv2.imwrite(f'{OUT}/ref.jpg', ref, [cv2.IMWRITE_JPEG_QUALITY, 90])
    save_rgba(f'{OUT}/ref.rgba', ref)
    # toinen referenssikuva: iltavalo + hieman vino kuvakulma (alueiden siirron testiin)
    M2 = cv2.getPerspectiveTransform(np.float32([[0, 0], [W, 0], [W, H], [0, H]]), np.float32([[60, 40], [W - 20, 0], [W - 90, H - 10], [20, H - 60]]))
    ref2 = cv2.warpPerspective(lighting(wall, 0.75, 1.25, (1.0, 0.92, 0.8), noise=4, seed=2), M2, (W, H), borderValue=(40, 40, 40))
    save_rgba(f'{OUT}/ref2.rgba', ref2)
    cv2.imwrite(f'{OUT}/ref2.jpg', ref2, [cv2.IMWRITE_JPEG_QUALITY, 90])
    regions2 = []
    for r in regions:
        p = cv2.perspectiveTransform((np.float32(r['poly']) * [W, H]).reshape(-1, 1, 2), M2).reshape(-1, 2) / [W, H]
        regions2.append({'name': r['name'], 'poly': p.tolist()})

    wall_moved, _ = make_wall(clutter_shift=1, seed_clutter=3)
    frames = []
    k = 0
    for r in regions:
        p = np.array(r['poly']) * [W, H]
        for trial in range(6):
            # satunnainen piste oven sisältä (ei aivan reunalta)
            x = rng.uniform(p[0][0] + 40, p[1][0] - 40); y = rng.uniform(p[0][1] + 40, p[2][1] - 40)
            view_w = float(rng.choice([700, 900, 1200, 1600]))
            M, c = cam_homography(x, y, view_w, rng.uniform(-12, 12), rng.uniform(-0.25, 0.25), rng.uniform(-0.2, 0.2))
            src = wall_moved if trial % 2 else wall
            fr = cv2.warpPerspective(src, M, (FW, FH), flags=cv2.INTER_LINEAR, borderValue=(60, 60, 60))
            fr = lighting(fr, rng.uniform(0.55, 1.3), rng.uniform(0.8, 1.4), tuple(rng.uniform(0.8, 1.05, 3)), noise=rng.uniform(2, 7), blur=rng.uniform(0, 1.2), vignette=rng.uniform(0, 0.35), seed=k)
            name = f'f{k:03d}'
            save_rgba(f'{OUT}/{name}.rgba', fr)
            if k < 6: cv2.imwrite(f'{OUT}/{name}.jpg', fr)
            frames.append({'file': name, 'truth': door_at(regions, c[0], c[1]), 'view_w': view_w, 'M': M.tolist()})
            k += 1
    # negatiivisia: näkymä jääkaappiin/ikkunaan (ei kaappia) ja täysin muu kuva
    for (x, y) in [(1850, 400), (2240, 330), (1850, 1000)]:
        M, c = cam_homography(x, y, 900, 5, 0.1, 0)
        fr = lighting(cv2.warpPerspective(wall, M, (FW, FH), borderValue=(60, 60, 60)), 0.9, noise=3, seed=k)
        save_rgba(f'{OUT}/f{k:03d}.rgba', fr); frames.append({'file': f'f{k:03d}', 'truth': None}); k += 1
    other = noise_tex(FH, FW, (120, 130, 140), 30, 20, 99)
    cv2.circle(other, (300, 200), 80, (20, 200, 20), -1)
    save_rgba(f'{OUT}/f{k:03d}.rgba', other); frames.append({'file': f'f{k:03d}', 'truth': None, 'foreign': True}); k += 1

    # liikesarja: laaja kuva -> zoomaus U2-oveen niin lähelle että vain ovi näkyy -> sivuttain U3:een
    seq = []
    path = []
    for i in range(30): t = i / 29; path.append((700 + 0 * t, 380, 1800 - 1500 * t))       # zoomaa U2:een
    for i in range(30): t = i / 29; path.append((700 + 370 * t, 380, 300))                  # liu'u U3:een lähikuvassa
    for i in range(20): t = i / 19; path.append((1070, 380 + 700 * t, 300 + 200 * t))         # alas L3:een
    for i, (x, y, vw) in enumerate(path):
        M, c = cam_homography(x, y, vw, 3 * math.sin(i / 7), 0.08 * math.sin(i / 9), 0)
        fr = lighting(cv2.warpPerspective(wall, M, (FW, FH), borderValue=(60, 60, 60)), 0.85, 1.1, noise=4, blur=0.7, seed=500 + i)
        save_rgba(f'{OUT}/s{i:03d}.rgba', fr)
        seq.append({'file': f's{i:03d}', 'truth': door_at(regions, c[0], c[1]), 'view_w': vw})
    cv2.imwrite(f'{OUT}/s045.jpg', cv2.cvtColor(np.frombuffer(open(f'{OUT}/s045.rgba', 'rb').read(), np.uint8).reshape(FH, FW, 4), cv2.COLOR_RGBA2BGR))

    json.dump({'W': W, 'H': H, 'FW': FW, 'FH': FH, 'regions': regions, 'regions2': regions2, 'frames': frames, 'seq': seq}, open(f'{OUT}/meta.json', 'w'))
    # video Chromen valekameraa varten (y4m), sama liikesarja edestakaisin
    with open(f'{OUT}/cam.y4m', 'wb') as v:
        v.write(f'YUV4MPEG2 W{FW} H{FH} F15:1 Ip A1:1 C420jpeg\n'.encode())
        order = list(range(len(path))) + list(range(len(path) - 1, -1, -1))
        for i in order:
            rgba = np.frombuffer(open(f'{OUT}/s{i:03d}.rgba', 'rb').read(), np.uint8).reshape(FH, FW, 4)
            yuv = cv2.cvtColor(cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR), cv2.COLOR_BGR2YUV_I420)
            v.write(b'FRAME\n'); v.write(yuv.tobytes())
    print('frames', len(frames), 'seq', len(seq))

main()
