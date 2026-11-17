"""Päästä päähän -testi: oikea Chrome, valekamera (synteettinen keittiövideo), kartoitus käyttöliittymän kautta."""
import asyncio, json, os, sys, time
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, 'test', 'data')
OUT = os.path.join(ROOT, 'test', 'shots'); os.makedirs(OUT, exist_ok=True)
URL = 'http://localhost:8765/'

async def main():
    meta = json.load(open(os.path.join(DATA, 'meta.json')))
    async with async_playwright() as p:
        b = await p.chromium.launch(executable_path='/usr/bin/google-chrome', headless=True, args=[
            '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream',
            f'--use-file-for-fake-video-capture={DATA}/cam.y4m'])
        ctx = await b.new_context(viewport={'width': 390, 'height': 800}, device_scale_factor=2, has_touch=False)
        await ctx.grant_permissions(['camera'], origin=URL)
        page = await ctx.new_page()
        errs = []
        page.on('console', lambda m: errs.append(f'{m.type}: {m.text}') if m.type in ('error', 'warning') else None)
        page.on('pageerror', lambda e: errs.append(f'pageerror: {e}'))
        await page.goto(URL)
        await page.wait_for_timeout(800)
        await page.screenshot(path=f'{OUT}/01_start.png')
        # lisää kartoituskuva
        await page.set_input_files('#inp-gallery', f'{DATA}/ref.jpg')
        await page.wait_for_selector('#editor:not(.hidden)', timeout=60000)
        await page.wait_for_timeout(1500)
        await page.screenshot(path=f'{OUT}/02_editor.png')
        # merkitse ovet käyttöliittymän kautta: U2, U3, L3 sekä U1
        names = {'U1': 'Mausteet', 'U2': 'Leivonta', 'U3': 'Kahvi ja tee', 'L3': 'Kattilat'}
        for i, (rid, nm) in enumerate(names.items()):
            poly = next(r['poly'] for r in meta['regions'] if r['name'] == rid)
            a = await page.evaluate('([x,y]) => { const r = document.querySelector("#ed-canvas").getBoundingClientRect(); const p = Editor.toScr(x,y); return [p[0]+r.left, p[1]+r.top]; }', poly[0])
            c = await page.evaluate('([x,y]) => { const r = document.querySelector("#ed-canvas").getBoundingClientRect(); const p = Editor.toScr(x,y); return [p[0]+r.left, p[1]+r.top]; }', poly[2])
            await page.click('#ed-add')
            await page.mouse.move(*a); await page.mouse.down()
            for k in range(1, 6): await page.mouse.move(a[0] + (c[0] - a[0]) * k / 5, a[1] + (c[1] - a[1]) * k / 5)
            await page.mouse.up()
            await page.wait_for_selector('#pk-new', timeout=5000)
            await page.fill('#pk-new', nm); await page.click('#pk-create')
            await page.wait_for_timeout(300)
            if i == 1: await page.screenshot(path=f'{OUT}/03_marked.png')
        n = await page.evaluate('Editor.v.regions.length'); print('regions', n)
        await page.click('#ed-back'); await page.wait_for_timeout(300)
        await page.screenshot(path=f'{OUT}/04_views.png')
        # sisällöt
        await page.click('#tabs button[data-scr="scr-cabs"]')
        contents = {'Leivonta': ['vehnäjauhot', 'sokeri', 'leivinjauhe', 'vaniljasokeri'], 'Kahvi ja tee': ['kahvi', 'suodatinpussit', 'vihreä tee'], 'Kattilat': ['iso kattila', 'paistinpannu'], 'Mausteet': ['suola', 'mustapippuri']}
        for nm, items in contents.items():
            await page.click(f'#cab-list .li:has-text("{nm}")')
            for it in items:
                await page.fill('#cb-add', it); await page.press('#cb-add', 'Enter')
            await page.click('#sheet .sheet-x')
        await page.fill('#search', 'sokeri'); await page.wait_for_timeout(200)
        await page.screenshot(path=f'{OUT}/05_search.png')
        if os.environ.get('THROTTLE'):
            cdp = await ctx.new_cdp_session(page)
            await cdp.send('Emulation.setCPUThrottlingRate', {'rate': float(os.environ['THROTTLE'])})
        # skanneriin, kohteena sokeri
        await page.click('#search-results [data-show]')
        await page.wait_for_timeout(500)
        await page.evaluate('''() => { window.__log = []; const o = Scan.onResult.bind(Scan); Scan.onResult = r => { window.__log.push([Math.round(performance.now()), r.mode, r.cabinetId ? cabName(r.cabinetId) : '-', r.debug ? r.debug.ms : 0, r.det ? r.det.ms : 0, r.debug ? r.debug.read : 0, r.debug ? r.debug.src : '']); o(r); }; }''')
        print('video', await page.evaluate('[document.querySelector("#video").videoWidth, document.querySelector("#video").videoHeight]'))
        seen = []
        t0 = time.time()
        for k in range(36):
            await page.wait_for_timeout(500)
            st = await page.evaluate('({t: document.querySelector("#rc-title").textContent, s: document.querySelector("#status").textContent, m: Scan.last && Scan.last.mode, c: Scan.last && Scan.last.cabinetId && cabName(Scan.last.cabinetId), ms: Scan.last && Scan.last.debug && Scan.last.debug.ms, fps: Scan.fps})')
            seen.append(st)
            if k in (4, 9, 14, 19): await page.screenshot(path=f'{OUT}/06_scan_{k:02d}.png')
        for s in seen: print(f"{s['m']:>6} {str(s['c']):>14} | kortti: {s['t']:<14} | {s['ms']} ms {s['fps']:.1f} fps")
        log = await page.evaluate('window.__log')
        print('käsiteltyjä kuvia', len(log), 'ajassa', (log[-1][0]-log[0][0])/1000, 's')
        print(' '.join(f"{x[1][0]}{x[2][:3]}" for x in log[:400]))
        import statistics
        print('seuranta ms mediaani', statistics.median([x[3] for x in log if x[3]]), '| tunnistus ms mediaani', statistics.median([x[4] for x in log if x[4]] or [0]))
        print('kuvalähde', set(x[6] for x in log if len(x) > 6), '| luku ms mediaani', statistics.median([x[5] or 0 for x in log]), '| synkronoitu piirto:', await page.evaluate('Scan.syncOn'), '| video piilossa:', await page.evaluate('document.querySelector("#video").style.visibility'))
        gaps = [b[0]-a[0] for a,b in zip(log, log[1:])]
        print('suurin tauko tulosten välillä', max(gaps), 'ms; tulosväli mediaani', statistics.median(gaps), 'ms')
        # vianetsintä päälle
        await page.click('#btn-debug'); await page.wait_for_timeout(2500)
        await page.screenshot(path=f'{OUT}/07_debug.png')
        # varmuuskopio/offline: SW ei toimi http:ssä, mutta tarkistetaan että sivun uudelleenlataus säilyttää tiedot
        await page.reload(); await page.wait_for_timeout(4000)
        kept = await page.evaluate('[S.cabinets.size, S.views.size, [...S.views.values()][0].regions.length]')
        print('after reload', kept)
        print('\n'.join(errs) or 'ei konsolivirheitä')
        await b.close()

asyncio.run(main())
