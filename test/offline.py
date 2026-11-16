import asyncio
from playwright.async_api import async_playwright
async def main():
    async with async_playwright() as p:
        b = await p.chromium.launch(executable_path='/usr/bin/google-chrome', headless=True, args=['--use-fake-ui-for-media-stream','--use-fake-device-for-media-stream'])
        ctx = await b.new_context(viewport={'width':390,'height':800})
        page = await ctx.new_page()
        await page.goto('http://localhost:8765/')
        await page.wait_for_function('navigator.serviceWorker.controller || false', timeout=60000) if False else None
        await page.evaluate('navigator.serviceWorker.ready')
        await page.wait_for_timeout(3000)
        await page.reload(); await page.wait_for_timeout(1500)
        ctrl = await page.evaluate('!!navigator.serviceWorker.controller')
        await ctx.set_offline(True)
        await page.reload(); await page.wait_for_timeout(1000)
        await page.wait_for_function('Engine.ready', timeout=60000)
        print('SW controls page:', ctrl, '| offline reload OK, engine:', await page.evaluate('Engine.status'))
        await b.close()
asyncio.run(main())
