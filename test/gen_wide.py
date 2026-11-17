# Laajat pystykuvat (koko seinä puhelimen kamerassa). Käyttö: python3 test/gen_wide.py test/data
import cv2, numpy as np, json, math, os
import sys
D=sys.argv[1] if len(sys.argv)>1 else 'test/data'; O=D+'/wide'; os.makedirs(O, exist_ok=True)
meta=json.load(open(D+'/meta.json')); W,H=meta['W'],meta['H']
wall=cv2.imread(D+'/ref.jpg')
# ympäristö seinän ympärille (katto/lattia), ettei reunat ole tasaväriä
big=np.full((H*3,W*3,3),0,np.uint8); r=np.random.default_rng(3)
n=r.normal(0,1,(H*3//24+2,W*3//24+2)).astype(np.float32); n=cv2.resize(n,(W*3,H*3))
big[:]=np.clip(150+n[...,None]*25,0,255).astype(np.uint8)
big[H:2*H,W:2*W]=wall
FW,FH=360,640
rng=np.random.default_rng(11); out=[]
def door_at(x,y):
    for reg in meta['regions']:
        p=np.array(reg['poly'])*[W,H]
        if p[0][0]<=x<=p[1][0] and p[0][1]<=y<=p[2][1]: return reg['name']
    return None
k=0
for vw in [900,1300,1800,2400,3000]:
    for t in range(8):
        reg=meta['regions'][rng.integers(0,8)]; p=np.array(reg['poly'])*[W,H]
        cx=rng.uniform(p[0][0]+40,p[1][0]-40); cy=rng.uniform(p[0][1]+40,p[2][1]-40)
        vh=vw*FH/FW; a=math.radians(rng.uniform(-8,8)); tx=rng.uniform(-.2,.2); ty=rng.uniform(-.15,.15)
        def wp(u,v):
            x,y=u*vw/2*(1+tx*v), v*vh/2*(1+ty*u)
            return [cx+W+x*math.cos(a)-y*math.sin(a), cy+H+x*math.sin(a)+y*math.cos(a)]
        src=np.float32([wp(-1,-1),wp(1,-1),wp(1,1),wp(-1,1)]); dst=np.float32([[0,0],[FW,0],[FW,FH],[0,FH]])
        M=cv2.getPerspectiveTransform(src,dst)
        fr=cv2.warpPerspective(big,M,(FW,FH),flags=cv2.INTER_AREA)
        f=fr.astype(np.float32)/255; f=np.power(np.clip(f*rng.uniform(.6,1.2),0,1),rng.uniform(.8,1.4))*255
        f+=rng.normal(0,4,f.shape); f=cv2.GaussianBlur(np.clip(f,0,255).astype(np.uint8),(0,0),rng.uniform(.3,1.0))
        g=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY)
        open(f'{O}/w{k:03d}.gray','wb').write(g.tobytes())
        if k%8==0: cv2.imwrite(f'{O}/w{k:03d}.jpg',f)
        Mi=np.linalg.inv(M); c=Mi@[FW/2,FH/2,1]; c=c[:2]/c[2]-[W,H]
        out.append({'file':f'w{k:03d}','vw':vw,'truth':door_at(*c)}); k+=1
json.dump(out,open(O+'/meta.json','w')); print(k)
