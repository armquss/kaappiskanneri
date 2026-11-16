const fs=require('fs');const {createEngine}=require('../vision.js');
(async()=>{let cv=require('@techstark/opencv-js');if(cv instanceof Promise)cv=await cv;else if(!cv.Mat)await new Promise(r=>cv.onRuntimeInitialized=r);
const meta=JSON.parse(fs.readFileSync('test/data/meta.json'));
const load=(f,w,h)=>({data:new Uint8ClampedArray(fs.readFileSync('test/data/'+f+'.rgba')),width:w,height:h});
const eng=createEngine(cv,JSON.parse(process.env.P||'{}'));eng.addView('A',load('ref',meta.W,meta.H),meta.regions.map(r=>({cabinetId:r.name,poly:r.poly})));
for(const stride of (process.env.ST||'1,2,3').split(',').map(Number)){eng.resetTracking();let out=[];
 for(let i=0;i<meta.seq.length;i+=stride){const fr=meta.seq[i];const r=eng.processFrame(load(fr.file,640,480),{debug:true});out.push(`${i}:${r.mode[0]}${r.mode==='lost'?'['+(r.debug.rejects||[]).join()+']':''}:${r.cabinetId||'-'}`)}
 console.log('stride',stride,out.join(' '))}})();
