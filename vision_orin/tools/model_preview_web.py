#!/usr/bin/env python3
"""Browser preview using the production model decisions. Optional --teensy uses the existing BEANS hold-test protocol."""
import argparse
import json
import os
from pathlib import Path
import secrets
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from model_runtime.core import Detector, ROI_KEYS, ROI_PATHS, load_roi, regions, settings
from model_runtime.capture import Camera

BENEFITS_CONFIG = Path(os.environ.get('LARC_BENEFITS_CONFIG', ROOT/'benefits/benefits_config.json'))
BENEFITS_KEYS = ('p1_x', 'p1_y', 'p2_x', 'p2_y', 'p3_x', 'p3_y', 'roi_size')

def validate_benefits_roi(values, width, height):
    if set(values) != set(BENEFITS_KEYS) or any(type(v) is not int for v in values.values()):
        raise ValueError('Use integer values for every benefits ROI field')
    size = values['roi_size']
    if not 2 <= size <= min(width, height):
        raise ValueError('ROI size must be at least 2 pixels and fit the image')
    half = size // 2
    for point in ('p1', 'p2', 'p3'):
        x, y = values[point+'_x'], values[point+'_y']
        if not half <= x <= width-half or not half <= y <= height-half:
            raise ValueError(f'{point} must keep its square inside the image')


def benefits_result(frame, cfg, captured_at):
    import cv2
    from benefits.benefits import analyze, BOX_NAMES
    started = time.monotonic()
    result = analyze(frame, cfg)
    image = frame.copy()
    for label, (x1,y1,x2,y2) in zip('LCR', result['boxes']):
        cv2.rectangle(image, (x1,y1), (x2,y2), (255,255,255), 2)
        cv2.putText(image, label, (x1,max(18,y1-6)), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 2)
    message = f"VISION:FE:{result['box']:02X}"
    cv2.putText(image, f"{result['align']} box={BOX_NAMES[result['box']]} {message}",
                (8,25), cv2.FONT_HERSHEY_SIMPLEX, .65, (80,220,120), 2)
    age = (time.monotonic()-captured_at)*1000
    error = 'Frame too old; outputs neutral' if age > 400 else None
    return dict(message='VISION:FE:00' if error else message, error=error,
                align=result['align'], box=BOX_NAMES[result['box']],
                colours=[BOX_NAMES[c] for c in result['colours']],
                percentages=result['pcts'], frame_age_ms=round(age,1),
                processing_ms=round((time.monotonic()-started)*1000,1),
                width=frame.shape[1], height=frame.shape[0]), image


HTML = '''<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>LARC model preview</title><style>
body{font:16px system-ui;background:#121820;color:#eee;margin:24px;max-width:1400px}
.roi-view{position:relative;display:inline-block;max-width:100%;touch-action:none}.roi-view img{display:block;user-select:none}.roi-view svg{position:absolute;inset:0;width:100%;height:100%;cursor:crosshair}section{background:#202b38;padding:20px;border-radius:12px;margin:20px 0}img{width:100%;height:auto}.roi-view{width:100%}button.selected{background:#8ce0ba} .command{font-size:1.15rem;color:#8ce0ba}
input{width:85px;margin:8px;padding:8px}button{padding:10px;cursor:pointer}pre{white-space:pre-wrap}label{display:inline-block}
</style><h1>LARC · Model preview</h1>
<p>SERVO_MODE Views use original camera orientation and pixel coordinates. Right ZED view = upper intake; left = lower.</p>
<p>Drag on an intake or separator image to select an ROI, then click Save ROI to apply it immediately. Intake views share width and vertical limits; horizontal centers are separate. Model weights are loaded at startup.</p>
<p><b>Timing (ms):</b> inference = complete YOLO call; network = neural network only; decision = model + ROI logic; processing = model + decisions (+ preview annotations); frame_age = camera delivery to result; output_interval = time between results. These exclude servo movement and browser/network delay.</p><p id="runtime">Starting preview…</p><p>Benefits: select L, C or R, click its position on the image, then Save ROI. Benefits uses color detection, not the beans model.</p><pre id="teensy">Serial disabled</pre><div id="panels"></div><script>
const roles=ROLES, token=TOKEN;
for(const role of roles){const s=document.createElement('section');s.innerHTML=`<h2>${role}</h2><div class="roi-view"><img id="img-${role}" draggable="false"><svg id="overlay-${role}"></svg></div><p class="command" id="command-${role}">Waiting for frames…</p><div id="points-${role}"></div><details><summary>Detection details</summary><pre id="state-${role}">Starting…</pre></details><form id="form-${role}"></form><p id="saved-${role}"></p>`;document.querySelector('#panels').append(s);}
let initialized=false;
const dimensions={};
let selectedPoint="p1";
for(const role of roles){
 const overlay=document.querySelector(`#overlay-${role}`);let start=null;
 function point(e){const box=overlay.getBoundingClientRect(),d=dimensions[role];if(!d||!box.width||!box.height)return null;return [Math.max(0,Math.min(d[0],Math.round((e.clientX-box.left)*d[0]/box.width))),Math.max(0,Math.min(d[1],Math.round((e.clientY-box.top)*d[1]/box.height)))];}
 if(role==='benefits'){
  const controls=document.querySelector('#points-benefits');
  for(const [key,label] of [['p1','L'],['p2','C'],['p3','R']]){
   const b=document.createElement('button');b.type='button';b.textContent=label;b.classList.toggle('selected',key===selectedPoint);
   b.onclick=()=>{selectedPoint=key;for(const button of controls.children)button.classList.toggle('selected',button===b);};controls.append(b);
  }
  overlay.onclick=e=>{const p=point(e);const form=document.querySelector('#form-benefits');if(!p||!form.elements.namedItem('roi_size'))return;
   const d=dimensions[role],half=Math.floor(Number(form.elements.namedItem('roi_size').value)/2);
   const x=Math.max(half,Math.min(d[0]-half,p[0])),y=Math.max(half,Math.min(d[1]-half,p[1]));
   form.elements.namedItem(selectedPoint+'_x').value=x;form.elements.namedItem(selectedPoint+'_y').value=y;
   overlay.innerHTML=`<rect x="${x-half}" y="${y-half}" width="${half*2}" height="${half*2}" fill="cyan" fill-opacity="0.2" stroke="cyan" stroke-width="2"/>`;
   document.querySelector('#saved-benefits').textContent='Point selected — Save ROI to apply and keep it.';
  };
  continue;
 }
 function bounds(end){const d=dimensions[role];let [x1,x2]=[Math.min(start[0],end[0]),Math.max(start[0],end[0])];if(role==='intake'){const half=d[0]/2,offset=start[0]<half?0:half;x1=Math.max(offset,x1);x2=Math.min(offset+half,x2);}return [x1,Math.min(start[1],end[1]),x2,Math.max(start[1],end[1])];}
 overlay.onpointerdown=e=>{if(e.button!==0)return;start=point(e);if(start){overlay.setPointerCapture(e.pointerId);e.preventDefault();}};
 overlay.onpointermove=e=>{if(!start)return;const b=bounds(point(e));overlay.innerHTML=`<rect x="${b[0]}" y="${b[1]}" width="${b[2]-b[0]}" height="${b[3]-b[1]}" fill="cyan" fill-opacity="0.2" stroke="cyan" stroke-width="2"/>`;};
 overlay.onpointerup=e=>{if(!start)return;const b=bounds(point(e)),d=dimensions[role],side=start[0]<d[0]/2?'det_cy_l':'det_cy_r';start=null;if(b[2]-b[0]<2||b[3]-b[1]<2)return;
 const form=document.querySelector(`#form-${role}`),set=(k,v)=>{form.elements.namedItem(k).value=v;};
 if(role==='separator'){set('roi_x',b[0]);set('roi_y',b[1]);set('roi_w',b[2]-b[0]);set('roi_h',b[3]-b[1]);}
 else{const half=d[0]/2,t=Math.floor((b[2]-b[0])/2),offset=side==='det_cy_l'?0:half;set(side,b[0]-offset+t);set('det_thick',t);set('trigger_x',b[1]);set('trig_y2',b[3]);for(const key of ['det_cy_l','det_cy_r'])set(key,Math.max(t,Math.min(half-t,Number(form.elements.namedItem(key).value))));}
 document.querySelector(`#saved-${role}`).textContent='Selection ready — click Save ROI to apply. Intake width and vertical limits are shared.';
 };
 overlay.onpointercancel=()=>{start=null;overlay.innerHTML='';};
}

async function poll(){try{const data=await (await fetch('/state')).json();document.querySelector('#runtime').textContent=data.runtime||'';document.querySelector('#teensy').textContent=data.teensy ? JSON.stringify(data.teensy,null,2) : 'Preview only — serial disabled';for(const role of roles){let x=data[role]||{};if(x.width&&x.height){dimensions[role]=[x.width,x.height];document.querySelector(`#overlay-${role}`).setAttribute("viewBox",`0 0 ${x.width} ${x.height}`);}
document.querySelector(`#state-${role}`).textContent=JSON.stringify(x,null,2);
document.querySelector(`#command-${role}`).textContent=(x.message||'Waiting for frames…')+(x.box?' · '+x.align+' · '+x.box:'')+(x.error?' · '+x.error:'');
document.querySelector(`#img-${role}`).src=`/frame/${role}?t=${Date.now()}`;
if(!initialized){let form=document.querySelector(`#form-${role}`);for(const [k,v] of Object.entries(data.roi[role])){let label=document.createElement('label');label.textContent=k;let input=document.createElement('input');input.type='number';input.name=k;input.value=v;input.required=true;label.append(input);form.append(label);}let button=document.createElement('button');button.textContent='Save ROI';form.append(button);
form.onsubmit=async e=>{e.preventDefault();let values=Object.fromEntries([...new FormData(form)].map(([k,v])=>[k,Number(v)]));let r=await fetch('/roi/'+role,{method:'POST',headers:{'Content-Type':'application/json','X-Preview-Token':token},body:JSON.stringify(values)});document.querySelector(`#saved-${role}`).textContent=await r.text();if(r.ok)document.querySelector(`#overlay-${role}`).innerHTML="";};}
}initialized=true;}catch(e){console.log(e)}setTimeout(poll,700)}poll();</script>'''

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bind', default='127.0.0.1')
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--teensy', action='store_true', help='Enable real servo control via the BEANS hold sketch')
    p.add_argument('--serial-device')
    p.add_argument('--model')
    p.add_argument('--device')
    p.add_argument('--no-separator', action='store_true')
    p.add_argument('--intake-device')
    p.add_argument('--separator-device')
    p.add_argument('--benefits-device')
    p.add_argument('--no-benefits', action='store_true', help='Hide the benefits camera')
    args=p.parse_args()
    model_roles=['intake'] if args.no_separator else ['intake','separator']
    roles=model_roles + ([] if args.no_benefits else ['benefits'])
    options=settings(weights=args.model, device=args.device)
    detector=None
    token=secrets.token_urlsafe(24)
    lock=threading.Lock()
    stop=threading.Event()
    state={}
    jpeg={}
    configs={r:load_roi(r) for r in model_roles}
    roi_keys={r:ROI_KEYS[r] for r in model_roles}
    roi_paths={r:ROI_PATHS[r] for r in model_roles}
    if 'benefits' in roles:
        from benefits.benefits import DEFAULT_CFG
        saved=json.loads(BENEFITS_CONFIG.read_text()) if BENEFITS_CONFIG.exists() else {}
        configs['benefits']={**DEFAULT_CFG, **saved}
        roi_keys['benefits']=BENEFITS_KEYS
        roi_paths['benefits']=BENEFITS_CONFIG
    cameras={}
    runtime='Loading model — camera views will appear as they become ready.'
    teensy = None
    if args.teensy:
        sys.path.insert(0, str(ROOT/'link'))
        from beans_bench import BeansBench
        try:
            teensy = BeansBench(device=args.serial_device)
        except Exception:
            for camera in cameras.values(): camera.close()
            raise
    def worker_body():
        nonlocal detector, runtime
        import cv2
        try:
            detector=Detector(options)
            for role in roles:
                if stop.is_set(): return
                with lock: runtime=f'Opening {role} camera…'
                try:
                    cameras[role]=Camera(role, getattr(args, role+'_device'))
                except Exception as exc:
                    with lock: state[role]={'error':str(exc), 'message':'NEUTRAL'}
            with lock: runtime='Preview running. Benefits is preview only; no benefit door commands are sent.'
        except Exception as exc:
            with lock: runtime=f'Startup failed: {type(exc).__name__}: {exc}'
            return
        seen={r:0 for r in cameras}
        while not stop.is_set():
            worked=False
            for role,camera in cameras.items():
                frame,stamp,seq,error=camera.latest()
                if error or frame is None or time.monotonic()-stamp > .5:
                    if teensy and role in model_roles: teensy.publish(role, {'error': error or 'No fresh frame'}, stamp)
                    with lock:
                        state[role]={'error':error or 'Waiting for fresh camera frames', 'message':'NEUTRAL'}
                    continue
                if seq == seen[role]:
                    continue
                worked=True
                seen[role]=seq
                with lock:
                    cfg=dict(configs[role])
                try:
                    if role=='benefits':
                        out,image=benefits_result(frame,cfg,stamp)
                    else:
                        out,image=detector.run(frame,role,cfg,stamp,annotate=True)
                    if teensy and role in model_roles: teensy.publish(role, out, stamp)
                    ok,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])
                    with lock:
                        state[role]={**out,'updated':time.monotonic()}
                        if ok:
                            jpeg[role]=encoded.tobytes()
                except Exception as exc:
                    if teensy and role in model_roles: teensy.publish(role, {'error': str(exc)}, stamp)
                    with lock:
                        state[role]={'error':str(exc), 'message':'NEUTRAL'}
            if not worked:
                stop.wait(.02)
    def worker():
        nonlocal runtime
        try:
            worker_body()
        except Exception as exc:
            with lock: runtime=f'Preview stopped: {type(exc).__name__}: {exc}'
        finally:
            for camera in list(cameras.values()): camera.close()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):
            pass
        def reply(self,status,body,kind='text/plain'):
            if isinstance(body,str): body=body.encode()
            self.send_response(status)
            self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError): pass
        def do_GET(self):
            path=urlparse(self.path).path
            if path=='/':
                self.reply(200,HTML.replace('ROLES',json.dumps(roles)).replace('TOKEN',json.dumps(token)).replace('SERVO_MODE', 'REAL SERVO CONTROL enabled — waiting for Teensy BEANS request.' if teensy else 'Preview only — no servo commands.'),'text/html; charset=utf-8')
            elif path=='/state':
                with lock:
                    payload={r:dict(state.get(r,{})) for r in roles}
                    for r in roles:
                        if 'updated' in payload[r]:
                            age=(time.monotonic()-payload[r].pop('updated'))*1000
                            payload[r]['result_age_ms']=round(age)
                            if age>500:
                                payload[r]['message']='STALE — NEUTRAL'
                    payload['roi']={r:{k:configs[r][k] for k in roi_keys[r]} for r in roles}
                    payload['runtime']=runtime
                payload['teensy'] = teensy.snapshot() if teensy else None
                self.reply(200,json.dumps(payload),'application/json')
            elif path.startswith('/frame/'):
                with lock: body=jpeg.get(path.split('/')[-1])
                self.reply(200 if body else 404,body or b'Waiting for image','image/jpeg' if body else 'text/plain')
            else: self.reply(404,'Not found')
        def do_POST(self):
            role=urlparse(self.path).path.removeprefix('/roi/')
            if role not in roles or self.headers.get('X-Preview-Token')!=token:
                return self.reply(403,'Invalid request')
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<4096: raise ValueError('Invalid request size')
                values=json.loads(self.rfile.read(length))
                if set(values)!=set(roi_keys[role]) or any(type(v) is not int for v in values.values()):
                    raise ValueError('Use integer values for all ROI fields')
                with lock:
                    current=state.get(role,{})
                    w,h=current.get('width'),current.get('height')
                    if not w or not h: raise ValueError('Wait for a camera image before saving its ROI')
                    new={**configs[role],**values}
                    if role=='benefits':
                        validate_benefits_roi(values,w,h)
                    else:
                        regions(role,new,(h,w),detector.expected[role])
                    path=roi_paths[role]
                    document=json.loads(path.read_text()) if path.exists() else dict(configs[role])
                    # One backup of the original; retain all HSV/other keys.
                    backup=path.with_suffix(path.suffix+'.before-model-preview')
                    if path.exists() and not backup.exists(): backup.write_text(path.read_text())
                    document.update(values)
                    if role=='separator' and isinstance(document.get('roi'),dict):
                        document['roi']={k:values['roi_'+k] for k in ('x','y','w','h')}
                    tmp=path.with_suffix('.json.tmp')
                    tmp.write_text(json.dumps(document,indent=2)+'\n')
                    tmp.replace(path)
                    configs[role]=new
                self.reply(200,'Saved and applied to this preview. Restart the dispatcher after closing the preview to use the saved settings.')
            except (ValueError,TypeError,KeyError,OSError) as exc:
                self.reply(400,str(exc))
    server = None
    thread = None
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        server=ThreadingHTTPServer((args.bind,args.port),Handler)
        thread=threading.Thread(target=worker,daemon=True)
        thread.start()
        print(f'Preview: http://{args.bind}:{args.port} — ' + ('REAL servo control enabled' if teensy else 'no servo actuation'),flush=True)
        server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        stop.set()
        if teensy: teensy.close()
        if server: server.server_close()
        if thread: thread.join(timeout=3)

if __name__=='__main__': main()
