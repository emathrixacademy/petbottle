#!/usr/bin/env python3
"""
Pi Admin — Autonomous Robot Control Panel.
Port 8080. All navigator calls proxied through here (no CORS issues).
"""

import os, csv, time, threading, json
import urllib.request
from datetime import datetime
from flask import Flask, jsonify, request, send_file, Response

app = Flask(__name__)
NAV = "http://127.0.0.1:5000"
DATA_DIR = "/home/set-admin/testing/test_data"
os.makedirs(DATA_DIR, exist_ok=True)

recording = {"active": False, "session": None, "file": None, "writer": None,
             "model": "", "start": 0, "rows": 0, "lock": threading.Lock()}

def get_ble_status():
    try:
        d = nav_get("/stats", timeout=3)
        return d.get("bleConnected", False)
    except Exception:
        return False

def nav_get(path, timeout=5):
    try:
        req = urllib.request.urlopen(f"{NAV}{path}", timeout=timeout)
        return json.loads(req.read())
    except Exception as e:
        return {"ok": False, "error": str(e)}

def recorder_loop():
    while True:
        with recording["lock"]:
            active = recording["active"]
        if not active:
            time.sleep(1)
            continue
        try:
            d = nav_get("/stats", timeout=3)
            row = [datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                   d.get("model", ""), d.get("bottles", 0), d.get("persons", 0),
                   d.get("fps", 0), d.get("inference_ms", 0), d.get("state", "")]
            with recording["lock"]:
                if recording["writer"]:
                    recording["writer"].writerow(row)
                    recording["file"].flush()
                    recording["rows"] += 1
        except Exception as e:
            print(f"[recorder] error: {e}")
        time.sleep(1)

threading.Thread(target=recorder_loop, daemon=True).start()

# ── Proxy routes ─────────────────────────────────────────────

@app.route("/nav/stats")
def nav_stats():
    return jsonify(nav_get("/stats"))

@app.route("/nav/models")
def nav_models():
    return jsonify(nav_get("/models"))

@app.route("/nav/switch_model")
def nav_switch():
    model = request.args.get("model", "")
    return jsonify(nav_get(f"/switch_model?model={model}"))

@app.route("/nav/start")
def nav_start():
    return jsonify(nav_get("/start"))

@app.route("/nav/stop")
def nav_stop():
    return jsonify(nav_get("/stop"))

@app.route("/nav/cmdlog")
def nav_cmdlog():
    return jsonify(nav_get("/cmdlog"))

@app.route("/nav/data_log")
def nav_data_log():
    try:
        req = urllib.request.urlopen(f"{NAV}/data_log", timeout=10)
        data = req.read()
        return Response(data, mimetype="text/csv",
                       headers={"Content-Disposition": "attachment; filename=navigation_log.csv"})
    except Exception as e:
        return Response(f"Navigator not available: {e}", status=502)

@app.route("/nav/video_feed")
def nav_video():
    try:
        req = urllib.request.urlopen(f"{NAV}/video_feed", timeout=10)
        def generate():
            while True:
                chunk = req.read(4096)
                if not chunk:
                    break
                yield chunk
        return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
    except Exception as e:
        return Response(f"Navigator not available: {e}", status=502)

# ── Main page ────────────────────────────────────────────────

PAGE_HTML = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>PET Bottle Robot</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#1a1a2e;color:#e0e0e0;font-family:'Segoe UI',system-ui,sans-serif;
  padding:12px;-webkit-user-select:none;user-select:none;
  -webkit-tap-highlight-color:transparent}
h1{color:#4fc3f7;font-size:1.4rem;text-align:center;font-weight:800;margin-bottom:2px}
.subtitle{text-align:center;font-size:.8rem;color:#888;margin-bottom:10px}
.card{background:#16213e;border-radius:14px;padding:14px;margin:10px 0;
  box-shadow:0 2px 12px rgba(0,0,0,.2);border:1px solid #1e2d4a}
.card h2{font-size:.75rem;color:#888;font-weight:600;text-transform:uppercase;
  letter-spacing:.5px;margin-bottom:8px}
.conn-row{display:flex;align-items:center;gap:8px}
.conn-dot{width:10px;height:10px;border-radius:50%;flex-shrink:0}
.conn-dot.on{background:#43a047;box-shadow:0 0 8px rgba(67,160,71,.4)}
.conn-dot.off{background:#e53935;box-shadow:0 0 8px rgba(229,57,53,.3)}
.conn-label{flex:1;font-weight:600;font-size:.9rem;color:#e0e0e0}
.conn-ip{color:#666;font-size:.75rem}
.auto-section{text-align:center;margin:6px 0}
.auto-btn{width:100%;padding:18px;border:none;border-radius:14px;
  font-size:1.2rem;font-weight:800;cursor:pointer;color:#fff;
  transition:transform .08s,box-shadow .2s;touch-action:manipulation}
.auto-btn:active{transform:scale(.96)}
.auto-btn.start{background:linear-gradient(135deg,#43a047,#2e7d32);
  box-shadow:0 4px 16px rgba(46,125,50,.35)}
.auto-btn.stop{background:linear-gradient(135deg,#e53935,#c62828);
  box-shadow:0 4px 16px rgba(198,40,40,.35)}
.auto-btn:disabled{opacity:.4;cursor:not-allowed}
.state-badge{display:inline-block;padding:6px 16px;border-radius:20px;
  font-size:.85rem;font-weight:700;margin-top:8px;letter-spacing:.5px;color:#fff}
.video-wrap{position:relative;text-align:center}
.video-wrap img{width:100%;max-width:640px;border-radius:10px;border:2px solid #333}
.live-badge{position:absolute;top:8px;right:8px;padding:4px 10px;border-radius:6px;
  font-weight:700;font-size:.8rem;color:#fff}
.models{display:flex;gap:6px;justify-content:center}
.mbtn{flex:1;padding:10px 0;border:2px solid #333;border-radius:10px;
  background:#0d1117;color:#888;font-weight:700;font-size:.85rem;
  cursor:pointer;text-align:center;transition:all .15s}
.mbtn.active{border-color:#43a047;color:#fff;background:#43a047;
  box-shadow:0 0 12px rgba(67,160,71,.3)}
.mbtn:active{transform:scale(.94)}
#mStatus{font-size:.72rem;color:#888;text-align:center;margin-top:4px}
.stats-grid{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px}
.sg{text-align:center;background:#0d1117;border:1px solid #1e2d4a;border-radius:8px;padding:8px 4px}
.sg .v{font-size:1.2rem;font-weight:800;color:#4fc3f7}
.sg .l{font-size:.6rem;color:#666;margin-top:2px;font-weight:600}
.us-grid{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px;margin-top:8px}
.us-item{text-align:center;background:#0d1117;border:1px solid #1e2d4a;border-radius:8px;padding:6px 4px}
.us-item .v{font-size:1rem;font-weight:700;font-family:monospace}
.us-item .l{font-size:.55rem;color:#666;font-weight:600}
.us-item.stop .v{color:#e53935}
.us-item.slow .v{color:#e65100}
.us-item.warn .v{color:#fbc02d}
.us-item.clear .v{color:#2e7d32}
.obstacle-status{margin-top:8px;padding:8px 12px;border-radius:8px;
  text-align:center;font-weight:700;font-size:.85rem;
  background:#0d1117;border:1px solid #1e2d4a}
.obstacle-status .lbl{font-size:.6rem;color:#666;font-weight:600;letter-spacing:.5px}
.obstacle-status .val{margin-top:2px}
.obstacle-status.clear .val{color:#2e7d32}
.obstacle-status.warn .val{color:#fbc02d}
.obstacle-status.slow .val{color:#e65100}
.obstacle-status.stop .val{color:#e53935}
.emergency{width:100%;padding:16px;border:none;border-radius:12px;
  font-size:1.1rem;font-weight:800;cursor:pointer;color:#fff;
  background:#d32f2f;box-shadow:0 3px 10px rgba(211,47,47,.3);
  touch-action:manipulation;transition:transform .08s}
.emergency:active{transform:scale(.96)}
.rec-row{display:flex;align-items:center;gap:8px;margin-top:6px}
.rec-btn{padding:8px 14px;border:none;border-radius:8px;font-size:.8rem;
  font-weight:700;cursor:pointer;color:#fff;transition:transform .08s}
.rec-btn:active{transform:scale(.94)}
#recInfo{font-size:.7rem;color:#888;flex:1}
.sessions{max-height:150px;overflow-y:auto;margin-top:6px}
.sess{display:flex;justify-content:space-between;align-items:center;
  background:#0d1117;border:1px solid #1e2d4a;border-radius:6px;
  padding:6px 10px;margin:3px 0;font-size:.75rem}
.sess .name{color:#4fc3f7;font-weight:600}
.dl{color:#2e7d32;text-decoration:none;font-weight:700;font-size:.75rem}
.footer{text-align:center;margin-top:12px;font-size:.65rem;color:#555}
</style></head><body>

<h1>PET Bottle Robot</h1>
<div class="subtitle">Autonomous Collection System</div>

<div class="card">
  <h2>Robot Connection (BLE)</h2>
  <div class="conn-row">
    <div class="conn-dot off" id="connDot"></div>
    <span class="conn-label" id="connName">Checking...</span>
    <span class="conn-ip" id="connDetail">BLE UART</span>
  </div>
  <div id="connStatus" style="font-size:.7rem;color:#888;text-align:center;margin-top:4px">Auto-connects on boot</div>
</div>

<div class="card">
  <h2>Autonomous Mode</h2>
  <div class="auto-section">
    <button class="auto-btn start" id="autoBtn" onclick="toggleAuto()">START AUTONOMOUS</button>
    <div><span class="state-badge" id="stateBadge" style="background:#90a4ae">WAITING</span></div>
  </div>
</div>

<div class="card">
  <h2>Live Camera</h2>
  <div class="video-wrap">
    <img id="vidFeed" src="/nav/video_feed" alt="Loading..."
      onerror="this.style.opacity=0.3;var self=this;setTimeout(function(){self.src='/nav/video_feed?t='+Date.now();self.style.opacity=1},3000)">
    <div class="live-badge" id="badge" style="background:#90a4ae">WAITING</div>
  </div>
</div>

<div class="card">
  <h2>Detection</h2>
  <div class="stats-grid" style="grid-template-columns:repeat(6,1fr)">
    <div class="sg"><div class="v" id="sBot" style="color:#2e7d32">0</div><div class="l">Bottles</div></div>
    <div class="sg"><div class="v" id="sFps">--</div><div class="l">FPS</div></div>
    <div class="sg"><div class="v" id="sInf" style="color:#e65100">--</div><div class="l">Infer</div></div>
    <div class="sg"><div class="v" id="sMod" style="color:#7b1fa2;font-size:.7rem">--</div><div class="l">Model</div></div>
    <div class="sg"><div class="v" id="sPick" style="color:#1565c0">0</div><div class="l">Pickups</div></div>
    <div class="sg"><div class="v" id="sAvoid" style="color:#c62828">0</div><div class="l">Avoids</div></div>
  </div>
  <div class="us-grid">
    <div class="us-item clear" id="usF"><div class="v">--</div><div class="l">FRONT</div></div>
    <div class="us-item clear" id="usL"><div class="v">--</div><div class="l">LEFT</div></div>
    <div class="us-item clear" id="usR"><div class="v">--</div><div class="l">RIGHT</div></div>
    <div class="us-item clear" id="usB"><div class="v">--</div><div class="l">BACK</div></div>
  </div>
  <div class="obstacle-status clear" id="obsStatus">
    <div class="lbl">CLOSEST OBSTACLE</div>
    <div class="val" id="obsVal">-- &middot; waiting for ESP32</div>
  </div>
</div>

<div class="card">
  <h2>YOLO Model</h2>
  <div class="models">
    <button class="mbtn" onclick="switchModel('yolov5')">YOLOv5</button>
    <button class="mbtn" onclick="switchModel('yolov7')">YOLOv7</button>
    <button class="mbtn" onclick="switchModel('yolov8')">YOLOv8</button>
  </div>
  <div id="mStatus"></div>
</div>

<div style="margin:12px 0">
  <button class="emergency" onclick="emergencyStop()">EMERGENCY STOP</button>
</div>

<div class="card">
  <h2>Data Recording</h2>
  <div class="rec-row">
    <button class="rec-btn" id="recBtn" style="background:#1976d2" onclick="toggleRec()">Record</button>
    <span id="recInfo">Not recording</span>
  </div>
  <div class="sessions" id="sessions"></div>
</div>

<div class="card">
  <h2>Pi &rarr; ESP32 Commands</h2>
  <div id="cmdLog" style="max-height:120px;overflow-y:auto;font-family:monospace;font-size:.7rem;
    background:#0d1117;border:1px solid #1e2d4a;border-radius:8px;padding:8px;color:#4fc3f7;
    white-space:pre-wrap"></div>
</div>

<div class="card">
  <h2>Navigation CSV</h2>
  <a href="/nav/data_log" download="navigation_log.csv" class="rec-btn"
    style="display:inline-block;background:#1976d2;text-decoration:none;padding:8px 14px;border-radius:8px;color:#fff;font-weight:700;font-size:.8rem">Download CSV</a>
</div>

<div class="footer">PET Bottle Robot &mdash; Autonomous Collection System</div>

<script>
var autoRunning=false;
var stateColors={WAITING:'#90a4ae',SCANNING:'#1976d2',ROAMING:'#2e7d32',
  VERIFYING:'#e65100',APPROACHING:'#1565c0',ALIGNING:'#f57f17',
  PICKING_UP:'#7b1fa2',AVOIDING:'#c62828',STOPPED:'#78909c',
  MISSION_COMPLETE:'#43a047'};

function poll(){
  fetch('/nav/stats').then(function(r){return r.json()}).then(function(d){
    document.getElementById('sBot').textContent=d.bottles||0;
    document.getElementById('sFps').textContent=d.fps||'--';
    document.getElementById('sInf').textContent=(d.inference_ms||'--')+'ms';
    document.getElementById('sMod').textContent=d.model||'--';
    var st=d.state||'WAITING';
    var sc=stateColors[st]||'#90a4ae';
    document.getElementById('badge').textContent=st;
    document.getElementById('badge').style.background=sc;
    var sb=document.getElementById('stateBadge');
    sb.textContent=st;sb.style.background=sc;
    var ble=d.bleConnected||false;
    var dot=document.getElementById('connDot');
    var cn=document.getElementById('connName');
    dot.className='conn-dot '+(ble?'on':'off');
    cn.textContent=ble?'ESP32 Connected (BLE)':'ESP32 Disconnected';
    var btn=document.getElementById('autoBtn');
    if(st==='WAITING'||st==='STOPPED'){
      autoRunning=false;btn.textContent='START AUTONOMOUS';btn.className='auto-btn start';
      btn.disabled=!ble;
      if(!ble)btn.textContent='WAITING FOR BLE...';
    }else{
      autoRunning=true;btn.textContent='STOP AUTONOMOUS';btn.className='auto-btn stop';
      btn.disabled=false;
    }
    document.getElementById('sPick').textContent=d.pickups||0;
    document.getElementById('sAvoid').textContent=d.avoidances||0;
    if(d.ultrasonic){
      setUS('usF',d.ultrasonic.s1);setUS('usR',d.ultrasonic.s2);
      setUS('usB',d.ultrasonic.s3);setUS('usL',d.ultrasonic.s4);
      updateObstacle(d.ultrasonic);
    }
  }).catch(function(){});
}
function pollCmdLog(){
  fetch('/nav/cmdlog').then(function(r){return r.json()}).then(function(d){
    var el=document.getElementById('cmdLog');
    if(d&&d.log){el.textContent=d.log.map(function(e){return e.time+' '+e.cmd}).join('\\n');
      el.scrollTop=el.scrollHeight;}
  }).catch(function(){});
}
setInterval(pollCmdLog,2000);pollCmdLog();
function usClass(v){
  // Match navigator thresholds: STOP=60, SLOW=100, TURN=150 cm
  if(v>=999)return 'clear';
  if(v<60)return 'stop';
  if(v<100)return 'slow';
  if(v<150)return 'warn';
  return 'clear';
}
function setUS(id,val){
  var el=document.getElementById(id);if(!el)return;
  var v=val||999;
  el.querySelector('.v').textContent=v>=999?'--':v+'cm';
  el.className='us-item '+usClass(v);
}
function updateObstacle(us){
  var vals=[us.s1,us.s2,us.s3,us.s4].map(function(x){return x||999});
  var dirs=['FRONT','RIGHT','BACK','LEFT'];
  var minIdx=0;
  for(var i=1;i<4;i++){if(vals[i]<vals[minIdx])minIdx=i;}
  var min=vals[minIdx];
  var dir=dirs[minIdx];
  var cls=usClass(min);
  var msg;
  if(min>=999)msg='-- &middot; no echo (path clear or out of range)';
  else if(cls==='stop')msg=min+'cm '+dir+' &middot; STOP / BACKING UP';
  else if(cls==='slow')msg=min+'cm '+dir+' &middot; SLOW CRAWL';
  else if(cls==='warn')msg=min+'cm '+dir+' &middot; STEERING AWAY';
  else msg=min+'cm '+dir+' &middot; CLEAR';
  var el=document.getElementById('obsStatus');
  el.className='obstacle-status '+cls;
  document.getElementById('obsVal').innerHTML=msg;
}
setInterval(poll,1000);poll();

function toggleAuto(){
  var btn=document.getElementById('autoBtn');btn.disabled=true;
  var sb=document.getElementById('stateBadge');
  fetch(autoRunning?'/nav/stop':'/nav/start').then(function(r){return r.json()}).then(function(d){
    btn.disabled=false;
    if(d.ok===false){
      sb.textContent=d.msg||'ERROR';sb.style.background='#c62828';
      setTimeout(poll,2000);
    }else{poll();}
  }).catch(function(){btn.disabled=false});
}

function emergencyStop(){
  fetch('/nav/stop').then(function(){poll()});
}

function loadModels(){
  fetch('/nav/models').then(function(r){return r.json()}).then(function(d){
    if(!d.ok)return;
    var nameMap={yolov5:'YOLOv5s-COCO',yolov7:'YOLOv7-COCO',yolov8:'PetBot-v8s'};
    document.querySelectorAll('.mbtn').forEach(function(b){
      b.classList.toggle('active',b.textContent.toLowerCase()===d.active);
    });
    var st=document.getElementById('mStatus');
    st.textContent='Active: '+(nameMap[d.active]||d.active);st.style.color='#2e7d32';
  }).catch(function(){});
}
loadModels();

function switchModel(key){
  document.getElementById('mStatus').textContent='Switching...';
  document.getElementById('mStatus').style.color='#e65100';
  fetch('/nav/switch_model?model='+key).then(function(r){return r.json()}).then(function(d){
    if(d.ok){document.getElementById('mStatus').textContent='Active: '+d.name;
      document.getElementById('mStatus').style.color='#2e7d32';loadModels();
    }else{document.getElementById('mStatus').textContent=d.msg||d.error||'Failed';
      document.getElementById('mStatus').style.color='#c62828';}
  }).catch(function(){});
}


function toggleRec(){fetch('/toggle_recording').then(function(){pollRec();loadSessions()})}
function pollRec(){
  fetch('/rec_status').then(function(r){return r.json()}).then(function(d){
    var btn=document.getElementById('recBtn'),info=document.getElementById('recInfo');
    if(d.active){btn.textContent='Stop';btn.style.background='#c62828';
      info.textContent=d.model+' | '+d.rows+' samples | '+d.duration+'s';info.style.color='#c62828';
    }else{btn.textContent='Record';btn.style.background='#1976d2';
      info.textContent=d.rows>0?'Last: '+d.rows+' samples':'Not recording';info.style.color='#78909c';}
  }).catch(function(){});
}
setInterval(pollRec,3000);pollRec();

function loadSessions(){
  fetch('/sessions').then(function(r){return r.json()}).then(function(d){
    var el=document.getElementById('sessions');
    if(!d.sessions||!d.sessions.length){el.innerHTML='';return}
    el.innerHTML=d.sessions.slice(0,5).map(function(s){
      return '<div class="sess"><div><span class="name">'+s.name+'</span></div>'+
      '<a class="dl" href="/download/'+s.file+'">CSV</a></div>';}).join('');
  });
}
loadSessions();
</script>
</body></html>"""

@app.route("/")
def index():
    return PAGE_HTML

@app.route("/toggle_recording")
def toggle_recording():
    with recording["lock"]:
        if recording["active"]:
            if recording["file"]:
                recording["file"].close()
            recording["active"] = False
            return jsonify({"status": "stopped", "rows": recording["rows"]})
        else:
            model = "unknown"
            try:
                d = nav_get("/stats", timeout=3)
                model = d.get("model", "unknown")
            except Exception:
                model = "unknown"
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = f"{model}_{ts}.csv"
            fpath = os.path.join(DATA_DIR, fname)
            f = open(fpath, "w", newline="")
            writer = csv.writer(f)
            writer.writerow(["timestamp", "model", "bottles", "persons", "fps", "inference_ms", "state"])
            recording["active"] = True
            recording["session"] = fname
            recording["file"] = f
            recording["writer"] = writer
            recording["model"] = model
            recording["start"] = time.time()
            recording["rows"] = 0
            return jsonify({"status": "recording", "file": fname, "model": model})

@app.route("/rec_status")
def rec_status():
    with recording["lock"]:
        if recording["active"]:
            dur = int(time.time() - recording["start"])
            return jsonify({"active": True, "model": recording["model"],
                           "rows": recording["rows"], "duration": dur})
        return jsonify({"active": False, "rows": recording["rows"]})

@app.route("/sessions")
def sessions():
    files = sorted([f for f in os.listdir(DATA_DIR) if f.endswith(".csv")], reverse=True)
    result = []
    for f in files[:20]:
        fpath = os.path.join(DATA_DIR, f)
        try:
            with open(fpath) as csvf:
                rows = sum(1 for _ in csvf) - 1
            result.append({"file": f, "name": f.replace(".csv", ""), "rows": rows})
        except Exception:
            result.append({"file": f, "name": f, "rows": 0})
    return jsonify({"sessions": result})

@app.route("/download/<filename>")
def download(filename):
    fpath = os.path.join(DATA_DIR, filename)
    if os.path.exists(fpath):
        return send_file(fpath, as_attachment=True)
    return jsonify({"error": "not found"}), 404

@app.route("/status")
def status():
    return jsonify({"ble": get_ble_status()})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
