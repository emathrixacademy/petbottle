#!/usr/bin/env python3
"""
Pi Admin — WiFi connect, model switch, live video, bottle count, data recording.
Port 8080. All navigator calls proxied through here (no CORS issues).
"""

import subprocess, socket, os, csv, time, threading, json
import urllib.request
from datetime import datetime
from flask import Flask, jsonify, request, send_file, Response

app = Flask(__name__)
NAV = "http://127.0.0.1:5000"
DATA_DIR = "/home/set-admin/testing/test_data"
os.makedirs(DATA_DIR, exist_ok=True)

# Recording state
recording = {"active": False, "session": None, "file": None, "writer": None,
             "model": "", "start": 0, "rows": 0, "lock": threading.Lock()}

def run(cmd, timeout=10):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception as e:
        return str(e)

def get_wifi():
    return run("iwgetid -r") or "Not connected"

def get_ip():
    return run("ip -4 addr show wlan0 | grep -oP 'inet \\K[\\d.]+'") or "--"

def nav_get(path, timeout=5):
    """Fetch from navigator Flask (localhost:5000)."""
    try:
        req = urllib.request.urlopen(f"{NAV}{path}", timeout=timeout)
        return json.loads(req.read())
    except Exception as e:
        return {"ok": False, "error": str(e)}

# Background recorder
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
        except:
            pass
        time.sleep(1)

threading.Thread(target=recorder_loop, daemon=True).start()

# ── Proxy routes to navigator (avoids CORS) ──────────────────

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

@app.route("/nav/video_feed")
def nav_video():
    """Proxy the MJPEG stream from navigator."""
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

@app.route("/")
def index():
    wifi = get_wifi()
    ip = get_ip()
    connected = "PetBottle_Robot" in wifi
    return f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pi Admin</title>
<style>
*{{box-sizing:border-box;margin:0;padding:0}}
body{{background:#1a1a2e;color:#e0e0e0;font-family:system-ui;padding:12px;
  -webkit-user-select:none;user-select:none}}
h1{{color:#4fc3f7;font-size:1.3rem;text-align:center;margin-bottom:12px}}
.card{{background:#16213e;border-radius:12px;padding:14px;margin:10px 0}}
.card h2{{font-size:.8rem;color:#888;margin-bottom:8px}}
.wifi-row{{display:flex;align-items:center;gap:8px}}
.wifi-row .ssid{{flex:1;font-weight:bold;font-size:.9rem}}
.wbtn{{padding:10px 18px;border:none;border-radius:8px;font-size:.85rem;
  font-weight:bold;cursor:pointer;color:#fff;touch-action:manipulation}}
.wbtn:active{{transform:scale(.96)}}
.video-wrap{{position:relative;text-align:center}}
.video-wrap img{{width:100%;max-width:640px;border-radius:10px;border:2px solid #333}}
.badge{{position:absolute;top:8px;right:8px;padding:4px 10px;border-radius:6px;
  font-weight:700;font-size:.8rem;color:#111}}
.models{{display:flex;gap:8px;justify-content:center}}
.mbtn{{flex:1;padding:12px 0;border:2px solid #333;border-radius:10px;background:#0d1117;
  color:#888;font-weight:700;font-size:.9rem;cursor:pointer;text-align:center}}
.mbtn.active{{border-color:#4fc3f7;color:#4fc3f7;background:#16213e}}
.mbtn:active{{transform:scale(.94)}}
#mStatus{{font-size:.75rem;color:#888;text-align:center;margin-top:6px}}
.stats-grid{{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px;margin:8px 0}}
.sg{{text-align:center;background:#0d1117;border-radius:8px;padding:8px 4px}}
.sg .v{{font-size:1.3rem;font-weight:700;color:#4fc3f7}}
.sg .l{{font-size:.65rem;color:#666;margin-top:2px}}
.rec-row{{display:flex;align-items:center;gap:8px;margin-top:8px}}
.rec-btn{{padding:10px 16px;border:none;border-radius:8px;font-size:.85rem;
  font-weight:bold;cursor:pointer;color:#fff}}
.rec-btn:active{{transform:scale(.96)}}
#recInfo{{font-size:.75rem;color:#888;flex:1}}
.sessions{{max-height:200px;overflow-y:auto;margin-top:8px}}
.sess{{display:flex;justify-content:space-between;align-items:center;
  background:#0d1117;border-radius:6px;padding:8px 10px;margin:4px 0;font-size:.8rem}}
.sess .name{{color:#4fc3f7}}.sess .meta{{color:#888;font-size:.7rem}}
.dl{{color:#66bb6a;text-decoration:none;font-weight:bold;font-size:.8rem}}
</style></head><body>
<h1>Pi Admin</h1>

<!-- WIFI -->
<div class="card">
  <h2>ESP32 WiFi</h2>
  <div class="wifi-row">
    <span class="ssid" id="wifiName" style="color:{'#66bb6a' if connected else '#ef5350'}">{wifi}</span>
    <span style="color:#555;font-size:.75rem">{ip}</span>
    <button class="wbtn" id="wifiBtn"
      style="background:{'#c62828' if connected else '#1565c0'}"
      onclick="toggleWifi()">{'Disconnect' if connected else 'Connect'}</button>
  </div>
  <div id="wifiStatus" style="font-size:.7rem;color:#888;text-align:center;margin-top:4px"></div>
</div>

<!-- LIVE VIDEO -->
<div class="card">
  <h2>Live Camera</h2>
  <div class="video-wrap">
    <img id="vidFeed" src="/nav/video_feed" alt="Loading..."
      onerror="this.style.opacity=0.3;setTimeout(()=>{{this.src='/nav/video_feed?t='+Date.now();this.style.opacity=1}},3000)">
    <div class="badge" id="badge" style="background:#ff9800">--</div>
  </div>
</div>

<!-- MODELS -->
<div class="card">
  <h2>YOLO Model</h2>
  <div class="models" id="modelBtns">
    <button class="mbtn" onclick="switchModel('yolov5')">YOLOv5</button>
    <button class="mbtn" onclick="switchModel('yolov7')">YOLOv7</button>
    <button class="mbtn" onclick="switchModel('yolov8')">YOLOv8</button>
  </div>
  <div id="mStatus"></div>
</div>

<!-- LIVE STATS -->
<div class="card">
  <h2>Detection Stats</h2>
  <div class="stats-grid">
    <div class="sg"><div class="v" id="sBot" style="color:#66bb6a">0</div><div class="l">Bottles</div></div>
    <div class="sg"><div class="v" id="sFps">--</div><div class="l">FPS</div></div>
    <div class="sg"><div class="v" id="sInf" style="color:#ffd54f">--</div><div class="l">Infer ms</div></div>
    <div class="sg"><div class="v" id="sMod" style="color:#ce93d8">--</div><div class="l">Model</div></div>
  </div>
</div>

<!-- RECORDING -->
<div class="card">
  <h2>Data Recording</h2>
  <div class="rec-row">
    <button class="rec-btn" id="recBtn" style="background:#2e7d32" onclick="toggleRec()">Start Recording</button>
    <span id="recInfo">Not recording</span>
  </div>
  <h2 style="margin-top:12px">Saved Sessions</h2>
  <div class="sessions" id="sessions">Loading...</div>
</div>

<script>
const stateColors={{WAITING:'#ff9800',SCANNING:'#ffeb3b',ROAMING:'#66bb6a',
  VERIFYING:'#ffcc80',APPROACHING:'#4fc3f7',ALIGNING:'#ffd54f',
  PICKING_UP:'#ab47bc',AVOIDING:'#ef5350',STOPPED:'#666'}};

function poll(){{
  fetch('/nav/stats').then(r=>r.json()).then(d=>{{
    document.getElementById('sBot').textContent=d.bottles||0;
    document.getElementById('sFps').textContent=d.fps||'--';
    document.getElementById('sInf').textContent=(d.inference_ms||'--')+'ms';
    document.getElementById('sMod').textContent=d.model||'--';
    let b=document.getElementById('badge');
    b.textContent=d.state||'--';
    b.style.background=stateColors[d.state]||'#666';
  }}).catch(()=>{{}});
}}
setInterval(poll,1200); poll();

function pollRec(){{
  fetch('/rec_status').then(r=>r.json()).then(d=>{{
    let btn=document.getElementById('recBtn');
    let info=document.getElementById('recInfo');
    if(d.active){{
      btn.textContent='Stop Recording';btn.style.background='#c62828';
      info.textContent=d.model+' | '+d.rows+' samples | '+d.duration+'s';
      info.style.color='#ef5350';
    }}else{{
      btn.textContent='Start Recording';btn.style.background='#2e7d32';
      if(d.rows>0)info.textContent='Last: '+d.rows+' samples saved';
      info.style.color='#888';
    }}
  }}).catch(()=>{{}});
}}
setInterval(pollRec,2000); pollRec();

function loadModels(){{
  fetch('/nav/models').then(r=>r.json()).then(d=>{{
    if(!d.ok)return;
    document.querySelectorAll('.mbtn').forEach(b=>{{
      let key=b.textContent.toLowerCase().replace('yolo','yolov');
      b.classList.toggle('active',key===d.active);
    }});
  }}).catch(()=>{{}});
}}
loadModels();

function switchModel(key){{
  document.getElementById('mStatus').textContent='Switching to '+key+'...';
  document.getElementById('mStatus').style.color='#ffd54f';
  fetch('/nav/switch_model?model='+key).then(r=>r.json()).then(d=>{{
    if(d.ok){{
      document.getElementById('mStatus').textContent='Active: '+d.name;
      document.getElementById('mStatus').style.color='#66bb6a';
      loadModels();
    }}else{{
      document.getElementById('mStatus').textContent=d.msg||d.error||'Failed';
      document.getElementById('mStatus').style.color='#ef5350';
    }}
  }}).catch(e=>{{
    document.getElementById('mStatus').textContent='Error: '+e;
    document.getElementById('mStatus').style.color='#ef5350';
  }});
}}

function toggleWifi(){{
  let btn=document.getElementById('wifiBtn');
  let st=document.getElementById('wifiStatus');
  let name=document.getElementById('wifiName');
  if(name.textContent.includes('PetBottle_Robot')){{
    st.textContent='Disconnecting...';
    fetch('/disconnect_esp32').then(r=>r.json()).then(d=>{{
      st.textContent=d.status;name.textContent=d.wifi||'Not connected';
      name.style.color='#ef5350';btn.textContent='Connect';btn.style.background='#1565c0';
    }}).catch(e=>{{st.textContent='Error'}});
  }}else{{
    st.textContent='Connecting...';
    fetch('/connect_esp32').then(r=>r.json()).then(d=>{{
      st.textContent=d.status;name.textContent=d.wifi||'--';
      if(d.wifi&&d.wifi.includes('PetBottle')){{
        name.style.color='#66bb6a';btn.textContent='Disconnect';btn.style.background='#c62828';
      }}
    }}).catch(e=>{{st.textContent='Error'}});
  }}
}}

function toggleRec(){{
  fetch('/toggle_recording').then(r=>r.json()).then(d=>{{
    pollRec(); loadSessions();
  }});
}}

function loadSessions(){{
  fetch('/sessions').then(r=>r.json()).then(d=>{{
    let el=document.getElementById('sessions');
    if(!d.sessions||d.sessions.length===0){{el.innerHTML='<div style="color:#555;font-size:.8rem">No sessions yet</div>';return;}}
    el.innerHTML=d.sessions.map(s=>
      '<div class="sess"><div><span class="name">'+s.name+'</span><br>'+
      '<span class="meta">'+s.model+' | '+s.rows+' samples | '+s.duration+'</span></div>'+
      '<a class="dl" href="/download/'+s.file+'">CSV</a></div>'
    ).join('');
  }});
}}
loadSessions();
</script>
</body></html>"""

# ── WiFi routes ──────────────────────────────────────────────

@app.route("/connect_esp32")
def connect_esp32():
    out = run("sudo nmcli con up ESP32-Robot 2>&1", timeout=15)
    return jsonify({"status": out, "wifi": get_wifi(), "ip": get_ip()})

@app.route("/disconnect_esp32")
def disconnect_esp32():
    out = run("sudo nmcli con down ESP32-Robot 2>&1", timeout=10)
    return jsonify({"status": out, "wifi": get_wifi(), "ip": get_ip()})

# ── Recording routes ─────────────────────────────────────────

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
            except:
                pass
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
            parts = f.replace(".csv", "").split("_")
            model = parts[0] if parts else "?"
            with open(fpath) as csvf:
                reader = csv.reader(csvf)
                next(reader)
                timestamps = [r[0] for r in reader if r]
            dur = f"{len(timestamps)}s" if len(timestamps) >= 2 else f"{rows} rows"
            result.append({"file": f, "name": f.replace(".csv", ""), "model": model,
                          "rows": rows, "duration": dur})
        except:
            result.append({"file": f, "name": f, "model": "?", "rows": 0, "duration": "?"})
    return jsonify({"sessions": result})

@app.route("/download/<filename>")
def download(filename):
    fpath = os.path.join(DATA_DIR, filename)
    if os.path.exists(fpath):
        return send_file(fpath, as_attachment=True)
    return jsonify({"error": "not found"}), 404

@app.route("/status")
def status():
    return jsonify({"wifi": get_wifi(), "ip": get_ip()})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
