#!/usr/bin/env python3
"""
Pi Admin — Autonomous Robot Control Panel.
Port 8080. All navigator calls proxied through here (no CORS issues).
"""

import os, csv, time, threading, json, subprocess, shutil
import urllib.request, urllib.parse
from datetime import datetime
from flask import Flask, jsonify, request, send_file, Response
from werkzeug.utils import secure_filename

app = Flask(__name__)
NAV = "http://127.0.0.1:5000"
DATA_DIR = "/home/set-admin/testing/test_data"
os.makedirs(DATA_DIR, exist_ok=True)

ESP32_IP = None
ESP32_BASE = None
_esp32_fail_count = 0
_esp32_discovering = False
_esp32_discover_lock = threading.Lock()

def bg_find_esp32():
    global _esp32_discovering
    with _esp32_discover_lock:
        if _esp32_discovering:
            return
        _esp32_discovering = True
    try:
        find_esp32()
    finally:
        with _esp32_discover_lock:
            _esp32_discovering = False

def find_esp32():
    global ESP32_IP, ESP32_BASE
    if ESP32_IP:
        # Verify it's still reachable
        try:
            urllib.request.urlopen(f"http://{ESP32_IP}/sensor", timeout=3)
            return ESP32_IP
        except Exception:
            ESP32_IP = None
            ESP32_BASE = None
    # Ask navigator for the ESP32 IP (fastest path)
    try:
        d = nav_get("/stats", timeout=3)
        ip = d.get("esp32_ip")
        if ip:
            ESP32_IP = ip
            ESP32_BASE = f"http://{ip}"
            return ip
    except Exception:
        pass
    # Parallel network scan as fallback
    import socket, concurrent.futures
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        my_ip = s.getsockname()[0]
        s.close()
    except Exception:
        return None
    prefix = my_ip.rsplit(".", 1)[0]
    def probe(ip):
        try:
            req = urllib.request.urlopen(f"http://{ip}/sensor", timeout=0.5)
            if b"ultrasonic" in req.read():
                return ip
        except Exception:
            pass
        return None
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as pool:
        futures = {pool.submit(probe, f"{prefix}.{i}"): i for i in range(1, 255)}
        for future in concurrent.futures.as_completed(futures, timeout=15):
            result = future.result()
            if result:
                ESP32_IP = result
                ESP32_BASE = f"http://{result}"
                # Tell navigator about the new IP so it reconnects immediately
                try:
                    nav_get(f"/esp32_reconnect?ip={result}", timeout=2)
                except Exception:
                    pass
                return result
    return None

def esp32_cmd(command):
    if not ESP32_BASE:
        find_esp32()
    if ESP32_BASE:
        try:
            url = f"{ESP32_BASE}/cmd?c={urllib.parse.quote(command)}"
            req = urllib.request.urlopen(url, timeout=2)
            resp = req.read().decode("utf-8", errors="replace").strip()
            return {"ok": True, "cmd": command, "resp": resp[:80]}
        except Exception:
            pass
    # Fallback: route through navigator (which has its own ESP32 connection)
    try:
        result = nav_get(f"/esp32_cmd?c={urllib.parse.quote(command)}", timeout=3)
        if result.get("ok"):
            return {"ok": True, "cmd": command, "resp": result.get("resp", "OK"), "via": "navigator"}
    except Exception:
        pass
    return {"ok": False, "cmd": command, "error": "ESP32 not reachable (direct or via navigator)"}

def esp32_sensor():
    if not ESP32_BASE:
        find_esp32()
    if ESP32_BASE:
        try:
            req = urllib.request.urlopen(f"{ESP32_BASE}/sensor", timeout=2)
            return json.loads(req.read())
        except Exception:
            pass
    # Fallback: get sensor data from navigator's cached copy
    try:
        d = nav_get("/stats", timeout=3)
        if d.get("ultrasonic"):
            return {"ultrasonic": d["ultrasonic"], "ok": True, "via": "navigator"}
    except Exception:
        pass
    return {"ok": False, "error": "ESP32 not reachable"}

recording = {"active": False, "session": None, "file": None, "writer": None,
             "model": "", "start": 0, "rows": 0, "lock": threading.Lock()}

def get_ble_status():
    try:
        d = nav_get("/stats", timeout=3)
        return d.get("espConnected", False)
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

# ── Manual control routes ────────────────────────────────────

@app.route("/manual/cmd")
def manual_cmd():
    c = request.args.get("c", "")
    if not c:
        return jsonify({"ok": False, "error": "no command"})
    # Tell navigator to pause its periodic stops before sending command
    nav_get("/manual_mode", timeout=1)
    return jsonify(esp32_cmd(c))

@app.route("/manual/sensor")
def manual_sensor():
    return jsonify(esp32_sensor())

@app.route("/manual/discover")
def manual_discover():
    global ESP32_IP, ESP32_BASE
    ESP32_IP = None
    ESP32_BASE = None
    ip = find_esp32()
    if ip:
        return jsonify({"ok": True, "ip": ip})
    return jsonify({"ok": False, "error": "ESP32 not found on network"})

@app.route("/set_esp32_ip")
def set_esp32_ip():
    global ESP32_IP, ESP32_BASE
    ip = request.args.get("ip", "").strip()
    if not ip:
        return jsonify({"ok": False, "error": "no ip"})
    # Quick reachability check
    try:
        urllib.request.urlopen(f"http://{ip}/sensor", timeout=3)
    except Exception as e:
        return jsonify({"ok": False, "error": f"ESP32 not reachable at {ip}: {e}"})
    ESP32_IP = ip
    ESP32_BASE = f"http://{ip}"
    return jsonify({"ok": True, "ip": ip})

@app.route("/esp32_status")
def esp32_status():
    global ESP32_IP, ESP32_BASE, _esp32_fail_count
    ip = ESP32_IP
    connected = False
    if ip:
        try:
            urllib.request.urlopen(f"http://{ip}/sensor", timeout=3)
            connected = True
            _esp32_fail_count = 0
        except Exception:
            _esp32_fail_count += 1
            if _esp32_fail_count >= 3:  # require 3 consecutive failures before disconnecting
                ESP32_IP = None
                ESP32_BASE = None
                _esp32_fail_count = 0
                ip = None
    if not connected and not ESP32_IP:
        threading.Thread(target=bg_find_esp32, daemon=True).start()
    return jsonify({"connected": connected, "ip": ip or ""})

@app.route("/emergency_stop")
def emergency_stop():
    results = {}
    # Direct ESP32 halt — always attempted regardless of navigator state
    results["esp32"] = esp32_cmd("PISTOP")
    # Also tell navigator to stop autonomous mode
    try:
        results["navigator"] = nav_get("/stop", timeout=2)
    except Exception as e:
        results["navigator"] = {"ok": False, "error": str(e)}
    return jsonify({"ok": True, "results": results})

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
.tabs{display:flex;gap:4px;margin-bottom:10px}
.tab{flex:1;padding:10px 0;border:2px solid #333;border-radius:10px;
  background:#0d1117;color:#888;font-weight:700;font-size:.85rem;
  cursor:pointer;text-align:center;transition:all .15s}
.tab.active{border-color:#4fc3f7;color:#fff;background:#1565c0}
.tab:active{transform:scale(.96)}
.panel{display:none}
.panel.active{display:block}
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

/* Manual controls */
.ctrl-group{margin:8px 0}
.ctrl-group h3{font-size:.7rem;color:#4fc3f7;font-weight:700;text-transform:uppercase;
  letter-spacing:.5px;margin-bottom:6px}
.dpad{display:grid;grid-template-columns:1fr 1fr 1fr;grid-template-rows:1fr 1fr 1fr;
  gap:4px;max-width:220px;margin:0 auto}
.dpad .cbtn{width:100%;aspect-ratio:1;border:none;border-radius:10px;
  font-size:.7rem;font-weight:800;cursor:pointer;color:#fff;
  transition:transform .06s;touch-action:manipulation}
.dpad .cbtn:active{transform:scale(.92)}
.dpad .empty{background:transparent}
.cbtn-fwd{background:linear-gradient(135deg,#1976d2,#1565c0)}
.cbtn-bwd{background:linear-gradient(135deg,#1976d2,#1565c0)}
.cbtn-left{background:linear-gradient(135deg,#e65100,#bf360c)}
.cbtn-right{background:linear-gradient(135deg,#e65100,#bf360c)}
.cbtn-stop{background:linear-gradient(135deg,#e53935,#c62828);font-size:.65rem!important}
.motor-row{display:flex;gap:6px;justify-content:center;flex-wrap:wrap}
.motor-btn{flex:1;min-width:60px;padding:12px 6px;border:none;border-radius:10px;
  font-size:.75rem;font-weight:700;cursor:pointer;color:#fff;
  transition:transform .06s;touch-action:manipulation;text-align:center}
.motor-btn:active{transform:scale(.92)}
.motor-btn.up{background:linear-gradient(135deg,#43a047,#2e7d32)}
.motor-btn.down{background:linear-gradient(135deg,#e65100,#bf360c)}
.motor-btn.stp{background:linear-gradient(135deg,#78909c,#546e7a)}
.motor-btn.open{background:linear-gradient(135deg,#1976d2,#1565c0)}
.motor-btn.close{background:linear-gradient(135deg,#7b1fa2,#6a1b9a)}
.motor-btn.buzz{background:linear-gradient(135deg,#fbc02d,#f9a825);color:#333}
.motor-btn.pickup{background:linear-gradient(135deg,#43a047,#2e7d32);padding:14px 6px}
.motor-btn.abort{background:linear-gradient(135deg,#e53935,#c62828);padding:14px 6px}
.speed-row{display:flex;align-items:center;gap:8px;margin:6px 0;justify-content:center}
.speed-row label{font-size:.7rem;color:#888;font-weight:600}
.speed-row input[type=range]{flex:1;max-width:160px;accent-color:#4fc3f7}
.speed-row .speed-val{font-size:.85rem;font-weight:800;color:#4fc3f7;min-width:30px;text-align:center}
#manualLog{max-height:80px;overflow-y:auto;font-family:monospace;font-size:.65rem;
  background:#0d1117;border:1px solid #1e2d4a;border-radius:8px;padding:6px;color:#43a047;
  white-space:pre-wrap;margin-top:6px}
.sensor-live{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:6px}
.slv{background:#0d1117;border:1px solid #1e2d4a;border-radius:8px;padding:6px 8px}
.slv .sl{font-size:.6rem;color:#666;font-weight:600}
.slv .sv{font-size:.85rem;font-weight:700;color:#4fc3f7;font-family:monospace}
</style></head><body>

<h1>PET Bottle Robot</h1>
<div class="subtitle">Autonomous Collection System</div>

<div class="tabs">
  <div class="tab active" onclick="showTab('autonomous')">Autonomous</div>
  <div class="tab" onclick="showTab('manual')">Manual Control</div>
  <div class="tab" onclick="showTab('data')">Data</div>
</div>

<!-- ═══ AUTONOMOUS TAB ═══ -->
<div class="panel active" id="panel-autonomous">

<div class="card">
  <h2>Robot Connection (WiFi)</h2>
  <div class="conn-row">
    <div class="conn-dot off" id="connDot"></div>
    <span class="conn-label" id="connName">Checking...</span>
    <span class="conn-ip" id="connDetail">WiFi HTTP</span>
  </div>
  <div id="connStatus" style="font-size:.7rem;color:#888;text-align:center;margin-top:4px">Auto-connects on boot</div>
  <div style="display:flex;gap:6px;margin-top:8px;align-items:center">
    <input id="espIpInput" type="text" placeholder="ESP32 IP (e.g. 192.168.43.50)"
      style="flex:1;padding:7px 10px;border-radius:8px;border:1px solid #1e2d4a;
             background:#0d1117;color:#e0e0e0;font-size:.8rem;outline:none">
    <button onclick="manualConnect()"
      style="padding:7px 12px;border:none;border-radius:8px;background:#1976d2;
             color:#fff;font-weight:700;font-size:.8rem;cursor:pointer;white-space:nowrap">
      Connect
    </button>
    <button onclick="autoDiscover()"
      style="padding:7px 12px;border:none;border-radius:8px;background:#546e7a;
             color:#fff;font-weight:700;font-size:.8rem;cursor:pointer;white-space:nowrap">
      Scan
    </button>
  </div>
  <div id="espConnMsg" style="font-size:.7rem;margin-top:4px;text-align:center"></div>
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
    <div id="camOffline" style="display:none;position:absolute;top:50%;left:50%;
      transform:translate(-50%,-50%);background:rgba(0,0,0,.75);color:#fff;
      padding:14px 22px;border-radius:10px;font-weight:700;text-align:center;font-size:.9rem">
      Navigator Offline<br>
      <span style="font-size:.72rem;color:#aaa;font-weight:400">Start navigator.py on the Pi</span>
    </div>
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
    <div class="val" id="obsVal">-- &middot; no data yet</div>
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

</div><!-- end autonomous panel -->

<!-- ═══ MANUAL CONTROL TAB ═══ -->
<div class="panel" id="panel-manual">

<div class="card">
  <h2>Wheels</h2>
  <div class="speed-row">
    <label>Speed</label>
    <input type="range" id="wheelSpd" min="10" max="80" value="40" oninput="document.getElementById('wheelSpdVal').textContent=this.value">
    <span class="speed-val" id="wheelSpdVal">40</span>
  </div>
  <div class="dpad">
    <div class="cbtn empty"></div>
    <button class="cbtn cbtn-fwd" ontouchstart="mc('PIFW'+gs())" onmousedown="mc('PIFW'+gs())">FWD</button>
    <div class="cbtn empty"></div>
    <button class="cbtn cbtn-left" ontouchstart="mc('PITL'+gs())" onmousedown="mc('PITL'+gs())">LEFT</button>
    <button class="cbtn cbtn-stop" ontouchstart="mc('PIX')" onmousedown="mc('PIX')">STOP</button>
    <button class="cbtn cbtn-right" ontouchstart="mc('PITR'+gs())" onmousedown="mc('PITR'+gs())">RIGHT</button>
    <div class="cbtn empty"></div>
    <button class="cbtn cbtn-bwd" ontouchstart="mc('PIBW'+gs())" onmousedown="mc('PIBW'+gs())">BWD</button>
    <div class="cbtn empty"></div>
  </div>
</div>

<div class="card">
  <h2>Arm Lift</h2>
  <div class="motor-row">
    <button class="motor-btn up" ontouchstart="mc('PIAU')" onmousedown="mc('PIAU')">ARM UP</button>
    <button class="motor-btn stp" ontouchstart="mc('PIAS')" onmousedown="mc('PIAS')">STOP</button>
    <button class="motor-btn down" ontouchstart="mc('PIAD')" onmousedown="mc('PIAD')">ARM DOWN</button>
  </div>
</div>

<div class="card">
  <h2>Swing Platform</h2>
  <div class="motor-row">
    <button class="motor-btn open" ontouchstart="mc('PISWL')" onmousedown="mc('PISWL')">SWING L</button>
    <button class="motor-btn stp" ontouchstart="mc('PISWS')" onmousedown="mc('PISWS')">STOP</button>
    <button class="motor-btn open" ontouchstart="mc('PISWR')" onmousedown="mc('PISWR')">SWING R</button>
  </div>
</div>

<div class="card">
  <h2>Scoopers (Servos)</h2>
  <div class="motor-row">
    <button class="motor-btn open" ontouchstart="mc('PISO')" onmousedown="mc('PISO')">OPEN</button>
    <button class="motor-btn close" ontouchstart="mc('PISC')" onmousedown="mc('PISC')">CLOSE</button>
  </div>
</div>

<div class="card">
  <h2>Pickup Sequence</h2>
  <div class="motor-row">
    <button class="motor-btn pickup" ontouchstart="mc('P')" onmousedown="mc('P')">START PICKUP</button>
    <button class="motor-btn abort" ontouchstart="mc('PA')" onmousedown="mc('PA')">ABORT</button>
  </div>
</div>

<div class="card">
  <h2>Buzzer</h2>
  <div class="motor-row">
    <button class="motor-btn buzz" ontouchstart="mc('PIBZ')" onmousedown="mc('PIBZ')">BEEP</button>
    <button class="motor-btn buzz" ontouchstart="mc('PIBZL')" onmousedown="mc('PIBZL')">LONG BEEP</button>
  </div>
</div>

<div class="card">
  <h2>Sensors (Live)</h2>
  <div class="us-grid">
    <div class="us-item clear" id="musF"><div class="v">--</div><div class="l">FRONT</div></div>
    <div class="us-item clear" id="musL"><div class="v">--</div><div class="l">LEFT</div></div>
    <div class="us-item clear" id="musR"><div class="v">--</div><div class="l">RIGHT</div></div>
    <div class="us-item clear" id="musB"><div class="v">--</div><div class="l">BACK</div></div>
  </div>
  <div class="sensor-live">
    <div class="slv"><div class="sl">LIMIT SW1 (BOTTOM)</div><div class="sv" id="mLim1">--</div></div>
    <div class="slv"><div class="sl">LIMIT SW2 (TOP)</div><div class="sv" id="mLim2">--</div></div>
    <div class="slv"><div class="sl">IR PROX (BIN)</div><div class="sv" id="mIR">--</div></div>
    <div class="slv"><div class="sl">PICKUP STATE</div><div class="sv" id="mPU">--</div></div>
  </div>
</div>

<div id="manualLog"></div>

</div><!-- end manual panel -->

<!-- ═══ DATA TAB ═══ -->
<div class="panel" id="panel-data">

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

<div class="card">
  <h2>Software Update</h2>
  <a href="/upload" class="rec-btn"
    style="display:inline-block;background:#7b1fa2;text-decoration:none;padding:8px 14px;border-radius:8px;color:#fff;font-weight:700;font-size:.8rem">Pi OTA Upload</a>
</div>

</div><!-- end data panel -->

<div style="margin:12px 0">
  <button class="emergency" onclick="emergencyStop()">EMERGENCY STOP</button>
</div>

<div class="footer">PET Bottle Robot &mdash; Autonomous Collection System</div>

<script>
var autoRunning=false;
var _navOk=false;
var _espOk=false;
var stateColors={WAITING:'#90a4ae',SCANNING:'#1976d2',ROAMING:'#2e7d32',
  VERIFYING:'#e65100',APPROACHING:'#1565c0',ALIGNING:'#f57f17',
  PICKING_UP:'#7b1fa2',AVOIDING:'#c62828',STOPPED:'#78909c',
  MISSION_COMPLETE:'#43a047'};

function showTab(name){
  document.querySelectorAll('.tab').forEach(function(t,i){
    var tabs=['autonomous','manual','data'];
    t.classList.toggle('active',tabs[i]===name);
  });
  document.querySelectorAll('.panel').forEach(function(p){
    p.classList.toggle('active',p.id==='panel-'+name);
  });
}

function gs(){return document.getElementById('wheelSpd').value}

var manualLines=[];
function mlog(msg){
  manualLines.push(new Date().toLocaleTimeString()+' '+msg);
  if(manualLines.length>20)manualLines.shift();
  var el=document.getElementById('manualLog');
  if(el){el.textContent=manualLines.join('\n');el.scrollTop=el.scrollHeight;}
}

function mc(cmd){
  mlog('> '+cmd);
  fetch('/manual/cmd?c='+encodeURIComponent(cmd)).then(function(r){return r.json()}).then(function(d){
    if(d.ok)mlog('  '+d.resp);
    else mlog('  ERR: '+(d.error||'failed'));
  }).catch(function(e){mlog('  ERR: '+e)});
}

function pollManualSensors(){
  fetch('/manual/sensor').then(function(r){return r.json()}).then(function(d){
    if(d.ultrasonic){
      setUSm('musF',d.ultrasonic.s1);setUSm('musR',d.ultrasonic.s2);
      setUSm('musB',d.ultrasonic.s3);setUSm('musL',d.ultrasonic.s4);
    }
    if(d.limits!==undefined){
      document.getElementById('mLim1').textContent=d.limit1?'PRESSED':'open';
      document.getElementById('mLim1').style.color=d.limit1?'#e53935':'#43a047';
      document.getElementById('mLim2').textContent=d.limit2?'PRESSED':'open';
      document.getElementById('mLim2').style.color=d.limit2?'#e53935':'#43a047';
    }
    if(d.irProx!==undefined){
      document.getElementById('mIR').textContent=d.irProx?'FULL':'empty';
      document.getElementById('mIR').style.color=d.irProx?'#e53935':'#43a047';
    }
    if(d.pickup!==undefined){
      document.getElementById('mPU').textContent=d.pickup||'idle';
    }
  }).catch(function(){});
}

function setUSm(id,val){
  var el=document.getElementById(id);if(!el)return;
  var v=val||999;
  el.querySelector('.v').textContent=v>=999?'--':v+'cm';
  el.className='us-item '+usClass(v);
}

function updateAutoBtn(){
  var btn=document.getElementById('autoBtn');
  btn.disabled=false;
  btn.style.opacity='1';
  if(autoRunning){
    btn.className='auto-btn stop';btn.textContent='STOP AUTONOMOUS';
  }else{
    btn.className='auto-btn start';btn.textContent='START AUTONOMOUS';
  }
}

function setEspMsg(msg,ok){
  var el=document.getElementById('espConnMsg');
  if(el){el.textContent=msg;el.style.color=ok?'#43a047':'#e53935';}
}

function manualConnect(){
  var ip=document.getElementById('espIpInput').value.trim();
  if(!ip){setEspMsg('Enter an IP address first',false);return;}
  var btn=document.querySelector('button[onclick="manualConnect()"]');
  if(btn){btn.disabled=true;btn.textContent='...';}
  setEspMsg('Connecting to '+ip+'...','#e65100');
  fetch('/set_esp32_ip?ip='+encodeURIComponent(ip))
    .then(function(r){return r.json()})
    .then(function(d){
      if(btn){btn.disabled=false;btn.textContent='Connect';}
      if(d.ok){setEspMsg('Connected to '+d.ip,true);pollESP32();}
      else{setEspMsg(d.error||'Failed',false);}
    }).catch(function(e){
      if(btn){btn.disabled=false;btn.textContent='Connect';}
      setEspMsg('Error: '+e,false);
    });
}

function autoDiscover(){
  var btn=document.querySelector('button[onclick="autoDiscover()"]');
  if(btn){btn.disabled=true;btn.textContent='Scanning...';}
  setEspMsg('Scanning network...','#e65100');
  fetch('/manual/discover')
    .then(function(r){return r.json()})
    .then(function(d){
      if(btn){btn.disabled=false;btn.textContent='Scan';}
      if(d.ok){
        document.getElementById('espIpInput').value=d.ip;
        setEspMsg('Found at '+d.ip,true);pollESP32();
      }else{setEspMsg('Not found — enter IP manually',false);}
    }).catch(function(e){
      if(btn){btn.disabled=false;btn.textContent='Scan';}
      setEspMsg('Scan error: '+e,false);
    });
}

function pollESP32(){
  fetch('/esp32_status').then(function(r){return r.json()}).then(function(d){
    _espOk=d.connected||false;
    var dot=document.getElementById('connDot');
    var cn=document.getElementById('connName');
    dot.className='conn-dot '+(_espOk?'on':'off');
    cn.textContent=_espOk?'ESP32 Connected ('+d.ip+')':'ESP32 Disconnected';
    if(_espOk&&d.ip){document.getElementById('espIpInput').value=d.ip;}
    updateAutoBtn();
  }).catch(function(){_espOk=false;updateAutoBtn();});
}
setInterval(pollESP32,3000);pollESP32();

function setNavOffline(){
  _navOk=false;
  var co=document.getElementById('camOffline');if(co)co.style.display='block';
  document.getElementById('badge').textContent='OFFLINE';
  document.getElementById('badge').style.background='#546e7a';
  document.getElementById('stateBadge').textContent='NAVIGATOR OFFLINE';
  document.getElementById('stateBadge').style.background='#546e7a';
  document.getElementById('sFps').textContent='--';
  document.getElementById('sInf').textContent='--ms';
  document.getElementById('sMod').textContent='--';
  updateAutoBtn();
}

function poll(){
  fetch('/nav/stats').then(function(r){return r.json()}).then(function(d){
    if(d.ok===false){setNavOffline();return;}
    _navOk=true;
    var co=document.getElementById('camOffline');
    if(co){
      if(d.cameraOk===false){
        co.style.display='block';
        co.innerHTML='Camera Not Available<br><span style="font-size:.72rem;color:#aaa;font-weight:400">Check camera/ribbon cable on Pi</span>';
      }else{
        co.style.display='none';
      }
    }
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
    if(st==='WAITING'||st==='STOPPED'){
      autoRunning=false;
    }else{
      autoRunning=true;
    }
    document.getElementById('sPick').textContent=d.pickups||0;
    document.getElementById('sAvoid').textContent=d.avoidances||0;
    if(d.ultrasonic){
      setUS('usF',d.ultrasonic.s1);setUS('usR',d.ultrasonic.s2);
      setUS('usB',d.ultrasonic.s3);setUS('usL',d.ultrasonic.s4);
      updateObstacle(d.ultrasonic);
    }
    updateAutoBtn();
  }).catch(function(){setNavOffline();});
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
setInterval(pollManualSensors,1000);

function toggleAuto(){
  var btn=document.getElementById('autoBtn');
  var sb=document.getElementById('stateBadge');
  if(autoRunning){
    // Optimistic: flip to WAITING immediately — halt motors right away
    autoRunning=false;
    updateAutoBtn();
    fetch('/manual/cmd?c=PISTOP').catch(function(){});
    fetch('/manual/cmd?c=PISWS').catch(function(){});
    fetch('/nav/stop').then(function(){poll();}).catch(function(){poll();});
  }else{
    // Optimistic: show STARTING immediately, dim while server confirms
    btn.textContent='STARTING...';
    btn.className='auto-btn start';
    btn.style.opacity='0.65';
    fetch('/nav/start').then(function(r){return r.json()}).then(function(d){
      btn.style.opacity='1';
      if(d.ok===false){
        sb.textContent=d.msg||'ERROR';sb.style.background='#c62828';
        autoRunning=false;
        updateAutoBtn();
      }else{
        autoRunning=true;
        updateAutoBtn();
      }
      poll();
    }).catch(function(){btn.style.opacity='1';autoRunning=false;updateAutoBtn();});
  }
}

function emergencyStop(){
  // Flash the button red as instant feedback
  var eb=document.querySelector('.emergency');
  if(eb){eb.style.background='#b71c1c';eb.textContent='STOPPING...';}
  fetch('/emergency_stop').then(function(r){return r.json()}).then(function(d){
    if(eb){eb.style.background='#d32f2f';eb.textContent='EMERGENCY STOP';}
    autoRunning=false;updateAutoBtn();poll();
  }).catch(function(){
    // Fallback: try direct command even if the route failed
    fetch('/manual/cmd?c=PISTOP').catch(function(){});
    if(eb){eb.style.background='#d32f2f';eb.textContent='EMERGENCY STOP';}
  });
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
  // Instant visual selection before server responds
  document.querySelectorAll('.mbtn').forEach(function(b){
    b.classList.toggle('active',b.textContent.toLowerCase()===key);
  });
  document.getElementById('mStatus').textContent='Switching...';
  document.getElementById('mStatus').style.color='#e65100';
  fetch('/nav/switch_model?model='+key).then(function(r){return r.json()}).then(function(d){
    if(d.ok){
      document.getElementById('mStatus').textContent='Active: '+d.name;
      document.getElementById('mStatus').style.color='#2e7d32';
      loadModels();
    }else{
      document.getElementById('mStatus').textContent=d.msg||d.error||'Failed';
      document.getElementById('mStatus').style.color='#c62828';
      loadModels(); // revert to actual active model
    }
  }).catch(function(){loadModels();});
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

# ── Pi OTA File Upload ─────────────────────────────────────────
RASPI_DIR = os.path.dirname(os.path.abspath(__file__))
ALLOWED_FILES = {"navigator.py", "camera.py", "pi_admin.py", "server.py"}

@app.route("/upload", methods=["GET"])
def upload_page():
    return UPLOAD_HTML

@app.route("/upload", methods=["POST"])
def upload_file():
    if "file" not in request.files:
        return jsonify({"ok": False, "error": "No file selected"}), 400
    f = request.files["file"]
    fname = secure_filename(f.filename)
    if fname not in ALLOWED_FILES:
        return jsonify({"ok": False, "error": f"Not allowed. Valid files: {', '.join(sorted(ALLOWED_FILES))}"}), 400
    dest = os.path.join(RASPI_DIR, fname)
    backup = dest + ".bak"
    if os.path.exists(dest):
        shutil.copy2(dest, backup)
    f.save(dest)
    restart_msg = ""
    if fname == "navigator.py" or fname == "camera.py":
        try:
            subprocess.run(["sudo", "systemctl", "restart", "petbottle-navigator"],
                           timeout=10, check=True)
            restart_msg = "Navigator service restarted."
        except Exception as e:
            restart_msg = f"Upload OK but restart failed: {e}"
    elif fname == "pi_admin.py":
        restart_msg = "Admin uploaded. Restarting in 2 seconds..."
        threading.Timer(2.0, lambda: os.execv(
            "/usr/bin/python3", ["python3", os.path.abspath(__file__)]
        )).start()
    return jsonify({"ok": True, "file": fname, "message": restart_msg})

UPLOAD_HTML = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Pi OTA Update</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0f0f0f;color:#e0e0e0;font-family:'Segoe UI',system-ui,sans-serif;
  display:flex;flex-direction:column;align-items:center;justify-content:center;
  min-height:100vh;padding:24px;-webkit-tap-highlight-color:transparent}
.card{background:#1a1a2e;border-radius:20px;padding:40px 32px;max-width:480px;
  width:100%;text-align:center;box-shadow:0 8px 40px rgba(0,0,0,.5)}
h1{font-size:1.5rem;color:#4fc3f7;margin-bottom:4px}
.sub{color:#888;font-size:.85rem;margin-bottom:24px}
.allowed{background:#111;border-radius:10px;padding:12px;margin-bottom:20px;
  font-size:.8rem;color:#aaa;font-family:monospace}
label.pick{display:inline-block;background:#1976d2;color:#fff;font-size:1.1rem;
  font-weight:700;border-radius:14px;padding:16px 36px;cursor:pointer;margin:8px;
  box-shadow:0 2px 8px rgba(25,118,210,.25)}
label.pick:active{transform:scale(.96)}
input[type=file]{display:none}
button{font-size:1.1rem;font-weight:700;border:none;border-radius:14px;
  padding:16px 36px;cursor:pointer;background:#43a047;color:#fff;margin:8px;
  box-shadow:0 2px 8px rgba(67,160,71,.2)}
button:disabled{opacity:.4;cursor:not-allowed}
button:active{transform:scale(.96)}
.fname{margin:12px 0;font-size:.9rem;color:#4fc3f7;font-weight:600}
.bar{height:6px;background:#333;border-radius:3px;margin:16px 0;overflow:hidden}
.bar-fill{height:100%;background:#4fc3f7;width:0%;transition:width .3s}
#status{font-size:.95rem;margin-top:16px;font-weight:600}
.back{display:inline-block;margin-top:20px;color:#666;font-size:.85rem;text-decoration:none}
.back:hover{color:#aaa}
</style></head><body>
<div class="card">
<h1>Pi Software Update</h1>
<p class="sub">Upload Python files to update the robot brain</p>
<div class="allowed">Allowed: navigator.py, camera.py, pi_admin.py, server.py</div>
<label class="pick">Choose .py File<input type="file" id="fileIn" accept=".py" onchange="fileSelected(this)"></label>
<div class="fname" id="fname"></div>
<button id="uploadBtn" onclick="doUpload()" disabled>Upload &amp; Apply</button>
<div class="bar"><div class="bar-fill" id="bar"></div></div>
<div id="status"></div>
<a class="back" href="/">&larr; Back to Dashboard</a>
</div>
<script>
var selFile=null;
function fileSelected(inp){
  selFile=inp.files[0];
  document.getElementById('fname').textContent=selFile?selFile.name:'';
  document.getElementById('uploadBtn').disabled=!selFile;
  document.getElementById('status').textContent='';
  document.getElementById('bar').style.width='0%';
}
function doUpload(){
  if(!selFile)return;
  var btn=document.getElementById('uploadBtn');
  btn.disabled=true;btn.textContent='Uploading...';
  document.getElementById('status').textContent='';
  document.getElementById('status').style.color='#4fc3f7';
  document.getElementById('bar').style.width='30%';
  var fd=new FormData();
  fd.append('file',selFile);
  var xhr=new XMLHttpRequest();
  xhr.open('POST','/upload',true);
  xhr.upload.onprogress=function(e){
    if(e.lengthComputable){
      var pct=Math.round(e.loaded/e.total*90);
      document.getElementById('bar').style.width=pct+'%';
    }
  };
  xhr.onload=function(){
    document.getElementById('bar').style.width='100%';
    try{
      var d=JSON.parse(xhr.responseText);
      if(d.ok){
        document.getElementById('status').textContent='OK: '+d.file+' uploaded. '+(d.message||'');
        document.getElementById('status').style.color='#66bb6a';
        btn.textContent='Upload & Apply';
      }else{
        document.getElementById('status').textContent='Error: '+(d.error||'Unknown');
        document.getElementById('status').style.color='#ef5350';
        document.getElementById('bar').style.background='#e53935';
        btn.textContent='Upload & Apply';btn.disabled=false;
      }
    }catch(e){
      document.getElementById('status').textContent='Error: bad response';
      document.getElementById('status').style.color='#ef5350';
      btn.textContent='Upload & Apply';btn.disabled=false;
    }
  };
  xhr.onerror=function(){
    document.getElementById('status').textContent='Network error';
    document.getElementById('status').style.color='#ef5350';
    btn.textContent='Upload & Apply';btn.disabled=false;
  };
  xhr.send(fd);
}
</script>
</body></html>"""

if __name__ == "__main__":
    def _bg_discover():
        import time as _t
        _t.sleep(3)
        ip = find_esp32()
        if ip:
            print(f"  [pi_admin] ESP32 found at {ip}")
        else:
            print(f"  [pi_admin] ESP32 not found yet — will retry on first command")
    threading.Thread(target=_bg_discover, daemon=True).start()
    app.run(host="0.0.0.0", port=8080, threaded=True)
