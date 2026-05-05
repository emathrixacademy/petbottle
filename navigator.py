# #!/usr/bin/env python3
# """
# PET Bottle Robot — Autonomous Navigator
# Runs on Raspberry Pi. Fuses camera (YOLO) + ultrasonic sensors from ESP32
# for real-time obstacle avoidance and bottle pickup.

# The robot continuously:
#   1. Roams open space looking for PET bottles
#   2. Avoids obstacles (ultrasonics + camera person/object detection)
#   3. Approaches detected bottles
#   4. Triggers the pickup sequence on the ESP32
#   5. Resumes roaming

# Transport: Pi <-> ESP32 is WiFi HTTP. Both devices connect to the same
# mobile hotspot. ESP32 has static IP 192.168.43.100.

# Usage:
#   python3 navigator.py                    # default (YOLOv8)
#   python3 navigator.py --model yolov8     # default model
#   python3 navigator.py --esp32-ip 192.168.43.100
#   python3 navigator.py --no-show          # headless
# """

# import cv2
# import numpy as np
# import argparse

# import time
# import threading
# import json
# from pathlib import Path
# from enum import Enum, auto
# from flask import Flask, Response, jsonify, request

# import urllib.request, urllib.error, urllib.parse

# try:
#     from vision_ai import verify_target, analyze_scene, assess_obstacle
#     AI_AVAILABLE = True
# except ImportError:
#     AI_AVAILABLE = False

# # Import detection system from camera.py
# from camera import (
#     ModelManager, MODEL_CONFIGS, MODEL_RANGE,
#     DetectionTracker, draw_detections, preprocess,
#     postprocess_nms, postprocess_raw, postprocess_petbottle,
#     filter_held_bottles, filter_pet_shape, get_orientation, overlay_info,
#     CONF_THRESHOLD, COCO_BOTTLE_ID, COCO_PERSON_ID,
# )
# from hailo_platform import (
#     HEF, VDevice, HailoStreamInterface,
#     InferVStreams, ConfigureParams,
#     InputVStreamParams, OutputVStreamParams, FormatType
# )


# # ══════════════════════════════════════════════════════════════
# # Configuration
# # ══════════════════════════════════════════════════════════════

# ESP32_IP = None  # auto-discovered at startup
# ESP32_BASE_URL = None


# def discover_esp32(timeout=30):
#     """Scan the local network for the ESP32 by probing /sensor on each host."""
#     import socket
#     import struct

#     # Get our own IP and subnet
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     try:
#         s.connect(("8.8.8.8", 80))
#         my_ip = s.getsockname()[0]
#     except Exception:
#         my_ip = "10.0.0.1"
#     finally:
#         s.close()

#     # Derive network prefix (scan last octet 1-254)
#     prefix = my_ip.rsplit(".", 1)[0]
#     print(f"  Scanning {prefix}.1-254 for ESP32...")

#     def probe(ip):
#         try:
#             req = urllib.request.urlopen(f"http://{ip}/sensor", timeout=0.5)
#             data = req.read().decode("utf-8", errors="replace")
#             if "ultrasonic" in data:
#                 return ip
#         except Exception:
#             pass
#         return None

#     import concurrent.futures
#     with concurrent.futures.ThreadPoolExecutor(max_workers=50) as pool:
#         futures = {pool.submit(probe, f"{prefix}.{i}"): i for i in range(1, 255)}
#         deadline = time.time() + timeout
#         for future in concurrent.futures.as_completed(futures):
#             if time.time() > deadline:
#                 break
#             result = future.result()
#             if result:
#                 print(f"  ESP32 found at {result}")
#                 return result

#     print("  WARNING: ESP32 not found on network")
#     return None

# # Obstacle avoidance thresholds (cm)
# STOP_DIST       = 25            # emergency stop — back up immediately
# SLOW_DIST       = 40            # reduce speed
# TURN_DIST       = 60            # start steering away

# # Driving speeds — SLOW AND SAFE (people nearby!)
# # Track drive (tank-style): forward/backward = 40%, turning = 60%
# # Tracks need more power to skid-steer than to drive straight
# ROAM_SPEED      = 40            # forward/backward — 40% duty
# SLOW_SPEED      = 30            # near obstacles — slow crawl
# APPROACH_SPEED  = 40            # approaching a bottle — 40%
# TURN_SPEED      = 60            # turning — 60% (tracks need more to skid-steer)
# SCAN_SPEED      = 60            # scanning rotation — 60%
# ALIGN_SPEED     = 35            # fine alignment turns before pickup — slow to avoid overshoot
# VERIFY_SPEED    = 30            # verification approach — extra cautious

# # Bottle detection & verification
# PICKUP_DIST_CM    = 20          # front ultrasonic ≤ 20 cm → close enough to pick
# PICKUP_FILL_FALLBACK = 0.25     # camera fallback: bottle fills 25% of frame → pick (for lying bottles ultrasonic misses)
# BOTTLE_CENTER_TOL = 0.15        # tolerance from frame center (fraction)
# VERIFY_FRAMES     = 75          # must see bottle in N frames before approaching
# SCAN_STEP_TURN_S  = 0.4         # seconds to spin per step (~25° at SCAN_SPEED 60%)
# SCAN_PAUSE_S      = 2.5         # seconds to pause and look after each step
# SCAN_MAX_STEPS    = 15          # steps per scan cycle (~375° total = full rotation + margin)

# # COCO obstacle classes (things the robot should avoid)
# COCO_OBSTACLE_CLASSES = {
#     0: "person", 56: "chair", 57: "couch", 58: "potted plant",
#     59: "bed", 60: "dining table", 62: "tv",
# }


# # ══════════════════════════════════════════════════════════════
# # Web Stream (Flask MJPEG server for ESP32 web UI)
# # ══════════════════════════════════════════════════════════════

# stream_app = Flask(__name__)
# stream_lock = threading.Lock()
# stream_frame = None  # latest JPEG-encoded frame
# stream_stats = {"fps": 0, "bottles": 0, "persons": 0, "model": "", "inference_ms": 0, "state": "WAITING",
#                 "ultrasonic": {"s1": 999, "s2": 999, "s3": 999, "s4": 999},
#                 "binFull": False, "pickups": 0, "avoidances": 0, "espConnected": False,
#                 "esp32_ip": None, "cameraOk": False}
# _navigator_ref = None  # set by Navigator.__init__ so Flask routes can control it
# _manual_mode_until = 0.0  # epoch time until which manual mode suppresses periodic stops

# # Data logger — records events during navigation
# data_log = []
# data_log_lock = threading.Lock()

# def log_event(event_type, details):
#     ts = time.strftime("%Y-%m-%d %H:%M:%S")
#     entry = {"time": ts, "event": event_type, **details}
#     with data_log_lock:
#         data_log.append(entry)

# def update_stream(frame_bgr, bottles, persons, model_name, inference_ms, state_name,
#                   ultrasonic=None, bin_full=None):
#     """Called from navigator loop to update the shared frame and stats."""
#     global stream_frame, stream_stats
#     _, jpeg = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
#     with stream_lock:
#         stream_frame = jpeg.tobytes()
#         stream_stats["bottles"] = len(bottles)
#         stream_stats["persons"] = len(persons)
#         stream_stats["model"] = model_name
#         stream_stats["inference_ms"] = round(inference_ms, 1)
#         stream_stats["state"] = state_name
#         if ultrasonic:
#             stream_stats["ultrasonic"] = ultrasonic
#         if bin_full is not None:
#             stream_stats["binFull"] = bool(bin_full)
#         if _navigator_ref:
#             stream_stats["pickups"] = _navigator_ref.pickup_success_count
#             stream_stats["avoidances"] = getattr(_navigator_ref, 'avoid_count', 0)
#             stream_stats["espConnected"] = _navigator_ref.esp32._connected.is_set()

# def generate_mjpeg():
#     while True:
#         with stream_lock:
#             frame = stream_frame
#         if frame is None:
#             time.sleep(0.03)
#             continue
#         yield (b'--frame\r\n'
#                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
#         time.sleep(0.03)

# @stream_app.route('/video_feed')
# def video_feed():
#     return Response(generate_mjpeg(),
#                     mimetype='multipart/x-mixed-replace; boundary=frame')

# @stream_app.route('/stats')
# def get_stats():
#     with stream_lock:
#         return jsonify(stream_stats)

# @stream_app.route('/esp32_reconnect')
# def esp32_reconnect():
#     """Allow pi_admin (or other callers) to update the ESP32 IP when it changes."""
#     global stream_stats
#     ip = request.args.get('ip', '').strip()
#     if not ip or not _navigator_ref:
#         return jsonify({"ok": False, "error": "no ip or navigator not ready"})
#     _navigator_ref.esp32.base_url = f"http://{ip}"
#     _navigator_ref.esp32._poll_fails = 0
#     with stream_lock:
#         stream_stats["esp32_ip"] = ip
#     print(f"  ESP32 IP updated to {ip} via /esp32_reconnect")
#     return jsonify({"ok": True, "ip": ip})


# @stream_app.route('/manual_mode')
# def set_manual_mode():
#     global _manual_mode_until
#     _manual_mode_until = time.time() + 30.0  # suppress periodic stops for 30 s
#     # Also reset the stop timer so the next periodic stop is 30 s from now
#     if _navigator_ref:
#         _navigator_ref._stop_sent_time = time.time()
#     return jsonify({"ok": True})

# @stream_app.route('/start')
# def nav_start():
#     if _navigator_ref and _navigator_ref.state in (State.WAITING, State.STOPPED,
#                                                      State.MISSION_COMPLETE):
#         if not _navigator_ref.esp32._connected.is_set():
#             return jsonify({"ok": False, "state": "WAITING", "msg": "ESP32 not connected via WiFi yet"})
#         _navigator_ref.pickup_success_count = 0
#         _navigator_ref.ir_ever_tripped = False
#         _navigator_ref.infer_fail_streak = 0
#         _navigator_ref._reset_scan()
#         return jsonify({"ok": True, "state": "SCANNING"})
#     elif _navigator_ref:
#         return jsonify({"ok": False, "state": _navigator_ref.state.name, "msg": "already running"})
#     return jsonify({"ok": False, "msg": "navigator not ready"})

# @stream_app.route('/stop')
# def nav_stop():
#     if _navigator_ref:
#         _navigator_ref.state = State.WAITING
#         _navigator_ref._stop_sent_time = 0
#         _navigator_ref.esp32._last_cmd = None
#         _navigator_ref.esp32._last_swing_cmd = None
#         _navigator_ref.esp32.emergency_stop()
#         _navigator_ref.esp32.cmd("PISWS")
#         _navigator_ref.esp32.emergency_stop()
#         return jsonify({"ok": True, "state": "WAITING"})
#     return jsonify({"ok": False, "msg": "navigator not ready"})

# @stream_app.route('/esp32_cmd')
# def esp32_proxy_cmd():
#     c = request.args.get("c", "")
#     if not c:
#         return jsonify({"ok": False, "error": "no command"})
#     if _navigator_ref and _navigator_ref.esp32._connected.is_set():
#         _navigator_ref.esp32.cmd(c)
#         return jsonify({"ok": True, "resp": "OK"})
#     return jsonify({"ok": False, "error": "ESP32 not connected"})

# @stream_app.route('/resume')
# def nav_resume():
#     if _navigator_ref and _navigator_ref.state == State.STOPPED:
#         _navigator_ref._reset_scan()
#         return jsonify({"ok": True, "state": "SCANNING"})
#     elif _navigator_ref:
#         return jsonify({"ok": False, "state": _navigator_ref.state.name})
#     return jsonify({"ok": False, "msg": "navigator not ready"})

# @stream_app.route('/models')
# def list_models():
#     if _navigator_ref:
#         mgr = _navigator_ref.manager
#         available = list(mgr.models.keys())
#         return jsonify({"ok": True, "models": available, "active": mgr.active_key})
#     return jsonify({"ok": False, "msg": "navigator not ready"})

# @stream_app.route('/switch_model')
# def switch_model():
#     key = request.args.get('model', '')
#     if not _navigator_ref:
#         return jsonify({"ok": False, "msg": "navigator not ready"})
#     mgr = _navigator_ref.manager
#     if key not in mgr.models:
#         return jsonify({"ok": False, "msg": f"unknown model '{key}'", "available": list(mgr.models.keys())})
#     mgr.activate(key)
#     return jsonify({"ok": True, "active": mgr.active_key, "name": mgr.active_name})

# @stream_app.route('/')
# def stream_index():
#     return R"""<!DOCTYPE html>
# <html>
# <head>
# <meta charset="UTF-8">
# <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
# <title>PET Bottle Navigator</title>
# <style>
# *{box-sizing:border-box;margin:0;padding:0}
# body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:#0a0a1a;color:#eee;padding:12px;
#   -webkit-user-select:none;user-select:none}
# h1{text-align:center;font-size:1.3rem;margin-bottom:8px;color:#4fc3f7}
# .video-wrap{position:relative;text-align:center;margin-bottom:10px}
# .video-wrap img{width:100%;max-width:640px;border-radius:12px;border:2px solid #333}
# .badge{position:absolute;top:10px;right:10px;padding:6px 14px;border-radius:8px;
#   font-weight:700;font-size:.9rem;color:#111}
# .wifi-dot{position:absolute;top:10px;left:10px;padding:6px 14px;border-radius:8px;
#   font-weight:700;font-size:.75rem}
# .section{max-width:640px;margin:10px auto}
# .controls{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;margin:12px 0}
# button{font-size:1rem;font-weight:700;border:none;border-radius:12px;
#   padding:12px 24px;cursor:pointer;transition:all .15s;touch-action:manipulation}
# button:active{transform:scale(.94)}
# .go{background:#66bb6a;color:#111}
# .stop{background:#ef5350;color:#fff}
# .models{display:flex;gap:8px;justify-content:center;flex-wrap:wrap;margin:8px 0}
# .mbtn{background:#1a1a3e;color:#888;border:2px solid #333;padding:10px 20px;border-radius:10px}
# .mbtn.active{color:#fff;border-color:#4fc3f7;background:#1a2a4e}
# .mbtn.v5{border-color:#ffeb3b}.mbtn.v5.active{color:#ffeb3b;background:#2a2a1e;border-color:#ffeb3b}
# .mbtn.v7{border-color:#e040fb}.mbtn.v7.active{color:#e040fb;background:#2a1a2e;border-color:#e040fb}
# .mbtn.v8{border-color:#4fc3f7}.mbtn.v8.active{color:#4fc3f7;background:#1a2a4e;border-color:#4fc3f7}
# .stats{background:#1a1a2e;border-radius:12px;padding:12px;
#   display:grid;grid-template-columns:repeat(auto-fit,minmax(80px,1fr));gap:8px}
# .stat{text-align:center}
# .stat .val{font-size:1.3rem;font-weight:700;color:#4fc3f7}
# .stat .lbl{font-size:.65rem;color:#888;margin-top:2px}
# .us-bar{background:#1a1a2e;border-radius:12px;padding:12px;margin-top:8px;
#   display:grid;grid-template-columns:repeat(4,1fr);gap:8px;text-align:center}
# .us-bar .val{font-size:1.1rem;font-weight:700}
# .us-bar .lbl{font-size:.65rem;color:#888}
# #log{background:#000;border-radius:8px;padding:8px;margin-top:8px;
#   font-family:monospace;font-size:.7rem;color:#4fc3f7;
#   height:80px;overflow-y:auto;white-space:pre-wrap}
# .dl-btn{background:#1a1a3e;color:#4fc3f7;border:1px solid #333;padding:8px 16px;
#   border-radius:8px;font-size:.85rem;margin-top:8px;display:inline-block;text-decoration:none}
# .footer{text-align:center;color:#555;font-size:.65rem;margin-top:12px}
# </style>
# </head>
# <body>
# <h1>PET Bottle Navigator</h1>

# <div class="video-wrap">
#   <img src="/video_feed" alt="Live Feed">
#   <div class="badge" id="badge" style="background:#ff9800">WAITING</div>
#   <div class="wifi-dot" id="wifi" style="background:#ef5350;color:#fff">WiFi: --</div>
# </div>

# <div class="section">
#   <div class="controls">
#     <button class="go" onclick="act('/start')">START</button>
#     <button class="stop" onclick="act('/stop')">STOP</button>
#   </div>

#   <div class="models">
#     <button class="mbtn v5" onclick="switchModel('yolov5')">YOLOv5</button>
#     <button class="mbtn v7" onclick="switchModel('yolov7')">YOLOv7</button>
#     <button class="mbtn v8 active" onclick="switchModel('yolov8')">YOLOv8</button>
#   </div>

#   <div class="stats">
#     <div class="stat"><div class="val" id="fps">-</div><div class="lbl">FPS</div></div>
#     <div class="stat"><div class="val" id="bottles">0</div><div class="lbl">Bottles</div></div>
#     <div class="stat"><div class="val" id="persons">0</div><div class="lbl">Persons</div></div>
#     <div class="stat"><div class="val" id="infer">-</div><div class="lbl">Infer ms</div></div>
#     <div class="stat"><div class="val" id="model">-</div><div class="lbl">Model</div></div>
#     <div class="stat"><div class="val" id="bin">-</div><div class="lbl">Bin</div></div>
#     <div class="stat"><div class="val" id="pickups">0</div><div class="lbl">Pickups</div></div>
#     <div class="stat"><div class="val" id="avoids">0</div><div class="lbl">Avoidances</div></div>
#   </div>

#   <div class="us-bar">
#     <div><div class="val" id="us-f" style="color:#4fc3f7">--</div><div class="lbl">Front</div></div>
#     <div><div class="val" id="us-r" style="color:#4fc3f7">--</div><div class="lbl">Right</div></div>
#     <div><div class="val" id="us-b" style="color:#4fc3f7">--</div><div class="lbl">Back</div></div>
#     <div><div class="val" id="us-l" style="color:#4fc3f7">--</div><div class="lbl">Left</div></div>
#   </div>

#   <div id="log">Ready.
# </div>
#   <div id="cmdlog" style="background:#0a0a2a;border-radius:8px;padding:8px;margin-top:8px;
#     font-family:monospace;font-size:.7rem;color:#66bb6a;height:60px;overflow-y:auto;white-space:pre-wrap">Pi -> ESP32 commands:
# </div>
#   <div style="margin-top:8px;display:flex;gap:8px">
#     <a class="dl-btn" href="/data_log" download="navigation_log.csv">Download CSV Log</a>
#     <button class="dl-btn" onclick="fetch('/clear_log').then(()=>{log.textContent='Log cleared.\\n'})">Clear Log</button>
#   </div>
# </div>

# <div class="footer">PET Bottle Collector Robot &mdash; Emathrix Academy</div>

# <script>
# const stateColors={WAITING:'#ff9800',SCANNING:'#ffeb3b',ROAMING:'#66bb6a',
#   VERIFYING:'#ffcc80',APPROACHING:'#4fc3f7',ALIGNING:'#ffd54f',
#   PICKING_UP:'#ab47bc',AVOIDING:'#ef5350',STOPPED:'#666',
#   MISSION_COMPLETE:'#ffc107'};
# const log=document.getElementById('log');

# function act(url){
#   fetch(url).then(r=>r.json()).then(d=>{
#     log.textContent+='> '+url+' -> '+JSON.stringify(d)+'\n';
#     log.scrollTop=log.scrollHeight;
#   }).catch(e=>{log.textContent+='ERR: '+e+'\n'});
# }

# function switchModel(m){
#   fetch('/switch_model?model='+m).then(r=>r.json()).then(d=>{
#     if(d.ok){
#       document.querySelectorAll('.mbtn').forEach(b=>b.classList.remove('active'));
#       document.querySelector('.mbtn.'+m.replace('yolo','')).classList.add('active');
#       log.textContent+='> Model: '+d.active+'\n';
#       log.scrollTop=log.scrollHeight;
#     }
#   });
# }

# function usColor(v){
#   if(v>=999) return '#555';
#   if(v<60) return '#ef5350';
#   if(v<100) return '#ff9800';
#   return '#66bb6a';
# }

# function poll(){
#   fetch('/stats').then(r=>r.json()).then(d=>{
#     document.getElementById('fps').textContent=d.fps||'-';
#     document.getElementById('bottles').textContent=d.bottles||0;
#     document.getElementById('persons').textContent=d.persons||0;
#     document.getElementById('infer').textContent=d.inference_ms||'-';
#     document.getElementById('model').textContent=d.model||'-';
#     document.getElementById('pickups').textContent=d.pickups||0;
#     document.getElementById('avoids').textContent=d.avoidances||0;
#     const binEl=document.getElementById('bin');
#     binEl.textContent=d.binFull?'FULL':'OK';
#     binEl.style.color=d.binFull?'#ffc107':'#4fc3f7';
#     const b=document.getElementById('badge');
#     b.textContent=d.state||'?';
#     b.style.background=stateColors[d.state]||'#666';
#     if(d.state==='MISSION_COMPLETE') b.textContent='BIN FULL';
#     const wifi=document.getElementById('wifi');
#     wifi.textContent=d.espConnected?'WiFi: Connected':'WiFi: Disconnected';
#     wifi.style.background=d.espConnected?'#66bb6a':'#ef5350';
#     if(d.ultrasonic){
#       var u=d.ultrasonic;
#       ['f','r','b','l'].forEach((k,i)=>{
#         var key='s'+(i+1);var v=u[key]||999;
#         var el=document.getElementById('us-'+k);
#         el.textContent=v>=999?'--':v+'cm';
#         el.style.color=usColor(v);
#       });
#     }
#   }).catch(()=>{});
# }
# function pollCmd(){
#   fetch('/cmdlog').then(r=>r.json()).then(d=>{
#     if(d.ok&&d.log.length){
#       var cl=document.getElementById('cmdlog');
#       cl.textContent='Pi -> ESP32:\n';
#       d.log.slice(-10).forEach(e=>{
#         var c=e.resp.startsWith('OK')?'':'! ';
#         cl.textContent+=c+e.time+' '+e.cmd+' -> '+e.resp+'\n';
#       });
#       cl.scrollTop=cl.scrollHeight;
#     }
#   }).catch(()=>{});
# }
# setInterval(poll,500);
# setInterval(pollCmd,1000);
# poll();
# </script>
# </body>
# </html>"""

# @stream_app.route('/cmdlog')
# def get_cmdlog():
#     if _navigator_ref:
#         log = _navigator_ref.esp32.recent_log
#         return jsonify({"ok": True, "log": [{"time": t, "cmd": c, "resp": r} for t, c, r in log]})
#     return jsonify({"ok": False, "log": []})

# @stream_app.route('/data_log')
# def download_data_log():
#     import csv, io
#     si = io.StringIO()
#     writer = csv.writer(si)
#     writer.writerow(["time", "event", "model", "bottles_detected", "persons_detected",
#                      "inference_ms", "bottle_distance", "us_front", "us_right",
#                      "us_back", "us_left", "state", "detail"])
#     with data_log_lock:
#         for entry in data_log:
#             writer.writerow([
#                 entry.get("time", ""),
#                 entry.get("event", ""),
#                 entry.get("model", ""),
#                 entry.get("bottles", ""),
#                 entry.get("persons", ""),
#                 entry.get("inference_ms", ""),
#                 entry.get("bottle_distance", ""),
#                 entry.get("us_front", ""),
#                 entry.get("us_right", ""),
#                 entry.get("us_back", ""),
#                 entry.get("us_left", ""),
#                 entry.get("state", ""),
#                 entry.get("detail", ""),
#             ])
#     output = si.getvalue()
#     return Response(output, mimetype="text/csv",
#                     headers={"Content-Disposition": "attachment;filename=navigation_log.csv"})

# @stream_app.route('/clear_log')
# def clear_log():
#     with data_log_lock:
#         data_log.clear()
#     return jsonify({"ok": True, "msg": "log cleared"})

# def start_stream_server(port=5000):
#     """Run Flask in a background thread."""
#     stream_app.run(host='0.0.0.0', port=port, threaded=True, use_reloader=False)


# # ══════════════════════════════════════════════════════════════
# # Robot States
# # ══════════════════════════════════════════════════════════════

# class State(Enum):
#     WAITING          = auto()   # safety hold — motors disabled until user confirms
#     SCANNING         = auto()   # slow rotation to look around for bottles
#     ROAMING          = auto()   # slow forward movement, exploring
#     VERIFYING        = auto()   # bottle maybe seen — moving closer slowly to confirm
#     APPROACHING      = auto()   # bottle confirmed — driving towards it slowly
#     ALIGNING         = auto()   # close to bottle, fine-tuning position
#     PICKING_UP       = auto()   # running pickup sequence on ESP32
#     AVOIDING         = auto()   # obstacle detected, steering away slowly
#     STOPPED          = auto()   # emergency stop / idle
#     MISSION_COMPLETE = auto()   # bin full (IR proximity tripped) — terminal state


# # Bin-full detection (E18-D80NK IR proximity sensor over bin opening)
# BIN_SETTLE_DELAY_S   = 1.5  # wait after pickup for dropped bottle to settle
# BIN_DEBOUNCE_SAMPLES = 3    # consecutive irProx=true readings to declare full
# BIN_DEBOUNCE_PERIOD  = 0.25 # seconds between samples

# # Backup safeguard for the bin-full IR sensor. With INPUT_PULLUP, an
# # unplugged or broken E18-D80NK reads HIGH → "false" → bin always
# # reports empty → mission never ends. Hard-cap pickups so the robot
# # can't loop forever in that failure mode.
# MAX_PICKUPS_BACKSTOP = 20   # absolute cap regardless of IR sensor

# # Sensor freshness watchdog: ESP32 pushes a JSON packet every 100 ms.
# # If we haven't seen one for this long, assume the link is dead (USB
# # unplug, reader thread crash) and force a stop — otherwise the
# # navigator would keep deciding based on frozen sensor values.
# SENSOR_STALE_S = 10.0

# # YOLO inference watchdog: if the model errors N times in a row,
# # stop driving — we have no eyes.
# INFER_FAIL_LIMIT = 15


# # ══════════════════════════════════════════════════════════════
# # ESP32 Communication — WiFi HTTP transport
# # ══════════════════════════════════════════════════════════════

# class ESP32WiFiLink:
#     """Talks to the ESP32 over WiFi HTTP.
#     Sends commands via GET /cmd?c=<CMD> and polls sensor data
#     via GET /sensor every 200ms in a background thread."""

#     MAX_LOG = 20

#     def __init__(self, ip=ESP32_IP, block=False):
#         self.base_url = f"http://{ip}"
#         self.sensor_data = {}
#         self._sensor_data_ts = 0.0
#         self.cmd_log = []
#         self._last_cmd = None
#         self._lock = threading.Lock()
#         self._running = True
#         self._connected = threading.Event()
#         self._poll_thread = threading.Thread(target=self._poll_sensors, daemon=True)
#         self._poll_thread.start()
#         if block:
#             if not self._connected.wait(timeout=15):
#                 print(f"  WARNING: ESP32 not responding at {ip} — running camera-only mode")
#         else:
#             print(f"  WiFi connecting in background — polling {self.base_url}/sensor")

#     def _poll_sensors(self):
#         _last_rediscover = 0.0
#         _rediscovering = False
#         _rediscover_lock = threading.Lock()

#         def _do_rediscover():
#             nonlocal _rediscovering
#             try:
#                 new_ip = discover_esp32()
#                 if new_ip:
#                     new_url = f"http://{new_ip}"
#                     if new_url != self.base_url:
#                         print(f"  ESP32 found at new IP: {new_ip} (was {self.base_url})")
#                         self.base_url = new_url
#                         with stream_lock:
#                             stream_stats["esp32_ip"] = new_ip
#                     self._poll_fails = 0
#             finally:
#                 with _rediscover_lock:
#                     _rediscovering = False

#         while self._running:
#             try:
#                 req = urllib.request.urlopen(f"{self.base_url}/sensor", timeout=2)
#                 raw = req.read().decode("utf-8", errors="replace")
#                 parsed = json.loads(raw)
#                 with self._lock:
#                     self.sensor_data = parsed
#                     self._sensor_data_ts = time.time()
#                 self._poll_fails = 0
#                 if not self._connected.is_set():
#                     self._connected.set()
#                     print(f"  WiFi connected to ESP32 at {self.base_url}")
#             except Exception:
#                 self._poll_fails = getattr(self, '_poll_fails', 0) + 1
#                 if self._poll_fails >= 5:
#                     with self._lock:
#                         self._sensor_data_ts = 0.0
#                     if self._connected.is_set():
#                         self._connected.clear()
#                         print(f"  WiFi lost connection to ESP32")
#                     # Kick off background rediscovery every 15 s while disconnected
#                     now = time.time()
#                     with _rediscover_lock:
#                         if not _rediscovering and now - _last_rediscover >= 15.0:
#                             _rediscovering = True
#                             _last_rediscover = now
#                             threading.Thread(target=_do_rediscover, daemon=True).start()
#             time.sleep(0.2)

#     def _log(self, command, response):
#         if command == self._last_cmd and command not in ("PIX", "PISTOP", "P", "PA"):
#             return
#         self._last_cmd = command
#         ts = time.strftime("%H:%M:%S")
#         with self._lock:
#             self.cmd_log.append((ts, command, response))
#             if len(self.cmd_log) > self.MAX_LOG:
#                 self.cmd_log.pop(0)

#     @property
#     def recent_log(self):
#         with self._lock:
#             return list(self.cmd_log)

#     @property
#     def ultrasonic(self):
#         with self._lock:
#             us = self.sensor_data.get("ultrasonic", {})
#             return {
#                 "s1": us.get("s1", 999),
#                 "s2": us.get("s2", 999),
#                 "s3": us.get("s3", 999),
#                 "s4": us.get("s4", 999),
#             }

#     @property
#     def ultrasonic_min(self):
#         us = self.ultrasonic
#         return min(us["s1"], us["s2"], us["s3"], us["s4"])

#     @property
#     def sensors(self):
#         with self._lock:
#             return dict(self.sensor_data)

#     @property
#     def is_connected(self):
#         with self._lock:
#             return "ultrasonic" in self.sensor_data

#     def sensor_age(self):
#         with self._lock:
#             if self._sensor_data_ts == 0.0:
#                 return float('inf')
#             return time.time() - self._sensor_data_ts

#     def cmd(self, command):
#         now = time.time()
#         no_dedup = command in ("PIX", "PISTOP", "PA", "PISWS")
#         is_swing = command in ("PISWL", "PISWR")
#         if is_swing:
#             if command == getattr(self, '_last_swing_cmd', None) and now - getattr(self, '_last_swing_time', 0) < 2.0:
#                 return
#             self._last_swing_cmd = command
#             self._last_swing_time = now
#         elif not no_dedup and command == self._last_cmd and now - getattr(self, '_last_cmd_time', 0) < 0.5:
#             return
#         self._last_cmd = command
#         self._last_cmd_time = now
#         try:
#             url = f"{self.base_url}/cmd?c={urllib.parse.quote(command)}"
#             req = urllib.request.urlopen(url, timeout=2)
#             resp = req.read().decode("utf-8", errors="replace").strip()
#             self._log(command, resp[:20] if resp else "OK")
#         except Exception as e:
#             self._log(command, f"ERR: {e}")

#     def forward(self, speed=ROAM_SPEED):
#         self.cmd(f"PIFW{speed}")

#     def backward(self, speed=ROAM_SPEED):
#         self.cmd(f"PIBW{speed}")

#     def turn_left(self, speed=TURN_SPEED):
#         self.cmd(f"PITL{speed}")

#     def turn_right(self, speed=TURN_SPEED):
#         self.cmd(f"PITR{speed}")

#     def differential(self, left_speed, right_speed):
#         self.cmd(f"PIDW{left_speed},{right_speed}")

#     def stop(self):
#         self.cmd("PIX")

#     def emergency_stop(self):
#         self.cmd("PISTOP")

#     def swing_left(self):
#         self.cmd("PISWL")

#     def swing_right(self):
#         self.cmd("PISWR")

#     def swing_stop(self):
#         self.cmd("PISWS")

#     def pickup(self):
#         self.cmd("P")

#     def shutdown(self):
#         self._running = False
#         try:
#             self.stop()
#         except Exception:
#             pass


# # ══════════════════════════════════════════════════════════════
# # Autonomous Navigator
# # ══════════════════════════════════════════════════════════════

# class Navigator:
#     """
#     Main brain: fuses camera + ultrasonics for real-time navigation.
#     Roams → detects bottle → approaches → picks up → resumes.
#     """

#     def __init__(self, manager, esp32, conf_thresh, no_show, out_dir):
#         global _navigator_ref
#         self.manager     = manager
#         self.esp32       = esp32
#         self.conf_thresh = conf_thresh
#         self.no_show     = no_show
#         self.out_dir     = out_dir

#         self.state       = State.WAITING
#         _navigator_ref   = self  # expose to Flask routes
#         self.tracker     = DetectionTracker()
#         self.turn_dir    = 1         # 1 = right, -1 = left (roaming direction)
#         self.turn_timer  = 0.0
#         self.avoid_timer = 0.0
#         self.scan_start  = 0.0       # when scanning started
#         self.scan_step   = 0         # current step in step-scan cycle
#         self.scan_phase  = "turn"    # "turn" or "pause"
#         self.scan_phase_start = 0.0  # when current phase started
#         self.verify_count = 0        # frames bottle has been seen during verify
#         self.verify_lost  = 0        # frames bottle was NOT seen during verify
#         self.approach_lost_count = 0
#         # Mission counters (drive bin-full backstop & sensor sanity check)
#         self.pickup_success_count = 0   # successful pickups this mission
#         self.avoid_count          = 0   # obstacle avoidance events
#         self.ir_ever_tripped      = False  # has irProx ever read True since boot?
#         # Watchdog state (sensor freshness + YOLO inference health)
#         self.infer_fail_streak    = 0
#         # AI vision state
#         self._current_frame       = None
#         self._ai_verified         = False
#         self._ai_scene_time       = 0.0
#         self._ai_scene_interval   = 8.0

#     def _open_camera(self):
#         """Try picamera2 then V4L2 indices 0-2. Returns (use_picamera2, picam2, cap).
#         Retries every 5 s until a camera becomes available."""
#         while True:
#             # 1. Try picamera2 (CSI ribbon)
#             try:
#                 from picamera2 import Picamera2
#                 picam2 = Picamera2()
#                 picam2.configure(picam2.create_preview_configuration(
#                     main={"format": "BGR888", "size": (640, 480)}
#                 ))
#                 picam2.start()
#                 with stream_lock:
#                     stream_stats["cameraOk"] = True
#                 print("  Camera: picamera2 (CSI ribbon)")
#                 return True, picam2, None
#             except Exception as e:
#                 print(f"  picamera2 unavailable ({e})")

#             # 2. Try V4L2 USB camera on indices 0, 1, 2
#             for idx in range(3):
#                 cap = cv2.VideoCapture(idx)
#                 if cap.isOpened():
#                     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#                     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#                     with stream_lock:
#                         stream_stats["cameraOk"] = True
#                     print(f"  Camera: V4L2 /dev/video{idx}")
#                     return False, None, cap
#                 cap.release()

#             # No camera found — wait and retry
#             print("  No camera found — retrying in 5 s...")
#             with stream_lock:
#                 stream_stats["cameraOk"] = False
#             time.sleep(5)

#     def run(self):
#         """Main loop — camera + sensors + navigation."""
#         use_picamera2, picam2, cap = self._open_camera()

#         if not self.no_show:
#             cv2.namedWindow("PET Bottle Navigator", cv2.WINDOW_NORMAL)

#         print(f"\n{'='*60}")
#         print(f"  PET Bottle Robot — AUTONOMOUS NAVIGATOR")
#         print(f"  Camera + Ultrasonic Fusion")
#         print(f"  ESP32: WiFi ({ESP32_BASE_URL})")
#         print(f"  SAFETY MODE: Motors DISABLED")
#         print(f"  Press G to GO (enable motors) | Q to quit")
#         print(f"{'='*60}\n")

#         # Send stop command to ESP32 to ensure motors are off
#         self.esp32.stop()

#         frame_idx = 0

#         try:
#             while True:
#                 # ── Grab frame ────────────────────────────────
#                 if use_picamera2:
#                     frame_rgb = picam2.capture_array()
#                     frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
#                 else:
#                     ret, frame = cap.read()
#                     if not ret:
#                         print("\n  Camera lost — reconnecting...")
#                         cap.release()
#                         with stream_lock:
#                             stream_stats["cameraOk"] = False
#                         _, picam2, cap = self._open_camera()
#                         use_picamera2 = picam2 is not None
#                         continue

#                 orig_h, orig_w = frame.shape[:2]
#                 frame_area = orig_w * orig_h

#                 # ── Run detection ─────────────────────────────
#                 # Inference is wrapped: a Hailo runtime hiccup, model
#                 # load fault, or driver glitch must NOT crash the main
#                 # loop and leave the wheels at last-commanded speed.
#                 # On a streak of consecutive failures we stop driving.
#                 t0 = time.time()
#                 try:
#                     bottles, persons = self.manager.infer(
#                         frame, self.conf_thresh, apply_range_filter=False)
#                     self.infer_fail_streak = 0
#                 except Exception as e:
#                     bottles, persons = [], []
#                     self.infer_fail_streak += 1
#                     if self.infer_fail_streak % 50 == 1:
#                         print(f"\n  >>> INFER ERROR ({self.infer_fail_streak}): {e}")
#                 ms = (time.time() - t0) * 1000

#                 smoothed = self.tracker.update(bottles)

#                 # ── Get ultrasonic data ───────────────────────
#                 us = self.esp32.ultrasonic  # {s1:front, s2:right, s3:back, s4:left}

#                 # ── Sensor freshness (log only, no stop) ──────
#                 age = self.esp32.sensor_age()
#                 if age > SENSOR_STALE_S and frame_idx % 100 == 0:
#                     print(f"\n  >>> SENSOR DATA AGE: {age:.1f}s (ESP32 may be slow)")

#                 # ── Store frame for AI vision ─────────────────
#                 self._current_frame = frame.copy()

#                 # ── Navigation decision ───────────────────────
#                 self._navigate(smoothed, persons, orig_w, orig_h,
#                                frame_area, us)

#                 # ── Data logging (1 Hz) ──────────────────────
#                 if frame_idx % 30 == 0 and self.state not in (State.WAITING, State.STOPPED):
#                     bottle_dist = ""
#                     if smoothed:
#                         best = max(smoothed, key=lambda b: (b[2]-b[0])*(b[3]-b[1]))
#                         b_fill = ((best[2]-best[0])*(best[3]-best[1])) / frame_area
#                         bottle_dist = f"{b_fill:.3f}"
#                     log_event("detection", {
#                         "model": self.manager.active_name,
#                         "bottles": len(smoothed),
#                         "persons": len(persons),
#                         "inference_ms": round(ms, 1),
#                         "bottle_distance": bottle_dist,
#                         "us_front": us["s1"], "us_right": us["s2"],
#                         "us_back": us["s3"], "us_left": us["s4"],
#                         "state": self.state.name,
#                     })

#                 # ── Draw results ──────────────────────────────
#                 result = draw_detections(frame.copy(), smoothed, persons)
#                 self._draw_nav_overlay(result, smoothed, us, ms, frame_idx)

#                 # ── Update web stream ────────────────────────
#                 model_name = self.manager.active_name
#                 bin_full = self.esp32.sensors.get("irProx", False)
#                 update_stream(result, smoothed, persons, model_name, ms,
#                               self.state.name, us, bin_full=bin_full)

#                 # ── FPS counter for stream stats ─────────────
#                 self._fps_count = getattr(self, '_fps_count', 0) + 1
#                 if not hasattr(self, '_fps_timer'):
#                     self._fps_timer = time.time()
#                 if time.time() - self._fps_timer >= 1.0:
#                     with stream_lock:
#                         stream_stats["fps"] = self._fps_count
#                     self._fps_count = 0
#                     self._fps_timer = time.time()

#                 # ── Console output ────────────────────────────
#                 icon = "OK" if smoothed else "--"
#                 print(f"\r  [{icon}] {self.state.name:12s} "
#                       f"US F:{us['s1']:3d} R:{us['s2']:3d} "
#                       f"B:{us['s3']:3d} L:{us['s4']:3d} | "
#                       f"{len(smoothed)} det | {ms:.0f}ms   ",
#                       end="", flush=True)

#                 # ── Display + keyboard ────────────────────────
#                 if not self.no_show:
#                     cv2.imshow("PET Bottle Navigator", result)
#                     key = cv2.waitKey(1) & 0xFF
#                     if key in (ord('q'), ord('Q'), 27):
#                         break
#                     elif key in (ord('g'), ord('G')):
#                         if self.state == State.WAITING:
#                             if not self.esp32._connected.is_set():
#                                 print("\n  >>> WAITING for ESP32 WiFi connection...")
#                             else:
#                                 self._reset_scan()
#                                 print("\n  >>> MOTORS ENABLED — SCANNING (slow look-around)")
#                     elif key in (ord('s'), ord('S')):
#                         if self.state == State.STOPPED:
#                             self.state = State.ROAMING
#                             print("\n  >>> RESUMED")
#                         # S key does not interrupt active autonomous operation
#                 else:
#                     # Headless mode: autonomous start is controlled ONLY
#                     # via the /start Flask route (pi_admin dashboard button).
#                     # No stdin auto-start — prevents accidental motor activation
#                     # when running as a systemd service (/dev/null stdin).
#                     pass

#                 frame_idx += 1

#         finally:
#             self.esp32.stop()
#             if use_picamera2 and picam2 is not None:
#                 picam2.stop()
#             elif cap is not None:
#                 cap.release()
#             if not self.no_show:
#                 cv2.destroyAllWindows()
#             print()

#     # ── Navigation logic ──────────────────────────────────────

#     def _navigate(self, bottles, persons, w, h, frame_area, us):
#         """State machine: slow, careful navigation. Safety is #1 priority.
#         us = dict with keys s1(front), s2(right), s3(back), s4(left) in cm."""

#         # ── Safety hold: force stop all motors (once on entry, repeat every 30s) ──
#         if self.state == State.WAITING:
#             now = time.time()
#             if now < _manual_mode_until:  # manual control active — don't override
#                 return
#             if now - getattr(self, '_stop_sent_time', 0) > 30.0:
#                 self._stop_sent_time = now
#                 self.esp32._last_cmd = None
#                 self.esp32.emergency_stop()
#                 self.esp32.swing_stop()
#             return

#         # ── Stopped state: force stop all motors (once on entry, repeat every 30s) ──
#         if self.state == State.STOPPED:
#             now = time.time()
#             if now < _manual_mode_until:  # manual control active — don't override
#                 return
#             if now - getattr(self, '_stop_sent_time', 0) > 30.0:
#                 self._stop_sent_time = now
#                 self.esp32._last_cmd = None
#                 self.esp32.emergency_stop()
#                 self.esp32.swing_stop()
#             return

#         # ── Mission complete (bin full): terminal state ───
#         # Stays here until user presses STOP (-> WAITING) and START again.
#         if self.state == State.MISSION_COMPLETE:
#             return

#         # ── Picking up: wait for sequence to finish ───────
#         if self.state == State.PICKING_UP:
#             return

#         us_front = us["s1"]
#         us_right = us["s2"]
#         us_back  = us["s3"]
#         us_left  = us["s4"]
#         us_min_forward = min(us_front, us_left, us_right)  # sensors facing forward path

#         # ── Avoiding state: back up SLOWLY and turn away ──
#         # Must be checked FIRST so the avoidance timer isn't reset by
#         # the STOP_DIST check below while the robot is still backing up.
#         if self.state == State.AVOIDING:
#             elapsed = time.time() - self.avoid_timer
#             if elapsed < 1.5:
#                 # Back up (check rear sensor first)
#                 if us_back > STOP_DIST:
#                     self.esp32.backward(SLOW_SPEED)
#                 else:
#                     # Blocked behind too — skip to turning
#                     if us_left < us_right:
#                         self.esp32.turn_right(SCAN_SPEED)
#                     else:
#                         self.esp32.turn_left(SCAN_SPEED)
#             elif elapsed < 3.0:
#                 # Turn away from the closer side
#                 if us_left < us_right:
#                     self.esp32.turn_right(SCAN_SPEED)
#                 else:
#                     self.esp32.turn_left(SCAN_SPEED)
#             elif elapsed < 3.4:
#                 # Settle: full stop, give the ultrasonic burst time to
#                 # take a fresh forward reading. Without this pause we'd
#                 # exit AVOIDING based on the value sampled mid-turn,
#                 # which lags reality by hundreds of ms.
#                 self.esp32.stop()
#             else:
#                 # Re-check front clearance before resuming. If something
#                 # is still too close (the obstacle moved with us, or a
#                 # new one entered the path during the turn), restart the
#                 # avoid cycle instead of charging into it.
#                 if us_front < STOP_DIST:
#                     print(f"\n  >>> AVOIDING re-check failed "
#                           f"(F={us_front} min_fwd={us_min_forward}) "
#                           f"— restarting avoid")
#                     self.avoid_timer = time.time()
#                 else:
#                     self.esp32.stop()
#                     self._reset_scan()
#             return

#         # ── PERSON DETECTED → steer away (but don't kill autonomous) ──
#         for p in persons:
#             pw = p[2] - p[0]
#             ph = p[3] - p[1]
#             p_fill = (pw * ph) / frame_area
#             if p_fill > 0.25:  # person very close → avoid
#                 self.esp32.stop()
#                 self._enter_avoiding("PERSON DETECTED")
#                 return

#         # ── Emergency stop: front ultrasonic too close ────
#         # Skip obstacle avoidance when actively pursuing a bottle —
#         # the bottle itself triggers the ultrasonic.
#         _pursuing = self.state in (State.VERIFYING, State.APPROACHING,
#                                    State.ALIGNING)
#         _bottle_in_sight = len(bottles) > 0
#         if us_front < STOP_DIST and not _pursuing:
#             # A bottle in sight during scan/roam IS the obstacle triggering
#             # the sensor — redirect straight to verification instead of avoiding.
#             if _bottle_in_sight and self.state in (State.SCANNING, State.ROAMING):
#                 self.esp32.stop()
#                 self.state = State.VERIFYING
#                 self.verify_count = 1
#                 self.verify_lost = 0
#                 print("\n  >>> Bottle within reach (US triggered) — starting verification")
#                 return
#             self.esp32.stop()
#             self._enter_avoiding(f"FRONT < {STOP_DIST}cm")
#             return

#         # ── Ultrasonic: slow zone → steer away gently (only while ROAMING) ──
#         # Skip if a bottle is visible — the bottle itself may be the obstacle.
#         if us_min_forward < TURN_DIST and not _pursuing and self.state == State.ROAMING:
#             if not _bottle_in_sight:
#                 if us_left < us_right:
#                     self.esp32.turn_right(SCAN_SPEED)
#                 else:
#                     self.esp32.turn_left(SCAN_SPEED)
#                 return

#         # ── SCANNING: step-scan — turn ~25°, pause 2.5s to look, repeat ──
#         if self.state == State.SCANNING:
#             now = time.time()

#             if bottles:
#                 self.esp32.stop()
#                 self.state = State.VERIFYING
#                 self.verify_count = 1
#                 self.verify_lost = 0
#                 print("\n  >>> Possible bottle spotted — verifying...")
#                 return

#             if self.scan_step >= SCAN_MAX_STEPS:
#                 # Full rotation done, no bottle found — roam forward
#                 self.esp32.stop()
#                 if AI_AVAILABLE and (now - self._ai_scene_time) > self._ai_scene_interval:
#                     self._ai_scene_time = now
#                     sensor = self.esp32.sensors
#                     def _ai_scan():
#                         result = analyze_scene(self._current_frame, sensor)
#                         if self.state == State.WAITING:
#                             return
#                         action = result.get("action", "go_forward")
#                         reason = result.get("reason", "")
#                         spotted = result.get("bottles_spotted", 0)
#                         print(f"\n  >>> [AI] {action}: {reason} (bottles_spotted={spotted})")
#                         if self.state == State.WAITING:
#                             return
#                         if spotted > 0:
#                             self._reset_scan()
#                         elif action == "turn_left":
#                             self.esp32.turn_left(SCAN_SPEED)
#                             time.sleep(1.5)
#                             if self.state == State.WAITING:
#                                 return
#                             self.esp32.stop()
#                             self._reset_scan()
#                         elif action == "turn_right":
#                             self.esp32.turn_right(SCAN_SPEED)
#                             time.sleep(1.5)
#                             if self.state == State.WAITING:
#                                 return
#                             self.esp32.stop()
#                             self._reset_scan()
#                         else:
#                             self.state = State.ROAMING
#                             self.turn_timer = time.time()
#                     threading.Thread(target=_ai_scan, daemon=True).start()
#                 else:
#                     self.state = State.ROAMING
#                     self.turn_timer = time.time()
#                 self.turn_dir *= -1
#                 return

#             phase_elapsed = now - self.scan_phase_start

#             if self.scan_phase == "turn":
#                 if phase_elapsed < SCAN_STEP_TURN_S:
#                     if self.turn_dir > 0:
#                         self.esp32.turn_right(SCAN_SPEED)
#                     else:
#                         self.esp32.turn_left(SCAN_SPEED)
#                 else:
#                     self.esp32.stop()
#                     self.scan_phase = "pause"
#                     self.scan_phase_start = now
#             elif self.scan_phase == "pause":
#                 if phase_elapsed >= SCAN_PAUSE_S:
#                     self.scan_step += 1
#                     self.scan_phase = "turn"
#                     self.scan_phase_start = now
#             return

#         # ── VERIFYING: stay still or creep closer to confirm bottle ──
#         if self.state == State.VERIFYING:
#             if bottles:
#                 self.verify_count += 1
#                 self.verify_lost = 0

#                 best = max(bottles, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
#                 bx1, by1, bx2, by2, bconf = best
#                 bw = bx2 - bx1
#                 bh = by2 - by1
#                 b_fill = (bw * bh) / frame_area

#                 if self.verify_count >= VERIFY_FRAMES:
#                     # Bottle confirmed by YOLO — go straight to pickup
#                     b_cx_norm = ((bx1 + bx2) / 2.0) / w
#                     close_by_us = us_front <= PICKUP_DIST_CM
#                     close_by_cam = b_fill >= PICKUP_FILL_FALLBACK
#                     if ((close_by_us or close_by_cam)
#                             and abs(b_cx_norm - 0.5) < BOTTLE_CENTER_TOL):
#                         # Already in range and centered — pick up now
#                         trigger = f"US={us_front}cm" if close_by_us else f"CAM={b_fill:.0%}"
#                         print(f"\n  >>> Bottle CONFIRMED & in range ({trigger}) — starting pickup")
#                         self._start_pickup()
#                     else:
#                         # Not close enough yet — approach and pickup triggers automatically
#                         print(f"\n  >>> Bottle CONFIRMED ({self.verify_count} frames) — approaching")
#                         self.state = State.APPROACHING
#                         self.approach_lost_count = 0
#                     return
#                 elif b_fill < 0.03:
#                     # Too far to be sure — creep forward very slowly
#                     self.esp32.forward(VERIFY_SPEED)
#                 else:
#                     # Close enough to verify from here — just wait
#                     self.esp32.stop()
#             else:
#                 self.verify_lost += 1
#                 self.esp32.stop()
#                 if self.verify_lost > 20:
#                     # Lost it — false alarm, go back to scanning
#                     print("\n  >>> False alarm — resuming scan")
#                     self._reset_scan()
#                     self.verify_count = 0
#             return

#         # ── APPROACHING: bottle confirmed, move towards it slowly ──
#         if self.state == State.APPROACHING:
#             if bottles:
#                 self.approach_lost_count = 0
#                 best = max(bottles, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
#                 bx1, by1, bx2, by2, bconf = best
#                 bw = bx2 - bx1
#                 bh = by2 - by1
#                 b_fill = (bw * bh) / frame_area
#                 b_cx = (bx1 + bx2) / 2.0
#                 b_cx_norm = b_cx / w  # 0.0 = left, 1.0 = right

#                 close_by_us = us_front <= PICKUP_DIST_CM
#                 close_by_cam = b_fill >= PICKUP_FILL_FALLBACK
#                 if close_by_us or close_by_cam:
#                     # Close enough (ultrasonic OR camera fallback for lying bottles)
#                     if abs(b_cx_norm - 0.5) < BOTTLE_CENTER_TOL:
#                         self._start_pickup()
#                     else:
#                         # Align slowly — use ALIGN_SPEED to avoid overshooting
#                         self.state = State.ALIGNING
#                         if b_cx_norm < 0.5:
#                             self.esp32.turn_left(ALIGN_SPEED)
#                         else:
#                             self.esp32.turn_right(ALIGN_SPEED)
#                 else:
#                     # Drive towards bottle slowly, steer gently with differential
#                     if abs(b_cx_norm - 0.5) > 0.25:
#                         if b_cx_norm < 0.5:
#                             self.esp32.differential(VERIFY_SPEED, APPROACH_SPEED)
#                         else:
#                             self.esp32.differential(APPROACH_SPEED, VERIFY_SPEED)
#                     else:
#                         self.esp32.forward(APPROACH_SPEED)
#             else:
#                 if us_front <= PICKUP_DIST_CM:
#                     print(f"\n  >>> Camera lost bottle but US={us_front}cm — starting pickup")
#                     self._start_pickup()
#                 elif us_front <= SLOW_DIST:
#                     self.approach_lost_count = 0
#                     self.esp32.forward(SLOW_SPEED)
#                 else:
#                     self.approach_lost_count += 1
#                     self.esp32.stop()
#                     if self.approach_lost_count > 45:
#                         print("\n  >>> Lost bottle — scanning again")
#                         self._reset_scan()
#             return

#         # ── ALIGNING: fine-tune position before pickup ────
#         if self.state == State.ALIGNING:
#             if bottles:
#                 best = max(bottles, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
#                 bx1, by1, bx2, by2, bconf = best
#                 b_cx_norm = ((bx1 + bx2) / 2.0) / w
#                 if abs(b_cx_norm - 0.5) < BOTTLE_CENTER_TOL:
#                     self._start_pickup()
#                 else:
#                     if b_cx_norm < 0.5:
#                         self.esp32.turn_left(ALIGN_SPEED)
#                     else:
#                         self.esp32.turn_right(ALIGN_SPEED)
#             else:
#                 self.esp32.stop()
#                 self._reset_scan()
#             return

#         # ── ROAMING: move forward slowly, then scan again ─
#         if self.state == State.ROAMING:
#             now = time.time()
#             elapsed = now - self.turn_timer

#             if bottles:
#                 # Spotted something while roaming — stop and verify
#                 self.esp32.stop()
#                 self.state = State.VERIFYING
#                 self.verify_count = 1
#                 self.verify_lost = 0
#                 print("\n  >>> Possible bottle spotted — verifying...")
#                 return

#             if elapsed < 5.0:
#                 # Move forward slowly for 5 seconds
#                 speed = ROAM_SPEED
#                 if us_min_forward < SLOW_DIST:
#                     speed = SLOW_SPEED
#                 self.esp32.forward(speed)
#             else:
#                 # Done moving — stop and scan again
#                 self.esp32.stop()
#                 self._reset_scan()

#     def _reset_scan(self):
#         """Enter SCANNING with fresh step-scan state."""
#         self.state = State.SCANNING
#         self.scan_start = time.time()
#         self.scan_step = 0
#         self.scan_phase = "turn"
#         self.scan_phase_start = time.time()

#     def _start_pickup(self):
#         """Stop, run pickup sequence in a thread, then resume scanning."""
#         self.state = State.PICKING_UP
#         self.esp32.emergency_stop()  # PISTOP — halt all motors before pickup
#         self.esp32.emergency_stop()  # send twice to ensure delivery over WiFi

#         def _do_pickup():
#             time.sleep(1.0)          # wait for robot to fully coast to a stop
#             print("\n  >>> PICKUP SEQUENCE STARTED")
#             self.esp32.pickup()
#             time.sleep(2.0)          # let ESP32 transition out of idle before polling

#             timeout = time.time() + 50
#             pickup_ok = False
#             sequence_started = False
#             while time.time() < timeout:
#                 if self.state == State.WAITING:
#                     print("\n  >>> PICKUP ABORTED — user pressed STOP")
#                     self.esp32.cmd("PA")
#                     return
#                 sensor = self.esp32.sensors
#                 pu_state = sensor.get("pickup", "idle")
#                 if not sequence_started:
#                     # Wait until ESP32 leaves idle — confirms sequence actually started
#                     if pu_state != "idle":
#                         sequence_started = True
#                         print(f"\n  >>> Pickup sequence active ({pu_state})")
#                 else:
#                     # Sequence started — wait for it to return to idle/done
#                     if pu_state in ("idle", "done"):
#                         pickup_ok = True
#                         break
#                 time.sleep(0.5)
#             else:
#                 print("\n  >>> PICKUP TIMEOUT waiting for ESP32 — aborting")
#                 self.esp32.cmd("PA")
#                 time.sleep(1)

#             if self.state == State.WAITING:
#                 print("\n  >>> PICKUP thread respecting STOP — not resuming")
#                 return

#             if pickup_ok:
#                 self.pickup_success_count += 1

#                 if self.pickup_success_count >= MAX_PICKUPS_BACKSTOP:
#                     print(f"\n  >>> BACKSTOP HIT ({MAX_PICKUPS_BACKSTOP} pickups) — "
#                           f"MISSION COMPLETE (IR sensor may be faulty: "
#                           f"ever_tripped={self.ir_ever_tripped})")
#                     self.esp32.stop()
#                     self.esp32.cmd("PISTOP")
#                     self.tracker = DetectionTracker()
#                     self.state = State.MISSION_COMPLETE
#                     return

#                 if self._is_bin_full():
#                     print("\n  >>> BIN FULL (IR proximity tripped) — MISSION COMPLETE")
#                     self.esp32.stop()
#                     self.esp32.cmd("PISTOP")
#                     self.tracker = DetectionTracker()
#                     self.state = State.MISSION_COMPLETE
#                     return

#                 if self.pickup_success_count >= 5 and not self.ir_ever_tripped:
#                     print(f"\n  >>> WARNING: {self.pickup_success_count} pickups "
#                           f"and IR sensor has never read true — check E18-D80NK "
#                           f"wiring/aim")

#             log_event("pickup", {
#                 "detail": f"pickup #{self.pickup_success_count}" if pickup_ok else "pickup failed/timeout",
#                 "state": "PICKING_UP",
#                 "model": self.manager.active_name,
#             })

#             if self.state == State.WAITING:
#                 print("\n  >>> PICKUP thread respecting STOP — not resuming")
#                 return

#             print(f"\n  >>> PICKUP DONE ({self.pickup_success_count}) — "
#                   f"scanning for next bottle")
#             self.tracker = DetectionTracker()
#             self._reset_scan()

#         threading.Thread(target=_do_pickup, daemon=True).start()

#     def _enter_avoiding(self, reason):
#         """Transition into AVOIDING. Resets stale verify/approach counters
#         so when SCANNING resumes the state machine starts fresh — otherwise
#         a bottle 'confirmed' before the obstacle would still appear confirmed
#         after the avoid maneuver, even though the robot has since rotated."""
#         self.state = State.AVOIDING
#         self.avoid_timer = time.time()
#         self.verify_count = 0
#         self.verify_lost = 0
#         self.approach_lost_count = 0
#         self.esp32.stop()
#         self.avoid_count += 1
#         us = self.esp32.ultrasonic
#         log_event("avoidance", {
#             "detail": reason, "state": "AVOIDING",
#             "model": self.manager.active_name,
#             "us_front": us["s1"], "us_right": us["s2"],
#             "us_back": us["s3"], "us_left": us["s4"],
#         })
#         print(f"\n  >>> AVOIDING ({reason})")
#         if AI_AVAILABLE and self._current_frame is not None:
#             sensor = self.esp32.sensors
#             import threading
#             def _ai_assess():
#                 result = assess_obstacle(self._current_frame, sensor)
#                 obstacle = result.get("obstacle", "unknown")
#                 direction = result.get("avoid_direction", "right")
#                 ai_reason = result.get("reason", "")
#                 print(f"\n  >>> [AI] obstacle={obstacle}, avoid={direction}: {ai_reason}")
#             threading.Thread(target=_ai_assess, daemon=True).start()

#     def _is_bin_full(self):
#         """Sample the E18-D80NK IR proximity reading after a settling delay.
#         Conservative: ALL N samples must read true to declare bin full.
#         A single false reading aborts — better to do an extra pickup loop
#         than to end the mission prematurely on a noisy IR sample.

#         Side-effect: sets self.ir_ever_tripped if any sample reads true,
#         so a never-tripping sensor can be flagged as likely unplugged."""
#         time.sleep(BIN_SETTLE_DELAY_S)  # let bottle stop bouncing
#         all_true = True
#         for _ in range(BIN_DEBOUNCE_SAMPLES):
#             sample = bool(self.esp32.sensors.get("irProx", False))
#             if sample:
#                 self.ir_ever_tripped = True
#             else:
#                 all_true = False
#             time.sleep(BIN_DEBOUNCE_PERIOD)
#         return all_true

#     # ── HUD overlay ───────────────────────────────────────

#     def _draw_nav_overlay(self, frame, bottles, us, ms, frame_idx):
#         """Draw navigation HUD on frame.
#         us = dict with keys s1(front), s2(right), s3(back), s4(left)."""
#         h, w = frame.shape[:2]

#         # State badge (top-right)
#         state_colors = {
#             State.WAITING:          (0, 165, 255),
#             State.SCANNING:         (255, 255, 0),
#             State.ROAMING:          (0, 255, 0),
#             State.VERIFYING:        (255, 200, 100),
#             State.APPROACHING:      (0, 200, 255),
#             State.ALIGNING:         (255, 200, 0),
#             State.PICKING_UP:       (255, 0, 255),
#             State.AVOIDING:         (0, 0, 255),
#             State.STOPPED:          (80, 80, 80),
#             State.MISSION_COMPLETE: (0, 215, 255),  # gold (BGR)
#         }
#         color = state_colors.get(self.state, (255, 255, 255))
#         badge = f"[{self.state.name}]"
#         (tw, th), _ = cv2.getTextSize(badge, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
#         cv2.rectangle(frame, (w - tw - 20, 0), (w, th + 16), color, -1)
#         cv2.putText(frame, badge, (w - tw - 10, th + 8),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

#         # Ultrasonic bars — all 4 sensors
#         bar_max_h = 100
#         bar_w = 30
#         margin = 10
#         # Layout: left edge=S4(left), center-left=S1(front), center-right=S3(back), right edge=S2(right)
#         bar_positions = [
#             ("F", us["s1"], margin),
#             ("L", us["s4"], margin + bar_w + 6),
#             ("R", us["s2"], w - bar_w - margin),
#             ("B", us["s3"], w - 2 * bar_w - margin - 6),
#         ]

#         for side, dist, x_pos in bar_positions:
#             fill_h = int(min(dist, 200) / 200.0 * bar_max_h)
#             bar_color = (0, 255, 0) if dist > SLOW_DIST else (
#                 (0, 200, 255) if dist > STOP_DIST else (0, 0, 255)
#             )
#             bar_top = h - margin - bar_max_h
#             cv2.rectangle(frame, (x_pos, bar_top),
#                           (x_pos + bar_w, h - margin), (40, 40, 40), -1)
#             cv2.rectangle(frame, (x_pos, bar_top + (bar_max_h - fill_h)),
#                           (x_pos + bar_w, h - margin), bar_color, -1)
#             cv2.putText(frame, f"{side}:{dist}",
#                         (x_pos, bar_top - 5),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

#         # Detection count + FPS (top-left)
#         status = f"BOTTLES: {len(bottles)}" if bottles else "SCANNING..."
#         det_color = (0, 255, 0) if bottles else (0, 200, 255)
#         cv2.putText(frame, status, (10, 35),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, det_color, 2)
#         if ms > 0:
#             cv2.putText(frame, f"{ms:.0f}ms  {1000/ms:.0f}FPS",
#                         (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
#                         (200, 200, 200), 1)

#         # Sensor data (top-left, below FPS)
#         sensor = self.esp32.sensors
#         wheels = sensor.get("wheels", {})
#         cv2.putText(frame, f"Wheels L:{wheels.get('left',0)} R:{wheels.get('right',0)}"
#                     f"  Lift:{sensor.get('lift',0)}  Base:{sensor.get('base',0)}",
#                     (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

#         # Command log panel (right side)
#         log = self.esp32.recent_log
#         if log:
#             panel_x = w - 280
#             panel_y = 40
#             panel_h = min(len(log), 10) * 18 + 10
#             # Semi-transparent background
#             overlay = frame.copy()
#             cv2.rectangle(overlay, (panel_x - 5, panel_y - 5),
#                           (w - 5, panel_y + panel_h), (0, 0, 0), -1)
#             cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
#             cv2.putText(frame, "CMD LOG (Pi -> ESP32)",
#                         (panel_x, panel_y + 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
#             for i, (ts, cmd, resp) in enumerate(log[-10:]):
#                 y = panel_y + 25 + i * 18
#                 resp_color = (0, 255, 0) if resp.startswith("OK") else (0, 0, 255)
#                 cv2.putText(frame, f"{ts} {cmd:8s} -> {resp}",
#                             (panel_x, y),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.33, resp_color, 1)

#         # Verify progress bar (when verifying)
#         if self.state == State.VERIFYING:
#             progress = min(self.verify_count / VERIFY_FRAMES, 1.0)
#             bar_x, bar_y = w // 2 - 100, h - 50
#             cv2.rectangle(frame, (bar_x, bar_y), (bar_x + 200, bar_y + 15), (40, 40, 40), -1)
#             cv2.rectangle(frame, (bar_x, bar_y),
#                           (bar_x + int(200 * progress), bar_y + 15), (0, 200, 255), -1)
#             cv2.putText(frame, f"Verifying: {self.verify_count}/{VERIFY_FRAMES}",
#                         (bar_x, bar_y - 5),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 100), 1)

#         # Bottom hint
#         if self.state == State.WAITING:
#             hint = "G=GO  Q=Quit"
#         elif self.state == State.STOPPED:
#             hint = "S=Resume  Q=Quit"
#         else:
#             hint = "Q=Quit  (autonomous — stop disabled)"
#         cv2.putText(frame, hint,
#                     (w // 2 - 100, h - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)


# # ══════════════════════════════════════════════════════════════
# # Main
# # ══════════════════════════════════════════════════════════════

# def main():
#     parser = argparse.ArgumentParser(
#         description="PET Bottle Robot — Autonomous Navigator (Pi + ESP32)"
#     )
#     parser.add_argument("--model", default="yolov8",
#                         choices=["yolov5", "yolov7", "yolov8"],
#                         help="YOLO model (default: yolov8)")
#     parser.add_argument("--esp32-ip", default=None,
#                         help="ESP32 WiFi IP address (default: auto-discover)")
#     parser.add_argument("--conf", type=float, default=CONF_THRESHOLD,
#                         help=f"Confidence threshold (default: {CONF_THRESHOLD})")
#     parser.add_argument("--no-show", action="store_true",
#                         help="Headless mode (no display)")
#     parser.add_argument("--output", default="./results",
#                         help="Directory to save snapshots")
#     args = parser.parse_args()

#     out_dir = Path(args.output)
#     out_dir.mkdir(parents=True, exist_ok=True)

#     print(f"\n{'='*60}")
#     print(f"  PET Bottle Robot — Autonomous Navigator")
#     print(f"  Camera + Ultrasonic Fusion (WiFi transport)")
#     print(f"{'='*60}")

#     # Start web stream server (background thread)
#     print(f"  Starting video stream on port 5000...")
#     stream_thread = threading.Thread(target=start_stream_server, args=(5000,), daemon=True)
#     stream_thread.start()
#     print(f"  Stream: http://0.0.0.0:5000/video_feed")

#     # Discover or use provided ESP32 IP (retry forever until found)
#     esp32_ip = args.esp32_ip
#     if not esp32_ip:
#         while True:
#             esp32_ip = discover_esp32()
#             if esp32_ip:
#                 break
#             print("  ESP32 not found — retrying in 10s (waiting for it to join hotspot)...")
#             time.sleep(10)

#     print(f"  Connecting to ESP32 via WiFi ({esp32_ip})...")
#     with stream_lock:
#         stream_stats["esp32_ip"] = esp32_ip
#     esp32 = ESP32WiFiLink(ip=esp32_ip, block=False)

#     # Load YOLO models
#     print(f"  Loading YOLO models...")
#     with VDevice() as target:
#         manager = ModelManager(target)

#         manager.activate(args.model)
#         print(f"  Active: {manager.active_name}")

#         try:
#             nav = Navigator(manager, esp32, args.conf,
#                             args.no_show, out_dir)
#             nav.run()
#         finally:
#             esp32.shutdown()
#             manager.cleanup()


# if __name__ == "__main__":
#     main()
# KINOMMENT KO MUNA LAHAT NG CODE DITO KAHAPON PARA SA BAGONG CODE NGAUN DITO SA BABA


#!/usr/bin/env python3
"""
PET Bottle Robot — Autonomous Navigator (LiDAR + RealSense Edition)
Runs on Raspberry Pi 5. Fuses:
  - LD06 360° LiDAR (UART serial) for obstacle avoidance
  - Intel RealSense D4xx (USB) for YOLO detection + depth-based bottle distance

The robot continuously:
  1. Roams open space looking for PET bottles
  2. Avoids obstacles (LiDAR 360° + camera person/object detection)
  3. Approaches detected bottles (depth camera for accurate distance)
  4. Triggers the pickup sequence on the ESP32
  5. Resumes roaming

Transport: Pi <-> ESP32 is WiFi HTTP. Both devices connect to the same
mobile hotspot. ESP32 is auto-discovered at startup.

Hardware:
  - LD06 LiDAR on UART (/dev/ttyAMA0 or /dev/ttyUSB0) at 230400 baud
  - Intel RealSense D435/D455 on USB 3.0
  - Hailo-8 NPU for YOLO inference (USB)

Usage:
  python3 navigator.py                          # default (YOLOv8)
  python3 navigator.py --model yolov8
  python3 navigator.py --esp32-ip 192.168.43.100
  python3 navigator.py --lidar-port /dev/ttyAMA0
  python3 navigator.py --no-show                # headless
"""

import cv2
import numpy as np
import argparse
import serial
import struct

import time
import threading
import json
from pathlib import Path
from enum import Enum, auto
from flask import Flask, Response, jsonify, request

import urllib.request, urllib.error, urllib.parse

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("WARNING: pyrealsense2 not installed — RealSense camera unavailable")

try:
    from vision_ai import verify_target, analyze_scene, assess_obstacle
    AI_AVAILABLE = True
except ImportError:
    AI_AVAILABLE = False

# Import detection system from camera.py
from camera import (
    ModelManager, MODEL_CONFIGS, MODEL_RANGE,
    DetectionTracker, draw_detections, preprocess,
    postprocess_nms, postprocess_raw, postprocess_petbottle,
    filter_held_bottles, filter_pet_shape, get_orientation, overlay_info,
    CONF_THRESHOLD, COCO_BOTTLE_ID, COCO_PERSON_ID,
)
from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams, FormatType
)

# CRC table for LD06 LiDAR packet validation (from config.py)
CRC_TABLE_LD06 = [
    0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
    0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
    0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
    0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
    0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
    0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
    0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
    0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
    0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
    0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
    0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
    0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
    0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
    0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
    0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
    0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8,
]


# ══════════════════════════════════════════════════════════════
# LD06 LiDAR Driver
# ══════════════════════════════════════════════════════════════

LIDAR_OFFSET_DEG = 270  # mounting offset — LD06 0° to robot forward

# Zone definitions: front/right/back/left mapped to angular ranges (robot frame)
# After applying LIDAR_OFFSET, 0° = robot forward, 90° = robot right, etc.
LIDAR_ZONES = {
    "front":  (-45, 45),     # -45° to +45° centered on forward
    "right":  (45, 135),
    "back":   (135, 225),
    "left":   (225, 315),
}

# Minimum valid range (mm) — below this is noise
LIDAR_MIN_RANGE_MM = 20
# Maximum valid range (mm) — LD06 spec is 12000
LIDAR_MAX_RANGE_MM = 12000


def crc8_ld06(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE_LD06[(crc ^ b) & 0xFF]
    return crc


class LD06LiDAR:
    """LD06 360° LiDAR driver. Reads UART serial in a background thread,
    parses 47-byte packets, and provides zone distances (cm) compatible
    with the original ultrasonic interface (front/right/back/left)."""

    HEADER = 0x54
    PKT_LEN = 47  # 1 header + 1 ver_len + 2 speed + 2 start_angle + 36 data + 2 end_angle + 2 timestamp + 1 crc

    def __init__(self, port="/dev/ttyAMA0", baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self._lock = threading.Lock()
        self._running = False
        self._connected = threading.Event()
        self._scan_points = []  # list of (angle_deg, distance_mm, confidence)
        self._zone_distances = {"front": 999, "right": 999, "back": 999, "left": 999}
        self._last_update = 0.0
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    @property
    def zones(self):
        """Return zone distances in cm (matches old ultrasonic dict format)."""
        with self._lock:
            return dict(self._zone_distances)

    @property
    def zone_min(self):
        z = self.zones
        return min(z["front"], z["right"], z["back"], z["left"])

    @property
    def raw_points(self):
        with self._lock:
            return list(self._scan_points)

    @property
    def is_connected(self):
        return self._connected.is_set()

    @property
    def data_age(self):
        with self._lock:
            if self._last_update == 0.0:
                return float('inf')
            return time.time() - self._last_update

    def _reader_loop(self):
        while self._running:
            try:
                ser = serial.Serial(self.port, self.baudrate, timeout=1)
                print(f"  LiDAR: opened {self.port} @ {self.baudrate}")
                self._connected.set()
                buf = bytearray()

                while self._running:
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf.extend(chunk)

                    while len(buf) >= self.PKT_LEN:
                        # Find header byte
                        idx = buf.find(self.HEADER)
                        if idx < 0:
                            buf.clear()
                            break
                        if idx > 0:
                            del buf[:idx]
                        if len(buf) < self.PKT_LEN:
                            break

                        pkt = bytes(buf[:self.PKT_LEN])
                        del buf[:self.PKT_LEN]

                        # Validate CRC (last byte)
                        if crc8_ld06(pkt[:-1]) != pkt[-1]:
                            continue

                        self._parse_packet(pkt)

            except serial.SerialException as e:
                if self._connected.is_set():
                    self._connected.clear()
                    print(f"  LiDAR: serial error ({e}) — reconnecting...")
                time.sleep(2)
            except Exception as e:
                print(f"  LiDAR: unexpected error ({e})")
                time.sleep(2)

    def _parse_packet(self, pkt):
        """Parse a 47-byte LD06 packet: 12 measurement points."""
        # ver_len = pkt[1]  # 0x2C = version 1, 12 points
        speed = struct.unpack_from('<H', pkt, 2)[0]  # degrees/sec * 100
        start_angle = struct.unpack_from('<H', pkt, 4)[0] / 100.0  # degrees
        end_angle = struct.unpack_from('<H', pkt, 42)[0] / 100.0
        timestamp = struct.unpack_from('<H', pkt, 44)[0]

        # Handle angle wrap-around
        if end_angle < start_angle:
            angle_span = (end_angle + 360.0) - start_angle
        else:
            angle_span = end_angle - start_angle

        points = []
        for i in range(12):
            offset = 6 + i * 3
            dist_mm = struct.unpack_from('<H', pkt, offset)[0]
            confidence = pkt[offset + 2]

            # Interpolate angle for this point
            if angle_span > 0:
                angle = start_angle + (angle_span * i / 11.0)
            else:
                angle = start_angle

            # Apply mounting offset and normalize to 0-360
            angle = (angle + LIDAR_OFFSET_DEG) % 360.0

            # Filter invalid ranges
            if dist_mm < LIDAR_MIN_RANGE_MM or dist_mm > LIDAR_MAX_RANGE_MM:
                continue
            if confidence < 100:
                continue

            points.append((angle, dist_mm, confidence))

        if not points:
            return

        with self._lock:
            self._scan_points.extend(points)
            # Keep last ~2 full rotations (~4000 points at 4500 pts/sec)
            if len(self._scan_points) > 5000:
                self._scan_points = self._scan_points[-4000:]
            self._last_update = time.time()
            self._update_zones()

    def _update_zones(self):
        """Compute minimum distance per zone from recent scan points.
        Uses points from the last 200ms (~1 full rotation at 10Hz spin)."""
        cutoff = time.time() - 0.2
        # We don't have per-point timestamps, so use all buffered points
        # and rely on the rolling buffer being ~200ms worth

        zone_mins = {"front": 99900, "right": 99900, "back": 99900, "left": 99900}

        for angle, dist_mm, conf in self._scan_points[-1200:]:
            for zone_name, (a_start, a_end) in LIDAR_ZONES.items():
                # Normalize angle check for ranges that wrap around 0°
                a_start_n = a_start % 360
                a_end_n = a_end % 360

                if a_start_n < a_end_n:
                    in_zone = a_start_n <= angle < a_end_n
                else:
                    in_zone = angle >= a_start_n or angle < a_end_n

                if in_zone and dist_mm < zone_mins[zone_name]:
                    zone_mins[zone_name] = dist_mm

        # Convert mm to cm, cap at 999
        for zone_name in zone_mins:
            dist_cm = zone_mins[zone_name] / 10.0
            self._zone_distances[zone_name] = min(int(dist_cm), 999)


# ══════════════════════════════════════════════════════════════
# RealSense Camera
# ══════════════════════════════════════════════════════════════

class RealSenseCamera:
    """Intel RealSense D4xx camera driver. Provides aligned color + depth
    frames with post-processing filters for clean depth data."""

    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self._pipeline = None
        self._align = None
        self._depth_scale = 0.001  # default, updated on connect
        self._connected = False

        # Depth post-processing filters (from SLAM robot pipeline)
        self._decimation = rs.decimation_filter()
        self._decimation.set_option(rs.option.filter_magnitude, 2)
        self._spatial = rs.spatial_filter()
        self._spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self._spatial.set_option(rs.option.filter_smooth_delta, 20)
        self._temporal = rs.temporal_filter()
        self._hole_fill = rs.hole_filling_filter()

    def start(self):
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height,
                             rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height,
                             rs.format.z16, self.fps)

        profile = self._pipeline.start(config)

        # Get depth scale (meters per depth unit)
        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale = depth_sensor.get_depth_scale()

        # Align depth to color frame
        self._align = rs.align(rs.stream.color)

        self._connected = True
        print(f"  RealSense: {self.width}x{self.height} @ {self.fps}fps "
              f"(depth_scale={self._depth_scale:.6f})")

    def stop(self):
        if self._pipeline:
            self._pipeline.stop()
            self._connected = False

    @property
    def is_connected(self):
        return self._connected

    def get_frames(self):
        """Return (color_bgr, depth_meters) or (None, None) on failure.
        depth_meters is a float32 array in meters."""
        if not self._pipeline:
            return None, None

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)
        except RuntimeError:
            return None, None

        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        # Apply depth post-processing filters
        depth_frame = self._decimation.process(depth_frame)
        depth_frame = self._spatial.process(depth_frame)
        depth_frame = self._temporal.process(depth_frame)
        depth_frame = self._hole_fill.process(depth_frame)

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_meters = depth_image * self._depth_scale

        # Depth may be smaller after decimation — resize to match color
        if depth_meters.shape[:2] != color_image.shape[:2]:
            depth_meters = cv2.resize(depth_meters,
                                      (color_image.shape[1], color_image.shape[0]),
                                      interpolation=cv2.INTER_NEAREST)

        return color_image, depth_meters

    def get_bottle_depth_cm(self, depth_meters, bbox):
        """Get the distance to a detected bottle in cm using its depth ROI.
        bbox = (x1, y1, x2, y2, conf). Uses 5th percentile of the ROI
        to get the closest surface (the bottle front face)."""
        if depth_meters is None:
            return 999

        x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        h, w = depth_meters.shape[:2]

        # Clamp to frame bounds
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)

        if x2 <= x1 or y2 <= y1:
            return 999

        # Sample center 60% of the bounding box to avoid edge noise
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        rw = int((x2 - x1) * 0.3)
        rh = int((y2 - y1) * 0.3)
        roi = depth_meters[max(0, cy - rh):min(h, cy + rh),
                           max(0, cx - rw):min(w, cx + rw)]

        if roi.size == 0:
            return 999

        # Filter out zero (no depth) and clip unreasonable values
        valid = roi[(roi > 0.05) & (roi < 10.0)]
        if valid.size == 0:
            return 999

        # 5th percentile = closest surface of the bottle
        dist_m = np.percentile(valid, 5)
        return int(dist_m * 100)  # meters to cm


# ══════════════════════════════════════════════════════════════
# Configuration
# ══════════════════════════════════════════════════════════════

ESP32_IP = None  # auto-discovered at startup
ESP32_BASE_URL = None


def discover_esp32(timeout=30):
    """Scan the local network for the ESP32 by probing /sensor on each host.
    ESP32 /sensor no longer has ultrasonic data but still returns limits/speeds/irProx."""
    import socket

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        my_ip = s.getsockname()[0]
    except Exception:
        my_ip = "10.0.0.1"
    finally:
        s.close()

    prefix = my_ip.rsplit(".", 1)[0]
    print(f"  Scanning {prefix}.1-254 for ESP32...")

    def probe(ip):
        try:
            req = urllib.request.urlopen(f"http://{ip}/sensor", timeout=0.5)
            data = req.read().decode("utf-8", errors="replace")
            # Accept any valid JSON with known ESP32 fields
            if "limits" in data or "wheels" in data or "pickup" in data:
                return ip
        except Exception:
            pass
        return None

    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as pool:
        futures = {pool.submit(probe, f"{prefix}.{i}"): i for i in range(1, 255)}
        deadline = time.time() + timeout
        for future in concurrent.futures.as_completed(futures):
            if time.time() > deadline:
                break
            result = future.result()
            if result:
                print(f"  ESP32 found at {result}")
                return result

    print("  WARNING: ESP32 not found on network")
    return None


# Obstacle avoidance thresholds (cm) — same as original
STOP_DIST       = 25
SLOW_DIST       = 40
TURN_DIST       = 60

# Driving speeds
ROAM_SPEED      = 40
SLOW_SPEED      = 30
APPROACH_SPEED  = 40
TURN_SPEED      = 60
SCAN_SPEED      = 60
ALIGN_SPEED     = 35
VERIFY_SPEED    = 30

# Bottle detection & verification
PICKUP_DIST_CM    = 20          # depth camera ≤ 20 cm → close enough to pick
PICKUP_FILL_FALLBACK = 0.25     # camera fallback: bottle fills 25% of frame
BOTTLE_CENTER_TOL = 0.15
VERIFY_FRAMES     = 75
SCAN_STEP_TURN_S  = 0.4
SCAN_PAUSE_S      = 2.5
SCAN_MAX_STEPS    = 15

COCO_OBSTACLE_CLASSES = {
    0: "person", 56: "chair", 57: "couch", 58: "potted plant",
    59: "bed", 60: "dining table", 62: "tv",
}


# ══════════════════════════════════════════════════════════════
# Web Stream (Flask MJPEG server)
# ══════════════════════════════════════════════════════════════

stream_app = Flask(__name__)
stream_lock = threading.Lock()
stream_frame = None
stream_stats = {"fps": 0, "bottles": 0, "persons": 0, "model": "", "inference_ms": 0, "state": "WAITING",
                "lidar": {"front": 999, "right": 999, "back": 999, "left": 999},
                "binFull": False, "pickups": 0, "avoidances": 0, "espConnected": False,
                "esp32_ip": None, "cameraOk": False, "lidarOk": False, "depthOk": False}
_navigator_ref = None
_manual_mode_until = 0.0

data_log = []
data_log_lock = threading.Lock()

def log_event(event_type, details):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    entry = {"time": ts, "event": event_type, **details}
    with data_log_lock:
        data_log.append(entry)

def update_stream(frame_bgr, bottles, persons, model_name, inference_ms, state_name,
                  lidar_zones=None, bin_full=None):
    global stream_frame, stream_stats
    _, jpeg = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
    with stream_lock:
        stream_frame = jpeg.tobytes()
        stream_stats["bottles"] = len(bottles)
        stream_stats["persons"] = len(persons)
        stream_stats["model"] = model_name
        stream_stats["inference_ms"] = round(inference_ms, 1)
        stream_stats["state"] = state_name
        if lidar_zones:
            stream_stats["lidar"] = lidar_zones
        if bin_full is not None:
            stream_stats["binFull"] = bool(bin_full)
        if _navigator_ref:
            stream_stats["pickups"] = _navigator_ref.pickup_success_count
            stream_stats["avoidances"] = getattr(_navigator_ref, 'avoid_count', 0)
            stream_stats["espConnected"] = _navigator_ref.esp32._connected.is_set()
            stream_stats["lidarOk"] = _navigator_ref.lidar.is_connected
            stream_stats["depthOk"] = _navigator_ref.realsense.is_connected

def generate_mjpeg():
    while True:
        with stream_lock:
            frame = stream_frame
        if frame is None:
            time.sleep(0.03)
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)

@stream_app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@stream_app.route('/stats')
def get_stats():
    with stream_lock:
        return jsonify(stream_stats)

@stream_app.route('/esp32_reconnect')
def esp32_reconnect():
    global stream_stats
    ip = request.args.get('ip', '').strip()
    if not ip or not _navigator_ref:
        return jsonify({"ok": False, "error": "no ip or navigator not ready"})
    _navigator_ref.esp32.base_url = f"http://{ip}"
    _navigator_ref.esp32._poll_fails = 0
    with stream_lock:
        stream_stats["esp32_ip"] = ip
    print(f"  ESP32 IP updated to {ip} via /esp32_reconnect")
    return jsonify({"ok": True, "ip": ip})


@stream_app.route('/manual_mode')
def set_manual_mode():
    global _manual_mode_until
    _manual_mode_until = time.time() + 30.0
    if _navigator_ref:
        _navigator_ref._stop_sent_time = time.time()
    return jsonify({"ok": True})

@stream_app.route('/start')
def nav_start():
    if _navigator_ref and _navigator_ref.state in (State.WAITING, State.STOPPED,
                                                     State.MISSION_COMPLETE):
        if not _navigator_ref.esp32._connected.is_set():
            return jsonify({"ok": False, "state": "WAITING", "msg": "ESP32 not connected via WiFi yet"})
        _navigator_ref.pickup_success_count = 0
        _navigator_ref.ir_ever_tripped = False
        _navigator_ref.infer_fail_streak = 0
        _navigator_ref._reset_scan()
        return jsonify({"ok": True, "state": "SCANNING"})
    elif _navigator_ref:
        return jsonify({"ok": False, "state": _navigator_ref.state.name, "msg": "already running"})
    return jsonify({"ok": False, "msg": "navigator not ready"})

@stream_app.route('/stop')
def nav_stop():
    if _navigator_ref:
        _navigator_ref.state = State.WAITING
        _navigator_ref._stop_sent_time = 0
        _navigator_ref.esp32._last_cmd = None
        _navigator_ref.esp32._last_swing_cmd = None
        _navigator_ref.esp32.emergency_stop()
        _navigator_ref.esp32.cmd("PISWS")
        _navigator_ref.esp32.emergency_stop()
        return jsonify({"ok": True, "state": "WAITING"})
    return jsonify({"ok": False, "msg": "navigator not ready"})

@stream_app.route('/esp32_cmd')
def esp32_proxy_cmd():
    c = request.args.get("c", "")
    if not c:
        return jsonify({"ok": False, "error": "no command"})
    if _navigator_ref and _navigator_ref.esp32._connected.is_set():
        _navigator_ref.esp32.cmd(c)
        return jsonify({"ok": True, "resp": "OK"})
    return jsonify({"ok": False, "error": "ESP32 not connected"})

@stream_app.route('/resume')
def nav_resume():
    if _navigator_ref and _navigator_ref.state == State.STOPPED:
        _navigator_ref._reset_scan()
        return jsonify({"ok": True, "state": "SCANNING"})
    elif _navigator_ref:
        return jsonify({"ok": False, "state": _navigator_ref.state.name})
    return jsonify({"ok": False, "msg": "navigator not ready"})

@stream_app.route('/models')
def list_models():
    if _navigator_ref:
        mgr = _navigator_ref.manager
        available = list(mgr.models.keys())
        return jsonify({"ok": True, "models": available, "active": mgr.active_key})
    return jsonify({"ok": False, "msg": "navigator not ready"})

@stream_app.route('/switch_model')
def switch_model():
    key = request.args.get('model', '')
    if not _navigator_ref:
        return jsonify({"ok": False, "msg": "navigator not ready"})
    mgr = _navigator_ref.manager
    if key not in mgr.models:
        return jsonify({"ok": False, "msg": f"unknown model '{key}'", "available": list(mgr.models.keys())})
    mgr.activate(key)
    return jsonify({"ok": True, "active": mgr.active_key, "name": mgr.active_name})

@stream_app.route('/')
def stream_index():
    return R"""<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>PET Bottle Navigator (LiDAR + RealSense)</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:#0a0a1a;color:#eee;padding:12px;
  -webkit-user-select:none;user-select:none}
h1{text-align:center;font-size:1.3rem;margin-bottom:8px;color:#4fc3f7}
.video-wrap{position:relative;text-align:center;margin-bottom:10px}
.video-wrap img{width:100%;max-width:640px;border-radius:12px;border:2px solid #333}
.badge{position:absolute;top:10px;right:10px;padding:6px 14px;border-radius:8px;
  font-weight:700;font-size:.9rem;color:#111}
.wifi-dot{position:absolute;top:10px;left:10px;padding:6px 14px;border-radius:8px;
  font-weight:700;font-size:.75rem}
.sensor-dots{position:absolute;bottom:10px;left:10px;display:flex;gap:8px}
.sdot{padding:4px 10px;border-radius:6px;font-size:.7rem;font-weight:700}
.section{max-width:640px;margin:10px auto}
.controls{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;margin:12px 0}
button{font-size:1rem;font-weight:700;border:none;border-radius:12px;
  padding:12px 24px;cursor:pointer;transition:all .15s;touch-action:manipulation}
button:active{transform:scale(.94)}
.go{background:#66bb6a;color:#111}
.stop{background:#ef5350;color:#fff}
.models{display:flex;gap:8px;justify-content:center;flex-wrap:wrap;margin:8px 0}
.mbtn{background:#1a1a3e;color:#888;border:2px solid #333;padding:10px 20px;border-radius:10px}
.mbtn.active{color:#fff;border-color:#4fc3f7;background:#1a2a4e}
.mbtn.v5{border-color:#ffeb3b}.mbtn.v5.active{color:#ffeb3b;background:#2a2a1e;border-color:#ffeb3b}
.mbtn.v7{border-color:#e040fb}.mbtn.v7.active{color:#e040fb;background:#2a1a2e;border-color:#e040fb}
.mbtn.v8{border-color:#4fc3f7}.mbtn.v8.active{color:#4fc3f7;background:#1a2a4e;border-color:#4fc3f7}
.stats{background:#1a1a2e;border-radius:12px;padding:12px;
  display:grid;grid-template-columns:repeat(auto-fit,minmax(80px,1fr));gap:8px}
.stat{text-align:center}
.stat .val{font-size:1.3rem;font-weight:700;color:#4fc3f7}
.stat .lbl{font-size:.65rem;color:#888;margin-top:2px}
.lidar-bar{background:#1a1a2e;border-radius:12px;padding:12px;margin-top:8px;
  display:grid;grid-template-columns:repeat(4,1fr);gap:8px;text-align:center}
.lidar-bar .val{font-size:1.1rem;font-weight:700}
.lidar-bar .lbl{font-size:.65rem;color:#888}
#log{background:#000;border-radius:8px;padding:8px;margin-top:8px;
  font-family:monospace;font-size:.7rem;color:#4fc3f7;
  height:80px;overflow-y:auto;white-space:pre-wrap}
.dl-btn{background:#1a1a3e;color:#4fc3f7;border:1px solid #333;padding:8px 16px;
  border-radius:8px;font-size:.85rem;margin-top:8px;display:inline-block;text-decoration:none}
.footer{text-align:center;color:#555;font-size:.65rem;margin-top:12px}
</style>
</head>
<body>
<h1>PET Bottle Navigator (LiDAR + RealSense)</h1>

<div class="video-wrap">
  <img src="/video_feed" alt="Live Feed">
  <div class="badge" id="badge" style="background:#ff9800">WAITING</div>
  <div class="wifi-dot" id="wifi" style="background:#ef5350;color:#fff">WiFi: --</div>
  <div class="sensor-dots">
    <div class="sdot" id="lidar-dot" style="background:#ef5350;color:#fff">LiDAR</div>
    <div class="sdot" id="depth-dot" style="background:#ef5350;color:#fff">Depth</div>
  </div>
</div>

<div class="section">
  <div class="controls">
    <button class="go" onclick="act('/start')">START</button>
    <button class="stop" onclick="act('/stop')">STOP</button>
  </div>

  <div class="models">
    <button class="mbtn v5" onclick="switchModel('yolov5')">YOLOv5</button>
    <button class="mbtn v7" onclick="switchModel('yolov7')">YOLOv7</button>
    <button class="mbtn v8 active" onclick="switchModel('yolov8')">YOLOv8</button>
  </div>

  <div class="stats">
    <div class="stat"><div class="val" id="fps">-</div><div class="lbl">FPS</div></div>
    <div class="stat"><div class="val" id="bottles">0</div><div class="lbl">Bottles</div></div>
    <div class="stat"><div class="val" id="persons">0</div><div class="lbl">Persons</div></div>
    <div class="stat"><div class="val" id="infer">-</div><div class="lbl">Infer ms</div></div>
    <div class="stat"><div class="val" id="model">-</div><div class="lbl">Model</div></div>
    <div class="stat"><div class="val" id="bin">-</div><div class="lbl">Bin</div></div>
    <div class="stat"><div class="val" id="pickups">0</div><div class="lbl">Pickups</div></div>
    <div class="stat"><div class="val" id="avoids">0</div><div class="lbl">Avoidances</div></div>
  </div>

  <div class="lidar-bar">
    <div><div class="val" id="lid-f" style="color:#4fc3f7">--</div><div class="lbl">Front (LiDAR)</div></div>
    <div><div class="val" id="lid-r" style="color:#4fc3f7">--</div><div class="lbl">Right</div></div>
    <div><div class="val" id="lid-b" style="color:#4fc3f7">--</div><div class="lbl">Back</div></div>
    <div><div class="val" id="lid-l" style="color:#4fc3f7">--</div><div class="lbl">Left</div></div>
  </div>

  <div id="log">Ready.
</div>
  <div id="cmdlog" style="background:#0a0a2a;border-radius:8px;padding:8px;margin-top:8px;
    font-family:monospace;font-size:.7rem;color:#66bb6a;height:60px;overflow-y:auto;white-space:pre-wrap">Pi -> ESP32 commands:
</div>
  <div style="margin-top:8px;display:flex;gap:8px">
    <a class="dl-btn" href="/data_log" download="navigation_log.csv">Download CSV Log</a>
    <button class="dl-btn" onclick="fetch('/clear_log').then(()=>{log.textContent='Log cleared.\n'})">Clear Log</button>
  </div>
</div>

<div class="footer">PET Bottle Collector Robot (LiDAR + RealSense) &mdash; Emathrix Academy</div>

<script>
const stateColors={WAITING:'#ff9800',SCANNING:'#ffeb3b',ROAMING:'#66bb6a',
  VERIFYING:'#ffcc80',APPROACHING:'#4fc3f7',ALIGNING:'#ffd54f',
  PICKING_UP:'#ab47bc',AVOIDING:'#ef5350',STOPPED:'#666',
  MISSION_COMPLETE:'#ffc107'};
const log=document.getElementById('log');

function act(url){
  fetch(url).then(r=>r.json()).then(d=>{
    log.textContent+='> '+url+' -> '+JSON.stringify(d)+'\n';
    log.scrollTop=log.scrollHeight;
  }).catch(e=>{log.textContent+='ERR: '+e+'\n'});
}

function switchModel(m){
  fetch('/switch_model?model='+m).then(r=>r.json()).then(d=>{
    if(d.ok){
      document.querySelectorAll('.mbtn').forEach(b=>b.classList.remove('active'));
      document.querySelector('.mbtn.'+m.replace('yolo','')).classList.add('active');
      log.textContent+='> Model: '+d.active+'\n';
      log.scrollTop=log.scrollHeight;
    }
  });
}

function zoneColor(v){
  if(v>=999) return '#555';
  if(v<25) return '#ef5350';
  if(v<60) return '#ff9800';
  return '#66bb6a';
}

function poll(){
  fetch('/stats').then(r=>r.json()).then(d=>{
    document.getElementById('fps').textContent=d.fps||'-';
    document.getElementById('bottles').textContent=d.bottles||0;
    document.getElementById('persons').textContent=d.persons||0;
    document.getElementById('infer').textContent=d.inference_ms||'-';
    document.getElementById('model').textContent=d.model||'-';
    document.getElementById('pickups').textContent=d.pickups||0;
    document.getElementById('avoids').textContent=d.avoidances||0;
    const binEl=document.getElementById('bin');
    binEl.textContent=d.binFull?'FULL':'OK';
    binEl.style.color=d.binFull?'#ffc107':'#4fc3f7';
    const b=document.getElementById('badge');
    b.textContent=d.state||'?';
    b.style.background=stateColors[d.state]||'#666';
    if(d.state==='MISSION_COMPLETE') b.textContent='BIN FULL';
    const wifi=document.getElementById('wifi');
    wifi.textContent=d.espConnected?'WiFi: Connected':'WiFi: Disconnected';
    wifi.style.background=d.espConnected?'#66bb6a':'#ef5350';
    // LiDAR zones
    if(d.lidar){
      var z=d.lidar;
      [['f','front'],['r','right'],['b','back'],['l','left']].forEach(function(pair){
        var el=document.getElementById('lid-'+pair[0]);
        var v=z[pair[1]]||999;
        el.textContent=v>=999?'--':v+'cm';
        el.style.color=zoneColor(v);
      });
    }
    // Sensor status dots
    var lidarDot=document.getElementById('lidar-dot');
    lidarDot.style.background=d.lidarOk?'#66bb6a':'#ef5350';
    var depthDot=document.getElementById('depth-dot');
    depthDot.style.background=d.depthOk?'#66bb6a':'#ef5350';
  }).catch(()=>{});
}
function pollCmd(){
  fetch('/cmdlog').then(r=>r.json()).then(d=>{
    if(d.ok&&d.log.length){
      var cl=document.getElementById('cmdlog');
      cl.textContent='Pi -> ESP32:\n';
      d.log.slice(-10).forEach(e=>{
        var c=e.resp.startsWith('OK')?'':'! ';
        cl.textContent+=c+e.time+' '+e.cmd+' -> '+e.resp+'\n';
      });
      cl.scrollTop=cl.scrollHeight;
    }
  }).catch(()=>{});
}
setInterval(poll,500);
setInterval(pollCmd,1000);
poll();
</script>
</body>
</html>"""

@stream_app.route('/cmdlog')
def get_cmdlog():
    if _navigator_ref:
        log = _navigator_ref.esp32.recent_log
        return jsonify({"ok": True, "log": [{"time": t, "cmd": c, "resp": r} for t, c, r in log]})
    return jsonify({"ok": False, "log": []})

@stream_app.route('/data_log')
def download_data_log():
    import csv, io
    si = io.StringIO()
    writer = csv.writer(si)
    writer.writerow(["time", "event", "model", "bottles_detected", "persons_detected",
                     "inference_ms", "bottle_depth_cm", "lidar_front", "lidar_right",
                     "lidar_back", "lidar_left", "state", "detail"])
    with data_log_lock:
        for entry in data_log:
            writer.writerow([
                entry.get("time", ""),
                entry.get("event", ""),
                entry.get("model", ""),
                entry.get("bottles", ""),
                entry.get("persons", ""),
                entry.get("inference_ms", ""),
                entry.get("bottle_depth_cm", ""),
                entry.get("lidar_front", ""),
                entry.get("lidar_right", ""),
                entry.get("lidar_back", ""),
                entry.get("lidar_left", ""),
                entry.get("state", ""),
                entry.get("detail", ""),
            ])
    output = si.getvalue()
    return Response(output, mimetype="text/csv",
                    headers={"Content-Disposition": "attachment;filename=navigation_log.csv"})

@stream_app.route('/clear_log')
def clear_log():
    with data_log_lock:
        data_log.clear()
    return jsonify({"ok": True, "msg": "log cleared"})

def start_stream_server(port=5000):
    stream_app.run(host='0.0.0.0', port=port, threaded=True, use_reloader=False)


# ══════════════════════════════════════════════════════════════
# Robot States
# ══════════════════════════════════════════════════════════════

class State(Enum):
    WAITING          = auto()
    SCANNING         = auto()
    ROAMING          = auto()
    VERIFYING        = auto()
    APPROACHING      = auto()
    ALIGNING         = auto()
    PICKING_UP       = auto()
    AVOIDING         = auto()
    STOPPED          = auto()
    MISSION_COMPLETE = auto()


BIN_SETTLE_DELAY_S   = 1.5
BIN_DEBOUNCE_SAMPLES = 3
BIN_DEBOUNCE_PERIOD  = 0.25
MAX_PICKUPS_BACKSTOP = 20
SENSOR_STALE_S = 10.0
INFER_FAIL_LIMIT = 15


# ══════════════════════════════════════════════════════════════
# ESP32 Communication — WiFi HTTP transport
# ══════════════════════════════════════════════════════════════

class ESP32WiFiLink:
    """Talks to the ESP32 over WiFi HTTP.
    Sends commands via GET /cmd?c=<CMD> and polls sensor data
    via GET /sensor every 200ms in a background thread.
    NOTE: ESP32 no longer provides ultrasonic data — LiDAR handles obstacle detection.
    ESP32 still provides: limits, wheels, pickup state, irProx."""

    MAX_LOG = 20

    def __init__(self, ip=ESP32_IP, block=False):
        self.base_url = f"http://{ip}"
        self.sensor_data = {}
        self._sensor_data_ts = 0.0
        self.cmd_log = []
        self._last_cmd = None
        self._lock = threading.Lock()
        self._running = True
        self._connected = threading.Event()
        self._poll_thread = threading.Thread(target=self._poll_sensors, daemon=True)
        self._poll_thread.start()
        if block:
            if not self._connected.wait(timeout=15):
                print(f"  WARNING: ESP32 not responding at {ip} — running camera-only mode")
        else:
            print(f"  WiFi connecting in background — polling {self.base_url}/sensor")

    def _poll_sensors(self):
        _last_rediscover = 0.0
        _rediscovering = False
        _rediscover_lock = threading.Lock()

        def _do_rediscover():
            nonlocal _rediscovering
            try:
                new_ip = discover_esp32()
                if new_ip:
                    new_url = f"http://{new_ip}"
                    if new_url != self.base_url:
                        print(f"  ESP32 found at new IP: {new_ip} (was {self.base_url})")
                        self.base_url = new_url
                        with stream_lock:
                            stream_stats["esp32_ip"] = new_ip
                    self._poll_fails = 0
            finally:
                with _rediscover_lock:
                    _rediscovering = False

        while self._running:
            try:
                req = urllib.request.urlopen(f"{self.base_url}/sensor", timeout=2)
                raw = req.read().decode("utf-8", errors="replace")
                parsed = json.loads(raw)
                with self._lock:
                    self.sensor_data = parsed
                    self._sensor_data_ts = time.time()
                self._poll_fails = 0
                if not self._connected.is_set():
                    self._connected.set()
                    print(f"  WiFi connected to ESP32 at {self.base_url}")
            except Exception:
                self._poll_fails = getattr(self, '_poll_fails', 0) + 1
                if self._poll_fails >= 5:
                    with self._lock:
                        self._sensor_data_ts = 0.0
                    if self._connected.is_set():
                        self._connected.clear()
                        print(f"  WiFi lost connection to ESP32")
                    now = time.time()
                    with _rediscover_lock:
                        if not _rediscovering and now - _last_rediscover >= 15.0:
                            _rediscovering = True
                            _last_rediscover = now
                            threading.Thread(target=_do_rediscover, daemon=True).start()
            time.sleep(0.2)

    def _log(self, command, response):
        if command == self._last_cmd and command not in ("PIX", "PISTOP", "P", "PA"):
            return
        self._last_cmd = command
        ts = time.strftime("%H:%M:%S")
        with self._lock:
            self.cmd_log.append((ts, command, response))
            if len(self.cmd_log) > self.MAX_LOG:
                self.cmd_log.pop(0)

    @property
    def recent_log(self):
        with self._lock:
            return list(self.cmd_log)

    @property
    def sensors(self):
        with self._lock:
            return dict(self.sensor_data)

    @property
    def is_connected(self):
        with self._lock:
            return bool(self.sensor_data)

    def sensor_age(self):
        with self._lock:
            if self._sensor_data_ts == 0.0:
                return float('inf')
            return time.time() - self._sensor_data_ts

    def cmd(self, command):
        now = time.time()
        no_dedup = command in ("PIX", "PISTOP", "PA", "PISWS")
        is_swing = command in ("PISWL", "PISWR")
        if is_swing:
            if command == getattr(self, '_last_swing_cmd', None) and now - getattr(self, '_last_swing_time', 0) < 2.0:
                return
            self._last_swing_cmd = command
            self._last_swing_time = now
        elif not no_dedup and command == self._last_cmd and now - getattr(self, '_last_cmd_time', 0) < 0.5:
            return
        self._last_cmd = command
        self._last_cmd_time = now
        try:
            url = f"{self.base_url}/cmd?c={urllib.parse.quote(command)}"
            req = urllib.request.urlopen(url, timeout=2)
            resp = req.read().decode("utf-8", errors="replace").strip()
            self._log(command, resp[:20] if resp else "OK")
        except Exception as e:
            self._log(command, f"ERR: {e}")

    def forward(self, speed=ROAM_SPEED):
        self.cmd(f"PIFW{speed}")

    def backward(self, speed=ROAM_SPEED):
        self.cmd(f"PIBW{speed}")

    def turn_left(self, speed=TURN_SPEED):
        self.cmd(f"PITL{speed}")

    def turn_right(self, speed=TURN_SPEED):
        self.cmd(f"PITR{speed}")

    def differential(self, left_speed, right_speed):
        self.cmd(f"PIDW{left_speed},{right_speed}")

    def stop(self):
        self.cmd("PIX")

    def emergency_stop(self):
        self.cmd("PISTOP")

    def swing_left(self):
        self.cmd("PISWL")

    def swing_right(self):
        self.cmd("PISWR")

    def swing_stop(self):
        self.cmd("PISWS")

    def pickup(self):
        self.cmd("P")

    def shutdown(self):
        self._running = False
        try:
            self.stop()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════
# Autonomous Navigator
# ══════════════════════════════════════════════════════════════

class Navigator:
    """
    Main brain: fuses RealSense camera (YOLO + depth) + LD06 LiDAR for
    real-time navigation. Same state machine as original — roam, detect,
    approach, pick up, resume.
    """

    def __init__(self, manager, esp32, lidar, realsense, conf_thresh, no_show, out_dir):
        global _navigator_ref
        self.manager     = manager
        self.esp32       = esp32
        self.lidar       = lidar
        self.realsense   = realsense
        self.conf_thresh = conf_thresh
        self.no_show     = no_show
        self.out_dir     = out_dir

        self.state       = State.WAITING
        _navigator_ref   = self
        self.tracker     = DetectionTracker()
        self.turn_dir    = 1
        self.turn_timer  = 0.0
        self.avoid_timer = 0.0
        self.scan_start  = 0.0
        self.scan_step   = 0
        self.scan_phase  = "turn"
        self.scan_phase_start = 0.0
        self.verify_count = 0
        self.verify_lost  = 0
        self.approach_lost_count = 0
        self.pickup_success_count = 0
        self.avoid_count          = 0
        self.ir_ever_tripped      = False
        self.infer_fail_streak    = 0
        self._current_frame       = None
        self._current_depth       = None  # depth frame (meters) for bottle distance
        self._ai_verified         = False
        self._ai_scene_time       = 0.0
        self._ai_scene_interval   = 8.0

    def run(self):
        """Main loop — RealSense camera + LiDAR + navigation."""
        if not self.no_show:
            cv2.namedWindow("PET Bottle Navigator", cv2.WINDOW_NORMAL)

        print(f"\n{'='*60}")
        print(f"  PET Bottle Robot — AUTONOMOUS NAVIGATOR")
        print(f"  RealSense Depth Camera + LD06 LiDAR Fusion")
        print(f"  ESP32: WiFi ({ESP32_BASE_URL})")
        print(f"  SAFETY MODE: Motors DISABLED")
        print(f"  Press G to GO (enable motors) | Q to quit")
        print(f"{'='*60}\n")

        self.esp32.stop()

        frame_idx = 0

        try:
            while True:
                # ── Grab RealSense frames ────────────────────
                frame, depth_m = self.realsense.get_frames()
                if frame is None:
                    print("\n  RealSense: frame lost — retrying...")
                    with stream_lock:
                        stream_stats["cameraOk"] = False
                    time.sleep(0.1)
                    continue

                with stream_lock:
                    stream_stats["cameraOk"] = True

                orig_h, orig_w = frame.shape[:2]
                frame_area = orig_w * orig_h

                # ── Run YOLO detection ───────────────────────
                t0 = time.time()
                try:
                    bottles, persons = self.manager.infer(
                        frame, self.conf_thresh, apply_range_filter=False)
                    self.infer_fail_streak = 0
                except Exception as e:
                    bottles, persons = [], []
                    self.infer_fail_streak += 1
                    if self.infer_fail_streak % 50 == 1:
                        print(f"\n  >>> INFER ERROR ({self.infer_fail_streak}): {e}")
                ms = (time.time() - t0) * 1000

                smoothed = self.tracker.update(bottles)

                # ── Get LiDAR zone distances ─────────────────
                lz = self.lidar.zones  # {front, right, back, left} in cm

                # ── Get bottle depth from RealSense ──────────
                bottle_depth_cm = 999
                if smoothed and depth_m is not None:
                    best = max(smoothed, key=lambda b: (b[2]-b[0])*(b[3]-b[1]))
                    bottle_depth_cm = self.realsense.get_bottle_depth_cm(depth_m, best)

                # ── Sensor freshness (log only) ──────────────
                lidar_age = self.lidar.data_age
                if lidar_age > SENSOR_STALE_S and frame_idx % 100 == 0:
                    print(f"\n  >>> LIDAR DATA AGE: {lidar_age:.1f}s (LiDAR may be disconnected)")

                esp_age = self.esp32.sensor_age()
                if esp_age > SENSOR_STALE_S and frame_idx % 100 == 0:
                    print(f"\n  >>> ESP32 DATA AGE: {esp_age:.1f}s (ESP32 may be slow)")

                # ── Store frames for AI vision ───────────────
                self._current_frame = frame.copy()
                self._current_depth = depth_m

                # ── Navigation decision ──────────────────────
                self._navigate(smoothed, persons, orig_w, orig_h,
                               frame_area, lz, bottle_depth_cm)

                # ── Data logging (1 Hz) ──────────────────────
                if frame_idx % 30 == 0 and self.state not in (State.WAITING, State.STOPPED):
                    log_event("detection", {
                        "model": self.manager.active_name,
                        "bottles": len(smoothed),
                        "persons": len(persons),
                        "inference_ms": round(ms, 1),
                        "bottle_depth_cm": bottle_depth_cm if bottle_depth_cm < 999 else "",
                        "lidar_front": lz["front"], "lidar_right": lz["right"],
                        "lidar_back": lz["back"], "lidar_left": lz["left"],
                        "state": self.state.name,
                    })

                # ── Draw results ─────────────────────────────
                result = draw_detections(frame.copy(), smoothed, persons)
                self._draw_nav_overlay(result, smoothed, lz, bottle_depth_cm, ms, frame_idx)

                # ── Update web stream ────────────────────────
                model_name = self.manager.active_name
                bin_full = self.esp32.sensors.get("irProx", False)
                update_stream(result, smoothed, persons, model_name, ms,
                              self.state.name, lz, bin_full=bin_full)

                # ── FPS counter ──────────────────────────────
                self._fps_count = getattr(self, '_fps_count', 0) + 1
                if not hasattr(self, '_fps_timer'):
                    self._fps_timer = time.time()
                if time.time() - self._fps_timer >= 1.0:
                    with stream_lock:
                        stream_stats["fps"] = self._fps_count
                    self._fps_count = 0
                    self._fps_timer = time.time()

                # ── Console output ───────────────────────────
                icon = "OK" if smoothed else "--"
                depth_str = f"D:{bottle_depth_cm}cm" if bottle_depth_cm < 999 else "D:---"
                print(f"\r  [{icon}] {self.state.name:12s} "
                      f"LiDAR F:{lz['front']:3d} R:{lz['right']:3d} "
                      f"B:{lz['back']:3d} L:{lz['left']:3d} | "
                      f"{depth_str} | {len(smoothed)} det | {ms:.0f}ms   ",
                      end="", flush=True)

                # ── Display + keyboard ───────────────────────
                if not self.no_show:
                    cv2.imshow("PET Bottle Navigator", result)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), ord('Q'), 27):
                        break
                    elif key in (ord('g'), ord('G')):
                        if self.state == State.WAITING:
                            if not self.esp32._connected.is_set():
                                print("\n  >>> WAITING for ESP32 WiFi connection...")
                            else:
                                self._reset_scan()
                                print("\n  >>> MOTORS ENABLED — SCANNING (slow look-around)")
                    elif key in (ord('s'), ord('S')):
                        if self.state == State.STOPPED:
                            self.state = State.ROAMING
                            print("\n  >>> RESUMED")
                else:
                    pass

                frame_idx += 1

        finally:
            self.esp32.stop()
            self.realsense.stop()
            self.lidar.stop()
            if not self.no_show:
                cv2.destroyAllWindows()
            print()

    # ── Navigation logic ──────────────────────────────────────

    def _navigate(self, bottles, persons, w, h, frame_area, lz, bottle_depth_cm):
        """State machine: same logic as original but using LiDAR zones
        instead of ultrasonic sensors, and depth camera for bottle distance.
        lz = dict with keys front/right/back/left in cm.
        bottle_depth_cm = RealSense depth to closest detected bottle in cm."""

        if self.state == State.WAITING:
            now = time.time()
            if now < _manual_mode_until:
                return
            if now - getattr(self, '_stop_sent_time', 0) > 30.0:
                self._stop_sent_time = now
                self.esp32._last_cmd = None
                self.esp32.emergency_stop()
                self.esp32.swing_stop()
            return

        if self.state == State.STOPPED:
            now = time.time()
            if now < _manual_mode_until:
                return
            if now - getattr(self, '_stop_sent_time', 0) > 30.0:
                self._stop_sent_time = now
                self.esp32._last_cmd = None
                self.esp32.emergency_stop()
                self.esp32.swing_stop()
            return

        if self.state == State.MISSION_COMPLETE:
            return

        if self.state == State.PICKING_UP:
            return

        lid_front = lz["front"]
        lid_right = lz["right"]
        lid_back  = lz["back"]
        lid_left  = lz["left"]
        lid_min_forward = min(lid_front, lid_left, lid_right)

        # ── Avoiding state: back up and turn away ─────────
        if self.state == State.AVOIDING:
            elapsed = time.time() - self.avoid_timer
            if elapsed < 1.5:
                if lid_back > STOP_DIST:
                    self.esp32.backward(SLOW_SPEED)
                else:
                    if lid_left < lid_right:
                        self.esp32.turn_right(SCAN_SPEED)
                    else:
                        self.esp32.turn_left(SCAN_SPEED)
            elif elapsed < 3.0:
                if lid_left < lid_right:
                    self.esp32.turn_right(SCAN_SPEED)
                else:
                    self.esp32.turn_left(SCAN_SPEED)
            elif elapsed < 3.4:
                self.esp32.stop()
            else:
                if lid_front < STOP_DIST:
                    print(f"\n  >>> AVOIDING re-check failed "
                          f"(F={lid_front} min_fwd={lid_min_forward}) "
                          f"— restarting avoid")
                    self.avoid_timer = time.time()
                else:
                    self.esp32.stop()
                    self._reset_scan()
            return

        # ── PERSON DETECTED → steer away ─────────────────
        for p in persons:
            pw = p[2] - p[0]
            ph = p[3] - p[1]
            p_fill = (pw * ph) / frame_area
            if p_fill > 0.25:
                self.esp32.stop()
                self._enter_avoiding("PERSON DETECTED")
                return

        # ── Emergency stop: LiDAR front too close ────────
        _pursuing = self.state in (State.VERIFYING, State.APPROACHING,
                                   State.ALIGNING)
        _bottle_in_sight = len(bottles) > 0
        if lid_front < STOP_DIST and not _pursuing:
            if _bottle_in_sight and self.state in (State.SCANNING, State.ROAMING):
                self.esp32.stop()
                self.state = State.VERIFYING
                self.verify_count = 1
                self.verify_lost = 0
                print("\n  >>> Bottle within reach (LiDAR triggered) — starting verification")
                return
            self.esp32.stop()
            self._enter_avoiding(f"FRONT < {STOP_DIST}cm (LiDAR)")
            return

        # ── LiDAR: slow zone → steer away gently (only while ROAMING) ──
        if lid_min_forward < TURN_DIST and not _pursuing and self.state == State.ROAMING:
            if not _bottle_in_sight:
                if lid_left < lid_right:
                    self.esp32.turn_right(SCAN_SPEED)
                else:
                    self.esp32.turn_left(SCAN_SPEED)
                return

        # ── SCANNING: step-scan — turn ~25°, pause 2.5s, repeat ──
        if self.state == State.SCANNING:
            now = time.time()

            if bottles:
                self.esp32.stop()
                self.state = State.VERIFYING
                self.verify_count = 1
                self.verify_lost = 0
                print("\n  >>> Possible bottle spotted — verifying...")
                return

            if self.scan_step >= SCAN_MAX_STEPS:
                self.esp32.stop()
                if AI_AVAILABLE and (now - self._ai_scene_time) > self._ai_scene_interval:
                    self._ai_scene_time = now
                    sensor = self.esp32.sensors
                    def _ai_scan():
                        result = analyze_scene(self._current_frame, sensor)
                        if self.state == State.WAITING:
                            return
                        action = result.get("action", "go_forward")
                        reason = result.get("reason", "")
                        spotted = result.get("bottles_spotted", 0)
                        print(f"\n  >>> [AI] {action}: {reason} (bottles_spotted={spotted})")
                        if self.state == State.WAITING:
                            return
                        if spotted > 0:
                            self._reset_scan()
                        elif action == "turn_left":
                            self.esp32.turn_left(SCAN_SPEED)
                            time.sleep(1.5)
                            if self.state == State.WAITING:
                                return
                            self.esp32.stop()
                            self._reset_scan()
                        elif action == "turn_right":
                            self.esp32.turn_right(SCAN_SPEED)
                            time.sleep(1.5)
                            if self.state == State.WAITING:
                                return
                            self.esp32.stop()
                            self._reset_scan()
                        else:
                            self.state = State.ROAMING
                            self.turn_timer = time.time()
                    threading.Thread(target=_ai_scan, daemon=True).start()
                else:
                    self.state = State.ROAMING
                    self.turn_timer = time.time()
                self.turn_dir *= -1
                return

            phase_elapsed = now - self.scan_phase_start

            if self.scan_phase == "turn":
                if phase_elapsed < SCAN_STEP_TURN_S:
                    if self.turn_dir > 0:
                        self.esp32.turn_right(SCAN_SPEED)
                    else:
                        self.esp32.turn_left(SCAN_SPEED)
                else:
                    self.esp32.stop()
                    self.scan_phase = "pause"
                    self.scan_phase_start = now
            elif self.scan_phase == "pause":
                if phase_elapsed >= SCAN_PAUSE_S:
                    self.scan_step += 1
                    self.scan_phase = "turn"
                    self.scan_phase_start = now
            return

        # ── VERIFYING: stay still or creep closer to confirm bottle ──
        if self.state == State.VERIFYING:
            if bottles:
                self.verify_count += 1
                self.verify_lost = 0

                best = max(bottles, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
                bx1, by1, bx2, by2, bconf = best
                bw = bx2 - bx1
                bh = by2 - by1
                b_fill = (bw * bh) / frame_area

                if self.verify_count >= VERIFY_FRAMES:
                    b_cx_norm = ((bx1 + bx2) / 2.0) / w
                    # Use depth camera for distance instead of ultrasonic
                    close_by_depth = bottle_depth_cm <= PICKUP_DIST_CM
                    close_by_cam = b_fill >= PICKUP_FILL_FALLBACK
                    if ((close_by_depth or close_by_cam)
                            and abs(b_cx_norm - 0.5) < BOTTLE_CENTER_TOL):
                        trigger = f"DEPTH={bottle_depth_cm}cm" if close_by_depth else f"CAM={b_fill:.0%}"
                        print(f"\n  >>> Bottle CONFIRMED & in range ({trigger}) — starting pickup")
                        self._start_pickup()
                    else:
                        print(f"\n  >>> Bottle CONFIRMED ({self.verify_count} frames, "
                              f"depth={bottle_depth_cm}cm) — approaching")
                        self.state = State.APPROACHING
                        self.approach_lost_count = 0
                    return
                elif b_fill < 0.03:
                    self.esp32.forward(VERIFY_SPEED)
                else:
                    self.esp32.stop()
            else:
                self.verify_lost += 1
                self.esp32.stop()
                if self.verify_lost > 20:
                    print("\n  >>> False alarm — resuming scan")
                    self._reset_scan()
                    self.verify_count = 0
            return

        # ── APPROACHING: bottle confirmed, move towards it ───
        if self.state == State.APPROACHING:
            if bottles:
                self.approach_lost_count = 0
                best = max(bottles, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
                bx1, by1, bx2, by2, bconf = best
                bw = bx2 - bx1
                bh = by2 - by1
                b_fill = (bw * bh) / frame_area
                b_cx = (bx1 + bx2) / 2.0
                b_cx_norm = b_cx / w

                close_by_depth = bottle_depth_cm <= PICKUP_DIST_CM
                close_by_cam = b_fill >= PICKUP_FILL_FALLBACK
                if close_by_depth or close_by_cam:
                    if abs(b_cx_norm - 0.5) < BOTTLE_CENTER_TOL:
                        self._start_pickup()
                    else:
                        self.state = State.ALIGNING
                        if b_cx_norm < 0.5:
                            self.esp32.turn_left(ALIGN_SPEED)
                        else:
                            self.esp32.turn_right(ALIGN_SPEED)
                else:
                    if abs(b_cx_norm - 0.5) > 0.25:
                        if b_cx_norm < 0.5:
                            self.esp32.differential(VERIFY_SPEED, APPROACH_SPEED)
                        else:
                            self.esp32.differential(APPROACH_SPEED, VERIFY_SPEED)
                    else:
                        self.esp32.forward(APPROACH_SPEED)
            else:
                # Camera lost bottle — use depth + LiDAR as fallback
                if bottle_depth_cm <= PICKUP_DIST_CM:
                    print(f"\n  >>> Camera lost bottle but DEPTH={bottle_depth_cm}cm — starting pickup")
                    self._start_pickup()
                elif lid_front <= SLOW_DIST:
                    self.approach_lost_count = 0
                    self.esp32.forward(SLOW_SPEED)
                else:
                    self.approach_lost_count += 1
                    self.esp32.stop()
                    if self.approach_lost_count > 45:
                        print("\n  >>> Lost bottle — scanning again")
                        self._reset_scan()
            return

        # ── ALIGNING: fine-tune position before pickup ────
        if self.state == State.ALIGNING:
            if bottles:
                best = max(bottles, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
                bx1, by1, bx2, by2, bconf = best
                b_cx_norm = ((bx1 + bx2) / 2.0) / w
                if abs(b_cx_norm - 0.5) < BOTTLE_CENTER_TOL:
                    self._start_pickup()
                else:
                    if b_cx_norm < 0.5:
                        self.esp32.turn_left(ALIGN_SPEED)
                    else:
                        self.esp32.turn_right(ALIGN_SPEED)
            else:
                self.esp32.stop()
                self._reset_scan()
            return

        # ── ROAMING: move forward, then scan again ────────
        if self.state == State.ROAMING:
            now = time.time()
            elapsed = now - self.turn_timer

            if bottles:
                self.esp32.stop()
                self.state = State.VERIFYING
                self.verify_count = 1
                self.verify_lost = 0
                print("\n  >>> Possible bottle spotted — verifying...")
                return

            if elapsed < 5.0:
                speed = ROAM_SPEED
                if lid_min_forward < SLOW_DIST:
                    speed = SLOW_SPEED
                self.esp32.forward(speed)
            else:
                self.esp32.stop()
                self._reset_scan()

    def _reset_scan(self):
        self.state = State.SCANNING
        self.scan_start = time.time()
        self.scan_step = 0
        self.scan_phase = "turn"
        self.scan_phase_start = time.time()

    def _start_pickup(self):
        self.state = State.PICKING_UP
        self.esp32.emergency_stop()
        self.esp32.emergency_stop()

        def _do_pickup():
            time.sleep(1.0)
            print("\n  >>> PICKUP SEQUENCE STARTED")
            self.esp32.pickup()
            time.sleep(2.0)

            timeout = time.time() + 50
            pickup_ok = False
            sequence_started = False
            while time.time() < timeout:
                if self.state == State.WAITING:
                    print("\n  >>> PICKUP ABORTED — user pressed STOP")
                    self.esp32.cmd("PA")
                    return
                sensor = self.esp32.sensors
                pu_state = sensor.get("pickup", "idle")
                if not sequence_started:
                    if pu_state != "idle":
                        sequence_started = True
                        print(f"\n  >>> Pickup sequence active ({pu_state})")
                else:
                    if pu_state in ("idle", "done"):
                        pickup_ok = True
                        break
                time.sleep(0.5)
            else:
                print("\n  >>> PICKUP TIMEOUT waiting for ESP32 — aborting")
                self.esp32.cmd("PA")
                time.sleep(1)

            if self.state == State.WAITING:
                print("\n  >>> PICKUP thread respecting STOP — not resuming")
                return

            if pickup_ok:
                self.pickup_success_count += 1

                if self.pickup_success_count >= MAX_PICKUPS_BACKSTOP:
                    print(f"\n  >>> BACKSTOP HIT ({MAX_PICKUPS_BACKSTOP} pickups) — "
                          f"MISSION COMPLETE (IR sensor may be faulty: "
                          f"ever_tripped={self.ir_ever_tripped})")
                    self.esp32.stop()
                    self.esp32.cmd("PISTOP")
                    self.tracker = DetectionTracker()
                    self.state = State.MISSION_COMPLETE
                    return

                if self._is_bin_full():
                    print("\n  >>> BIN FULL (IR proximity tripped) — MISSION COMPLETE")
                    self.esp32.stop()
                    self.esp32.cmd("PISTOP")
                    self.tracker = DetectionTracker()
                    self.state = State.MISSION_COMPLETE
                    return

                if self.pickup_success_count >= 5 and not self.ir_ever_tripped:
                    print(f"\n  >>> WARNING: {self.pickup_success_count} pickups "
                          f"and IR sensor has never read true — check E18-D80NK "
                          f"wiring/aim")

            log_event("pickup", {
                "detail": f"pickup #{self.pickup_success_count}" if pickup_ok else "pickup failed/timeout",
                "state": "PICKING_UP",
                "model": self.manager.active_name,
            })

            if self.state == State.WAITING:
                print("\n  >>> PICKUP thread respecting STOP — not resuming")
                return

            print(f"\n  >>> PICKUP DONE ({self.pickup_success_count}) — "
                  f"scanning for next bottle")
            self.tracker = DetectionTracker()
            self._reset_scan()

        threading.Thread(target=_do_pickup, daemon=True).start()

    def _enter_avoiding(self, reason):
        self.state = State.AVOIDING
        self.avoid_timer = time.time()
        self.verify_count = 0
        self.verify_lost = 0
        self.approach_lost_count = 0
        self.esp32.stop()
        self.avoid_count += 1
        lz = self.lidar.zones
        log_event("avoidance", {
            "detail": reason, "state": "AVOIDING",
            "model": self.manager.active_name,
            "lidar_front": lz["front"], "lidar_right": lz["right"],
            "lidar_back": lz["back"], "lidar_left": lz["left"],
        })
        print(f"\n  >>> AVOIDING ({reason})")
        if AI_AVAILABLE and self._current_frame is not None:
            sensor = self.esp32.sensors
            def _ai_assess():
                result = assess_obstacle(self._current_frame, sensor)
                obstacle = result.get("obstacle", "unknown")
                direction = result.get("avoid_direction", "right")
                ai_reason = result.get("reason", "")
                print(f"\n  >>> [AI] obstacle={obstacle}, avoid={direction}: {ai_reason}")
            threading.Thread(target=_ai_assess, daemon=True).start()

    def _is_bin_full(self):
        time.sleep(BIN_SETTLE_DELAY_S)
        all_true = True
        for _ in range(BIN_DEBOUNCE_SAMPLES):
            sample = bool(self.esp32.sensors.get("irProx", False))
            if sample:
                self.ir_ever_tripped = True
            else:
                all_true = False
            time.sleep(BIN_DEBOUNCE_PERIOD)
        return all_true

    # ── HUD overlay ───────────────────────────────────────

    def _draw_nav_overlay(self, frame, bottles, lz, bottle_depth_cm, ms, frame_idx):
        """Draw navigation HUD on frame.
        lz = dict with keys front/right/back/left in cm.
        bottle_depth_cm = depth camera distance to nearest bottle."""
        h, w = frame.shape[:2]

        # State badge (top-right)
        state_colors = {
            State.WAITING:          (0, 165, 255),
            State.SCANNING:         (255, 255, 0),
            State.ROAMING:          (0, 255, 0),
            State.VERIFYING:        (255, 200, 100),
            State.APPROACHING:      (0, 200, 255),
            State.ALIGNING:         (255, 200, 0),
            State.PICKING_UP:       (255, 0, 255),
            State.AVOIDING:         (0, 0, 255),
            State.STOPPED:          (80, 80, 80),
            State.MISSION_COMPLETE: (0, 215, 255),
        }
        color = state_colors.get(self.state, (255, 255, 255))
        badge = f"[{self.state.name}]"
        (tw, th), _ = cv2.getTextSize(badge, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.rectangle(frame, (w - tw - 20, 0), (w, th + 16), color, -1)
        cv2.putText(frame, badge, (w - tw - 10, th + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        # LiDAR distance bars (replacing ultrasonic bars)
        bar_max_h = 100
        bar_w = 30
        margin = 10
        bar_positions = [
            ("F", lz["front"], margin),
            ("L", lz["left"], margin + bar_w + 6),
            ("R", lz["right"], w - bar_w - margin),
            ("B", lz["back"], w - 2 * bar_w - margin - 6),
        ]

        for side, dist, x_pos in bar_positions:
            fill_h = int(min(dist, 200) / 200.0 * bar_max_h)
            bar_color = (0, 255, 0) if dist > SLOW_DIST else (
                (0, 200, 255) if dist > STOP_DIST else (0, 0, 255)
            )
            bar_top = h - margin - bar_max_h
            cv2.rectangle(frame, (x_pos, bar_top),
                          (x_pos + bar_w, h - margin), (40, 40, 40), -1)
            cv2.rectangle(frame, (x_pos, bar_top + (bar_max_h - fill_h)),
                          (x_pos + bar_w, h - margin), bar_color, -1)
            cv2.putText(frame, f"{side}:{dist}",
                        (x_pos, bar_top - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # Detection count + FPS (top-left)
        status = f"BOTTLES: {len(bottles)}" if bottles else "SCANNING..."
        det_color = (0, 255, 0) if bottles else (0, 200, 255)
        cv2.putText(frame, status, (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, det_color, 2)
        if ms > 0:
            cv2.putText(frame, f"{ms:.0f}ms  {1000/ms:.0f}FPS",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (200, 200, 200), 1)

        # Bottle depth display (below FPS)
        if bottle_depth_cm < 999:
            depth_color = (0, 255, 0) if bottle_depth_cm > PICKUP_DIST_CM else (0, 200, 255)
            cv2.putText(frame, f"Bottle depth: {bottle_depth_cm} cm",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        depth_color, 2)

        # Sensor data (ESP32 wheels/limits)
        sensor = self.esp32.sensors
        wheels = sensor.get("wheels", {})
        cv2.putText(frame, f"Wheels L:{wheels.get('left',0)} R:{wheels.get('right',0)}"
                    f"  Lift:{sensor.get('lift',0)}  Base:{sensor.get('base',0)}",
                    (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        # LiDAR + Depth status indicators
        lidar_ok = self.lidar.is_connected
        depth_ok = self.realsense.is_connected
        cv2.putText(frame, f"LiDAR:{'OK' if lidar_ok else 'OFF'}  Depth:{'OK' if depth_ok else 'OFF'}",
                    (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 0) if (lidar_ok and depth_ok) else (0, 0, 255), 1)

        # Command log panel (right side)
        log = self.esp32.recent_log
        if log:
            panel_x = w - 280
            panel_y = 40
            panel_h = min(len(log), 10) * 18 + 10
            overlay = frame.copy()
            cv2.rectangle(overlay, (panel_x - 5, panel_y - 5),
                          (w - 5, panel_y + panel_h), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
            cv2.putText(frame, "CMD LOG (Pi -> ESP32)",
                        (panel_x, panel_y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
            for i, (ts, cmd, resp) in enumerate(log[-10:]):
                y = panel_y + 25 + i * 18
                resp_color = (0, 255, 0) if resp.startswith("OK") else (0, 0, 255)
                cv2.putText(frame, f"{ts} {cmd:8s} -> {resp}",
                            (panel_x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.33, resp_color, 1)

        # Verify progress bar
        if self.state == State.VERIFYING:
            progress = min(self.verify_count / VERIFY_FRAMES, 1.0)
            bar_x, bar_y = w // 2 - 100, h - 50
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + 200, bar_y + 15), (40, 40, 40), -1)
            cv2.rectangle(frame, (bar_x, bar_y),
                          (bar_x + int(200 * progress), bar_y + 15), (0, 200, 255), -1)
            cv2.putText(frame, f"Verifying: {self.verify_count}/{VERIFY_FRAMES}",
                        (bar_x, bar_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 100), 1)

        # Bottom hint
        if self.state == State.WAITING:
            hint = "G=GO  Q=Quit"
        elif self.state == State.STOPPED:
            hint = "S=Resume  Q=Quit"
        else:
            hint = "Q=Quit  (autonomous — stop disabled)"
        cv2.putText(frame, hint,
                    (w // 2 - 100, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)


# ══════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="PET Bottle Robot — Autonomous Navigator (LiDAR + RealSense)"
    )
    parser.add_argument("--model", default="yolov8",
                        choices=["yolov5", "yolov7", "yolov8"],
                        help="YOLO model (default: yolov8)")
    parser.add_argument("--esp32-ip", default=None,
                        help="ESP32 WiFi IP address (default: auto-discover)")
    parser.add_argument("--lidar-port", default="/dev/ttyAMA0",
                        help="LD06 LiDAR serial port (default: /dev/ttyAMA0)")
    parser.add_argument("--lidar-baud", type=int, default=230400,
                        help="LiDAR baud rate (default: 230400)")
    parser.add_argument("--conf", type=float, default=CONF_THRESHOLD,
                        help=f"Confidence threshold (default: {CONF_THRESHOLD})")
    parser.add_argument("--no-show", action="store_true",
                        help="Headless mode (no display)")
    parser.add_argument("--output", default="./results",
                        help="Directory to save snapshots")
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"  PET Bottle Robot — Autonomous Navigator")
    print(f"  LD06 LiDAR + Intel RealSense + YOLO (WiFi transport)")
    print(f"{'='*60}")

    # Start web stream server
    print(f"  Starting video stream on port 5000...")
    stream_thread = threading.Thread(target=start_stream_server, args=(5000,), daemon=True)
    stream_thread.start()
    print(f"  Stream: http://0.0.0.0:5000/video_feed")

    # Start LD06 LiDAR
    print(f"  Starting LD06 LiDAR on {args.lidar_port}...")
    lidar = LD06LiDAR(port=args.lidar_port, baudrate=args.lidar_baud)
    lidar.start()
    # Wait briefly for first data
    for _ in range(20):
        if lidar.is_connected:
            break
        time.sleep(0.5)
    if lidar.is_connected:
        print(f"  LiDAR: connected and streaming")
    else:
        print(f"  WARNING: LiDAR not responding on {args.lidar_port} — continuing without LiDAR")

    # Start Intel RealSense camera
    print(f"  Starting Intel RealSense camera...")
    realsense = RealSenseCamera(width=640, height=480, fps=30)
    try:
        realsense.start()
    except Exception as e:
        print(f"  FATAL: RealSense camera failed to start: {e}")
        print(f"  Check USB 3.0 connection and pyrealsense2 installation")
        lidar.stop()
        return

    # Discover ESP32
    esp32_ip = args.esp32_ip
    if not esp32_ip:
        while True:
            esp32_ip = discover_esp32()
            if esp32_ip:
                break
            print("  ESP32 not found — retrying in 10s (waiting for it to join hotspot)...")
            time.sleep(10)

    global ESP32_BASE_URL
    ESP32_BASE_URL = f"http://{esp32_ip}"
    print(f"  Connecting to ESP32 via WiFi ({esp32_ip})...")
    with stream_lock:
        stream_stats["esp32_ip"] = esp32_ip
    esp32 = ESP32WiFiLink(ip=esp32_ip, block=False)

    # Load YOLO models
    print(f"  Loading YOLO models...")
    with VDevice() as target:
        manager = ModelManager(target)
        manager.activate(args.model)
        print(f"  Active: {manager.active_name}")

        try:
            nav = Navigator(manager, esp32, lidar, realsense,
                            args.conf, args.no_show, out_dir)
            nav.run()
        finally:
            esp32.shutdown()
            manager.cleanup()
            lidar.stop()
            realsense.stop()


if __name__ == "__main__":
    main()
