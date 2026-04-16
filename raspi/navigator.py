#!/usr/bin/env python3
"""
PET Bottle Robot — Autonomous Navigator
Runs on Raspberry Pi. Fuses camera (YOLO) + ultrasonic sensors from ESP32
for real-time obstacle avoidance and bottle pickup.

The robot continuously:
  1. Roams open space looking for PET bottles
  2. Avoids obstacles (ultrasonics + camera person/object detection)
  3. Approaches detected bottles
  4. Triggers the pickup sequence on the ESP32
  5. Resumes roaming

Usage:
  python3 navigator.py                    # default (YOLOv8, auto-connect)
  python3 navigator.py --model yolov8     # default model
  python3 navigator.py --esp32-ip 192.168.4.1
  python3 navigator.py --no-show          # headless
"""

import cv2
import numpy as np
import argparse
import time
import threading
import requests
import json
from pathlib import Path
from enum import Enum, auto
from flask import Flask, Response, jsonify, request

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


# ══════════════════════════════════════════════════════════════
# Configuration
# ══════════════════════════════════════════════════════════════

ESP32_IP        = "192.168.4.1"
ESP32_TIMEOUT   = 0.5           # HTTP timeout (seconds)
SENSOR_POLL_HZ  = 10            # ultrasonic poll rate

# Obstacle avoidance thresholds (cm)
# Robot is ~100cm long x 30cm wide — sensors are at the front
STOP_DIST       = 60            # emergency stop — back up immediately
SLOW_DIST       = 100           # reduce speed
TURN_DIST       = 150           # start steering away (1.5m clearance)

# Driving speeds — SLOW AND SAFE (people nearby!)
# Track drive (tank-style): forward/backward = 40%, turning = 60%
# Tracks need more power to skid-steer than to drive straight
ROAM_SPEED      = 40            # forward/backward — 40% duty
SLOW_SPEED      = 30            # near obstacles — slow crawl
APPROACH_SPEED  = 40            # approaching a bottle — 40%
TURN_SPEED      = 60            # turning — 60% (tracks need more to skid-steer)
SCAN_SPEED      = 60            # scanning rotation — 60%
ALIGN_SPEED     = 35            # fine alignment turns before pickup — slow to avoid overshoot
VERIFY_SPEED    = 30            # verification approach — extra cautious

# Bottle detection & verification
BOTTLE_CLOSE_FILL = 0.15        # bottle fills 15% of frame → close enough to pick
BOTTLE_CENTER_TOL = 0.15        # tolerance from frame center (fraction)
VERIFY_FRAMES     = 10          # must see bottle in N frames before approaching
SCAN_DURATION     = 8.0         # seconds to scan (slow 360 rotation)

# COCO obstacle classes (things the robot should avoid)
COCO_OBSTACLE_CLASSES = {
    0: "person", 56: "chair", 57: "couch", 58: "potted plant",
    59: "bed", 60: "dining table", 62: "tv",
}


# ══════════════════════════════════════════════════════════════
# Web Stream (Flask MJPEG server for ESP32 web UI)
# ══════════════════════════════════════════════════════════════

stream_app = Flask(__name__)
stream_lock = threading.Lock()
stream_frame = None  # latest JPEG-encoded frame
stream_stats = {"fps": 0, "bottles": 0, "persons": 0, "model": "", "inference_ms": 0, "state": "WAITING",
                "ultrasonic": {"s1": 999, "s2": 999, "s3": 999, "s4": 999}}
_navigator_ref = None  # set by Navigator.__init__ so Flask routes can control it

def update_stream(frame_bgr, bottles, persons, model_name, inference_ms, state_name, ultrasonic=None):
    """Called from navigator loop to update the shared frame and stats."""
    global stream_frame, stream_stats
    _, jpeg = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
    with stream_lock:
        stream_frame = jpeg.tobytes()
        stream_stats["bottles"] = len(bottles)
        stream_stats["persons"] = len(persons)
        stream_stats["model"] = model_name
        stream_stats["inference_ms"] = round(inference_ms, 1)
        stream_stats["state"] = state_name
        if ultrasonic:
            stream_stats["ultrasonic"] = ultrasonic

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

@stream_app.route('/start')
def nav_start():
    if _navigator_ref and _navigator_ref.state == State.WAITING:
        _navigator_ref.state = State.SCANNING
        _navigator_ref.scan_start = time.time()
        return jsonify({"ok": True, "state": "SCANNING"})
    elif _navigator_ref:
        return jsonify({"ok": False, "state": _navigator_ref.state.name, "msg": "already running"})
    return jsonify({"ok": False, "msg": "navigator not ready"})

@stream_app.route('/stop')
def nav_stop():
    if _navigator_ref:
        _navigator_ref.esp32.stop()
        _navigator_ref.state = State.WAITING
        return jsonify({"ok": True, "state": "WAITING"})
    return jsonify({"ok": False, "msg": "navigator not ready"})

@stream_app.route('/resume')
def nav_resume():
    if _navigator_ref and _navigator_ref.state == State.STOPPED:
        _navigator_ref.state = State.SCANNING
        _navigator_ref.scan_start = time.time()
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
<title>PET Bottle Navigator</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:#111;color:#eee;padding:12px;
  -webkit-user-select:none;user-select:none}
h1{text-align:center;font-size:1.3rem;margin-bottom:8px}
.video-wrap{position:relative;text-align:center;margin-bottom:10px}
.video-wrap img{width:100%;max-width:640px;border-radius:12px;border:2px solid #333}
.badge{position:absolute;top:10px;right:10px;padding:6px 14px;border-radius:8px;
  font-weight:700;font-size:.9rem;color:#111}
.controls{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;margin:12px 0}
button{font-size:1.1rem;font-weight:700;border:none;border-radius:12px;
  padding:14px 28px;cursor:pointer;transition:transform .08s;
  touch-action:manipulation}
button:active{transform:scale(.94)}
.go{background:#66bb6a;color:#111}
.stop{background:#ef5350;color:#fff}
.stats{background:#1a1a2e;border-radius:12px;padding:12px;margin:10px auto;
  max-width:640px;display:grid;grid-template-columns:repeat(auto-fit,minmax(100px,1fr));gap:8px}
.stat{text-align:center}
.stat .val{font-size:1.4rem;font-weight:700;color:#4fc3f7}
.stat .lbl{font-size:.7rem;color:#888;margin-top:2px}
#log{background:#000;border-radius:8px;padding:8px;margin:10px auto;
  max-width:640px;font-family:monospace;font-size:.75rem;color:#4fc3f7;
  height:60px;overflow-y:auto;white-space:pre-wrap}
</style>
</head>
<body>
<h1>PET Bottle Navigator</h1>

<div class="video-wrap">
  <img src="/video_feed" alt="Live Feed">
  <div class="badge" id="badge" style="background:#ff9800">WAITING</div>
</div>

<div class="controls">
  <button class="go" onclick="act('/start')">START</button>
  <button class="stop" onclick="act('/stop')">STOP</button>
</div>

<div class="stats">
  <div class="stat"><div class="val" id="fps">-</div><div class="lbl">FPS</div></div>
  <div class="stat"><div class="val" id="bottles">0</div><div class="lbl">Bottles</div></div>
  <div class="stat"><div class="val" id="persons">0</div><div class="lbl">Persons</div></div>
  <div class="stat"><div class="val" id="infer">-</div><div class="lbl">Infer ms</div></div>
  <div class="stat"><div class="val" id="model">-</div><div class="lbl">Model</div></div>
</div>

<div id="log">Ready.\n</div>

<script>
const stateColors={WAITING:'#ff9800',SCANNING:'#ffeb3b',ROAMING:'#66bb6a',
  VERIFYING:'#ffcc80',APPROACHING:'#4fc3f7',ALIGNING:'#ffd54f',
  PICKING_UP:'#ab47bc',AVOIDING:'#ef5350',STOPPED:'#666'};
const log=document.getElementById('log');

function act(url){
  fetch(url).then(r=>r.json()).then(d=>{
    log.textContent+='> '+url+' -> '+JSON.stringify(d)+'\n';
    log.scrollTop=log.scrollHeight;
  }).catch(e=>{log.textContent+='ERR: '+e+'\n'});
}

function poll(){
  fetch('/stats').then(r=>r.json()).then(d=>{
    document.getElementById('fps').textContent=d.fps||'-';
    document.getElementById('bottles').textContent=d.bottles||0;
    document.getElementById('persons').textContent=d.persons||0;
    document.getElementById('infer').textContent=d.inference_ms||'-';
    document.getElementById('model').textContent=d.model||'-';
    const b=document.getElementById('badge');
    b.textContent=d.state||'?';
    b.style.background=stateColors[d.state]||'#666';
  }).catch(()=>{});
}
setInterval(poll,1000);
poll();
</script>
</body>
</html>"""

def start_stream_server(port=5000):
    """Run Flask in a background thread."""
    stream_app.run(host='0.0.0.0', port=port, threaded=True, use_reloader=False)


# ══════════════════════════════════════════════════════════════
# Robot States
# ══════════════════════════════════════════════════════════════

class State(Enum):
    WAITING     = auto()   # safety hold — motors disabled until user confirms
    SCANNING    = auto()   # slow rotation to look around for bottles
    ROAMING     = auto()   # slow forward movement, exploring
    VERIFYING   = auto()   # bottle maybe seen — moving closer slowly to confirm
    APPROACHING = auto()   # bottle confirmed — driving towards it slowly
    ALIGNING    = auto()   # close to bottle, fine-tuning position
    PICKING_UP  = auto()   # running pickup sequence on ESP32
    AVOIDING    = auto()   # obstacle detected, steering away slowly
    STOPPED     = auto()   # emergency stop / idle


# ══════════════════════════════════════════════════════════════
# ESP32 Communication (thread-safe, non-blocking)
# ══════════════════════════════════════════════════════════════

class ESP32Link:
    """Talks to the ESP32 robot controller over WiFi HTTP.
    Uses PI-prefixed commands that bypass the ESP32's test-UI mode system,
    so the navigator always has direct motor control.
    Logs all commands and responses for the live monitor."""

    MAX_LOG = 20  # keep last 20 command/response entries

    def __init__(self, ip=ESP32_IP):
        self.base_url = f"http://{ip}"
        self.sensor_data = {}
        self._lock = threading.Lock()
        self._running = True
        self.cmd_log = []  # list of (timestamp, command, response)
        self._last_cmd = None  # avoid logging duplicate consecutive commands
        self._sensor_thread = threading.Thread(target=self._poll_sensors, daemon=True)
        self._sensor_thread.start()

    def _poll_sensors(self):
        """Background thread: polls /sensor at SENSOR_POLL_HZ."""
        while self._running:
            try:
                r = requests.get(f"{self.base_url}/sensor", timeout=ESP32_TIMEOUT)
                data = r.json()
                with self._lock:
                    self.sensor_data = data
            except Exception:
                pass
            time.sleep(1.0 / SENSOR_POLL_HZ)

    def _log(self, command, response):
        """Log command/response, skip duplicates."""
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
        """Get recent command log (thread-safe copy)."""
        with self._lock:
            return list(self.cmd_log)

    @property
    def ultrasonic(self):
        """Returns dict with all 4 sensors: {s1: front, s2: right, s3: back, s4: left}.
        ESP32 /sensor returns: {"ultrasonic": {"s1": N, "s2": N, "s3": N, "s4": N}}"""
        with self._lock:
            us = self.sensor_data.get("ultrasonic", {})
            return {
                "s1": us.get("s1", 999),  # front
                "s2": us.get("s2", 999),  # right
                "s3": us.get("s3", 999),  # back
                "s4": us.get("s4", 999),  # left
            }

    @property
    def ultrasonic_min(self):
        """Returns the minimum distance across all 4 sensors."""
        us = self.ultrasonic
        return min(us["s1"], us["s2"], us["s3"], us["s4"])

    @property
    def sensors(self):
        """Returns full sensor dict (thread-safe copy)."""
        with self._lock:
            return dict(self.sensor_data)

    @property
    def is_connected(self):
        with self._lock:
            # If we got ultrasonic data, the ESP32 is alive
            return "ultrasonic" in self.sensor_data

    def cmd(self, command):
        """Send a command to the ESP32 (non-blocking), log it."""
        try:
            r = requests.get(f"{self.base_url}/cmd", params={"c": command},
                             timeout=ESP32_TIMEOUT)
            self._log(command, r.text.strip())
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
        """Drive wheels at different speeds for gentle steering.
        Speeds can be negative (reverse). Clamped to [-80, 80]."""
        self.cmd(f"PIDW{left_speed},{right_speed}")

    def stop(self):
        self.cmd("PIX")

    def emergency_stop(self):
        """Stop ALL motors, servos, buzzer — everything."""
        self.cmd("PISTOP")

    def pickup(self):
        """Triggers the full pickup sequence on ESP32 (blocking ~15s)."""
        try:
            r = requests.get(f"{self.base_url}/cmd", params={"c": "P"},
                             timeout=30)
            self._log("P", r.text.strip())
        except Exception as e:
            self._log("P", f"ERR: {e}")

    def shutdown(self):
        self._running = False
        self.stop()


# ══════════════════════════════════════════════════════════════
# Autonomous Navigator
# ══════════════════════════════════════════════════════════════

class Navigator:
    """
    Main brain: fuses camera + ultrasonics for real-time navigation.
    Roams → detects bottle → approaches → picks up → resumes.
    """

    def __init__(self, manager, esp32, conf_thresh, no_show, out_dir):
        global _navigator_ref
        self.manager     = manager
        self.esp32       = esp32
        self.conf_thresh = conf_thresh
        self.no_show     = no_show
        self.out_dir     = out_dir

        self.state       = State.WAITING
        _navigator_ref   = self  # expose to Flask routes
        self.tracker     = DetectionTracker()
        self.turn_dir    = 1         # 1 = right, -1 = left (roaming direction)
        self.turn_timer  = 0.0
        self.avoid_timer = 0.0
        self.scan_start  = 0.0       # when scanning started
        self.verify_count = 0        # frames bottle has been seen during verify
        self.verify_lost  = 0        # frames bottle was NOT seen during verify
        self.approach_lost_count = 0

    def run(self):
        """Main loop — camera + sensors + navigation."""
        # Camera setup
        try:
            from picamera2 import Picamera2
            picam2 = Picamera2()
            picam2.configure(picam2.create_preview_configuration(
                main={"format": "BGR888", "size": (640, 480)}
            ))
            picam2.start()
            use_picamera2 = True
            print("  Using picamera2 (CSI ribbon)")
        except Exception as e:
            print(f"  picamera2 not available ({e}), falling back to V4L2...")
            use_picamera2 = False
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("Cannot open camera")
                return
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.no_show:
            cv2.namedWindow("PET Bottle Navigator", cv2.WINDOW_NORMAL)

        print(f"\n{'='*60}")
        print(f"  PET Bottle Robot — AUTONOMOUS NAVIGATOR")
        print(f"  Camera + Ultrasonic Fusion")
        print(f"  ESP32: {self.esp32.base_url}")
        print(f"  SAFETY MODE: Motors DISABLED")
        print(f"  Press G to GO (enable motors) | Q to quit")
        print(f"{'='*60}\n")

        # Send stop command to ESP32 to ensure motors are off
        self.esp32.stop()

        frame_idx = 0

        try:
            while True:
                # ── Grab frame ────────────────────────────────
                if use_picamera2:
                    frame_rgb = picam2.capture_array()
                    frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                else:
                    ret, frame = cap.read()
                    if not ret:
                        break

                orig_h, orig_w = frame.shape[:2]
                frame_area = orig_w * orig_h

                # ── Run detection ─────────────────────────────
                t0 = time.time()
                bottles, persons = self.manager.infer(
                    frame, self.conf_thresh, apply_range_filter=False)
                ms = (time.time() - t0) * 1000

                smoothed = self.tracker.update(bottles)

                # ── Get ultrasonic data ───────────────────────
                us = self.esp32.ultrasonic  # {s1:front, s2:right, s3:back, s4:left}

                # ── Navigation decision ───────────────────────
                self._navigate(smoothed, persons, orig_w, orig_h,
                               frame_area, us)

                # ── Draw results ──────────────────────────────
                result = draw_detections(frame.copy(), smoothed, persons)
                self._draw_nav_overlay(result, smoothed, us, ms, frame_idx)

                # ── Update web stream ────────────────────────
                model_name = self.manager.active_name
                update_stream(result, smoothed, persons, model_name, ms, self.state.name, us)

                # ── FPS counter for stream stats ─────────────
                self._fps_count = getattr(self, '_fps_count', 0) + 1
                if not hasattr(self, '_fps_timer'):
                    self._fps_timer = time.time()
                if time.time() - self._fps_timer >= 1.0:
                    with stream_lock:
                        stream_stats["fps"] = self._fps_count
                    self._fps_count = 0
                    self._fps_timer = time.time()

                # ── Console output ────────────────────────────
                icon = "OK" if smoothed else "--"
                print(f"\r  [{icon}] {self.state.name:12s} "
                      f"US F:{us['s1']:3d} R:{us['s2']:3d} "
                      f"B:{us['s3']:3d} L:{us['s4']:3d} | "
                      f"{len(smoothed)} det | {ms:.0f}ms   ",
                      end="", flush=True)

                # ── Display + keyboard ────────────────────────
                if not self.no_show:
                    cv2.imshow("PET Bottle Navigator", result)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), ord('Q'), 27):
                        break
                    elif key in (ord('g'), ord('G')):
                        if self.state == State.WAITING:
                            self.state = State.SCANNING
                            self.scan_start = time.time()
                            print("\n  >>> MOTORS ENABLED — SCANNING (slow look-around)")
                    elif key in (ord('s'), ord('S')):
                        if self.state == State.STOPPED:
                            self.state = State.ROAMING
                            print("\n  >>> RESUMED")
                        elif self.state != State.WAITING:
                            self.esp32.stop()
                            self.state = State.STOPPED
                            print("\n  >>> MANUAL STOP")
                else:
                    # Headless mode: check for Enter key on stdin (cross-platform)
                    if self.state == State.WAITING:
                        try:
                            import sys
                            if sys.platform == "win32":
                                import msvcrt
                                if msvcrt.kbhit():
                                    msvcrt.getch()
                                    self.state = State.SCANNING
                                    self.scan_start = time.time()
                                    print("\n  >>> MOTORS ENABLED — SCANNING (slow look-around)")
                            else:
                                import select
                                if select.select([sys.stdin], [], [], 0)[0]:
                                    sys.stdin.readline()
                                    self.state = State.SCANNING
                                    self.scan_start = time.time()
                                    print("\n  >>> MOTORS ENABLED — SCANNING (slow look-around)")
                        except Exception:
                            pass  # stdin not available (e.g., systemd service)

                frame_idx += 1

        finally:
            self.esp32.stop()
            if use_picamera2:
                picam2.stop()
            else:
                cap.release()
            if not self.no_show:
                cv2.destroyAllWindows()
            print()

    # ── Navigation logic ──────────────────────────────────────

    def _navigate(self, bottles, persons, w, h, frame_area, us):
        """State machine: slow, careful navigation. Safety is #1 priority.
        us = dict with keys s1(front), s2(right), s3(back), s4(left) in cm."""

        # ── Safety hold: no motor commands until user confirms ──
        if self.state == State.WAITING:
            return

        # ── Stopped state: do nothing ─────────────────────
        if self.state == State.STOPPED:
            return

        # ── Picking up: wait for sequence to finish ───────
        if self.state == State.PICKING_UP:
            return

        us_front = us["s1"]
        us_right = us["s2"]
        us_back  = us["s3"]
        us_left  = us["s4"]
        us_min_forward = min(us_front, us_left, us_right)  # sensors facing forward path

        # ── Avoiding state: back up SLOWLY and turn away ──
        # Must be checked FIRST so the avoidance timer isn't reset by
        # the STOP_DIST check below while the robot is still backing up.
        if self.state == State.AVOIDING:
            elapsed = time.time() - self.avoid_timer
            if elapsed < 1.5:
                # Back up (check rear sensor first)
                if us_back > STOP_DIST:
                    self.esp32.backward(SLOW_SPEED)
                else:
                    self.esp32.stop()  # blocked front AND back
            elif elapsed < 3.0:
                # Turn away from the closer side
                if us_left < us_right:
                    self.esp32.turn_right(SCAN_SPEED)
                else:
                    self.esp32.turn_left(SCAN_SPEED)
            else:
                self.esp32.stop()
                self.state = State.SCANNING
                self.scan_start = time.time()
            return

        # ── PERSON DETECTED → immediate stop (safety first!) ──
        for p in persons:
            pw = p[2] - p[0]
            ph = p[3] - p[1]
            p_fill = (pw * ph) / frame_area
            if p_fill > 0.10:  # person visible → stop immediately
                self.esp32.stop()
                self.state = State.AVOIDING
                self.avoid_timer = time.time()
                print("\n  >>> PERSON DETECTED — stopping for safety")
                return

        # ── Emergency stop: front ultrasonic too close ────
        if us_front < STOP_DIST:
            self.esp32.stop()
            self.state = State.AVOIDING
            self.avoid_timer = time.time()
            return

        # ── Ultrasonic: slow zone → steer away gently ────
        if us_min_forward < TURN_DIST:
            if us_left < us_right:
                self.esp32.turn_right(SCAN_SPEED)
            else:
                self.esp32.turn_left(SCAN_SPEED)
            return

        # ── SCANNING: slow rotation to look around 360° ──
        if self.state == State.SCANNING:
            elapsed = time.time() - self.scan_start

            if bottles:
                # Saw something during scan — stop and verify
                self.esp32.stop()
                self.state = State.VERIFYING
                self.verify_count = 1
                self.verify_lost = 0
                print("\n  >>> Possible bottle spotted — verifying...")
                return

            if elapsed < SCAN_DURATION:
                # Keep rotating slowly
                if self.turn_dir > 0:
                    self.esp32.turn_right(SCAN_SPEED)
                else:
                    self.esp32.turn_left(SCAN_SPEED)
            else:
                # Scan complete, nothing found — move forward slowly then scan again
                self.esp32.stop()
                self.state = State.ROAMING
                self.turn_timer = time.time()
                self.turn_dir *= -1  # alternate scan direction next time
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
                    # Confirmed! Now approach
                    print(f"\n  >>> Bottle CONFIRMED ({self.verify_count} frames) — approaching slowly")
                    self.state = State.APPROACHING
                    self.approach_lost_count = 0
                    return
                elif b_fill < 0.03:
                    # Too far to be sure — creep forward very slowly
                    self.esp32.forward(VERIFY_SPEED)
                else:
                    # Close enough to verify from here — just wait
                    self.esp32.stop()
            else:
                self.verify_lost += 1
                self.esp32.stop()
                if self.verify_lost > 20:
                    # Lost it — false alarm, go back to scanning
                    print("\n  >>> False alarm — resuming scan")
                    self.state = State.SCANNING
                    self.scan_start = time.time()
                    self.verify_count = 0
            return

        # ── APPROACHING: bottle confirmed, move towards it slowly ──
        if self.state == State.APPROACHING:
            if bottles:
                self.approach_lost_count = 0
                best = max(bottles, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
                bx1, by1, bx2, by2, bconf = best
                bw = bx2 - bx1
                bh = by2 - by1
                b_fill = (bw * bh) / frame_area
                b_cx = (bx1 + bx2) / 2.0
                b_cx_norm = b_cx / w  # 0.0 = left, 1.0 = right

                if b_fill >= BOTTLE_CLOSE_FILL:
                    # Close enough — check alignment then pick up
                    if abs(b_cx_norm - 0.5) < BOTTLE_CENTER_TOL:
                        self._start_pickup()
                    else:
                        # Align slowly — use ALIGN_SPEED to avoid overshooting
                        self.state = State.ALIGNING
                        if b_cx_norm < 0.5:
                            self.esp32.turn_left(ALIGN_SPEED)
                        else:
                            self.esp32.turn_right(ALIGN_SPEED)
                else:
                    # Drive towards bottle slowly, steer gently with differential
                    if abs(b_cx_norm - 0.5) > 0.25:
                        if b_cx_norm < 0.5:
                            self.esp32.differential(VERIFY_SPEED, APPROACH_SPEED)
                        else:
                            self.esp32.differential(APPROACH_SPEED, VERIFY_SPEED)
                    else:
                        self.esp32.forward(APPROACH_SPEED)
            else:
                self.approach_lost_count += 1
                self.esp32.stop()
                if self.approach_lost_count > 20:
                    print("\n  >>> Lost bottle — scanning again")
                    self.state = State.SCANNING
                    self.scan_start = time.time()
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
                self.state = State.SCANNING
                self.scan_start = time.time()
            return

        # ── ROAMING: move forward slowly, then scan again ─
        if self.state == State.ROAMING:
            now = time.time()
            elapsed = now - self.turn_timer

            if bottles:
                # Spotted something while roaming — stop and verify
                self.esp32.stop()
                self.state = State.VERIFYING
                self.verify_count = 1
                self.verify_lost = 0
                print("\n  >>> Possible bottle spotted — verifying...")
                return

            if elapsed < 5.0:
                # Move forward slowly for 5 seconds
                speed = ROAM_SPEED
                if us_min_forward < SLOW_DIST:
                    speed = SLOW_SPEED
                self.esp32.forward(speed)
            else:
                # Done moving — stop and scan again
                self.esp32.stop()
                self.state = State.SCANNING
                self.scan_start = time.time()

    def _start_pickup(self):
        """Stop, run pickup sequence in a thread, then resume scanning."""
        self.state = State.PICKING_UP
        self.esp32.stop()
        time.sleep(0.3)

        def _do_pickup():
            print("\n  >>> PICKUP SEQUENCE STARTED")
            self.esp32.pickup()
            # Wait for the ESP32 pickup state machine to actually finish.
            # The HTTP response comes back instantly — the arm is still moving.
            # Poll /sensor until pickup field returns "idle" or "done".
            timeout = time.time() + 35  # 30s ESP32 timeout + 5s margin
            while time.time() < timeout:
                sensor = self.esp32.sensors
                pu_state = sensor.get("pickup", "idle")
                if pu_state in ("idle", "done"):
                    break
                time.sleep(0.5)
            else:
                print("\n  >>> PICKUP TIMEOUT waiting for ESP32 — aborting")
                self.esp32.cmd("PA")  # send abort
                time.sleep(1)

            print("\n  >>> PICKUP DONE — scanning for next bottle")
            self.tracker = DetectionTracker()  # reset tracker
            self.state = State.SCANNING
            self.scan_start = time.time()

        threading.Thread(target=_do_pickup, daemon=True).start()

    # ── HUD overlay ───────────────────────────────────────

    def _draw_nav_overlay(self, frame, bottles, us, ms, frame_idx):
        """Draw navigation HUD on frame.
        us = dict with keys s1(front), s2(right), s3(back), s4(left)."""
        h, w = frame.shape[:2]

        # State badge (top-right)
        state_colors = {
            State.WAITING:     (0, 165, 255),
            State.SCANNING:    (255, 255, 0),
            State.ROAMING:     (0, 255, 0),
            State.VERIFYING:   (255, 200, 100),
            State.APPROACHING: (0, 200, 255),
            State.ALIGNING:    (255, 200, 0),
            State.PICKING_UP:  (255, 0, 255),
            State.AVOIDING:    (0, 0, 255),
            State.STOPPED:     (80, 80, 80),
        }
        color = state_colors.get(self.state, (255, 255, 255))
        badge = f"[{self.state.name}]"
        (tw, th), _ = cv2.getTextSize(badge, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.rectangle(frame, (w - tw - 20, 0), (w, th + 16), color, -1)
        cv2.putText(frame, badge, (w - tw - 10, th + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        # Ultrasonic bars — all 4 sensors
        bar_max_h = 100
        bar_w = 30
        margin = 10
        # Layout: left edge=S4(left), center-left=S1(front), center-right=S3(back), right edge=S2(right)
        bar_positions = [
            ("F", us["s1"], margin),
            ("L", us["s4"], margin + bar_w + 6),
            ("R", us["s2"], w - bar_w - margin),
            ("B", us["s3"], w - 2 * bar_w - margin - 6),
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

        # Sensor data (top-left, below FPS)
        sensor = self.esp32.sensors
        wheels = sensor.get("wheels", {})
        cv2.putText(frame, f"Wheels L:{wheels.get('left',0)} R:{wheels.get('right',0)}"
                    f"  Lift:{sensor.get('lift',0)}  Base:{sensor.get('base',0)}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        # Command log panel (right side)
        log = self.esp32.recent_log
        if log:
            panel_x = w - 280
            panel_y = 40
            panel_h = min(len(log), 10) * 18 + 10
            # Semi-transparent background
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

        # Verify progress bar (when verifying)
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
        hint = "G=GO  S=Stop/Resume  Q=Quit" if self.state == State.WAITING else "S=Stop/Resume  Q=Quit"
        cv2.putText(frame, hint,
                    (w // 2 - 100, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)


# ══════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="PET Bottle Robot — Autonomous Navigator (Pi + ESP32)"
    )
    parser.add_argument("--model", default="yolov8",
                        choices=["yolov5", "yolov7", "yolov8"],
                        help="YOLO model (default: yolov8)")
    parser.add_argument("--esp32-ip", default=ESP32_IP,
                        help=f"ESP32 WiFi IP (default: {ESP32_IP})")
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
    print(f"  Camera + Ultrasonic Fusion")
    print(f"{'='*60}")

    # Start web stream server (background thread)
    print(f"  Starting video stream on port 5000...")
    stream_thread = threading.Thread(target=start_stream_server, args=(5000,), daemon=True)
    stream_thread.start()
    print(f"  Stream: http://0.0.0.0:5000/video_feed")

    # Connect to ESP32
    print(f"  Connecting to ESP32 at {args.esp32_ip}...")
    esp32 = ESP32Link(ip=args.esp32_ip)
    time.sleep(0.5)
    us = esp32.ultrasonic
    if any(v < 999 for v in us.values()):
        print(f"  ESP32 connected! Ultrasonic: F={us['s1']}cm R={us['s2']}cm B={us['s3']}cm L={us['s4']}cm")
    else:
        print(f"  WARNING: ESP32 not responding — running camera-only mode")

    # Load YOLO models
    print(f"  Loading YOLO models...")
    with VDevice() as target:
        manager = ModelManager(target)

        manager.activate(args.model)
        print(f"  Active: {manager.active_name}")

        try:
            nav = Navigator(manager, esp32, args.conf,
                            args.no_show, out_dir)
            nav.run()
        finally:
            esp32.shutdown()
            manager.cleanup()


if __name__ == "__main__":
    main()
