#!/usr/bin/env python3
"""
PET Bottle Detection Web App
Stream live camera + YOLO detection from Raspberry Pi via browser.

Usage (on Pi):
  cd ~/testing
  python3 web_detect.py

Then open in browser:
  http://100.97.223.3:5000       (via Tailscale, from anywhere)
  http://192.168.18.49:5000      (via local WiFi)
"""

import sys, os, time, threading
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, request, jsonify
from pathlib import Path

# Add the testing directory to path so we can import camera.py
sys.path.insert(0, str(Path(__file__).parent))

from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams, FormatType
)

from camera import (
    MODEL_CONFIGS, MODEL_RANGE, RANGE_LIMITS,
    COCO_BOTTLE_ID, COCO_PERSON_ID, CONF_THRESHOLD,
    ModelManager, preprocess, draw_detections,
    get_orientation, DetectionTracker
)

# ── App Config ──
app = Flask(__name__)
FRAME_W, FRAME_H = 640, 480

# ── Global State ──
lock = threading.Lock()
current_frame = None
current_model = "yolov8"
model_manager = None
vdevice = None
tracker = DetectionTracker()
stats = {"fps": 0, "bottles": 0, "persons": 0, "model": "yolov8", "inference_ms": 0}


def init_hailo():
    """Initialize Hailo device and load all models."""
    global vdevice, model_manager
    print("Initializing Hailo-8...")
    vdevice = VDevice()
    model_manager = ModelManager(vdevice)
    model_manager.activate("yolov8")
    print("Hailo-8 ready.")


def camera_loop():
    """Background thread: capture frames, run detection, update global state."""
    global current_frame, stats

    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(
            main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
        ))
        cam.start()
        print(f"Camera started: {FRAME_W}x{FRAME_H}")
        use_picam = True
    except Exception as e:
        print(f"Picamera2 failed ({e}), trying OpenCV...")
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        use_picam = False

    fps_counter = 0
    fps_timer = time.time()

    while True:
        # Capture
        if use_picam:
            frame = cam.capture_array()
        else:
            ret, frame = cam.read()
            if not ret:
                time.sleep(0.01)
                continue

        # Convert RGB to BGR for OpenCV drawing
        if use_picam:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            frame_bgr = frame

        # Run detection
        t0 = time.time()
        with lock:
            active = current_model

        rgb_frame = frame if use_picam else cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        try:
            if active == "all":
                bottles, persons = model_manager.infer_ensemble(rgb_frame, CONF_THRESHOLD)
            else:
                if model_manager.active_key != active:
                    model_manager.activate(active)
                # Wait for pipeline to be ready
                if model_manager._pipeline is None:
                    time.sleep(0.1)
                    continue
                bottles, persons = model_manager.infer(
                    rgb_frame,
                    CONF_THRESHOLD,
                    apply_range_filter=False
                )
        except Exception as e:
            bottles, persons = [], []
            if "NETWORK_GROUP_NOT_ACTIVATED" not in str(e):
                print(f"Inference error: {e}")
            time.sleep(0.1)
            continue

        inference_ms = (time.time() - t0) * 1000

        # Track detections
        bottles = tracker.update(bottles)

        # Draw detections
        draw_detections(frame_bgr, bottles, persons)

        # Draw info overlay
        model_name = "Ensemble" if active == "all" else model_manager.active_name
        color = (255, 255, 255) if active == "all" else model_manager.active_color

        cv2.putText(frame_bgr, f"{model_name}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame_bgr, f"Bottles: {len(bottles)}  Persons: {len(persons)}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame_bgr, f"{inference_ms:.0f}ms  {stats['fps']:.0f} FPS",
                    (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Encode to JPEG
        _, jpeg = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])

        with lock:
            current_frame = jpeg.tobytes()
            stats["bottles"] = len(bottles)
            stats["persons"] = len(persons)
            stats["model"] = model_name
            stats["inference_ms"] = round(inference_ms, 1)

        # FPS counter
        fps_counter += 1
        if time.time() - fps_timer >= 1.0:
            with lock:
                stats["fps"] = fps_counter
            fps_counter = 0
            fps_timer = time.time()


def generate_mjpeg():
    """Yield MJPEG frames for the video stream."""
    while True:
        with lock:
            frame = current_frame
        if frame is None:
            time.sleep(0.01)
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)  # ~30 FPS max


# ── Routes ──

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)


@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/switch_model', methods=['POST'])
def switch_model():
    global current_model
    model = request.json.get('model', 'yolov8')
    if model in ('yolov5', 'yolov7', 'yolov8', 'all'):
        with lock:
            current_model = model
        return jsonify({"status": "ok", "model": model})
    return jsonify({"status": "error", "msg": "invalid model"}), 400


@app.route('/stats')
def get_stats():
    with lock:
        return jsonify(stats)


# ── HTML Template ──

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PET Bottle Detection</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    background: #0a0a1a;
    color: #e0e0e0;
    min-height: 100vh;
}
.header {
    background: linear-gradient(135deg, #1a1a3e, #0d0d2b);
    padding: 16px 24px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    border-bottom: 1px solid #2a2a5e;
}
.header h1 {
    font-size: 1.3rem;
    color: #4fc3f7;
}
.status {
    display: flex;
    gap: 16px;
    font-size: 0.85rem;
}
.status .stat {
    background: #1a1a3e;
    padding: 6px 14px;
    border-radius: 8px;
    border: 1px solid #2a2a5e;
}
.stat-val { color: #66bb6a; font-weight: 700; }
.main {
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 20px;
    gap: 16px;
}
.video-container {
    background: #111;
    border-radius: 12px;
    overflow: hidden;
    box-shadow: 0 4px 30px rgba(0,0,0,0.5);
    max-width: 100%;
}
.video-container img {
    display: block;
    width: 640px;
    max-width: 100%;
    height: auto;
}
.controls {
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
    justify-content: center;
}
.model-btn {
    font-size: 1rem;
    font-weight: 600;
    border: 2px solid #333;
    border-radius: 12px;
    padding: 12px 28px;
    cursor: pointer;
    background: #1a1a3e;
    color: #ccc;
    transition: all 0.2s;
}
.model-btn:hover { border-color: #4fc3f7; color: #fff; }
.model-btn.active { border-color: #4fc3f7; color: #4fc3f7; background: #1a2a4e; }
.model-btn.v5 { border-color: #ffeb3b; }
.model-btn.v5.active { color: #ffeb3b; background: #2a2a1e; border-color: #ffeb3b; }
.model-btn.v7 { border-color: #e040fb; }
.model-btn.v7.active { color: #e040fb; background: #2a1a2e; border-color: #e040fb; }
.model-btn.v8 { border-color: #4fc3f7; }
.model-btn.v8.active { color: #4fc3f7; background: #1a2a4e; border-color: #4fc3f7; }
.model-btn.all { border-color: #66bb6a; }
.model-btn.all.active { color: #66bb6a; background: #1a2e1a; border-color: #66bb6a; }
.footer {
    text-align: center;
    padding: 12px;
    color: #555;
    font-size: 0.75rem;
}
</style>
</head>
<body>

<div class="header">
    <h1>PET Bottle Detection</h1>
    <div class="status">
        <div class="stat">Model: <span class="stat-val" id="s-model">-</span></div>
        <div class="stat">Bottles: <span class="stat-val" id="s-bottles">0</span></div>
        <div class="stat">Persons: <span class="stat-val" id="s-persons">0</span></div>
        <div class="stat">FPS: <span class="stat-val" id="s-fps">0</span></div>
        <div class="stat"><span class="stat-val" id="s-ms">0</span> ms</div>
    </div>
</div>

<div class="main">
    <div class="video-container">
        <img src="/video_feed" alt="Live Detection Feed">
    </div>

    <div class="controls">
        <button class="model-btn v5" onclick="switchModel('yolov5')">YOLOv5</button>
        <button class="model-btn v7" onclick="switchModel('yolov7')">YOLOv7</button>
        <button class="model-btn v8 active" onclick="switchModel('yolov8')">YOLOv8</button>
        <button class="model-btn all" onclick="switchModel('all')">Ensemble</button>
    </div>
</div>

<div class="footer">
    PET Bottle Collector Robot &mdash; Hailo-8 + Raspberry Pi 5 &mdash; Emathrix Academy
</div>

<script>
function switchModel(model) {
    fetch('/switch_model', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({model: model})
    }).then(() => {
        document.querySelectorAll('.model-btn').forEach(b => b.classList.remove('active'));
        document.querySelector('.model-btn.' + (model === 'all' ? 'all' : model.replace('yolo',''))).classList.add('active');
    });
}

function updateStats() {
    fetch('/stats').then(r => r.json()).then(d => {
        document.getElementById('s-model').textContent = d.model;
        document.getElementById('s-bottles').textContent = d.bottles;
        document.getElementById('s-persons').textContent = d.persons;
        document.getElementById('s-fps').textContent = d.fps;
        document.getElementById('s-ms').textContent = d.inference_ms;
    }).catch(() => {});
}
setInterval(updateStats, 500);
updateStats();
</script>

</body>
</html>
"""


if __name__ == '__main__':
    init_hailo()

    # Give Hailo a moment to fully settle
    time.sleep(1)

    # Start camera + detection thread
    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()

    print("\n  Web app running at:")
    print("    http://0.0.0.0:5000")
    print("    http://100.97.223.3:5000  (Tailscale - anywhere)")
    print()

    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
