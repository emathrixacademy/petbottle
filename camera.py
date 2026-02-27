import io
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
from ultralytics import YOLO

app = Flask(__name__)
picam2 = Picamera2()

# 1. Load YOLOv8n (Nano)
# Class 39 is 'bottle' in the COCO dataset
model = YOLO('yolov8n.pt')

def setup_camera():
    # Maintain your preferred 720p 60FPS High-Speed configuration
    config = picam2.create_preview_configuration(
        main={"size": (1280, 720), "format": "BGR888"},
        controls={"FrameRate": 60} 
    )
    picam2.configure(config)

    try:
        # Optimization: HDR off for speed, Continuous AF for a moving robot
        picam2.set_controls({"HdrMode": 0}) 
        picam2.set_controls({"AfMode": 2}) 
    except Exception as e:
        print(f"Control error: {e}")
    
    picam2.start()
    print("AI BOTTLE COLLECTOR: Camera & YOLOv8n Initialized.")

setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Autonomous Robot AI Feed</title>
        <style>
            body { margin: 0; background: #000; color: #00ff00; font-family: monospace; overflow: hidden; }
            .stats { position: absolute; top: 10; left: 10; background: rgba(0,0,0,0.8); padding: 8px; border: 1px solid #0f0; z-index: 10; }
            img { width: 100vw; height: 100vh; object-fit: contain; }
            .target-box { color: #f1c40f; }
        </style>
    </head>
    <body>
        <div class="stats">
            AI: YOLOv8n | <span class="target-box">TARGET: PET BOTTLE</span><br>
            STREAM: 720p High-Speed | FOV: Wide
        </div>
        <img src="{{ url_for('video_feed') }}">
    </body>
</html>
"""

def generate_frames():
    while True:
        try:
            # Capture using the fast array method
            frame = picam2.capture_array()

            if frame is None:
                continue

            # 2. RUN AI INFERENCE
            # We filter for classes=[39] (Bottle) only
            # conf=0.3 is a good balance to avoid false positives
            results = model.predict(frame, classes=[39], conf=0.3, verbose=False)
            
            # Use results[0].plot() to automatically draw boxes and labels
            annotated_frame = results[0].plot()

            # Encode the resulting frame as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
            _, buffer = cv2.imencode('.jpg', annotated_frame, encode_param)
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
        except Exception as e:
            print(f"Stream error: {e}")
            break

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Host on 0.0.0.0 so you can access it from your laptop browser
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        print("\nStopping Robot Camera AI...")
    finally:
        try:
            picam2.stop()
            picam2.close()
            print("Resources released.")
        except:
            pass