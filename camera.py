import io
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
from ultralytics import YOLO

# Initialize Flask and Camera
app = Flask(__name__)
picam2 = Picamera2()

# Load YOLOv8 Nano - This will download yolov8n.pt to your current folder
# Class 39 in the COCO dataset is 'bottle'
model = YOLO('yolov8n.pt')

def setup_camera():
    # 640x360 is the most efficient size for YOLOv8 on Pi 5
    # It maintains the 16:9 aspect ratio of the Wide Camera
    config = picam2.create_preview_configuration(
        main={"size": (640, 360), "format": "BGR888"},
        controls={"FrameRate": 30} 
    )
    picam2.configure(config)

    try:
        # High FPS usually works better with HDR off
        picam2.set_controls({"HdrMode": 0}) 
        # Continuous Autofocus is critical for a moving robot
        picam2.set_controls({"AfMode": 2}) 
    except Exception as e:
        print(f"Control setup warning: {e}")
    
    picam2.start()
    print("AI Robot Vision System: ACTIVE (No VENV)")

setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Autonomous Bottle Collector - AI Feed</title>
        <style>
            body { margin: 0; background: #111; color: #0f0; font-family: 'Courier New', monospace; text-align: center; }
            .container { position: relative; display: inline-block; width: 100%; }
            img { width: 100vw; height: auto; max-height: 85vh; border-bottom: 2px solid #0f0; }
            .telemetry { padding: 10px; background: #000; font-size: 1.2em; }
            .warn { color: yellow; font-size: 0.8em; }
        </style>
    </head>
    <body>
        <div class="container">
            <img src="{{ url_for('video_feed') }}">
        </div>
        <div class="telemetry">
            DETECTION: BOTTLES ONLY | FOV: WIDE | AI: YOLOv8n
        </div>
        <div class="warn">Running in System Environment (No VENV)</div>
    </body>
</html>
"""

def generate_frames():
    while True:
        try:
            # Capture the frame
            frame = picam2.capture_array()
            if frame is None:
                continue

            # Run YOLOv8 Inference
            # conf=0.4 filters out weak detections
            # classes=[39] ensures it only looks for bottles
            results = model.predict(frame, conf=0.4, classes=[39], verbose=False)
            
            # Use results[0].plot() to draw boxes on the frame
            annotated_frame = results[0].plot()

            # Encode the resulting frame as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            _, buffer = cv2.imencode('.jpg', annotated_frame, encode_param)
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
        except Exception as e:
            print(f"Error during stream: {e}")
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
        # Start server - accessible at http://192.168.18.49:5000
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down AI...")
    finally:
        try:
            picam2.stop()
            picam2.close()
        except:
            pass