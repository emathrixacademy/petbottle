import io
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

app = Flask(__name__)
picam2 = Picamera2()

def setup_camera():
    # 1. High FPS Mode: 1280x720 is a native binned mode for Module 3.
    # This is often MORE stable for the Pi 5 than smaller custom resolutions.
    config = picam2.create_preview_configuration(
        main={"size": (1280, 720), "format": "BGR888"},
        controls={"FrameRate": 60} 
    )
    picam2.configure(config)

    # 2. Performance Tuning
    try:
        # Disable HDR for high FPS stability
        picam2.set_controls({"HdrMode": 0}) 
        # Set Continuous Autofocus
        picam2.set_controls({"AfMode": 2}) 
    except Exception as e:
        print(f"Control error: {e}")
    
    picam2.start()
    print("Camera initialized in HIGH FPS mode (1280x720 @ 60 FPS).")

setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Robot High-Speed Feed</title>
        <style>
            body { margin: 0; background: #000; color: #00ff00; font-family: monospace; overflow: hidden; }
            .stats { position: absolute; top: 10; left: 10; background: rgba(0,0,0,0.8); padding: 5px; border: 1px solid #0f0; z-index: 10; }
            img { width: 100vw; height: 100vh; object-fit: contain; }
        </style>
    </head>
    <body>
        <div class="stats">MODE: HIGH_FPS | RES: 720p | FOV: WIDE</div>
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

            # JPEG Quality 60 reduces network congestion
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
            
            # Note: Picamera2 capture_array returns RGB, but imencode wants BGR.
            # Keeping the swap to ensure skin/bottles look natural.
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            _, buffer = cv2.imencode('.jpg', frame_bgr, encode_param)
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
        except Exception as e:
            print(f"Stream error: {e}")
            # If the camera frontend times out, we break the loop to allow a clean exit
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
        # Host on all interfaces for robot accessibility
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        print("\nStopping Robot Camera...")
    finally:
        # Wrapped in a try/except to prevent hanging on a timed-out camera
        try:
            picam2.stop()
            picam2.close()
            print("Camera resources released.")
        except:
            print("Camera was already unresponsive.")