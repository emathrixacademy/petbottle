import io
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

app = Flask(__name__)
picam2 = Picamera2()

def setup_camera():
    # 1. High FPS Mode: 640x360 is extremely fast and keeps the 16:9 Wide FOV.
    # We set a high frame_rate in the configuration.
    config = picam2.create_preview_configuration(
        main={"size": (640, 360), "format": "BGR888"},
        controls={"FrameRate": 60} 
    )
    picam2.configure(config)

    # 2. Performance Tuning
    # Disable HDR for high FPS (HDR requires double exposure, which halves FPS)
    try:
        picam2.set_controls({"HdrMode": 0}) 
        # Set Focus to manual/fixed or fast continuous
        picam2.set_controls({"AfMode": 2}) 
    except Exception as e:
        print(f"Control error: {e}")
    
    picam2.start()
    print("Camera initialized in HIGH FPS mode (60 FPS targeted).")

setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Robot High-Speed Feed</title>
        <style>
            body { margin: 0; background: #000; color: #00ff00; font-family: monospace; overflow: hidden; }
            .stats { position: absolute; top: 10; left: 10; background: rgba(0,0,0,0.8); padding: 5px; border: 1px solid #0f0; }
            img { width: 100vw; height: 100vh; object-fit: contain; image-rendering: pixelated; }
        </style>
    </head>
    <body>
        <div class="stats">MODE: HIGH_FPS | FOV: WIDE | LATENCY: ULTRA_LOW</div>
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

            # Lower JPEG quality (60) to ensure the network can keep up with the high FPS
            # If the network chokes, the FPS will drop regardless of the camera speed.
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
            
            # Note: picamera2 capture_array usually returns RGB. 
            # We skip the color conversion if we want absolute max FPS, 
            # but if you look blue, keep the next line:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            _, buffer = cv2.imencode('.jpg', frame_bgr, encode_param)
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
            # No sleep here—let the hardware clock drive the loop!
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
        # Access at http://192.168.18.49:5000
        # 'threaded=True' is vital to keep the web server from blocking the camera loop
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        picam2.stop()
        picam2.close()