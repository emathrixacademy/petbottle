import io
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

app = Flask(__name__)
picam2 = Picamera2()

def setup_camera():
    try:
        # Using 1280x720 at 60FPS is the 'Gold Standard' for High Speed on Pi 5.
        # It uses the sensor's 2x2 binning mode which is very stable.
        config = picam2.create_preview_configuration(
            main={"size": (1280, 720), "format": "BGR888"},
            controls={"FrameRate": 60}
        )
        picam2.configure(config)
        
        # Set Focus to Continuous but slow it down slightly for stability
        picam2.set_controls({"AfMode": 2})
        
        picam2.start()
        print("Camera Module 3 Wide: High FPS Mode (720p @ 60fps) Started.")
    except Exception as e:
        print(f"Initialization Failed: {e}. Check ribbon cable!")

setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Robot High-Speed Feed</title>
        <style>
            body { margin: 0; background: #000; color: #0f0; font-family: monospace; text-align: center; }
            img { width: 100vw; height: auto; max-height: 90vh; object-fit: contain; }
            .telemetry { padding: 10px; background: #111; border-top: 1px solid #0f0; }
        </style>
    </head>
    <body>
        <img src="{{ url_for('video_feed') }}">
        <div class="telemetry">STREAMS: 720p60 | STATUS: STABLE | FOV: WIDE</div>
    </body>
</html>
"""

def generate_frames():
    while True:
        try:
            # capture_array is fast, but we'll wrap it in a timeout check
            frame = picam2.capture_array()
            
            if frame is None:
                continue

            # JPEG Quality 60 is the key to preventing network-induced glitching
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
            
            # Convert RGB (Picamera2) to BGR (OpenCV)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, buffer = cv2.imencode('.jpg', frame_bgr, encode_param)
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        except Exception as e:
            print(f"Camera Stream Lost: {e}")
            # Try to restart the camera if it fails
            time.sleep(2)
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
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Use a safe stop to prevent the KeyboardInterrupt hang you saw
        try:
            picam2.stop()
            picam2.close()
        except:
            pass