import io
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

# Initialize Flask
app = Flask(__name__)

# Initialize Camera
picam2 = Picamera2()
# We explicitly request 'RGB' from the camera
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()

# Simple HTML Interface
HTML_PAGE = """
<html>
    <head>
        <title>Bottle Collector Robot - Fixed Color</title>
        <style>
            body { font-family: Arial; text-align: center; background: #1a1a1a; color: white; }
            img { border: 4px solid #2ecc71; border-radius: 8px; width: 80%; max-width: 800px; }
        </style>
    </head>
    <body>
        <h1>AUTONOMOUS BOTTLE COLLECTOR</h1>
        <p>Color Correction: <span style="color: #2ecc71;">NATURAL RGB</span></p>
        <img src="{{ url_for('video_feed') }}">
    </body>
</html>
"""

def generate_frames():
    while True:
        # 1. Capture frame (Coming in as RGB from the camera config)
        frame = picam2.capture_array()

        # 2. FIX: OpenCV expects BGR to encode JPEG correctly. 
        # Since the camera is giving us RGB, we swap them here.
        corrected_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # 3. Add text overlay (Optional)
        cv2.putText(corrected_frame, "BOTTLE_CAM_01", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 4. Encode the corrected frame
        _, buffer = cv2.imencode('.jpg', corrected_frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        print("Starting Corrected Robot Web Interface...")
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        picam2.stop()
        picam2.close()