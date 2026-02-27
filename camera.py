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
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# Simple HTML Interface
HTML_PAGE = """
<html>
    <head>
        <title>Bottle Collector Robot Feed</title>
        <style>
            body { font-family: Arial; text-align: center; background: #1a1a1a; color: white; }
            .container { margin-top: 20px; }
            img { border: 4px solid #3498db; border-radius: 8px; width: 80%; max-width: 800px; }
            .status { color: #2ecc71; font-weight: bold; }
        </style>
    </head>
    <body>
        <h1>AUTONOMOUS BOTTLE COLLECTOR</h1>
        <div class="status">● LIVE FEED ACTIVE</div>
        <div class="container">
            <img src="{{ url_for('video_feed') }}">
        </div>
        <p>Connected to: CAM/DISP 0 (IMX708)</p>
    </body>
</html>
"""

def generate_frames():
    while True:
        # Capture frame as a NumPy array (OpenCV format)
        # This allows you to add detection logic later in this loop
        frame = picam2.capture_array()

        # Optional: Add a timestamp or text to the frame using OpenCV
        cv2.putText(frame, time.strftime("%H:%M:%S"), (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Encode the frame as JPEG
        _, buffer = cv2.imencode('.jpg', frame)
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
        print("Starting Robot Web Interface...")
        # Host 0.0.0.0 makes it visible to your laptop
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        picam2.stop()
        picam2.close()