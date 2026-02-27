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

# Step 1: Explicitly configure for a standard format
# We use 'BGR888' because it's OpenCV's native language. 
# This avoids needing to swap channels manually later.
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
picam2.configure(config)
picam2.start()

print("Camera started. Waiting 2 seconds for Auto White Balance to calibrate...")
time.sleep(2) # Give the sensor time to realize you aren't actually blue

# Simple HTML Interface
HTML_PAGE = """
<html>
    <head>
        <title>Robot Collector - Color Fixed</title>
        <style>
            body { font-family: Arial, sans-serif; text-align: center; background: #1a1a1a; color: white; }
            .feed-container { margin-top: 20px; }
            img { border: 5px solid #2ecc71; border-radius: 12px; width: 80%; max-width: 800px; box-shadow: 0 0 20px rgba(0,0,0,0.5); }
            .info { color: #bdc3c7; font-size: 0.9em; margin-top: 10px; }
        </style>
    </head>
    <body>
        <h1>AUTONOMOUS BOTTLE COLLECTOR</h1>
        <div class="feed-container">
            <img src="{{ url_for('video_feed') }}">
        </div>
        <div class="info">Sensor: IMX708 | Mode: BGR-Native | AWB: Auto</div>
    </body>
</html>
"""

def generate_frames():
    while True:
        # Since we configured the camera to output BGR888, 
        # picam2.capture_array() returns a format OpenCV loves.
        frame = picam2.capture_array()

        # Check if frame is empty
        if frame is None:
            continue

        # Add a "Natural Color" label to the stream
        cv2.putText(frame, "COLOR_OK", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Encode the frame as JPEG
        # The JPEG encoder expects BGR by default in the OpenCV implementation
        success, buffer = cv2.imencode('.jpg', frame)
        if not success:
            continue
            
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
        print("Web server running on http://0.0.0.0:5000")
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        picam2.stop()
        picam2.close()