import io
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

app = Flask(__name__)
picam2 = Picamera2()

def setup_camera():
    # Use 16:9 for the Wide sensor's full field of view
    # 1280x720 is very stable for real-time robot navigation
    config = picam2.create_preview_configuration(main={"size": (1280, 720)})
    picam2.configure(config)

    # Enable HDR for better visibility in shadows/sunlight
    try:
        picam2.set_controls({"HdrMode": 1})
    except:
        pass

    # Enable Continuous Autofocus for a moving robot
    picam2.set_controls({"AfMode": 2})
    
    picam2.start()
    print("Camera Module 3 Wide: High-Potential Mode Active.")

setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Wide-View Robot Eyes</title>
        <style>
            body { margin: 0; background: #000; color: #0f0; font-family: monospace; text-align: center; }
            img { width: 100vw; height: auto; max-height: 85vh; border-bottom: 2px solid #0f0; }
            .telemetry { padding: 10px; font-size: 1.1em; background: #111; }
        </style>
    </head>
    <body>
        <img src="{{ url_for('video_feed') }}">
        <div class="telemetry">
            [ FOV: WIDE ] [ HDR: ON ] [ AF: CONT ] [ IP: 192.168.18.49 ]
        </div>
    </body>
</html>
"""

def generate_frames():
    while True:
        try:
            # Capture the frame as a NumPy array (RGB/BGR)
            # This is the 'Raw' data after the Pi 5's ISP processing
            frame = picam2.capture_array()

            if frame is None:
                continue

            # Add a small timestamp so you know the stream hasn't frozen
            timestamp = time.strftime("%H:%M:%S")
            cv2.putText(frame, timestamp, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Convert to JPEG using OpenCV to control quality and prevent glitching
            # [70] quality is the sweet spot for a clear but fast robot stream
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            
            # Since capture_array typically returns RGB for Picamera2, 
            # and OpenCV imencode expects BGR, we swap them.
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, buffer = cv2.imencode('.jpg', frame_bgr, encode_param)
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
            # Cap at ~20fps to keep the network pipeline clean
            time.sleep(0.05)

        except Exception as e:
            print(f"Stream Error: {e}")
            time.sleep(1) # Wait a bit before retrying

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Access on your laptop at http://192.168.18.49:5000
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down Robot Camera...")
    finally:
        picam2.stop()
        picam2.close()