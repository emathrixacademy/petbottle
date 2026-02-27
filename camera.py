import io
import time
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

# Initialize Flask
app = Flask(__name__)

# Initialize Picamera2
picam2 = Picamera2()

# Configure for "Raw" viewable stream
# We use the 'main' stream for high quality and 'mjpeg' for the web
config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "BGR888"},
    video={"size": (1280, 720), "format": "mjpeg"}
)
picam2.configure(config)
picam2.start()

# HTML for the Robot Interface
HTML_PAGE = """
<html>
    <head>
        <title>Raw Robot Feed</title>
        <style>
            body { margin: 0; background: #000; color: #0f0; font-family: monospace; text-align: center; }
            img { width: 100vw; height: auto; max-height: 90vh; object-fit: contain; border-bottom: 2px solid #0f0; }
            .stats { padding: 10px; font-size: 1.2em; }
        </style>
    </head>
    <body>
        <img src="{{ url_for('video_feed') }}">
        <div class="stats">RAW STREAM: 1280x720 @ 30FPS | SOURCE: CAM/DISP 0</div>
    </body>
</html>
"""

def generate_raw_frames():
    try:
        while True:
            # capture_file with format='mjpeg' pulls the raw encoded 
            # frame directly from the hardware encoder
            buf = io.BytesIO()
            picam2.capture_file(buf, format='jpeg')
            frame = buf.getvalue()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    except Exception as e:
        print(f"Stream Error: {e}")
    finally:
        # Cleanup is handled by the main thread, but safety first
        pass

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_raw_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        print("Launching RAW robot stream on http://0.0.0.0:5000")
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down camera...")
    finally:
        picam2.stop()
        picam2.close()