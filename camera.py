import io
import time
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

# Initialize Flask and Camera
app = Flask(__name__)
picam2 = Picamera2()

# HTML Template as a string for a single-file solution
HTML_PAGE = """
<html>
    <head>
        <title>Robot Eye - Bottle Collector</title>
        <style>
            body { font-family: sans-serif; text-align: center; background: #222; color: white; }
            img { border: 5px solid #444; border-radius: 10px; max-width: 90%; }
        </style>
    </head>
    <body>
        <h1>Pet Bottle Collector - Live Feed</h1>
        <img src="{{ url_for('video_feed') }}">
        <p>Status: <span style="color: #0f0;">Streaming Online</span></p>
    </body>
</html>
"""

def generate_frames():
    # Configure Camera Module 3
    # Use 640x480 for lower latency on the robot's network
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    try:
        while True:
            # Capture frame into a byte buffer
            buf = io.BytesIO()
            picam2.capture_file(buf, format='jpeg')
            frame = buf.getvalue()
            
            # Yield the frame in MJPEG format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            # Small sleep to prevent CPU Maxing
            time.sleep(0.01)
    finally:
        picam2.stop()
        picam2.close()

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # host='0.0.0.0' allows access from any device on your network
    try:
        print("Starting Robot Web Server...")
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nStopping Server...")