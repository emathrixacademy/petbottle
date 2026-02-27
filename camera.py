import io
import time
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

app = Flask(__name__)
picam2 = Picamera2()

def setup_camera():
    # We keep the Wide FOV (16:9) but use a resolution that 
    # doesn't overwhelm the network (1536x864).
    # This is still much higher than standard 720p.
    config = picam2.create_preview_configuration(main={"size": (1536, 864)})
    picam2.configure(config)

    # Enable HDR for the wide sensor
    try:
        picam2.set_controls({"HdrMode": 1})
    except:
        pass

    # Continuous Autofocus is a must for a robot
    picam2.set_controls({"AfMode": 2})
    
    picam2.start()
    print("Camera Ready: Wide FOV Optimized.")

setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Robot Collector - Smooth Stream</title>
        <style>
            body { margin: 0; background: #000; display: flex; flex-direction: column; align-items: center; }
            img { width: 100vw; height: auto; max-width: 1200px; border-bottom: 2px solid #2ecc71; }
            .telemetry { color: #2ecc71; font-family: monospace; padding: 10px; }
        </style>
    </head>
    <body>
        <img src="{{ url_for('video_feed') }}">
        <div class="telemetry">MODE: WIDE-STABLE | HDR: ON | AF: CONT</div>
    </body>
</html>
"""

def generate_frames():
    while True:
        buf = io.BytesIO()
        # We set quality=70 to reduce the 'glitching' caused by network lag
        # This makes the file size much smaller while keeping the FOV
        picam2.capture_file(buf, format='jpeg', quality=70)
        frame = buf.getvalue()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
        # Limit to ~20-25 FPS to prevent the browser from choking
        time.sleep(0.04)

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Use threaded=True to handle the web requests smoothly
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        picam2.close()