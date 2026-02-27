import io
import time
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

app = Flask(__name__)
picam2 = Picamera2()

def setup_camera():
    # Maintain Wide FOV (16:9)
    # Using 1536x864 is a great balance of "Full Potential" and "Network Stability"
    config = picam2.create_preview_configuration(main={"size": (1536, 864)})
    picam2.configure(config)

    # Enable HDR for difficult lighting
    try:
        picam2.set_controls({"HdrMode": 1})
    except:
        pass

    # Enable Continuous Autofocus for the moving robot
    picam2.set_controls({"AfMode": 2})
    
    picam2.start()
    print("Camera Module 3 Wide: Optimized & Ready.")

setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Robot Collector - Smooth Wide Stream</title>
        <style>
            body { margin: 0; background: #000; display: flex; flex-direction: column; align-items: center; color: #0f0; }
            img { width: 100vw; height: auto; max-width: 1280px; border-bottom: 2px solid #2ecc71; }
            .telemetry { font-family: 'Courier New', monospace; padding: 15px; background: rgba(0,255,0,0.1); width: 100%; text-align: center; }
        </style>
    </head>
    <body>
        <img src="{{ url_for('video_feed') }}">
        <div class="telemetry">
            SYSTEM: ACTIVE | FOV: WIDE (16:9) | HDR: ON | AF: CONTINUOUS
        </div>
    </body>
</html>
"""

def generate_frames():
    while True:
        try:
            buf = io.BytesIO()
            # FIX: Quality is passed via the 'options' dictionary in capture_file
            # Reducing quality to 70 prevents the network buffer 'glitching'
            picam2.capture_file(buf, format='jpeg', options={'quality': 70})
            frame = buf.getvalue()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            # 20 FPS cap to keep the robot's Wi-Fi connection stable
            time.sleep(0.05)
        except Exception as e:
            print(f"Frame generation error: {e}")
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
        # Use threaded=True to ensure the UI remains responsive
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nRobot shutting down...")
    finally:
        picam2.stop()
        picam2.close()