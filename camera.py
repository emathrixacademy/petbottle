import io
import time
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

app = Flask(__name__)
picam2 = Picamera2()

def setup_camera():
    # 1. Access the full 16:9 wide sensor area
    # Note: 4608x2592 is the max, but for a web stream, 2304x1296 
    # provides a massive field of view without lagging the network.
    config = picam2.create_preview_configuration(main={"size": (2304, 1296)})
    picam2.configure(config)

    # 2. Enable HDR (High Dynamic Range)
    # This helps the robot see details in shadows and bright spots
    try:
        picam2.set_controls({"HdrMode": 1})
        print("HDR Mode Enabled")
    except Exception as e:
        print(f"HDR not supported or failed: {e}")

    # 3. Set Continuous Autofocus
    # Critical for a moving robot to keep objects sharp
    picam2.set_controls({"AfMode": 2}) # 2 is 'Continuous' (AfModeContinuous)
    
    picam2.start()
    print("Camera Module 3 Wide initialized at high potential.")

# Initialize hardware
setup_camera()

HTML_PAGE = """
<html>
    <head>
        <title>Wide-View Robot Eyes</title>
        <style>
            body { margin: 0; background: #111; color: #0f0; font-family: 'Courier New', monospace; text-align: center; }
            img { width: 100%; height: auto; border-bottom: 3px solid #0f0; }
            .telemetry { padding: 15px; display: grid; grid-template-columns: 1fr 1fr; text-align: left; max-width: 600px; margin: auto; }
            .label { color: #888; }
        </style>
    </head>
    <body>
        <img src="{{ url_for('video_feed') }}">
        <div class="telemetry">
            <div><span class="label">SENSOR:</span> IMX708 Wide</div>
            <div><span class="label">RES:</span> 2304x1296 (16:9)</div>
            <div><span class="label">HDR:</span> ACTIVE</div>
            <div><span class="label">AF:</span> CONTINUOUS</div>
        </div>
    </body>
</html>
"""

def generate_frames():
    while True:
        # Capture the high-res frame using the hardware encoder
        buf = io.BytesIO()
        picam2.capture_file(buf, format='jpeg')
        frame = buf.getvalue()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Access via http://<pi-ip>:5000
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        picam2.stop()
        picam2.close()