import cv2
import numpy as np
import time
from picamera2 import Picamera2
from ultralytics import YOLO

# 1. Initialize Camera and YOLOv8
picam2 = Picamera2()
model = YOLO('yolov8n.pt')

def setup_camera():
    # Use 1280x720 60FPS for high potential and stability
    config = picam2.create_preview_configuration(
        main={"size": (1280, 720), "format": "BGR888"},
        controls={"FrameRate": 60} 
    )
    picam2.configure(config)
    try:
        picam2.set_controls({"HdrMode": 0, "AfMode": 2}) 
    except:
        pass
    picam2.start()
    print("Camera & YOLOv8 Active. Sending OpenCV UDP Stream...")

# 2. Setup GStreamer Writer
# REPLACE THIS IP with your laptop's IP address (found via ipconfig/ifconfig)
TARGET_IP = '192.168.18.7' 
TARGET_PORT = 5000

# Pipeline: Captures BGR -> Converts to YUV -> Encodes H264 (Hardware) -> Sends UDP
gst_str = (
    f'appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=2500 speed-preset=ultrafast ! '
    f'rtph264pay ! udpsink host={TARGET_IP} port={TARGET_PORT}'
)

out = cv2.VideoWriter(gst_str, cv2.CAP_GSTREAMER, 0, 30, (1280, 720), True)

setup_camera()

try:
    while True:
        # Capture the raw frame
        frame = picam2.capture_array()
        if frame is None:
            continue

        # Run YOLOv8 Inference (Class 39 = Bottle)
        results = model.predict(frame, classes=[39], conf=0.4, verbose=False)

        # Draw Custom High-Visibility Borders
        for r in results:
            for box in r.boxes:
                # Get coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Neon Green Border (Thick 4px)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 4)
                
                # Header box for label
                cv2.rectangle(frame, (x1, y1 - 35), (x1 + 160, y1), (0, 255, 0), -1)
                cv2.putText(frame, "PET BOTTLE", (x1 + 5, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        # Stream the frame to your laptop
        out.write(frame)

except KeyboardInterrupt:
    print("\nRobot Vision Stopping...")
finally:
    picam2.stop()
    picam2.close()
    out.release()