import cv2
import numpy as np
import time
from picamera2 import Picamera2
from ultralytics import YOLO

# 1. Initialize YOLOv8n (Nano) 
# Class 39 is 'bottle' in the COCO dataset
model = YOLO('yolov8n.pt')

# 2. Initialize Camera
picam2 = Picamera2()

def setup_camera():
    # Using 1280x720 for a perfect balance of Wide FOV and AI speed
    config = picam2.create_preview_configuration(
        main={"size": (1280, 720), "format": "BGR888"},
        controls={"FrameRate": 60} 
    )
    picam2.configure(config)
    
    try:
        # Optimization: HDR off for raw speed, Continuous AF for robot movement
        picam2.set_controls({"HdrMode": 0, "AfMode": 2})
    except Exception as e:
        print(f"Control setup warning: {e}")
        
    picam2.start()
    print("AI Vision System Started Locally.")

setup_camera()

# Create a named window and set it to full screen if desired
window_name = "AUTONOMOUS ROBOT - AI VISION"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

try:
    while True:
        start_time = time.time()
        
        # Capture frame directly into NumPy array
        frame = picam2.capture_array()
        if frame is None:
            continue

        # Inference - Only detect Bottles (Class 39)
        # conf=0.4 avoids flickering/false detections
        results = model.predict(frame, classes=[39], conf=0.4, verbose=False)

        # Draw Custom Thick Border around detections
        for r in results:
            for box in r.boxes:
                # Bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Neon Green Border (Thick 4px)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 4)
                
                # Label Header
                cv2.rectangle(frame, (x1, y1 - 35), (x1 + 170, y1), (0, 255, 0), -1)
                cv2.putText(frame, "PET BOTTLE", (x1 + 5, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        # Calculate FPS
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # --- LOCAL DISPLAY ---
        cv2.imshow(window_name, frame)

        # Press 'q' on the keyboard to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nShutting down AI...")
finally:
    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()