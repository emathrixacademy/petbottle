import cv2
import numpy as np
import time
from picamera2 import Picamera2
from ultralytics import YOLO

# 1. Initialize YOLOv8n (Nano) 
# This model detects 80 different classes (bottles, people, cars, etc.)
model = YOLO('yolov8n.pt')

# 2. Initialize Camera
picam2 = Picamera2()

def setup_camera():
    # 1280x720 @ 60FPS: Best for High-Speed AI performance on Pi 5
    config = picam2.create_preview_configuration(
        main={"size": (1280, 720), "format": "BGR888"},
        controls={"FrameRate": 60} 
    )
    picam2.configure(config)
    
    try:
        # Optimization: HDR off for max FPS, Continuous AF for robot movement
        picam2.set_controls({"HdrMode": 0, "AfMode": 2})
    except Exception as e:
        print(f"Control setup warning: {e}")
        
    picam2.start()
    print("AI Vision System Started: Detecting All Objects (Local Display)")

setup_camera()

# Create the display window
window_name = "ROBOT VISION - HIGH FPS ALL-OBJECT DETECTION"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

try:
    while True:
        start_time = time.time()
        
        # Capture frame
        frame = picam2.capture_array()
        if frame is None:
            continue

        # --- AI INFERENCE ---
        # No 'classes' filter = Detects all 80 categories
        # conf=0.3 is lower to show more potential objects
        results = model.predict(frame, conf=0.3, verbose=False)

        # Draw Custom Thick Borders for all detected objects
        for r in results:
            for box in r.boxes:
                # Coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Get class name and confidence
                cls_id = int(box.cls[0])
                label = model.names[cls_id].upper()
                conf = float(box.conf[0])
                
                # Assign a unique color based on the class ID for better variety
                color = (int((cls_id * 50) % 255), 255, int((cls_id * 100) % 255))
                
                # Thick Border (4px)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 4)
                
                # Label Background and Text
                text = f"{label} {conf:.2f}"
                (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, y1 - 25), (x1 + w + 10, y1), color, -1)
                cv2.putText(frame, text, (x1 + 5, y1 - 7), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Calculate and Display FPS
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show local window
        cv2.imshow(window_name, frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nShutting down AI...")
finally:
    # Safe cleanup
    try:
        picam2.stop()
        picam2.close()
    except:
        pass
    cv2.destroyAllWindows()