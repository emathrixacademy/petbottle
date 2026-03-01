#!/usr/bin/env python3
"""
PET Bottle Detection - Hailo AI Hat + Raspberry Pi Camera
Replaces YOLO CPU inference with Hailo HEF acceleration
"""

import cv2
import numpy as np
import time
from picamera2 import Picamera2
from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams, FormatType
)

# ── Settings ───────────────────────────────────────────────
HEF_PATH       = "best.hef"
CLASS_NAMES    = ["PET-Bottle"]
CONF_THRESHOLD = 0.4
INPUT_SIZE     = (416, 416)   # must match HEF compile size
CAMERA_SIZE    = (1280, 720)
CAMERA_FPS     = 60
# ───────────────────────────────────────────────────────────


def setup_camera():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": CAMERA_SIZE, "format": "BGR888"},
        controls={"FrameRate": CAMERA_FPS}
    )
    picam2.configure(config)
    try:
        picam2.set_controls({"HdrMode": 0, "AfMode": 2})
    except Exception as e:
        print(f"Camera control warning: {e}")
    picam2.start()
    print("✅ Camera started")
    return picam2


def preprocess(frame):
    """Resize + normalize to float32 [0,1] for Hailo"""
    img = cv2.resize(frame, INPUT_SIZE)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    return np.expand_dims(img, axis=0)  # [1, H, W, C]


def postprocess(outputs, orig_w, orig_h):
    """Convert raw Hailo output to bounding boxes"""
    detections = []
    for name, data in outputs.items():
        data = np.squeeze(data)

        if data.ndim == 2:
            # Transpose if shape is [5, N] → [N, 5]
            if data.shape[0] <= (4 + len(CLASS_NAMES)):
                data = data.T
            for det in data:
                if len(det) < 5:
                    continue
                x_c, y_c, w, h = det[0], det[1], det[2], det[3]
                scores = det[4:]
                conf = float(np.max(scores))
                cls_id = int(np.argmax(scores))
                if conf < CONF_THRESHOLD:
                    continue
                x1 = int((x_c - w / 2) * orig_w)
                y1 = int((y_c - h / 2) * orig_h)
                x2 = int((x_c + w / 2) * orig_w)
                y2 = int((y_c + h / 2) * orig_h)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(orig_w, x2), min(orig_h, y2)
                detections.append((x1, y1, x2, y2, conf, cls_id))

        elif data.ndim == 3:
            c, fh, fw = data.shape
            data = data.reshape(c, -1).T
            for det in data:
                if len(det) < 5:
                    continue
                x_c, y_c, bw, bh = det[0], det[1], det[2], det[3]
                scores = det[4:]
                conf = float(np.max(scores))
                cls_id = int(np.argmax(scores))
                if conf < CONF_THRESHOLD:
                    continue
                x1 = int((x_c - bw / 2) * orig_w)
                y1 = int((y_c - bh / 2) * orig_h)
                x2 = int((x_c + bw / 2) * orig_w)
                y2 = int((y_c + bh / 2) * orig_h)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(orig_w, x2), min(orig_h, y2)
                detections.append((x1, y1, x2, y2, conf, cls_id))

    return detections


def draw_detections(frame, detections):
    for (x1, y1, x2, y2, conf, cls_id) in detections:
        label = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f"cls{cls_id}"
        color = (0, 255, 0)

        # Thick bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 4)

        # Label background + text
        text = f"{label.upper()} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x1, y1 - 25), (x1 + tw + 10, y1), color, -1)
        cv2.putText(frame, text, (x1 + 5, y1 - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return frame


def main():
    # Load HEF
    print(f"Loading HEF: {HEF_PATH}")
    hef = HEF(HEF_PATH)

    # Setup Hailo AI Hat
    with VDevice() as target:
        configure_params = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe
        )
        network_groups = target.configure(hef, configure_params)
        network_group = network_groups[0]
        network_group_params = network_group.create_params()

        input_vstreams_params = InputVStreamParams.make(
            network_group, format_type=FormatType.FLOAT32
        )
        output_vstreams_params = OutputVStreamParams.make(
            network_group, format_type=FormatType.FLOAT32
        )

        # Setup camera
        picam2 = setup_camera()
        input_name = hef.get_input_vstream_infos()[0].name

        window_name = "ROBOT VISION - HAILO AI HAT PET BOTTLE DETECTION"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
            with network_group.activate(network_group_params):
                print("✅ Hailo AI Hat inference running! Press 'q' to quit.")

                try:
                    while True:
                        start_time = time.time()

                        # Capture
                        frame = picam2.capture_array()
                        if frame is None:
                            continue

                        orig_h, orig_w = frame.shape[:2]

                        # Preprocess → infer → postprocess
                        input_data = preprocess(frame)
                        raw_outputs = infer_pipeline.infer({input_name: input_data})
                        detections = postprocess(raw_outputs, orig_w, orig_h)

                        # Draw boxes
                        frame = draw_detections(frame, detections)

                        # FPS + detection count overlay
                        fps = 1.0 / (time.time() - start_time)
                        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(frame, f"Detections: {len(detections)}", (20, 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                        cv2.putText(frame, "Hailo AI Hat", (20, 120),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

                        cv2.imshow(window_name, frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                except KeyboardInterrupt:
                    print("\nShutting down AI...")
                finally:
                    try:
                        picam2.stop()
                        picam2.close()
                    except:
                        pass
                    cv2.destroyAllWindows()
                    print("✅ Done.")


if __name__ == "__main__":
    main()