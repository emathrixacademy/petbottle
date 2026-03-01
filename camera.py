#!/usr/bin/env python3
"""
PET Bottle Detection - Hailo AI Hat + Raspberry Pi Camera
Fixed postprocessing for actual output shapes:
  best/activation1: (1, 1, 3549, 1)  - confidence scores
  best/concat14:    (1, 1, 3549, 64) - box regression (DFL)
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
CONF_THRESHOLD = 0.3
INPUT_SIZE     = (416, 416)
CAMERA_SIZE    = (1280, 720)
CAMERA_FPS     = 60
REG_MAX        = 16   # DFL reg_max for YOLOv8
# ───────────────────────────────────────────────────────────

# YOLOv8 anchor grid for 416x416:
# stride 8  → 52x52 = 2704
# stride 16 → 26x26 = 676
# stride 32 → 13x13 = 169
# Total = 3549
def generate_anchors(input_size=416, strides=[8, 16, 32]):
    anchors = []
    for stride in strides:
        grid_size = input_size // stride
        for y in range(grid_size):
            for x in range(grid_size):
                anchors.append((x + 0.5, y + 0.5, stride))
    return anchors

ANCHORS = generate_anchors()


def dfl_decode(reg, reg_max=16):
    """Decode DFL box regression [N, 64] → ltrb distances [N, 4]"""
    N = reg.shape[0]
    reg = reg.reshape(N, 4, reg_max)
    reg = reg - reg.max(axis=-1, keepdims=True)
    exp = np.exp(reg)
    reg = exp / exp.sum(axis=-1, keepdims=True)
    weights = np.arange(reg_max, dtype=np.float32)
    ltrb = (reg * weights).sum(axis=-1)  # [N, 4]
    return ltrb


def postprocess(outputs, orig_w, orig_h):
    conf_raw = outputs.get("best/activation1")  # (1,1,3549,1)
    reg_raw  = outputs.get("best/concat14")      # (1,1,3549,64)

    if conf_raw is None or reg_raw is None:
        return []

    conf = conf_raw.reshape(-1)      # [3549]
    reg  = reg_raw.reshape(-1, 64)   # [3549, 64]

    ltrb = dfl_decode(reg, REG_MAX)  # [3549, 4]

    detections = []
    for i, (score, (l, t, r, b)) in enumerate(zip(conf, ltrb)):
        if score < CONF_THRESHOLD:
            continue

        cx, cy, stride = ANCHORS[i]

        # Pixel coords in input space
        x1 = (cx - l) * stride
        y1 = (cy - t) * stride
        x2 = (cx + r) * stride
        y2 = (cy + b) * stride

        # Scale to original frame
        sx = orig_w / INPUT_SIZE[0]
        sy = orig_h / INPUT_SIZE[1]
        x1 = int(x1 * sx)
        y1 = int(y1 * sy)
        x2 = int(x2 * sx)
        y2 = int(y2 * sy)

        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(orig_w, x2), min(orig_h, y2)

        if x2 <= x1 or y2 <= y1:
            continue

        detections.append([x1, y1, x2, y2, float(score), 0])

    if not detections:
        return []

    # NMS
    boxes  = [[d[0], d[1], d[2]-d[0], d[3]-d[1]] for d in detections]  # xywh for NMS
    scores = [d[4] for d in detections]
    indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESHOLD, 0.45)

    if len(indices) == 0:
        return []
    return [detections[i] for i in indices.flatten()]


def draw_detections(frame, detections):
    for (x1, y1, x2, y2, conf, cls_id) in detections:
        label = CLASS_NAMES[int(cls_id)] if int(cls_id) < len(CLASS_NAMES) else f"cls{cls_id}"
        color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 4)
        text = f"{label.upper()} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x1, y1 - 25), (x1 + tw + 10, y1), color, -1)
        cv2.putText(frame, text, (x1 + 5, y1 - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return frame


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


def main():
    print(f"Loading HEF: {HEF_PATH}")
    hef = HEF(HEF_PATH)

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

        picam2 = setup_camera()
        input_name = hef.get_input_vstream_infos()[0].name

        window_name = "ROBOT VISION - HAILO AI HAT PET BOTTLE DETECTION"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
            with network_group.activate(network_group_params):
                print("✅ Hailo AI Hat running! Press 'q' to quit.")

                try:
                    while True:
                        start_time = time.time()

                        frame = picam2.capture_array()
                        if frame is None:
                            continue

                        orig_h, orig_w = frame.shape[:2]

                        # Preprocess
                        img = cv2.resize(frame, INPUT_SIZE)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        img = img.astype(np.float32) / 255.0
                        input_data = np.expand_dims(img, axis=0)

                        # Infer
                        raw_outputs = infer_pipeline.infer({input_name: input_data})

                        # Postprocess
                        detections = postprocess(raw_outputs, orig_w, orig_h)

                        # Draw
                        frame = draw_detections(frame, detections)

                        # Overlay
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