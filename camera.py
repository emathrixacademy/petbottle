#!/usr/bin/env python3
"""
PET Bottle detection with COCO yolov8s HEF (bottle class 39)
Usage:
  python3 camera.py                          # live camera (default)
  python3 camera.py --camera 0               # choose camera index
  python3 camera.py --image sample.jpg       # run on single image
  python3 camera.py --images ./test_images/  # run on image folder
  python3 camera.py --conf 0.20              # set confidence threshold
  python3 camera.py --no-show                # headless / save only
"""

import cv2
import numpy as np
import argparse
import time
from pathlib import Path
from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams, FormatType
)

# ── Settings ───────────────────────────────────────────────
HEF_PATH       = "yolov8s-coco.hef"
COCO_BOTTLE_ID = 39
CONF_THRESHOLD = 0.20
RESULTS_DIR    = "./results"
SMOOTH_FRAMES  = 10
# ───────────────────────────────────────────────────────────


def postprocess_coco(outputs, orig_w, orig_h, conf_thresh):
    """NMS-format postprocessor — keeps bottle class only.
    Uses a lower threshold for large (close-range) detections because
    COCO models score close/large objects lower than mid-range ones.
    """
    nms_key = next((k for k in outputs if 'nms' in k.lower() or 'output' in k.lower()), None)
    if nms_key is None:
        return []
    batch = outputs[nms_key][0]  # shape: [num_classes, max_det, 5]
    if COCO_BOTTLE_ID >= len(batch):
        return []

    frame_area = orig_w * orig_h
    detections = []
    for det in batch[COCO_BOTTLE_ID]:
        score = float(det[4])
        if score <= 0:
            continue
        # det: [y1, x1, y2, x2, score] normalised 0-1
        y1 = int(det[0] * orig_h); x1 = int(det[1] * orig_w)
        y2 = int(det[2] * orig_h); x2 = int(det[3] * orig_w)
        if x2 <= x1 or y2 <= y1:
            continue

        box_area = (x2 - x1) * (y2 - y1)
        fill     = box_area / frame_area  # 0.0–1.0

        # Close-range: bottle fills large portion — lower threshold aggressively
        if fill > 0.40:
            effective_thresh = conf_thresh * 0.25
        elif fill > 0.20:
            effective_thresh = conf_thresh * 0.50
        else:
            effective_thresh = conf_thresh

        if score < effective_thresh:
            continue
        detections.append([x1, y1, x2, y2, score])
    return detections


def get_orientation(x1, y1, x2, y2, frame_h):
    """
    Orientation detection for 1.5L and 500ml PET bottles.
    Aspect ratio is the primary and authoritative signal.
      - ratio > 1.15  → laydown (wider than tall)
      - ratio < 0.85  → upright (taller than wide)
      - 0.85–1.15     → ambiguous, use vertical fill as tiebreaker
    """
    w, h = x2 - x1, y2 - y1
    if w == 0 or h == 0:
        return "upright"
    ratio = w / h
    if ratio > 1.15:
        return "laydown"
    if ratio < 0.85:
        return "upright"

    # Ambiguous zone — close bottles fill most of frame so vertical_fill is
    # unreliable. Fall back to pure aspect ratio with a small margin.
    box_fill = (w * h) / (frame_h * frame_h)  # approx fill ratio
    if box_fill > 0.25:
        # Close-range: trust aspect ratio only
        return "laydown" if ratio > 1.0 else "upright"

    remaining = frame_h - y1
    vertical_fill = h / remaining if remaining > 0 else 1.0
    return "upright" if vertical_fill > 0.4 else "laydown"


def draw_orientation_arrow(frame, x1, y1, x2, y2, orientation, color):
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    arm = min(x2 - x1, y2 - y1) // 3
    if orientation == "upright":
        cv2.arrowedLine(frame, (cx, cy + arm), (cx, cy - arm), color, 2, tipLength=0.4)
    else:
        cv2.arrowedLine(frame, (cx - arm, cy), (cx + arm, cy), color, 2, tipLength=0.4)


def draw_detections(frame, detections):
    frame_h = frame.shape[0]
    for (x1, y1, x2, y2, conf) in detections:
        orientation = get_orientation(x1, y1, x2, y2, frame_h)
        tag   = "LAYDOWN" if orientation == "laydown" else "UPRIGHT"
        color = (0, 165, 255) if orientation == "laydown" else (0, 255, 0)
        text  = f"BOTTLE {tag}  {conf:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        draw_orientation_arrow(frame, x1, y1, x2, y2, orientation, color)

        (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        badge_y = max(y1 - th - baseline - 8, 0)
        cv2.rectangle(frame, (x1, badge_y), (x1 + tw + 10, y1), color, -1)
        cv2.putText(frame, text, (x1 + 5, y1 - baseline - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
    return frame


class DetectionTracker:
    """Keeps detections alive for SMOOTH_FRAMES after last seen — prevents flicker."""
    def __init__(self, max_age=SMOOTH_FRAMES, iou_thresh=0.35):
        self.max_age    = max_age
        self.iou_thresh = iou_thresh
        self.tracked    = []  # [x1,y1,x2,y2,conf,age]

    def _iou(self, a, b):
        ix1 = max(a[0], b[0]); iy1 = max(a[1], b[1])
        ix2 = min(a[2], b[2]); iy2 = min(a[3], b[3])
        iw = max(0, ix2 - ix1); ih = max(0, iy2 - iy1)
        inter = iw * ih
        union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
        return inter / union if union > 0 else 0.0

    def update(self, detections):
        for t in self.tracked:
            t[5] += 1
        for det in detections:
            best_iou, best_idx = 0, -1
            for i, t in enumerate(self.tracked):
                iou = self._iou(det, t)
                if iou > best_iou:
                    best_iou, best_idx = iou, i
            if best_iou >= self.iou_thresh:
                self.tracked[best_idx][:5] = list(det)
                self.tracked[best_idx][5]  = 0
            else:
                self.tracked.append(list(det) + [0])
        self.tracked = [t for t in self.tracked if t[5] <= self.max_age]
        return [[t[0], t[1], t[2], t[3], t[4]] for t in self.tracked]


def preprocess(frame, is_rgb=False, input_size=None):
    size = input_size if input_size else (640, 640)
    img = cv2.resize(frame, size)
    if not is_rgb:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return np.expand_dims(img.astype(np.uint8), axis=0)


def infer_multiscale(infer_pipeline, input_name, frame, orig_w, orig_h, conf_thresh, input_size):
    """
    Run inference at 3 scales by padding/zooming the frame.
    Close-range bottles (filling most of frame) appear as normal-sized
    bottles in the zoomed-out passes, giving the model a familiar view.

    Scales:
      1.0 — normal (original frame)
      0.6 — zoom out: bottle appears smaller, more of scene visible
      0.4 — zoom out more: works for very close bottles
    """
    all_dets = []

    for scale in [1.0, 0.6, 0.4]:
        if scale == 1.0:
            img = frame
        else:
            # Shrink frame and pad with black to fill input_size
            new_w = int(orig_w * scale)
            new_h = int(orig_h * scale)
            small = cv2.resize(frame, (new_w, new_h))
            img   = np.zeros((orig_h, orig_w, 3), dtype=np.uint8)
            pad_x = (orig_w - new_w) // 2
            pad_y = (orig_h - new_h) // 2
            img[pad_y:pad_y+new_h, pad_x:pad_x+new_w] = small

        input_data = preprocess(img, input_size=input_size)
        outputs    = infer_pipeline.infer({input_name: input_data})
        dets       = postprocess_coco(outputs, orig_w, orig_h, conf_thresh)

        if scale < 1.0:
            # Transform boxes back from padded coords to original frame coords
            pad_x = (orig_w - int(orig_w * scale)) // 2
            pad_y = (orig_h - int(orig_h * scale)) // 2
            mapped = []
            for (x1, y1, x2, y2, score) in dets:
                rx1 = int((x1 - pad_x) / scale)
                ry1 = int((y1 - pad_y) / scale)
                rx2 = int((x2 - pad_x) / scale)
                ry2 = int((y2 - pad_y) / scale)
                rx1 = max(0, rx1); ry1 = max(0, ry1)
                rx2 = min(orig_w, rx2); ry2 = min(orig_h, ry2)
                if rx2 > rx1 and ry2 > ry1:
                    mapped.append([rx1, ry1, rx2, ry2, score])
            dets = mapped

        all_dets.extend(dets)

    # NMS across all scales
    if not all_dets:
        return []
    boxes  = [[d[0], d[1], d[2]-d[0], d[3]-d[1]] for d in all_dets]
    scores = [d[4] for d in all_dets]
    idx    = cv2.dnn.NMSBoxes(boxes, scores, conf_thresh * 0.25, 0.4)
    return [all_dets[i] for i in (idx.flatten() if len(idx) else [])]


def overlay_info(frame, detections, ms, extra=""):
    h = frame.shape[0]
    status = f"DETECTED ({len(detections)})" if detections else "NONE"
    color  = (0, 255, 0) if detections else (0, 200, 255)
    cv2.putText(frame, status, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    if ms > 0:
        cv2.putText(frame, f"Infer: {ms:.1f}ms  {1000/ms:.1f}FPS",
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)
    if extra:
        cv2.putText(frame, extra, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
    return frame


def run_camera(infer_pipeline, input_name, conf_thresh, camera_index, no_show, out_dir, input_size):
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(
            main={"format": "BGR888", "size": (640, 480)}
        ))
        picam2.start()
        use_picamera2 = True
        print("  Using picamera2 (CSI ribbon)")
    except Exception as e:
        print(f"  picamera2 not available ({e}), falling back to V4L2...")
        use_picamera2 = False
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            print(f"Cannot open camera (index={camera_index})")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not no_show:
        cv2.namedWindow("PET Bottle Detection", cv2.WINDOW_NORMAL)

    print(f"\n{'='*55}")
    print(f"  Live Camera | conf={conf_thresh} | Q to quit")
    print(f"{'='*55}\n")

    frame_idx = 0
    ms        = 0.0
    tracker   = DetectionTracker()

    try:
        while True:
            if use_picamera2:
                frame_rgb = picam2.capture_array()
                frame     = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cap.read()
                if not ret:
                    print("  Frame capture failed")
                    break

            orig_h, orig_w = frame.shape[:2]

            t0 = time.time()
            raw_dets = infer_multiscale(infer_pipeline, input_name, frame,
                                        orig_w, orig_h, conf_thresh, input_size)
            ms = (time.time() - t0) * 1000

            detections = tracker.update(raw_dets)

            result = draw_detections(frame.copy(), detections)
            overlay_info(result, detections, ms, f"Frame {frame_idx}")

            icon = "OK" if detections else "--"
            print(f"\r  [{icon}] frame={frame_idx:5d} | {len(detections):2d} det | {ms:.1f}ms", end="", flush=True)

            if not no_show:
                cv2.imshow("PET Bottle Detection", result)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), ord('Q'), 27):
                    break
                if key == ord('s'):
                    snap = out_dir / f"snap_{frame_idx:05d}.jpg"
                    cv2.imwrite(str(snap), result)
                    print(f"\n  Saved: {snap}")

            frame_idx += 1
    finally:
        if use_picamera2:
            picam2.stop()
        else:
            cap.release()
        if not no_show:
            cv2.destroyAllWindows()
        print()


def run_images(infer_pipeline, input_name, conf_thresh, images_dir, no_show, out_dir, single=None, input_size=None):
    if single:
        image_files = [single]
    else:
        img_dir = Path(images_dir)
        image_files = []
        for ext in ['.jpg', '.jpeg', '.png', '.bmp', '.JPG', '.PNG']:
            image_files.extend(img_dir.glob(f'*{ext}'))
        image_files = sorted(image_files)

    if not image_files:
        print(f"No images found in: {images_dir or single}")
        return

    print(f"\n{'='*55}")
    print(f"  Testing {len(image_files)} images | conf={conf_thresh}")
    print(f"{'='*55}")

    if not no_show:
        cv2.namedWindow("PET Bottle Detection", cv2.WINDOW_NORMAL)
        print("  Press ANY KEY for next image | Q to quit\n")

    total_det = 0
    total_ms  = 0

    for idx, img_path in enumerate(image_files):
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"  Cannot load: {img_path.name}")
            continue

        orig_h, orig_w = frame.shape[:2]
        input_data = preprocess(frame, input_size=input_size)

        t0 = time.time()
        raw_outputs = infer_pipeline.infer({input_name: input_data})
        ms = (time.time() - t0) * 1000

        detections = postprocess_coco(raw_outputs, orig_w, orig_h, conf_thresh)
        total_det += len(detections)
        total_ms  += ms

        result = draw_detections(frame.copy(), detections)
        overlay_info(result, detections, ms, f"[{idx+1}/{len(image_files)}] {img_path.name}")

        out_path = out_dir / f"result_{img_path.name}"
        cv2.imwrite(str(out_path), result)

        icon = "OK" if detections else "--"
        print(f"  [{icon}] [{idx+1:3d}/{len(image_files)}] {img_path.name:<35} "
              f"| {len(detections):2d} det | {ms:.1f}ms")

        if not no_show:
            cv2.imshow("PET Bottle Detection", result)
            key = cv2.waitKey(0) & 0xFF
            if key in (ord('q'), ord('Q')):
                break

    if not no_show:
        cv2.destroyAllWindows()

    avg_ms = total_ms / len(image_files) if image_files else 0
    print(f"\n{'='*55}")
    print(f"  RESULTS SUMMARY")
    print(f"{'='*55}")
    print(f"  Images tested     : {len(image_files)}")
    print(f"  Total detections  : {total_det}")
    print(f"  Avg infer time    : {avg_ms:.1f}ms  ({1000/avg_ms:.1f} FPS)")
    print(f"  Results saved to  : {out_dir}/")
    print(f"{'='*55}\n")


def main():
    parser = argparse.ArgumentParser(description="PET Bottle detection (COCO yolov8s HEF)")
    parser.add_argument("--image",   default=None,  help="Path to a single image file")
    parser.add_argument("--images",  default=None,  help="Path to image folder")
    parser.add_argument("--camera",  type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--output",  default=RESULTS_DIR, help="Directory to save results")
    parser.add_argument("--conf",    type=float, default=CONF_THRESHOLD,
                        help=f"Confidence threshold (default: {CONF_THRESHOLD})")
    parser.add_argument("--no-show", action="store_true", help="Headless mode")
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading HEF: {HEF_PATH}")
    hef = HEF(HEF_PATH)
    print("Input streams:")
    for i in hef.get_input_vstream_infos():
        print(f"  {i.name}  shape={i.shape}")
    print("Output streams:")
    for o in hef.get_output_vstream_infos():
        print(f"  {o.name}  shape={o.shape}")

    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)

    with VDevice() as target:
        network_groups       = target.configure(hef, configure_params)
        network_group        = network_groups[0]
        network_group_params = network_group.create_params()

        input_vstreams_params  = InputVStreamParams.make(network_group, format_type=FormatType.UINT8)
        output_vstreams_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)

        input_info = hef.get_input_vstream_infos()[0]
        input_name = input_info.name
        h, w       = input_info.shape[0], input_info.shape[1]
        input_size = (w, h)
        print(f"Model input size: {input_size}")

        with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
            with network_group.activate(network_group_params):
                if args.image:
                    run_images(infer_pipeline, input_name, args.conf,
                               None, args.no_show, out_dir,
                               single=Path(args.image), input_size=input_size)
                elif args.images:
                    run_images(infer_pipeline, input_name, args.conf,
                               args.images, args.no_show, out_dir,
                               input_size=input_size)
                else:
                    run_camera(infer_pipeline, input_name, args.conf,
                               args.camera, args.no_show, out_dir,
                               input_size=input_size)


if __name__ == "__main__":
    main()
