#!/usr/bin/env python3
"""
PET Bottle detection with COCO yolov8s model (bottle class 39)
Usage:
  python3 camera.py                          # live camera (default)
  python3 camera.py --camera 0               # choose camera index
  python3 camera.py --image sample.jpg       # run on single image
  python3 camera.py --images ./test_images/  # run on image folder
  python3 camera.py --conf 0.25              # set confidence threshold
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
HEF_PATH        = "yolov8s-coco.hef"
COCO_BOTTLE_ID  = 39
CONF_THRESHOLD  = 0.20
RESULTS_DIR     = "./results"
SMOOTH_FRAMES   = 10
CPU_CONF        = 0.25   # confidence for CPU YOLOv8n fallback
# ───────────────────────────────────────────────────────────

# ── CPU YOLOv8n + YOLOv8m ensemble for laydown detection ──
_cpu_models = None   # list of loaded models, or False if unavailable

def load_cpu_models():
    """Load YOLOv8n and YOLOv8m on CPU (once). Auto-downloads weights."""
    global _cpu_models
    if _cpu_models is not None:
        return _cpu_models
    try:
        from ultralytics import YOLO
        _cpu_models = []
        for name in ("yolov8n.pt", "yolov8m.pt"):
            m = YOLO(name)
            _cpu_models.append(m)
            print(f"  CPU {name} loaded")
    except Exception as e:
        print(f"  ultralytics not available ({e}) — horizontal laydown may be missed")
        _cpu_models = False
    return _cpu_models

def _rotate_boxes_back(boxes_list, rot_code, orig_w, orig_h):
    """Transform bounding boxes from rotated frame back to original frame coords."""
    result = []
    for (rx1, ry1, rx2, ry2, score) in boxes_list:
        if rot_code == cv2.ROTATE_90_CLOCKWISE:
            # rotated size: (orig_h, orig_w) → rot frame w=orig_h, h=orig_w
            ox1 = ry1;         oy1 = orig_w - rx2
            ox2 = ry2;         oy2 = orig_w - rx1
        elif rot_code == cv2.ROTATE_180:
            ox1 = orig_w - rx2; oy1 = orig_h - ry2
            ox2 = orig_w - rx1; oy2 = orig_h - ry1
        elif rot_code == cv2.ROTATE_90_COUNTERCLOCKWISE:
            ox1 = orig_h - ry2; oy1 = rx1
            ox2 = orig_h - ry1; oy2 = rx2
        else:
            ox1, oy1, ox2, oy2 = rx1, ry1, rx2, ry2
        ox1, ox2 = sorted([max(0, ox1), min(orig_w, ox2)])
        oy1, oy2 = sorted([max(0, oy1), min(orig_h, oy2)])
        if ox2 > ox1 and oy2 > oy1:
            result.append([ox1, oy1, ox2, oy2, score])
    return result


def cpu_detect(frame, conf_thresh):
    """
    Run YOLOv8n + YOLOv8m ensemble at 4 rotations (0°, 90°, 180°, 270°).
    Catches bottles in any orientation — horizontal laydown becomes
    upright from the model's perspective after 90° rotation.
    """
    models = load_cpu_models()
    if not models:
        return []

    orig_h, orig_w = frame.shape[:2]
    dets = []

    rotations = [
        (None,                          orig_w, orig_h),
        (cv2.ROTATE_90_CLOCKWISE,       orig_w, orig_h),
        (cv2.ROTATE_180,                orig_w, orig_h),
        (cv2.ROTATE_90_COUNTERCLOCKWISE,orig_w, orig_h),
    ]

    for rot_code, ow, oh in rotations:
        img = cv2.rotate(frame, rot_code) if rot_code is not None else frame
        raw = []
        for model in models:
            for box in model(img, classes=[COCO_BOTTLE_ID], conf=conf_thresh, verbose=False)[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                raw.append([x1, y1, x2, y2, float(box.conf[0])])
        if rot_code is not None:
            raw = _rotate_boxes_back(raw, rot_code, ow, oh)
        dets.extend(raw)

    # NMS across all rotations to remove duplicates
    if not dets:
        return []
    boxes  = [[d[0], d[1], d[2]-d[0], d[3]-d[1]] for d in dets]
    scores = [d[4] for d in dets]
    idx    = cv2.dnn.NMSBoxes(boxes, scores, conf_thresh, 0.4)
    return [dets[i] for i in (idx.flatten() if len(idx) else [])]

def merge_detections(hailo_dets, cpu_dets, iou_thresh=0.4):
    """Add CPU detections that don't overlap with Hailo detections."""
    merged = list(hailo_dets)
    for d in cpu_dets:
        x1, y1, x2, y2 = d[0], d[1], d[2], d[3]
        duplicate = False
        for m in merged:
            ix1 = max(x1, m[0]); iy1 = max(y1, m[1])
            ix2 = min(x2, m[2]); iy2 = min(y2, m[3])
            iw = max(0, ix2 - ix1); ih = max(0, iy2 - iy1)
            inter = iw * ih
            union = (x2-x1)*(y2-y1) + (m[2]-m[0])*(m[3]-m[1]) - inter
            if union > 0 and inter / union > iou_thresh:
                duplicate = True
                break
        if not duplicate:
            merged.append(d)
    return merged
# ───────────────────────────────────────────────────────────


def postprocess_coco(outputs, orig_w, orig_h, conf_thresh):
    """NMS-format postprocessor for yolov8s-coco.hef — keeps bottle class only."""
    nms_key = next((k for k in outputs if 'nms' in k.lower() or 'output' in k.lower()), None)
    if nms_key is None:
        return []
    batch = outputs[nms_key][0]  # shape: [num_classes, max_det, 5]
    if COCO_BOTTLE_ID >= len(batch):
        return []
    detections = []
    for det in batch[COCO_BOTTLE_ID]:
        score = float(det[4])
        if score < conf_thresh:
            continue
        # det: [y1, x1, y2, x2, score] normalised 0-1
        y1 = int(det[0] * orig_h); x1 = int(det[1] * orig_w)
        y2 = int(det[2] * orig_h); x2 = int(det[3] * orig_w)
        if x2 <= x1 or y2 <= y1:
            continue
        detections.append([x1, y1, x2, y2, score])
    return detections


def get_orientation(x1, y1, x2, y2, frame_h):
    """
    Orientation detection for 1.5L and 500ml PET bottles.
    Aspect ratio is the primary and authoritative signal.
      - ratio > 1.15  → definitely laydown (wider than tall)
      - ratio < 0.85  → definitely upright (taller than wide)
      - 0.85–1.15     → ambiguous, use vertical fill as tiebreaker
    """
    w, h = x2 - x1, y2 - y1
    if w == 0 or h == 0:
        return "upright"

    ratio = w / h  # >1 = wider than tall

    if ratio > 1.15:
        return "laydown"
    if ratio < 0.85:
        return "upright"

    # Ambiguous zone: box is nearly square (common for 500ml upright)
    # Use vertical fill — upright bottles extend tall in frame
    remaining = frame_h - y1
    vertical_fill = h / remaining if remaining > 0 else 1.0
    return "upright" if vertical_fill > 0.4 else "laydown"


def draw_orientation_arrow(frame, x1, y1, x2, y2, orientation, color):
    """Draw an arrow inside the box showing bottle orientation."""
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    w, h = x2 - x1, y2 - y1
    arm = min(w, h) // 3

    if orientation == "upright":
        # Vertical arrow pointing up
        pt_tail = (cx, cy + arm)
        pt_head = (cx, cy - arm)
    else:
        # Horizontal arrow pointing right
        pt_tail = (cx - arm, cy)
        pt_head = (cx + arm, cy)

    cv2.arrowedLine(frame, pt_tail, pt_head, color, 2, tipLength=0.4)


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
    """
    Temporal smoother: keeps a detection alive for SMOOTH_FRAMES frames
    after it was last seen, preventing flicker when the model misses a frame.
    Uses IoU matching to track boxes across frames.
    """
    def __init__(self, max_age=SMOOTH_FRAMES, iou_thresh=0.35):
        self.max_age   = max_age
        self.iou_thresh = iou_thresh
        self.tracked   = []  # list of [x1,y1,x2,y2,conf, age]

    def _iou(self, a, b):
        ix1 = max(a[0], b[0]); iy1 = max(a[1], b[1])
        ix2 = min(a[2], b[2]); iy2 = min(a[3], b[3])
        iw  = max(0, ix2 - ix1); ih = max(0, iy2 - iy1)
        inter = iw * ih
        ua = (a[2]-a[0])*(a[3]-a[1]); ub = (b[2]-b[0])*(b[3]-b[1])
        union = ua + ub - inter
        return inter / union if union > 0 else 0.0

    def update(self, detections):
        # Age all tracked boxes
        for t in self.tracked:
            t[5] += 1

        # Match new detections to existing tracks
        matched = set()
        for det in detections:
            best_iou, best_idx = 0, -1
            for i, t in enumerate(self.tracked):
                iou = self._iou(det, t)
                if iou > best_iou:
                    best_iou, best_idx = iou, i
            if best_iou >= self.iou_thresh:
                # Update existing track with new position + reset age
                self.tracked[best_idx][:5] = list(det)
                self.tracked[best_idx][5]  = 0
                matched.add(best_idx)
            else:
                # New detection
                self.tracked.append(list(det) + [0])

        # Remove stale tracks
        self.tracked = [t for t in self.tracked if t[5] <= self.max_age]

        # Return all live tracks (age <= max_age)
        return [[t[0], t[1], t[2], t[3], t[4]] for t in self.tracked]


def preprocess(frame, is_rgb=False, input_size=None):
    size = input_size if input_size else (640, 640)
    img = cv2.resize(frame, size)
    if not is_rgb:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return np.expand_dims(img.astype(np.uint8), axis=0)


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

    frame_idx  = 0
    ms         = 0.0
    tracker    = DetectionTracker()
    last_cpu   = []   # cached CPU detections
    CPU_EVERY  = 3    # run CPU inference every N frames

    try:
        while True:
            if use_picamera2:
                frame_rgb = picam2.capture_array()
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                input_data = preprocess(frame_rgb, is_rgb=True, input_size=input_size)
            else:
                ret, frame = cap.read()
                if not ret:
                    print("  Frame capture failed")
                    break
                input_data = preprocess(frame, input_size=input_size)

            orig_h, orig_w = frame.shape[:2]

            t0 = time.time()
            raw_outputs = infer_pipeline.infer({input_name: input_data})
            ms = (time.time() - t0) * 1000

            hailo_dets = postprocess_coco(raw_outputs, orig_w, orig_h, conf_thresh)
            if frame_idx % CPU_EVERY == 0:
                last_cpu = cpu_detect(frame, CPU_CONF)
            raw_dets   = merge_detections(hailo_dets, last_cpu)
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

        hailo_dets = postprocess_coco(raw_outputs, orig_w, orig_h, conf_thresh)
        cpu_dets   = cpu_detect(frame, CPU_CONF)
        detections = merge_detections(hailo_dets, cpu_dets)
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
    parser = argparse.ArgumentParser(description="PET Bottle detection (COCO yolov8s)")
    parser.add_argument("--image",   default=None,  help="Path to a single image file")
    parser.add_argument("--images",  default=None,  help="Path to image folder")
    parser.add_argument("--camera",  type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--output",  default=RESULTS_DIR, help="Directory to save results")
    parser.add_argument("--conf",    type=float, default=CONF_THRESHOLD,
                        help=f"Confidence threshold (default: {CONF_THRESHOLD})")
    parser.add_argument("--no-show", action="store_true", help="Headless mode — no display window")
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
        network_groups = target.configure(hef, configure_params)
        network_group  = network_groups[0]
        network_group_params = network_group.create_params()

        input_vstreams_params  = InputVStreamParams.make(network_group, format_type=FormatType.UINT8)
        output_vstreams_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)

        input_info = hef.get_input_vstream_infos()[0]
        input_name = input_info.name
        h, w = input_info.shape[0], input_info.shape[1]
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
