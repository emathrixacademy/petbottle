#!/usr/bin/env python3
"""
PET Bottle detection with petbottle.hef — images or live camera
Usage:
  python3 camera.py                          # live camera (default)
  python3 camera.py --camera 0               # choose camera index
  python3 camera.py --images ./test_images/  # run on image folder
  python3 camera.py --conf 0.4               # set confidence threshold
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
HEF_MODELS = {
    1: "petbottle-yolov8m.hef",
    2: "petbottle-yolov8n.hef",
    3: "petbottle-yolov8s.hef",
}
CLASS_NAMES    = ["PET-Bottle"]
CONF_THRESHOLD = 0.25
INPUT_SIZE     = (416, 416)
REG_MAX        = 16
RESULTS_DIR    = "./results"
# ───────────────────────────────────────────────────────────


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
    N = reg.shape[0]
    reg = reg.reshape(N, 4, reg_max)
    reg = reg - reg.max(axis=-1, keepdims=True)
    exp = np.exp(reg)
    reg = exp / exp.sum(axis=-1, keepdims=True)
    weights = np.arange(reg_max, dtype=np.float32)
    return (reg * weights).sum(axis=-1)


def find_outputs(outputs):
    """Auto-detect confidence and regression tensors by shape."""
    conf_raw = reg_raw = None
    total_anchors = sum(
        (INPUT_SIZE[0] // s) * (INPUT_SIZE[1] // s) for s in [8, 16, 32]
    )  # 3549 for 416x416
    for _, tensor in outputs.items():
        flat = tensor.reshape(-1)
        if flat.shape[0] == total_anchors:
            conf_raw = flat
        elif tensor.reshape(-1, 64).shape[0] == total_anchors:
            reg_raw = tensor
    return conf_raw, reg_raw


def postprocess(outputs, orig_w, orig_h, conf_thresh, output_names=None):
    conf_raw = reg_raw = None
    if output_names:
        for name in output_names:
            if "activation" in name:
                conf_raw = outputs.get(name)
            elif "concat" in name:
                reg_raw  = outputs.get(name)
    if conf_raw is None or reg_raw is None:
        conf_raw, reg_raw = find_outputs(outputs)
    if conf_raw is None or reg_raw is None:
        return []

    conf = conf_raw.reshape(-1)
    reg  = reg_raw.reshape(-1, 64)
    ltrb = dfl_decode(reg, REG_MAX)

    detections = []
    for i, (score, (l, t, r, b)) in enumerate(zip(conf, ltrb)):
        if score < conf_thresh:
            continue
        cx, cy, stride = ANCHORS[i]
        x1 = int((cx - l) * stride * orig_w / INPUT_SIZE[0])
        y1 = int((cy - t) * stride * orig_h / INPUT_SIZE[1])
        x2 = int((cx + r) * stride * orig_w / INPUT_SIZE[0])
        y2 = int((cy + b) * stride * orig_h / INPUT_SIZE[1])
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(orig_w, x2), min(orig_h, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        w, h = x2 - x1, y2 - y1
        aspect = h / w if w > 0 else 0
        if aspect < 0.8 or aspect > 6.0:  # filter non-bottle shapes
            continue
        detections.append([x1, y1, x2, y2, float(score), 0])

    if not detections:
        return []

    boxes  = [[d[0], d[1], d[2]-d[0], d[3]-d[1]] for d in detections]
    scores = [d[4] for d in detections]
    indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thresh, 0.3)
    if len(indices) == 0:
        return []
    return [detections[i] for i in indices.flatten()]


def draw_detections(frame, detections):
    for (x1, y1, x2, y2, conf, cls_id) in detections:
        label = CLASS_NAMES[int(cls_id)] if int(cls_id) < len(CLASS_NAMES) else f"cls{cls_id}"
        text  = f"{label.upper()}  {conf:.2f}"

        # Bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

        # Label badge
        (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        badge_y = max(y1 - th - baseline - 8, 0)
        cv2.rectangle(frame, (x1, badge_y), (x1 + tw + 10, y1), (0, 255, 0), -1)
        cv2.putText(frame, text, (x1 + 5, y1 - baseline - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    return frame


def preprocess(frame, is_rgb=False):
    img = cv2.resize(frame, INPUT_SIZE)
    if not is_rgb:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    return np.expand_dims(img, axis=0)


def overlay_info(frame, detections, ms, extra=""):
    h = frame.shape[0]
    status = f"DETECTED ({len(detections)})" if detections else "NONE"
    color  = (0, 255, 0) if detections else (0, 200, 255)
    cv2.putText(frame, status, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    cv2.putText(frame, f"Infer: {ms:.1f}ms  {1000/ms:.1f}FPS" if ms > 0 else "",
                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)
    if extra:
        cv2.putText(frame, extra, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
    return frame


def run_camera(infer_pipeline, input_name, conf_thresh, camera_index, no_show, out_dir, output_names=None):
    # Use picamera2 for Pi Camera ribbon (CSI)
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
            print(f"❌ Cannot open camera (index={camera_index})")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not no_show:
        cv2.namedWindow("PET Bottle Detection", cv2.WINDOW_NORMAL)

    print(f"\n{'='*55}")
    print(f"  Live Camera | conf={conf_thresh} | Q to quit")
    print(f"{'='*55}\n")

    frame_idx = 0
    ms = 0.0

    try:
        while True:
            if use_picamera2:
                frame_rgb = picam2.capture_array()
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                input_data = preprocess(frame_rgb, is_rgb=True)
            else:
                ret, frame = cap.read()
                if not ret:
                    print("  ⚠️  Frame capture failed")
                    break
                input_data = preprocess(frame)

            orig_h, orig_w = frame.shape[:2]

            t0 = time.time()
            raw_outputs = infer_pipeline.infer({input_name: input_data})
            ms = (time.time() - t0) * 1000

            detections = postprocess(raw_outputs, orig_w, orig_h, conf_thresh, output_names)

            result = draw_detections(frame.copy(), detections)
            overlay_info(result, detections, ms, f"Frame {frame_idx}")

            icon = "✅" if detections else "⬜"
            print(f"\r  {icon} frame={frame_idx:5d} | {len(detections):2d} det | {ms:.1f}ms", end="", flush=True)

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


def run_images(infer_pipeline, input_name, conf_thresh, images_dir, no_show, out_dir, single=None, output_names=None):
    if single:
        image_files = [single]
    else:
        img_dir = Path(images_dir)
        image_files = []
        for ext in ['.jpg', '.jpeg', '.png', '.bmp', '.JPG', '.PNG']:
            image_files.extend(img_dir.glob(f'*{ext}'))
        image_files = sorted(image_files)

    if not image_files:
        print(f"❌ No images found in: {images_dir or single}")
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
            print(f"  ⚠️  Cannot load: {img_path.name}")
            continue

        orig_h, orig_w = frame.shape[:2]
        input_data = preprocess(frame)

        t0 = time.time()
        raw_outputs = infer_pipeline.infer({input_name: input_data})
        ms = (time.time() - t0) * 1000

        detections = postprocess(raw_outputs, orig_w, orig_h, conf_thresh)
        total_det += len(detections)
        total_ms  += ms

        result = draw_detections(frame.copy(), detections)
        overlay_info(result, detections, ms, f"[{idx+1}/{len(image_files)}] {img_path.name}")

        out_path = out_dir / f"result_{img_path.name}"
        cv2.imwrite(str(out_path), result)

        icon = "✅" if detections else "⬜"
        print(f"  {icon} [{idx+1:3d}/{len(image_files)}] {img_path.name:<35} "
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
    parser = argparse.ArgumentParser(description="PET Bottle detection — camera or images")
    parser.add_argument("--model",   type=int, default=1, choices=[1, 2, 3],
                        help="Model: 1=yolov8m (default), 2=yolov8n, 3=yolov8s")
    parser.add_argument("--image",   default=None,
                        help="Path to a single image file")
    parser.add_argument("--images",  default=None,
                        help="Path to image folder (omit for live camera)")
    parser.add_argument("--camera",  type=int, default=0,
                        help="Camera index for V4L2 fallback (default: 0)")
    parser.add_argument("--output",  default=RESULTS_DIR,
                        help="Directory to save results / snapshots")
    parser.add_argument("--conf",    type=float, default=CONF_THRESHOLD,
                        help="Confidence threshold (default: 0.3)")
    parser.add_argument("--no-show", action="store_true",
                        help="Headless mode — no display window")
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    hef_path = HEF_MODELS[args.model]
    print(f"Loading HEF: {hef_path}  (model {args.model})")
    hef = HEF(hef_path)

    # Print detected stream names for debugging
    print("Input streams:")
    for i in hef.get_input_vstream_infos():
        print(f"  {i.name}  shape={i.shape}")
    print("Output streams:")
    for o in hef.get_output_vstream_infos():
        print(f"  {o.name}  shape={o.shape}")

    with VDevice() as target:
        configure_params = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe
        )
        network_groups = target.configure(hef, configure_params)
        network_group  = network_groups[0]
        network_group_params = network_group.create_params()

        input_vstreams_params  = InputVStreamParams.make(
            network_group, format_type=FormatType.FLOAT32
        )
        output_vstreams_params = OutputVStreamParams.make(
            network_group, format_type=FormatType.FLOAT32
        )
        input_name = hef.get_input_vstream_infos()[0].name
        output_names = [o.name for o in hef.get_output_vstream_infos()]

        with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
            with network_group.activate(network_group_params):
                if args.image:
                    run_images(infer_pipeline, input_name, args.conf,
                               None, args.no_show, out_dir,
                               single=Path(args.image), output_names=output_names)
                elif args.images:
                    run_images(infer_pipeline, input_name, args.conf,
                               args.images, args.no_show, out_dir,
                               output_names=output_names)
                else:
                    run_camera(infer_pipeline, input_name, args.conf,
                               args.camera, args.no_show, out_dir,
                               output_names=output_names)


if __name__ == "__main__":
    main()