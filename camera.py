#!/usr/bin/env python3
"""
Test best.hef detection on images
Usage:
  python3 test_hef.py --images ./test_images/
  python3 test_hef.py --images ./test_images/ --no-show   (headless)
  python3 test_hef.py --images ./test_images/ --conf 0.5
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
HEF_PATH       = "best_hailo.hef"
CLASS_NAMES    = ["PET-Bottle"]
CONF_THRESHOLD = 0.3
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


def postprocess(outputs, orig_w, orig_h, conf_thresh):
    conf_raw = outputs.get("best/activation1")
    reg_raw  = outputs.get("best/concat14")
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
        detections.append([x1, y1, x2, y2, float(score), 0])

    if not detections:
        return []

    boxes  = [[d[0], d[1], d[2]-d[0], d[3]-d[1]] for d in detections]
    scores = [d[4] for d in detections]
    indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thresh, 0.45)
    if len(indices) == 0:
        return []
    return [detections[i] for i in indices.flatten()]


def draw_detections(frame, detections):
    for (x1, y1, x2, y2, conf, cls_id) in detections:
        label = CLASS_NAMES[int(cls_id)] if int(cls_id) < len(CLASS_NAMES) else f"cls{cls_id}"

        # Outer border (red)
        cv2.rectangle(frame, (x1-2, y1-2), (x2+2, y2+2), (0, 0, 255), 6)
        # Inner border (green)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Corner accents
        clen = 20
        ct   = 4
        c    = (0, 255, 0)
        cv2.line(frame, (x1, y1), (x1+clen, y1), c, ct)
        cv2.line(frame, (x1, y1), (x1, y1+clen), c, ct)
        cv2.line(frame, (x2, y1), (x2-clen, y1), c, ct)
        cv2.line(frame, (x2, y1), (x2, y1+clen), c, ct)
        cv2.line(frame, (x1, y2), (x1+clen, y2), c, ct)
        cv2.line(frame, (x1, y2), (x1, y2-clen), c, ct)
        cv2.line(frame, (x2, y2), (x2-clen, y2), c, ct)
        cv2.line(frame, (x2, y2), (x2, y2-clen), c, ct)

        # Label
        text = f"{label.upper()} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.rectangle(frame, (x1, y1-30), (x1+tw+10, y1), (0, 0, 255), -1)
        cv2.putText(frame, text, (x1+5, y1-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return frame


def main():
    parser = argparse.ArgumentParser(description="Test best.hef on images")
    parser.add_argument("--images",  default="./test_images",
                        help="Path to test images folder")
    parser.add_argument("--output",  default=RESULTS_DIR,
                        help="Path to save result images")
    parser.add_argument("--conf",    type=float, default=CONF_THRESHOLD,
                        help="Confidence threshold (default: 0.3)")
    parser.add_argument("--no-show", action="store_true",
                        help="Save results only, do not display")
    args = parser.parse_args()

    # Collect images
    img_dir = Path(args.images)
    image_files = []
    for ext in ['.jpg', '.jpeg', '.png', '.bmp', '.JPG', '.PNG']:
        image_files.extend(img_dir.glob(f'*{ext}'))
    image_files = sorted(image_files)

    if not image_files:
        print(f"❌ No images found in: {args.images}")
        return

    print(f"Found {len(image_files)} images")

    # Create output dir
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load HEF
    print(f"Loading HEF: {HEF_PATH}")
    hef = HEF(HEF_PATH)

    total_det = 0
    total_ms  = 0

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
        input_name = hef.get_input_vstream_infos()[0].name

        with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
            with network_group.activate(network_group_params):
                print(f"\n{'='*55}")
                print(f"  Testing {len(image_files)} images | conf={args.conf}")
                print(f"{'='*55}")

                if not args.no_show:
                    cv2.namedWindow("HEF Test", cv2.WINDOW_NORMAL)
                    print("  Press ANY KEY for next image | Q to quit\n")

                for idx, img_path in enumerate(image_files):
                    frame = cv2.imread(str(img_path))
                    if frame is None:
                        print(f"  ⚠️  Cannot load: {img_path.name}")
                        continue

                    orig_h, orig_w = frame.shape[:2]

                    # Preprocess
                    img = cv2.resize(frame, INPUT_SIZE)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img = img.astype(np.float32) / 255.0
                    input_data = np.expand_dims(img, axis=0)

                    # Infer
                    t0 = time.time()
                    raw_outputs = infer_pipeline.infer({input_name: input_data})
                    ms = (time.time() - t0) * 1000

                    # Postprocess
                    detections = postprocess(raw_outputs, orig_w, orig_h, args.conf)
                    total_det += len(detections)
                    total_ms  += ms

                    # Draw
                    result = draw_detections(frame.copy(), detections)

                    # Info overlay
                    status = "✅ DETECTED" if detections else "⬜ NONE"
                    cv2.putText(result, f"{status} ({len(detections)})", (10, 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                                (0, 255, 0) if detections else (0, 200, 255), 2)
                    cv2.putText(result, f"Infer: {ms:.1f}ms", (10, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                    cv2.putText(result, f"[{idx+1}/{len(image_files)}] {img_path.name}",
                                (10, orig_h - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

                    # Save
                    out_path = out_dir / f"result_{img_path.name}"
                    cv2.imwrite(str(out_path), result)

                    # Log
                    icon = "✅" if detections else "⬜"
                    print(f"  {icon} [{idx+1:3d}/{len(image_files)}] {img_path.name:<35} "
                          f"| {len(detections):2d} det | {ms:.1f}ms")

                    # Display
                    if not args.no_show:
                        cv2.imshow("HEF Test", result)
                        key = cv2.waitKey(0) & 0xFF
                        if key == ord('q') or key == ord('Q'):
                            break

    if not args.no_show:
        cv2.destroyAllWindows()

    avg_ms = total_ms / len(image_files) if image_files else 0
    detected = sum(1 for _ in range(len(image_files)))

    print(f"\n{'='*55}")
    print(f"  RESULTS SUMMARY")
    print(f"{'='*55}")
    print(f"  Images tested     : {len(image_files)}")
    print(f"  Total detections  : {total_det}")
    print(f"  Avg infer time    : {avg_ms:.1f}ms  ({1000/avg_ms:.1f} FPS)")
    print(f"  Results saved to  : {out_dir}/")
    print(f"{'='*55}\n")


if __name__ == "__main__":
    main()