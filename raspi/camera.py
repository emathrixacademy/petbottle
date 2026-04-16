#!/usr/bin/env python3
"""
PET Bottle detection with YOLOv5 / YOLOv7 / YOLOv8 model stitching on Hailo-8
Usage:
  python3 camera.py                          # live camera (default, YOLOv8)
  python3 camera.py --model yolov5           # use YOLOv5
  python3 camera.py --model yolov7           # use YOLOv7
  python3 camera.py --model yolov8           # use YOLOv8 (default)
  python3 camera.py --model all              # ensemble: run all 3, merge results
  python3 camera.py --camera 0               # choose camera index
  python3 camera.py --image sample.jpg       # run on single image
  python3 camera.py --images ./test_images/  # run on image folder
  python3 camera.py --conf 0.20              # set confidence threshold
  python3 camera.py --no-show                # headless / save only

Toggle models live with keyboard:
  5 = YOLOv5   |   7 = YOLOv7   |   8 = YOLOv8   |   A = All (ensemble)
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

# ── Model Definitions ────────────────────────────────────────
MODEL_CONFIGS = {
    "yolov5": {
        "name": "PetBot-Custom",
        "hef_path": "petbottle-yolov8s.hef",   # Best custom PET bottle model (0.64 max)
        "type": "petbottle",
        "color": (0, 255, 0),       # green
        "anchors": None,
        "strides": None,
    },
    "yolov7": {
        "name": "PetBot-Alt",
        "hef_path": "petbottle.hef",           # Alternate custom model (weaker)
        "type": "petbottle",
        "color": (255, 0, 255),     # magenta
        "anchors": None,
        "strides": None,
    },
    "yolov8": {
        "name": "COCO-Bottle",
        "hef_path": "yolov8s-coco.hef",        # COCO generic bottle (class 39)
        "type": "nms",
        "color": (255, 200, 0),     # cyan-ish
        "anchors": None,
        "strides": None,
    },
}

# ── PET Bottle Shape Filter ────────────────────────────────
# PET bottles have a specific shape: tall and narrow (aspect ratio > 1.5)
# or lying down (aspect ratio < 0.7). Reject square-ish objects.
# Also reject very large detections (likely background false positives)
PET_MIN_ASPECT = 0.3    # min width/height ratio (very flat = lying bottle)
PET_MAX_ASPECT = 3.5    # max width/height ratio
PET_MAX_FILL   = 0.35   # max % of frame (reject huge false positives)
PET_MIN_FILL   = 0.001  # min % of frame (reject tiny noise)

COCO_BOTTLE_ID = 39
COCO_PERSON_ID = 0
CONF_THRESHOLD = 0.20
RESULTS_DIR    = "./results"
SMOOTH_FRAMES  = 10

# ── Model Specialisation by Range ────────────────────────────
# All 3 models detect bottles in ANY orientation (standing, lying,
# tilted) on the ground.  The robot picks them up, so every bottle
# matters.  Each model covers a different detection range:
#   YOLOv5  → FAR    (small bottles, <10% of frame)
#   YOLOv7  → MID    (medium bottles, 10-35% of frame)
#   YOLOv8  → CLOSE  (large/nearby bottles, >35% of frame)
# In single-model mode the range filter is applied.
# In ensemble mode all ranges are combined for full coverage.
MODEL_RANGE = {
    "yolov5": None,   # no range filter — detect at all distances
    "yolov7": None,
    "yolov8": None,
}
# fill-ratio thresholds (box area / frame area)
RANGE_LIMITS = {
    "far":   (0.00, 0.10),
    "mid":   (0.05, 0.40),   # slight overlap for smooth handoff
    "close": (0.25, 1.00),   # slight overlap for smooth handoff
}

# IoU threshold to consider a bottle "held" by a person
HELD_IOU_THRESH = 0.30
# ──────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════
# Postprocessors
# ══════════════════════════════════════════════════════════════

def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))


def postprocess_nms(outputs, orig_w, orig_h, conf_thresh):
    """Postprocessor for YOLOv8 with built-in NMS.
    Returns (bottle_detections, person_detections).
    """
    nms_key = next((k for k in outputs if 'nms' in k.lower() or 'output' in k.lower()), None)
    if nms_key is None:
        return [], []
    batch = outputs[nms_key][0]  # shape: [num_classes, max_det, 5]

    frame_area = orig_w * orig_h

    def _extract_class(class_id):
        if class_id >= len(batch):
            return []
        dets = []
        for det in batch[class_id]:
            score = float(det[4])
            if score <= 0:
                continue
            y1 = int(det[0] * orig_h); x1 = int(det[1] * orig_w)
            y2 = int(det[2] * orig_h); x2 = int(det[3] * orig_w)
            if x2 <= x1 or y2 <= y1:
                continue
            box_area = (x2 - x1) * (y2 - y1)
            fill = box_area / frame_area
            if fill > 0.40:
                effective_thresh = conf_thresh * 0.25
            elif fill > 0.20:
                effective_thresh = conf_thresh * 0.50
            else:
                effective_thresh = conf_thresh
            if score < effective_thresh:
                continue
            dets.append([x1, y1, x2, y2, score])
        return dets

    bottles = _extract_class(COCO_BOTTLE_ID)
    persons = _extract_class(COCO_PERSON_ID)
    return bottles, persons


def _decode_raw_class(data, class_id, anchors, strides, sorted_layers,
                      orig_w, orig_h, conf_thresh, input_w, input_h):
    """Decode raw YOLOv5/v7 outputs for a single COCO class.
    Returns list of [x1, y1, x2, y2, score].
    """
    detections = []
    for layer_idx, (name, tensor) in enumerate(sorted_layers):
        if layer_idx >= len(anchors):
            break
        dat = tensor[0]
        if dat.ndim < 3:
            continue

        grid_h, grid_w = dat.shape[0], dat.shape[1]
        num_anchors = len(anchors[layer_idx])
        stride = strides[layer_idx]
        channels_per_anchor = dat.shape[2] // num_anchors
        num_classes = channels_per_anchor - 5
        if num_classes <= class_id:
            continue

        dat = dat.reshape(grid_h, grid_w, num_anchors, channels_per_anchor)

        grid_x, grid_y = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
        grid_x = grid_x[:, :, np.newaxis].astype(np.float32)
        grid_y = grid_y[:, :, np.newaxis].astype(np.float32)

        tx = dat[..., 0].astype(np.float32)
        ty = dat[..., 1].astype(np.float32)
        tw = dat[..., 2].astype(np.float32)
        th = dat[..., 3].astype(np.float32)
        obj = dat[..., 4].astype(np.float32)
        cls = dat[..., 5 + class_id].astype(np.float32)

        needs_sigmoid = (np.max(obj) > 1.0) or (np.min(obj) < 0.0)
        if needs_sigmoid:
            tx = _sigmoid(tx); ty = _sigmoid(ty)
            tw = _sigmoid(tw); th = _sigmoid(th)
            obj = _sigmoid(obj); cls = _sigmoid(cls)

        cx = (tx * 2.0 - 0.5 + grid_x) * stride
        cy = (ty * 2.0 - 0.5 + grid_y) * stride
        anchor_arr = np.array(anchors[layer_idx], dtype=np.float32)
        bw = (tw * 2.0) ** 2 * anchor_arr[:, 0]
        bh = (th * 2.0) ** 2 * anchor_arr[:, 1]

        conf = obj * cls
        mask = conf > (conf_thresh * 0.25)
        if not np.any(mask):
            continue

        cx_f = cx[mask]; cy_f = cy[mask]
        bw_f = bw[mask]; bh_f = bh[mask]
        conf_f = conf[mask]

        x1 = np.clip((cx_f - bw_f / 2) / input_w * orig_w, 0, orig_w).astype(int)
        y1 = np.clip((cy_f - bh_f / 2) / input_h * orig_h, 0, orig_h).astype(int)
        x2 = np.clip((cx_f + bw_f / 2) / input_w * orig_w, 0, orig_w).astype(int)
        y2 = np.clip((cy_f + bh_f / 2) / input_h * orig_h, 0, orig_h).astype(int)

        for i in range(len(conf_f)):
            if x2[i] > x1[i] and y2[i] > y1[i]:
                detections.append([int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i]), float(conf_f[i])])

    if not detections:
        return []
    boxes = [[d[0], d[1], d[2] - d[0], d[3] - d[1]] for d in detections]
    scores = [d[4] for d in detections]
    idx = cv2.dnn.NMSBoxes(boxes, scores, conf_thresh * 0.25, 0.4)
    return [detections[i] for i in (idx.flatten() if len(idx) else [])]


def postprocess_raw(outputs, orig_w, orig_h, conf_thresh, anchors, strides, input_w, input_h):
    """Postprocessor for raw YOLOv5 / YOLOv7 outputs (no built-in NMS).
    Returns (bottle_detections, person_detections).
    """
    sorted_layers = sorted(
        outputs.items(), key=lambda kv: kv[1][0].size, reverse=True,
    )
    bottles = _decode_raw_class(
        outputs, COCO_BOTTLE_ID, anchors, strides, sorted_layers,
        orig_w, orig_h, conf_thresh, input_w, input_h,
    )
    persons = _decode_raw_class(
        outputs, COCO_PERSON_ID, anchors, strides, sorted_layers,
        orig_w, orig_h, conf_thresh, input_w, input_h,
    )
    return bottles, persons


def postprocess_petbottle(outputs, orig_w, orig_h, conf_thresh, input_w, input_h):
    """Postprocessor for custom PET bottle YOLOv8 models (1 class, DFL output).
    Outputs: activation1 (1, N, 1) = class scores, concat14 (1, N, 64) = box DFL.
    Returns (bottle_detections, []) — no person detection in custom models.
    """
    # Find the score and box tensors
    score_key = next((k for k in outputs if 'activation' in k), None)
    box_key = next((k for k in outputs if 'concat' in k), None)
    if score_key is None or box_key is None:
        return [], []

    scores_raw = outputs[score_key][0]  # (N, 1) or (1, N, 1)
    boxes_raw = outputs[box_key][0]     # (N, 64) or (1, N, 64)

    # Flatten to 1D scores and 2D boxes
    scores_raw = scores_raw.reshape(-1)
    boxes_raw = boxes_raw.reshape(-1, 64) if boxes_raw.ndim > 1 else boxes_raw

    scores = scores_raw
    mask = scores > conf_thresh
    if not np.any(mask):
        return [], []

    scores_f = scores[mask]
    boxes_f = boxes_raw[mask]  # (M, 64)

    # DFL decode: 64 = 4 sides × 16 bins
    # YOLOv8 uses Distribution Focal Loss for box regression
    # Each side has 16 values that form a distribution
    dfl = boxes_f.reshape(-1, 4, 16)
    # Softmax per side
    dfl_exp = np.exp(dfl - np.max(dfl, axis=2, keepdims=True))
    dfl_softmax = dfl_exp / np.sum(dfl_exp, axis=2, keepdims=True)
    # Expected value (weighted sum with indices 0..15)
    proj = np.arange(16, dtype=np.float32)
    dfl_decoded = np.sum(dfl_softmax * proj, axis=2)  # (M, 4) = [left, top, right, bottom]

    # Determine grid positions for each detection
    # Input is 416x416, strides are [8, 16, 32]
    strides = [8, 16, 32]
    grid_sizes = [(input_h // s, input_w // s) for s in strides]
    # Total anchors: sum of grid_h * grid_w for each stride
    anchors_per_stride = [gh * gw for gh, gw in grid_sizes]
    total_anchors = sum(anchors_per_stride)

    # Build grid of (cx, cy, stride) for all anchor points
    all_cx = np.zeros(total_anchors, dtype=np.float32)
    all_cy = np.zeros(total_anchors, dtype=np.float32)
    all_strides = np.zeros(total_anchors, dtype=np.float32)
    offset = 0
    for (gh, gw), stride in zip(grid_sizes, strides):
        n = gh * gw
        gy, gx = np.meshgrid(np.arange(gh), np.arange(gw), indexing='ij')
        all_cx[offset:offset+n] = (gx.flatten() + 0.5) * stride
        all_cy[offset:offset+n] = (gy.flatten() + 0.5) * stride
        all_strides[offset:offset+n] = stride
        offset += n

    # Apply mask to grid
    indices = np.where(mask)[0]
    cx = all_cx[indices]
    cy = all_cy[indices]
    st = all_strides[indices]

    # Decode boxes: x1 = cx - left*stride, y1 = cy - top*stride, etc.
    x1 = cx - dfl_decoded[:, 0] * st
    y1 = cy - dfl_decoded[:, 1] * st
    x2 = cx + dfl_decoded[:, 2] * st
    y2 = cy + dfl_decoded[:, 3] * st

    # Scale to original image size
    x1 = np.clip(x1 / input_w * orig_w, 0, orig_w).astype(int)
    y1 = np.clip(y1 / input_h * orig_h, 0, orig_h).astype(int)
    x2 = np.clip(x2 / input_w * orig_w, 0, orig_w).astype(int)
    y2 = np.clip(y2 / input_h * orig_h, 0, orig_h).astype(int)

    detections = []
    for i in range(len(scores_f)):
        if x2[i] > x1[i] and y2[i] > y1[i]:
            detections.append([int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i]), float(scores_f[i])])

    if not detections:
        return [], []

    # NMS
    boxes = [[d[0], d[1], d[2]-d[0], d[3]-d[1]] for d in detections]
    det_scores = [d[4] for d in detections]
    idx = cv2.dnn.NMSBoxes(boxes, det_scores, conf_thresh * 0.5, 0.45)
    bottles = [detections[i] for i in (idx.flatten() if len(idx) else [])]
    return bottles, []


# ══════════════════════════════════════════════════════════════
# Orientation, view filtering, and held-by-human filtering
# ══════════════════════════════════════════════════════════════

def get_orientation(x1, y1, x2, y2, frame_h):
    """Classify a bottle detection as upright / laydown / tilted."""
    w, h = x2 - x1, y2 - y1
    if w == 0 or h == 0:
        return "upright"
    ratio = w / h
    if ratio > 1.15:
        return "laydown"
    if ratio < 0.85:
        return "upright"
    # Ambiguous zone → tilted
    box_fill = (w * h) / (frame_h * frame_h)
    if box_fill > 0.25:
        return "tilted"
    remaining = frame_h - y1
    vertical_fill = h / remaining if remaining > 0 else 1.0
    if vertical_fill > 0.4:
        return "upright"
    return "tilted"


def _iou(a, b):
    """IoU between two [x1,y1,x2,y2,...] boxes."""
    ix1 = max(a[0], b[0]); iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2]); iy2 = min(a[3], b[3])
    iw = max(0, ix2 - ix1); ih = max(0, iy2 - iy1)
    inter = iw * ih
    aa = (a[2] - a[0]) * (a[3] - a[1])
    ab = (b[2] - b[0]) * (b[3] - b[1])
    union = aa + ab - inter
    return inter / union if union > 0 else 0.0


def _bottle_inside_person(bottle, person):
    """Check if bottle centre is inside the person box (likely being held)."""
    bcx = (bottle[0] + bottle[2]) / 2
    bcy = (bottle[1] + bottle[3]) / 2
    return (person[0] <= bcx <= person[2]) and (person[1] <= bcy <= person[3])


def filter_held_bottles(bottles, persons):
    """Remove bottles that overlap significantly with a person (being held).
    A bottle is considered held if:
      - Its IoU with any person > HELD_IOU_THRESH, OR
      - Its centre is inside a person bounding box
    """
    if not persons:
        return bottles
    free = []
    for b in bottles:
        held = False
        for p in persons:
            if _iou(b, p) > HELD_IOU_THRESH or _bottle_inside_person(b, p):
                held = True
                break
        if not held:
            free.append(b)
    return free


def filter_pet_shape(bottles, frame_area):
    """Reject detections that don't look like PET bottles based on shape.
    PET bottles are typically tall+narrow (standing) or wide+flat (lying).
    Rejects: square objects (filament spools, cups), very large/tiny detections.
    """
    filtered = []
    for b in bottles:
        w = b[2] - b[0]
        h = b[3] - b[1]
        if w <= 0 or h <= 0:
            continue
        aspect = w / h
        fill = (w * h) / frame_area if frame_area > 0 else 0

        # Reject if too large (background false positive)
        if fill > PET_MAX_FILL:
            continue
        # Reject if too tiny (noise)
        if fill < PET_MIN_FILL:
            continue
        # Reject if aspect ratio is too square (0.7-1.4 = square-ish = not a bottle)
        # PET bottles: standing ~0.3-0.6, lying ~1.5-3.5
        if 0.7 < aspect < 1.4 and fill < 0.10:
            continue  # square small object, likely not a bottle
        # Accept if within general bottle shape range
        if PET_MIN_ASPECT <= aspect <= PET_MAX_ASPECT:
            filtered.append(b)
    return filtered


def filter_by_range(bottles, range_key, frame_area):
    """Keep only bottles in this model's assigned range.
    range_key = "far" | "mid" | "close" | None (keep all).
    """
    if range_key is None:
        return bottles
    lo, hi = RANGE_LIMITS[range_key]
    result = []
    for b in bottles:
        box_area = (b[2] - b[0]) * (b[3] - b[1])
        fill = box_area / frame_area if frame_area > 0 else 0
        if lo <= fill <= hi:
            result.append(b)
    return result


# ══════════════════════════════════════════════════════════════
# Drawing helpers
# ══════════════════════════════════════════════════════════════

def draw_orientation_arrow(frame, x1, y1, x2, y2, orientation, color):
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    arm = min(x2 - x1, y2 - y1) // 3
    if orientation == "upright":
        cv2.arrowedLine(frame, (cx, cy + arm), (cx, cy - arm), color, 2, tipLength=0.4)
    elif orientation == "laydown":
        cv2.arrowedLine(frame, (cx - arm, cy), (cx + arm, cy), color, 2, tipLength=0.4)
    else:  # tilted
        cv2.arrowedLine(frame, (cx - arm, cy + arm), (cx + arm, cy - arm), color, 2, tipLength=0.4)


def draw_detections(frame, detections, persons=None):
    """Draw bottle detections and (optionally) person boxes."""
    frame_h = frame.shape[0]

    # Draw person boxes (semi-transparent, so user sees why bottles are filtered)
    if persons:
        for (px1, py1, px2, py2, _) in persons:
            cv2.rectangle(frame, (px1, py1), (px2, py2), (100, 100, 100), 2)
            cv2.putText(frame, "PERSON", (px1 + 4, py1 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)

    # Draw bottle detections
    for (x1, y1, x2, y2, conf) in detections:
        orientation = get_orientation(x1, y1, x2, y2, frame_h)
        tag_map = {"upright": "UPRIGHT", "laydown": "LAYDOWN", "tilted": "TILTED"}
        color_map = {
            "upright": (0, 255, 0),      # green
            "laydown": (0, 165, 255),     # orange
            "tilted":  (255, 200, 0),     # cyan
        }
        tag   = tag_map.get(orientation, "UPRIGHT")
        color = color_map.get(orientation, (0, 255, 0))
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
        self.tracked    = []

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


# ══════════════════════════════════════════════════════════════
# Model Manager — loads, switches, and runs YOLO models
# ══════════════════════════════════════════════════════════════

class ModelManager:
    """Manages YOLOv5/v7/v8 on a Hailo VDevice with live toggling."""

    MODEL_ORDER = ["yolov5", "yolov7", "yolov8"]

    def __init__(self, target, model_keys=None):
        """
        target     : an open VDevice
        model_keys : list of keys into MODEL_CONFIGS to load
                     (default: all three)
        """
        self.target = target
        self.models = {}          # key -> runtime info dict
        self.active_key = None
        self._pipeline = None
        self._ng_ctx = None

        keys = model_keys or self.MODEL_ORDER
        for key in keys:
            cfg = MODEL_CONFIGS[key]
            hef_path = Path(cfg["hef_path"])
            if not hef_path.exists():
                print(f"  WARNING: {hef_path} not found — skipping {cfg['name']}")
                continue
            hef = HEF(str(hef_path))
            input_info  = hef.get_input_vstream_infos()[0]
            output_info = hef.get_output_vstream_infos()
            h, w = input_info.shape[0], input_info.shape[1]

            self.models[key] = {
                "cfg": cfg,
                "hef": hef,
                "input_name": input_info.name,
                "input_size": (w, h),
                "output_names": [o.name for o in output_info],
            }
            print(f"  Loaded {cfg['name']:8s}  input={w}x{h}  outputs={len(output_info)}")

        if not self.models:
            raise RuntimeError("No models could be loaded!")

    # ── activation helpers ────────────────────────────────────
    def _deactivate_current(self):
        """Deactivate current model, free all Hailo SRAM."""
        import gc
        if self._pipeline is not None:
            try:
                self._pipeline.__exit__(None, None, None)
            except Exception:
                pass
        if self._ng_ctx is not None:
            try:
                self._ng_ctx.__exit__(None, None, None)
            except Exception:
                pass
        # Must delete ALL references for Hailo to free SRAM
        self._pipeline = None
        self._ng_ctx = None
        if hasattr(self, '_ng'):
            del self._ng
            self._ng = None
        if hasattr(self, '_ngs'):
            del self._ngs
            self._ngs = None
        self.active_key = None
        gc.collect()

    def activate(self, key):
        """Configure and activate a model by key. Only one model at a time."""
        if key == self.active_key and self._pipeline is not None:
            return  # already active
        if key not in self.models:
            print(f"  Model '{key}' not available")
            return

        self._deactivate_current()

        mi = self.models[key]
        hef = mi["hef"]
        params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        self._ngs = self.target.configure(hef, params)
        self._ng = self._ngs[0]
        ng_params = self._ng.create_params()
        inp_params = InputVStreamParams.make(self._ng, format_type=FormatType.UINT8)
        out_params = OutputVStreamParams.make(self._ng, format_type=FormatType.FLOAT32)

        self._ng_ctx = self._ng.activate(ng_params)
        self._ng_ctx.__enter__()

        self._pipeline = InferVStreams(self._ng, inp_params, out_params)
        self._pipeline.__enter__()

        self.active_key = key
        print(f"\n  >>> Switched to {mi['cfg']['name']}")

    def cleanup(self):
        self._deactivate_current()
        self.models.clear()

    # ── inference ─────────────────────────────────────────────
    def infer(self, frame, conf_thresh, apply_range_filter=True):
        """Run inference on *frame* with the currently active model.
        Returns (bottle_detections, person_detections).
        Detects ALL bottle orientations (standing, lying, tilted).
        Bottles held by a person are filtered out.
        Range filter keeps only bottles in this model's assigned range.
        """
        mi  = self.models[self.active_key]
        cfg = mi["cfg"]
        input_name = mi["input_name"]
        input_size = mi["input_size"]
        orig_h, orig_w = frame.shape[:2]

        input_data = preprocess(frame, input_size=input_size)
        outputs = self._pipeline.infer({input_name: input_data})

        if cfg["type"] == "nms":
            bottles, persons = postprocess_nms(outputs, orig_w, orig_h, conf_thresh)
        elif cfg["type"] == "petbottle":
            bottles, persons = postprocess_petbottle(
                outputs, orig_w, orig_h, conf_thresh,
                input_size[0], input_size[1],
            )
        else:
            bottles, persons = postprocess_raw(
                outputs, orig_w, orig_h, conf_thresh,
                cfg["anchors"], cfg["strides"],
                input_size[0], input_size[1],
            )

        # Filter out bottles held by a person
        bottles = filter_held_bottles(bottles, persons)

        # Filter by PET bottle shape (reject square objects, huge/tiny detections)
        bottles = filter_pet_shape(bottles, orig_w * orig_h)

        # Filter by range specialisation (far / mid / close)
        if apply_range_filter:
            range_key = MODEL_RANGE.get(self.active_key)
            bottles = filter_by_range(bottles, range_key, orig_w * orig_h)

        return bottles, persons

    @property
    def active_name(self):
        if self.active_key and self.active_key in self.models:
            return self.models[self.active_key]["cfg"]["name"]
        return "none"

    @property
    def active_color(self):
        if self.active_key and self.active_key in self.models:
            return self.models[self.active_key]["cfg"]["color"]
        return (255, 255, 255)


# ══════════════════════════════════════════════════════════════
# Overlay
# ══════════════════════════════════════════════════════════════

def overlay_info(frame, detections, ms, model_name, model_color, mode, extra=""):
    h, w = frame.shape[:2]

    # Model badge (top-right)
    badge = f"[{mode}] {model_name}"
    (tw, th), bl = cv2.getTextSize(badge, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.rectangle(frame, (w - tw - 20, 0), (w, th + 16), model_color, -1)
    cv2.putText(frame, badge, (w - tw - 10, th + 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

    # Detection status (top-left)
    status = f"DETECTED ({len(detections)})" if detections else "NONE"
    color  = (0, 255, 0) if detections else (0, 200, 255)
    cv2.putText(frame, status, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    if ms > 0:
        cv2.putText(frame, f"Infer: {ms:.1f}ms  {1000/ms:.1f}FPS",
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)

    # Toggle hint (bottom)
    hint = "5=v5 7=v7 8=v8 Q=Quit | Held=filtered"
    cv2.putText(frame, hint, (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 120, 120), 1)

    if extra:
        cv2.putText(frame, extra, (10, h - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
    return frame


# ══════════════════════════════════════════════════════════════
# Camera loop
# ══════════════════════════════════════════════════════════════

def run_camera(manager, conf_thresh, camera_index, no_show, out_dir, mode):
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
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not no_show:
        cv2.namedWindow("PET Bottle Detection", cv2.WINDOW_NORMAL)

    print(f"\n{'='*60}")
    print(f"  PET Bottle Pickup Robot — Live Detection")
    print(f"  model={manager.active_name} | mode={mode}")
    print(f"  conf={conf_thresh} | Held-by-human filter: ON")
    print(f"  Detects ALL orientations: standing, lying, tilted")
    print(f"  Models: 5=YOLOv5  7=YOLOv7  8=YOLOv8  Q=Quit")
    print(f"{'='*60}\n")

    frame_idx = 0
    ms = 0.0
    tracker = DetectionTracker()

    try:
        while True:
            if use_picamera2:
                frame_rgb = picam2.capture_array()
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cap.read()
                if not ret:
                    print("  Frame capture failed")
                    break

            t0 = time.time()
            raw_dets, persons = manager.infer(frame, conf_thresh)
            ms = (time.time() - t0) * 1000

            detections = tracker.update(raw_dets)

            result = draw_detections(frame.copy(), detections, persons)
            range_tag = MODEL_RANGE.get(manager.active_key, "all")
            disp_label = f"SINGLE:{range_tag.upper() if range_tag else 'ALL'}"
            overlay_info(result, detections, ms,
                         manager.active_name, manager.active_color,
                         disp_label, f"Frame {frame_idx}")

            icon = "OK" if detections else "--"
            print(f"\r  [{icon}] {manager.active_name:8s} frame={frame_idx:5d} | "
                  f"{len(detections):2d} det | {ms:.1f}ms   ", end="", flush=True)

            if not no_show:
                cv2.imshow("PET Bottle Detection", result)
                key = cv2.waitKey(1) & 0xFF

                # ── Live model toggle ────────────────────────
                if key in (ord('q'), ord('Q'), 27):
                    break
                elif key == ord('5'):
                    manager.activate("yolov5")
                    tracker = DetectionTracker()  # reset tracker on switch
                elif key == ord('7'):
                    manager.activate("yolov7")
                    tracker = DetectionTracker()
                elif key == ord('8'):
                    manager.activate("yolov8")
                    tracker = DetectionTracker()
                elif key == ord('s'):
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


# ══════════════════════════════════════════════════════════════
# Image batch mode
# ══════════════════════════════════════════════════════════════

def run_images(manager, conf_thresh, images_dir, no_show, out_dir, single=None):
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

    model_keys = [manager.active_key]

    print(f"\n{'='*60}")
    print(f"  Testing {len(image_files)} images x {len(model_keys)} model(s)")
    print(f"  conf={conf_thresh}")
    print(f"{'='*60}")

    if not no_show:
        cv2.namedWindow("PET Bottle Detection", cv2.WINDOW_NORMAL)
        print("  Press ANY KEY for next | Q to quit\n")

    for model_key in model_keys:
        manager.activate(model_key)
        model_name = manager.active_name
        total_det = 0
        total_ms = 0

        for idx, img_path in enumerate(image_files):
            frame = cv2.imread(str(img_path))
            if frame is None:
                print(f"  Cannot load: {img_path.name}")
                continue

            t0 = time.time()
            detections, persons = manager.infer(frame, conf_thresh)
            ms = (time.time() - t0) * 1000
            total_det += len(detections)
            total_ms += ms

            result = draw_detections(frame.copy(), detections, persons)
            range_tag = MODEL_RANGE.get(manager.active_key, "all")
            overlay_info(result, detections, ms, model_name,
                         manager.active_color, f"SINGLE:{range_tag.upper() if range_tag else 'ALL'}",
                         f"[{idx+1}/{len(image_files)}] {img_path.name}")

            out_path = out_dir / f"result_{model_key}_{img_path.name}"
            cv2.imwrite(str(out_path), result)

            icon = "OK" if detections else "--"
            print(f"  [{icon}] {model_name:8s} [{idx+1:3d}/{len(image_files)}] "
                  f"{img_path.name:<30} | {len(detections):2d} det | {ms:.1f}ms")

            if not no_show:
                cv2.imshow("PET Bottle Detection", result)
                key = cv2.waitKey(0) & 0xFF
                if key in (ord('q'), ord('Q')):
                    break

        avg_ms = total_ms / len(image_files) if image_files else 0
        print(f"\n  --- {model_name} SUMMARY ---")
        print(f"  Total detections : {total_det}")
        print(f"  Avg infer time   : {avg_ms:.1f}ms  ({1000/avg_ms:.1f} FPS)" if avg_ms > 0 else "")

    if not no_show:
        cv2.destroyAllWindows()

    print(f"\n  Results saved to: {out_dir}/\n")


# ══════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="PET Bottle detection — YOLOv5/v7/v8 model stitching on Hailo-8"
    )
    parser.add_argument("--model", default="yolov8",
                        choices=["yolov5", "yolov7", "yolov8"],
                        help="Model to use (default: yolov8)")
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

    print(f"\n{'='*60}")
    print(f"  PET Bottle Detector — YOLO Model Stitching")
    print(f"{'='*60}")
    print(f"  Loading models...")

    with VDevice() as target:
        manager = ModelManager(target)

        manager.activate(args.model)

        print(f"\n  Active model: {manager.active_name}")

        try:
            if args.image:
                run_images(manager, args.conf, None, args.no_show, out_dir,
                           single=Path(args.image))
            elif args.images:
                run_images(manager, args.conf, args.images, args.no_show, out_dir)
            else:
                run_camera(manager, args.conf, args.camera, args.no_show, out_dir,
                           args.model)
        finally:
            manager.cleanup()


if __name__ == "__main__":
    main()
