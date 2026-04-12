#!/usr/bin/env python3
"""
PET Bottle Robot — Autonomous Navigator
Runs on Raspberry Pi. Fuses camera (YOLO) + ultrasonic sensors from ESP32
for real-time obstacle avoidance and bottle pickup.

The robot continuously:
  1. Roams open space looking for PET bottles
  2. Avoids obstacles (ultrasonics + camera person/object detection)
  3. Approaches detected bottles
  4. Triggers the pickup sequence on the ESP32
  5. Resumes roaming

Usage:
  python3 navigator.py                    # default (YOLOv8, auto-connect)
  python3 navigator.py --model all        # ensemble mode
  python3 navigator.py --esp32-ip 192.168.4.1
  python3 navigator.py --no-show          # headless
"""

import cv2
import numpy as np
import argparse
import time
import threading
import requests
import json
from pathlib import Path
from enum import Enum, auto

# Import detection system from camera.py
from camera import (
    ModelManager, MODEL_CONFIGS, MODEL_RANGE,
    DetectionTracker, draw_detections, preprocess,
    postprocess_nms, postprocess_raw, filter_held_bottles,
    get_orientation, overlay_info,
    CONF_THRESHOLD, COCO_BOTTLE_ID, COCO_PERSON_ID,
)
from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams, FormatType
)


# ══════════════════════════════════════════════════════════════
# Configuration
# ══════════════════════════════════════════════════════════════

ESP32_IP        = "192.168.4.1"
ESP32_TIMEOUT   = 0.5           # HTTP timeout (seconds)
SENSOR_POLL_HZ  = 10            # ultrasonic poll rate

# Obstacle avoidance thresholds (cm)
# Robot is ~100cm long x 30cm wide — sensors are at the front
STOP_DIST       = 50            # emergency stop — back up immediately
SLOW_DIST       = 80            # reduce speed
TURN_DIST       = 120           # start steering away (1.2m clearance)

# Driving speeds (ESP32 caps at DRIVE_MAX=80, drivers explode above 100)
ROAM_SPEED      = 60            # normal roaming speed
SLOW_SPEED      = 35            # near obstacles
APPROACH_SPEED  = 40            # approaching a bottle
TURN_SPEED      = 50            # turning speed

# Bottle approach
BOTTLE_CLOSE_FILL = 0.15        # bottle fills 15% of frame → close enough to pick
BOTTLE_CENTER_TOL = 0.15        # tolerance from frame center (fraction)

# COCO obstacle classes (things the robot should avoid)
COCO_OBSTACLE_CLASSES = {
    0: "person", 56: "chair", 57: "couch", 58: "potted plant",
    59: "bed", 60: "dining table", 62: "tv",
}


# ══════════════════════════════════════════════════════════════
# Robot States
# ══════════════════════════════════════════════════════════════

class State(Enum):
    ROAMING     = auto()   # exploring open space, looking for bottles
    APPROACHING = auto()   # bottle detected, driving towards it
    ALIGNING    = auto()   # close to bottle, fine-tuning position
    PICKING_UP  = auto()   # running pickup sequence on ESP32
    AVOIDING    = auto()   # obstacle detected, steering away
    STOPPED     = auto()   # emergency stop / idle


# ══════════════════════════════════════════════════════════════
# ESP32 Communication (thread-safe, non-blocking)
# ══════════════════════════════════════════════════════════════

class ESP32Link:
    """Talks to the ESP32 robot controller over WiFi HTTP."""

    def __init__(self, ip=ESP32_IP):
        self.base_url = f"http://{ip}"
        self.sensor_data = {"left": 999, "right": 999, "motors": 1}
        self._lock = threading.Lock()
        self._running = True
        self._sensor_thread = threading.Thread(target=self._poll_sensors, daemon=True)
        self._sensor_thread.start()

    def _poll_sensors(self):
        """Background thread: polls /sensor at SENSOR_POLL_HZ."""
        while self._running:
            try:
                r = requests.get(f"{self.base_url}/sensor", timeout=ESP32_TIMEOUT)
                data = r.json()
                with self._lock:
                    self.sensor_data = data
            except Exception:
                pass  # connection hiccup — keep last known values
            time.sleep(1.0 / SENSOR_POLL_HZ)

    @property
    def ultrasonic(self):
        """Returns (left_cm, right_cm)."""
        with self._lock:
            return self.sensor_data.get("left", 999), self.sensor_data.get("right", 999)

    @property
    def is_connected(self):
        with self._lock:
            return self.sensor_data.get("motors", 0) == 1

    def cmd(self, command):
        """Send a command to the ESP32 (non-blocking)."""
        try:
            requests.get(f"{self.base_url}/cmd", params={"c": command},
                         timeout=ESP32_TIMEOUT)
        except Exception:
            pass

    def forward(self, speed=ROAM_SPEED):
        self.cmd(f"F{speed}")

    def backward(self, speed=ROAM_SPEED):
        self.cmd(f"J{speed}")

    def turn_left(self, speed=TURN_SPEED):
        self.cmd(f"TL{speed}")

    def turn_right(self, speed=TURN_SPEED):
        self.cmd(f"TR{speed}")

    def stop(self):
        self.cmd("X")

    def pickup(self):
        """Triggers the full pickup sequence on ESP32 (blocking ~15s)."""
        try:
            requests.get(f"{self.base_url}/cmd", params={"c": "P"},
                         timeout=30)
        except Exception:
            pass

    def shutdown(self):
        self._running = False
        self.stop()


# ══════════════════════════════════════════════════════════════
# Autonomous Navigator
# ══════════════════════════════════════════════════════════════

class Navigator:
    """
    Main brain: fuses camera + ultrasonics for real-time navigation.
    Roams → detects bottle → approaches → picks up → resumes.
    """

    def __init__(self, manager, esp32, conf_thresh, ensemble, no_show, out_dir):
        self.manager     = manager
        self.esp32       = esp32
        self.conf_thresh = conf_thresh
        self.ensemble    = ensemble
        self.no_show     = no_show
        self.out_dir     = out_dir

        self.state       = State.ROAMING
        self.tracker     = DetectionTracker()
        self.turn_dir    = 1         # 1 = right, -1 = left (roaming direction)
        self.turn_timer  = 0.0
        self.avoid_timer = 0.0
        self.approach_lost_count = 0

    def run(self):
        """Main loop — camera + sensors + navigation."""
        # Camera setup
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
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("Cannot open camera")
                return
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.no_show:
            cv2.namedWindow("PET Bottle Navigator", cv2.WINDOW_NORMAL)

        print(f"\n{'='*60}")
        print(f"  PET Bottle Robot — AUTONOMOUS NAVIGATOR")
        print(f"  Camera + Ultrasonic Fusion")
        print(f"  ESP32: {self.esp32.base_url}")
        print(f"  State: ROAMING | Q to quit | S to stop/resume")
        print(f"{'='*60}\n")

        frame_idx = 0

        try:
            while True:
                # ── Grab frame ────────────────────────────────
                if use_picamera2:
                    frame_rgb = picam2.capture_array()
                    frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                else:
                    ret, frame = cap.read()
                    if not ret:
                        break

                orig_h, orig_w = frame.shape[:2]
                frame_area = orig_w * orig_h

                # ── Run detection ─────────────────────────────
                t0 = time.time()
                if self.ensemble:
                    bottles, persons = self.manager.infer_ensemble(frame, self.conf_thresh)
                else:
                    bottles, persons = self.manager.infer(
                        frame, self.conf_thresh, apply_range_filter=False)
                ms = (time.time() - t0) * 1000

                smoothed = self.tracker.update(bottles)

                # ── Get ultrasonic data ───────────────────────
                us_left, us_right = self.esp32.ultrasonic

                # ── Navigation decision ───────────────────────
                self._navigate(smoothed, persons, orig_w, orig_h,
                               frame_area, us_left, us_right)

                # ── Draw results ──────────────────────────────
                result = draw_detections(frame.copy(), smoothed, persons)
                self._draw_nav_overlay(result, smoothed, us_left, us_right, ms, frame_idx)

                # ── Console output ────────────────────────────
                icon = "OK" if smoothed else "--"
                print(f"\r  [{icon}] {self.state.name:12s} "
                      f"US L:{us_left:3d} R:{us_right:3d} | "
                      f"{len(smoothed)} det | {ms:.0f}ms   ",
                      end="", flush=True)

                # ── Display + keyboard ────────────────────────
                if not self.no_show:
                    cv2.imshow("PET Bottle Navigator", result)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), ord('Q'), 27):
                        break
                    elif key in (ord('s'), ord('S')):
                        if self.state == State.STOPPED:
                            self.state = State.ROAMING
                            print("\n  >>> RESUMED")
                        else:
                            self.esp32.stop()
                            self.state = State.STOPPED
                            print("\n  >>> MANUAL STOP")

                frame_idx += 1

        finally:
            self.esp32.stop()
            if use_picamera2:
                picam2.stop()
            else:
                cap.release()
            if not self.no_show:
                cv2.destroyAllWindows()
            print()

    # ── Navigation logic ──────────────────────────────────────

    def _navigate(self, bottles, persons, w, h, frame_area, us_left, us_right):
        """State machine: decide what the robot does each frame."""
        us_min = min(us_left, us_right)

        # ── Emergency stop: ultrasonic too close ──────────
        if us_min < STOP_DIST and self.state != State.PICKING_UP:
            self.esp32.stop()
            self.state = State.AVOIDING
            self.avoid_timer = time.time()
            return

        # ── Avoiding state: back up and turn away ─────────
        if self.state == State.AVOIDING:
            elapsed = time.time() - self.avoid_timer
            if elapsed < 0.8:
                self.esp32.backward(SLOW_SPEED)
            elif elapsed < 1.8:
                if us_left < us_right:
                    self.esp32.turn_right(TURN_SPEED)
                else:
                    self.esp32.turn_left(TURN_SPEED)
            else:
                self.state = State.ROAMING
            return

        # ── Stopped state: do nothing ─────────────────────
        if self.state == State.STOPPED:
            return

        # ── Picking up: wait for sequence to finish ───────
        if self.state == State.PICKING_UP:
            return

        # ── Camera-based obstacle check (persons nearby) ──
        for p in persons:
            pw = p[2] - p[0]
            ph = p[3] - p[1]
            p_fill = (pw * ph) / frame_area
            if p_fill > 0.20:  # person is large/close → avoid
                self.esp32.stop()
                self.state = State.AVOIDING
                self.avoid_timer = time.time()
                return

        # ── Ultrasonic: slow zone ─────────────────────────
        speed = ROAM_SPEED
        if us_min < SLOW_DIST:
            speed = SLOW_SPEED
        if us_min < TURN_DIST and us_left != us_right:
            # Steer away from closer side
            if us_left < us_right:
                self.esp32.turn_right(TURN_SPEED)
                return
            else:
                self.esp32.turn_left(TURN_SPEED)
                return

        # ── Bottle detected? ──────────────────────────────
        if bottles:
            # Pick the largest (closest) bottle
            best = max(bottles, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
            bx1, by1, bx2, by2, bconf = best
            bw = bx2 - bx1
            bh = by2 - by1
            b_fill = (bw * bh) / frame_area
            b_cx = (bx1 + bx2) / 2.0
            b_cx_norm = b_cx / w  # 0.0 = left, 1.0 = right

            if b_fill >= BOTTLE_CLOSE_FILL:
                # Close enough — check alignment then pick up
                if abs(b_cx_norm - 0.5) < BOTTLE_CENTER_TOL:
                    self._start_pickup()
                    return
                else:
                    # Align: steer towards bottle center
                    self.state = State.ALIGNING
                    if b_cx_norm < 0.5:
                        self.esp32.turn_left(50)
                    else:
                        self.esp32.turn_right(50)
                    return
            else:
                # Bottle seen but far — approach it
                self.state = State.APPROACHING
                self.approach_lost_count = 0
                if abs(b_cx_norm - 0.5) > 0.25:
                    # Steer towards bottle
                    if b_cx_norm < 0.5:
                        self.esp32.cmd(f"W{APPROACH_SPEED//2},{APPROACH_SPEED}")
                    else:
                        self.esp32.cmd(f"W{APPROACH_SPEED},{APPROACH_SPEED//2}")
                else:
                    self.esp32.forward(APPROACH_SPEED)
                return

        # ── No bottle: roam ───────────────────────────────
        if self.state == State.APPROACHING:
            self.approach_lost_count += 1
            if self.approach_lost_count > 15:  # lost bottle for ~15 frames
                self.state = State.ROAMING
        else:
            self.state = State.ROAMING

        if self.state == State.ROAMING:
            now = time.time()
            # Periodically change direction to explore
            if now - self.turn_timer > 8.0:
                self.turn_timer = now
                self.turn_dir *= -1  # alternate left/right sweeps

            # Gentle S-curve roaming: mostly forward with slight turns
            cycle = (now % 4.0) / 4.0  # 0..1 every 4 seconds
            if cycle < 0.7:
                self.esp32.forward(speed)
            else:
                if self.turn_dir > 0:
                    self.esp32.turn_right(TURN_SPEED)
                else:
                    self.esp32.turn_left(TURN_SPEED)

    def _start_pickup(self):
        """Stop, run pickup sequence in a thread, then resume roaming."""
        self.state = State.PICKING_UP
        self.esp32.stop()
        time.sleep(0.3)

        def _do_pickup():
            print("\n  >>> PICKUP SEQUENCE STARTED")
            self.esp32.pickup()
            print("\n  >>> PICKUP DONE — resuming roam")
            self.tracker = DetectionTracker()  # reset tracker
            self.state = State.ROAMING

        threading.Thread(target=_do_pickup, daemon=True).start()

    # ── HUD overlay ───────────────────────────────────────

    def _draw_nav_overlay(self, frame, bottles, us_left, us_right, ms, frame_idx):
        """Draw navigation HUD on frame."""
        h, w = frame.shape[:2]

        # State badge (top-right)
        state_colors = {
            State.ROAMING:     (0, 255, 0),
            State.APPROACHING: (0, 200, 255),
            State.ALIGNING:    (255, 200, 0),
            State.PICKING_UP:  (255, 0, 255),
            State.AVOIDING:    (0, 0, 255),
            State.STOPPED:     (80, 80, 80),
        }
        color = state_colors.get(self.state, (255, 255, 255))
        badge = f"[{self.state.name}]"
        (tw, th), _ = cv2.getTextSize(badge, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.rectangle(frame, (w - tw - 20, 0), (w, th + 16), color, -1)
        cv2.putText(frame, badge, (w - tw - 10, th + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        # Ultrasonic bars (bottom-left and bottom-right)
        bar_max_h = 100
        bar_w = 30
        margin = 10

        for side, dist, x_pos in [
            ("L", us_left, margin),
            ("R", us_right, w - bar_w - margin),
        ]:
            fill_h = int(min(dist, 200) / 200.0 * bar_max_h)
            bar_color = (0, 255, 0) if dist > SLOW_DIST else (
                (0, 200, 255) if dist > STOP_DIST else (0, 0, 255)
            )
            bar_top = h - margin - bar_max_h
            cv2.rectangle(frame, (x_pos, bar_top),
                          (x_pos + bar_w, h - margin), (40, 40, 40), -1)
            cv2.rectangle(frame, (x_pos, bar_top + (bar_max_h - fill_h)),
                          (x_pos + bar_w, h - margin), bar_color, -1)
            cv2.putText(frame, f"{side}:{dist}",
                        (x_pos, bar_top - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # Detection count + FPS (top-left)
        status = f"BOTTLES: {len(bottles)}" if bottles else "SCANNING..."
        det_color = (0, 255, 0) if bottles else (0, 200, 255)
        cv2.putText(frame, status, (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, det_color, 2)
        if ms > 0:
            cv2.putText(frame, f"{ms:.0f}ms  {1000/ms:.0f}FPS",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (200, 200, 200), 1)

        # Bottom hint
        cv2.putText(frame, "S=Stop/Resume  Q=Quit",
                    (w // 2 - 100, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)


# ══════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="PET Bottle Robot — Autonomous Navigator (Pi + ESP32)"
    )
    parser.add_argument("--model", default="yolov8",
                        choices=["yolov5", "yolov7", "yolov8", "all"],
                        help="YOLO model (default: yolov8). 'all' = ensemble")
    parser.add_argument("--esp32-ip", default=ESP32_IP,
                        help=f"ESP32 WiFi IP (default: {ESP32_IP})")
    parser.add_argument("--conf", type=float, default=CONF_THRESHOLD,
                        help=f"Confidence threshold (default: {CONF_THRESHOLD})")
    parser.add_argument("--no-show", action="store_true",
                        help="Headless mode (no display)")
    parser.add_argument("--output", default="./results",
                        help="Directory to save snapshots")
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"  PET Bottle Robot — Autonomous Navigator")
    print(f"  Camera + Ultrasonic Fusion")
    print(f"{'='*60}")

    # Connect to ESP32
    print(f"  Connecting to ESP32 at {args.esp32_ip}...")
    esp32 = ESP32Link(ip=args.esp32_ip)
    time.sleep(0.5)
    us_l, us_r = esp32.ultrasonic
    if us_l < 999 or us_r < 999:
        print(f"  ESP32 connected! Ultrasonic: L={us_l}cm R={us_r}cm")
    else:
        print(f"  WARNING: ESP32 not responding — running camera-only mode")

    # Load YOLO models
    print(f"  Loading YOLO models...")
    with VDevice() as target:
        manager = ModelManager(target)

        if args.model == "all":
            first_key = list(manager.models.keys())[0]
            manager.activate(first_key)
        else:
            manager.activate(args.model)

        ensemble = (args.model == "all")
        print(f"  Active: {manager.active_name} | Ensemble: {ensemble}")

        try:
            nav = Navigator(manager, esp32, args.conf, ensemble,
                            args.no_show, out_dir)
            nav.run()
        finally:
            esp32.shutdown()
            manager.cleanup()


if __name__ == "__main__":
    main()
