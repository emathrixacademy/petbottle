"""
Vision AI — advanced target verification and scene analysis.
Uses cloud vision API for intelligent object classification
and navigation strategy beyond what the local ML model can achieve.
"""

import base64
import json
import os
import time
import urllib.request
import urllib.error
import cv2

API_KEY = os.environ.get("ANTHROPIC_API_KEY", "")
API_URL = "https://api.anthropic.com/v1/messages"
MODEL = "claude-haiku-4-5-20251001"

_last_call_time = 0
MIN_CALL_INTERVAL = 1.0


def _encode_frame(frame, quality=60):
    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode('utf-8')


def _call_api(prompt, image_b64, max_tokens=200):
    global _last_call_time
    if not API_KEY:
        return None

    elapsed = time.time() - _last_call_time
    if elapsed < MIN_CALL_INTERVAL:
        time.sleep(MIN_CALL_INTERVAL - elapsed)

    headers = {
        "Content-Type": "application/json",
        "x-api-key": API_KEY,
        "anthropic-version": "2023-06-01"
    }

    body = json.dumps({
        "model": MODEL,
        "max_tokens": max_tokens,
        "messages": [{
            "role": "user",
            "content": [
                {
                    "type": "image",
                    "source": {
                        "type": "base64",
                        "media_type": "image/jpeg",
                        "data": image_b64
                    }
                },
                {
                    "type": "text",
                    "text": prompt
                }
            ]
        }]
    }).encode('utf-8')

    req = urllib.request.Request(API_URL, data=body, headers=headers)
    try:
        _last_call_time = time.time()
        resp = urllib.request.urlopen(req, timeout=15)
        result = json.loads(resp.read().decode('utf-8'))
        return result["content"][0]["text"]
    except Exception as e:
        print(f"  [AI] error: {e}")
        return None


def _parse_json(response):
    if not response:
        return None
    try:
        start = response.index('{')
        end = response.rindex('}') + 1
        return json.loads(response[start:end])
    except (ValueError, json.JSONDecodeError):
        return None


def verify_target(frame, bbox):
    """Verify if a detected object is actually a PET bottle.
    bbox: (x1, y1, x2, y2) or (x1, y1, x2, y2, conf).
    Returns: (is_pet_bottle: bool, reason: str)
    """
    x1, y1, x2, y2 = [int(c) for c in bbox[:4]]
    h, w = frame.shape[:2]
    pad = 30
    x1 = max(0, x1 - pad)
    y1 = max(0, y1 - pad)
    x2 = min(w, x2 + pad)
    y2 = min(h, y2 + pad)

    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return False, "empty crop"

    img_b64 = _encode_frame(crop, quality=70)

    prompt = (
        "Is this a PET plastic bottle (water, soda, juice bottle)? "
        "Respond ONLY in JSON: {\"is_pet\": true/false, \"reason\": \"one sentence\"}\n"
        "true = PET plastic bottle. false = cup, can, bag, glass bottle, "
        "cardboard, shoe, or any non-PET item."
    )

    response = _call_api(prompt, img_b64, max_tokens=80)
    data = _parse_json(response)
    if data:
        return data.get("is_pet", False), data.get("reason", "unknown")

    return True, "api unavailable — trusting local model"


def analyze_scene(frame, sensor_data):
    """Analyze the full scene and suggest navigation strategy.
    Returns: dict with 'action', 'reason', 'bottles_spotted'
    Actions: 'turn_left', 'turn_right', 'go_forward', 'stop'
    """
    img_b64 = _encode_frame(frame, quality=50)

    us = sensor_data.get("ultrasonic", {})
    prompt = (
        "You are the navigation brain of a PET bottle collecting robot on the ground. "
        f"Ultrasonic distances: front={us.get('s1', 999)}cm, right={us.get('s2', 999)}cm, "
        f"back={us.get('s3', 999)}cm, left={us.get('s4', 999)}cm.\n"
        "Look at this camera image from the robot's perspective. "
        "Do you see any PET bottles on the ground? Where should the robot go?\n"
        "Respond ONLY in JSON: {\"action\": \"turn_left\"/\"turn_right\"/\"go_forward\"/\"stop\", "
        "\"reason\": \"one sentence\", \"bottles_spotted\": 0}"
    )

    response = _call_api(prompt, img_b64, max_tokens=120)
    data = _parse_json(response)
    if data:
        return data

    return {"action": "go_forward", "reason": "api unavailable", "bottles_spotted": 0}


def assess_obstacle(frame, sensor_data):
    """Identify obstacle and suggest best avoidance direction.
    Returns: dict with 'avoid_direction' ('left'/'right'/'backward'), 'obstacle', 'reason'
    """
    img_b64 = _encode_frame(frame, quality=50)

    us = sensor_data.get("ultrasonic", {})
    prompt = (
        "A ground robot detected an obstacle ahead. "
        f"Ultrasonic: front={us.get('s1', 999)}cm, right={us.get('s2', 999)}cm, "
        f"back={us.get('s3', 999)}cm, left={us.get('s4', 999)}cm.\n"
        "What is the obstacle? Which direction is safest to go around?\n"
        "Respond ONLY in JSON: {\"obstacle\": \"what it is\", "
        "\"avoid_direction\": \"left\"/\"right\"/\"backward\", \"reason\": \"one sentence\"}"
    )

    response = _call_api(prompt, img_b64, max_tokens=100)
    data = _parse_json(response)
    if data:
        return data

    if us.get("s2", 999) > us.get("s4", 999):
        return {"avoid_direction": "right", "obstacle": "unknown", "reason": "api unavailable — more space on right"}
    return {"avoid_direction": "left", "obstacle": "unknown", "reason": "api unavailable — more space on left"}
