# --- overlay drawing helpers ---
# controller_mp312.py
import json
import os
import time
from pathlib import Path

import cv2 as cv
import mediapipe as mp
import numpy as np

# ---------- CONFIG ----------
CONFIG_PATH = Path(__file__).with_name("config.json")


def _load_config(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as config_file:
            return json.load(config_file)
    except FileNotFoundError as exc:
        raise FileNotFoundError(
            f"Configuration file not found: {path}"
        ) from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError("Config JSON is malformed") from exc


_config = _load_config(CONFIG_PATH)

try:
    PORT = _config["PORT"]
    BAUD = int(_config["BAUD"])
    CAM_INDEX = int(_config.get("CAM_INDEX", 0))
    model_path_value = _config.get("MODEL_PATH", "hand_landmarker.task")
    MODEL_PATH = (
        model_path_value
        if os.path.isabs(model_path_value)
        else os.path.abspath(model_path_value)
    )
    NUM_HANDS = int(_config.get("NUM_HANDS", 1))
    SMOOTH_ALPHA = float(_config.get("SMOOTH_ALPHA", 0.7))
    SEND_HZ = float(_config.get("SEND_HZ", 20))
    PAN_TILT_DEFAULT = int(_config.get("PAN_TILT_DEFAULT", 180))
except KeyError as exc:
    raise RuntimeError("Missing required config key") from exc
except (TypeError, ValueError) as exc:
    raise RuntimeError("Invalid config value type") from exc

# -----------------------------

# MediaPipe hand skeleton pairs (index into the 21 landmarks)
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),        # index
    (5, 9), (9, 10), (10, 11), (11, 12),   # middle
    (9, 13), (13, 14), (14, 15), (15, 16),  # ring
    (13, 17), (17, 18), (18, 19), (19, 20)  # pinky
]

LABELS = ["T", "I", "M", "R", "P"]


def draw_hand_overlay(frame_bgr, landmarks):
    """
    landmarks: MediaPipe normalized landmarks (x, y in [0, 1])
    Draw connections + joints.
    """
    height, width = frame_bgr.shape[:2]
    for start_idx, end_idx in HAND_CONNECTIONS:
        start_landmark = landmarks[start_idx]
        end_landmark = landmarks[end_idx]
        start_x = int(start_landmark.x * width)
        start_y = int(start_landmark.y * height)
        end_x = int(end_landmark.x * width)
        end_y = int(end_landmark.y * height)
        cv.line(
            frame_bgr,
            (start_x, start_y),
            (end_x, end_y),
            (200, 200, 200),
            2,
        )

    for landmark in landmarks:
        pos_x = int(landmark.x * width)
        pos_y = int(landmark.y * height)
        cv.circle(frame_bgr, (pos_x, pos_y), 4, (255, 255, 255), -1)


class FPSMeter:
    def __init__(self, span=30):
        self.times = []
        self.span = span

    def tick(self):
        now = time.time()
        self.times.append(now)
        if len(self.times) > self.span:
            self.times.pop(0)

    def fps(self):
        if len(self.times) < 2:
            return 0.0
        delta_time = self.times[-1] - self.times[0]
        return (len(self.times) - 1) / delta_time if delta_time > 0 else 0.0


def put_hud(frame_bgr, angles5, fps, tx_hz):
    """Draw heads-up info on the frame."""
    y_pos = 24
    cv.putText(
        frame_bgr,
        f"FPS: {fps:5.1f}  TX: {tx_hz:4.1f} Hz",
        (10, y_pos),
        cv.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    y_pos += 24
    if angles5:
        text = "Angles: " + " ".join(
            f"{label}:{angle:3d}" for label, angle in zip(LABELS, angles5)
        )
        cv.putText(
            frame_bgr,
            text,
            (10, y_pos),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )


# ---------- MediaPipe setup (Python Tasks API) ----------
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(
        f"Model not found: {MODEL_PATH}\n"
        "Download 'Hand Landmarker (full)' .task file from the Models section "
        "of the MediaPipe Hand Landmarker docs and save it next to this "
        "script."
    )

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=VisionRunningMode.VIDEO,
    num_hands=NUM_HANDS,
    min_hand_detection_confidence=0.6,
    min_hand_presence_confidence=0.6,
    min_tracking_confidence=0.6,
)



# ---------- Main ----------
def main():
    cap = cv.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    with HandLandmarker.create_from_options(options) as landmarker:
        fpsm = FPSMeter()
        last_tx = time.time()
        tx_count = 0
        start_time = time.time()
        tx_hz = 0.0
        while True:
            fpsm.tick()
            ok, frame_bgr = cap.read()
            if not ok:
                break

            #mirror flip for user-friendly selfie view
            frame_bgr = cv.flip(frame_bgr, 1)
            frame_rgb = cv.cvtColor(frame_bgr, cv.COLOR_BGR2RGB)
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=frame_rgb,
            )

            timestamp_ms = int((time.time() - start_time) * 1000)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            angles = None
            has_hand_landmarks = result and result.hand_landmarks
            if has_hand_landmarks and len(result.hand_landmarks) > 0:
                landmarks = result.hand_landmarks[0]
                draw_hand_overlay(frame_bgr, landmarks)

            now = time.time()

            if now - last_tx >= 1.0:
                tx_hz = tx_count / (now - last_tx)
                last_tx = now
                tx_count = 0
            else:
                # reuse previous tx_hz to keep HUD stable
                pass

            # finally, draw HUD
            filtered_angles = angles if angles is not None else None
            put_hud(frame_bgr, filtered_angles, fpsm.fps(), tx_hz)

            cv.imshow(
                "uHand Controller (Py 3.12 + MediaPipe) (Overlay On)",
                frame_bgr,
            )
            if (cv.waitKey(1) & 0xFF) == 27:  # ESC
                break

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
