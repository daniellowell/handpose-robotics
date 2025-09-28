#!/usr/bin/env python3
"""
uHand controller (Python 3.12)

Two modes:
  1) Default: camera + MediaPipe → flex % → "P:p0..p5" to Arduino ~20 Hz
  2) --calibrate: interactive finger-by-finger limits (deg), saved to EEPROM

Finger order (matches your Arduino sketch):
  [thumb, index, middle, ring, pinky, swivel]

Requirements:
  pip install mediapipe opencv-python numpy pyserial
Place MediaPipe model file next to this script as: hand_landmarker.task
"""

# Standard library imports
import argparse
import math
import os
import time

# Third-party imports
import cv2 as cv
import numpy as np
import serial

# ------------------ User defaults ------------------
PORT = "/dev/tty.usbmodem1101"   # <-- set your UNO serial port
BAUD = 9600
MODEL_PATH = "hand_landmarker.task"
CAM_INDEX = 0
USE_AVFOUNDATION = True          # macOS camera backend
SEND_HZ = 20
SMOOTH_ALPHA = 0.6
MIRROR_PREVIEW = True            # mirror camera only (not math)
PRINT_TX = True                  # print each outbound serial line

# Constants
FLEXION_OPEN_THRESHOLD = -0.15
FLEXION_CLOSE_THRESHOLD = 0.30
ESC_KEY = 27
SPACE_KEY = 32
MIN_NORM_THRESHOLD = 1e-6
DEFAULT_NORM = 1e-4
SERIAL_COMMAND_DELAY = 0.05


FINGER_NAMES = ["thumb","index","middle","ring","pinky","swivel"]
# ---------------------------------------------------

# -------- MediaPipe (only used in live mode) --------
try:
    import mediapipe as mp
    MP_AVAILABLE = True
    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode
except Exception:
    MP_AVAILABLE = False
    mp = None  # ensure symbol exists when MediaPipe is unavailable
    BaseOptions = None  # type: ignore[assignment]
    HandLandmarker = None  # type: ignore[assignment]
    HandLandmarkerOptions = None  # type: ignore[assignment]
    VisionRunningMode = None  # type: ignore[assignment]

# MediaPipe landmark indices
TIP =  [4, 8,12,16,20]   # thumb..pinky
MCP =  [2, 5, 9,13,17]   # thumb CMC (2) works well; others MCPs
HAND_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (5,9),(9,10),(10,11),(11,12),
    (9,13),(13,14),(14,15),(15,16),
    (13,17),(17,18),(18,19),(19,20),
]

def draw_hand_overlay(frame_bgr, lm):
    h, w = frame_bgr.shape[:2]
    for a,b in HAND_CONNECTIONS:
        ax, ay = int(lm[a].x*w), int(lm[a].y*h)
        bx, by = int(lm[b].x*w), int(lm[b].y*h)
        cv.line(frame_bgr, (ax,ay), (bx,by), (200,200,200), 2)
    for p in lm:
        x, y = int(p.x*w), int(p.y*h)
        cv.circle(frame_bgr, (x,y), 4, (255,255,255), -1)

class FPSMeter:
    def __init__(self, span=30): self.t, self.span = [], span
    def tick(self):
        now = time.time(); self.t.append(now)
        if len(self.t) > self.span: self.t.pop(0)
    def fps(self):
        if len(self.t) < 2: return 0.0
        dt = self.t[-1]-self.t[0]
        return (len(self.t)-1)/dt if dt>0 else 0.0

def open_camera(index=0):
    if USE_AVFOUNDATION:
        cap = cv.VideoCapture(index, cv.CAP_AVFOUNDATION)
        if cap.isOpened(): return cap
    return cv.VideoCapture(index)

def ema_vec(prev, new, alpha=0.6):
    prev = np.asarray(prev, float); new = np.asarray(new, float)
    return (alpha*new + (1.0-alpha)*prev).tolist()


def flexion_percent(landmarks):
    """
    Return [thumb,index,middle,ring,pinky,swivel] in 0..100 (% closed)
    Heuristic: TIP below MCP => more closed. Normalize by hand size.
    """
    wrist = landmarks[0]; mid_mcp = landmarks[9]
    norm = math.hypot(mid_mcp.x - wrist.x, mid_mcp.y - wrist.y)
    if norm < MIN_NORM_THRESHOLD:
        norm = DEFAULT_NORM

    pcts = []
    for tip, mcp in zip(TIP, MCP):
        dy = landmarks[tip].y - landmarks[mcp].y  # + if tip lower than knuckle
        raw = dy / norm
        # Tune these for your camera angle / feel
        OPEN_AT, CLOSE_AT = FLEXION_OPEN_THRESHOLD, FLEXION_CLOSE_THRESHOLD
        pct = (raw - OPEN_AT) / (CLOSE_AT - OPEN_AT) * 100.0
        pct = max(0.0, min(100.0, pct))
        pcts.append(pct)

    pcts.append(50.0)  # swivel neutral (adjust if you want wrist yaw mapping)
    return pcts

# ---------------------- Serial ----------------------
def send_line(ser, text, print_tx=PRINT_TX):
    try:
        ser.write(text.encode())
        if print_tx: print(text.strip())
    except Exception as e:
        print(f"[WARN] serial write failed: {e}")

# -------------------- Live mode ---------------------
def run_live(args):
    if not MP_AVAILABLE:
        print("ERROR: MediaPipe not available. Install with: pip install mediapipe")
        return
    if not os.path.exists(args.model):
        print(f"Missing model file: {args.model}")
        return

    # Initialize variables first
    ser = None
    cap = None

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
        cap = open_camera(args.cam)
        
        # Check if camera was successfully opened
        if cap is None:
            if isinstance(args.cam, int):
                print(f"Camera device {args.cam} not found. Try a different camera index (0, 1, 2, etc.)")
            else:
                print(f"Failed to open camera from source: {args.cam}")
            return
            
        if not cap.isOpened():
            print("Could not open camera.")
            return
    
        # MediaPipe hand landmarker
        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=args.model),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.6,
            min_tracking_confidence=0.6,
        )

        fpsm = FPSMeter(); smooth = [50]*6
        send_period = 1.0 / max(1, args.send_hz); last_send = 0.0
        t0 = time.time()

        with HandLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = cap.read()
                if not ok: break
                rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                ts_ms = int((time.time()-t0)*1000)

                try:
                    result = landmarker.detect_for_video(mp_image, ts_ms)
                    pcts = None
                    if result and result.hand_landmarks:
                        lm = result.hand_landmarks[0]
                        pcts = flexion_percent(lm)
                        draw_hand_overlay(frame, lm)
                except Exception as e:
                    print(f"[WARN] MediaPipe detection failed: {e}")
                    pcts = None

                frame_vis = cv.flip(frame, 1) if MIRROR_PREVIEW else frame
                now = time.time()
                if pcts is not None:
                    smooth = ema_vec(smooth, pcts, args.smooth)
                    if (now - last_send) >= send_period:
                        vals = [int(round(v)) for v in smooth]              # 0..100
                        line = "P:{},{},{},{},{},{}\n".format(*vals)       # percent mode
                        send_line(ser, line, args.print_tx)
                        last_send = now

                fpsm.tick()
                fps = fpsm.fps()
                cv.putText(frame_vis, f"FPS {fps:4.1f}  TX {args.send_hz:02d}Hz",
                           (10,24), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                if pcts is not None:
                    labels = ["T","I","M","R","P","S"]
                    txt = " ".join(f"{l}:{int(v):3d}%" for l,v in zip(labels, smooth))
                    cv.putText(frame_vis, txt, (10,50), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                cv.imshow("uHand Pose Controller (P:%)", frame_vis)
                if (cv.waitKey(1) & 0xFF) == ESC_KEY: break  # ESC

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error in live mode: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        if cap and cap.isOpened():
            cap.release()
        cv.destroyAllWindows()

# ----------------- Calibrate mode -------------------
HELP_TEXT = """
Calibration keys (targeting ONE finger at a time):
  j / l : -1 / +1 degree
  J / L : -5 / +5 degrees
  m     : mark current as MIN for this finger
  n     : mark current as MAX for this finger
  space : next finger
  b     : back (previous finger)
  r     : reset angle to 90
  g     : send GET (print current limits from Arduino)
  s     : SAVE all limits to EEPROM
  q     : quit

Tip: keep movements between comfortable mechanical limits (no buzzing).
"""

def clamp(v, lo=0, hi=180): return max(lo, min(hi, v))

def run_calibrate(args):
    # Serial only (no MediaPipe/camera needed)
    ser = None
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"ERROR opening serial {args.port}: {e}"); return

    print(HELP_TEXT)
    # Degrees we are currently outputting (start neutral)
    deg = [90,90,90,90,90,90]
    # Per-finger min/max we're collecting (None until set)
    mins = [None] * 6
    maxs = [None] * 6

    finger = 0
    print(f"Calibrating finger 0 ({FINGER_NAMES[0]})")

    # Small OpenCV window just to capture keys & show status
    img = np.zeros((240,720,3), dtype=np.uint8)

    def update_status():
        img[:] = 0
        cv.putText(img, f"Calibrating: {FINGER_NAMES[finger]} (index {finger})",
                   (10,40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv.putText(img, f"Angle={deg[finger]:3d}   MIN={str(mins[finger])}   MAX={str(maxs[finger])}",
                   (10,80), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv.putText(img, "Keys: j/J - / l/L + | m set MIN | n set MAX | space next | b back",
                   (10,120), cv.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
        cv.putText(img, "r reset | g GET | s SAVE | q quit",
                   (10,150), cv.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
        cv.imshow("uHand Calibrate", img)

    def send_degrees():
        # Send full 6-value degrees line (Arduino treats as degrees mode)
        line = "{},{},{},{},{},{}\n".format(*[int(x) for x in deg])
        send_line(ser, line, args.print_tx)

    # move only the current finger, keep others at their last set deg
    send_degrees()
    update_status()

    while True:
        k = cv.waitKey(0) & 0xFF  # Mask to handle cross-platform key codes
        if k == 255:  # Invalid key, skip
            continue
            
        if k == ord('q'):
            break
        elif k == ord('j'):            # -1
            deg[finger] = clamp(deg[finger]-1)
            send_degrees()
        elif k == ord('l'):            # +1
            deg[finger] = clamp(deg[finger]+1)
            send_degrees()
        elif k == ord('J'):            # -5
            deg[finger] = clamp(deg[finger]-5)
            send_degrees()
        elif k == ord('L'):            # +5
            deg[finger] = clamp(deg[finger]+5)
            send_degrees()
        elif k == ord('r'):
            deg[finger] = 90
            send_degrees()
        elif k == ord('m'):
            mins[finger] = int(deg[finger])
            print(f"MIN set for {FINGER_NAMES[finger]} -> {mins[finger]}")
        elif k == ord('n'):
            maxs[finger] = int(deg[finger])
            print(f"MAX set for {FINGER_NAMES[finger]} -> {maxs[finger]}")
        elif k == ord('g'):
            send_line(ser, "GET\n", args.print_tx)
        elif k == ord('s'):
            # Push any collected mins/maxs to Arduino with CAL, then SAVE
            for i in range(6):
                m = mins[i]
                M = maxs[i]
                if m is not None and M is not None:
                    lo, hi = sorted((m, M))
                    send_line(ser, f"CAL {i} {lo} {hi}\n", args.print_tx)
                    time.sleep(SERIAL_COMMAND_DELAY)
            send_line(ser, "SAVE\n", args.print_tx)
            print("Saved to EEPROM.")
        elif k == ord(' '):  # space : next finger (use ord(' ') instead of SPACE_KEY)
            finger = (finger + 1) % 6
            print(f"→ Next: {finger} ({FINGER_NAMES[finger]})")
            send_degrees()
        elif k == ord('b'):  # back
            finger = (finger - 1) % 6
            print(f"→ Back: {finger} ({FINGER_NAMES[finger]})")
            send_degrees()

        update_status()

    cv.destroyAllWindows()
    ser.close()
    print("Calibration mode finished.")

# -------------------- CLI entry ---------------------
def main():
    p = argparse.ArgumentParser(description="uHand controller")
    p.add_argument("--port", default=PORT)
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--model", default=MODEL_PATH)
    p.add_argument("--cam", type=int, default=CAM_INDEX)
    p.add_argument("--send-hz", type=int, default=SEND_HZ)
    p.add_argument("--smooth", type=float, default=SMOOTH_ALPHA)
    p.add_argument("--no-print-tx", dest="print_tx", action="store_false",
                   help="do not print outbound serial lines")
    p.add_argument("--calibrate", action="store_true",
                   help="interactive per-finger min/max setup (no camera)")

    args = p.parse_args()
    if args.calibrate:
        run_calibrate(args)
    else:
        run_live(args)

if __name__ == "__main__":
    main()
# EOF
