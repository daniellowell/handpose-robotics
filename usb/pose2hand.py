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
import json
import math
import os
import platform
import signal
import sys
import time

# Third-party imports
import cv2 as cv
import numpy as np
import serial

# ------------------ User defaults ------------------
PORT = "/dev/tty.usbmodem1101"   # Serial device for the UNO (update per environment)
BAUD = 9600                       # Baud rate expected by the Arduino firmware
MODEL_PATH = "hand_landmarker.task"  # MediaPipe model path
CAM_INDEX = 0                     # Default camera index used by OpenCV
USE_AVFOUNDATION = True           # Prefer macOS AVFoundation backend when present
SEND_HZ = 20                      # Desired serial update rate (Hz)
SMOOTH_ALPHA = 0.6                # EMA smoothing factor for finger percentages
MIRROR_PREVIEW = True             # Mirror preview only; math still uses original orientation
PRINT_TX = True                   # Emit outbound serial lines to stdout

# Constants
FLEXION_OPEN_THRESHOLD = -0.15    # Normalised TIP-MCP delta treated as fully open (0 %)
FLEXION_CLOSE_THRESHOLD = 0.30    # Normalised TIP-MCP delta treated as fully closed (100 %)
ESC_KEY = 27                      # ESC keycode for OpenCV loop exit
SPACE_KEY = 32                    # Spacebar keycode used to advance fingers during calibration
MIN_NORM_THRESHOLD = 1e-6         # Lower guard for hand span calculation
DEFAULT_NORM = 1e-4               # Fallback normaliser when span is extremely small
SERIAL_LIVE_TIMEOUT = 0.05        # Serial timeout while streaming live pose data
SERIAL_CAL_TIMEOUT = 0.1          # Serial timeout during calibration routine
SERIAL_COMMAND_DELAY = 0.05       # Pause between calibration packets to let servos settle
CAL_SETTLE_REPEATS = 4            # Number of repeated calibration sends to overcome EMA smoothing
CALIB_CLAMP_MIN = 0               # Lower bound for servo angles during calibration
CALIB_CLAMP_MAX = 180             # Upper bound for servo angles during calibration
CALIB_START_DEG = 90              # Neutral angle applied when calibration starts/resets
SWIVEL_NEUTRAL_PERCENT = 50.0     # Default percent value for the swivel channel
MP_MIN_HAND_DETECTION_CONF = 0.6  # MediaPipe minimum detection confidence
MP_MIN_HAND_PRESENCE_CONF = 0.6   # MediaPipe minimum presence confidence
MP_MIN_TRACKING_CONF = 0.6        # MediaPipe minimum tracking confidence


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

def module_version(mod, attr="__version__", fallback_attr="VERSION", default="unknown"):
    if mod is None:
        return default
    if hasattr(mod, attr):
        return getattr(mod, attr)
    if hasattr(mod, fallback_attr):
        return getattr(mod, fallback_attr)
    return default


def build_env_snapshot(port):
    return {
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "opencv": module_version(cv),
        "mediapipe": module_version(mp) if MP_AVAILABLE else "missing",
        "pyserial": module_version(serial),
        "port": port,
    }


# Visualise the MediaPipe landmarks so the operator can see what the tracker detects.
def draw_hand_overlay(frame_bgr, lm):
    h, w = frame_bgr.shape[:2]
    for a,b in HAND_CONNECTIONS:
        ax, ay = int(lm[a].x*w), int(lm[a].y*h)
        bx, by = int(lm[b].x*w), int(lm[b].y*h)
        cv.line(frame_bgr, (ax,ay), (bx,by), (200,200,200), 2)
    for p in lm:
        x, y = int(p.x*w), int(p.y*h)
        cv.circle(frame_bgr, (x,y), 4, (255,255,255), -1)

# Rolling FPS measurement used for the HUD overlay.
class FPSMeter:
    def __init__(self, span=30): self.t, self.span = [], span
    def tick(self):
        now = time.time(); self.t.append(now)
        if len(self.t) > self.span: self.t.pop(0)
    def fps(self):
        if len(self.t) < 2: return 0.0
        dt = self.t[-1]-self.t[0]
        return (len(self.t)-1)/dt if dt>0 else 0.0

# Try AVFoundation first (macOS), then fall back to the default backend.
def open_camera(index=0):
    if USE_AVFOUNDATION:
        cap = cv.VideoCapture(index, cv.CAP_AVFOUNDATION)
        if cap.isOpened(): return cap
    return cv.VideoCapture(index)

# Simple vector EMA used to smooth noisy flexion percentages.
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

    pcts.append(SWIVEL_NEUTRAL_PERCENT)  # swivel neutral (adjust if you want wrist yaw mapping)
    return pcts

# ---------------------- Serial ----------------------
# Send a newline-terminated command to the Arduino and optionally log diagnostics.
def send_line(ser, text, print_tx=PRINT_TX, log_fn=None, log_event_name="serial_write", log_data=None):
    try:
        ser.write(text.encode())
        if log_fn:
            payload = {"line": text.strip()}
            if log_data:
                payload.update(log_data)
            log_fn(log_event_name, **payload)
        if print_tx:
            print(text.strip())
    except Exception as e:
        if log_fn:
            log_fn("serial_write_error", line=text.strip(), error=str(e))
        print(f"[WARN] serial write failed: {e}")

# -------------------- Live mode ---------------------
# Capture MediaPipe landmarks, map them to servo percentages, and stream packets.
def run_live(args):
    log_handle = None
    if args.log_json:
        try:
            log_handle = open(args.log_json, "a", encoding="utf-8")
        except OSError as exc:
            print(f"[WARN] could not open log file {args.log_json}: {exc}")
            log_handle = None

    def log_event(event, **data):
        if not args.log_json and log_handle is None:
            return
        entry = {"ts": time.time(), "event": event}
        entry.update(data)
        if log_handle:
            log_handle.write(json.dumps(entry) + "\n")
            log_handle.flush()
        if args.log_json:
            print(json.dumps(entry))

    ser = None
    cap = None
    hand_present_prev = None
    frame_index = 0

    env_snapshot = build_env_snapshot(args.port)
    log_event("env_snapshot", **env_snapshot)

    shutdown_requested = False
    sent_neutral = False
    original_handlers = {}

    def handle_signal(signum, frame):
        nonlocal shutdown_requested, sent_neutral
        if shutdown_requested:
            return
        shutdown_requested = True
        log_event("shutdown", signal=signum)
        if ser and ser.is_open and not sent_neutral:
            send_line(ser, "NEUTRAL\n", args.print_tx, log_event, "serial_send", {"reason": "shutdown"})
            sent_neutral = True
        cv.destroyAllWindows()
        raise KeyboardInterrupt()

    for sig in (signal.SIGINT, signal.SIGTERM):
        original_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, handle_signal)

    try:
        if not MP_AVAILABLE:
            log_event("live_error", reason="mediapipe_unavailable")
            print("ERROR: MediaPipe not available. Install with: pip install mediapipe")
            return
        if not os.path.exists(args.model):
            log_event("model_missing", path=args.model)
            print(f"Missing model file: {args.model}")
            print("Download with:")
            print("  curl -o hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")
            sys.exit(1)

        try:
            model_size = os.path.getsize(args.model)
        except OSError:
            model_size = None
        log_event("model_loaded", path=args.model, size=model_size)

        log_event("live_start", port=args.port, baud=args.baud, cam=args.cam,
                  send_hz=args.send_hz, smooth=args.smooth, mirror=MIRROR_PREVIEW)

        ser = serial.Serial(args.port, args.baud, timeout=SERIAL_LIVE_TIMEOUT)
        log_event("serial_open", status="ok", port=args.port, baud=args.baud)

        cap = open_camera(args.cam)
        if cap is None:
            log_event("camera_open_failed", cam=args.cam, reason="open_camera_none")
            if isinstance(args.cam, int):
                print(f"Camera device {args.cam} not found. Try a different camera index (0, 1, 2, etc.)")
            else:
                print(f"Failed to open camera from source: {args.cam}")
            return

        if not cap.isOpened():
            log_event("camera_open_failed", cam=args.cam, reason="not_opened")
            print("Could not open camera.")
            return

        log_event("camera_open", cam=args.cam)

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=args.model),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=MP_MIN_HAND_DETECTION_CONF,
            min_hand_presence_confidence=MP_MIN_HAND_PRESENCE_CONF,
            min_tracking_confidence=MP_MIN_TRACKING_CONF,
        )
        log_event("mediapipe_init", model=args.model, num_hands=1)

        fpsm = FPSMeter()
        smooth = [50] * 6
        send_period = 1.0 / max(1, args.send_hz)
        last_send = 0.0
        t0 = time.time()

        with HandLandmarker.create_from_options(options) as landmarker:
            log_event("mediapipe_ready")
            while True:
                ok, frame = cap.read()
                if not ok:
                    log_event("camera_frame_error", frame_index=frame_index + 1)
                    break

                frame_index += 1
                fpsm.tick()
                fps = fpsm.fps()

                rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                ts_ms = int((time.time() - t0) * 1000)

                pcts = None
                try:
                    result = landmarker.detect_for_video(mp_image, ts_ms)
                    if result and result.hand_landmarks:
                        lm = result.hand_landmarks[0]
                        # Convert landmark geometry into 0-100 % flexion values per finger.
                        pcts = flexion_percent(lm)
                        draw_hand_overlay(frame, lm)
                except Exception as exc:
                    log_event("mediapipe_error", error=str(exc))
                    print(f"[WARN] MediaPipe detection failed: {exc}")
                    pcts = None

                hand_present = pcts is not None
                if hand_present != hand_present_prev:
                    log_event("hand_presence", present=hand_present, frame=frame_index)
                    hand_present_prev = hand_present

                frame_vis = cv.flip(frame, 1) if MIRROR_PREVIEW else frame
                now = time.time()
                if pcts is not None:
                    if args.print_tx:
                        raw_vals = ", ".join(f"{v:.2f}" for v in pcts)
                        print(f"[RAW] {raw_vals}")
                    # Blend raw measurements to suppress jitter before converting to servo targets.
                    smooth = ema_vec(smooth, pcts, args.smooth)
                    if (now - last_send) >= send_period:
                        # Percentages must be integers (0-100) for the Arduino firmware.
                        vals = [int(round(v)) for v in smooth]
                        line = "P:{},{},{},{},{},{}\n".format(*vals)
                        log_payload = {
                            "frame": frame_index,
                            "raw_percentages": [round(float(v), 4) for v in pcts],
                            "smoothed_percentages": vals,
                            "elapsed_since_last": now - last_send,
                            "send_period": send_period,
                            "hand_present": True,
                            "fps": fps,
                            "ts_ms": ts_ms,
                        }
                        send_line(ser, line, args.print_tx, log_event, "serial_send", log_payload)
                        last_send = now

                cv.putText(frame_vis, f"FPS {fps:4.1f}  TX {args.send_hz:02d}Hz",
                           (10,24), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                if pcts is not None:
                    labels = ["T","I","M","R","P","S"]
                    txt = " ".join(f"{l}:{int(v):3d}%" for l, v in zip(labels, smooth))
                    cv.putText(frame_vis, txt, (10,50), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                cv.imshow("uHand Pose Controller (P:%)", frame_vis)
                key = cv.waitKey(1) & 0xFF
                if key == ESC_KEY:
                    log_event("live_exit", reason="esc_key", frame=frame_index)
                    break

    except KeyboardInterrupt:
        log_event("live_exit", reason="keyboard_interrupt")
        print()
        print("Stopping...")
    except Exception as exc:
        log_event("live_error", reason="runtime_exception", error=str(exc))
        print(f"Error in live mode: {exc}")
    finally:
        for sig, handler in original_handlers.items():
            signal.signal(sig, handler)
        if ser and ser.is_open:
            ser.close()
        if cap and cap.isOpened():
            cap.release()
        cv.destroyAllWindows()
        log_event("live_stop")
        if log_handle:
            log_handle.close()
# ----------------- Calibrate mode -------------------
HELP_TEXT = (
    "\nCalibration keys (targeting ONE finger at a time):\n"
    "  j / l : -1 / +1 degree\n"
    "  J / L : -5 / +5 degrees\n"
    "  m     : mark current as MIN for this finger\n"
    "  n     : mark current as MAX for this finger\n"
    "  space : next finger\n"
    "  b     : back (previous finger)\n"
    f"  r     : reset angle to {CALIB_START_DEG}\n"
    "  g     : send GET (print current limits from Arduino)\n"
    "  s     : SAVE all limits to EEPROM\n"
    "  q     : quit\n\n"
    "Tip: keep movements between comfortable mechanical limits (no buzzing).\n"
)

# Clamp helper used throughout calibration to keep servo angles safe.
def clamp(v, lo=CALIB_CLAMP_MIN, hi=CALIB_CLAMP_MAX): return max(lo, min(hi, v))

# Serial-only calibration loop; lets the user mark servo min/max ranges.
def run_calibrate(args):
    log_handle = None
    if args.log_json:
        try:
            log_handle = open(args.log_json, "a", encoding="utf-8")
        except OSError as exc:
            print(f"[WARN] could not open log file {args.log_json}: {exc}")
            log_handle = None

    def log_event(event, **data):
        if not args.log_json and log_handle is None:
            return
        entry = {"ts": time.time(), "event": event}
        entry.update(data)
        if log_handle:
            log_handle.write(json.dumps(entry) + "\n")
            log_handle.flush()
        if args.log_json:
            print(json.dumps(entry))

    # Serial only (no MediaPipe/camera needed)
    ser = None
    deg = [CALIB_START_DEG] * 6  # Current degrees we are commanding per finger
    mins = [None] * 6               # Collected MIN (open) angles per finger index
    maxs = [None] * 6               # Collected MAX (closed) angles per finger index
    finger = 0

    env_snapshot = build_env_snapshot(args.port)
    log_event("env_snapshot", **env_snapshot)

    shutdown_requested = False
    sent_neutral = False
    original_handlers = {}

    def handle_signal(signum, frame):
        nonlocal shutdown_requested, sent_neutral
        if shutdown_requested:
            return
        shutdown_requested = True
        log_event("shutdown", signal=signum)
        if ser and ser.is_open and not sent_neutral:
            send_line(ser, "NEUTRAL\n", args.print_tx, log_event, "serial_send", {"reason": "shutdown"})
            sent_neutral = True
        cv.destroyAllWindows()
        raise KeyboardInterrupt()

    for sig in (signal.SIGINT, signal.SIGTERM):
        original_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, handle_signal)

    try:
        log_event("calibrate_start", port=args.port, baud=args.baud)
        try:
            ser = serial.Serial(args.port, args.baud, timeout=SERIAL_CAL_TIMEOUT)
        except Exception as exc:
            log_event("calibrate_error", reason="serial_open", error=str(exc))
            print(f"ERROR opening serial {args.port}: {exc}")
            return

        log_event("serial_open", status="ok", port=args.port, baud=args.baud)

        print(HELP_TEXT)
        log_event("calibrate_help_shown")
        print(f"Calibrating finger 0 ({FINGER_NAMES[0]})")
        log_event("calibrate_finger_focus", finger=finger)

        # Small OpenCV window just to capture keys & show status
        img = np.zeros((240, 720, 3), dtype=np.uint8)

        def update_status():
            img[:] = 0
            shown_min = mins[finger]
            shown_max = maxs[finger]
            if shown_min is not None and shown_max is not None and shown_min > shown_max:
                shown_min, shown_max = shown_max, shown_min
            cv.putText(img, f"Calibrating: {FINGER_NAMES[finger]} (index {finger})",
                       (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv.putText(img, f"Angle={deg[finger]:3d}   MIN={str(shown_min)}   MAX={str(shown_max)}",
                       (10, 80), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv.putText(img, "Keys: j/J - / l/L + | m set MIN | n set MAX | space next | b back",
                       (10, 120), cv.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv.putText(img, "r reset | g GET | s SAVE | q quit",
                       (10, 150), cv.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv.imshow("uHand Calibrate", img)

        def send_degrees(event_name="calibrate_apply_degrees"):
            angles = [int(x) for x in deg]
            line = "{},{},{},{},{},{}\n".format(*angles)
            send_line(ser, line, args.print_tx, log_event, event_name,
                      {"finger": finger, "angles": angles})
            # Repeat sends (without spamming logs) so firmware EMA settles
            for _ in range(CAL_SETTLE_REPEATS):
                time.sleep(SERIAL_COMMAND_DELAY)
                send_line(ser, line, False)

        # move only the current finger, keep others at their last set deg
        send_degrees("calibrate_initial")
        update_status()

        try:
            while True:
                k = cv.waitKey(0) & 0xFF  # Mask to handle cross-platform key codes
                if k == 255:
                    continue

                if k == ord('q'):
                    log_event("calibrate_exit", reason="user_quit", finger=finger)
                    break
                elif k == ord('j'):
                    deg[finger] = clamp(deg[finger] - 1)
                    log_event("calibrate_adjust", finger=finger, key='j', angle=int(deg[finger]))
                    send_degrees()
                elif k == ord('l'):
                    deg[finger] = clamp(deg[finger] + 1)
                    log_event("calibrate_adjust", finger=finger, key='l', angle=int(deg[finger]))
                    send_degrees()
                elif k == ord('J'):
                    deg[finger] = clamp(deg[finger] - 5)
                    log_event("calibrate_adjust", finger=finger, key='J', angle=int(deg[finger]))
                    send_degrees()
                elif k == ord('L'):
                    deg[finger] = clamp(deg[finger] + 5)
                    log_event("calibrate_adjust", finger=finger, key='L', angle=int(deg[finger]))
                    send_degrees()
                elif k == ord('r'):
                    deg[finger] = CALIB_START_DEG
                    log_event("calibrate_reset", finger=finger, angle=int(deg[finger]))
                    send_degrees()
                elif k == ord('m'):
                    mins[finger] = int(deg[finger])
                    log_event("calibrate_mark_min", finger=finger, angle=mins[finger])
                    print(f"MIN set for {FINGER_NAMES[finger]} -> {mins[finger]}")
                    if maxs[finger] is not None and mins[finger] > maxs[finger]:
                        print("  (MIN currently above MAX; values will be sorted when saving)")
                elif k == ord('n'):
                    maxs[finger] = int(deg[finger])
                    log_event("calibrate_mark_max", finger=finger, angle=maxs[finger])
                    print(f"MAX set for {FINGER_NAMES[finger]} -> {maxs[finger]}")
                    if mins[finger] is not None and mins[finger] > maxs[finger]:
                        print("  (MAX currently below MIN; values will be sorted when saving)")
                elif k == ord('g'):
                    log_event("calibrate_get", finger=finger)
                    send_line(ser, "GET\n", args.print_tx, log_event, "calibrate_get_command",
                              {"finger": finger})
                elif k == ord('s'):
                    ranges = []
                    for i in range(6):
                        m = mins[i]
                        M = maxs[i]
                        if m is not None and M is not None:
                            lo, hi = sorted((m, M))
                            ranges.append({"finger": i, "lo": lo, "hi": hi})
                            # Persist the finger's calibrated range to firmware one finger at a time.
                            cmd = f"CAL {i} {lo} {hi}\n"
                            send_line(ser, cmd, args.print_tx, log_event, "calibrate_send_cal",
                                      {"finger": i, "lo": lo, "hi": hi})
                            time.sleep(SERIAL_COMMAND_DELAY)
                    send_line(ser, "SAVE\n", args.print_tx, log_event, "calibrate_save_command",
                              {"ranges": ranges})
                    log_event("calibrate_save", ranges=ranges)
                    print("Saved to EEPROM.")
                elif k == ord(' '):
                    finger = (finger + 1) % 6
                    log_event("calibrate_next_finger", finger=finger)
                    print(f"→ Next: {finger} ({FINGER_NAMES[finger]})")
                    send_degrees("calibrate_focus_finger")
                elif k == ord('b'):
                    finger = (finger - 1) % 6
                    log_event("calibrate_prev_finger", finger=finger)
                    print(f"→ Back: {finger} ({FINGER_NAMES[finger]})")
                    send_degrees("calibrate_focus_finger")

                update_status()
        except KeyboardInterrupt:
            log_event("calibrate_exit", reason="keyboard_interrupt", finger=finger)
            print()
            print("Stopping calibration...")
    finally:
        for sig, handler in original_handlers.items():
            signal.signal(sig, handler)
        for sig, handler in original_handlers.items():
            signal.signal(sig, handler)
        if ser and ser.is_open:
            ser.close()
        cv.destroyAllWindows()
        log_event("calibrate_stop", mins=mins, maxs=maxs)
        if log_handle:
            log_handle.close()

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
    p.add_argument("--log-json", metavar="PATH",
                   help="write diagnostic events as JSON lines to PATH")
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
