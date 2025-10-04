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
import collections

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
SMOOTH_ALPHA = 0.6               # EMA smoothing factor for finger percentages default (0-1, higher=more smoothing)
MIRROR_PREVIEW = True             # Mirror preview only; math still uses original orientation
PRINT_TX = True                   # Emit outbound serial lines to stdout
EMA_ALPHA = 0.5                   # Default EMA alpha when --smooth not provided
DEADBAND  = 2.0                   # percent points
SLEW_MAX  = 8.0                   # max change per send (pp)
QUANTIZE  = 1.0                   # snap to nearest 1% to reduce chatter
# ---------------------------------------------------

# Constants
FLEXION_OPEN_THRESHOLD = -0.15    # Normalised TIP-MCP delta treated as fully open (0 %) [deprecated, kept for reference]
FLEXION_CLOSE_THRESHOLD = 0.30    # Normalised TIP-MCP delta treated as fully closed (100 %) [deprecated, kept for reference]

# Joint-angle based flexion thresholds (set by CLI args, defaults here)
FINGER_OPEN_DEG = 70.0           # Default open angle for fingers (PIP/DIP)
FINGER_CLOSE_DEG = 100.0           # Default closed angle for fingers (PIP/DIP)
THUMB_OPEN_DEG = 70.0            # Default open angle for thumb (IP)
THUMB_CLOSE_DEG = 100.0           # Default closed angle for thumb (IP)
DIP_BLEND = 0.30                  # Default DIP blend factor (0.35 = 35% DIP, 65% PIP) 

# Global variable for storing joint angles for logging
_last_joint_angles_for_log = {}

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
PIP_IDX = [3, 6, 10, 14, 18]  # thumb IP (3) used as PIP-equivalent for thumb
DIP_IDX = [3, 7, 11, 15, 19]  # thumb uses 3 again
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

def _angle_at(a, b, c):
    """Compute interior angle at b in degrees given three landmarks a-b-c (using 3D x,y,z)."""
    vx1, vy1, vz1 = a.x - b.x, a.y - b.y, a.z - b.z
    vx2, vy2, vz2 = c.x - b.x, c.y - b.y, c.z - b.z
    n1 = math.sqrt(vx1*vx1 + vy1*vy1 + vz1*vz1)
    n2 = math.sqrt(vx2*vx2 + vy2*vy2 + vz2*vz2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 180.0
    cosang = (vx1*vx2 + vy1*vy2 + vz1*vz2) / (n1*n2)
    cosang = max(-1.0, min(1.0, cosang))
    angle = math.degrees(math.acos(cosang))
    # Sanity check: filter only impossible angles < 10° (likely detection glitches)
    # Real finger flexion can reach 30-40° when fully closed
    if angle < 10.0:
        return 180.0  # Return open position as fallback for obvious glitches
    return angle

def _angle_to_pct(theta, open_deg, close_deg):
    """Map angle θ to 0..100% closed linearly."""
    if open_deg == close_deg:
        return 0.0
    lo, hi = (open_deg, close_deg) if open_deg < close_deg else (close_deg, open_deg)
    t = (theta - lo) / (hi - lo)
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    # If open<close, 0% at open, 100% at close; else invert
    return t*100.0 if open_deg < close_deg else (1.0 - t)*100.0


def flexion_percent(landmarks, enable_logging=False):
    """
    Returns [thumb,index,middle,ring,pinky,swivel] in 0..100 (% closed)
    Uses joint angles (thumb IP; others PIP with DIP blend) for stability.
    """
    global _last_joint_angles_for_log

    out = []
    angles_log = {} if enable_logging else None

    # Thumb (IP angle at index 3 formed by (2-3-4))
    try:
        th_ip = _angle_at(landmarks[2], landmarks[3], landmarks[4])
        if not math.isfinite(th_ip):
            th_ip = 180.0
        th_pct = _angle_to_pct(th_ip, THUMB_OPEN_DEG, THUMB_CLOSE_DEG)
        out.append(th_pct)
        if angles_log is not None:
            angles_log["thumb_ip"] = round(float(th_ip), 1)
    except Exception:
        out.append(50.0)  # fallback
        if angles_log is not None:
            angles_log["thumb_ip"] = None

    # Index..Pinky
    specs = [
        (5,6,7,8),     # index
        (9,10,11,12),  # middle
        (13,14,15,16), # ring
        (17,18,19,20), # pinky
    ]
    finger_names_short = ["index", "middle", "ring", "pinky"]
    for idx, (mcp_i, pip_i, dip_i, tip_i) in enumerate(specs):
        try:
            pip_theta = _angle_at(landmarks[mcp_i], landmarks[pip_i], landmarks[dip_i])
            dip_theta = _angle_at(landmarks[pip_i], landmarks[dip_i], landmarks[tip_i])
            if not math.isfinite(pip_theta):
                pip_theta = 180.0
            if not math.isfinite(dip_theta):
                dip_theta = 180.0
            pip_pct   = _angle_to_pct(pip_theta, FINGER_OPEN_DEG, FINGER_CLOSE_DEG)
            dip_pct   = _angle_to_pct(dip_theta, FINGER_OPEN_DEG, FINGER_CLOSE_DEG)
            pct = (1.0 - DIP_BLEND) * pip_pct + DIP_BLEND * dip_pct
            out.append(pct)
            if angles_log is not None:
                angles_log[f"{finger_names_short[idx]}_pip"] = round(float(pip_theta), 1)
                angles_log[f"{finger_names_short[idx]}_dip"] = round(float(dip_theta), 1)
        except Exception:
            out.append(50.0)  # fallback
            if angles_log is not None:
                angles_log[f"{finger_names_short[idx]}_pip"] = None
                angles_log[f"{finger_names_short[idx]}_dip"] = None

    # Swivel remains neutral for now
    out.append(SWIVEL_NEUTRAL_PERCENT)

    # Store angle diagnostics for logging (only if logging enabled)
    if angles_log is not None:
        _last_joint_angles_for_log = angles_log

    return out

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

def read_arduino_status(ser):
    """Non-blocking read of Arduino STATUS response. Returns servo angles dict or None."""
    try:
        # Check if data is available without blocking
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('{') and 'servos' in line:
                try:
                    data = json.loads(line)
                    return {
                        'servo_angles': data.get('servos', []),
                        'servo_limits': data.get('limits', {})
                    }
                except json.JSONDecodeError:
                    pass
        return None
    except Exception:
        return None

def request_arduino_status(ser):
    """Non-blocking request for STATUS from Arduino."""
    try:
        ser.write(b"STATUS\n")
    except Exception:
        pass

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

    # Optimize logging: use no-op function when logging is disabled
    if args.log_json or log_handle:
        def log_event(event, **data):
            entry = {"ts": time.time(), "event": event}
            entry.update(data)
            if log_handle:
                log_handle.write(json.dumps(entry) + "\n")
                log_handle.flush()
            if args.log_json:
                print(json.dumps(entry))
    else:
        def log_event(event, **data):
            pass  # No-op when logging disabled

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

        smooth_alpha = min(max(args.smooth, 0.0), 1.0)
        if smooth_alpha != args.smooth:
            print(f"[WARN] --smooth value {args.smooth} clamped to {smooth_alpha} (expected 0.0-1.0 range)")

        log_event("live_start", port=args.port, baud=args.baud, cam=args.cam,
                  send_hz=args.send_hz, smooth=args.smooth, mirror=MIRROR_PREVIEW,
                  finger_open_deg=FINGER_OPEN_DEG, finger_close_deg=FINGER_CLOSE_DEG,
                  thumb_open_deg=THUMB_OPEN_DEG, thumb_close_deg=THUMB_CLOSE_DEG,
                  dip_blend=DIP_BLEND)

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
        last_status_query = 0.0
        status_query_period = 2.0  # Query servo status every 2 seconds (only if logging enabled)
        status_logging_enabled = bool(args.log_json or log_handle)
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
                        pcts = flexion_percent(lm, enable_logging=status_logging_enabled)
                        draw_hand_overlay(frame, lm)
                except Exception as exc:
                    log_event("mediapipe_error", error=str(exc))
                    print(f"[WARN] MediaPipe detection failed: {exc}")
                    pcts = None

                hand_present = pcts is not None
                prev_present = bool(hand_present_prev)
                if prev_present and not hand_present:
                    if ser and ser.is_open:
                        send_line(ser, "NEUTRAL\n", args.print_tx, log_event, "serial_send",
                                  {"reason": "hand_lost", "frame": frame_index})
                    smooth = [50] * 6
                elif not prev_present and hand_present:
                    smooth = [50] * 6

                if hand_present != hand_present_prev:
                    log_event("hand_presence", present=hand_present, frame=frame_index)
                    hand_present_prev = hand_present

                frame_vis = cv.flip(frame, 1) if MIRROR_PREVIEW else frame
                now = time.time()
                if pcts is not None:
                    # Blend raw measurements to suppress jitter before converting to servo targets.
                    smooth = ema_vec(smooth, pcts, args.smooth)
                    if (now - last_send) >= send_period:
                        # Percentages must be integers (0-100) for the Arduino firmware.
                        vals = [int(round(v)) for v in smooth]
                        line = "P:{},{},{},{},{},{}\n".format(*vals)

                        # Only build log payload if logging is enabled (avoid dict construction overhead)
                        if status_logging_enabled:
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
                            # Add joint angle diagnostics if available
                            if _last_joint_angles_for_log:
                                log_payload["angles"] = _last_joint_angles_for_log
                        else:
                            log_payload = None

                        send_line(ser, line, args.print_tx, log_event, "serial_send", log_payload)
                        last_send = now

                # Periodically query servo status from Arduino (only if logging enabled)
                if status_logging_enabled and (now - last_status_query) >= status_query_period:
                    request_arduino_status(ser)
                    last_status_query = now

                # Check for and log any pending STATUS responses (non-blocking)
                if status_logging_enabled:
                    status = read_arduino_status(ser)
                    if status:
                        log_event("servo_status", **status)

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
            # Use single-servo command to avoid jitter on inactive fingers
            line = "S:{},{}\n".format(finger, int(deg[finger]))
            send_line(ser, line, args.print_tx, log_event, event_name,
                      {"finger": finger, "angle": int(deg[finger])})

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
                    # Read the response from Arduino
                    time.sleep(0.2)  # Give Arduino time to respond
                    try:
                        # Read "LIMITS thumb,index,middle,ring,pinky,swivel:"
                        ser.readline()
                        # Read "MIN: x,x,x,x,x,x"
                        min_line = ser.readline().decode('utf-8', errors='ignore').strip()
                        # Read "MAX: x,x,x,x,x,x"
                        max_line = ser.readline().decode('utf-8', errors='ignore').strip()

                        if min_line.startswith("MIN:"):
                            min_vals = [int(v.strip()) for v in min_line.split(":")[1].split(",")]
                            for i in range(6):
                                mins[i] = min_vals[i]
                        if max_line.startswith("MAX:"):
                            max_vals = [int(v.strip()) for v in max_line.split(":")[1].split(",")]
                            for i in range(6):
                                maxs[i] = max_vals[i]
                        print(f"Loaded from EEPROM - MIN: {mins}, MAX: {maxs}")
                        log_event("calibrate_get_response", mins=mins, maxs=maxs)
                    except Exception as e:
                        print(f"Failed to read EEPROM values: {e}")
                        log_event("calibrate_get_error", error=str(e))
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
    global FINGER_OPEN_DEG, FINGER_CLOSE_DEG, THUMB_OPEN_DEG, THUMB_CLOSE_DEG, DIP_BLEND

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

    # Joint-angle threshold flags
    p.add_argument("--finger-open", type=float, default=180.0,
                   help="open angle for fingers in degrees (default: 180.0)")
    p.add_argument("--finger-close", type=float, default=90.0,
                   help="closed angle for fingers in degrees (default: 90.0)")
    p.add_argument("--thumb-open", type=float, default=165.0,
                   help="open angle for thumb in degrees (default: 165.0)")
    p.add_argument("--thumb-close", type=float, default=95.0,
                   help="closed angle for thumb in degrees (default: 95.0)")
    p.add_argument("--dip-blend", type=float, default=0.35,
                   help="DIP blend factor, 0-1 (default: 0.35)")

    args = p.parse_args()

    # Store threshold values in module-level constants
    FINGER_OPEN_DEG  = args.finger_open
    FINGER_CLOSE_DEG = args.finger_close
    THUMB_OPEN_DEG   = args.thumb_open
    THUMB_CLOSE_DEG  = args.thumb_close
    DIP_BLEND        = args.dip_blend

    if args.calibrate:
        run_calibrate(args)
    else:
        run_live(args)

if __name__ == "__main__":
    main()
# EOF
