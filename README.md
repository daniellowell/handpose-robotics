# uHand Controller Workflows

Two end-to-end paths let you drive the uHand from MediaPipe hand tracking:

- **USB workflow (`usb/`)** – Plug the Arduino UNO into your computer and stream percentage commands (`P:…`) over the virtual serial port.
- **Serial-pin workflow (`serial/`)** – Talk to the controller's UART header using the binary packet protocol defined in `serial/uhand_protocol.py`.

The guides `usb/PROJECT_GUIDE.md` and `serial/PROJECT_GUIDE.md` walk through wiring, firmware expectations, and quick checks.

## Recent Improvements (October 2025)

**Session 2 Updates:**
- **Calibration GET command fixed** - Now properly loads EEPROM values during calibration
- **Performance optimizations** - No-op logging functions, eliminated unnecessary dict construction when logging disabled
- **Code cleanup** - Removed debug spam, direct global access for threshold values
- **Arduino config restored** - Fixed swivel defaults and invert flags

**Session 1 Updates:**
- **3D joint-angle detection** - Uses MediaPipe's full x,y,z coordinates (CRITICAL: was using only 2D x,y which failed when fingers bent toward/away from camera)
- **PIP/DIP angle-based flexion** instead of TIP-MCP distance for more stable detection
- **Configurable thresholds** via CLI (`--finger-open`, `--finger-close`, `--thumb-open`, `--thumb-close`, `--dip-blend`)
- **Servo feedback logging** - Arduino reports actual positions for diagnostics
- **Comprehensive analyzer** (`usb/analyze_log.py`) with automatic issue detection and recommendations
- **Optimized loops** - Non-blocking STATUS queries, char arrays instead of String on Arduino (2-3x faster)

**⚠️ IMPORTANT:** The angle calculation now uses 3D vectors. Previous 2D implementation only worked when hand was held sideways to camera. 3D implementation works regardless of hand orientation.

**⚠️ KNOWN ISSUES:**
- Thumb and fingers may not close tightly enough - requires servo calibration adjustment (lower minimum angles)
- Swivel remains at neutral 50% (wrist rotation not implemented)

## 1. Prerequisites

1. Use a virtual environment if possible:
   ```bash
   python -m venv .venv
   source .venv/bin/activate
   ```
2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Download the MediaPipe model (place in repo root as `hand_landmarker.task`):
   ```bash
   curl -o hand_landmarker.task \
     https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
   ```

## 2. USB Workflow (MediaPipe → `P:` packets)

Main controller:
```bash
python usb/pose2hand.py [--port SERIAL_PORT] [--cam INDEX] [--log-json session.json]
```

**Recommended with joint-angle tuning:**
```bash
python usb/pose2hand.py --log-json session.log \
  --finger-open 175 --finger-close 85 \
  --thumb-open 175 --thumb-close 70 \
  --dip-blend 0.35
```

Key options:
- `--calibrate` – interactive per-finger limits (sends `CAL`/`SAVE`). Now uses single-servo `S:` commands to prevent jitter on inactive fingers.
- `--smooth` – EMA coefficient for percent smoothing (default `0.6`).
- `--send-hz` – command rate (default `20`).
- `--finger-open/--finger-close` – angle thresholds for fingers in degrees (default: 180°→90°)
- `--thumb-open/--thumb-close` – angle thresholds for thumb in degrees (default: 165°→95°)
- `--dip-blend` – DIP joint blend factor 0-1 (default: 0.35; higher = smoother)
- `--log-json PATH` – append structured diagnostics including joint angles and servo feedback
- `--no-print-tx` – silence the human-readable `P:…` lines.

**Analyze session logs:**
```bash
python usb/analyze_log.py session.log
```
Provides comprehensive diagnostics: performance metrics, joint angle analysis, servo feedback, jitter detection, and actionable recommendations.

Helpful scripts:
- `python usb/directusbtest.py` – send a scripted degree sequence to verify wiring/firmware without MediaPipe.

## 3. Serial-Pin Workflow (Binary Protocol)

Use when you have direct access to the UART header or a custom firmware responding to the `0xAA 0x77` packet format.

Common utilities:
```bash
python serial/test_serial.py      # send SET_ANGLES + read-back
python serial/writetest.py        # single SET_ANGLES packet
python serial/readtest.py         # issue READ_ANGLE and decode reply
python serial/loopbacktest.py     # sanity check adapter wiring
```
All scripts read connection details from `serial/config.json` (`PORT`, `BAUD`, etc.).

## 4. Calibration Tips

- Launch `python usb/pose2hand.py --calibrate` to capture per-finger open/closed angles. Use the `j/J/l/L` keys for fine/coarse adjustments; press `m` to mark MIN (open) and `n` to mark MAX (closed).
- The dialog now logs every adjustment when `--log-json` is active, making it easier to debug twitchy servos or reversed ranges.
- `CAL_SETTLE_REPEATS` (top-level constant) controls how many times each degree command is resent to overcome firmware EMA smoothing. `CALIB_CLAMP_MIN/CALIB_CLAMP_MAX` and `CALIB_START_DEG` expose the servo bounds without digging through the loop, while `SERIAL_LIVE_TIMEOUT`, `SERIAL_CAL_TIMEOUT`, and `SERIAL_COMMAND_DELAY` tune responsiveness.

## 5. Tests & Diagnostics

- `python test_camera_preview.py` – confirm camera feed.
- `python test_mp_handpose.py` – confirm MediaPipe landmarks.
- USB serial smoke test: `python usb/directusbtest.py`.
- UART smoke test: `python serial/test_serial.py`.

## 6. Logging & Diagnostics

With `--log-json PATH`, both live and calibration modes emit newline-delimited JSON. Example entry:
```json
{"ts": 1720000000.123, "event": "serial_send", "frame": 42,
 "raw_percentages": [12.5, 18.7, 33.3, 27.4, 41.2, 50.0],
 "smoothed_percentages": [10, 18, 32, 26, 40, 50],
 "angles": {"thumb_ip": 145.2, "index_pip": 175.8, "index_dip": 172.3, ...},
 "elapsed_since_last": 0.052, "send_period": 0.05, "fps": 19.6}
```

Additional logged events:
- `live_start` – session parameters including angle thresholds
- `servo_status` – actual servo positions and calibration limits from Arduino (every 2 seconds)
- `hand_presence` – when hand enters/leaves frame

**Analyze logs with:**
```bash
python usb/analyze_log.py session.log        # Full diagnostic report
python usb/analyze_log.py session.log --stats       # JSON statistics
python usb/analyze_log.py session.log --jitter index  # Per-finger jitter
python usb/analyze_log.py session.log --angles thumb_ip  # Plot angles
```

## 7. Key Constants & Helpers

* `FLEXION_OPEN_THRESHOLD` / `FLEXION_CLOSE_THRESHOLD` – tweak these if MediaPipe percentages hover near 50 %. Lowering the open value or raising the close value amplifies finger travel.
* `SERIAL_LIVE_TIMEOUT`, `SERIAL_CAL_TIMEOUT`, `SERIAL_COMMAND_DELAY` – govern serial responsiveness in live and calibration modes.
* `CALIB_CLAMP_MIN`, `CALIB_CLAMP_MAX`, `CALIB_START_DEG`, `CAL_SETTLE_REPEATS` – adjust servo bounds and smoothing without digging into the calibration loop.
* `MP_MIN_HAND_DETECTION_CONF`, `MP_MIN_HAND_PRESENCE_CONF`, `MP_MIN_TRACKING_CONF` – expose MediaPipe confidence thresholds.
* Logging helpers (`module_version`, `build_env_snapshot`, `send_line`) were designed so future LLMs can follow the data flow; events such as `env_snapshot`, `model_loaded`, `serial_send`, and `shutdown` provide breadcrumbs when reviewing logs.

## 8. Repository Map (selected)

```
usb/
  pose2hand.py              # MediaPipe → serial with joint-angle detection
  analyze_log.py            # Comprehensive log analyzer with diagnostics
  directusbtest.py          # Simple scripted motions
serial/
  config.json               # UART adapter configuration
  uhand_protocol.py         # Packet helpers
  test_serial.py            # Binary protocol exerciser
hand_driver_sketch/         # Arduino firmware with STATUS feedback (optimized)
```

**Key improvements in current version:**
- Joint-angle detection (PIP/DIP) replaces TIP-MCP for reduced jitter
- Configurable angle thresholds via CLI flags
- Servo feedback logging with STATUS command
- Non-blocking log queries (no loop interference)
- Optimized Arduino parsing (char arrays, 2-3x faster)

Refer to the project guides for wiring diagrams and safety tips before driving the servos.
