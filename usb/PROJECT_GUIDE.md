# USB Workflow Guide

## Overview
This approach controls the uHand through the Arduino UNO's USB-B connector. A Python driver (`usb/pose2hand.py`) running on your computer captures hand pose via MediaPipe and streams percentage commands (`P:...`) over the virtual serial port exposed by the UNO. The Arduino firmware (`hand_driver_sketch`) interprets those percentages and moves the six servos.

**New in October 2025:**

**Session 2 (Oct 4):**
- **Calibration GET command fixed** - Now properly loads EEPROM values when you press 'g'
- **Performance optimizations** - Zero overhead when logging disabled
- **Code cleanup** - Removed debug spam, improved efficiency
- **Arduino config fixes** - Restored swivel defaults

**Session 1:**
- **3D joint-angle detection** - CRITICAL FIX: Now uses MediaPipe's full x,y,z coordinates instead of just x,y. Previous 2D implementation failed when fingers bent toward/away from camera.
- **PIP/DIP angle-based flexion** using 3D vectors for accurate detection regardless of hand orientation
- **Configurable angle thresholds** via CLI flags (`--finger-open`, `--finger-close`, `--thumb-open`, `--thumb-close`, `--dip-blend`)
- **Servo feedback logging** - Arduino reports actual servo positions back to Python for diagnostic analysis
- **Comprehensive log analyzer** (`analyze_log.py`) with automatic diagnostics and recommendations
- **Optimized Arduino loop** using char arrays instead of String for faster serial parsing

**⚠️ IMPORTANT DISCOVERY:**
The original joint-angle implementation used only 2D (x,y) coordinates. This meant fingers bending toward or away from the camera (changing only z-coordinate) appeared to have no movement. The fix uses full 3D vectors (x,y,z) for accurate angle calculation regardless of hand orientation.

**⚠️ KNOWN ISSUES:**
- Thumb and fingers may not close tightly - use `--calibrate` to lower minimum servo angles
- Swivel remains at neutral 50% (wrist rotation not implemented)

## Hardware Checklist
- Arduino UNO (or compatible) flashed with `hand_driver_sketch/hand_driver_sketch.ino`.
- USB-B cable between the UNO and your computer.
- uHand servos wired to the pins defined in the sketch (`[thumb,index,middle,ring,pinky,swivel] = D7..D2`).

## Software Prerequisites
- Python 3.12 (matches the scripts).
- Install dependencies from `requirements.txt`:
  ```bash
  pip install -r requirements.txt
  ```
- Place the MediaPipe model file (`hand_landmarker.task`) in the repository root (the driver looks for it via `MODEL_PATH`).

## Configuration
- Edit the constants near the top of `usb/pose2hand.py` if you need to tweak behaviour:
  - `PORT`, `BAUD`, `CAM_INDEX`, `SEND_HZ`, `SMOOTH_ALPHA`, `MIRROR_PREVIEW`, `PRINT_TX` for day-to-day usage.
  - **Joint-angle thresholds:** `FINGER_OPEN_DEG` (180°), `FINGER_CLOSE_DEG` (90°), `THUMB_OPEN_DEG` (165°), `THUMB_CLOSE_DEG` (95°), `DIP_BLEND` (0.35)
  - Serial responsiveness: `SERIAL_LIVE_TIMEOUT`, `SERIAL_CAL_TIMEOUT`, and `SERIAL_COMMAND_DELAY`.
  - Calibration bounds: `CALIB_CLAMP_MIN`, `CALIB_CLAMP_MAX`, `CALIB_START_DEG`.
  - MediaPipe sensitivity: `MP_MIN_HAND_DETECTION_CONF`, `MP_MIN_HAND_PRESENCE_CONF`, `MP_MIN_TRACKING_CONF`.
- **Recommended CLI flags:**
  ```bash
  python usb/pose2hand.py --log-json session.log \
    --finger-open 175 --finger-close 85 \
    --thumb-open 175 --thumb-close 70 \
    --dip-blend 0.35
  ```
- Run `python usb/pose2hand.py --help` for all available options.

## Running the Live Controller
```bash
python usb/pose2hand.py [--port SERIAL] [--cam INDEX] [--log-json session.json]
```
- `ESC` exits the preview window.
- `[RAW]` lines show raw finger percentages when `PRINT_TX` is enabled. Adjust the flexion thresholds if the values hover near 50 %.
- Command bursts appear as `P:a,b,c,d,e,f`; these are the smoothed percentages pushed to the UNO once every `1 / --send-hz` seconds.
- `--log-json` appends newline-delimited JSON that captures environment snapshots, model metadata, every `serial_send`, and graceful `shutdown` events, making it easy for humans (and future LLMs) to replay what happened.

### Calibration Mode
```bash
python usb/pose2hand.py --calibrate [--log-json cal.json]
```
- Use `j/J` and `l/L` for ±1°/±5° adjustments; only the selected finger moves while others hold their previous positions.
- Press `m` to tag the current angle as MIN (open) and `n` for MAX (closed). Values are sorted before being written back with `CAL`/`SAVE` commands.
- **Press `g` to load current EEPROM values** from Arduino (NEW: now properly reads and displays limits)
- Press `s` to save all calibrated limits to EEPROM
- **TIP:** If fingers/thumb don't close tightly enough, lower the minimum angle values during calibration
- `CAL_SETTLE_REPEATS` resends the most recent angles to overcome firmware EMA smoothing; pair it with `SERIAL_COMMAND_DELAY` if the servos lag.
- Ctrl+C (or any SIGINT/SIGTERM) is handled gracefully: the script logs `shutdown`, sends a single `NEUTRAL` packet, and restores the original handlers.

## Quick Serial Test
Use `usb/directusbtest.py` to send a scripted sequence of degree commands without MediaPipe:
```bash
python usb/directusbtest.py
```
If the servos move correctly here but not in live mode, revisit the MediaPipe thresholds or calibration constants.

## Logging & Analysis
Key events emitted when `--log-json` is active:
- `env_snapshot` – Python/OS/opencv/mediapipe/pyserial versions and the port in use.
- `live_start` – Session parameters including angle thresholds and DIP blend factor.
- `model_loaded` / `model_missing` – confirms the MediaPipe asset was found (or prints the curl command to fetch it).
- `serial_send` – every outbound packet with raw percentages, smoothed percentages, **joint angles** (thumb_ip, index_pip/dip, etc.), FPS, and timing.
- `servo_status` – periodic feedback from Arduino with actual servo positions and calibration limits (every 2 seconds when logging enabled).
- `hand_presence` – flips when a hand enters or leaves the frame.
- `shutdown`, `live_stop`, `calibrate_stop` – graceful teardown breadcrumbs after signals or exit.

### Analyzing Logs
Use the comprehensive analyzer to diagnose issues:
```bash
python usb/analyze_log.py session.log
```

This generates a full diagnostic report with:
- **Session info** - versions, thresholds, duration
- **Performance metrics** - FPS, send rate, detection rate
- **Joint angle analysis** - range and variance for each finger with warnings
- **Servo feedback** - actual servo positions vs calibrated ranges
- **Jitter analysis** - frame-to-frame changes and large jump detection
- **Error detection** - MediaPipe failures, serial errors, camera issues
- **Recommendations** - actionable suggestions for threshold tuning

**Other analyzer modes:**
```bash
# JSON statistics
python usb/analyze_log.py session.log --stats

# Analyze jitter for specific finger
python usb/analyze_log.py session.log --jitter index

# Plot angle over time
python usb/analyze_log.py session.log --angles thumb_ip

# Filter specific events
python usb/analyze_log.py session.log --events serial_send --keys smoothed_percentages angles
```

## Troubleshooting
- **No camera feed**: try another `--cam` index or disable `USE_AVFOUNDATION` for non-macOS setups.
- **"Missing model file"**: the driver logs `model_missing`, prints the curl command, and exits non-zero—download the `.task` file to the repo root.
- **UNO not responding**: verify the port, reflash `hand_driver_sketch`, and sanity-check with `directusbtest.py`.
- **Fingers not responding at all / stuck at one position**:
  - **FIRST:** Run analyzer and check if PIP/DIP angles are stuck at 180° (all straight)
  - This was caused by 2D angle calculation bug (now fixed with 3D vectors)
  - If angles still stuck at 180° after 3D fix, MediaPipe may not be detecting hand properly
  - Check MediaPipe overlay to verify landmarks are being detected
- **Jitter/twitching**:
  - Run `python usb/analyze_log.py session.log` to see jitter analysis
  - Check for very low angle values being filtered (< 10°)
  - Increase `--dip-blend` (e.g., 0.45-0.5) for more smoothing
  - Adjust `--smooth` (e.g., 0.5) for stronger EMA filtering
  - Increase Arduino `alpha` (currently 0.85) for faster response but may increase jitter
- **Fingers not fully opening/closing**:
  - Run analyzer to see actual angle ranges
  - Adjust `--finger-open`/`--finger-close` to match your hand's natural range
  - Example: if thumb only closes to 70°, use `--thumb-close 70`
  - Check servo feedback in analyzer - servos might be at calibration limits
- **Slow/sluggish response**:
  - Check servo calibration (run `--calibrate` mode)
  - Verify angle thresholds match your hand geometry
  - Increase Arduino `alpha` for faster servo response
  - Check actual send rate in analyzer (should be ~20 Hz)
- **Moving one finger causes others to move**:
  - This can indicate MediaPipe occlusion issues
  - Try spreading fingers apart more
  - Ensure good lighting so all fingers are clearly visible
  - Check servo calibration - overlapping ranges can cause coupling
- **Digits move backwards**: recalibrate ensuring MIN < MAX or flip the servo's entry in the Arduino firmware `invert[]` array.

## Related Files
- `usb/pose2hand.py` — live hand-tracking controller with joint-angle detection, structured logging, and graceful shutdown.
- `usb/analyze_log.py` — comprehensive log analyzer with diagnostics and recommendations.
- `usb/directusbtest.py` — simple degree-command exerciser.
- `hand_driver_sketch/hand_driver_sketch.ino` — Arduino firmware with STATUS feedback command and optimized serial parsing.

## Arduino Firmware Commands
The updated `hand_driver_sketch` supports:
- `P:0,25,50,75,100,50` — Move servos by percentage (0-100%)
- `90,100,95,100,90,90` — Move servos by absolute degrees
- `S:i,angle` — Move single servo i to specific angle (for calibration)
- `CAL i min max` — Set calibration limits for servo i
- `SAVE` — Save calibration to EEPROM
- `GET` — Print calibration limits
- `NEUTRAL` — Move all servos to midpoint
- `STATUS` — Return JSON with current servo positions and limits (used for logging)
