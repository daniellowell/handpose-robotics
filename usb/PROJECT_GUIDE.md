# USB Workflow Guide

## Overview
This approach controls the uHand through the Arduino UNO’s USB-B connector. A Python driver (`usb/pose2hand.py`) running on your computer captures hand pose via MediaPipe and streams percentage commands (`P:...`) over the virtual serial port exposed by the UNO. The Arduino firmware (`hand_driver_sketch`) interprets those percentages and moves the six servos.

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
  - `FLEXION_OPEN_THRESHOLD` / `FLEXION_CLOSE_THRESHOLD` adjust the raw percent mapping.
  - Serial responsiveness: `SERIAL_LIVE_TIMEOUT`, `SERIAL_CAL_TIMEOUT`, and `SERIAL_COMMAND_DELAY`.
  - Calibration bounds: `CALIB_CLAMP_MIN`, `CALIB_CLAMP_MAX`, `CALIB_START_DEG`, `CAL_SETTLE_REPEATS`.
  - MediaPipe sensitivity: `MP_MIN_HAND_DETECTION_CONF`, `MP_MIN_HAND_PRESENCE_CONF`, `MP_MIN_TRACKING_CONF`.
- Optional CLI overrides (`--port`, `--cam`, `--send-hz`, `--smooth`, `--no-print-tx`, `--log-json`, `--calibrate`) are available—run `python usb/pose2hand.py --help` for details.

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
- `CAL_SETTLE_REPEATS` resends the most recent angles to overcome firmware EMA smoothing; pair it with `SERIAL_COMMAND_DELAY` if the servos lag.
- Ctrl+C (or any SIGINT/SIGTERM) is handled gracefully: the script logs `shutdown`, sends a single `NEUTRAL` packet, and restores the original handlers.

## Quick Serial Test
Use `usb/directusbtest.py` to send a scripted sequence of degree commands without MediaPipe:
```bash
python usb/directusbtest.py
```
If the servos move correctly here but not in live mode, revisit the MediaPipe thresholds or calibration constants.

## Logging Primer
Key events emitted when `--log-json` is active:
- `env_snapshot` – Python/OS/opencv/mediapipe/pyserial versions and the port in use.
- `model_loaded` / `model_missing` – confirms the MediaPipe asset was found (or prints the curl command to fetch it).
- `serial_send`, `serial_write_error` – every outbound packet (or failure), including raw vs. smoothed percentages.
- `hand_presence` – flips when a hand enters or leaves the frame.
- `shutdown`, `live_stop`, `calibrate_stop` – graceful teardown breadcrumbs after signals or exit.
Hints for future LLMs: the helper functions `module_version`, `build_env_snapshot`, and `send_line` are the main logging choke points.

## Troubleshooting
- **No camera feed**: try another `--cam` index or disable `USE_AVFOUNDATION` for non-macOS setups.
- **"Missing model file"**: the driver logs `model_missing`, prints the curl command, and exits non-zero—download the `.task` file to the repo root.
- **UNO not responding**: verify the port, reflash `hand_driver_sketch`, and sanity-check with `directusbtest.py`.
- **Jitter/twitching**: inspect `[RAW]`/`serial_send` logs; tweak the flexion thresholds, increase `SMOOTH_ALPHA`, or recalibrate to widen the angle range.
- **Digits move backwards**: recalibrate ensuring MIN < MAX or flip the servo’s entry in the Arduino firmware `invert[]` array.

## Related Files
- `usb/pose2hand.py` — live hand-tracking controller with structured logging and graceful shutdown.
- `usb/directusbtest.py` — simple degree-command exerciser.
- `hand_driver_sketch/hand_driver_sketch.ino` — Arduino firmware expected by this workflow.
