# uHand Controller Workflows

Two end-to-end paths let you drive the uHand from MediaPipe hand tracking:

- **USB workflow (`usb/`)** – Plug the Arduino UNO into your computer and stream percentage commands (`P:…`) over the virtual serial port.
- **Serial-pin workflow (`serial/`)** – Talk to the controller’s UART header using the binary packet protocol defined in `serial/uhand_protocol.py`.

The guides `usb/PROJECT_GUIDE.md` and `serial/PROJECT_GUIDE.md` walk through wiring, firmware expectations, and quick checks.

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

Key options:
- `--calibrate` – interactive per-finger limits (sends `CAL`/`SAVE`).
- `--smooth` – EMA coefficient for percent smoothing (default `0.6`).
- `--send-hz` – command rate (default `20`).
- `--log-json PATH` – append structured diagnostics to `PATH` and echo JSON to stdout. Expect events such as `env_snapshot`, `model_loaded`, `serial_send`, `hand_presence`, and graceful `shutdown` notifications.
- `--no-print-tx` – silence the human-readable `P:…` lines.

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

## 6. Logging

With `--log-json PATH`, both live and calibration modes emit newline-delimited JSON. Example entry:
```json
{"ts": 1720000000.123, "event": "serial_send", "frame": 42,
 "raw_percentages": [12.5, 18.7, 33.3, 27.4, 41.2, 50.0],
 "smoothed_percentages": [10, 18, 32, 26, 40, 50],
 "elapsed_since_last": 0.052, "send_period": 0.05, "fps": 19.6}
```
This is invaluable when diagnosing weak actuation, reversed motion, or dropouts.

## 7. Key Constants & Helpers

* `FLEXION_OPEN_THRESHOLD` / `FLEXION_CLOSE_THRESHOLD` – tweak these if MediaPipe percentages hover near 50 %. Lowering the open value or raising the close value amplifies finger travel.
* `SERIAL_LIVE_TIMEOUT`, `SERIAL_CAL_TIMEOUT`, `SERIAL_COMMAND_DELAY` – govern serial responsiveness in live and calibration modes.
* `CALIB_CLAMP_MIN`, `CALIB_CLAMP_MAX`, `CALIB_START_DEG`, `CAL_SETTLE_REPEATS` – adjust servo bounds and smoothing without digging into the calibration loop.
* `MP_MIN_HAND_DETECTION_CONF`, `MP_MIN_HAND_PRESENCE_CONF`, `MP_MIN_TRACKING_CONF` – expose MediaPipe confidence thresholds.
* Logging helpers (`module_version`, `build_env_snapshot`, `send_line`) were designed so future LLMs can follow the data flow; events such as `env_snapshot`, `model_loaded`, `serial_send`, and `shutdown` provide breadcrumbs when reviewing logs.

## 8. Repository Map (selected)

```
usb/
  pose2hand.py              # MediaPipe → serial controller (env snapshots, shutdown logging)
  directusbtest.py          # simple scripted motions
serial/
  config.json               # UART adapter configuration
  uhand_protocol.py         # packet helpers
  test_serial.py            # binary protocol exerciser
hand_driver_sketch/         # Arduino firmware (expects percent packets)
```

Refer to the project guides for wiring diagrams and safety tips before driving the servos.
