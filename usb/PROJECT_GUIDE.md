# USB Workflow Guide

## Overview
This approach controls the uHand through the Arduino UNO’s USB-B connector. A Python driver running on your computer captures hand pose via MediaPipe and streams percentage commands (`P:...`) over the virtual serial port exposed by the UNO. The Arduino firmware (`hand_driver_sketch`) interprets those percentages and moves the six servos.

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
- Place the MediaPipe model file (`hand_landmarker.task`) next to this guide (already tracked in the repo).

## Configuration
- Edit `usb/pose2hand_usb_driver.py` defaults if needed:
  - `PORT` → your UNO serial device (e.g. `/dev/tty.usbmodemXXXX` or `COM3`).
  - `CAM_INDEX` → camera index to use.
  - `MIRROR_PREVIEW`, `SEND_HZ`, `SMOOTH_ALPHA`, etc. for tuning.
- Optional CLI overrides are available (see `--help`).

## Running the Live Controller
```bash
python usb/pose2hand_usb_driver.py
```
- A preview window opens; use `ESC` to exit.
- Raw flex percentages print as `[RAW]` when `PRINT_TX` is enabled. Adjust `FLEXION_OPEN_THRESHOLD` / `FLEXION_CLOSE_THRESHOLD` if the range is too small.
- Serial packets show as `P:a,b,c,d,e,f`. The Arduino should move smoothly once calibrated.

### Calibration Mode
If the servo limits are off, run:
```bash
python usb/pose2hand_usb_driver.py --calibrate
```
Follow the on-screen instructions to set per-finger min/max values; the script sends `CAL` and `SAVE` commands to persist them in EEPROM.

## Quick Serial Test
`usb/directusbtest.py` sends a scripted sequence of degree commands:
```bash
python usb/directusbtest.py
```
Use this to confirm wiring and servo motion without MediaPipe.

## Troubleshooting
- **No camera feed**: try another `--cam` index or disable `USE_AVFOUNDATION` for non-macOS setups.
- **"Missing model file"**: ensure `hand_landmarker.task` is present (download from MediaPipe if missing).
- **UNO not responding**: verify the correct `PORT` and that `hand_driver_sketch` firmware is flashed. The sketch must be the percent-based one; test with `directusbtest.py`.
- **Jitter/twitching**: inspect `[RAW]` logs, tweak flexion thresholds, or increase `SMOOTH_ALPHA`. Re-run calibration to tighten servo limits.

## Related Files
- `usb/pose2hand_usb_driver.py` — live hand-tracking controller.
- `usb/directusbtest.py` — simple degree-command exerciser.
- `hand_driver_sketch/hand_driver_sketch.ino` — Arduino firmware expected by this workflow.
