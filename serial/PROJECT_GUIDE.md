# Serial-Pin Workflow Guide

## Overview
This workflow drives the uHand through a raw UART connection. Instead of plugging the Arduino into USB, you connect directly to the controller's serial pins (e.g. via an FTDI or USB-to-UART adapter) and exchange binary packets defined in `serial/uhand_protocol.py`. This is useful when embedding the hand in another system or when the USB-B port is unavailable.

## Hardware Checklist
- uHand controller flashed with firmware that speaks the `0xAA 0x77` packet protocol (`SET_ANGLES`, `READ_ANGLE`, `CAL`, etc.).
- USB-to-TTL serial adapter (3.3 V or 5 V levels matching the board).
- Wiring:
  - Adapter `TX` → controller `RX`.
  - Adapter `RX` → controller `TX`.
  - Common `GND` between adapter and controller.
  - Provide regulated power to the servos/controller as required (do **not** power servos solely from the USB adapter).

## Software Prerequisites
- Python 3.12 with `pyserial` installed (included in `requirements.txt`).
- Update `serial/config.json` with the USB-to-UART adapter's device path (e.g. `/dev/tty.usbserial-XXXX` or `COM5`).
- Ensure `hand_landmarker.task` is accessible if you plan to reuse the MediaPipe workflows from the USB approach.

## Packet Protocol Summary
- Packets start with header bytes `0xAA 0x77`.
- `SET_ANGLES (0x01)` expects six bytes (servo angles 0–180) followed by an 8-bit checksum.
- `READ_ANGLE (0x11)` requests the current angles; the controller should reply with `0xAA 0x77 0x11 len data chk`.
- Helper functions for packing/parsing live in `serial/uhand_protocol.py` and are used throughout the scripts here.

## Useful Scripts
- `serial/test_serial.py` — sends a `SET_ANGLES` packet, reads back a response, and prints parsed angles.
- `serial/writetest.py` — transmits a single `SET_ANGLES` packet for quick actuation tests.
- `serial/readtest.py` — issues `READ_ANGLE` and attempts to decode the reply, useful for confirming the firmware implementation.
- `serial/loopbacktest.py` — simple loopback sanity check; echoing bytes verifies the adapter wiring.
- `serial/baudtest.py` — experiment with baud-rate stability if you change the default 9600.

Run any script with:
```bash
python serial/test_serial.py
```
Each script loads connection settings from `serial/config.json`, so keep that file in sync with your hardware.

## Workflow Steps
1. Flash firmware that understands the binary protocol (e.g. the custom controller firmware bundled with the uHand board).
2. Wire the adapter to the controller (TX↔RX, GND, and power).
3. Edit `serial/config.json` for your adapter's port and desired baud rate.
4. Smoke test with `loopbacktest.py` (if the adapter supports hardware loopback) or by shorting TX/RX on the controller header.
5. Command the hand using `test_serial.py` or integrate `uhand_protocol.py` into your own application.

## Troubleshooting
- **No response / timeout**: verify wiring orientation (TX↔RX), power, and matching baud rates. Some firmware only replies after a successful `SET_ANGLES` command.
- **Checksum failures**: make sure nothing else is on the bus and that your ground reference is solid; noisy connections can flip bits.
- **Servos twitch but don't follow**: confirm the firmware actually maps the received angles to servo outputs. Use `writetest.py` with simple positions to rule out MediaPipe mapping issues.
- **Adapter not detected**: update `config.json` and check OS-level driver installation. On macOS, some adapters show up as `/dev/tty.usbserial-*` or `/dev/cu.usbserial-*`.

## Related Files
- `serial/uhand_protocol.py` — packet helpers shared by the scripts.
- `serial/config.json` — central place for port/baud configuration.
- `usb/PROJECT_GUIDE.md` — complementary guide for the USB-B workflow when you prefer to use the UNO's native USB port.
