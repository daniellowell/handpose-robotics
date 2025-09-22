# test_serial.py
import json
import os
import time, serial
from uhand_protocol import pack_set_angles, pack_read_angles, parse_read_angles

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.json")
try:
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        _config = json.load(f)
    PORT = _config["PORT"]  
    BAUD = int(_config.get("BAUD", 9600))
except FileNotFoundError as exc:
    raise FileNotFoundError(
        f"Configuration file not found: {CONFIG_PATH}"
    ) from exc
except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
    raise RuntimeError("Invalid or incomplete serial configuration in config.json") from exc



with serial.Serial(PORT, BAUD, timeout=0.5) as ser:
    # Move: [thumb, index, middle, ring, pinky, pan-tilt]
    ser.write(pack_set_angles([73, 10, 0, 57, 90, 180]))
    time.sleep(1.0)

    ser.write(pack_read_angles())
    resp = ser.read(64)
    try:
        angles = parse_read_angles(resp)
        print("uHand angles:", angles)
    except Exception as e:
        print("Failed to parse response:", e)
    print("Raw response:", resp.hex())
    time.sleep(0.5)
    ser.write(pack_set_angles([0, 0, 0, 0, 0, 180]))
    time.sleep(1.0)
    ser.write(pack_read_angles())
    resp = ser.read(64)
    try:
        angles = parse_read_angles(resp)
        print("uHand angles:", angles)
    except Exception as e:
        print("Failed to parse response:", e)
    print("Raw response:", resp.hex())
    time.sleep(0.5)
    ser.write(pack_set_angles([90, 90, 90, 90, 90, 180]))
    time.sleep(1.0)
    ser.write(pack_read_angles())
    resp = ser.read(64)
    try:
        angles = parse_read_angles(resp)
        print("uHand angles:", angles)
    except Exception as e:
        print("Failed to parse response:", e)
    print("Raw response:", resp.hex())
    time.sleep(0.5)
    ser.write(pack_set_angles([0, 0, 0, 0, 0, 180]))  # back to rest
    
