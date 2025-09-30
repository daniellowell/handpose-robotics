import serial, time
import json
import os

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



ser = serial.Serial(PORT, BAUD, timeout=1)

test_str = "Hello uHand test!\n"
ser.write(test_str.encode("utf-8"))
time.sleep(0.1)
data = ser.read(ser.in_waiting or 1).decode("utf-8", errors="ignore")

print("Sent:", repr(test_str))
print("Received:", repr(data))

ser.close()