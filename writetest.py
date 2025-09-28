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

from math import sin
def pkt(a6):
    func=0x01; data=bytes(a6); ln=len(data)
    chk=(~(func+ln+sum(data)))&0xFF
    return bytes([0xAA,0x77,func,ln])+data+bytes([chk])
ser=serial.Serial(PORT, BAUD, timeout=0.2)
ser.write(pkt([73,10,0,57,90,180])); time.sleep(0.5)
ser.close()
print("Sent SET_ANGLES.")
