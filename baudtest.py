import serial, time
import json
import os

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.json")
try:
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        _config = json.load(f)
    PORT = _config["PORT"]  
except FileNotFoundError as exc:
    raise FileNotFoundError(
        f"Configuration file not found: {CONFIG_PATH}"
    ) from exc
except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
    raise RuntimeError("Invalid or incomplete serial configuration in config.json") from exc



BAUDS=[9600,19200,38400,57600,115200]
def pkt(a6):
    f=0x01; d=bytes(a6); ln=len(d); c=(~(f+ln+sum(d)))&0xFF
    return bytes([0xAA,0x77,f,ln])+d+bytes([c])

for b in BAUDS:
    try:
        with serial.Serial(PORT, b, timeout=0.2) as ser:
            time.sleep(0.2)
            ser.write(pkt([0,0,0,0,0,90])); time.sleep(0.25)
            ser.write(pkt([180,180,180,180,180,90])); time.sleep(0.25)
        print(f"Tried baud {b}. Did it move?")
    except Exception as e:
        print(f"Baud {b}: error {e}")
