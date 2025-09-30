# test_read_angles.py
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


def pack_read_angles():
    func=0x11; ln=0
    chk=(~(func+ln)) & 0xFF
    return bytes([0xAA,0x77,func,ln,chk])

with serial.Serial(PORT, BAUD, timeout=0.5) as ser:
    # Clear any stale bytes
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)

    ser.write(pack_read_angles())

    # Read up to 2 seconds, resync on header 0xAA 0x77
    deadline = time.time()+2.0
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            # Try to find header
            i = buf.find(b'\xAA\x77')
            if i != -1 and len(buf) >= i+5:
                func = buf[i+2]
                ln   = buf[i+3]
                need = i+4+ln+1
                if len(buf) >= need:
                    data = bytes(buf[i+4:i+4+ln])
                    chk  = buf[need-1]
                    ok = ((~(func+ln+sum(data))) & 0xFF) == chk
                    print("FUNC=0x%02X len=%d data=%s chk=0x%02X ok=%s"
                          % (func, ln, list(data), chk, ok))
                    break
    else:
        print("No response to READ_ANGLE within timeout. (This is normal on some firmware.)")