# uhand_protocol.py
HEADER = (0xAA, 0x77)
FUNC_SET_ANGLES = 0x01     # data len = 6 (angles 0..180)
FUNC_READ_ANGLE = 0x11     # data len = 0, reply has 6 bytes

def _chk(func, length, data_sum=0):
    return (~(func + length + data_sum)) & 0xFF

def pack_set_angles(a6):
    a = [max(0, min(180, int(x))) for x in a6]
    data = bytes(a)
    return bytes(HEADER) + bytes([FUNC_SET_ANGLES, 6]) + data + bytes([_chk(FUNC_SET_ANGLES, 6, sum(data))])

def pack_read_angles():
    return bytes(HEADER) + bytes([FUNC_READ_ANGLE, 0, _chk(FUNC_READ_ANGLE, 0)])

def parse_read_angles(resp: bytes):
    # Expect: AA 77 11 06 <6 bytes> <chk>
    if len(resp) < 11 or resp[0]!=0xAA or resp[1]!=0x77 or resp[2]!=FUNC_READ_ANGLE:
        raise ValueError("Bad response")
    n = resp[3]
    return list(resp[4:4+n])
