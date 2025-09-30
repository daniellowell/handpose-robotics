import serial, time
PORT = "/dev/tty.usbmodem1101"  # <-- update
ser = serial.Serial(PORT, 9600, timeout=1)

def send(a,b,c,d,e,f):
    ser.write(f"{a},{b},{c},{d},{e},{f}\n".encode()); time.sleep(0.4)

#  'a,b,c,d,e,f' (0..180). Order: thumb,index,middle,ring,pinky,swivel

# Neutral
send(90,90,90,90,90,90)
# open pinky a bit

for x in range(4):


    send(80,80,80,85,120,90)
    send(80,80,80,85,90,90)
    send(80,80,80,85,60,90)
    # Open a bit
    send(80,80,80,120,80,90)
    send(80,80,80,90,80,90)
    send(80,80,80,60,80,90)
    # Open a bit
    send(80,80,120,85,80,90)
    send(80,80,90,85,80,90)
    send(80,80,16,85,80,90)
    # Open a bit
    send(80,120,80,85,80,90)
    send(80,90,80,85,80,90)
    send(80,60,80,85,80,90)
    # Open a bit
    send(120,80,80,85,80,90)
    send(90,80,80,85,80,90)
    send(60,80,80,85,80,90)

# Close a bit
send(100,100,100,95,100,90)
# Back to neutral
send(90,90,90,90,90,90)

ser.close()
