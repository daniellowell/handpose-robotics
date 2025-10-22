#!/usr/bin/env python3
"""
Interactive servo tester - control each finger individually with keyboard.
"""

import serial
import time
import sys

PORT = "/dev/tty.usbmodem1101"
BAUD = 9600

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Swivel"]

print("="*70)
print("           INDIVIDUAL SERVO TESTER")
print("="*70)
print()
print("Test each finger servo independently to verify they work.")
print()

try:
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    time.sleep(2.0)

    # Read startup
    print("Arduino connection established:")
    while ser.in_waiting:
        print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")

    # Get calibration
    print("\nCurrent calibration:")
    ser.reset_input_buffer()
    ser.write(b"GET\n")
    time.sleep(0.3)
    for _ in range(3):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"  {line}")

    print("\n" + "="*70)
    print("CONTROLS:")
    print("="*70)
    print("  0-5  : Select finger (0=Thumb, 1=Index, 2=Middle, 3=Ring, 4=Pinky, 5=Swivel)")
    print("  o    : Move selected finger to OPEN (0%)")
    print("  c    : Move selected finger to CLOSED (100%)")
    print("  m    : Move selected finger to MIDDLE (50%)")
    print("  a    : Move ALL fingers to selected position")
    print("  +/-  : Increase/decrease position by 10%")
    print("  n    : NEUTRAL - all servos to 50%")
    print("  q    : Quit")
    print("="*70)
    print()

    selected_finger = 1  # Start with index
    current_positions = [50, 50, 50, 50, 50, 50]  # Track current % for each servo

    def send_command(positions):
        """Send P: command with given positions."""
        cmd = f"P:{positions[0]},{positions[1]},{positions[2]},{positions[3]},{positions[4]},{positions[5]}\n"
        ser.write(cmd.encode())
        print(f"→ Sent: {cmd.strip()}")
        return positions

    def print_status():
        """Print current status."""
        print(f"\n[Selected: {FINGER_NAMES[selected_finger]} (servo {selected_finger})]")
        print("Positions: ", end="")
        for i, name in enumerate(FINGER_NAMES):
            marker = "→" if i == selected_finger else " "
            print(f"{marker}{name}:{current_positions[i]:3d}%  ", end="")
        print()

    # Start at neutral
    current_positions = send_command([50, 50, 50, 50, 50, 50])
    print_status()

    print("\nReady! Press a key...")

    try:
        import sys, tty, termios

        def getch():
            """Get single character from stdin (Unix/Mac)."""
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

        while True:
            key = getch()

            if key == 'q':
                print("\nQuitting...")
                break

            elif key in '012345':
                selected_finger = int(key)
                print(f"\n✓ Selected: {FINGER_NAMES[selected_finger]}")
                print_status()

            elif key == 'o':
                # Open selected finger
                current_positions[selected_finger] = 0
                current_positions = send_command(current_positions)
                print(f"✓ {FINGER_NAMES[selected_finger]} → OPEN (0%)")
                print_status()

            elif key == 'c':
                # Close selected finger
                current_positions[selected_finger] = 100
                current_positions = send_command(current_positions)
                print(f"✓ {FINGER_NAMES[selected_finger]} → CLOSED (100%)")
                print_status()

            elif key == 'm':
                # Middle selected finger
                current_positions[selected_finger] = 50
                current_positions = send_command(current_positions)
                print(f"✓ {FINGER_NAMES[selected_finger]} → MIDDLE (50%)")
                print_status()

            elif key == '+' or key == '=':
                # Increase by 10%
                current_positions[selected_finger] = min(100, current_positions[selected_finger] + 10)
                current_positions = send_command(current_positions)
                print(f"✓ {FINGER_NAMES[selected_finger]} → {current_positions[selected_finger]}%")
                print_status()

            elif key == '-' or key == '_':
                # Decrease by 10%
                current_positions[selected_finger] = max(0, current_positions[selected_finger] - 10)
                current_positions = send_command(current_positions)
                print(f"✓ {FINGER_NAMES[selected_finger]} → {current_positions[selected_finger]}%")
                print_status()

            elif key == 'a':
                # Move ALL to current selected position
                target = current_positions[selected_finger]
                current_positions = [target] * 5 + [50]  # Keep swivel at 50
                current_positions = send_command(current_positions)
                print(f"✓ ALL fingers → {target}%")
                print_status()

            elif key == 'n':
                # Neutral all
                current_positions = send_command([50, 50, 50, 50, 50, 50])
                print("✓ ALL → NEUTRAL (50%)")
                print_status()

            elif key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt

            else:
                print(f"\nUnknown key: {repr(key)}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    # Return to neutral before exiting
    print("\nReturning to neutral...")
    ser.write(b"P:50,50,50,50,50,50\n")
    time.sleep(0.5)

    ser.close()
    print("✓ Serial connection closed")

except serial.SerialException as e:
    print(f"\nERROR: Could not open serial port {PORT}")
    print(f"  {e}")
    print("\nMake sure:")
    print("  - Arduino is connected via USB")
    print("  - Arduino IDE is closed")
    print("  - No other program is using the port")
    sys.exit(1)
except Exception as e:
    print(f"\nERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
