# uHand Troubleshooting Guide

## Quick Diagnostic Flowchart

```
Servos not moving?
├─→ Run test_basic_connection.py
│   ├─→ No Arduino response? → Check USB cable, upload firmware, press RESET
│   └─→ Arduino responds? → Continue...
│
├─→ Run test_arduino_direct.py
│   ├─→ No movement? → Power issue or Arduino pin damage
│   └─→ Servos move? → Python angle detection issue
│
└─→ Run test_each_servo.py
    ├─→ Some servos work, others don't? → Check wiring or remap pins
    └─→ All work? → Calibration or angle threshold issue
```

## Issues Fixed in Latest Version

### 1. Angle Thresholds Were Wrong
**Symptom:** Fingers barely moved, stuck around 50%, or moved backwards

**Cause:** Default thresholds were changed from correct values:
- Was: `FINGER_OPEN_DEG = 70, FINGER_CLOSE_DEG = 100`
- Now: `FINGER_OPEN_DEG = 180, FINGER_CLOSE_DEG = 90`

**Fixed in:** pose2hand.py lines 54-58

### 2. Arduino Smoothing Disabled
**Symptom:** Jerky, stuttery servo movements

**Cause:** EMA smoothing formula was commented out, servos jumped directly to target

**Fixed in:** hand_driver_sketch.ino line 172

### 3. Python Smoothing Too Aggressive
**Symptom:** Very slow response, fingers lag behind hand movements

**Cause:** `SMOOTH_ALPHA = 0.3` (70% old value, only 30% new)

**Fixed in:** pose2hand.py line 40 (now 0.6 = 40% old, 60% new)

### 4. Send Rate Too High
**Symptom:** Commands pile up in serial buffer, delayed response

**Cause:** `SEND_HZ = 100` overwhelmed 9600 baud serial connection

**Fixed in:** pose2hand.py line 39 (now 20Hz)

### 5. Calibration Position Drift
**Symptom:** Finger position after calibration doesn't match live mode position

**Cause:** Servos auto-detach after 2s during calibration, position drifts

**Fixed in:** pose2hand.py lines 646-660 (automatic keep-alive mechanism)

### 6. EEPROM Calibration Backwards for Inverted Servos
**Symptom:** Middle/Ring/Pinky move opposite direction or barely move

**Cause:** For `invert=1` servos, EEPROM needs MIN > MAX (Arduino inverts AFTER mapping)

**Fix:** Run `reset_calibration.py` or recalibrate properly

## Current Known Issues

### Issue: Specific Arduino Pins Don't Work (D6, D7)

**Symptoms:**
- Servo works when connected to other pins
- Servo doesn't work on D6 or D7
- Swapping connectors shows servo hardware is fine

**Diagnosis:**
```bash
python usb/test_each_servo.py
# Press 0-5 to select each servo
# Press o/c to move between open/close
# Note which servos actually move
```

**Causes:**
1. Arduino pin damaged (electrical short, overvoltage)
2. Loose wiring connection
3. Bad jumper wire

**Solutions:**

**Option A: Fix Wiring**
- Check all connections are secure
- Replace jumper wires
- Re-solder if using permanent connections

**Option B: Remap Pins in Arduino Code**

Edit `hand_driver_sketch/hand_driver_sketch.ino` line 38:

Current:
```c
static const uint8_t SERVO_PINS[6] = {7, 6, 5, 4, 3, 2};
//                                    ↑  ↑  ↑  ↑  ↑  ↑
//                                    T  I  M  R  P  S
```

Example fix (if D6 and D7 don't work, use D8 and D9):
```c
static const uint8_t SERVO_PINS[6] = {8, 9, 5, 4, 3, 2};
//                                    ↑  ↑
//                                    Changed to D8, D9
```

Then:
1. Move thumb servo wire to D8
2. Move index servo wire to D9
3. Re-upload firmware to Arduino

## Calibration Best Practices

### Understanding Inverted Servos

Arduino code has invert flags (line 41):
```c
static const uint8_t invert[6] = {0,0,1,1,1,0};
//                                T I M R P S
```

**For inverted servos (Middle, Ring, Pinky):**
- Arduino calculates: `physical_angle = 180 - commanded_angle`
- To get physical 20°, you must command 160° (180-160=20)
- Therefore: **MIN must be LARGER than MAX in EEPROM**

Example correct calibration:
```
Middle (invert=1): MIN=160° MAX=25°
  → 0% = 160° → physical 20° (open)
  → 100% = 25° → physical 155° (closed)
```

### Calibration Procedure

1. **Start calibration mode:**
   ```bash
   python usb/pose2hand.py --calibrate --log-json cal.log
   ```

2. **For each finger:**
   - Use `j/J` (decrease) and `l/L` (increase) to adjust angle
   - Move to physical OPEN position → press `m` to mark MIN
   - Move to physical CLOSED position → press `n` to mark MAX
   - Press `space` to move to next finger

3. **Save to EEPROM:**
   - Press `s` to save all calibrations
   - Press `g` to verify saved values
   - Press `q` to quit

4. **Important:**
   - For inverted servos, MIN value should be HIGHER than MAX value
   - Calibration keeps servos alive automatically (no drift)
   - Use wide ranges (90°+) for good response

### If Calibration Gets Corrupted

Reset to safe defaults:
```bash
python usb/reset_calibration.py
```

This sets:
- Thumb (normal): 35° → 160°
- Index (normal): 40° → 135° (or 160→35 if inverted)
- Middle (inverted): 160° → 35°
- Ring (inverted): 160° → 35°
- Pinky (inverted): 160° → 35°
- Swivel (normal): 80° → 100°

## Power Troubleshooting

### Symptoms of Insufficient Power
- Multiple servos move slowly
- Servos buzz but don't move
- Servos move fine individually but fail when multiple move
- Arduino resets when servos move

### Test for Power Issues
```bash
python usb/test_power.py
```

Compares:
- One servo moving (low power)
- All servos moving (high power)

If all-servo test is much slower → power issue

### Solutions

**Short term:**
- Use different USB port (some provide more power)
- Use powered USB hub
- Move fewer fingers simultaneously

**Proper solution:**
- External 5V 2A+ power supply for servo power rail
- Connect power supply GND to Arduino GND
- Servos powered from external supply, Arduino from USB
- This is REQUIRED for reliable 6-servo operation

## Diagnostic Tools Reference

### test_basic_connection.py
**Purpose:** Verify Arduino is responding to serial commands

**What it does:**
- Opens serial port
- Sends NEUTRAL, GET, STATUS, P: commands
- Reports which commands get responses

**When to use:**
- First step when nothing works
- Check if firmware is uploaded correctly
- Verify baud rate matches (9600)

### test_arduino_direct.py
**Purpose:** Test servos without Python angle detection

**What it does:**
- Sends simple P: commands (0%, 50%, 100%)
- Tests all servos together
- Tests index finger individually

**When to use:**
- Servos don't move in live mode
- Determine if issue is Arduino or Python

### test_each_servo.py
**Purpose:** Interactive individual servo control

**What it does:**
- Select servo with keys 0-5
- Move with o (open), c (close), m (middle)
- Adjust with +/- keys

**When to use:**
- Test specific servos
- Identify which pins work
- Verify servo hardware

### test_realtime.py
**Purpose:** See live angle detection and commands

**What it does:**
- Shows detected joint angles
- Shows calculated percentages
- Shows commands sent to Arduino
- Updates in real-time

**When to use:**
- Servos move but tracking is wrong
- Verify angle thresholds
- See if percentages make sense

### test_power.py
**Purpose:** Determine if power is limiting performance

**What it does:**
- Moves one servo (low power)
- Moves all servos (high power)
- Compares speeds

**When to use:**
- Multiple servos slow
- Servos work individually but not together

### diagnose_slow.py
**Purpose:** Analyze calibration and response

**What it does:**
- Checks EEPROM calibration ranges
- Tests servo response time
- Identifies small ranges or backwards calibration

**When to use:**
- Servos barely move
- Suspicious calibration values

### reset_calibration.py
**Purpose:** Clear bad EEPROM and set safe defaults

**What it does:**
- Sends CAL commands for all servos
- Saves to EEPROM
- Accounts for invert flags

**When to use:**
- Calibration is corrupted
- Starting fresh calibration
- After changing invert flags

## Advanced Troubleshooting

### Check Serial Communication

Monitor raw serial data:
```bash
# On Mac/Linux
screen /dev/tty.usbmodem1101 9600

# Then manually type commands:
STATUS
GET
P:50,50,50,50,50,50
```

### Check Arduino Pin Functionality

Use multimeter to check:
- 5V between 5V pin and GND
- Signal on servo pins (should be ~1-2ms pulses at 50Hz)

### Firmware Upload Issues

If Arduino doesn't respond:
1. Press RESET button on Arduino
2. Close Arduino IDE (locks serial port)
3. Re-upload hand_driver_sketch.ino
4. Check correct board selected (Arduino UNO)
5. Check correct port selected

### Python Environment Issues

If imports fail:
```bash
# Activate virtual environment
source .pose-venv/bin/activate  # or your venv path

# Reinstall dependencies
pip install -r requirements.txt

# Test imports
python -c "import serial; import cv2; import mediapipe"
```

## Getting Help

When reporting issues, include:

1. **What you tried:**
   - Which diagnostic scripts you ran
   - What the output was

2. **Logs:**
   - Calibration log (cal.log)
   - Live mode log (test.log)
   - Run with `--log-json` to capture

3. **System info:**
   - OS (macOS, Linux, Windows)
   - Arduino board (UNO, etc.)
   - Servo model
   - Power source (USB only, external supply)

4. **Observed behavior:**
   - Which fingers work/don't work
   - Specific error messages
   - When issue started (after transport, after code change, etc.)
