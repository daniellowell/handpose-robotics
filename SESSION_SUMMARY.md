# Session Summary: HandPose Robotics Debugging

**UPDATED: October 4, 2025 - Session 2**

## Current State

The uHand robotic hand controller uses MediaPipe hand tracking to control servo motors via an Arduino. Major issues from Session 1 have been resolved. Current focus is on calibration and fine-tuning.

## Session Timeline

### Initial Problem Statement
User reported:
- Fingers sluggish or unresponsive
- Unintended finger movements (moving one finger causes others to move)
- Thumb not moving predictably
- General jitter and twitching

### Attempts & Changes Made

#### 1. Joint-Angle Detection Implementation
**Changed from:** TIP-MCP distance-based flexion detection
**Changed to:** PIP/DIP joint angle-based detection

**Files modified:**
- `usb/pose2hand.py` - Replaced `flexion_percent()` function
- Added `_angle_at()` and `_angle_to_pct()` helper functions
- Added CLI flags: `--finger-open`, `--finger-close`, `--thumb-open`, `--thumb-close`, `--dip-blend`

**Reasoning:** Joint angles should be more stable than vertical distance measurements

**Result:** Did NOT solve the core issue. Angles were consistently reading 180° (open) even when hand was closed.

#### 2. Angle Glitch Filtering
**Attempt:** Added threshold to filter out impossible angles < 30°, later reduced to < 10°

**Files modified:**
- `usb/pose2hand.py` - Modified `_angle_at()` function

**Result:** Made problem worse - legitimate finger flexion was being filtered out as glitches

#### 3. Arduino Optimization
**Changes made:**
- Replaced String objects with char arrays for faster serial parsing
- Increased alpha (EMA smoothing) from 0.7 to 0.85 for faster response
- Reduced idle timeout from 4000ms to 2000ms
- Added `STATUS` command for servo feedback

**Files modified:**
- `hand_driver_sketch/hand_driver_sketch.ino`

**Result:** Improved performance but did not address core detection issue

#### 4. Servo Feedback & Logging
**Added:**
- `STATUS` command in Arduino to report actual servo positions
- Non-blocking status queries in Python (every 2 seconds when logging enabled)
- Comprehensive log analyzer with diagnostics

**Files created/modified:**
- `usb/analyze_log.py` - Created comprehensive diagnostic tool
- `usb/pose2hand.py` - Added servo status polling
- `hand_driver_sketch/hand_driver_sketch.ino` - Added STATUS command

**Result:** Excellent diagnostic capability but revealed the core problem wasn't in servo control

#### 5. Calibration Improvements
**Added:** Single-servo `S:i,angle` command to prevent jitter on inactive fingers during calibration

**Files modified:**
- `hand_driver_sketch/hand_driver_sketch.ino` - Added S: command
- `usb/pose2hand.py` - Modified calibration to use S: commands

**Result:** Improved calibration experience but didn't solve live tracking issues

#### 6. ROOT CAUSE DISCOVERED (Final)
**Problem:** Using only 2D (x,y) coordinates for angle calculation, ignoring z-depth

When hand is held with palm facing camera, fingers bending toward/away from camera only change the z-coordinate, not x or y. The 2D angle calculation always returned 180° (straight).

**Fix applied:**
- Modified `_angle_at()` to use full 3D vectors (x, y, z)
- Changed from 2D hypot to 3D magnitude calculation
- Updated dot product to include z component

**Files modified:**
- `usb/pose2hand.py` - Lines 161-169

## Current Issues

### Primary Issue (POSSIBLY RESOLVED)
**Angle calculation was 2D, needed to be 3D**
- MediaPipe provides x, y, z coordinates (all normalized 0-1)
- Original code only used x, y
- Fingers bending toward/away from camera were invisible in 2D projection
- **Status:** NEEDS TESTING - fix just applied

### Secondary Issues (Still Present)
1. **Send rate logging bug** - Analyzer reports 0.0 Hz despite packets being sent
2. **Thumb calibration** - Narrow range (60°-95°) may need adjustment
3. **General jitter** - 2-7% average frame-to-frame changes with occasional large jumps

## Files Modified This Session

### Core Implementation
1. **`usb/pose2hand.py`**
   - Replaced TIP-MCP with PIP/DIP joint angle detection
   - Added 3D angle calculation (CRITICAL FIX)
   - Added CLI flags for configurable thresholds
   - Added servo status polling (non-blocking)
   - Added debug error logging

2. **`hand_driver_sketch/hand_driver_sketch.ino`**
   - Optimized serial parsing (String → char arrays)
   - Increased alpha to 0.85
   - Reduced idle_ms to 2000
   - Added STATUS command for servo feedback
   - Added S:i,angle command for single-servo control

### Diagnostic Tools
3. **`usb/analyze_log.py`** (CREATED)
   - Comprehensive session diagnostics
   - Performance analysis
   - Joint angle analysis
   - Servo feedback display
   - Jitter detection
   - Automatic recommendations

### Documentation
4. **`usb/PROJECT_GUIDE.md`**
   - Updated with joint-angle features
   - Added logging & analysis section
   - Updated troubleshooting guide
   - Added Arduino command reference

5. **`README.md`**
   - Added "Recent Improvements" section
   - Updated USB workflow instructions
   - Added analyzer usage examples
   - Updated repository map

## Testing Required

### Immediate Test (CRITICAL)
```bash
cd handpose-robotics/usb
python3 pose2hand.py --log-json test-3d.log \
  --finger-open 175 --finger-close 85 \
  --thumb-open 175 --thumb-close 70
```

**Expected result:** Fingers should now respond to flexion regardless of hand orientation, since 3D angles are now being calculated.

### Validation Steps
1. Make a fist - all fingers should show angles < 90° in log
2. Open hand - all fingers should show angles > 170° in log
3. Move individual fingers - only that finger's percentage should change significantly
4. Check servo feedback - positions should track commanded percentages

### Log Analysis
```bash
python3 analyze_log.py test-3d.log
```

Look for:
- PIP/DIP angles varying (not stuck at 180°)
- Servo positions responding to commands
- Reduced jitter compared to previous sessions

## Key Learnings

1. **MediaPipe provides 3D coordinates** - Always use x, y, AND z for spatial calculations
2. **2D projection loses depth information** - Critical for fingers bending toward/away from camera
3. **Silent exception handling hides bugs** - Added debug logging to expose issues
4. **Visual overlay can be misleading** - 2D overlay looked correct but 3D geometry was wrong
5. **Always validate assumptions** - "Using 2D x,y" was never questioned until root cause analysis

## Session 2 Updates (October 4, 2025)

### Issues Addressed
1. **Calibration GET command fixed** - Now reads EEPROM values from Arduino
2. **Performance optimizations** - No-op logging when disabled, removed debug spam
3. **Swivel feature removed** - Was interfering with finger operation
4. **Arduino config restored** - Fixed swivel defaults

### Current Issues (Session 2)
1. **Thumb/finger grip not tight enough** - Servo calibration limits preventing full closure
2. **Swivel at neutral** - Wrist rotation not implemented

### Files Modified (Session 2)
- `usb/pose2hand.py` - Bug fixes, performance optimizations, swivel removal
- `hand_driver_sketch/hand_driver_sketch.ino` - Config fixes
- `README.md` - Updated with Session 2 changes
- `CHANGED_FILES.md` - Complete change history
- `CURRENT_SESSION_SUMMARY.md` - New detailed session summary

## Next Steps

### For User
1. **Recalibrate servo limits** using `--calibrate` mode
   - Lower minimum angles for tighter grip
   - Test each finger's full range of motion
2. **Verify performance improvements** are working correctly
3. **Consider swivel implementation** after fingers/thumb perfected

### If Continuing Development
2. Fine-tune angle thresholds based on real data
3. Optimize DIP blend factor for best smoothness/responsiveness balance
4. Consider removing 10° glitch filter if no longer needed
5. Update documentation with 3D angle discovery

## Next Steps (If 3D Fix Doesn't Work)

1. Verify MediaPipe z-coordinates are actually changing when fingers bend
2. Add visualization of 3D landmark positions
3. Consider alternative approaches:
   - Landmark distance ratios
   - Machine learning on landmark patterns
   - Hybrid 2D/3D approach based on hand orientation

## Command Reference

### Run with 3D angles
```bash
python3 usb/pose2hand.py --log-json session.log \
  --finger-open 175 --finger-close 85 \
  --thumb-open 175 --thumb-close 70 \
  --dip-blend 0.35
```

### Analyze session
```bash
python3 usb/analyze_log.py session.log
```

### Calibrate servos
```bash
python3 usb/pose2hand.py --calibrate
```

### Check USB ports (macOS)
```bash
ls /dev/tty.*
```
