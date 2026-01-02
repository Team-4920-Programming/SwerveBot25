# Pre-Match Calibration Routine Documentation

## Team 4920 - Swerve Drive Diagnostics System

This document describes the pre-match calibration routine used to verify the health and calibration of the swerve drive system before each match.

---

## Table of Contents

1. [Overview](#overview)
2. [How to Run](#how-to-run)
3. [Test Sequence](#test-sequence)
4. [Understanding Results](#understanding-results)
5. [SmartDashboard Keys](#smartdashboard-keys)
6. [Troubleshooting Guide](#troubleshooting-guide)
7. [Wheel Wear Detection](#wheel-wear-detection)
8. [Current Draw Analysis](#current-draw-analysis)
9. [Maintenance Schedule](#maintenance-schedule)
10. [Technical Details](#technical-details)

---

## Overview

The pre-match calibration routine is inspired by practices from top FRC teams like Team 254. It performs a comprehensive diagnostic of the swerve drive system including:

- **Gyro calibration and drift detection**
- **Absolute encoder verification**
- **Module PID response testing**
- **Odometry consistency checking**
- **Wheel wear detection**
- **Motor current draw analysis**

### Why This Matters

| Issue | Impact | Detection Method |
|-------|--------|------------------|
| Gyro drift | Autonomous paths curve/miss targets | Drift tolerance check |
| Encoder failure | Module points wrong direction | Encoder validity check |
| Wheel wear | Odometry inaccurate, paths drift | Distance comparison |
| Mechanical binding | Motor burnout, reduced speed | Current draw monitoring |
| PID tuning issues | Sluggish/oscillating modules | Response time test |

---

## How to Run

### Button Combination
**Press X button** on the driver Xbox controller

### Requirements
- Robot must be on a **flat, level surface**
- Robot must have **enough space to rotate in place** (~2 feet diameter)
- Robot must be **stationary** when calibration starts
- Keep hands/objects **clear of the robot** during the test

### Duration
The full calibration takes approximately **18-22 seconds**

### When to Run
- **Before each match** (in queue or on field)
- **At the start of each day** (baseline readings)
- **After any mechanical work** (belt changes, wheel replacement)
- **After a hard collision** (check for damage)

---

## Test Sequence

The calibration runs through these phases in order:

```
1. INIT (0.5s)
   └── Stops robot, prepares for calibration

2. GYRO_CALIBRATION (2.0s)
   └── Zeros gyro, requires robot to be stationary

3. GYRO_WAIT (1.0s)
   └── Monitors for gyro drift

4. MODULE_ANGLE_CHECK (instant)
   └── Reads all 4 absolute encoders

5. MODULE_CENTER (1.5s)
   └── Commands all modules to 0 degrees

6. ROTATION_TEST_START (instant)
   └── Records starting position

7. ROTATION_TEST_CW (2.0s)
   └── Rotates clockwise at 0.5 rad/s (~57 degrees)

8. ROTATION_TEST_CCW (2.0s)
   └── Rotates counter-clockwise at 0.5 rad/s (~57 degrees)

9. ROTATION_TEST_RETURN (0.5s)
   └── Verifies return to starting heading

10. MODULE_RESPONSE_TEST (2.8s)
    └── Tests each module's PID response (0.7s each)

11. ODOMETRY_CONSISTENCY_TEST (0.5s)
    └── Checks for translational drift during rotation

12. MODULE_CURRENT_TEST (4.8s)
    └── Tests current draw for each module (1.2s each)

13. COMPLETE
    └── All tests passed!
```

---

## Understanding Results

### Overall Status

| SmartDashboard Key | Meaning |
|-------------------|---------|
| `Calibration/All Critical Tests Passed` | Core functionality OK (gyro, encoders, PID) |
| `Calibration/Has Warnings` | Non-critical issues detected (wheel wear, current) |
| `Calibration Complete` | Calibration finished successfully |

### Pass/Fail Criteria

**Critical Tests (will FAIL calibration):**
- Gyro drift > 1.0 degrees during calibration
- Any encoder returns NaN or Infinite
- Any encoder reading outside ±360 degrees
- Module fails to center within 4 degrees
- Rotation return error > 10 degrees
- Module PID response error > 15 degrees

**Warning Tests (will show warnings but NOT fail):**
- Module PID response error 8-15 degrees (logs warning, consider tuning)
- Odometry drift > 5cm during rotation
- Module distance deviation > 5cm from average
- Angle motor current > 15A
- Drive motor current > 20A

---

## SmartDashboard Keys

### Calibration Status
| Key | Type | Description |
|-----|------|-------------|
| `Calibration Status` | String | Current phase description |
| `Calibration Complete` | Boolean | True when finished successfully |
| `Calibration Error` | String | Error message if failed |

### Module Encoder Data
| Key | Type | Description |
|-----|------|-------------|
| `Cal/FrontLeft Raw Angle` | Number | Raw encoder reading (degrees) |
| `Cal/FrontRight Raw Angle` | Number | Raw encoder reading (degrees) |
| `Cal/BackLeft Raw Angle` | Number | Raw encoder reading (degrees) |
| `Cal/BackRight Raw Angle` | Number | Raw encoder reading (degrees) |
| `Cal/[Module] OK` | Boolean | Encoder validation result |
| `Cal/[Module] Suggested Offset` | Number | Offset value for JSON config |

### Test Results
| Key | Type | Description |
|-----|------|-------------|
| `Calibration/Gyro OK` | Boolean | Gyro calibration passed |
| `Calibration/Rotation Test OK` | Boolean | Rotation tracking passed |
| `Calibration/Odometry OK` | Boolean | Position drift within tolerance |
| `Calibration/Module X Angle OK` | Boolean | Encoder X is valid |
| `Calibration/Module X PID OK` | Boolean | Module X responds correctly |
| `Calibration/Module X Current OK` | Boolean | Current draw is normal |

### Module Center Test Results
| Key | Type | Description |
|-----|------|-------------|
| `Cal/[Module] Center Error` | Number | Error from 0 degrees (forward) |
| `Cal/[Module] Centered` | Boolean | Module centered within tolerance |

### Module PID Response Results
| Key | Type | Description |
|-----|------|-------------|
| `Cal/[Module] Response Error` | Number | Error from 45 degree target |
| `Cal/[Module] PID OK` | Boolean | Module responded within tolerance |

### Rotation Test Diagnostics
| Key | Type | Description |
|-----|------|-------------|
| `Cal/Rotation Return Error` | Number | Total heading error after CW+CCW (degrees) |
| `Cal/Rotation Quality` | String | EXCELLENT, GOOD, or NEEDS_CALIBRATION |
| `Cal/CW Rotation (deg)` | Number | Actual CW rotation amount |
| `Cal/CCW Rotation (deg)` | Number | Actual CCW rotation amount |
| `Cal/Rotation Asymmetry` | Number | Difference between CW and CCW amounts |
| `Cal/[Module] Steering Return` | Number | Module steering angle return error |
| `Cal/Rotation Recommendation` | String | Specific action based on diagnosis |

### Advanced Diagnostics
| Key | Type | Description |
|-----|------|-------------|
| `Cal/Odometry Drift (m)` | Number | How far robot "moved" during rotation |
| `Cal/[Module] Distance` | Number | Distance traveled during rotation |
| `Cal/[Module] Peak Angle I` | Number | Peak angle motor current (Amps) |
| `Cal/[Module] Peak Drive I` | Number | Peak drive motor current (Amps) |
| `Cal/Wheel Wear Warning` | String | Module with suspected wear |

---

## Troubleshooting Guide

### Gyro Calibration Failed

**Symptoms:**
- `Calibration/Gyro OK` = false
- Calibration fails during GYRO_WAIT phase

**Possible Causes:**
1. Robot was moved during calibration
2. Gyro hardware malfunction
3. Excessive vibration (nearby robots, music)
4. Loose gyro mounting

**Solutions:**
1. Ensure robot is completely stationary
2. Wait for other robots to stop moving
3. Check gyro mounting screws
4. Verify gyro wiring connections

---

### Module Encoder Failed (NaN or out of range)

**Symptoms:**
- `Cal/[Module] OK` = false
- Raw angle shows NaN or very large number

**Possible Causes:**
1. Encoder cable disconnected
2. Encoder damaged
3. SparkMax not receiving encoder signal
4. CAN bus communication issue

**Solutions:**
1. Check encoder cable connections
2. Verify encoder LED is blinking
3. Check SparkMax client for encoder readings
4. Inspect encoder for physical damage
5. Try reseating connections

---

### Module Center Failed

**Symptoms:**
- Calibration fails during MODULE_CENTER phase
- One or more modules don't point forward

**Possible Causes:**
1. PID tuning incorrect
2. Mechanical binding
3. Encoder offset needs calibration
4. Motor controller fault

**Solutions:**
1. Manually check if module rotates freely
2. Verify encoder offset in JSON config files
3. Check for debris blocking rotation
4. Review PID constants in pidfproperties.json

---

### Rotation Test Failed

**Symptoms:**
- `Calibration/Rotation Test OK` = false
- `Cal/Rotation Quality` = "NEEDS_CALIBRATION"
- Robot doesn't return to starting heading (error > 5°)

**Understanding the Diagnostic Output:**

The calibration now provides detailed diagnostics to help identify the root cause:

| SmartDashboard Key | What It Tells You |
|-------------------|-------------------|
| `Cal/Rotation Return Error` | Total heading error after CW+CCW rotation |
| `Cal/CW Rotation (deg)` | How far robot actually rotated clockwise |
| `Cal/CCW Rotation (deg)` | How far robot actually rotated counter-clockwise |
| `Cal/Rotation Asymmetry` | Difference between CW and CCW rotation amounts |
| `Cal/[Module] Steering Return` | How well each module's steering returned to start |
| `Cal/Rotation Recommendation` | Specific action to take based on diagnosis |

**Interpreting Rotation Quality:**

| Quality | Return Error | Meaning |
|---------|-------------|---------|
| EXCELLENT | < 3° | Ready for competition autonomous |
| GOOD | 3-5° | Acceptable, minor tuning may help |
| NEEDS_CALIBRATION | > 5° | Issue detected - see recommendation |

**Diagnostic Scenarios:**

1. **Asymmetric Rotation (asymmetry > 5°)**
   - CW rotation differs significantly from CCW rotation
   - Indicates one-sided issue: debris, uneven wheel wear, or module alignment
   - Check the side that rotates less (more friction/slip on that side)

2. **Module Steering Inconsistency**
   - One module's steering doesn't return to starting angle
   - `Cal/[Module] Steering Return` > 5° for specific module
   - Check that module's encoder offset, PID tuning, or mechanical binding

3. **Symmetric Slip (low asymmetry, all modules similar)**
   - All modules slipping equally
   - Floor surface issue or all wheels worn evenly
   - Try different floor surface or measure all wheel diameters

**Solutions Based on Diagnosis:**

| Diagnosis | Recommended Action |
|-----------|-------------------|
| Asymmetric - CW worse | Check left side modules, debris, wheel wear |
| Asymmetric - CCW worse | Check right side modules, debris, wheel wear |
| Specific module issue | Recalibrate that module's encoder offset |
| General slippage | Test on carpet/grippy surface, check wheel condition |

---

### Module PID Response Failed or Warning

**Symptoms:**
- `Cal/[Module] PID OK` = false (error > 15 degrees)
- `Cal/[Module] Response Error` shows 8-15 degrees (warning)
- Module slow to reach target angle

**Understanding the Test:**
- The test commands each module to rotate to 45 degrees
- After 0.7 seconds, it checks how close the module got
- Error < 8°: Pass (good response)
- Error 8-15°: Pass with warning (consider tuning)
- Error > 15°: Fail (something is wrong)

**Possible Causes:**
1. PID constants need tuning (most common for warnings)
2. Motor underpowered
3. Mechanical resistance or friction
4. Encoder feedback issue
5. Static friction (module needs more force to start moving)

**Solutions:**
1. Increase P gain slightly in pidfproperties.json
2. Check for binding when manually rotating
3. Verify motor current limits aren't too low
4. Listen for unusual motor sounds
5. For warnings (8-15°): Robot will still drive fine, but autonomous precision may be affected

---

### High Odometry Drift

**Symptoms:**
- `Cal/Odometry Drift (m)` > 0.05
- `Calibration/Odometry OK` = false (warning)

**Possible Causes:**
1. Uneven wheel wear
2. Drive encoder calibration off
3. Module alignment issues
4. Wheel slippage

**Solutions:**
1. Measure wheel diameters (should all be equal)
2. Replace worn wheels
3. Verify drive encoder calibration
4. Check module mounting alignment

---

### High Motor Current

**Symptoms:**
- `Cal/[Module] Peak Angle I` > 15A
- `Cal/[Module] Peak Drive I` > 20A (when stationary)

**Possible Causes:**
1. Mechanical binding
2. Belt too tight
3. Debris in module
4. Bearing wear
5. Motor starting to fail

**Solutions:**
1. Inspect module for debris/damage
2. Check belt tension (should have slight play)
3. Lubricate bearings if allowed
4. Listen for grinding sounds
5. Feel for rough spots when rotating by hand

---

## Wheel Wear Detection

### How It Works

During the rotation test, each wheel travels along an arc. If all wheels have the same diameter, they should all travel the same distance (as measured by encoders). If one wheel is worn (smaller diameter), its encoder will report traveling a greater distance for the same physical movement.

### Interpreting Results

Check `Cal/[Module] Distance` values:

```
Example - Healthy Robot:
  FrontLeft:  0.453m
  FrontRight: 0.461m
  BackLeft:   0.448m
  BackRight:  0.455m
  → All within ~2% of each other ✓

Example - Worn BackRight Wheel:
  FrontLeft:  0.453m
  FrontRight: 0.458m
  BackLeft:   0.451m
  BackRight:  0.512m  ← 13% higher!
  → BackRight wheel is worn, needs replacement
```

### Wear Threshold

| Deviation | Status | Action |
|-----------|--------|--------|
| < 5% | Normal | No action needed |
| 5-10% | Warning | Plan wheel replacement |
| > 10% | Critical | Replace wheel before next match |

### Measuring Wheel Diameter

If wear is detected:
1. Remove the module from robot (if possible)
2. Measure wheel diameter with calipers
3. Compare to nominal diameter (usually 4" or 3")
4. Replace if worn more than 1/16" (1.5mm)

---

## Current Draw Analysis

### Normal Operating Currents

| Motor | Idle | During Rotation Test | Warning Level |
|-------|------|---------------------|---------------|
| Angle (NEO 550) | < 1A | 5-12A | > 15A |
| Drive (Kraken X60) | < 1A | 2-8A | > 20A |

### What High Current Indicates

**Angle Motor High Current:**
- Steering mechanism binding
- Azimuth bearing worn
- Belt too tight
- Debris between rotating parts

**Drive Motor High Current (when wheels aren't spinning):**
- Brake mode engaged incorrectly
- Gearbox binding
- Wheel rubbing on frame
- Drive train mechanical issue

### Current Trend Analysis

Run calibration multiple times throughout the day and compare:

```
Match 1:  FrontLeft Angle = 8.2A
Match 3:  FrontLeft Angle = 9.1A
Match 6:  FrontLeft Angle = 11.4A  ← Trending up, investigate!
Match 8:  FrontLeft Angle = 14.8A  ← Critical, check immediately!
```

---

## Maintenance Schedule

### Before Each Match
- Run full calibration
- Check for warnings
- Verify all modules pass

### Between Matches
- Note any warnings for pit crew
- Quick visual inspection
- Check for loose bolts

### After Every 3-4 Matches
- Check wheel tread condition
- Inspect belts for wear
- Listen for unusual sounds

### Daily (at competition)
- Run calibration first thing
- Record baseline values
- Compare to previous day
- Check battery voltage

### After Each Event
- Full mechanical inspection
- Replace worn wheels
- Check all bearings
- Lubricate as needed
- Reset encoder offsets if modules were removed

---

## Technical Details

### Tolerances Used

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Gyro drift tolerance | 1.0° | Prevents autonomous heading errors |
| Module center tolerance | 4.0° | Allows margin for uncalibrated offsets |
| Module PID response tolerance | 15.0° | Sanity check (warning at 8°) |
| Rotation return - EXCELLENT | < 3.0° | Ready for precision autonomous |
| Rotation return - GOOD | 3.0-5.0° | Acceptable for most autonomous |
| Rotation return - FAIL | > 5.0° | Needs calibration before competition |
| Rotation asymmetry threshold | 5.0° | Indicates one-sided mechanical issue |
| Module steering return threshold | 5.0° | Identifies problematic module |
| Odometry position tolerance | 0.05m | Detects significant wheel wear |
| Max angle motor current | 15A | NEO 550 safe continuous |
| Max drive motor current | 20A | Kraken X60 idle threshold |

### Test Timing

| Phase | Duration | Purpose |
|-------|----------|---------|
| Gyro calibration | 2.0s | Allow gyro to stabilize |
| Gyro wait | 1.0s | Detect drift |
| Module center | 1.5s | Allow PID to settle |
| Rotation CW | 2.0s | Test one direction (0.5 rad/s) |
| Rotation CCW | 2.0s | Test opposite direction (0.5 rad/s) |
| Module response | 0.7s each | PID response check with settling time |
| Current test | 1.0s each | Capture peak currents |

### Files Modified

- `PreMatchCalibrationCommand.java` - Main calibration command
- `SwerveSubsystem.java` - Added `getPreMatchCalibrationCommand()` method
- `RobotContainer.java` - Button binding (X button)

### Module Order

The code uses this module ordering (matches YAGSL convention):
```
Index 0: FrontLeft
Index 1: FrontRight
Index 2: BackLeft
Index 3: BackRight
```

### DogLog Keys

All calibration data is also logged to DogLog for post-match analysis:
- `Calibration/State` - Current state machine state
- `Calibration/[Module]/RawPosition` - Encoder readings
- `Calibration/[Module]/RelativeAngle` - Offset-corrected angle
- `Calibration/[Module]/AbsoluteAngle` - Raw absolute encoder position
- `Calibration/[Module]/CenterError` - Error when centering
- `Calibration/[Module]/ResponseTarget` - PID test target angle
- `Calibration/[Module]/ResponseActual` - Actual angle achieved
- `Calibration/[Module]/ResponseError` - PID response error
- `Calibration/[Module]/DistanceTraveled` - Wheel distances
- `Calibration/[Module]/PeakAngleCurrent` - Peak angle motor current
- `Calibration/[Module]/PeakDriveCurrent` - Peak drive motor current
- `Calibration/Odometry/Drift` - Position drift
- `Calibration/Warning` - Any warnings generated
- `Calibration/Error` - Any errors that caused failure

---

## Quick Reference Card

### Pre-Match Checklist
```
[ ] Robot on flat surface
[ ] Clear space around robot
[ ] Press X button
[ ] Wait 15-20 seconds
[ ] Check "All Critical Tests Passed" = true
[ ] Note any warnings for pit crew
[ ] Ready to queue!
```

### Emergency Fixes at Competition

| Problem | Quick Fix |
|---------|-----------|
| Gyro fails | Move to quieter area, retry |
| Encoder NaN | Reseat encoder cable |
| Module won't center | Check for debris, power cycle |
| High current | Manual rotation check, look for binding |
| Wheel wear warning | Swap wheels between matches |

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025 Season | Initial calibration routine |
| 1.1 | 2025 Season | Added wheel wear detection |
| 1.2 | 2025 Season | Added current draw monitoring |
| 1.3 | 2025 Season | Added odometry consistency check |
| 1.4 | 2025 Season | Tuned tolerances based on real-world testing |
|     |             | - Module center: uses relative angle (offset-corrected) |
|     |             | - PID response: 15° fail threshold, 8° warning threshold |
|     |             | - Added detailed SmartDashboard keys for debugging |
|     |             | - Fixed module response test timing bug |

---

*Document maintained by Team 4920 Programming*
