# AURORA Autonomous - Quick Reference Card

## 🚀 Quick Start

1. **Deploy to Robot**: Build and upload `AURORAAutonomous.java` to your robot
2. **Configure Hardware**: Ensure "odo" odometry device is in hardware config
3. **Edit Sequence**: Customize `runAutonomousSequence()` method
4. **Test & Tune**: Adjust PID values if needed

## 📋 Common Movement Commands

### Basic Movements (Robot-Relative)
```java
moveForward(24);      // Move forward 24 inches
moveBackward(18);     // Move backward 18 inches
moveLeft(12);         // Strafe left 12 inches
moveRight(12);        // Strafe right 12 inches
```

### Rotation
```java
turnToAngle(90);      // Turn to face 90° (field heading)
turnRelative(45);     // Turn 45° counterclockwise
turnRelative(-45);    // Turn 45° clockwise
```

### Advanced Positioning
```java
moveToPosition(24, 36, 90);     // Move to (24, 36) facing 90°
strafeToPosition(24, 36);       // Strafe to (24, 36), keep heading
holdPosition(2.0);              // Hold position for 2 seconds
```

## 🎯 Example Sequences

### Simple Forward and Back
```java
private void runAutonomousSequence() {
    moveForward(30);
    sleep(1000);
    moveBackward(30);
}
```

### Square Pattern
```java
private void runAutonomousSequence() {
    for (int i = 0; i < 4; i++) {
        moveForward(24);
        sleep(300);
        turnRelative(-90);
        sleep(300);
    }
}
```

### Scoring Routine
```java
private void runAutonomousSequence() {
    // Approach scoring zone
    moveForward(36);
    sleep(500);
    
    // Align with target
    turnToAngle(90);
    sleep(500);
    
    // Position for scoring
    moveLeft(6);
    sleep(500);
    
    // Score (add your scoring code)
    // robotManager.getShooterSystem().fireSingleShot();
    sleep(2000);
    
    // Return to safe zone
    moveRight(6);
    sleep(500);
    turnToAngle(0);
    sleep(500);
    moveBackward(36);
}
```

## ⚙️ Configuration Constants

### Movement Parameters
```java
POSITION_TOLERANCE = 0.5;    // Position accuracy (inches)
HEADING_TOLERANCE = 2.0;     // Heading accuracy (degrees)
MAX_MOVE_TIME = 5.0;         // Movement timeout (seconds)
MIN_POWER = 0.15;            // Minimum motor power
MAX_POWER = 0.6;             // Maximum motor power
```

### PID Tuning (Position)
```java
KP_POSITION = 0.08;          // Proportional gain
KI_POSITION = 0.001;         // Integral gain
KD_POSITION = 0.02;          // Derivative gain
```

### PID Tuning (Heading)
```java
KP_HEADING = 0.025;          // Proportional gain
KI_HEADING = 0.0005;         // Integral gain
KD_HEADING = 0.008;          // Derivative gain
```

## 🔧 Odometry Configuration

### In `initializeOdometry()`:
```java
// Set pod offsets (millimeters)
// Left is positive for X, Forward is positive for Y
odometry.setOffsets(-154, 0, DistanceUnit.MM);

// Set pod type
odometry.setEncoderResolution(
    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
);

// Set encoder directions
odometry.setEncoderDirections(
    GoBildaPinpointDriver.EncoderDirection.FORWARD,
    GoBildaPinpointDriver.EncoderDirection.FORWARD
);
```

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot overshoots target | Decrease `KP_POSITION` or `MAX_POWER` |
| Robot oscillates | Increase `KD_POSITION` or decrease `KP_POSITION` |
| Robot doesn't reach target | Increase `MAX_MOVE_TIME` or `POSITION_TOLERANCE` |
| Heading drifts | Check IMU calibration, increase `KP_HEADING` |
| Slow response | Increase `KP_POSITION` or `KP_HEADING` |
| Robot doesn't move | Check `MIN_POWER` (must overcome friction) |

## 📊 Getting Position Info

```java
// Get current position
Pose2D pos = getCurrentPosition();

// Extract components
double x = pos.getX(DistanceUnit.INCH);
double y = pos.getY(DistanceUnit.INCH);
double heading = pos.getHeading(AngleUnit.DEGREES);

// Display in telemetry
telemetry.addData("Position", "X: %.1f, Y: %.1f", x, y);
telemetry.addData("Heading", "%.1f°", heading);
telemetry.update();
```

## 🎮 Hardware Configuration Required

### Driver Station Configuration:
```
Control Hub
  Motors:
    - frontLeft (Motor, Port 0)
    - frontRight (Motor, Port 1)
    - backLeft (Motor, Port 2)
    - backRight (Motor, Port 3)
  
  I2C Bus 0:
    - odo (goBILDA® Pinpoint Odometry Computer)
```

## ⚡ Best Practices

✅ **DO:**
- Add `sleep()` between movements for stability
- Test individual movements before complex sequences
- Use telemetry to monitor position during testing
- Keep robot stationary during initialization
- Start with conservative MAX_POWER values

❌ **DON'T:**
- Skip calibration steps
- Use extremely tight tolerances initially
- Chain movements without pauses
- Ignore timeout warnings
- Forget to test in safe area first

## 🔄 Movement Direction Reference

**From Robot's Perspective:**
- `moveForward()` → Robot's front direction
- `moveBackward()` → Robot's rear direction
- `moveLeft()` → Robot's left side (port side)
- `moveRight()` → Robot's right side (starboard side)

**Field Coordinates:**
- X-axis → Typically field length
- Y-axis → Typically field width
- Heading 0° → Starting orientation
- Heading 90° → 90° counterclockwise from start

## 📝 Coordinate System

```
Field Layout (looking from above):

       Y+ (Field Width)
        ↑
        |
        |
   ←----+----→ X+ (Field Length)
        |
        |
        ↓

Robot at (0,0) heading 0°:
    Front = +Y direction
    Right = +X direction
```

## 🎯 PID Tuning Guide

### Step 1: Tune P (Proportional)
1. Set I and D to 0
2. Increase P until robot responds quickly
3. If it overshoots, reduce P slightly

### Step 2: Add D (Derivative)
1. Keep P from step 1
2. Increase D to reduce overshoot
3. Stop when oscillations are minimal

### Step 3: Add I (Integral)
1. Keep P and D from previous steps
2. Add small I to eliminate steady-state error
3. Keep I very small to avoid instability

### Typical Starting Values:
- **Position**: P=0.08, I=0.001, D=0.02
- **Heading**: P=0.025, I=0.0005, D=0.008

## 💡 Pro Tips

1. **Test movements in order of complexity:**
   - Start: Single straight movements
   - Then: Rotations
   - Finally: Combined movements

2. **Use consistent sleep times:**
   - 300-500ms between movements
   - 1000-2000ms for scoring actions

3. **Monitor odometry status:**
   ```java
   telemetry.addData("Odo Status", odometry.getDeviceStatus());
   telemetry.addData("Odo Freq", odometry.getFrequency());
   telemetry.update();
   ```

4. **Create reusable patterns:**
   ```java
   private void scoreRoutine() {
       moveToPosition(48, 36, 90);
       sleep(500);
       // Scoring code here
       sleep(1000);
       moveToPosition(12, 12, 0);
   }
   ```

5. **Always have a timeout:**
   - Every movement has `MAX_MOVE_TIME` protection
   - Prevents infinite loops if target unreachable

## 📞 Support

For detailed documentation, see: `AURORA_AUTONOMOUS_GUIDE.md`

For GoBilda Pinpoint issues: tech@gobilda.com

For code issues: Check telemetry output and verify hardware configuration

