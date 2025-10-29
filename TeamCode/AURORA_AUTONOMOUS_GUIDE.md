# AURORA Autonomous OpMode Guide

## Overview

The `AURORAAutonomous.java` OpMode provides intelligent, odometry-based autonomous navigation for your FTC robot. It integrates the AURORA system manager with the GoBilda Pinpoint Odometry Computer to deliver precise, self-correcting autonomous movements.

## Features

### ✨ Key Capabilities

- **Precise Odometry Navigation**: Uses the GoBilda Pinpoint for accurate position tracking
- **Robot-Relative Movement**: Easy-to-use commands relative to robot orientation
- **Field-Relative Positioning**: Absolute field positioning capabilities
- **IMU + Odometry Fusion**: Combines odometry and IMU for accurate heading tracking
- **Automatic Course Correction**: Real-time correction during movement
- **PID-Based Control**: Smooth, accurate position and heading control
- **Intelligent Error Handling**: Graceful degradation if odometry is unavailable

## Movement Functions

### Robot-Relative Movement (Recommended for most cases)

These functions move the robot relative to its current orientation, regardless of field position:

```java
// Move left 12 inches (strafe left relative to robot front)
moveLeft(12);

// Move right 8 inches (strafe right relative to robot front)
moveRight(8);

// Move forward 24 inches (drive forward in robot's current direction)
moveForward(24);

// Move backward 18 inches (drive backward in robot's current direction)
moveBackward(18);
```

### Rotation Functions

```java
// Turn to face 90 degrees (absolute field heading)
turnToAngle(90);

// Turn 45 degrees counterclockwise from current heading
turnRelative(45);

// Turn 90 degrees clockwise from current heading
turnRelative(-90);
```

### Field-Relative Movement (Advanced)

These functions use absolute field coordinates:

```java
// Move to position (24, 36) on the field at 90-degree heading
moveToPosition(24, 36, 90);

// Strafe to position (12, 24) while maintaining current heading
strafeToPosition(12, 24);
```

### Utility Functions

```java
// Get current robot position
Pose2D position = getCurrentPosition();
double x = position.getX(DistanceUnit.INCH);
double y = position.getY(DistanceUnit.INCH);
double heading = position.getHeading(AngleUnit.DEGREES);

// Hold current position for 2 seconds
holdPosition(2.0);
```

## Creating Your Autonomous Sequence

### Step 1: Customize the Configuration

Open `AURORAAutonomous.java` and adjust the odometry settings in the `initializeOdometry()` method:

```java
// Set odometry pod offsets for your robot (in millimeters)
// X offset: how far sideways from center (left is positive)
// Y offset: how far forward from center (forward is positive)
odometry.setOffsets(-154, 0, DistanceUnit.MM);

// Set the type of odometry pods you're using
odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
// OR for swingarm pods:
// odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

// Set encoder directions (adjust if needed)
odometry.setEncoderDirections(
    GoBildaPinpointDriver.EncoderDirection.FORWARD,
    GoBildaPinpointDriver.EncoderDirection.FORWARD
);
```

### Step 2: Write Your Autonomous Sequence

Edit the `runAutonomousSequence()` method with your specific tasks:

```java
private void runAutonomousSequence() {
    telemetry.addLine("Starting Autonomous Sequence...");
    telemetry.update();

    // Example: Navigate to scoring position
    moveForward(30);        // Drive forward to approach target
    sleep(500);             // Brief pause for stability
    
    turnToAngle(45);        // Face the scoring target
    sleep(500);
    
    moveRight(12);          // Strafe into scoring position
    sleep(500);
    
    // Score your game piece here
    // Example: robotManager.getShooterSystem().fireSingleShot();
    sleep(1000);
    
    // Return to safe zone
    moveLeft(12);           // Strafe back
    sleep(500);
    
    turnToAngle(0);         // Face forward again
    sleep(500);
    
    moveBackward(30);       // Return to starting area
    
    telemetry.addLine("✅ Sequence Complete!");
    telemetry.update();
}
```

### Step 3: Tune PID Parameters (If Needed)

If movements are overshooting or oscillating, adjust these constants at the top of the file:

```java
// Position control (for linear movement)
private static final double KP_POSITION = 0.08;  // Increase for faster response
private static final double KI_POSITION = 0.001; // Increase to eliminate steady-state error
private static final double KD_POSITION = 0.02;  // Increase to reduce overshoot

// Heading control (for rotation)
private static final double KP_HEADING = 0.025;  // Increase for faster rotation
private static final double KI_HEADING = 0.0005; // Increase to eliminate drift
private static final double KD_HEADING = 0.008;  // Increase to reduce oscillation
```

### Step 4: Adjust Movement Parameters

```java
// Tolerance for position and heading
private static final double POSITION_TOLERANCE = 0.5; // inches (smaller = more precise)
private static final double HEADING_TOLERANCE = 2.0;  // degrees (smaller = more precise)

// Maximum movement time before timeout
private static final double MAX_MOVE_TIME = 5.0; // seconds

// Motor power limits
private static final double MIN_POWER = 0.15;  // Minimum power (prevents stalling)
private static final double MAX_POWER = 0.6;   // Maximum power (safer for autonomous)
```

## Example Autonomous Sequences

### Example 1: Simple Square Pattern

```java
private void runAutonomousSequence() {
    // Drive a 24-inch square
    moveForward(24);
    sleep(300);
    
    turnRelative(-90);  // Turn right
    sleep(300);
    
    moveForward(24);
    sleep(300);
    
    turnRelative(-90);  // Turn right
    sleep(300);
    
    moveForward(24);
    sleep(300);
    
    turnRelative(-90);  // Turn right
    sleep(300);
    
    moveForward(24);
    sleep(300);
    
    turnRelative(-90);  // Turn right - back to start
}
```

### Example 2: Scoring Routine

```java
private void runAutonomousSequence() {
    // Start position to scoring position
    moveForward(36);              // Approach the scoring area
    sleep(500);
    
    turnToAngle(90);              // Face the basket
    sleep(500);
    
    moveLeft(6);                  // Fine adjustment
    sleep(500);
    
    // Score the specimen/sample
    if (robotManager.getShooterSystem() != null) {
        robotManager.getShooterSystem().autoShootSmart(1, false, 
            ShooterConfig.ShooterPreset.LONG_RANGE);
    }
    sleep(2000);                  // Wait for scoring
    
    // Return to safe zone
    moveRight(6);                 // Move away from basket
    sleep(500);
    
    turnToAngle(0);               // Face forward
    sleep(500);
    
    moveBackward(36);             // Return to starting area
    
    telemetry.addLine("✅ Scoring Complete!");
    telemetry.update();
}
```

### Example 3: Multiple Waypoints

```java
private void runAutonomousSequence() {
    // Navigate through multiple waypoints
    
    // Waypoint 1: Pick up sample
    moveToPosition(24, 12, 0);
    sleep(500);
    // [Pick up sample here]
    
    // Waypoint 2: Navigate to scoring area
    moveToPosition(48, 36, 90);
    sleep(500);
    
    // Waypoint 3: Score sample
    moveToPosition(54, 36, 90);
    sleep(500);
    // [Score sample here]
    
    // Waypoint 4: Return to safe zone
    moveToPosition(12, 12, 0);
    sleep(500);
    
    telemetry.addLine("✅ Multi-waypoint navigation complete!");
    telemetry.update();
}
```

## Understanding Coordinate Systems

### Robot-Relative Movements
- **Forward/Backward**: Along the robot's current front-back axis
- **Left/Right**: Perpendicular to the robot's front-back axis
- **Rotation**: Changes the robot's heading

These are independent of the robot's position on the field and always move relative to where the robot is currently facing.

### Field-Relative Movements
- **X-axis**: Typically runs the length of the field
- **Y-axis**: Typically runs the width of the field
- **Heading**: Angle relative to the starting orientation (0°)

The odometry system tracks the robot's absolute position on the field from the start.

## How It Works

### Odometry Integration
1. **Initialization**: The Pinpoint odometry computer is configured with pod offsets and encoder settings
2. **Position Tracking**: Odometry constantly updates X, Y position and heading
3. **Movement Commands**: When you call a movement function, it:
   - Records the starting position
   - Calculates the target position
   - Uses PID control to reach the target
   - Continuously corrects for drift

### Self-Correction
The system continuously:
- Monitors actual position vs. target position
- Calculates error in both position and heading
- Adjusts motor powers to minimize error
- Maintains heading during straight movements

### PID Control
- **P (Proportional)**: Responds proportionally to error (bigger error = more power)
- **I (Integral)**: Eliminates steady-state error over time
- **D (Derivative)**: Reduces overshoot and oscillation

## Troubleshooting

### Robot doesn't move accurately
1. **Check odometry pod alignment**: Ensure pods are properly installed
2. **Verify encoder directions**: X pod should increase when moving forward, Y pod when moving left
3. **Tune PID constants**: Increase KP for faster response, increase KD to reduce overshoot
4. **Check tolerances**: Tighter tolerances require more precise movements

### Robot oscillates around target
1. **Reduce KP**: Lower proportional gain
2. **Increase KD**: Higher derivative gain to dampen oscillations
3. **Lower MAX_POWER**: Reduce maximum motor power

### Robot doesn't reach target position
1. **Increase MAX_MOVE_TIME**: Give more time to reach position
2. **Increase tolerances**: Make POSITION_TOLERANCE and HEADING_TOLERANCE larger
3. **Check MIN_POWER**: Ensure it's high enough to overcome friction
4. **Verify odometry is working**: Check telemetry for position updates

### Heading drifts during movement
1. **Verify IMU calibration**: Robot must be stationary during initialization
2. **Check yaw scalar**: May need fine-tuning (see GoBilda documentation)
3. **Increase heading correction**: Raise KP_HEADING slightly

## Hardware Configuration

### Required Components
1. **4 Mecanum Drive Motors**: Named "frontLeft", "frontRight", "backLeft", "backRight"
2. **GoBilda Pinpoint Odometry Computer**: Named "odo" in hardware config
3. **Two Odometry Pods**: Properly mounted perpendicular to each other

### Hardware Configuration in Driver Station
```
Control Hub:
  Motors:
    - frontLeft (Motor, Port 0)
    - frontRight (Motor, Port 1)
    - backLeft (Motor, Port 2)
    - backRight (Motor, Port 3)
  
  I2C Bus 0:
    - odo (goBILDA® Pinpoint Odometry Computer)
```

## Advanced Features

### Custom Movement Profiles
You can create custom movement functions by combining primitives:

```java
// Custom L-shaped movement
private void moveLShape(double leg1, double leg2) {
    moveForward(leg1);
    sleep(300);
    turnRelative(-90);
    sleep(300);
    moveForward(leg2);
}

// Use in sequence
moveLShape(24, 18);
```

### Using with AURORA Subsystems
```java
// Integrate shooter system
if (robotManager.getShooterSystem() != null) {
    moveToPosition(48, 36, 90);  // Move to scoring position
    sleep(500);
    
    // Warm up shooter
    robotManager.getShooterSystem().startShooter();
    sleep(1000);
    
    // Fire shots
    robotManager.getShooterSystem().autoShootSmart(3, false, 
        ShooterConfig.ShooterPreset.LONG_RANGE);
    sleep(2000);
    
    // Stop shooter
    robotManager.getShooterSystem().stopShooter();
}
```

## Best Practices

1. **Always add sleep() between movements**: Gives the robot time to stabilize
2. **Test movements individually**: Start with simple sequences and build up
3. **Use telemetry**: Monitor position and errors during testing
4. **Calibrate odometry**: Ensure robot is stationary during initialization
5. **Tune for your robot**: PID values may need adjustment based on weight, friction, etc.
6. **Use appropriate tolerances**: Tighter = more precise but slower
7. **Set initial position**: If needed, use odometry.setPosition() to set starting coordinates
8. **Keep movements smooth**: Avoid abrupt direction changes without pauses

## Safety Notes

⚠️ **Important Safety Considerations:**

- Always test autonomous code in a safe, clear area
- Start with low MAX_POWER values and increase gradually
- Keep a hand on the emergency stop
- Verify all movements before competition
- Check battery level - low voltage affects performance
- Ensure odometry pods are secure and won't detach
- Test timeout behavior (MAX_MOVE_TIME)

## Support

For issues or questions:
1. Check telemetry output during autonomous for error messages
2. Verify hardware configuration matches code
3. Review the example sequences in this guide
4. Test individual movement functions separately
5. Check GoBilda Pinpoint documentation for odometry-specific issues

## Version History

- **v1.0**: Initial release with basic movement functions and PID control
- Integrated with AURORA system manager
- Based on GoBilda Pinpoint odometry system

