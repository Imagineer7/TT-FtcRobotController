# AURORA Fusion Localization System

**Version:** 1.0  
**Competition Season:** DECODE 2025-2026  
**Last Updated:** January 2025

---

## Quick Start

### Basic Usage (TeleOp)

```java
@TeleOp(name = "My TeleOp")
public class MyTeleOp extends LinearOpMode {
    
    private AuroraHardwareConfig hardware;
    private FusionLocalizer localizer;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();
        
        // Initialize localizer at origin
        localizer = new FusionLocalizer(hardware);
        
        // OR use predefined start position
        // localizer = new FusionLocalizer(
        //     hardware,
        //     PredefinedPoses.StartPosition.RED_AUDIENCE_LEFT
        // );
        
        waitForStart();
        
        while (opModeIsActive()) {
            // CRITICAL: Update localizer every loop
            localizer.update();
            
            // Get poses
            RobotPose2D relativePose = localizer.getRelativePose();  // For drivetrain
            RobotPose2D absolutePose = localizer.getAbsolutePose();  // For turret
            
            // Use relative pose for smooth drivetrain control
            drive.driveFieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                relativePose.heading  // Smooth, no jumps
            );
            
            // Use absolute pose for turret aiming
            turret.aimAt(
                targetX,
                targetY,
                absolutePose.x,  // Vision-corrected, minimal drift
                absolutePose.y
            );
            
            // Add telemetry
            localizer.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
```

---

## What is This?

The **Fusion Localization System** provides **two simultaneous pose estimates** for your FTC robot:

1. **Relative Pose (Odometry-Based)**
   - Smooth, continuous position tracking
   - **NO jumps or discontinuities**
   - Based on wheel encoders + IMU
   - Accumulates drift naturally
   - **Use for:** Drivetrain control, path following

2. **Absolute Pose (Vision-Corrected)**
   - Stable, field-referenced position
   - **Minimal drift** over time
   - Corrected using Limelight AprilTag vision
   - Gradual corrections (no snapping)
   - **Use for:** Turret auto-aim, field-relative tasks, autonomous waypoint navigation

### Why Two Poses?

**Problem:** Vision measurements are:
- Absolute but noisy
- Delayed (50-150ms latency)
- Occasionally invalid (no tags visible)

**Solution:** Maintain TWO estimates:
- **Relative** stays smooth for drivetrain (never waits for vision)
- **Absolute** converges to vision-corrected truth for long-term accuracy

---

## Key Features

### ✅ Extended Kalman Filter (EKF)
- Rigorous mathematical fusion of odometry + vision
- Uncertainty quantification via covariance matrices
- Optimal Kalman gain computation

### ✅ Latency Compensation
- Vision measurements applied to **historical** robot poses
- Correction propagated forward to present
- No backward time jumps

### ✅ Robust Vision Gating
- **6-level validation** before accepting measurements:
  1. Hardware checks (sensor operational)
  2. Quality checks (data freshness, consecutive failures)
  3. Motion checks (velocity, angular velocity limits)
  4. Geometric checks (distance to tag)
  5. Statistical checks (**Mahalanobis distance** gating)
  6. Safety checks (innovation magnitude)

### ✅ Gradual Corrections
- Alpha blending prevents pose snapping
- Configurable convergence rate (default: 10% per update)
- Smooth transition from odometry to vision

### ✅ Predefined Start Poses
- 6 field positions (RED/BLUE, AUDIENCE/NET_ZONE)
- Gamepad selection during initialization
- Accurate placement without spinning

### ✅ Comprehensive Telemetry
- Standard view (poses, vision status)
- Debug view (innovation, Mahalanobis distance, covariance)
- Statistics view (acceptance rate, pose divergence)

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────┐
│                  OpMode / Subsystems                   │
│  (Drivetrain uses relative, Turret uses absolute)     │
└─────────────────────┬──────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────┐
│              FusionLocalizer (Main API)                 │
│  • getRelativePose() → odometry-continuous             │
│  • getAbsolutePose() → vision-corrected                │
│  • update() → predict + (optional) correct             │
└───────┬────────────────────────┬────────────────────────┘
        │                        │
        ▼                        ▼
┌──────────────────┐    ┌──────────────────────────┐
│ StateEstimator   │    │   VisionCorrector        │
│ (Predict Step)   │    │   (Update Step)          │
│                  │    │                          │
│ • Odometry + IMU │    │ • Kalman gain            │
│ • Covariance ↑   │    │ • Latency compensation   │
└──────────────────┘    └──────────────────────────┘
```

---

## Classes

### Core API

**`FusionLocalizer`** - Main interface
- `update()` - Call every loop
- `getRelativePose()` - Get smooth odometry pose
- `getAbsolutePose()` - Get vision-corrected pose
- `addTelemetry()` - Display status

### State Representation

**`RobotPose2D`** - Pose with uncertainty
- Position: `x, y` (mm)
- Heading: `heading` (radians)
- Velocity: `vx, vy, vheading`
- Covariance: `3×3 matrix`

### Configuration

**`LocalizationConfig`** - Tunable parameters
- Process noise: `Q` (odometry drift)
- Measurement noise: `R` (vision accuracy)
- Gating thresholds: Mahalanobis distance
- Correction rates: Alpha blending factors

**`PredefinedPoses`** - Field start positions
- `RED_AUDIENCE_LEFT/RIGHT`
- `BLUE_AUDIENCE_LEFT/RIGHT`
- `RED_NET_ZONE`, `BLUE_NET_ZONE`

### Internal Components

**`StateEstimator`** - EKF prediction
- Odometry integration
- IMU heading fusion
- Covariance propagation

**`VisionCorrector`** - EKF update
- Kalman gain computation
- Historical state correction
- Innovation tracking

**`MeasurementValidator`** - Vision gating
- Multi-level validation
- Statistical outlier rejection
- Rejection reason tracking

**`PoseHistory`** - Temporal buffer
- 500ms pose history
- Timestamp-based retrieval
- Linear interpolation

**`Matrix3x3`** - Linear algebra
- Matrix operations (add, multiply, invert)
- Covariance manipulation

---

## Configuration & Tuning

### Conservative Config (Trust Vision Less)

```java
LocalizationConfig config = LocalizationConfig.conservative();
// Process noise: LOWER (trust odometry more)
// Measurement noise: HIGHER (trust vision less)
// Correction rate: SLOWER (0.05 vs 0.1)
// Gating: MORE RESTRICTIVE (threshold 2.0 vs 3.0)

FusionLocalizer localizer = new FusionLocalizer(hardware, startPose, config);
```

**Use when:**
- Poor vision conditions (lighting, distance)
- Many vision rejections occurring
- Prefer smooth over accurate

### Aggressive Config (Trust Vision More)

```java
LocalizationConfig config = LocalizationConfig.aggressive();
// Process noise: HIGHER (trust odometry less)
// Measurement noise: LOWER (trust vision more)
// Correction rate: FASTER (0.2 vs 0.1)
// Gating: MORE PERMISSIVE (threshold 4.0 vs 3.0)

FusionLocalizer localizer = new FusionLocalizer(hardware, startPose, config);
```

**Use when:**
- Excellent vision conditions (close, well-lit)
- High vision acceptance rate
- Need fast convergence

### Custom Tuning

```java
LocalizationConfig config = new LocalizationConfig();

// Process noise (how much odometry drifts per update)
config.processNoiseX = 5.0;       // mm/update
config.processNoiseY = 5.0;       // mm/update
config.processNoiseHeading = 0.02; // rad/update

// Measurement noise (vision accuracy)
config.measurementNoiseX = 50.0;       // mm
config.measurementNoiseY = 50.0;       // mm
config.measurementNoiseHeading = 0.1;  // rad

// Correction rate (how fast to apply corrections)
config.correctionAlpha = 0.1;  // 10% per update

// Gating (measurement rejection threshold)
config.mahalanobisThreshold = 3.0;  // 99.7% confidence

// Motion limits (reject vision if moving too fast)
config.maxVelocityForVision = 500.0;         // mm/s
config.maxAngularVelocityForVision = 0.52;   // rad/s (~30°/s)
```

**Tuning Tips:**
1. Start with defaults
2. If vision rejected too often → increase `mahalanobisThreshold`
3. If odometry drifts too much → increase `processNoise*`
4. If corrections too slow → increase `correctionAlpha`
5. If corrections too jerky → decrease `correctionAlpha`

---

## Advanced Usage

### Autonomous with Waypoint Navigation

```java
@Autonomous(name = "Vision-Guided Auto")
public class VisionAutoOpMode extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();
        
        // Start from known position
        FusionLocalizer localizer = new FusionLocalizer(
            hardware,
            PredefinedPoses.StartPosition.BLUE_NET_ZONE
        );
        
        waitForStart();
        
        // Navigate to waypoint (use absolute pose)
        double waypointX = 1500;  // mm
        double waypointY = 1000;  // mm
        
        while (opModeIsActive() && !atWaypoint(localizer, waypointX, waypointY)) {
            localizer.update();
            
            RobotPose2D pose = localizer.getAbsolutePose();
            
            // Compute vector to waypoint
            double dx = waypointX - pose.x;
            double dy = waypointY - pose.y;
            double distance = Math.hypot(dx, dy);
            double targetAngle = Math.atan2(dy, dx);
            
            // PID control to waypoint
            drive.driveToTarget(distance, targetAngle, pose.heading);
            
            telemetry.addData("Distance", "%.1f mm", distance);
            telemetry.update();
        }
        
        drive.stop();
    }
    
    private boolean atWaypoint(FusionLocalizer localizer, double x, double y) {
        RobotPose2D pose = localizer.getAbsolutePose();
        double distance = Math.hypot(x - pose.x, y - pose.y);
        return distance < 50;  // 50mm tolerance
    }
}
```

### Manual Position Reset

```java
// Reset to origin
if (gamepad1.start) {
    localizer.reset(new RobotPose2D(0, 0, 0));
}

// Reset to predefined position
if (gamepad1.back) {
    localizer.reset(PredefinedPoses.StartPosition.RED_AUDIENCE_LEFT);
}
```

### Accessing Uncertainty

```java
// Check if pose is reliable
double uncertainty = localizer.getPoseUncertainty();
if (uncertainty > 200) {  // 200mm = ~8 inches
    telemetry.addLine("⚠️ High uncertainty - waiting for vision");
}

// Check vision status
if (localizer.isVisionActive()) {
    telemetry.addLine("✅ Vision corrections active");
} else {
    long age = localizer.getTimeSinceLastVisionUpdate();
    telemetry.addData("⚠️ No vision for", "%d ms", age);
}
```

---

## Telemetry

### Standard View

```
═══ Fusion Localization ═══
Relative: (1234.5, -567.8) mm, 45.2°
Absolute: (1230.1, -565.3) mm, 45.0°
Uncertainty: 52.3 mm
Vision: ✅ Active (123 ms ago)
Accepted/Rejected: 47 / 2 (96%)

Odometry: ✅ 87.3 Hz
Limelight: ✅ Target
```

### Debug View

```
═══ Debug Info ═══
Innovation: (4.2, -3.1) mm
Mahalanobis: 1.87
Covariance Trace: 2847.3
History Size: 50 poses
History Span: 498 ms
Velocity: 234.5 mm/s
Angular Vel: 0.15 rad/s
```

---

## Troubleshooting

### Problem: Vision Always Rejected

**Symptoms:**
- `Rejected` count much higher than `Accepted`
- Last reject reason: "Mahalanobis distance too large"

**Solutions:**
1. Increase gating threshold:
   ```java
   config.mahalanobisThreshold = 4.0;  // More permissive
   ```
2. Reduce measurement noise (trust vision more):
   ```java
   config.measurementNoiseX = 30.0;  // Lower = trust more
   config.measurementNoiseY = 30.0;
   ```
3. Check robot is stationary during first measurement
4. Verify AprilTag is close (< 4 meters)

### Problem: Relative/Absolute Poses Diverging

**Symptoms:**
- Relative and absolute pose differ by > 200mm
- Poses don't converge over time

**Solutions:**
1. Enable relative pose blending:
   ```java
   config.blendRelativePose = true;
   config.relativePoseBlendAlpha = 0.02;  // Slow drift correction
   ```
2. Increase correction rate (if vision reliable):
   ```java
   config.correctionAlpha = 0.2;  // Faster convergence
   ```

### Problem: Pose Jumps/Jitters

**Symptoms:**
- Visible discontinuities in position
- Drivetrain control unstable

**Solutions:**
1. Reduce correction rate:
   ```java
   config.correctionAlpha = 0.05;  // Slower, smoother
   ```
2. Use ONLY `getRelativePose()` for drivetrain
3. Check `maxVelocityForVision` - increase if rejecting too much

### Problem: No Vision Updates

**Symptoms:**
- `Vision: ❌ INACTIVE`
- `isVisionActive()` returns false

**Checks:**
1. Limelight initialized? Check hardware config
2. AprilTags visible? Point camera at tags
3. Distance to tags < 4 meters?
4. Robot moving too fast? Reduce `maxVelocityForVision`
5. Check Limelight logs for pipeline issues

---

## Performance

### Computational Cost
- **Predict step:** ~0.5 ms (every loop)
- **Correct step:** ~2.0 ms (when vision valid)
- **Total:** < 5% of 50Hz loop time (20ms)

### Memory Usage
- **Pose history:** ~10 KB (500ms @ 100Hz)
- **State estimator:** ~1 KB (matrices, state)
- **Total:** < 15 KB (negligible)

### Accuracy (Expected)
- **Relative pose:** < 10mm short-term, 1-2% drift
- **Absolute pose:** < 50mm RMS, < 5° heading RMS
- **Long-term drift:** < 20mm/minute (with vision)

---

## Testing

### Test OpMode

Run `Test: Fusion Localization` from Driver Station:

1. **Init:** Select start position with D-Pad Up/Down
2. **Start:** System begins tracking
3. **Controls:**
   - A: Reset to origin
   - B: Reset to start position
   - X: Toggle debug telemetry
   - Y: Toggle statistics view

### Validation Checklist

- [ ] System initializes without errors
- [ ] Relative pose tracks odometry smoothly
- [ ] Absolute pose converges to vision when tags visible
- [ ] Vision acceptance rate > 80%
- [ ] No pose jumps during corrections
- [ ] Uncertainty decreases after vision correction
- [ ] Manual reset works correctly

---

## Mathematical Details

### State Vector
```
x = [x, y, heading]^T
```

### Motion Model (Predict)
```
x(k+1) = x(k) + dx
y(k+1) = y(k) + dy
heading(k+1) = heading_imu(k+1)  # Direct from IMU
```

### Covariance Prediction
```
P(k+1) = P(k) + Q
```

### Measurement Model (Update)
```
z = H * x + v

where H = I (direct measurement)
      v ~ N(0, R) (measurement noise)
```

### Kalman Update
```
Innovation: y = z - x_pred
Innovation Covariance: S = P + R
Kalman Gain: K = P * S^-1
State Update: x_new = x_pred + K * y
Covariance Update: P_new = (I - K) * P
```

### Mahalanobis Distance
```
d = sqrt((y)^T * S^-1 * (y))

Reject if: d > threshold (typically 3.0 for 99.7% confidence)
```

---

## References

- **FTC SDK:** https://ftc-docs.firstinspires.org/
- **Limelight:** https://docs.limelightvision.io/
- **Kalman Filters:** *Probabilistic Robotics* (Thrun et al.)
- **State Estimation:** *State Estimation for Robotics* (Barfoot)

---

## Support

For questions or issues:
1. Check [DESIGN.md](DESIGN.md) for detailed architecture
2. Review test OpMode for usage examples
3. Check troubleshooting section above
4. Enable debug telemetry to diagnose issues

---

**End of README**
