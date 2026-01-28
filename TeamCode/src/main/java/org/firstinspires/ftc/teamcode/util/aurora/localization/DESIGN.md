# AURORA Fusion Localization System Design

**Version:** 1.0  
**Date:** January 2025  
**Competition Season:** DECODE 2025-2026

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture](#architecture)
3. [Design Rationale](#design-rationale)
4. [Class Structure](#class-structure)
5. [State Estimation](#state-estimation)
6. [Vision Fusion](#vision-fusion)
7. [Startup Behavior](#startup-behavior)
8. [API Documentation](#api-documentation)
9. [Integration Guide](#integration-guide)
10. [Safety & Failure Handling](#safety--failure-handling)

---

## System Overview

### Purpose

The AURORA Fusion Localization System provides **deterministic, predictable, and stable** robot pose estimation for high-speed teleop, autonomous navigation, and auto-aim turret control during FTC competition matches.

### Key Features

- **Dual Pose Outputs:**
  - `getRelativePose()` - Smooth, continuous odometry-based pose (NO jumps)
  - `getAbsolutePose()` - Vision-corrected, field-referenced pose (minimal drift)

- **Sensor Fusion:**
  - Odometry + IMU (predict step, every loop)
  - Limelight AprilTag MegaTag2 (update step, when valid)

- **Robust Vision Integration:**
  - Latency compensation (vision applied to historical state)
  - Statistical gating (Mahalanobis distance)
  - Gradual corrections (NO pose snapping)
  - Automatic rejection of bad measurements

- **Startup Support:**
  - 6 predefined field start poses
  - Graceful degradation if no AprilTag visible
  - NO spinning to localize requirement

### Why This Design is Robust

1. **Separation of Concerns:** Relative pose (drivetrain) is NEVER corrupted by vision outliers
2. **Latency Compensation:** Vision updates applied to past state, then integrated forward
3. **Statistical Validation:** Mahalanobis gating prevents gross outliers
4. **Determinism:** Same inputs → same outputs, testable, debuggable
5. **Graceful Degradation:** Works with odometry-only if vision fails

---

## Architecture

### System Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    OpMode / Subsystems                      │
│  (Drivetrain uses relative, Turret uses absolute)           │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│             FusionLocalizer (Main API)                      │
│  - getRelativePose() → odometry-continuous                  │
│  - getAbsolutePose() → vision-corrected                     │
│  - update() → calls predict + (optional) correct            │
└────────┬────────────────────────────────┬───────────────────┘
         │                                │
         ▼                                ▼
┌────────────────────┐        ┌──────────────────────────────┐
│  State Estimator   │        │   Vision Corrector           │
│  (Predict Step)    │        │   (Update Step)              │
│                    │        │                              │
│  - EKF prediction  │        │  - Latency compensation      │
│  - Odometry + IMU  │        │  - Mahalanobis gating        │
│  - Covariance grow │        │  - Gradual correction        │
└─────────┬──────────┘        └──────────┬───────────────────┘
          │                              │
          ▼                              ▼
┌─────────────────────┐        ┌──────────────────────────────┐
│ RobotPose2D (State) │        │  Vision Measurement Queue    │
│  - x, y, heading    │        │  - Timestamped poses         │
│  - covariance (3x3) │        │  - Historical buffer         │
│  - timestamp        │        └──────────────────────────────┘
└─────────────────────┘
          ▲
          │
┌─────────┴──────────────────────────────────────────────────┐
│           AuroraHardwareConfig (Hardware Access)           │
│  - Odometry (GoBildaPinpointDriver)                        │
│  - IMU (BNO055/BHI260AP)                                   │
│  - Limelight (via LimelightVisionHelper)                   │
└────────────────────────────────────────────────────────────┘
```

### Data Flow

**Every Loop (Predict):**
1. OpMode calls `fusionLocalizer.update()`
2. Reads odometry delta from hardware
3. Reads IMU heading from hardware
4. Predicts new relative pose (accumulates drift)
5. Increases covariance (uncertainty grows)

**When Vision Valid (Correct):**
1. Checks if vision measurement available
2. Validates measurement (gating checks)
3. Retrieves historical state at vision timestamp
4. Computes innovation (measurement - prediction)
5. Applies Kalman correction with latency compensation
6. Updates absolute pose estimate
7. Optionally blends relative pose (slow drift correction)

### Design Pattern

This is an **Extended Kalman Filter (EKF)** with:
- **State:** `[x, y, heading]`
- **Motion Model:** Differential drive odometry
- **Measurement Model:** Absolute 2D pose from vision
- **Covariance:** 3x3 matrix for uncertainty quantification

---

## Design Rationale

### Why Two Pose Outputs?

**Problem:** Vision measurements are:
- Absolute but noisy
- Latent (delayed)
- Occasionally invalid (no tags, bad lighting)

**Solution:** Maintain TWO pose estimates:

1. **Relative Pose (Odometry-Only)**
   - **Use For:** Drivetrain control, path following
   - **Properties:** Smooth, continuous, NO jumps
   - **Drift:** Natural accumulation (acceptable for short term)
   - **Source:** Pure odometry + IMU integration

2. **Absolute Pose (Fused)**
   - **Use For:** Turret aiming, field-relative tasks, long-term positioning
   - **Properties:** Minimal drift, vision-corrected
   - **Updates:** Gradual, latency-compensated
   - **Source:** Kalman-fused odometry + vision

### Why Latency Compensation?

**Problem:** Vision has ~50-150ms latency:
- Image capture time
- Processing time
- Network transmission
- Pipeline switching

**Without Compensation:**
- Robot moves during vision processing
- Vision pose is "where robot WAS"
- Applying directly → pose jumps backward

**With Compensation:**
- Buffer historical poses (last 500ms)
- Apply vision to historical state
- Integrate correction forward to present
- Result: smooth, accurate fusion

### Why Mahalanobis Gating?

**Problem:** Vision occasionally returns outliers:
- Tag misidentification
- Motion blur
- Occlusion artifacts

**Euclidean Distance (Bad):**
```java
if (distance > threshold) reject();  // Works poorly
```
- Fixed threshold
- Ignores uncertainty
- Too conservative OR too permissive

**Mahalanobis Distance (Good):**
```java
double d = sqrt((dx)^T * P^-1 * (dx));
if (d > 3.0) reject();  // 99.7% confidence
```
- Accounts for covariance
- Statistical confidence intervals
- Adapts to uncertainty

### Why NOT Direct Pose Reset?

**Bad Approach (Common Mistake):**
```java
// NEVER DO THIS
if (hasVision()) {
    pose = visionPose;  // ❌ CAUSES JUMPS
}
```

**Problems:**
- Discontinuous pose
- Breaks drivetrain control
- Path following unstable
- Sensor noise directly visible

**Our Approach (Kalman Correction):**
```java
// Compute innovation
dx = visionPose - predictedPose;

// Apply weighted correction
pose = predictedPose + K * dx;  // K = Kalman gain
```

**Benefits:**
- Smooth corrections
- Noise filtering
- Uncertainty-weighted
- Testable behavior

---

## Class Structure

### Package: `org.firstinspires.ftc.teamcode.util.aurora.localization`

#### Core Classes

1. **`FusionLocalizer`** (Main API)
   - Public interface for localization system
   - Manages predict/update cycle
   - Provides relative and absolute pose
   - Handles initialization with start poses

2. **`RobotPose2D`** (State Representation)
   - Pose: `[x, y, heading]`
   - Covariance: `[3x3 matrix]`
   - Timestamp: `long` (milliseconds)
   - Velocity: `[vx, vy, vheading]`

3. **`StateEstimator`** (EKF Prediction)
   - Predict step using odometry + IMU
   - Motion model integration
   - Covariance propagation
   - Process noise management

4. **`VisionCorrector`** (EKF Update)
   - Vision measurement validation
   - Mahalanobis distance gating
   - Kalman gain computation
   - Latency-compensated correction

5. **`PoseHistory`** (Historical Buffer)
   - Ring buffer of past poses
   - Timestamp-based retrieval
   - Interpolation for exact timestamps
   - 500ms history depth

6. **`MeasurementValidator`** (Gating Logic)
   - Vision data quality checks
   - Statistical outlier detection
   - Angular velocity checks
   - Freshness validation

7. **`PredefinedPoses`** (Startup Positions)
   - 6 field start positions
   - Named constants for each
   - Coordinate transformations
   - Uncertainty initialization

#### Configuration Classes

8. **`LocalizationConfig`** (System Parameters)
   - Process noise covariances
   - Measurement noise covariances
   - Gating thresholds
   - Update intervals

#### Utility Classes

9. **`Matrix3x3`** (Linear Algebra)
   - 3x3 matrix operations
   - Inversion, multiplication
   - Symmetric property enforcement

10. **`PoseInterpolator`** (Temporal Interpolation)
    - Linear interpolation for poses
    - Timestamp alignment
    - Extrapolation guards

---

## State Estimation

### State Vector

```
x = [x, y, heading]^T

where:
  x       = robot X position (mm, field frame)
  y       = robot Y position (mm, field frame)
  heading = robot heading (radians, CCW from +X)
```

### Motion Model (Predict Step)

**Differential Drive Kinematics:**

```
x(k+1) = x(k) + dx
y(k+1) = y(k) + dy
heading(k+1) = heading_imu(k+1)  // Direct from IMU
```

Where odometry provides:
```
dx = delta_x * cos(heading) - delta_y * sin(heading)
dy = delta_x * sin(heading) + delta_y * cos(heading)
```

**Covariance Propagation:**

```
P(k+1) = F * P(k) * F^T + Q

where:
  F = Jacobian of motion model (identity for simple model)
  Q = Process noise covariance (grows uncertainty)
```

**Process Noise (Q):**
```
Q = diag([sigma_x^2, sigma_y^2, sigma_heading^2])

Tuned values:
  sigma_x       = 5.0 mm/update
  sigma_y       = 5.0 mm/update  
  sigma_heading = 0.02 rad/update
```

### Measurement Model (Update Step)

**Vision provides:**
```
z = [x_vision, y_vision, heading_vision]^T
```

**Measurement Equation:**
```
z = H * x + v

where:
  H = Measurement Jacobian (identity for direct measurement)
  v ~ N(0, R) = measurement noise
```

**Measurement Noise (R):**
```
R = diag([sigma_x_vision^2, sigma_y_vision^2, sigma_heading_vision^2])

Tuned values:
  sigma_x_vision       = 50.0 mm
  sigma_y_vision       = 50.0 mm
  sigma_heading_vision = 0.1 rad
```

### Kalman Update Equations

**Innovation:**
```
y = z - H*x_predicted
```

**Innovation Covariance:**
```
S = H * P * H^T + R
```

**Kalman Gain:**
```
K = P * H^T * S^-1
```

**State Update:**
```
x_corrected = x_predicted + K * y
```

**Covariance Update:**
```
P_corrected = (I - K*H) * P_predicted
```

---

## Vision Fusion

### Measurement Validation Pipeline

**Step 1: Hardware Checks**
- Limelight initialized? (initialization check)
- Target detected? (`hasTarget()`)
- Data fresh? (`isDataFresh()`)

**Step 2: Quality Checks**
- Data quality good? (consecutive reads)
- Robot velocity low? (< 500 mm/s)
- Angular velocity low? (< 30 deg/s)

**Step 3: Geometric Checks**
- Distance to target reasonable? (< 4000mm)
- Multiple tags visible? (MegaTag2 advantage)

**Step 4: Statistical Gating**
- Compute Mahalanobis distance
- Check against threshold (chi-squared, 99.7% = 3.0)
- Reject if innovation too large

**Step 5: Freshness Check**
- Vision timestamp not too old? (< 200ms)
- Vision timestamp not in future? (clock sync)

**Pseudo-code:**
```java
public boolean validateMeasurement(
    RobotPose2D predicted,
    RobotPose2D measurement,
    Matrix3x3 covariance,
    long timestamp
) {
    // Hardware checks
    if (!hardwareOk()) return false;
    
    // Quality checks
    if (velocity > maxVelocity) return false;
    if (angularVel > maxAngularVel) return false;
    
    // Geometric checks
    if (distance > maxDistance) return false;
    
    // Statistical gating (Mahalanobis)
    double d = computeMahalanobis(predicted, measurement, covariance);
    if (d > 3.0) return false;  // 99.7% confidence
    
    // Freshness check
    long age = currentTime - timestamp;
    if (age > 200 || age < -10) return false;
    
    return true;  // ACCEPT measurement
}
```

### Latency Compensation Strategy

**Problem:**
- Vision captured at time `t_capture`
- Processed and received at time `t_now`
- Robot has moved during `latency = t_now - t_capture`

**Solution (Historical State Correction):**

1. **Buffer Poses:**
   ```java
   // Maintain 500ms history of poses
   PoseHistory history = new PoseHistory(500);
   
   // Every update, save pose
   history.add(currentPose, timestamp);
   ```

2. **Retrieve Historical State:**
   ```java
   // Get pose at vision capture time
   RobotPose2D historicalPose = history.get(t_capture);
   ```

3. **Apply Correction to Historical State:**
   ```java
   // Compute innovation at historical time
   dx = visionPose - historicalPose;
   
   // Compute Kalman gain
   K = computeKalmanGain(historicalCovariance, R);
   
   // Correct historical state
   correctedHistorical = historicalPose + K * dx;
   ```

4. **Propagate Correction Forward:**
   ```java
   // Integrate correction to present
   // Method 1: Apply same delta to current
   currentPose = currentPose + (correctedHistorical - historicalPose);
   
   // Method 2: Re-integrate odometry from corrected historical
   // (more accurate but more complex)
   ```

**Implementation Note:** We use Method 1 (delta propagation) for:
- Simplicity
- Real-time performance
- Acceptable accuracy for latencies < 200ms

### Gradual Correction (Alpha Blending)

**Problem:**
- Large corrections can still cause visible jumps
- Want smooth transition from odometry to vision

**Solution:**
```java
// Apply correction gradually over multiple updates
double alpha = 0.1;  // 10% per update

// Blend odometry and corrected pose
poseFused = (1 - alpha) * poseOdometry + alpha * poseCorrected;

// Result: Correction spread over ~10 updates (~200ms)
```

**Benefit:**
- No visible jumps
- Smooth convergence
- Tunable responsiveness

### Update Rate Strategy

**Vision updates:** 2-5 Hz (limited by Limelight)
**Odometry updates:** 50-100 Hz (continuous)

**Strategy:**
- Predict every loop (50-100 Hz)
- Correct only when new vision available (2-5 Hz)
- Absolute pose smoothed via alpha blending

**Result:**
- Relative pose: always smooth, never waits for vision
- Absolute pose: gradually converges to vision-corrected state

---

## Startup Behavior

### Predefined Start Poses

**Field Positions (6 total):**

```java
public enum StartPosition {
    // Audience side (closer to driver station)
    RED_AUDIENCE_LEFT,      // Red alliance, left of audience wall
    RED_AUDIENCE_RIGHT,     // Red alliance, right of audience wall
    BLUE_AUDIENCE_LEFT,     // Blue alliance, left of audience wall
    BLUE_AUDIENCE_RIGHT,    // Blue alliance, right of audience wall
    
    // Net zone side (opposite side)
    RED_NET_ZONE,           // Red alliance, net zone
    BLUE_NET_ZONE           // Blue alliance, net zone
}
```

**Coordinate System:**
- Origin: Field center
- +X: Toward red alliance
- +Y: Toward audience wall
- Heading: 0 = facing +X (CCW positive)

**Pose Values (example, adjust for actual field):**
```java
RED_AUDIENCE_LEFT   = (1500mm, 1500mm, 0°)
RED_AUDIENCE_RIGHT  = (1500mm, -1500mm, 0°)
BLUE_AUDIENCE_LEFT  = (-1500mm, 1500mm, 180°)
BLUE_AUDIENCE_RIGHT = (-1500mm, -1500mm, 180°)
RED_NET_ZONE        = (1500mm, 0mm, 0°)
BLUE_NET_ZONE       = (-1500mm, 0mm, 180°)
```

### Initialization Sequence

**OpMode Startup:**

```java
// Create localizer with start pose
FusionLocalizer localizer = new FusionLocalizer(
    hardware,
    PredefinedPoses.RED_AUDIENCE_LEFT
);

// OR let operator select via gamepad
StartPosition selected = selectStartPosition(gamepad1);
FusionLocalizer localizer = new FusionLocalizer(hardware, selected);
```

**Initial Uncertainty:**
```
// Human placement accuracy
P(0) = diag([50mm, 50mm, 10deg])

// Uncertainty grows until first vision correction
```

### Behavior Without Vision at Start

**Scenario:** Robot starts, no AprilTag visible

**System Response:**
1. Initialize with predefined pose
2. Track relative pose via odometry
3. Increase uncertainty over time
4. When first AprilTag seen:
   - Validate measurement heavily (large Mahalanobis ok initially)
   - Apply large correction if validated
   - Reduce uncertainty after correction
5. Continue normal operation

**Key:** NO spinning required - system degrades gracefully

### Vision Bootstrapping

**First Vision Update (Special Case):**

```java
if (firstVisionUpdate) {
    // Relax gating threshold for initial correction
    mahalanobisThreshold = 5.0;  // vs 3.0 normal
    
    // Apply larger correction weight
    alpha = 0.5;  // vs 0.1 normal
    
    // Reduce uncertainty significantly
    P = P * 0.2;
    
    firstVisionUpdate = false;
}
```

**Rationale:**
- Initial pose has high uncertainty
- First vision correction most important
- Accept larger innovation initially
- Tighten gating after first update

---

## API Documentation

### Main API: `FusionLocalizer`

#### Constructor

```java
/**
 * Create fusion localizer with origin start
 */
public FusionLocalizer(AuroraHardwareConfig hardware);

/**
 * Create fusion localizer with predefined start pose
 */
public FusionLocalizer(
    AuroraHardwareConfig hardware,
    PredefinedPoses.StartPosition startPosition
);

/**
 * Create fusion localizer with custom start pose
 */
public FusionLocalizer(
    AuroraHardwareConfig hardware,
    RobotPose2D startPose
);
```

#### Core Methods

```java
/**
 * Update localization (call every loop)
 * Performs predict step and optional correct step
 */
public void update();

/**
 * Get smooth relative pose (odometry-based)
 * Use for: drivetrain control, path following
 * Properties: No jumps, continuous, drifts naturally
 */
public RobotPose2D getRelativePose();

/**
 * Get stable absolute pose (vision-corrected)
 * Use for: turret aiming, field-relative tasks
 * Properties: Minimal drift, gradual corrections
 */
public RobotPose2D getAbsolutePose();

/**
 * Get current heading (fused)
 */
public double getHeading(AngleUnit unit);

/**
 * Get current velocity
 */
public double getVelocityX(DistanceUnit unit);
public double getVelocityY(DistanceUnit unit);
public double getHeadingVelocity(AngleUnit unit);
```

#### Status Methods

```java
/**
 * Check if vision correction is active
 */
public boolean isVisionActive();

/**
 * Get time since last vision update
 */
public long getTimeSinceLastVisionUpdate();

/**
 * Get current pose uncertainty (trace of covariance)
 */
public double getPoseUncertainty();

/**
 * Check if system is initialized
 */
public boolean isInitialized();
```

#### Debug/Telemetry

```java
/**
 * Add comprehensive telemetry
 */
public void addTelemetry(Telemetry telemetry);

/**
 * Get detailed status string
 */
public String getStatusString();
```

### State Representation: `RobotPose2D`

```java
public class RobotPose2D {
    // Pose state
    public double x;            // mm, field frame
    public double y;            // mm, field frame
    public double heading;      // radians, CCW from +X
    
    // Uncertainty
    public Matrix3x3 covariance;  // 3x3 covariance matrix
    
    // Velocity
    public double vx;           // mm/s
    public double vy;           // mm/s
    public double vheading;     // rad/s
    
    // Timestamp
    public long timestamp;      // milliseconds
    
    // Convenience methods
    public Pose2D toPose2D();
    public double getX(DistanceUnit unit);
    public double getY(DistanceUnit unit);
    public double getHeading(AngleUnit unit);
}
```

---

## Integration Guide

### Basic Usage (TeleOp)

```java
@TeleOp(name = "FusionLocalizer Test")
public class FusionLocalizerTestOpMode extends LinearOpMode {
    
    private AuroraHardwareConfig hardware;
    private FusionLocalizer localizer;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();
        
        // Initialize localizer with start pose
        localizer = new FusionLocalizer(
            hardware,
            PredefinedPoses.RED_AUDIENCE_LEFT
        );
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update localizer (predict + optional correct)
            localizer.update();
            
            // Get poses
            RobotPose2D relativePose = localizer.getRelativePose();
            RobotPose2D absolutePose = localizer.getAbsolutePose();
            
            // Use relative pose for drivetrain
            drive.driveFieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                relativePose.heading  // Smooth, no jumps
            );
            
            // Use absolute pose for turret
            turret.aimAt(
                targetX,
                targetY,
                absolutePose.x,  // Vision-corrected
                absolutePose.y
            );
            
            // Telemetry
            localizer.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
```

### Autonomous Usage

```java
@Autonomous(name = "Vision-Guided Auto")
public class VisionAutoOpMode extends LinearOpMode {
    
    private FusionLocalizer localizer;
    
    @Override
    public void runOpMode() {
        // Initialize
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();
        
        // Start from known position
        localizer = new FusionLocalizer(
            hardware,
            PredefinedPoses.BLUE_NET_ZONE
        );
        
        waitForStart();
        
        // Navigate to waypoint using absolute pose
        while (opModeIsActive() && !atWaypoint()) {
            localizer.update();
            
            RobotPose2D pose = localizer.getAbsolutePose();
            
            // Compute control to waypoint
            double distance = Math.hypot(
                waypointX - pose.x,
                waypointY - pose.y
            );
            double angle = Math.atan2(
                waypointY - pose.y,
                waypointX - pose.x
            );
            
            // Drive toward waypoint
            drive.driveTo(distance, angle, pose.heading);
            
            telemetry.addData("Distance to waypoint", "%.1f mm", distance);
            telemetry.update();
        }
        
        // Arrived at waypoint
        drive.stop();
    }
}
```

### Integration with Existing Subsystems

**Drivetrain:**
```java
// Use relative pose for smooth control
RobotPose2D pose = localizer.getRelativePose();
drive.setFieldCentricHeading(pose.heading);
```

**Turret:**
```java
// Use absolute pose for field-relative aiming
RobotPose2D pose = localizer.getAbsolutePose();
turret.aimAt(targetX, targetY, pose.x, pose.y, pose.heading);
```

**Autonomous Path Follower:**
```java
// Use absolute pose for waypoint navigation
RobotPose2D pose = localizer.getAbsolutePose();
pathFollower.update(pose, targetPath);
```

---

## Safety & Failure Handling

### Failure Modes

**1. Odometry Failure**
- **Detection:** No odometry updates, status check
- **Response:** Disable localization, telemetry warning
- **Fallback:** IMU-only heading tracking

**2. Vision Failure**
- **Detection:** No targets, consecutive rejections
- **Response:** Continue odometry-only operation
- **Impact:** Absolute pose drifts, relative pose unaffected

**3. Measurement Outliers**
- **Detection:** Mahalanobis gating
- **Response:** Reject measurement, log event
- **Impact:** Single rejection has no effect

**4. Covariance Divergence**
- **Detection:** Trace(P) > threshold
- **Response:** Cap covariance growth, trigger warning
- **Fallback:** Reset covariance after valid vision

### Safety Checks

**Angular Velocity Limit:**
```java
// Reject vision if spinning too fast
if (Math.abs(vheading) > Math.toRadians(30)) {
    rejectVision("Angular velocity too high");
}
```

**Linear Velocity Limit:**
```java
// Reject vision if moving too fast
double velocity = Math.hypot(vx, vy);
if (velocity > 500) {  // mm/s
    rejectVision("Linear velocity too high");
}
```

**Innovation Magnitude:**
```java
// Reject vision if correction too large (safety)
double innovationNorm = Math.hypot(dx, dy);
if (innovationNorm > 1000) {  // 1 meter
    rejectVision("Innovation too large");
}
```

### Watchdog Timers

**Vision Timeout:**
```java
// If no vision for 30 seconds, increase uncertainty
if (timeSinceLastVision > 30000) {
    increaseUncertainty();
    telemetry.addLine("⚠️ No vision for 30s");
}
```

**Odometry Timeout:**
```java
// If odometry stops updating, disable system
if (timeSinceLastOdometry > 1000) {
    disable();
    telemetry.addLine("❌ Odometry timeout");
}
```

### Telemetry & Debugging

**Standard Telemetry:**
```
═══ Fusion Localization ═══
Relative Pose: (1234.5, -567.8, 45.2°)
Absolute Pose: (1230.1, -565.3, 45.0°)
Vision Status: Active (123ms ago)
Uncertainty: 52.3 mm

Odometry: ✅ 87.3 Hz
IMU: ✅ Ready
Vision: ✅ 3 tags, Fresh

Rejected: 2 (high velocity)
Accepted: 47
```

**Debug Mode (Detailed):**
```
Covariance Trace: 2847.3
Mahalanobis Last: 1.87
Innovation: (4.2, -3.1, 0.02)
Kalman Gain: [[0.15], [0.18], [0.20]]
Process Noise: (5.0, 5.0, 0.02)
```

### Error Recovery

**Automatic Recovery:**
1. **Odometry glitch:** Continue with last valid data
2. **Vision outlier:** Reject and continue
3. **High uncertainty:** Wait for valid vision

**Manual Recovery (OpMode):**
```java
// Reset to known position
if (gamepad1.start) {
    localizer.resetToStartPose(
        PredefinedPoses.RED_AUDIENCE_LEFT
    );
    telemetry.addLine("✅ Position reset");
}
```

---

## Performance Characteristics

### Computational Cost

- **Predict step:** ~0.5 ms (matrix ops, odometry read)
- **Correct step:** ~2.0 ms (Kalman update, gating)
- **Total per loop:** ~0.5-2.5 ms depending on vision

**Target:** < 5% of loop time (50Hz loop = 20ms available)

### Memory Usage

- **Pose history buffer:** ~10 KB (500 poses * 20 bytes)
- **State estimator:** ~1 KB (matrices, state)
- **Total:** < 15 KB (negligible)

### Accuracy (Expected)

**Relative Pose (Odometry):**
- Drift: ~1-2% of distance traveled
- Heading: ~1-2° per 10m traveled
- Short-term accuracy: < 10mm

**Absolute Pose (Fused):**
- With vision: < 50mm RMS error
- Heading: < 5° RMS error
- Long-term drift: < 20mm/minute

### Latency

- **Odometry:** < 5ms (direct read)
- **Vision:** 50-150ms (Limelight processing)
- **Output:** < 10ms end-to-end (with compensation)

---

## Testing Strategy

### Unit Tests

1. **Matrix3x3:** Inversion, multiplication, symmetry
2. **PoseHistory:** Add, retrieve, interpolation
3. **MeasurementValidator:** Gating logic, edge cases
4. **StateEstimator:** Prediction accuracy
5. **VisionCorrector:** Correction magnitude, latency compensation

### Integration Tests

1. **Odometry-only:** Straight line, rotation, circle
2. **Vision-only:** Static pose, measurement rejection
3. **Fusion:** Combined odometry + vision

### Hardware Tests

1. **Static test:** Robot stationary, vision updates
2. **Motion test:** Drive patterns, check pose continuity
3. **Vision loss:** Drive without vision, check drift
4. **Vision recovery:** Regain vision, check convergence

---

## Future Enhancements

### Phase 2 (Post-Competition)

1. **IMU Fusion:** Full 3-state IMU integration (not just heading)
2. **Multi-Tag Triangulation:** Use multiple tags simultaneously
3. **Adaptive Tuning:** Auto-tune Q and R based on conditions
4. **Wheel Slip Detection:** Detect and compensate for slip

### Phase 3 (Advanced)

1. **Unscented Kalman Filter:** Better nonlinear handling
2. **Multi-Hypothesis Tracking:** Handle ambiguous situations
3. **SLAM Integration:** Build map while localizing
4. **Field Element Recognition:** Use field markers beyond tags

---

## Glossary

- **EKF:** Extended Kalman Filter
- **Mahalanobis Distance:** Statistical distance metric accounting for covariance
- **Innovation:** Difference between measurement and prediction
- **Kalman Gain:** Weighting factor for correction
- **Process Noise:** Uncertainty added during prediction
- **Measurement Noise:** Uncertainty in sensor reading
- **Covariance:** Uncertainty matrix (3x3 for pose)
- **Latency Compensation:** Correcting for delayed measurements
- **MegaTag2:** Limelight multi-tag pose estimation mode

---

## References

- FTC SDK Documentation: https://ftc-docs.firstinspires.org/
- Limelight Documentation: https://docs.limelightvision.io/
- Probabilistic Robotics (Thrun et al.)
- State Estimation for Robotics (Barfoot)

---

**End of Design Document**
