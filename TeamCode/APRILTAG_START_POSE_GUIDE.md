# AprilTag-Based Starting Pose Detection Guide

## Overview

The `PedroAutonomousBuilder` now supports automatic starting pose detection using AprilTag localization. This allows your robot to determine its precise starting position at the beginning of autonomous, improving accuracy and consistency.

## Features

- **AprilTag Detection**: Automatically detects robot position using AprilTags at the start of autonomous
- **Fallback Default Pose**: Uses a configured default pose if AprilTag detection fails or times out
- **Configurable Parameters**: Customize detection timeout, confidence threshold, and more
- **Coordinate System Integration**: Automatically converts from AprilTag coordinates (center origin) to path follower coordinates (bottom-left origin)

## Coordinate Systems

The system handles two coordinate systems automatically:

### AprilTag Coordinate System
- **Origin**: Center of field (0, 0)
- **Corners**: (±72, ±72) inches
- **Used by**: FTC AprilTag detection system

### Path Follower Coordinate System (Pedro Pathing)
- **Origin**: Bottom-left corner (0, 0)
- **Corners**: (144, 144) inches
- **Used by**: Pedro Pathing path follower

The builder automatically converts between these systems, so you don't have to worry about it!

## Basic Setup

### 1. Initialize AprilTag Localizer

```java
AuroraAprilTagLocalizer aprilTagLocalizer = new AuroraAprilTagLocalizer(
    hardwareMap, 
    telemetry, 
    FieldMap.Alliance.BLUE,  // Your alliance color
    "Webcam 1"               // Webcam name from robot config
);
```

### 2. Configure Builder

```java
PedroAutonomousBuilder builder = new PedroAutonomousBuilder(follower)
    // Set the AprilTag localizer
    .withAprilTagLocalizer(aprilTagLocalizer)
    
    // Enable AprilTag detection for starting pose
    .setUseAprilTagForStart(true)
    
    // Set fallback pose if detection fails
    .setDefaultStartPose(9, 63, Math.toRadians(0))
    
    // Optional: Configure detection parameters
    .setAprilTagDetectionTimeout(3.0)       // Default: 2.0 seconds
    .setMinAprilTagConfidence(0.6);         // Default: 0.5 (50%)
```

### 3. Initialize Starting Pose

```java
// Call this after waitForStart() but before builder.start()
Pose startPose = builder.initializeStartingPose();

// The follower's starting pose is now set!
// You can verify which pose was used:
if (aprilTagLocalizer.hasValidPosition()) {
    telemetry.addLine("Using AprilTag-detected pose");
} else {
    telemetry.addLine("Using default pose (no AprilTag detected)");
}
```

## Configuration Methods

### `withAprilTagLocalizer(AuroraAprilTagLocalizer localizer)`
Set the AprilTag localizer instance to use for detection.

**Parameters:**
- `localizer`: Your initialized `AuroraAprilTagLocalizer` instance

**Returns:** Builder for method chaining

---

### `setUseAprilTagForStart(boolean enabled)`
Enable or disable AprilTag detection for starting pose.

**Parameters:**
- `enabled`: `true` to use AprilTag detection, `false` to always use default pose

**Default:** `false` (disabled)

**Returns:** Builder for method chaining

---

### `setDefaultStartPose(double x, double y, double heading)`
Set the fallback pose used when AprilTag detection fails.

**Parameters:**
- `x`: X coordinate in inches (path follower coordinate system)
- `y`: Y coordinate in inches (path follower coordinate system)
- `heading`: Heading in radians

**Default:** `(0, 0, 0)`

**Returns:** Builder for method chaining

---

### `setDefaultStartPose(Pose pose)`
Set the fallback pose using a Pose object.

**Parameters:**
- `pose`: Starting pose object

**Returns:** Builder for method chaining

---

### `setAprilTagDetectionTimeout(double timeoutSeconds)`
Set how long to wait for AprilTag detection before falling back to default pose.

**Parameters:**
- `timeoutSeconds`: Timeout duration in seconds

**Default:** `2.0` seconds

**Returns:** Builder for method chaining

---

### `setMinAprilTagConfidence(double confidence)`
Set minimum confidence required for AprilTag detection to be considered valid.

**Parameters:**
- `confidence`: Confidence threshold (0.0 - 1.0)

**Default:** `0.5` (50%)

**Returns:** Builder for method chaining

---

### `getStartingPose()`
Get the starting pose (with AprilTag detection if enabled).

**Returns:** `Pose` object with starting position (either detected or default)

**Note:** This is called automatically by `initializeStartingPose()`. You typically don't need to call it directly.

---

### `initializeStartingPose()`
Detect starting pose and set it on the follower. Call this after `waitForStart()` but before `builder.start()`.

**Returns:** The `Pose` that was set (for verification/logging)

## Complete Example

```java
@Autonomous(name = "Auto with AprilTag Start", group = "Competition")
public class MyAutonomous extends LinearOpMode {
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize follower
        Follower follower = new Follower(hardwareMap);
        
        // Initialize AprilTag localizer
        AuroraAprilTagLocalizer aprilTagLocalizer = new AuroraAprilTagLocalizer(
            hardwareMap, telemetry, FieldMap.Alliance.BLUE, "Webcam 1"
        );
        
        // Create builder with AprilTag detection
        PedroAutonomousBuilder builder = new PedroAutonomousBuilder(follower)
            .withAprilTagLocalizer(aprilTagLocalizer)
            .setUseAprilTagForStart(true)
            .setDefaultStartPose(9, 63, Math.toRadians(0))
            .setAprilTagDetectionTimeout(3.0)
            .setMinAprilTagConfidence(0.6);
        
        // Add your paths
        builder.addPath(path1)
               .addPath(path2)
               .addPath(path3);
        
        // Wait for start
        telemetry.addData("Status", "Ready - Press Play");
        telemetry.update();
        waitForStart();
        
        if (isStopRequested()) return;
        
        // Initialize starting pose (with AprilTag detection)
        telemetry.addData("Status", "Detecting position...");
        telemetry.update();
        
        Pose startPose = builder.initializeStartingPose();
        
        // Show detected pose
        telemetry.addData("Starting X", startPose.getX());
        telemetry.addData("Starting Y", startPose.getY());
        telemetry.addData("Starting Heading", Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Source", aprilTagLocalizer.hasValidPosition() ? 
            "AprilTag" : "Default");
        telemetry.update();
        
        sleep(500); // Brief pause to show pose
        
        // Start autonomous
        builder.start();
        
        // Main loop
        while (opModeIsActive() && !builder.isFinished()) {
            follower.update();
            String currentStep = builder.update();
            
            telemetry.addData("Current Step", currentStep);
            telemetry.addData("Progress", 
                builder.getCurrentStepIndex() + "/" + builder.getTotalSteps());
            telemetry.update();
        }
    }
}
```

## Best Practices

### 1. **Always Set a Default Pose**
Even when using AprilTag detection, always configure a sensible default pose. This ensures your robot can still run if AprilTag detection fails.

```java
.setDefaultStartPose(9, 63, Math.toRadians(0))  // Your actual starting position
```

### 2. **Tune Detection Timeout**
Balance between accuracy and time:
- **Shorter timeout (1-2s)**: Faster start, may miss AprilTags
- **Longer timeout (3-5s)**: Better detection, slower start

```java
.setAprilTagDetectionTimeout(2.5)  // 2.5 seconds is often a good compromise
```

### 3. **Adjust Confidence Threshold**
Based on your lighting and camera setup:
- **Lower confidence (0.3-0.5)**: More detections, less reliable
- **Higher confidence (0.6-0.8)**: Fewer detections, more reliable

```java
.setMinAprilTagConfidence(0.6)  // 60% confidence
```

### 4. **Verify in Telemetry**
Always show the detected pose and source in telemetry so you know if AprilTag detection worked:

```java
telemetry.addData("Pose Source", aprilTagLocalizer.hasValidPosition() ? 
    "AprilTag" : "Default");
telemetry.addData("Confidence", "%.1f%%", 
    aprilTagLocalizer.getPositionConfidence() * 100);
```

### 5. **Test Without AprilTags**
Make sure your autonomous still works with the default pose by testing with:
- AprilTag detection disabled
- AprilTags covered/removed
- Poor lighting conditions

## Troubleshooting

### AprilTags Not Detected
1. **Check camera connection** - Ensure webcam is properly connected
2. **Verify webcam name** - Must match your robot configuration
3. **Check lighting** - AprilTags need good lighting
4. **Increase timeout** - Try 3-5 seconds if detections are slow
5. **Lower confidence** - Try 0.4-0.5 if 0.6 is too strict

### Wrong Starting Position
1. **Check alliance color** - Must match actual alliance (RED/BLUE)
2. **Verify default pose** - Ensure coordinates are correct for your starting position
3. **Check coordinate system** - Remember path follower uses bottom-left origin

### Robot Jumps on Start
1. **Verify starting pose** - Logged pose should match physical position
2. **Check coordinate conversion** - Ensure using path follower coordinates (0-144)
3. **Increase confidence threshold** - May be using low-quality detection

## Additional Resources

- See `AprilTagStartPoseExample.java` for a complete working example
- See `PEDRO_AUTONOMOUS_BUILDER_GUIDE.md` for general builder usage
- See `AuroraAprilTagLocalizer` documentation for localization details

