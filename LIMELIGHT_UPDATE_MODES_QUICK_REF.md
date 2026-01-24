# Limelight Update Modes - Quick Reference

## Three Modes Available

```java
Localization.LimelightUpdateMode.AUTOMATIC   // Auto-update (default)
Localization.LimelightUpdateMode.MANUAL      // Manual trigger only
Localization.LimelightUpdateMode.DISABLED    // No Limelight updates
```

## Quick Setup

### AUTOMATIC Mode (Default)
```java
Localization loc = new Localization(hardwareMap);
// Already in AUTOMATIC mode by default

while (opModeIsActive()) {
    loc.update();  // Auto-updates when conditions met
}
```

### MANUAL Mode
```java
Localization loc = new Localization(hardwareMap);
loc.setLimelightUpdateMode(Localization.LimelightUpdateMode.MANUAL);

while (opModeIsActive()) {
    loc.update();  // Only updates odometry
    
    // Manually trigger Limelight update
    if (gamepad1.a) {
        boolean success = loc.updateWithLimelight();
        if (success) {
            // Updated successfully
        } else {
            // Conditions not met
        }
    }
}
```

### DISABLED Mode
```java
Localization loc = new Localization(hardwareMap);
loc.setLimelightUpdateMode(Localization.LimelightUpdateMode.DISABLED);

while (opModeIsActive()) {
    loc.update();  // Odometry only, no Limelight
}
```

## API Reference

### Set Mode
```java
localization.setLimelightUpdateMode(mode);
```

### Get Mode
```java
Localization.LimelightUpdateMode mode = localization.getLimelightUpdateMode();
```

### Manual Update (MANUAL mode only)
```java
boolean success = localization.updateWithLimelight();
// Returns true if update applied, false if conditions not met
```

### Check If Updated
```java
boolean wasUpdated = localization.wasOdometryUpdatedByLimelight();
```

## Mode Comparison

| Feature | AUTOMATIC | MANUAL | DISABLED |
|---------|-----------|--------|----------|
| Auto-updates | ✅ | ❌ | ❌ |
| Manual updates | N/A | ✅ | ❌ |
| Safety checks | ✅ | ✅ | N/A |
| CPU usage | Medium | Low | Very Low |

## Safety Checks (All Apply to MANUAL mode)

Manual updates are only applied if ALL conditions are met:
- ✅ Target within 4 feet
- ✅ Heading stable (10 readings)
- ✅ Velocity < 100 mm/s
- ✅ Data fresh and good quality
- ✅ Position jump < 300mm

## Common Patterns

### Before Critical Action
```java
loc.setLimelightUpdateMode(Localization.LimelightUpdateMode.MANUAL);

// Before shooting
if (loc.updateWithLimelight()) {
    fireBall();  // We have accurate pose
} else {
    telemetry.addLine("Can't get accurate pose");
}
```

### Periodic Manual Updates
```java
long lastUpdate = 0;

while (opModeIsActive()) {
    loc.update();
    
    if (System.currentTimeMillis() - lastUpdate > 2000) {
        if (loc.updateWithLimelight()) {
            lastUpdate = System.currentTimeMillis();
        }
    }
}
```

### Dynamic Mode Switching
```java
if (nearAprilTag) {
    loc.setLimelightUpdateMode(Localization.LimelightUpdateMode.AUTOMATIC);
} else {
    loc.setLimelightUpdateMode(Localization.LimelightUpdateMode.DISABLED);
}
```

## When to Use Each Mode

### Use AUTOMATIC when:
- ✅ Driving near AprilTags
- ✅ General operation
- ✅ You want continuous corrections

### Use MANUAL when:
- ✅ Need precise control over timing
- ✅ Before critical actions (shooting, scoring)
- ✅ Want to minimize processing
- ✅ Complex maneuvers need predictability

### Use DISABLED when:
- ✅ No AprilTags visible
- ✅ Fast movement across field
- ✅ Testing odometry only
- ✅ Limelight causing issues

## Example OpMode

See `LimelightManualUpdateExample.java` for a complete working example with:
- Mode switching with buttons
- Manual update triggering
- Status telemetry

## Debugging

### Check current mode:
```java
telemetry.addData("Mode", localization.getLimelightUpdateMode());
```

### Check why update failed:
```java
if (!localization.updateWithLimelight()) {
    telemetry.addData("Velocity", localization.getVelocityMagnitude(DistanceUnit.MM));
    telemetry.addData("Stable", localization.isLimelightHeadingStable());
}
```

---

**Quick Start:** Use `LimelightManualUpdateExample.java` to test all modes!

**Date:** January 24, 2026
