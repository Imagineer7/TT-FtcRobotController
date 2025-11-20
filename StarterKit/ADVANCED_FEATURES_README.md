# Advanced Features - Pedro Pathing + Panels Dashboard

This document provides a quick reference for the advanced features added in Step 2 of the FTC DECODE Starter Kit.

---

## 📦 What Was Added

### Core Components

**Pedro Pathing Module** (`auto/pedro/`):
- `PedroConstants.java` - Complete configuration for path following
- `PedroAutoBlue.java` - Example blue alliance autonomous
- `PedroAutoRed.java` - Example red alliance autonomous

**Panels Dashboard** (`dashboard/`):
- `PanelsDashboardHelper.java` - Unified API for telemetry and field visualization

**Odometry Support** (`util/odometry/`):
- `GoBildaPinpointDriver.java` - Driver for GoBilda Pinpoint odometry computer

### Documentation

- `docs/Pedro-AdvancedAuto.md` - Complete guide to Pedro Pathing
- `docs/Panels-Dashboard-Guide.md` - Complete guide to Panels Dashboard
- Updated `README.md` with Advanced Features section

### Dependencies

Added to `build.dependencies.gradle`:
- Pedro Pathing v2.0.4 (path following library)
- Panels v1.0.10 (dashboard and visualization)
- Camera Stream v1.0.0 (for camera integration)

---

## 🎯 Key Features

### Pedro Pathing

**Purpose:** Professional autonomous path following with ~1 inch accuracy

**Features:**
- Smooth bezier curve paths (no jerky movements)
- Automatic PID control (no manual tuning)
- Real-time pose tracking with odometry
- Field visualization on Panels dashboard

**Requirements:**
- GoBilda Pinpoint Odometry Computer (or custom dead wheels)
- Mecanum or X-drive drivetrain
- Understanding of field coordinate system

**Example Usage:**
```java
// Create follower
Follower follower = PedroConstants.createFollower(hardwareMap);

// Set starting pose
follower.setStartingPose(new Pose(56.5, 8.5, Math.toRadians(90)));

// Build path
PathChain myPath = follower.pathBuilder()
    .addBezierLine(
        new Point(startX, startY, Point.CARTESIAN),
        new Point(endX, endY, Point.CARTESIAN)
    )
    .setLinearHeadingInterpolation(startHeading, endHeading)
    .build();

// Follow path
follower.followPath(myPath);

// In loop
follower.update();
if (!follower.isBusy()) {
    // Path complete
}
```

### Panels Dashboard

**Purpose:** Enhanced telemetry and field visualization via web browser

**Features:**
- Real-time robot position on field view
- Path visualization
- Organized telemetry display
- Configurable parameters
- Remote monitoring

**Access:**
- Connect to Robot Controller Wi-Fi
- Open browser: http://192.168.43.1:8080

**Example Usage:**
```java
// Create helper
PanelsDashboardHelper dashboard = new PanelsDashboardHelper(telemetry);

// Log telemetry
dashboard.log("X Position", pose.getX());
dashboard.log("Y Position", pose.getY());

// Draw robot on field
dashboard.setRobotPose(pose);

// Update (call every loop)
dashboard.update();
```

### PanelsDashboardHelper API

**Telemetry Methods:**
- `log(caption, value)` - Log key-value pair
- `logLine(text)` - Log text line
- `update()` - Push updates to dashboard

**Field Visualization:**
- `setRobotPose(x, y, heading)` - Draw robot position
- `setRobotPose(Pose)` - Draw from Pedro Pose object
- `drawPath(waypoints)` - Draw path on field
- `drawTarget(x, y, radius)` - Draw target marker
- `clearFieldOverlays()` - Clear all drawings

**Status:**
- `isPanelsAvailable()` - Check if Panels connected
- `getPanelsUrl()` - Get dashboard URL

---

## 🚀 Getting Started

### For Pedro Pathing

1. **Read Documentation:**
   - [docs/Pedro-AdvancedAuto.md](docs/Pedro-AdvancedAuto.md)

2. **Install Hardware:**
   - GoBilda Pinpoint Odometry Computer
   - Connect to I2C port as "odo"

3. **Configure:**
   - Measure pod offsets accurately
   - Update `PedroConstants.java`
   - Tune robot mass and velocities

4. **Test:**
   - Run `PedroAutoBlue` or `PedroAutoRed`
   - Monitor on Panels dashboard
   - Verify position accuracy

### For Panels Dashboard

1. **Read Documentation:**
   - [docs/Panels-Dashboard-Guide.md](docs/Panels-Dashboard-Guide.md)

2. **Connect:**
   - Join Robot Controller Wi-Fi
   - Open http://192.168.43.1:8080

3. **Use in Code:**
   - Create `PanelsDashboardHelper`
   - Add telemetry calls
   - Call `update()` every loop

4. **View:**
   - See robot position on field view
   - Monitor telemetry in real-time
   - Verify coordinate system

---

## 📐 Coordinate System

Both Pedro Pathing and Panels use the same coordinate system:

```
         +Y (Blue Audience)
            ↑
            │
            │
─────────────┼─────────────→ +X (Red Wall)
            │
            │
       (0, 0) Field Center
```

**Key Points:**
- Origin at field center
- +X toward red alliance wall
- +Y toward blue audience side
- Heading: 0° = facing +X (counterclockwise positive)
- Field bounds: -72 to +72 inches (X and Y)

---

## 🎓 Learning Progression

### Week 1-4: Master Basics
- Use StarterTeleOp and StarterAutoSkeleton
- Get comfortable with robot operation
- Score 20-30 points reliably

### Week 5-6: Add Panels
- Connect Panels dashboard
- Add `PanelsDashboardHelper` to OpModes
- Improve telemetry organization
- Remote monitoring

### Week 7-8: Install Odometry
- Mount GoBilda Pinpoint
- Calibrate pod offsets
- Test localization accuracy
- Verify coordinate system

### Week 9+: Pedro Pathing
- Study example OpModes
- Create simple paths
- Tune path constraints
- Build competition autonomous
- Aim for 60-90+ points

---

## 🔧 Configuration Files

### PedroConstants.java

**Key Parameters to Tune:**
```java
// Robot characteristics
ROBOT_MASS_KG = 9.0               // Weigh your robot
X_VELOCITY = 60.0                 // Measure forward speed
Y_VELOCITY = 50.0                 // Measure strafe speed

// Path constraints
MAX_POWER = 0.85                  // Start at 85%, increase if needed
MAX_ACCELERATION = 80.0           // Adjust for smoothness
MAX_DECELERATION = 100.0          // Can brake harder
MAX_ANGULAR_VELOCITY = 2.0        // Radians per second

// Odometry offsets (CRITICAL!)
FORWARD_POD_Y_OFFSET = -6.62      // Measure accurately
STRAFE_POD_X_OFFSET = 4.71        // Measure accurately
```

### Motor Names

Must match robot configuration:
```java
FRONT_LEFT_MOTOR = "frontLeft"
FRONT_RIGHT_MOTOR = "frontRight"
BACK_LEFT_MOTOR = "backLeft"
BACK_RIGHT_MOTOR = "backRight"
ODOMETRY_HARDWARE_NAME = "odo"
```

---

## ⚙️ Build Configuration

### Gradle Dependencies

Added to `build.dependencies.gradle`:

```gradle
// Pedro Pathing
implementation('com.pedropathing:ftc:2.0.4') {
    exclude group: "com.acmerobotics.dashboard"
}

// Panels Dashboard
implementation('com.bylazar:fullpanels:1.0.10') {
    exclude group: "com.acmerobotics.dashboard"
}
implementation('com.bylazar:camerastream:1.0.0') {
    exclude group: "com.acmerobotics.dashboard"
}
```

### Repository

```gradle
maven { url = "https://mymaven.bylazar.com/releases" }
```

**Note:** FTC Dashboard is excluded to avoid conflicts with Panels.

---

## 🐛 Troubleshooting

### Pedro Pathing Issues

**Robot doesn't move:**
- Check motor names match configuration
- Verify motor directions
- Ensure `follower.update()` called in loop

**Position drifts:**
- Verify pod offsets are correct
- Check encoder connections
- Calibrate odometry
- Test with localization OpMode

**Paths overshoot:**
- Decrease MAX_POWER
- Increase MAX_DECELERATION
- Tune follower constants

### Panels Dashboard Issues

**Dashboard won't load:**
- Verify Wi-Fi connected to Control Hub
- Check URL: http://192.168.43.1:8080
- Try different browser (Chrome recommended)
- Restart Robot Controller

**Telemetry not updating:**
- Ensure `dashboard.update()` called every loop
- Check Panels library included
- Verify OpMode is running

**Field view shows wrong position:**
- Check starting pose set correctly
- Verify odometry calibration
- Confirm field preset (PEDRO_PATHING)

---

## 📚 Additional Resources

### Documentation
- [Pedro-AdvancedAuto.md](docs/Pedro-AdvancedAuto.md) - Complete Pedro Pathing guide
- [Panels-Dashboard-Guide.md](docs/Panels-Dashboard-Guide.md) - Complete Panels guide
- [README.md](README.md) - Main starter kit documentation

### Example Code
- `auto/pedro/PedroAutoBlue.java` - Blue alliance example
- `auto/pedro/PedroAutoRed.java` - Red alliance example
- `dashboard/PanelsDashboardHelper.java` - Helper class with full API

### Community
- FTC Discord: #programming channel
- Chief Delphi: FTC subforum
- Pedro Pathing GitHub: https://github.com/Pedro-Pathing

---

## 💡 Best Practices

### 1. Start Simple
- Test basic paths first (straight lines)
- Add complexity gradually
- Verify each change works before proceeding

### 2. Measure Accurately
- Pod offsets are CRITICAL
- Use calipers or precise ruler
- Double-check all measurements

### 3. Use Visualization
- Always test with Panels dashboard
- Verify paths visually before running
- Check robot position makes sense

### 4. Document Everything
- Comment why each waypoint is chosen
- Note field positions in code
- Keep coordinate system consistent

### 5. Test Incrementally
- Run each path 5+ times
- Test different starting battery levels
- Practice on actual competition field

---

## 🎯 Success Metrics

### Baseline (StarterAutoSkeleton)
- ±6 inch position accuracy
- 15-20 points autonomous
- 30-40 points total match

### With Pedro Pathing
- ±1 inch position accuracy
- 20-30 points autonomous
- 70-100 points total match
- Consistent multi-path strategies

### With Panels Dashboard
- Better telemetry organization
- Visual debugging of paths
- Remote monitoring capability
- Faster autonomous development

---

**Advanced features are optional but powerful.** Master basics first, then add these tools when ready for competition! 🚀

Good luck in the DECODE season!
