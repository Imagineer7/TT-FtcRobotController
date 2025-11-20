# Panels Dashboard Guide

This guide explains how to use the **Panels Dashboard** for enhanced telemetry and field visualization in the FTC DECODE Starter Kit.

---

## 🎯 What is Panels?

**Panels** (also known as FTControl) is a web-based dashboard for FTC robots that provides:

- **Rich telemetry display** - Better organized than Driver Station
- **Field visualization** - See robot position and paths in real-time
- **Configurable parameters** - Adjust values without redeploying code
- **OpMode control** - Start/stop OpModes from browser
- **Camera streams** - View robot camera feeds (if configured)

### Why Use Panels?

**Compared to standard Driver Station telemetry:**
- ✅ Better organization - Categorized data, graphs, field view
- ✅ More information - Can display more data simultaneously
- ✅ Visual debugging - See exactly where robot thinks it is
- ✅ Remote monitoring - View from any device on network
- ✅ Data logging - Record session data for analysis

**Best for:**
- Advanced autonomous development
- Path visualization and debugging
- Competition scouting (monitor robot remotely)
- Driver training (display more strategic info)

---

## 🚀 Quick Start

### Step 1: Connect to Robot

1. **Power on Robot Controller** (Control Hub or phone)

2. **Connect to Robot Wi-Fi:**
   - Network name: `ControlHub-XXXX` or similar
   - Password: (configured during setup)

3. **Open Panels Dashboard:**
   - Open web browser (Chrome recommended)
   - Navigate to: **http://192.168.43.1:8080**
   - You should see the Panels interface

**Tip:** Bookmark this URL for quick access!

### Step 2: Verify Connection

On the Panels dashboard, you should see:
- Robot status (connected/disconnected)
- Available OpModes
- Telemetry section (empty until OpMode runs)
- Field view (shows field outline)

**If dashboard doesn't load:**
- Check Wi-Fi connection
- Verify Robot Controller app is running
- Try different browser
- Check URL is correct

### Step 3: Run OpMode with Panels

1. **Select OpMode:**
   - In Panels: Click OpMode dropdown
   - Choose "Pedro Auto Blue (Advanced)" or "Starter TeleOp"

2. **Initialize:**
   - Click INIT button
   - Watch telemetry appear
   - Check for "Panels dashboard connected" message

3. **Start:**
   - Click START button (or use Driver Station)
   - Watch robot position on field view
   - Monitor telemetry in real-time

---

## 📊 Dashboard Interface

### Main Sections

**1. OpMode Control (Top)**
- Select OpMode from dropdown
- INIT / START / STOP buttons
- Connection status indicator

**2. Telemetry (Left Panel)**
- Organized in collapsible sections
- Key-value pairs from robot code
- Auto-updating every loop

**3. Field View (Center)**
- Top-down view of FTC field
- Robot position (green circle with heading line)
- Paths (blue lines)
- Targets and waypoints (red circles)

**4. Configuration (Right Panel - if enabled)**
- Adjustable parameters
- Requires @Configurable annotation in code

**5. Camera Stream (Bottom - if configured)**
- Live video from robot cameras
- Useful for driver practice and vision testing

---

## 💻 Using PanelsDashboardHelper

The starter kit includes `PanelsDashboardHelper` class that makes Panels easy to use.

### Basic Setup

**In your OpMode:**

```java
import org.firstinspires.ftc.teamcode.dashboard.PanelsDashboardHelper;

public class MyOpMode extends OpMode {
    private PanelsDashboardHelper dashboard;
    
    @Override
    public void init() {
        // Create dashboard helper
        dashboard = new PanelsDashboardHelper(telemetry);
        
        dashboard.logLine("MyOpMode initialized!");
        dashboard.update();
    }
    
    @Override
    public void loop() {
        // Log telemetry
        dashboard.log("X Position", robot.getX());
        dashboard.log("Y Position", robot.getY());
        
        // Update at end of loop
        dashboard.update();
    }
}
```

### Telemetry Methods

**Log key-value pairs:**
```java
dashboard.log("Caption", value);
dashboard.log("Speed", "%.2f in/s", currentSpeed);
```
- Displays on both Driver Station AND Panels
- Formatted values using String.format syntax

**Log text lines:**
```java
dashboard.logLine("✅ System ready");
dashboard.logLine("━━━━ SHOOTER ━━━━");
```
- Good for headers and status messages
- Use icons for visual clarity: ✅ ❌ ⚠️ ⏳

**Get raw telemetry:**
```java
Telemetry sdk = dashboard.getSdkTelemetry();
sdk.addData("Direct", "to Driver Station only");
```
- Use when you don't need Panels mirroring
- Useful for very frequent updates

### Field Visualization Methods

**Draw robot position:**
```java
dashboard.setRobotPose(xInches, yInches, headingDeg);
```
- Shows robot as green circle with heading indicator
- Updates in real-time as robot moves

**Draw robot from Pedro Pose:**
```java
Pose currentPose = follower.getPose();
dashboard.setRobotPose(currentPose);
```
- Convenience method for Pedro Pathing
- Automatically extracts x, y, heading

**Draw paths:**
```java
List<double[]> waypoints = new ArrayList<>();
waypoints.add(new double[]{0, 0});
waypoints.add(new double[]{24, 24});
waypoints.add(new double[]{48, 0});
dashboard.drawPath(waypoints);
```
- Shows planned path as blue line
- Useful for visualizing autonomous routes

**Draw targets:**
```java
dashboard.drawTarget(goalX, goalY, 6.0);
```
- Marks important positions on field
- Radius in inches

**Clear overlays:**
```java
dashboard.clearFieldOverlays();
```
- Call at start of loop to refresh drawings
- Prevents cluttering from previous frames

---

## 🗺️ Coordinate System

Panels uses the same coordinate system as Pedro Pathing:

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

**Important Points:**
- Origin at field center
- +X toward red wall
- +Y toward blue audience
- Heading: 0° = facing +X (counterclockwise positive)

**Field Bounds:**
- X: -72 to +72 inches
- Y: -72 to +72 inches
- 144" × 144" total (12 ft × 12 ft)

**When drawing:**
- Use field coordinates (not robot-relative)
- Convert robot sensors to field frame
- Keep units consistent (inches)

---

## 🎨 Customization

### Adding Custom Telemetry Sections

Organize telemetry into logical groups:

```java
// Drive system section
dashboard.logLine("━━━━ DRIVE SYSTEM ━━━━");
dashboard.log("X Position", "%.1f in", pose.getX());
dashboard.log("Y Position", "%.1f in", pose.getY());
dashboard.log("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
dashboard.logLine("");

// Shooter system section
dashboard.logLine("━━━━ SHOOTER ━━━━");
dashboard.log("Status", shooter.getStateString());
dashboard.log("RPM", "%.0f / %.0f", current, target);
dashboard.logLine("");
```

### Using @Configurable Annotation

Make parameters adjustable without redeploying:

```java
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.ConfigurableValue;

@Autonomous(name="My Auto")
@Configurable  // Enable configuration in Panels
public class MyAuto extends OpMode {
    
    @ConfigurableValue("Shooter Power")
    public static double shooterPower = 0.8;
    
    @ConfigurableValue("Wait Time (sec)")
    public static double waitTime = 1.5;
    
    // Use these values in your code
    // Adjust in Panels dashboard without redeploying!
}
```

**Benefits:**
- Quick tuning during practice
- Test different values rapidly
- No code redeployment needed

### Custom Field Overlays

Draw custom shapes and indicators:

```java
// Draw a zone or target area
for (int i = 0; i < 4; i++) {
    double x1 = zoneCorners[i][0];
    double y1 = zoneCorners[i][1];
    double x2 = zoneCorners[(i+1)%4][0];
    double y2 = zoneCorners[(i+1)%4][1];
    // Use Panels API directly for advanced shapes
}
```

---

## 🔍 Advanced Features

### Telemetry Graphs

Plot values over time:

```java
import com.bylazar.telemetry.PanelsTelemetry;

TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
panels.plot("RPM History", currentRPM);
```
- Creates real-time graph
- Useful for tuning PID loops
- Monitor trends and oscillations

### Field Presets

Panels includes field presets for different coordinate systems:

```java
import com.bylazar.field.PanelsField;

FieldManager field = PanelsField.INSTANCE.getField();

// Pedro Pathing preset (origin at center)
field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

// Road Runner preset (different convention)
field.setOffsets(PanelsField.INSTANCE.getPresets().getROAD_RUNNER());

// Custom offsets
field.setOffsets(customXOffset, customYOffset, customRotation);
```

### Camera Streams

Display robot camera on dashboard:

```java
import com.bylazar.camerastream.CameraStreamManager;

// In OpMode init
CameraStreamManager.INSTANCE.startStream(hardwareMap, "Webcam 1");
```
- Streams to Panels dashboard
- Useful for driver view or vision debugging
- Adjust frame rate and resolution as needed

---

## 🐛 Troubleshooting

### Dashboard Won't Load

**Check:**
1. Wi-Fi connected to Control Hub
2. URL is correct: http://192.168.43.1:8080
3. Robot Controller app is running
4. No firewall blocking port 8080

**Try:**
- Different browser (Chrome works best)
- Clear browser cache
- Restart Robot Controller
- Check Control Hub IP if using different network

### Telemetry Not Updating

**Common Causes:**
- Forgot to call `dashboard.update()` in loop
- OpMode not initialized/started
- Panels library not included in dependencies

**Fix:**
```java
@Override
public void loop() {
    // ... your code ...
    
    dashboard.update();  // MUST call this!
}
```

### Field View Shows Wrong Position

**Check:**
1. Starting pose set correctly in code
2. Odometry calibrated (pod offsets)
3. Coordinate system matches (Pedro vs custom)
4. Field preset selected correctly

**Debug:**
```java
dashboard.log("Raw Odo X", odometry.getX());
dashboard.log("Raw Odo Y", odometry.getY());
dashboard.log("Converted X", convertedX);
dashboard.log("Converted Y", convertedY);
```

### Robot Position Drifts

**Not a Panels issue - this is odometry:**
- Check encoder connections
- Verify pod offsets
- Calibrate odometry
- See Pedro-AdvancedAuto.md for odometry troubleshooting

### Panels Slows Down Robot

**Rare, but possible solutions:**
1. Reduce telemetry frequency:
   ```java
   if (loopCount % 5 == 0) {  // Update every 5 loops
       dashboard.update();
   }
   ```

2. Limit field drawings:
   - Don't redraw static elements every loop
   - Cache complex shapes

3. Disable camera stream if not needed

---

## 📚 Integration Examples

### Example 1: TeleOp with Panels

```java
@TeleOp(name="TeleOp with Panels")
@Configurable
public class PanelsTeleOp extends OpMode {
    private PanelsDashboardHelper dashboard;
    private StarterDrive drive;
    private StarterShooter shooter;
    
    @Override
    public void init() {
        dashboard = new PanelsDashboardHelper(telemetry);
        drive = new StarterDrive(hardwareMap, DriveType.MECANUM);
        shooter = new StarterShooter(hardwareMap);
    }
    
    @Override
    public void loop() {
        // Update subsystems
        drive.drive(
            -gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            gamepad1.right_stick_x
        );
        shooter.update(gamepad1);
        
        // Telemetry
        dashboard.logLine("━━━━ DRIVE ━━━━");
        dashboard.log("Max Power", "%.0f%%", drive.getMaxPower() * 100);
        dashboard.logLine("");
        
        dashboard.logLine("━━━━ SHOOTER ━━━━");
        dashboard.log("State", shooter.getStateString());
        dashboard.log("RPM", "%.0f", shooter.getCurrentRPM());
        
        dashboard.update();
    }
}
```

### Example 2: Autonomous with Path Visualization

```java
@Autonomous(name="Auto with Panels")
@Configurable
public class PanelsAuto extends OpMode {
    private PanelsDashboardHelper dashboard;
    private Follower follower;
    private PathChain myPath;
    
    @Override
    public void init() {
        dashboard = new PanelsDashboardHelper(telemetry);
        follower = PedroConstants.createFollower(hardwareMap);
        
        // Build path
        myPath = follower.pathBuilder()
            .addBezierLine(/* ... */)
            .build();
        
        // Draw planned path
        List<double[]> waypoints = extractWaypoints(myPath);
        dashboard.drawPath(waypoints);
        
        dashboard.update();
    }
    
    @Override
    public void loop() {
        follower.update();
        
        // Draw current position
        dashboard.clearFieldOverlays();
        dashboard.setRobotPose(follower.getPose());
        dashboard.drawPath(waypoints);  // Redraw path
        
        // Telemetry
        dashboard.log("Path Progress", "%.1f%%", 
            follower.getCurrentPath().getProgress() * 100);
        
        dashboard.update();
    }
}
```

---

## 💡 Best Practices

### 1. Organize Telemetry Logically

Group related data, use visual separators:
```java
dashboard.logLine("╔════════════════════════════════════╗");
dashboard.logLine("║          ROBOT STATUS              ║");
dashboard.logLine("╚════════════════════════════════════╝");
dashboard.logLine("");

dashboard.logLine("━━━━ LOCALIZATION ━━━━");
// Position data here

dashboard.logLine("━━━━ SUBSYSTEMS ━━━━");
// Subsystem status here
```

### 2. Use Meaningful Labels

```java
// Bad:
dashboard.log("x", x);
dashboard.log("y", y);

// Good:
dashboard.log("X Position (inches)", "%.1f", x);
dashboard.log("Y Position (inches)", "%.1f", y);
```

### 3. Limit Update Frequency When Needed

```java
// High-frequency sensor data
if (loopCount % 2 == 0) {  // Every other loop
    dashboard.log("High Freq Data", value);
}

// Low-priority data
if (loopCount % 10 == 0) {  // Every 10 loops
    dashboard.log("Low Priority", value);
}

dashboard.update();  // Still call every loop
```

### 4. Handle Panels Unavailable Gracefully

```java
if (!dashboard.isPanelsAvailable()) {
    // Panels not running - that's okay
    // PanelsDashboardHelper automatically falls back to SDK telemetry
    // Continue normally
}
```

### 5. Clear Old Drawings

```java
dashboard.clearFieldOverlays();  // Clear at START of loop
dashboard.setRobotPose(currentPose);
dashboard.drawPath(currentPath);
dashboard.update();  // Push at END of loop
```

---

## 🎓 Learning Resources

### Official Documentation
- Panels GitHub: [Link]
- FTC Control Documentation: [Link]
- Configuration Examples: [Link]

### Community Resources
- FTC Discord: #dashboard channel
- Example OpModes in this repo: `auto/pedro/`
- Video tutorials: [Search "FTC Panels Dashboard"]

---

## 🆘 Getting Help

**If you're stuck:**
1. Check this guide's troubleshooting section
2. Verify PanelsDashboardHelper is being used correctly
3. Test with example OpModes first
4. Ask in FTC community forums with specific error details

**When asking for help:**
- What were you trying to do?
- What happened instead?
- Code snippet showing usage
- Screenshot of Panels dashboard (if relevant)
- Error messages from Driver Station or Panels

---

**Ready to visualize your robot's performance?** Open Panels at http://192.168.43.1:8080 and start exploring! 🎨

Good luck with advanced autonomous development!
