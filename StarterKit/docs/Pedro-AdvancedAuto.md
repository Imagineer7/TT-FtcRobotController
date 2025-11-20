# Pedro Pathing - Advanced Autonomous Guide

This guide explains how to use **Pedro Pathing** for advanced autonomous programming in the FTC DECODE Starter Kit.

---

## 🎯 What is Pedro Pathing?

**Pedro Pathing** is a powerful path-following library for FTC robots that provides:

- **Smooth bezier curve paths** - No jerky movements, professional-looking autonomous
- **Automatic path following** - Library handles all the PID math for you
- **Real-time pose tracking** - Always knows where robot is on the field
- **Field visualization** - See paths and robot position live on Panels dashboard
- **Competition-tested** - Used by advanced FTC teams worldwide

### Why Use Pedro?

**Compared to time-based autonomous (StarterAutoSkeleton):**
- ✅ More accurate - reaches targets within ~1 inch
- ✅ More reliable - adapts to obstacles and drift
- ✅ More flexible - easy to create complex paths
- ✅ Better visualization - see exactly what robot will do

**Trade-offs:**
- ❌ Requires odometry hardware (GoBilda Pinpoint or dead wheels)
- ❌ More setup and tuning required
- ❌ Steeper learning curve

**Recommendation:** Start with StarterAutoSkeleton, upgrade to Pedro when ready for competition.

---

## 🔧 Requirements

### Hardware Requirements

**Minimum (Required):**
1. **GoBilda Pinpoint Odometry Computer**
   - Part number: [Link to product]
   - Plugs into I2C port on Control Hub
   - Uses 2 encoder pods + built-in IMU
   
2. **Mecanum or X-drive drivetrain**
   - 4 motors with encoders
   - Motor names must match PedroConstants

**Alternative (Advanced):**
- 3-wheel dead wheel odometry (requires custom implementation)
- 2-wheel + IMU odometry

### Software Requirements

1. **Pedro Pathing library** - Already included in starter kit dependencies
2. **Panels dashboard** - Already included, optional but highly recommended
3. **Android Studio** - For code development

---

## 📐 Coordinate System

Understanding the coordinate system is CRITICAL for Pedro Pathing.

### Field Coordinate System

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
- Origin (0, 0) is at **field center**
- +X points toward **red alliance wall**
- +Y points toward **blue audience side**
- Heading: 0° = facing +X, increases counterclockwise

**Examples:**
- Blue starting position: (+56, +8) or thereabouts
- Red starting position: (-56, +8) (mirror across Y-axis)
- Field dimensions: -72 to +72 inches in both X and Y

### Heading Convention

- 0° = Robot facing toward red wall (+X direction)
- 90° = Robot facing toward blue audience (+Y direction)
- 180° = Robot facing toward blue wall (-X direction)
- 270° = Robot facing toward red audience (-Y direction)

**Tip:** Use `Math.toRadians(degrees)` to convert degrees to radians for Pedro API.

---

## 🚀 Quick Start

### Step 1: Hardware Setup

1. **Install GoBilda Pinpoint:**
   - Mount securely to robot chassis
   - Connect to I2C port on Control Hub
   - Configure in Robot Configuration as "odo"

2. **Measure Pod Offsets:**
   - Find center of rotation (usually geometric center of robot)
   - Measure distance from center to forward pod (Y offset)
   - Measure distance from center to strafe pod (X offset)
   - Record values in inches

3. **Configure PedroConstants.java:**
   ```java
   public static final double FORWARD_POD_Y_OFFSET = -6.62;  // Your measurement
   public static final double STRAFE_POD_X_OFFSET = 4.71;    // Your measurement
   ```

### Step 2: Tune Drive Constants

1. **Measure Robot Mass:**
   - Stand on scale with robot
   - Subtract your weight
   - Convert to kg (divide pounds by 2.205)
   - Update in PedroConstants:
   ```java
   public static final double ROBOT_MASS_KG = 9.0;  // Your robot's mass
   ```

2. **Measure Velocity:**
   - Drive straight at full power for 3 seconds
   - Measure distance traveled
   - Calculate: distance (inches) / time (seconds)
   - Update X_VELOCITY and Y_VELOCITY in PedroConstants

3. **Test Movement:**
   - Run a simple forward path
   - Verify robot moves expected distance
   - Adjust velocities if needed

### Step 3: Run Example Autonomous

1. **Select OpMode:**
   - On Driver Station, select "Pedro Auto Blue (Advanced)"
   - Or "Pedro Auto Red (Advanced)" for red alliance

2. **Connect Panels Dashboard:**
   - Open browser: http://192.168.43.1:8080
   - You should see field view with robot position

3. **Run Autonomous:**
   - Press START on Driver Station
   - Watch robot follow path
   - Monitor telemetry on both DS and Panels

4. **Review Results:**
   - Did robot leave LAUNCH LINE? (+3 points)
   - Did it turn to correct heading?
   - Did shooter fire successfully?
   - Check field view for path accuracy

---

## 📝 Creating Custom Paths

### Basic Path Structure

```java
// In your OpMode's buildPaths() method:

PathChain myPath = follower.pathBuilder()
    .addBezierLine(
        new Point(startX, startY, Point.CARTESIAN),
        new Point(endX, endY, Point.CARTESIAN)
    )
    .setLinearHeadingInterpolation(startHeading, endHeading)
    .build();
```

### Path Types

**BezierLine (Straight Line):**
```java
.addBezierLine(
    new Point(0, 0, Point.CARTESIAN),
    new Point(24, 24, Point.CARTESIAN)
)
```
- Simplest path type
- Robot drives straight from start to end
- Good for leaving LAUNCH LINE, moving to BASE

**BezierCurve (Smooth Curve):**
```java
.addBezierCurve(
    new Point(0, 0, Point.CARTESIAN),      // Start
    new Point(12, 12, Point.CARTESIAN),    // Control point 1
    new Point(24, 0, Point.CARTESIAN)      // End
)
```
- Creates smooth curved path
- Control points define curve shape
- Good for avoiding obstacles, smooth turns

**Complex Paths (Multiple Segments):**
```java
PathChain complexPath = follower.pathBuilder()
    .addBezierLine(/* start to midpoint */)
    .addBezierCurve(/* curved section */)
    .addBezierLine(/* final approach */)
    .setLinearHeadingInterpolation(startHeading, endHeading)
    .build();
```

### Heading Interpolation

**Linear (Smooth Turn):**
```java
.setLinearHeadingInterpolation(
    Math.toRadians(0),    // Start heading
    Math.toRadians(90)    // End heading
)
```
- Robot smoothly rotates while following path
- Heading changes proportionally to progress
- Use for most cases

**Constant Heading:**
```java
.setConstantHeadingInterpolation(Math.toRadians(90))
```
- Robot maintains fixed heading throughout path
- Good for strafing while facing goal

**Tangent Heading:**
```java
.setTangentialHeadingInterpolation()
```
- Robot always faces direction of movement
- Good for intake/outtake while moving

---

## 🎯 Example Autonomous Strategies

### Strategy 1: Score Preloads + Park

```java
private void buildPaths() {
    // 1. Leave LAUNCH LINE
    PathChain leaveLine = follower.pathBuilder()
        .addBezierLine(
            new Point(startX, startY, Point.CARTESIAN),
            new Point(startX, startY + 30, Point.CARTESIAN)
        )
        .setLinearHeadingInterpolation(startHeading, startHeading)
        .build();
    
    // 2. Move to shooting position
    PathChain toShootPos = follower.pathBuilder()
        .addBezierLine(
            new Point(startX, startY + 30, Point.CARTESIAN),
            new Point(shootX, shootY, Point.CARTESIAN)
        )
        .setLinearHeadingInterpolation(startHeading, shootHeading)
        .build();
    
    // 3. Move to BASE for parking
    PathChain toBase = follower.pathBuilder()
        .addBezierLine(
            new Point(shootX, shootY, Point.CARTESIAN),
            new Point(baseX, baseY, Point.CARTESIAN)
        )
        .setLinearHeadingInterpolation(shootHeading, baseHeading)
        .build();
}
```

**State Machine:**
```java
switch (currentState) {
    case LEAVE_LINE:
        follower.followPath(leaveLine);
        // Transition when path complete
        break;
    
    case MOVE_TO_SHOOT:
        follower.followPath(toShootPos);
        break;
    
    case SHOOT:
        // Spin up and fire
        break;
    
    case PARK:
        follower.followPath(toBase);
        break;
}
```

### Strategy 2: Score + Cycle

```java
// More advanced: pickup additional artifacts and score again
PathChain toPickup = follower.pathBuilder()
    .addBezierCurve(
        new Point(shootX, shootY, Point.CARTESIAN),
        new Point(midX, midY, Point.CARTESIAN),
        new Point(pickupX, pickupY, Point.CARTESIAN)
    )
    .setLinearHeadingInterpolation(shootHeading, pickupHeading)
    .build();

// Return to shooting position
PathChain backToShoot = follower.pathBuilder()
    .addBezierCurve(
        new Point(pickupX, pickupY, Point.CARTESIAN),
        new Point(midX, midY, Point.CARTESIAN),
        new Point(shootX, shootY, Point.CARTESIAN)
    )
    .setLinearHeadingInterpolation(pickupHeading, shootHeading)
    .build();
```

---

## 🔧 Tuning Guide

### Path Constraints

Located in `PedroConstants.java`:

```java
public static final double MAX_POWER = 0.85;
public static final double MAX_ACCELERATION = 80.0;
public static final double MAX_DECELERATION = 100.0;
public static final double MAX_ANGULAR_VELOCITY = 2.0;
```

**If robot is too slow:**
- Increase MAX_POWER (up to 1.0)
- Increase MAX_ACCELERATION
- Test incrementally

**If robot overshoots targets:**
- Decrease MAX_POWER
- Increase MAX_DECELERATION
- Check odometry calibration

**If turns are jerky:**
- Decrease MAX_ANGULAR_VELOCITY
- Use longer heading interpolation distance

### Follower Constants

```java
public static final double FORWARD_ZERO_POWER_ACCEL = -35.66;
public static final double LATERAL_ZERO_POWER_ACCEL = -55.63;
```

**How to tune:**
1. Drive robot at full speed
2. Cut power completely
3. Time how long it takes to stop
4. Calculate deceleration
5. Update constants

**Why it matters:**
- Helps Pedro predict how quickly robot stops
- Improves accuracy at end of paths
- Compensates for different wheel types and robot mass

### Odometry Calibration

**If robot position drifts:**
1. Check pod offsets are correct
2. Verify encoder directions
3. Test with "Localization Test" OpMode
4. Run calibration routine

**Localization Test:**
- Drive forward 48 inches, check X increases by 48
- Strafe left 48 inches, check Y increases by 48
- Rotate 360°, check heading returns to 0°

---

## 🐛 Troubleshooting

### Robot doesn't move

**Check:**
1. Motor names match PedroConstants
2. Motor directions are correct
3. Paths are being followed in loop (follower.update())
4. OpMode is in "started" state

**Debug:**
```java
dashboard.log("Follower Busy", follower.isBusy());
dashboard.log("Target Pose", follower.getCurrentPath().getEndPose());
```

### Robot moves wrong direction

**Fix:**
1. Check motor directions in PedroConstants
2. Swap left/right motor directions if needed
3. Verify coordinate system understanding

### Position drifts over time

**Causes:**
- Incorrect pod offsets
- Encoders skipping (bad connection or mounting)
- Robot slipping on field tiles

**Solutions:**
1. Remeasure and update pod offsets
2. Check encoder connections
3. Add weight to robot for better traction
4. Clean mecanum wheels

### Paths don't match visualization

**Check:**
1. Panels field view is set to Pedro Pathing preset
2. Coordinate system conversion is correct
3. Starting pose is set correctly

### Panels dashboard not loading

**Solutions:**
1. Verify URL: http://192.168.43.1:8080
2. Check Wi-Fi connection to Control Hub
3. Try different browser (Chrome recommended)
4. Restart Robot Controller app

---

## 📚 Advanced Topics

### Custom Path Constraints

Create different constraints for different situations:

```java
// Slow, precise movement for scoring
PathConstraints slowConstraints = new PathConstraints(0.4, 30, 40, 1.0);

// Fast movement for cycling
PathConstraints fastConstraints = new PathConstraints(1.0, 120, 150, 3.0);

// Use custom constraints
Follower slowFollower = PedroConstants.createFollower(hardwareMap, slowConstraints);
```

### Wait Actions

Add waits during paths:

```java
PathChain pathWithWait = follower.pathBuilder()
    .addBezierLine(/* first segment */)
    .addTemporalMarker(() -> {
        // This code runs at this point in the path
        shooter.spinUp();
    })
    .addBezierLine(/* second segment */)
    .build();
```

### Combining with Vision

Use AprilTags to improve accuracy:

```java
// After detecting AprilTag
Pose correctedPose = getAprilTagPose();
follower.setCurrentPose(correctedPose);
// Continue with paths
```

### Multi-Robot Coordination

Avoid collisions with alliance partner:

```java
if (detectedAlliancePartner()) {
    follower.pause();
    // Wait for clear path
    follower.resume();
}
```

---

## 🎓 Learning Resources

### Official Pedro Docs
- GitHub: https://github.com/Pedro-Pathing/pedro-pathing
- Documentation: [Link to docs]
- Example code: See `auto/pedro/` in this repo

### Community Resources
- FTC Discord: #programming channel
- Chief Delphi: FTC subforum
- YouTube tutorials: [Search "FTC Pedro Pathing"]

### Practice Exercises

1. **Exercise 1: Simple Square**
   - Create 4 BezierLines forming a square
   - Test path accuracy
   - Tune path constraints

2. **Exercise 2: Figure-8**
   - Use BezierCurves to create smooth figure-8
   - Practice with heading interpolation
   - Visualize on Panels

3. **Exercise 3: Competition Simulation**
   - Recreate your competition autonomous
   - Add state machine for scoring
   - Integrate with shooter subsystem

---

## 💡 Tips for Success

1. **Start Simple**
   - Begin with straight lines (BezierLine)
   - Add curves once basics work
   - Test one path at a time

2. **Use Panels Dashboard**
   - Always verify paths visually before running
   - Watch robot position in real-time
   - Catch errors early

3. **Measure Accurately**
   - Pod offsets affect everything
   - Use calipers or precise ruler
   - Double-check measurements

4. **Test Incrementally**
   - Test each path separately
   - Verify odometry accuracy first
   - Add complexity gradually

5. **Document Your Paths**
   - Comment why each waypoint is chosen
   - Note field positions in inches
   - Keep coordinate system consistent

6. **Practice, Practice, Practice**
   - Run autonomous 20+ times before competition
   - Test with different battery levels
   - Practice on actual competition field if possible

---

## 🆘 Getting Help

**If you're stuck:**

1. Check this guide's troubleshooting section
2. Review PedroConstants.java comments
3. Look at example OpModes (PedroAutoBlue/Red)
4. Ask in FTC community forums
5. Post code snippets when asking questions

**When asking for help, include:**
- Robot hardware configuration
- Code snippet showing issue
- Error messages or unexpected behavior
- What you've already tried

---

**Ready to create advanced autonomous paths?** Start with PedroAutoBlue.java and customize it for your strategy! 🚀

Good luck in the DECODE season!
