# PathPlanner Usage Guide

## Overview

The `PathPlanner` is a powerful tool that converts sparse waypoints into detailed, smooth paths for autonomous navigation. It supports three different path generation modes and provides utilities for path manipulation.

---

## Quick Start

### Basic Usage

```java
import org.firstinspires.ftc.teamcode.util.tool.PathPlanner;
import org.firstinspires.ftc.teamcode.util.tool.PathPlanner.PathMode;
import org.firstinspires.ftc.teamcode.util.tool.Pose;
import java.util.ArrayList;
import java.util.List;

// Create planner
PathPlanner planner = new PathPlanner();

// Define sparse waypoints
List<Pose> waypoints = new ArrayList<>();
waypoints.add(new Pose(0, 0, 0));      // Start
waypoints.add(new Pose(24, 24, 90));   // Turn point
waypoints.add(new Pose(48, 24, 90));   // End

// Generate detailed path
List<Pose> path = planner.generatePath(waypoints, PathMode.SMOOTH_CURVE);

// Use path with PoseController
for (Pose pose : path) {
    poseController.setTarget(pose.x, pose.y, pose.heading);
    while (!poseController.atTarget()) {
        poseController.update();
    }
}
```

---

## Path Modes

### 1. STRAIGHT Mode
**Description:** Simple linear interpolation between waypoints with sharp corners

**When to use:**
- ✅ Simple, predictable paths
- ✅ When you need precise corner positioning
- ✅ Low computational overhead
- ✅ Tank drive robots (no strafing)

**Characteristics:**
- Fastest to compute
- Sharp direction changes at waypoints
- Robot may slow down at corners
- Least smooth motion

**Example:**
```java
List<Pose> path = planner.generatePath(waypoints, PathMode.STRAIGHT);
```

**Visual:**
```
    B------>C
    ^
    |
    |
    A
```
Sharp 90° corner at B

---

### 2. SMOOTH_CURVE Mode (Recommended)
**Description:** Smoothed path using Catmull-Rom-like interpolation with adjustable rounding

**When to use:**
- ✅ **General autonomous navigation (RECOMMENDED)**
- ✅ When you want smooth motion without sharp turns
- ✅ Mecanum/omni drives
- ✅ High-speed navigation
- ✅ Good balance of smoothness and control

**Characteristics:**
- Rounds corners smoothly
- Adjustable smoothing factor (0.0 to 1.0)
- Passes through or near waypoints (depending on smoothingFactor)
- Moderate computation cost

**Example:**
```java
PathPlanner planner = new PathPlanner();
planner.smoothingFactor = 0.7;  // 0.0 = sharp, 1.0 = maximum smoothing
List<Pose> path = planner.generatePath(waypoints, PathMode.SMOOTH_CURVE);
```

**Visual:**
```
    B~~~~~~>C
   /
  /
 A
```
Smooth curve through B

**Tuning smoothingFactor:**
- `0.0` - Sharp corners (like STRAIGHT mode)
- `0.3` - Gentle rounding
- `0.5` - **Default, good balance**
- `0.7` - Smooth curves
- `1.0` - Maximum smoothing (may cut corners significantly)

---

### 3. CUBIC_SPLINE Mode
**Description:** Cubic spline interpolation for the smoothest possible paths

**When to use:**
- ✅ Very high-speed navigation
- ✅ When absolute smoothness is critical
- ✅ Racing or time-trial scenarios
- ✅ Professional-level autonomous

**Characteristics:**
- Smoothest motion possible
- Continuous curvature (no sudden direction changes)
- May deviate from waypoints to maintain smoothness
- Higher computation cost
- Requires at least 3 waypoints for proper operation

**Example:**
```java
List<Pose> path = planner.generatePath(waypoints, PathMode.CUBIC_SPLINE);
```

**Visual:**
```
     ~~~~C
   ~~
  B
 /
A
```
Very smooth S-curve

---

## Configuration Parameters

### waypoint Spacing
Distance between generated waypoints (inches)

```java
planner.waypointSpacing = 3.0;  // Default: 3 inches
```

**Recommendations:**
- **Small robots (< 12")**: 2-3 inches
- **Medium robots (12-18")**: 3-5 inches  
- **Large robots (> 18")**: 5-8 inches
- **High speed**: Smaller spacing (more detail)
- **Low speed**: Larger spacing (less computation)

**Trade-offs:**
- Smaller = More detail, smoother tracking, MORE computation
- Larger = Less detail, faster computation, may miss curves

---

### smoothingFactor
How much to round corners in SMOOTH_CURVE mode (0.0 to 1.0)

```java
planner.smoothingFactor = 0.5;  // Default
```

**Recommendations:**
- **Precision tasks**: 0.2 - 0.3 (gentle rounding)
- **General use**: 0.4 - 0.6 (balanced)
- **High speed**: 0.7 - 0.9 (maximum smoothing)

---

### minPointsPerSegment
Minimum waypoints between each input waypoint pair

```java
planner.minPointsPerSegment = 5;  // Default
```

Ensures smooth motion even for very short segments.

---

### interpolateHeading
Whether to smoothly change heading along the path

```java
planner.interpolateHeading = true;  // Default: true
```

- `true` - Gradual heading changes (smooth rotation)
- `false` - Heading changes only at waypoints (may cause sudden spins)

---

## Advanced Usage

### Generate Path from Current Position

```java
Pose currentPose = new Pose(
    positionManager.getCurrentX(),
    positionManager.getCurrentY(),
    positionManager.getCurrentHeading()
);

List<Pose> targets = new ArrayList<>();
targets.add(new Pose(24, 24, 90));
targets.add(new Pose(48, 24, 0));

List<Pose> completePath = planner.generatePathFromStart(
    currentPose, 
    targets, 
    PathMode.SMOOTH_CURVE
);
```

---

### Calculate Path Statistics

```java
// Get total path length
double length = PathPlanner.calculatePathLength(path);
telemetry.addData("Path Length", "%.1f inches", length);

// Get pose at specific distance along path
Pose midpoint = PathPlanner.getPoseAtDistance(path, length / 2.0);
telemetry.addData("Midpoint", midpoint.toString());
```

---

### Simplify a Path

Reduce the number of waypoints while maintaining overall shape:

```java
// Original path has 100 waypoints
List<Pose> detailedPath = planner.generatePath(waypoints, PathMode.SMOOTH_CURVE);

// Simplified path might have only 30 waypoints
List<Pose> simplified = PathPlanner.simplifyPath(detailedPath, 1.0);  // 1 inch tolerance
```

**Use cases:**
- Reduce memory usage
- Speed up path following
- Visualization/debugging

---

### Reverse a Path

```java
List<Pose> forwardPath = planner.generatePath(waypoints, PathMode.SMOOTH_CURVE);
List<Pose> returnPath = PathPlanner.reversePath(forwardPath);
```

Automatically reverses heading by 180° for backing up.

---

## Integration with PoseController

### Method 1: Follow Path Sequentially

```java
PathPlanner planner = new PathPlanner();
planner.waypointSpacing = 4.0;

List<Pose> waypoints = new ArrayList<>();
waypoints.add(new Pose(0, 0, 0));
waypoints.add(new Pose(24, 24, 90));
waypoints.add(new Pose(48, 24, 90));

List<Pose> path = planner.generatePath(waypoints, PathMode.SMOOTH_CURVE);

for (Pose target : path) {
    poseController.setTarget(target.x, target.y, target.heading);
    while (!poseController.atTarget()) {
        poseController.update();
        telemetry.update();
    }
}
```

### Method 2: Use Waypoint Queue (Recommended)

You'll implement this next with the PoseController waypoint system.

---

## Common Patterns

### Pattern 1: Scoring Run

```java
PathPlanner planner = new PathPlanner();

// Fast approach to scoring zone
planner.smoothingFactor = 0.7;
planner.waypointSpacing = 5.0;

List<Pose> approachPoints = new ArrayList<>();
approachPoints.add(currentPose);
approachPoints.add(new Pose(36, 12, 45));
approachPoints.add(new Pose(48, 18, 90));  // Near scoring position

List<Pose> approachPath = planner.generatePath(approachPoints, PathMode.SMOOTH_CURVE);

// Precise final alignment
List<Pose> finalAlign = new ArrayList<>();
finalAlign.add(approachPath.get(approachPath.size() - 1));
finalAlign.add(new Pose(SCORING_X, SCORING_Y, SCORING_HEADING));

planner.smoothingFactor = 0.2;  // Less smoothing for precision
planner.waypointSpacing = 2.0;   // More detail
List<Pose> alignPath = planner.generatePath(finalAlign, PathMode.STRAIGHT);
```

### Pattern 2: Obstacle Avoidance

```java
List<Pose> waypoints = new ArrayList<>();
waypoints.add(currentPose);
waypoints.add(new Pose(12, 12, 45));    // Avoid obstacle
waypoints.add(new Pose(24, 18, 90));    // Clear of obstacle
waypoints.add(new Pose(36, 18, 90));    // Continue to target

List<Pose> path = planner.generatePath(waypoints, PathMode.SMOOTH_CURVE);
```

### Pattern 3: Figure-8 Path

```java
List<Pose> figure8 = new ArrayList<>();
figure8.add(new Pose(0, 0, 0));
figure8.add(new Pose(12, 12, 90));
figure8.add(new Pose(24, 0, 180));
figure8.add(new Pose(12, -12, 270));
figure8.add(new Pose(0, 0, 0));

PathPlanner planner = new PathPlanner();
planner.waypointSpacing = 2.0;
List<Pose> path = planner.generatePath(figure8, PathMode.CUBIC_SPLINE);
```

---

## Performance Considerations

### Computation Time

| Mode | Relative Speed | Waypoints (100 sparse → detailed) |
|------|---------------|-----------------------------------|
| STRAIGHT | Fastest (1x) | ~300-500 |
| SMOOTH_CURVE | Fast (1.5x) | ~300-500 |
| CUBIC_SPLINE | Moderate (2x) | ~300-500 |

**Note:** All modes are fast enough for real-time use. Pre-compute paths in `init()` if possible.

### Memory Usage

Each `Pose` object uses ~40 bytes. A typical detailed path:
- 100 waypoints × 40 bytes = 4 KB (negligible)
- 500 waypoints × 40 bytes = 20 KB (still fine)

**Recommendation:** Don't worry about memory unless you have thousands of waypoints.

---

## Troubleshooting

### Problem: Path doesn't look smooth
**Solution:**
- Reduce `waypointSpacing` (more detail)
- Increase `smoothingFactor` (more rounding)
- Use `CUBIC_SPLINE` mode instead of `SMOOTH_CURVE`

### Problem: Robot cuts corners too much
**Solution:**
- Reduce `smoothingFactor` (less rounding)
- Add more waypoints at critical turns
- Use `STRAIGHT` mode for precise corners

### Problem: Path generation is slow
**Solution:**
- Increase `waypointSpacing` (less detail)
- Reduce number of input waypoints
- Pre-compute paths in `init()` instead of during autonomous

### Problem: Robot doesn't follow path accurately
**Solution:**
- This is a tracking problem, not path planning
- Tune PoseController gains (kP, kV)
- Check odometry accuracy
- Reduce robot speed
- Use smaller `waypointSpacing` for more guidance

### Problem: Path goes through obstacles
**Solution:**
- PathPlanner doesn't do obstacle avoidance automatically
- Add waypoints manually to go around obstacles
- Or implement custom obstacle avoidance logic

---

## Best Practices

1. **Pre-compute paths in `init()`**
   ```java
   @Override
   public void init() {
       // Compute paths during initialization
       path1 = planner.generatePath(waypoints1, PathMode.SMOOTH_CURVE);
       path2 = planner.generatePath(waypoints2, PathMode.SMOOTH_CURVE);
   }
   ```

2. **Start with SMOOTH_CURVE mode**
   - Best balance for most use cases
   - Tune `smoothingFactor` to your needs

3. **Use appropriate `waypointSpacing`**
   - Match to your robot size and speed
   - Smaller for precision, larger for speed

4. **Test paths in teleop first**
   - Verify paths make sense
   - Adjust waypoints as needed

5. **Combine modes for different sections**
   - SMOOTH_CURVE for approach
   - STRAIGHT for final alignment

---

## Example: Complete Autonomous

```java
@Autonomous(name = "PathPlanner Auto")
public class PathPlannerAuto extends LinearOpMode {
    
    private PathPlanner planner;
    private PoseController poseController;
    private List<Pose> mainPath;
    
    @Override
    public void runOpMode() {
        // Initialize
        // ... hardware init ...
        
        planner = new PathPlanner();
        planner.waypointSpacing = 4.0;
        planner.smoothingFactor = 0.6;
        
        // Define autonomous route
        List<Pose> waypoints = new ArrayList<>();
        waypoints.add(new Pose(0, 0, 0));           // Start
        waypoints.add(new Pose(24, 12, 45));        // Turn 1
        waypoints.add(new Pose(48, 24, 90));        // Scoring position
        waypoints.add(new Pose(48, 48, 180));       // Turn 2
        waypoints.add(new Pose(12, 48, 180));       // Park
        
        // Generate detailed path
        mainPath = planner.generatePath(waypoints, PathMode.SMOOTH_CURVE);
        
        telemetry.addData("Path Length", "%.1f in", 
                         PathPlanner.calculatePathLength(mainPath));
        telemetry.addData("Waypoints", mainPath.size());
        telemetry.update();
        
        waitForStart();
        
        // Follow path
        for (Pose target : mainPath) {
            poseController.setTarget(target.x, target.y, target.heading);
            
            while (opModeIsActive() && !poseController.atTarget()) {
                poseController.update();
                
                telemetry.addData("Target", "X:%.1f Y:%.1f", target.x, target.y);
                telemetry.addData("Current", "X:%.1f Y:%.1f", 
                                 positionManager.getCurrentX(),
                                 positionManager.getCurrentY());
                telemetry.update();
            }
        }
    }
}
```

---

## Summary

✅ **PathPlanner** converts sparse waypoints into detailed, smooth paths  
✅ **Three modes**: STRAIGHT, SMOOTH_CURVE, CUBIC_SPLINE  
✅ **Configurable**: spacing, smoothing, heading interpolation  
✅ **Utilities**: length calculation, simplification, reversal  
✅ **Easy integration** with PoseController  

**Next step:** Integrate PathPlanner with PoseController's waypoint queue system for seamless autonomous navigation!

