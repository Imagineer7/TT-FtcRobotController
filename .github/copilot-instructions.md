# FTC DECODE Robot Codebase - Copilot Instructions

## Repository Overview

This is a **2025-2026 FTC DECODE season** robot codebase featuring the **AURORA (Advanced Unified Robot Operating & Response Architecture)** framework. The robot is designed for scoring ARTIFACTS (purple and green game pieces) into GOALS, building PATTERNS on RAMPS based on MOTIF, and returning to BASE for endgame points.

### Project Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── AURORATeleOp.java                    # Primary TeleOp with dual-driver support
├── ShooterTuningOpMode.java             # Shooter calibration/tuning
├── ShooterMLTrainingOpMode.java         # ML-based shooter optimization
├── [Alliance][Range]AR.java             # AURORA odometry-based autonomous (4 files)
├── [Alliance][Range]Range.java          # Pedro Pathing autonomous (4 files)
├── AdvancedPedroAutonomous.java         # Advanced Pedro example
├── PedroAutonomous.java                 # Basic Pedro example
├── util/
│   ├── aurora/                          # AURORA framework (core subsystems)
│   │   ├── AuroraManager.java           # Central robot coordinator
│   │   ├── SmartMechanumDrive.java      # Intelligent drive system
│   │   ├── EnhancedDecodeHelper.java    # Shooter/feeder control
│   │   ├── ShooterConfig.java           # Shooting presets
│   │   ├── ShooterBoostConfig.java      # ML shot compensation
│   │   ├── RpmLearningSystem.java       # ML RPM optimization
│   │   ├── SmartTelemetryManager.java   # Paginated telemetry
│   │   ├── PerformanceMonitor.java      # System analytics
│   │   ├── MovementRecorder.java        # Movement logging
│   │   ├── lightning/                   # Advanced autonomous systems
│   │   │   ├── AuroraLightningCore.java # High-level auto control
│   │   │   ├── PoseController.java      # PID position control
│   │   │   ├── PositionManager.java     # Odometry management
│   │   │   ├── OdoHelper.java           # Pinpoint odometry wrapper
│   │   │   ├── MotionProfiler.java      # Movement profiling
│   │   │   ├── EventLogger.java         # Event tracking
│   │   │   └── PerformanceMonitor.java  # Performance metrics
│   │   └── vision/                      # Vision systems
│   │       ├── AuroraAprilTagDetector.java
│   │       └── AuroraAprilTagLocalizer.java
│   ├── tool/                            # Utility tools
│   │   ├── GoBildaPinpointDriver.java   # Odometry hardware driver
│   │   ├── PathPlanner.java             # Path generation
│   │   ├── PathPlanningSystem.java      # Path execution
│   │   ├── FieldMap.java                # Field coordinate system
│   │   ├── Pose.java                    # Position/orientation
│   │   └── DeadWheelOdometry.java       # Odometry calculations
│   ├── MechanumDrive.java               # Basic mecanum (legacy)
│   └── MechanumFieldRelative.java       # Field-centric drive (legacy)
├── pedroPathing/                        # Pedro Pathing integration
│   ├── Constants.java                   # Pedro configuration
│   ├── PedroAutonomousBuilder.java      # Fluent auto builder
│   └── Tuning.java                      # Pedro tuning utilities
├── mechanisms/                          # Hardware abstractions
│   └── AprilTagWebcam.java             # Vision target detection
├── opmodes/                             # Additional OpModes
│   ├── LightningForward24Auto.java
│   └── LightningSquarePathAuto.java
├── webinterface/                        # Web dashboard customization
└── examples/                            # Sample programs
```

### Key Configuration Files

- **Hardware Config Name**: Expects motors named `frontLeft`, `frontRight`, `backLeft`, `backRight`
- **Odometry Device**: `"odo"` (GoBilda Pinpoint driver)
- **Shooter Motor**: `"shooter"` (single DC motor)
- **Feed Servos**: `"feedServo1"`, `"feedServo2"` (continuous rotation)
- **Light Servo**: `"light"` (RGB indicator)

---

## Subsystems and Responsibilities

### 1. AuroraManager (Central Coordinator)

**File**: `util/aurora/AuroraManager.java`

**Purpose**: Unified robot system manager that coordinates all subsystems and handles driver mode switching.

**Key Features**:
- Dual-driver mode support (single driver or driver + operator)
- Cross-subsystem coordination
- Emergency protocols
- Performance analytics
- Graceful degradation if subsystems fail to initialize

**Important Methods**:
- `AuroraManager(HardwareMap, Telemetry)` - Initialize all subsystems
- `update(Gamepad, Gamepad)` - Main update loop (call every cycle)
- `setDriverMode(DriverMode)` - Switch between SINGLE_DRIVER and DUAL_DRIVER
- `getDriveSystem()` - Get SmartMechanumDrive instance
- `getShooterSystem()` - Get EnhancedDecodeHelper instance
- `isSystemsHealthy()` - Check if robot is operational
- `cleanup()` - Safe shutdown of all systems

**Driver Modes**:
- `SINGLE_DRIVER`: Gamepad1 controls everything
- `DUAL_DRIVER`: Gamepad1 = movement/semi-auto, Gamepad2 = shooter/tools

### 2. SmartMechanumDrive (Drive System)

**File**: `util/aurora/SmartMechanumDrive.java`

**Purpose**: Intelligent mecanum drive with battery optimization, acceleration limiting, and multiple drive modes.

**Drive Modes**:
- `PRECISION` (0.3 max power) - Slow, accurate movements
- `NORMAL` (0.8 max power) - Balanced speed and control
- `SPORT` (1.0 max power) - Maximum speed and agility
- `EFFICIENCY` (0.6 max power) - Battery-optimized
- `AUTO_ADAPTIVE` (1.0 max power) - AI-optimized based on conditions

**Key Features**:
- Dynamic power scaling based on battery voltage
- Smooth acceleration curves (configurable)
- Field-relative and robot-relative drive modes
- Fine movement control (D-pad + bumpers)
- Driver performance analytics

**Important Methods**:
- `update()` - Main drive update loop
- `setDriveMode(DriveMode)` - Change drive mode
- `setFieldRelative(boolean)` - Toggle field-centric driving
- `setRobotHeading(double)` - Update heading for field-relative
- `setDriveInputs(double, double, double)` - Manual control (axial, lateral, yaw)
- `setFineMovement(double, double, double)` - Fine adjustments
- `stop()` - Emergency stop all motors
- `enableAccelerationLimiting(boolean)` - Toggle smooth acceleration

### 3. EnhancedDecodeHelper (Shooter System)

**File**: `util/aurora/EnhancedDecodeHelper.java`

**Purpose**: Advanced shooter control with RPM-based shooting, machine learning optimization, and shot compensation.

**Shooting Modes**:
- **Single Shot**: Fire one artifact with precise RPM control
- **Continuous Shooting**: Rapid fire with automatic spacing
- **Warmup Mode**: Run at 65% RPM to save battery while ready
- **Manual Power**: Direct motor control (bypass RPM)

**Key Features**:
- Closed-loop RPM control with adaptive PID
- Shot compensation boost (learned from data)
- RPM overshoot detection and prevention
- Battery voltage compensation
- Configurable shooting presets (SHORT_RANGE, LONG_RANGE, RAPID_FIRE, etc.)
- ML-based shot optimization (RpmLearningSystem)

**Important Methods**:
- `EnhancedDecodeHelper(HardwareMap)` - Initialize shooter
- `EnhancedDecodeHelper(HardwareMap, boolean)` - With odometry tracking
- `update()` - Main shooter loop (handles RPM control)
- `startShooter()` / `stopShooter()` - Basic controls
- `singleShot()` - Fire one shot
- `continuousShooting(boolean)` - Enable/disable rapid fire
- `setWarmupMode(boolean)` - Toggle warmup (power saving)
- `setManualPower(double)` - Direct motor control
- `setConfig(ShooterConfig)` - Change shooting preset
- `getCurrentRPM()` - Read flywheel speed
- `isRpmStable()` - Check if ready to shoot
- `resetStats()` - Clear performance counters
- `saveMlData()` - Store learning data
- `resetMlToDefaults()` - Clear ML learning

**ShooterConfig Presets** (`util/aurora/ShooterConfig.java`):
- `LONG_RANGE`: 85% power, 4000 RPM, 1.0s interval
- `SHORT_RANGE`: 100% power, 2780 RPM, 0.15s interval (fast)
- `RAPID_FIRE`: 80% power, 3900 RPM, 0.3s interval (balanced)
- `PRECISION`: 90% power, 4800 RPM, 2.0s interval (accurate)
- `BATTERY_SAVER`: 65% power, 3000 RPM, 1.8s interval (efficient)

### 4. SmartTelemetryManager (Paginated Telemetry)

**File**: `util/aurora/SmartTelemetryManager.java`

**Purpose**: Organized, paginated telemetry display for managing complex robot data.

**Telemetry Pages**:
1. **Overview**: System status, battery, uptime, critical warnings
2. **Drive**: Movement metrics, drive mode, field position
3. **Shooter**: RPM, shots fired, success rate, ML data
4. **Performance**: CPU usage, loop times, error counts
5. **Controls**: Button mappings and help text

**Important Methods**:
- `update(Telemetry)` - Refresh current page
- `nextPage()` / `previousPage()` - Navigate pages
- `getCurrentPage()` - Get active page name
- `setCustomData(String, String)` - Add custom telemetry items

### 5. Odometry & Autonomous Systems

#### GoBildaPinpointDriver (`util/tool/GoBildaPinpointDriver.java`)
- Hardware interface for Pinpoint odometry computer
- Configured with X_OFFSET = -154mm, Y_OFFSET = 0mm
- Returns Pose2D (x, y, heading) in field coordinates

#### PositionManager (`util/aurora/lightning/PositionManager.java`)
- Manages odometry updates and position tracking
- Integrates IMU + Pinpoint sensor fusion
- Provides field-relative positioning

#### PoseController (`util/aurora/lightning/PoseController.java`)
- PID-based closed-loop position control
- Smooth motion profiles (acceleration/deceleration)
- Waypoint navigation with automatic advancement
- Overshoot detection and correction

#### AuroraLightningCore (`util/aurora/lightning/AuroraLightningCore.java`)
- High-level autonomous control system
- Integrates PoseController + PositionManager + PathPlanner
- Waypoint-based navigation
- Non-blocking operation

#### Pedro Pathing (`pedroPathing/`)
- External path-following library (alternative to Lightning)
- BezierLine and BezierCurve path generation
- PathChain for complex sequences
- `PedroAutonomousBuilder` for fluent path construction

### 6. Vision Systems (`util/aurora/vision/`)

- `AuroraAprilTagDetector.java`: AprilTag detection
- `AuroraAprilTagLocalizer.java`: Field localization using tags

---

## TeleOp Behavior and Strategy

### Primary TeleOp OpMode: `AURORATeleOp.java`

**Annotation**: `@TeleOp(name="AURORA Enhanced TeleOp", group="Competition")`

**Driver Mode Toggle**: Back + D-pad Left (Gamepad1)

### Single Driver Mode (Default)
**Gamepad 1 - All Controls**

**Movement**:
- Left Stick: Forward/backward + strafe (axial/lateral)
- Right Stick: Rotation (yaw)
- D-pad: Fine XY movement (20% power)
- Left/Right Bumpers: Fine rotation (20% power)

**Drive Modes**:
- Left Stick Button: Toggle precision mode (30% max power)
- X Button: Toggle field-relative driving

**Semi-Auto Actions** (coordinated movement + shooting):
- Y Button: Basket approach + auto-shoot sequence
- B Button: Precision positioning
- A Button: Retreat to safe zone
- Triggers: Auto-rotate/align to targets

**Shooter Controls**:
- A Button: Single shot (uses selected range preset)
- Y Button: Continuous shooting (rapid fire)
- Right Trigger: Manual shooter power
- B Button: Manual feed servo / Reset stats
- X Button: Emergency stop shooting

**System**:
- D-pad Right: Cycle telemetry pages
- Back: Cancel semi-auto actions

### Dual Driver Mode
**Gamepad 1 - Driver (Movement & Semi-Auto)**
- All movement controls (same as single driver)
- All semi-auto actions (Y, B, A, triggers)
- Drive mode toggles (precision, field-relative)
- System controls (mode toggle, telemetry)

**Gamepad 2 - Operator (Shooter & ML)**
- Left Trigger: Warmup mode (65% RPM, power saving)
- A Button: Single shot
- Y Button: Continuous shooting
- Right Trigger: Manual shooter power
- B Button: Manual feed / Reset stats
- X Button: Emergency stop
- Left Bumper: Save ML learning data
- Right Bumper (hold 2s): Reset ML to defaults

**Emergency Stop**: Both Start buttons (gamepad1.start && gamepad2.start)

### TeleOp Strategy

The robot's TeleOp strategy emphasizes:
1. **Fast Artifact Cycling**: Short interval shooting (0.15s) for rapid scoring
2. **Dual-Driver Efficiency**: Driver focuses on navigation while operator manages shooter
3. **Power Management**: Warmup mode saves battery between scoring runs
4. **Adaptive Shooting**: ML system learns optimal shot compensation over time
5. **Field Awareness**: Field-relative drive for intuitive control
6. **Semi-Auto Assists**: Automated sequences (basket approach, precision position) reduce driver workload
7. **Real-time Telemetry**: Paginated display shows critical info without clutter

---

## Autonomous Behavior and Strategy

The team uses **two distinct autonomous frameworks**: AURORA Lightning (odometry-based) and Pedro Pathing (path-following).

### AURORA Lightning Autonomous OpModes

**Files**: `BLUEAutoAurora.java`, `REDAutoAurora.java`, `LONGBLUEAR.java`, `LONGREDAR.java`

**Annotation**: `@Autonomous(name="...", group="Timed Autonomous")`

**Approach**: Direct odometry control with robot-relative movement functions.

**Key Movement Functions**:
- `moveForward(inches)` - Move forward relative to robot
- `moveBackward(inches)` - Move backward relative to robot
- `moveLeft(inches)` - Strafe left relative to robot
- `moveRight(inches)` - Strafe right relative to robot
- `turnToAngle(degrees)` - Turn to absolute field angle
- `turnRelative(degrees)` - Turn relative to current heading
- `moveToPosition(x, y, heading)` - Absolute field positioning
- `strafeToPosition(x, y)` - Strafe while maintaining heading

**Features**:
- PID-based position control
- Smooth acceleration/deceleration profiles
- Overshoot detection and correction
- Automatic course correction during movement
- Position tolerance: 1.0 inches, Heading tolerance: 2.0 degrees
- Max move time: 5.0 seconds with timeout protection

**PID Tuning Constants**:
```java
KP_POSITION = 0.08, KI_POSITION = 0.001, KD_POSITION = 0.02
KP_HEADING = 0.025, KI_HEADING = 0.0005, KD_HEADING = 0.008
```

**Typical Sequence** (example from BLUEAutoAurora):
1. Initialize odometry at starting pose
2. Leave LAUNCH LINE (move forward)
3. Navigate to shooting position (strafe + turn)
4. Shoot pre-loaded artifacts
5. Navigate to artifact collection zone
6. Return to scoring position
7. Park in BASE for endgame points

**Sensors Used**:
- GoBilda Pinpoint odometry (primary positioning)
- IMU (heading fusion)
- Encoder-based motor control

### Pedro Pathing Autonomous OpModes

**Files**: `RedShortRange.java`, `BlueShortRange.java`, `RedLongRange.java`, `BlueLongRange.java`, `AdvancedPedroAutonomous.java`, `PedroAutonomous.java`

**Annotation**: `@Autonomous(name="...", group="Pedro Autonomous")`

**Approach**: Path-following with BezierLine and BezierCurve paths.

**Key Components**:
- `Follower` - Main path-following controller (from Pedro library)
- `PathChain` - Sequence of path segments
- `PedroAutonomousBuilder` - Fluent builder for autonomous sequences

**Path Types**:
- `BezierLine`: Straight line from point A to point B
- `BezierCurve`: Smooth curve through control points
- `BezierPath`: Complex multi-segment paths

**PedroAutonomousBuilder Actions**:
```java
.addPath(pathChain)               // Follow a path
.addTurnToHeading(radians)        // Turn in place
.addShootAction(count, preset)    // Fire shots
.addWait(seconds)                 // Pause
.addCustomAction(lambda)          // Custom code
```

**Example Sequence** (RedShortRange):
```java
autoBuilder = new PedroAutonomousBuilder(follower)
    .withShooter(shooter)
    .addPath(paths.Path1)                               // Move to position
    .addTurnToHeading(Math.toRadians(44))               // Aim at target
    .addShootAction(3, ShooterConfig.ShooterPreset.SHORT_RANGE)  // Fire 3 shots
    .addWait(0.5)                                       // Brief pause
    .addPath(paths.Path2);                              // Continue to next position
```

**Configuration** (`pedroPathing/Constants.java`):
- Follower PID tuning
- Mecanum drivetrain configuration
- Pinpoint odometry settings (matches Aurora: X_OFFSET=-154mm, Y_OFFSET=0mm)
- Path constraints (max speed, acceleration, deceleration)

**Advantages of Pedro**:
- Smooth, predictable paths
- Less tuning required (library handles control loops)
- Visual path planning tools
- Better for complex curved trajectories

### Autonomous Strategy Summary

**Primary Goals**:
1. **Leave LAUNCH LINE**: Gain autonomous points (required)
2. **Score Pre-loaded Artifacts**: Shoot into GOAL for CLASSIFIED/OVERFLOW points
3. **Build PATTERN on RAMP**: Follow MOTIF for bonus points (if time permits)
4. **Park in BASE**: Endgame points for partial/full BASE return

**Strategic Choices**:
- **Short Range** paths prioritize speed, close-to-basket scoring
- **Long Range** paths navigate around obstacles for optimal positioning
- Both systems use same odometry hardware (Pinpoint) for consistency
- Shooter presets matched to path (SHORT_RANGE vs LONG_RANGE)
- Alliance-specific starting positions and paths

**Sensor Integration**:
- Pinpoint odometry: Primary localization
- IMU: Heading correction
- AprilTags: Optional alignment assistance (not used in current OpModes)
- Color sensors: Could detect ARTIFACT color for PATTERN (not implemented)

---

## Coding Style and Conventions

### Naming Conventions

**Classes**:
- PascalCase: `AuroraManager`, `SmartMechanumDrive`
- Descriptive names indicating purpose
- OpModes: `[Alliance][Strategy]` (e.g., `BLUEAutoAurora`, `RedShortRange`)

**Methods**:
- camelCase: `update()`, `getCurrentRPM()`, `setDriveMode()`
- Action verbs: `start`, `stop`, `set`, `get`, `is`, `enable`
- Boolean methods start with `is`: `isRpmStable()`, `isSystemsHealthy()`

**Variables**:
- camelCase: `currentPower`, `targetRPM`, `shooterRunning`
- Constants: `UPPER_SNAKE_CASE`: `MAX_POWER`, `KP_POSITION`, `LIGHT_GREEN`
- Member variables: descriptive, not abbreviated
- Avoid single-letter names except loop counters

**Hardware Names** (configuration file):
- camelCase: `frontLeft`, `backRight`, `feedServo1`, `odo`
- Consistent naming across all OpModes

### Package Organization

```
org.firstinspires.ftc.teamcode/
├── [OpModes at root]              # All OpMode classes directly in teamcode
├── util/                          # Utility classes and subsystems
│   ├── aurora/                    # AURORA framework
│   │   ├── lightning/             # Advanced autonomous
│   │   └── vision/                # Vision processing
│   └── tool/                      # General-purpose tools
├── pedroPathing/                  # Pedro Pathing integration
├── mechanisms/                    # Hardware abstractions
├── opmodes/                       # Additional OpMode examples
└── webinterface/                  # Dashboard customization
```

### Code Structure Patterns

**OpMode Structure** (LinearOpMode):
```java
@TeleOp(name="...", group="...")  // or @Autonomous
public class MyOpMode extends LinearOpMode {
    // Hardware and subsystems
    private AuroraManager robotManager;
    private ElapsedTime runtime = new ElapsedTime();
    
    // State variables
    private boolean someState = false;
    
    @Override
    public void runOpMode() {
        // 1. Initialization
        telemetry.addLine("Initializing...");
        robotManager = new AuroraManager(hardwareMap, telemetry);
        telemetry.update();
        
        // 2. Wait for start
        waitForStart();
        runtime.reset();
        
        // 3. Main loop
        while (opModeIsActive()) {
            // Update subsystems
            robotManager.update(gamepad1, gamepad2);
            
            // Custom logic
            // ...
            
            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
        
        // 4. Cleanup
        robotManager.cleanup();
    }
}
```

**OpMode Structure** (Iterative OpMode for Pedro):
```java
@Autonomous(name="...", group="...")
public class MyOpMode extends OpMode {
    private Follower follower;
    private PedroAutonomousBuilder autoBuilder;
    
    @Override
    public void init() {
        // Initialize hardware
        follower = Constants.createFollower(hardwareMap);
        
        // Build autonomous sequence
        autoBuilder = new PedroAutonomousBuilder(follower)
            .addPath(...)
            .addShootAction(...);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    
    @Override
    public void start() {
        autoBuilder.start();
    }
    
    @Override
    public void loop() {
        follower.update();           // Required BEFORE builder update
        autoBuilder.update();
        
        // Telemetry
        telemetry.update();
    }
}
```

### Hardware Initialization Pattern

**Graceful Error Handling**:
```java
// Try to initialize with specific error messages
try {
    shooter = hardwareMap.get(DcMotor.class, "shooter");
    shooterInitialized = true;
} catch (Exception e) {
    shooter = null;
    shooterInitialized = false;
    telemetry.addLine("⚠️ Shooter not found: " + e.getMessage());
    telemetry.addLine("   Check hardware config for 'shooter'");
}

// Allow robot to run with partial functionality
if (!shooterInitialized && !driveInitialized) {
    telemetry.addLine("⚠️ WARNING: No major subsystems initialized");
} else {
    telemetry.addLine("✅ Robot ready (limited functionality)");
}
```

### Loop Pattern

**Main Loop Structure**:
```java
while (opModeIsActive()) {
    // 1. Read inputs
    double axial = -gamepad1.left_stick_y;
    double lateral = gamepad1.left_stick_x;
    
    // 2. Update subsystems (in order of dependency)
    robotManager.update(gamepad1, gamepad2);
    
    // 3. Custom logic
    if (gamepad1.a && !prevA) {
        // Button press detected
    }
    prevA = gamepad1.a;
    
    // 4. Telemetry (at end of loop)
    telemetry.addData("Runtime", runtime.toString());
    telemetry.update();
}
```

### Comments and Documentation

**JavaDoc Style** (for public methods):
```java
/**
 * Brief description of method
 *
 * Longer explanation if needed
 *
 * @param paramName Description of parameter
 * @return Description of return value
 */
public double getSomething(int paramName) {
    // ...
}
```

**Inline Comments**:
- Use sparingly, code should be self-documenting
- Explain "why", not "what"
- Complex algorithms or non-obvious logic deserve comments
- Use `//` for single-line, avoid `/* */` for inline

**Section Headers** (for long classes):
```java
// ======== Core Systems ========

// ======== State Management ========

// ======== Configuration ========
```

### Safety and Error Handling

**Always Check**:
- `opModeIsActive()` in loops
- Null checks for hardware that might fail to initialize
- Bounds checking for array/list access
- Division by zero

**Emergency Stop Pattern**:
```java
if (gamepad1.start && gamepad2.start) {
    emergencyStop = true;
    robotManager.stop();
    requestOpModeStop();
}
```

**Timeout Protection** (autonomous):
```java
moveTimer.reset();
while (opModeIsActive() && !atTarget() && moveTimer.seconds() < MAX_MOVE_TIME) {
    // Movement logic
}
if (moveTimer.seconds() >= MAX_MOVE_TIME) {
    telemetry.addLine("⚠️ Move timeout!");
}
```

### Telemetry Best Practices

**Formatting**:
```java
telemetry.addData("Label", "Value");
telemetry.addData("Number", "%.2f", doubleValue);  // 2 decimal places
telemetry.addLine("Status message");
telemetry.addLine();  // Blank line for spacing
telemetry.update();   // ALWAYS call update() at end of loop
```

**Use Icons**:
- ✅ for success/active
- ⚠️ for warnings
- ❌ for errors
- 🎮 for controls
- 🔋 for battery
- 📊 for stats

---

## Guidelines for Future Changes

### Adding a New OpMode

**Steps**:
1. Create new `.java` file in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
2. Add appropriate annotation: `@TeleOp` or `@Autonomous`
3. Extend `LinearOpMode` (for most cases) or `OpMode` (for Pedro)
4. Follow existing structure (init, waitForStart, loop, cleanup)
5. Use `AuroraManager` for TeleOp or existing autonomous framework
6. Test thoroughly before competition

**Example TeleOp**:
```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager;

@TeleOp(name="My New TeleOp", group="Competition")
public class MyNewTeleOp extends LinearOpMode {
    private AuroraManager robotManager;
    
    @Override
    public void runOpMode() {
        robotManager = new AuroraManager(hardwareMap, telemetry);
        
        waitForStart();
        
        while (opModeIsActive()) {
            robotManager.update(gamepad1, gamepad2);
            
            // Your custom logic here
            
            telemetry.update();
        }
        
        robotManager.cleanup();
    }
}
```

**Example Autonomous (Aurora Lightning)**:
```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

@Autonomous(name="My Auto", group="Timed Autonomous")
public class MyAutonomous extends LinearOpMode {
    private AuroraManager robotManager;
    private GoBildaPinpointDriver odometry;
    
    @Override
    public void runOpMode() {
        // Initialize
        robotManager = new AuroraManager(hardwareMap, telemetry);
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometry.setOffsets(-154, 0);  // X, Y offsets in mm
        odometry.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odometry.resetPosAndIMU();
        
        waitForStart();
        
        // Autonomous sequence
        moveForward(24);
        turnToAngle(90);
        moveLeft(12);
        // ...
    }
    
    // Copy movement functions from BLUEAutoAurora.java
    private void moveForward(double inches) {
        // Implementation...
    }
}
```

### Modifying Existing Subsystems

**Before Modifying**:
1. Understand the subsystem's role in the overall architecture
2. Check for dependencies (what other classes call this?)
3. Review existing method signatures and contracts
4. Test changes in isolation first

**SmartMechanumDrive Modifications**:
- New drive mode: Add enum value to `DriveMode` with max power and description
- Tuning: Adjust `ACCELERATION_LIMIT`, `LOW_VOLTAGE_THRESHOLD`, motor directions
- Features: Add methods, maintain existing API for backward compatibility

**EnhancedDecodeHelper Modifications**:
- New preset: Add to `ShooterConfig.ShooterPreset` enum
- PID tuning: Modify `RPM_KP`, `RPM_KI`, `RPM_KD` constants
- ML adjustment: Tune `learningRate`, `shotBoostDelay`, `shotBoostDuration`
- Always test RPM stability after changes

**AuroraManager Modifications**:
- New subsystem: Add member variable, initialize in constructor, call in `update()`
- Coordination: Add cross-system logic in `update()` method
- Error handling: Follow graceful degradation pattern (try/catch with warnings)

### Adding New Controls

**TeleOp Control Addition**:
1. Document in OpMode header comment (control scheme)
2. Implement state tracking (prevButton pattern for edge detection)
3. Update `SmartTelemetryManager` controls page
4. Test for conflicts with existing controls

**Edge Detection Pattern**:
```java
// Member variable
private boolean prevButtonA = false;

// In loop
if (gamepad1.a && !prevButtonA) {
    // Rising edge - button just pressed
    performAction();
}
prevButtonA = gamepad1.a;
```

### Integrating DECODE-Specific Behaviors

**Pattern Scoring** (not yet implemented):
- Add color sensor to detect ARTIFACT color (purple/green)
- Create `PatternBuilder` class in `util/aurora/`
- Read MOTIF from field (could use vision or predetermined)
- Build sequence to place artifacts on RAMP in correct PATTERN

**BASE Return** (partially implemented):
- Use odometry to navigate to BASE zones
- Add autonomous action for endgame BASE return
- Consider hang mechanism if robot has one

**ARTIFACT Management**:
- Track how many artifacts are loaded
- Prevent shooting when empty
- Add sensor to detect artifact presence

**Example Integration**:
```java
// In EnhancedDecodeHelper.java
private ColorSensor artifactSensor;
private int artifactsLoaded = 0;

public void updateArtifactCount() {
    if (artifactSensor.red() > threshold) {
        // Red artifact detected (or purple)
        artifactsLoaded++;
    }
}

public boolean canShoot() {
    return artifactsLoaded > 0 && isRpmStable();
}
```

### Improving Aiming/Trajectory

**Current System**: Fixed shooter presets based on distance estimation.

**Enhancements**:
1. **AprilTag Alignment**:
   - Use `util/aurora/vision/AuroraAprilTagLocalizer.java`
   - Auto-aim at GOAL based on AprilTag position
   - Calculate optimal heading and distance

2. **Dynamic RPM Calculation**:
   - Distance-based RPM formula: `RPM = f(distance, angle, batteryVoltage)`
   - Store lookup table or polynomial fit
   - Interpolate between presets

3. **Turret Control** (if hardware added):
   - Add servo for horizontal aiming
   - PID control to track moving target
   - Integrate with vision system

**Example**:
```java
// In EnhancedDecodeHelper.java
public double calculateRpmForDistance(double distanceInches) {
    // Linear interpolation between SHORT and LONG range
    if (distanceInches < 36) {
        return ShooterConfig.ShooterPreset.SHORT_RANGE.getTargetRPM();
    } else if (distanceInches > 72) {
        return ShooterConfig.ShooterPreset.LONG_RANGE.getTargetRPM();
    } else {
        double t = (distanceInches - 36) / (72 - 36);
        return lerp(SHORT_RANGE.getTargetRPM(), LONG_RANGE.getTargetRPM(), t);
    }
}
```

### Tuning PID Constants

**Drive PID** (Aurora autonomous):
- Located in: `BLUEAutoAurora.java`, `REDAutoAurora.java`, etc.
- Position: `KP_POSITION`, `KI_POSITION`, `KD_POSITION`
- Heading: `KP_HEADING`, `KI_HEADING`, `KD_HEADING`
- Test iteratively: start with P-only, add D for damping, add I for steady-state error

**Shooter RPM PID**:
- Located in: `EnhancedDecodeHelper.java`
- Normal: `RPM_KP`, `RPM_KI`, `RPM_KD`
- Recovery: `RPM_KP_RECOVERY`, `RPM_KI_RECOVERY`, `RPM_KD_RECOVERY`
- Use `ShooterTuningOpMode.java` for live tuning

**Pedro Pathing PID**:
- Located in: `pedroPathing/Constants.java`
- Follower constants: `followerConstants`
- Path constraints: `pathConstraints`
- Use Pedro's built-in tuning OpModes

### Adding Sensor Support

**Sensor Integration Pattern**:
1. Add hardware to configuration file
2. Create utility wrapper class in `util/tool/` or `mechanisms/`
3. Initialize in subsystem (e.g., `AuroraManager` or specific class)
4. Poll/update in main loop
5. Use for decision-making or telemetry

**Example: Distance Sensor**:
```java
// In AuroraManager.java
private DistanceSensor distanceSensor;

// In initializeSystems()
try {
    distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
    telemetry.addLine("✅ Distance sensor initialized");
} catch (Exception e) {
    distanceSensor = null;
    telemetry.addLine("⚠️ Distance sensor not found");
}

// In update()
if (distanceSensor != null) {
    double distance = distanceSensor.getDistance(DistanceUnit.INCH);
    // Use distance for collision avoidance, etc.
}
```

### Debugging Tips

**Common Issues**:
1. **"Can't find motor 'frontLeft'"**: Check hardware config names match code
2. **Robot drifts during autonomous**: Recalibrate odometry offsets, check motor directions
3. **Shooter RPM unstable**: Tune PID gains, check battery voltage, verify encoder counts
4. **Gamepad not responding**: Check driver mode, verify gamepad assignment (1 vs 2)
5. **OpMode crashes on init**: Check hardware initialization error handling

**Debugging Tools**:
- `telemetry.addData()` - Print values to Driver Station
- `RobotLog.dd(TAG, message)` - Log to logcat (view with adb)
- `PerformanceMonitor` - Track loop times, errors
- `ShooterTuningOpMode` - Live shooter calibration
- Pedro Tuning OpModes - Test path following

**Best Practices**:
- Add telemetry early and often
- Test one change at a time
- Use try/catch to prevent total failure
- Log errors with descriptive messages
- Comment out experimental code, don't delete

---

## Autonomous Framework Decision Guide

### When to Use AURORA Lightning

**Best For**:
- Simple straight-line movements
- Precise position control
- Incremental improvements to existing Aurora code
- Direct odometry access needed
- Full control over PID tuning

**Pros**:
- Fine-grained control over every movement
- Easier to debug (direct motor commands)
- Lightweight (no external library)
- Good for learning fundamentals

**Cons**:
- More code to write for complex paths
- Manual PID tuning required
- No built-in path planning

### When to Use Pedro Pathing

**Best For**:
- Smooth curved paths
- Complex multi-segment routes
- Rapid autonomous development
- Competition-ready reliability

**Pros**:
- Smooth, predictable motion
- Less tuning required
- Visual path planning
- Mature, tested library
- Active community support

**Cons**:
- External dependency
- Less control over low-level details
- Learning curve for path API
- Larger code footprint

**Recommendation**: Use Pedro Pathing for competition autonomous, Aurora Lightning for experimentation and learning.

---

## Important Notes

### Hardware Configuration

**Motor Directions** (in `SmartMechanumDrive.java`):
```java
leftFront.setDirection(DcMotor.Direction.REVERSE);
leftBack.setDirection(DcMotor.Direction.REVERSE);
// Right motors are FORWARD (default)
```
If robot moves incorrectly, swap REVERSE ↔ FORWARD for affected motors.

**Odometry Calibration** (in `pedroPathing/Constants.java` or Aurora code):
```java
X_OFFSET = -154mm  // Forward pod is 154mm BEHIND robot center
Y_OFFSET = 0mm     // Strafe pod is at robot center
```
If localization is wrong, re-measure offsets from robot center to odometry pods.

### Machine Learning System

**Location**: `util/aurora/RpmLearningSystem.java` + `EnhancedDecodeHelper.java`

The ML system learns optimal shot compensation parameters:
- `shotBoostDelay` - Time after feed servos fire before boost
- `shotBoostDuration` - How long to apply extra power
- `shotBoostPower` - Extra power percentage during boost

**Training Mode**: Enable in `ShooterMLTrainingOpMode.java`
- Fires shots and analyzes RPM recovery
- Adjusts parameters based on performance
- Stores learned values to file

**Production Mode**: ML disabled by default in `AURORATeleOp`
- Uses pre-trained values from configuration
- Can save/reset via gamepad controls (Gamepad2: LB=save, RB hold=reset)

**Files**: Learning data stored in `/sdcard/FIRST/shooter_learning_data.txt`

### Battery Management

**Voltage Compensation**: Enabled by default in `ShooterConfig` and `SmartMechanumDrive`
- Automatically increases motor power as battery drains
- Prevents performance degradation during match

**Low Voltage Detection**:
- Threshold: 10.5V
- Actions: Switch to efficiency mode, reduce max power, warn driver

**Best Practices**:
- Start matches with fully charged battery (>12.5V)
- Use warmup mode to conserve power between runs
- Monitor voltage telemetry during matches

---

## Summary

This FTC DECODE robot codebase features the **AURORA framework**, a sophisticated architecture for coordinating drive, shooter, and autonomous systems. The team has implemented two autonomous approaches (Aurora Lightning and Pedro Pathing) and uses advanced features like machine learning shot optimization, paginated telemetry, and dual-driver mode support.

**Key Strengths**:
- Well-organized package structure
- Graceful error handling
- Multiple autonomous strategies
- Advanced shooter control with ML
- Flexible driver configurations

**Areas for Enhancement** (future work):
- Pattern scoring based on MOTIF
- AprilTag-based auto-aim
- Dynamic RPM calculation
- Improved ARTIFACT tracking
- Automated endgame BASE return

When making changes:
1. Follow existing naming conventions and structure
2. Test incrementally and thoroughly
3. Maintain backward compatibility where possible
4. Document new features in comments and telemetry
5. Consider how changes affect both TeleOp and Autonomous
6. Always handle errors gracefully (null checks, try/catch)

**Remember**: This is a competition-ready codebase. Prioritize reliability and consistency over experimental features during competition season. Save experimental work for post-season or practice robots.

Good luck in the DECODE season! 🤖🏆
