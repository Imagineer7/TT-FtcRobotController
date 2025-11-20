# FTC DECODE Robot Codebase - Copilot Instructions (VALIDATED)

## ⚠️ IMPORTANT: Read This First

This document has been **validated through actual repository exploration and testing**. All file paths, class names, and build instructions have been verified against the actual codebase as of the creation of this document.

**Trust these instructions.** Only perform searches if information is incomplete or you discover it to be incorrect.

---

## Repository Overview

This is the **2025-2026 FTC DECODE season** robot controller codebase for team TT (Imagineer7). The project features the **AURORA (Advanced Unified Robot Operating & Response Architecture)** framework - a sophisticated, competition-ready system for controlling an FTC robot.

### Quick Facts
- **SDK Version**: FTC SDK 11.0.0 (DECODE season)
- **Language**: Java
- **Build System**: Gradle 8.9 with Android Gradle Plugin 8.7.0
- **Compile SDK**: Android 34 (API Level 34)
- **Min SDK**: Android 24 (API Level 24, Android 7.0)
- **Java Files**: 54 total in TeamCode
- **Project Type**: Android application (Robot Controller app)

### Dependencies
- **FTC SDK Components**: Inspection, Blocks, RobotCore, Hardware, Vision (11.0.0)
- **Pedro Pathing**: v2.0.4 (autonomous path following library)
- **Sloth Framework**: v0.2.4 (advanced robot control framework)
- **AndroidX**: AppCompat 1.2.0

---

## Build and Validation Instructions

### Prerequisites

**Required Software**:
1. **Android Studio Ladybug (2024.2) or later** - Required for development
2. **JDK 17** - Specified in gradle wrapper, auto-downloaded
3. **Android SDK** - Install via Android Studio
   - Compile SDK 34 (Android 14)
   - Build Tools 34.0.0
4. **Internet Connection** - Required for first build to download dependencies

**Required for First Build**:
- The project MUST have internet access on first build to download:
  - Android Gradle Plugin 8.7.0 from dl.google.com
  - FTC SDK libraries from Maven repositories
  - Pedro Pathing from https://mymaven.bylazar.com/releases
  - Sloth framework from https://repo.dairy.foundation/releases

### Build Commands

#### Gradle Wrapper Verification
```bash
./gradlew --version
```
**Expected Output**:
```
Gradle 8.9
Build time:    2024-07-11 14:37:41 UTC
Kotlin:        1.9.23
Groovy:        3.0.21
JVM:           17.0.17 (or compatible)
```

#### Build the Project
```bash
# Build debug APK (Robot Controller app)
./gradlew assembleDebug

# Build release APK (requires signing configuration)
./gradlew assembleRelease

# Clean build artifacts
./gradlew clean
```

**⚠️ Known Issues**:
1. **Offline Mode Fails**: Running `--offline` will fail because Gradle needs to download plugins
2. **Network Required**: First build requires internet to download dependencies from:
   - google() repository (dl.google.com)
   - mavenCentral()
   - Custom maven repos (Pedro Pathing, Sloth)
3. **Build Time**: Initial build takes 2-5 minutes, subsequent builds are faster

### Testing

**⚠️ No Automated Tests**: This repository does not contain:
- No JUnit tests
- No instrumentation tests  
- No CI/CD pipelines
- No automated testing framework

**Manual Testing Process**:
1. Build the APK: `./gradlew assembleDebug`
2. Install APK to Robot Controller device (Control Hub or Android phone)
3. Connect Driver Station app
4. Select OpMode and run manual tests
5. Monitor telemetry output and robot behavior

### Linting

**No Lint Configuration**: The repository does not include:
- No checkstyle.xml
- No custom lint rules
- No code style enforcement

**Recommendation**: Follow the coding conventions documented in this file (see Coding Style section below).

### Directory Structure (VERIFIED)

```
TT-FtcRobotController/
├── .git/                               # Git repository
├── .github/
│   ├── CONTRIBUTING.md                 # Contribution guidelines (from FTC SDK)
│   ├── PULL_REQUEST_TEMPLATE.md
│   └── copilot-instructions.md         # This file
├── .gitignore
├── build.gradle                        # Root build configuration
├── build.common.gradle                 # Shared Android config (compileSdk 34)
├── build.dependencies.gradle           # FTC SDK + Pedro Pathing dependencies
├── settings.gradle                     # Multi-module project settings
├── gradlew                             # Gradle wrapper script (Unix)
├── gradlew.bat                         # Gradle wrapper script (Windows)
├── gradle.properties                   # Gradle configuration (1GB heap)
├── install-pedro-pathing.sh            # Pedro Pathing setup script
├── libs/
│   └── ftc.debug.keystore              # Debug signing key
├── LICENSE                             # Project license
├── README.md                           # FTC SDK documentation (130KB)
├── doc/                                # FTC SDK documentation
├── FtcRobotController/                 # FTC SDK Robot Controller module
└── TeamCode/                           # TEAM CODE (your code goes here)
    ├── build.gradle                    # TeamCode build config (Sloth plugin)
    ├── install_pedro_files.sh          # Pedro Pathing installer
    ├── lib/                            # External libraries
    └── src/main/
        ├── AndroidManifest.xml
        ├── java/org/firstinspires/ftc/teamcode/
        │   ├── *.java                  # 14 OpModes at root
        │   ├── util/                   # Utility classes
        │   │   ├── aurora/             # AURORA framework (13 files)
        │   │   │   ├── lightning/      # Advanced autonomous (7 files)
        │   │   │   └── vision/         # Vision systems (2 files)
        │   │   └── tool/               # General tools (7 files)
        │   ├── pedroPathing/           # Pedro Pathing config (3 files)
        │   ├── mechanisms/             # Hardware abstractions (1 file)
        │   ├── opmodes/                # Example OpModes (2 files)
        │   ├── webinterface/           # Web dashboard (3 files)
        │   └── examples/               # Sample programs (3 files)
        └── res/                        # Android resources
```

**Total Java Files in TeamCode**: 54

---

## Key File Locations (VERIFIED)

### Configuration Files
- **Gradle Build**: `build.gradle`, `build.common.gradle`, `build.dependencies.gradle`
- **Gradle Wrapper**: `gradlew`, `gradlew.bat`, `gradle/wrapper/gradle-wrapper.properties`
- **Gradle Properties**: `gradle.properties` (heap size, AndroidX settings)
- **Android Manifest**: `TeamCode/src/main/AndroidManifest.xml`
- **Debug Keystore**: `libs/ftc.debug.keystore`

### Core Framework Files
- **AuroraManager**: `util/aurora/AuroraManager.java` - Central robot coordinator
- **SmartMechanumDrive**: `util/aurora/SmartMechanumDrive.java` - Drive system
- **EnhancedDecodeHelper**: `util/aurora/EnhancedDecodeHelper.java` - Shooter system
- **SmartTelemetryManager**: `util/aurora/SmartTelemetryManager.java` - Telemetry UI
- **ShooterConfig**: `util/aurora/ShooterConfig.java` - Shooting presets
- **RpmLearningSystem**: `util/aurora/RpmLearningSystem.java` - ML optimization

### Autonomous Systems
**AURORA Lightning** (odometry-based):
- `util/aurora/lightning/AuroraLightningCore.java` - High-level control
- `util/aurora/lightning/PoseController.java` - PID position control
- `util/aurora/lightning/PositionManager.java` - Odometry management
- `util/aurora/lightning/OdoHelper.java` - Pinpoint driver wrapper
- `util/aurora/lightning/MotionProfiler.java` - Motion planning

**Pedro Pathing** (path-following):
- `pedroPathing/Constants.java` - Robot configuration
- `pedroPathing/PedroAutonomousBuilder.java` - Fluent builder for auto
- `pedroPathing/Tuning.java` - Tuning utilities

### OpModes (Verified List)
**TeleOp**:
- `AURORATeleOp.java` - Main TeleOp (dual-driver support)
- `ShooterTuningOpMode.java` - Live shooter calibration
- `ShooterMLTrainingOpMode.java` - ML training mode

**AURORA Autonomous** (4 files):
- `BLUEAutoAurora.java` - Blue alliance, standard start
- `REDAutoAurora.java` - Red alliance, standard start
- `LONGBLUEAR.java` - Blue alliance, long range start
- `LONGREDAR.java` - Red alliance, long range start

**Pedro Pathing Autonomous** (6 files):
- `BlueShortRange.java` - Blue alliance, short range
- `BlueLongRange.java` - Blue alliance, long range
- `RedShortRange.java` - Red alliance, short range
- `RedLongRange.java` - Red alliance, long range
- `PedroAutonomous.java` - Basic Pedro example
- `AdvancedPedroAutonomous.java` - Advanced Pedro example

**Example OpModes**:
- `opmodes/LightningForward24Auto.java` - Simple forward movement test
- `opmodes/LightningSquarePathAuto.java` - Square path test
- `SensorGoBildaPinpointExample.java` - Odometry sensor test
- `examples/AprilTagStartPoseExample.java` - Vision example
- `examples/PathPlannerExample.java` - Path planning demo
- `examples/WebDashboardDemoOpMode.java` - Web interface demo

### Hardware Drivers
- **GoBilda Pinpoint**: `util/tool/GoBildaPinpointDriver.java` - Odometry computer
- **AprilTag Webcam**: `mechanisms/AprilTagWebcam.java` - Vision camera
- **AprilTag Detector**: `util/aurora/vision/AuroraAprilTagDetector.java`
- **AprilTag Localizer**: `util/aurora/vision/AuroraAprilTagLocalizer.java`

### Utility Classes
- `util/MechanumDrive.java` - Legacy basic mecanum
- `util/MechanumFieldRelative.java` - Legacy field-centric drive
- `util/tool/DeadWheelOdometry.java` - Odometry math
- `util/tool/FieldMap.java` - Field coordinate system
- `util/tool/Pose.java` - Position/orientation data structure
- `util/tool/PathPlanner.java` - Path generation
- `util/tool/PathPlanningSystem.java` - Path execution

---

## Subsystems and Architecture

### 1. AuroraManager (Central Coordinator)

**File**: `util/aurora/AuroraManager.java`

**Purpose**: Unified system manager coordinating all robot subsystems.

**Responsibilities**:
- Initialize and manage all subsystems (drive, shooter, sensors)
- Handle driver mode switching (single vs dual driver)
- Emergency stop protocols
- Performance monitoring
- Graceful degradation when hardware fails

**Key Methods**:
```java
AuroraManager(HardwareMap hwMap, Telemetry telemetry)  // Initialize all systems
void update(Gamepad gamepad1, Gamepad gamepad2)        // Main update loop
void setDriverMode(DriverMode mode)                     // Switch driver modes
SmartMechanumDrive getDriveSystem()                     // Get drive subsystem
EnhancedDecodeHelper getShooterSystem()                 // Get shooter subsystem
boolean isSystemsHealthy()                              // Check system status
void cleanup()                                          // Safe shutdown
```

**Driver Modes**:
- `SINGLE_DRIVER`: Gamepad1 controls everything (default)
- `DUAL_DRIVER`: Gamepad1=movement, Gamepad2=shooter/tools

**Hardware Initialization Pattern**:
```java
// Graceful error handling - robot continues with partial functionality
try {
    shooter = hardwareMap.get(DcMotor.class, "shooter");
    shooterInitialized = true;
} catch (Exception e) {
    shooter = null;
    shooterInitialized = false;
    telemetry.addLine("⚠️ Shooter not found");
    // Robot can still drive without shooter
}
```

### 2. SmartMechanumDrive (Drive System)

**File**: `util/aurora/SmartMechanumDrive.java`

**Purpose**: Intelligent mecanum drive with power optimization and multiple modes.

**Drive Modes**:
- `PRECISION` - 0.3 max power, slow and accurate
- `NORMAL` - 0.8 max power, balanced (default)
- `SPORT` - 1.0 max power, maximum speed
- `EFFICIENCY` - 0.6 max power, battery-saving
- `AUTO_ADAPTIVE` - 1.0 max power, AI-optimized

**Features**:
- Battery voltage compensation (power scales up as voltage drops)
- Smooth acceleration limiting (configurable ramp rate)
- Field-relative and robot-relative modes
- Fine movement control (D-pad precision adjustments)
- Performance analytics

**Key Methods**:
```java
void update()                                    // Main drive loop
void setDriveMode(DriveMode mode)                // Change drive mode
void setFieldRelative(boolean enabled)           // Toggle field-centric
void setRobotHeading(double heading)             // Update for field-relative
void setDriveInputs(double axial, double lateral, double yaw)  // Manual control
void setFineMovement(double x, double y, double rot)           // Precision adjustments
void stop()                                      // Emergency stop
void enableAccelerationLimiting(boolean enable)  // Toggle smooth acceleration
```

**Motor Configuration** (IMPORTANT):
```java
// Left motors are REVERSED, right motors are FORWARD
leftFront.setDirection(DcMotor.Direction.REVERSE);
leftBack.setDirection(DcMotor.Direction.REVERSE);
rightFront.setDirection(DcMotor.Direction.FORWARD);
rightBack.setDirection(DcMotor.Direction.FORWARD);
```

**Hardware Names** (must match robot configuration):
- `frontLeft` - Front left motor
- `frontRight` - Front right motor
- `backLeft` - Back left motor
- `backRight` - Back right motor

### 3. EnhancedDecodeHelper (Shooter System)

**File**: `util/aurora/EnhancedDecodeHelper.java`

**Purpose**: Advanced shooter with RPM control, ML optimization, and shot compensation.

**Shooting Modes**:
- **Single Shot**: Fire one artifact with precise RPM control
- **Continuous Shooting**: Rapid fire with automatic timing
- **Warmup Mode**: Run at 65% RPM (power saving while ready)
- **Manual Power**: Direct motor control (bypass RPM)

**Features**:
- Closed-loop RPM control with adaptive PID
- ML-based shot compensation (learns optimal boost parameters)
- RPM overshoot detection and prevention
- Battery voltage compensation
- Configurable shooting presets

**Key Methods**:
```java
EnhancedDecodeHelper(HardwareMap hwMap)                      // Basic init
EnhancedDecodeHelper(HardwareMap hwMap, boolean useOdometry) // With position tracking
void update()                                     // Main shooter loop (CALL EVERY CYCLE)
void startShooter()                               // Start flywheel
void stopShooter()                                // Stop flywheel
void singleShot()                                 // Fire one artifact
void continuousShooting(boolean enable)           // Enable/disable rapid fire
void setWarmupMode(boolean enable)                // Toggle warmup (power saving)
void setManualPower(double power)                 // Direct motor control
void setConfig(ShooterConfig config)              // Change shooting preset
double getCurrentRPM()                            // Read flywheel speed
boolean isRpmStable()                             // Check if ready to shoot
void resetStats()                                 // Clear performance counters
void saveMlData()                                 // Store ML learning data
void resetMlToDefaults()                          // Clear ML learning
```

**ShooterConfig Presets** (`util/aurora/ShooterConfig.java`):
```java
LONG_RANGE:     85% power, 4000 RPM, 1.0s interval  // Far shots
SHORT_RANGE:    100% power, 2780 RPM, 0.15s interval // Fast close shots
RAPID_FIRE:     80% power, 3900 RPM, 0.3s interval  // Balanced
PRECISION:      90% power, 4800 RPM, 2.0s interval  // Accurate
BATTERY_SAVER:  65% power, 3000 RPM, 1.8s interval  // Efficient
```

**Hardware Names** (must match robot configuration):
- `shooter` - Flywheel motor (DC motor with encoder)
- `feedServo1` - First feeder servo (continuous rotation)
- `feedServo2` - Second feeder servo (continuous rotation)
- `light` - RGB indicator servo (optional)

**PID Constants**:
```java
// Normal operation
RPM_KP = 0.0003, RPM_KI = 0.00001, RPM_KD = 0.0001

// Recovery from RPM drop
RPM_KP_RECOVERY = 0.0005, RPM_KI_RECOVERY = 0.00002, RPM_KD_RECOVERY = 0.00015
```

### 4. Odometry and Position Tracking

**GoBilda Pinpoint Driver** (`util/tool/GoBildaPinpointDriver.java`):
- Hardware interface for Pinpoint odometry computer
- Returns Pose2D (x, y, heading) in millimeters and radians
- **Configuration** (IMPORTANT):
  ```java
  X_OFFSET = -154mm  // Forward pod is 154mm BEHIND robot center
  Y_OFFSET = 0mm     // Strafe pod is at robot center
  ```
- **Hardware Name**: `"odo"` in robot configuration

**PositionManager** (`util/aurora/lightning/PositionManager.java`):
- Manages odometry updates and position tracking
- Sensor fusion: IMU + Pinpoint
- Provides field-relative positioning

**PoseController** (`util/aurora/lightning/PoseController.java`):
- PID-based closed-loop position control
- Smooth motion profiles (acceleration/deceleration)
- Waypoint navigation with automatic advancement
- Overshoot detection and correction

**PID Tuning Constants** (for autonomous):
```java
// Position control
KP_POSITION = 0.08, KI_POSITION = 0.001, KD_POSITION = 0.02

// Heading control
KP_HEADING = 0.025, KI_HEADING = 0.0005, KD_HEADING = 0.008
```

**Tolerances**:
- Position: 1.0 inches
- Heading: 2.0 degrees
- Max move time: 5.0 seconds (timeout protection)

### 5. SmartTelemetryManager (Paginated Telemetry)

**File**: `util/aurora/SmartTelemetryManager.java`

**Purpose**: Organized, paginated telemetry display for Driver Station.

**Telemetry Pages**:
1. **Overview**: System status, battery, uptime, critical warnings
2. **Drive**: Movement metrics, drive mode, field position
3. **Shooter**: RPM, shots fired, success rate, ML data
4. **Performance**: CPU usage, loop times, error counts
5. **Controls**: Button mappings and help text

**Navigation**:
- D-pad Right: Next page
- D-pad Left: Previous page (implicit)

**Key Methods**:
```java
void update(Telemetry telemetry)              // Refresh current page
void nextPage()                               // Navigate to next page
void previousPage()                           // Navigate to previous page
String getCurrentPage()                       // Get active page name
void setCustomData(String key, String value)  // Add custom telemetry
```

---

## TeleOp Control Scheme (AURORATeleOp.java)

### Driver Mode Toggle
**Back + D-pad Left (Gamepad1)**: Switch between single/dual driver modes

### Single Driver Mode (Default)
**Gamepad 1 - All Controls**

**Movement**:
- Left Stick Y: Forward/Backward (axial)
- Left Stick X: Strafe Left/Right (lateral)
- Right Stick X: Rotation (yaw)
- D-pad: Fine XY movement (20% power)
- Left/Right Bumpers: Fine rotation (20% power)

**Drive Modes**:
- Left Stick Button: Toggle precision mode (30% max power)
- X Button: Toggle field-relative driving

**Shooter Controls**:
- A Button: Single shot
- Y Button: Continuous shooting (rapid fire)
- Right Trigger: Manual shooter power
- B Button: Manual feed / Reset stats
- X Button: Emergency stop shooting

**System**:
- D-pad Right: Cycle telemetry pages
- Back: Cancel semi-auto actions

### Dual Driver Mode
**Gamepad 1 - Driver (Movement)**
- All movement controls (same as single driver)
- Semi-auto actions
- Drive mode toggles
- System controls

**Gamepad 2 - Operator (Shooter)**
- Left Trigger: Warmup mode (65% RPM, power saving)
- A Button: Single shot
- Y Button: Continuous shooting
- Right Trigger: Manual shooter power
- B Button: Manual feed / Reset stats
- X Button: Emergency stop
- Left Bumper: Save ML learning data
- Right Bumper (hold 2s): Reset ML to defaults

**Emergency Stop**: Both Start buttons (gamepad1.start && gamepad2.start)

---

## Autonomous Strategies

### Two Frameworks Available

The team uses **two distinct autonomous frameworks**:

1. **AURORA Lightning** (odometry-based) - Direct motor control with PID
2. **Pedro Pathing** (path-following) - Smooth bezier paths

### AURORA Lightning Autonomous

**Files**: `BLUEAutoAurora.java`, `REDAutoAurora.java`, `LONGBLUEAR.java`, `LONGREDAR.java`

**OpMode Type**: `LinearOpMode` with `@Autonomous` annotation

**Approach**: Direct odometry control with robot-relative movement functions.

**Movement Functions**:
```java
void moveForward(double inches)      // Move forward relative to robot
void moveBackward(double inches)     // Move backward relative to robot
void moveLeft(double inches)         // Strafe left relative to robot
void moveRight(double inches)        // Strafe right relative to robot
void turnToAngle(double degrees)     // Turn to absolute field angle
void turnRelative(double degrees)    // Turn relative to current heading
void moveToPosition(double x, double y, double heading)  // Absolute positioning
void strafeToPosition(double x, double y)                // Strafe while maintaining heading
```

**Typical Sequence**:
```java
// 1. Initialize odometry at starting pose
odometry.resetPosAndIMU();

// 2. Leave LAUNCH LINE (autonomous bonus points)
moveForward(24);

// 3. Navigate to shooting position
strafeToPosition(targetX, targetY);
turnToAngle(targetHeading);

// 4. Shoot pre-loaded artifacts
shooter.setConfig(ShooterConfig.ShooterPreset.SHORT_RANGE);
shooter.singleShot();
sleep(200);
shooter.singleShot();

// 5. Park in BASE for endgame points
moveToPosition(baseX, baseY, 0);
```

**Sensors Used**:
- GoBilda Pinpoint odometry (primary positioning)
- IMU (heading correction)
- Motor encoders (wheel position)

### Pedro Pathing Autonomous

**Files**: `RedShortRange.java`, `BlueShortRange.java`, `RedLongRange.java`, `BlueLongRange.java`, `PedroAutonomous.java`, `AdvancedPedroAutonomous.java`

**OpMode Type**: `OpMode` (iterative) with `@Autonomous` annotation

**Approach**: Path-following with BezierLine and BezierCurve paths.

**Key Components**:
- `Follower` - Main path-following controller (from Pedro library)
- `PathChain` - Sequence of path segments
- `PedroAutonomousBuilder` - Fluent builder for autonomous sequences

**OpMode Structure**:
```java
@Autonomous(name="...", group="Pedro Autonomous")
public class MyAuto extends OpMode {
    private Follower follower;
    private PedroAutonomousBuilder autoBuilder;
    
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        
        autoBuilder = new PedroAutonomousBuilder(follower)
            .withShooter(shooter)
            .addPath(pathChain)
            .addTurnToHeading(Math.toRadians(45))
            .addShootAction(3, ShooterConfig.ShooterPreset.SHORT_RANGE)
            .addWait(0.5);
    }
    
    @Override
    public void start() {
        autoBuilder.start();
    }
    
    @Override
    public void loop() {
        follower.update();     // MUST be called BEFORE builder.update()
        autoBuilder.update();
        telemetry.update();
    }
}
```

**PedroAutonomousBuilder Actions**:
```java
.addPath(pathChain)                                        // Follow a path
.addTurnToHeading(radians)                                 // Turn in place
.addShootAction(count, preset)                             // Fire shots
.addWait(seconds)                                          // Pause
.addCustomAction(Runnable)                                 // Custom code
```

**Configuration** (`pedroPathing/Constants.java`):
- Follower PID tuning
- Mecanum drivetrain configuration
- Pinpoint odometry settings (X_OFFSET=-154mm, Y_OFFSET=0mm)
- Path constraints (max speed, acceleration, deceleration)

**Advantages**:
- Smooth, predictable motion
- Less tuning required (library handles control)
- Visual path planning tools available
- Better for complex curved trajectories

**Disadvantages**:
- External dependency
- Less direct control over motion
- Learning curve for path API

### When to Use Which Framework

**Use AURORA Lightning when**:
- Simple straight-line movements
- Precise position control needed
- Incremental improvements to existing Aurora code
- Direct odometry access needed
- Full control over PID tuning required

**Use Pedro Pathing when**:
- Smooth curved paths required
- Complex multi-segment routes
- Rapid autonomous development needed
- Competition-ready reliability is priority

---

## Coding Style and Conventions

### Naming Conventions

**Classes** (PascalCase):
```java
AuroraManager           // Subsystem managers
SmartMechanumDrive      // Drive systems
EnhancedDecodeHelper    // Helper classes
BLUEAutoAurora          // OpModes (Alliance + Strategy)
```

**Methods** (camelCase):
```java
update()                 // Update loops
getCurrentRPM()          // Getters
setDriveMode()           // Setters
isRpmStable()            // Boolean checks (is/has prefix)
enableFeature()          // Enable/disable actions
```

**Variables** (camelCase):
```java
currentPower            // Current state
targetRPM               // Target values
shooterRunning          // Boolean flags
```

**Constants** (UPPER_SNAKE_CASE):
```java
MAX_POWER               // Maximum values
KP_POSITION             // PID constants
LIGHT_GREEN             // Color constants
```

**Hardware Names** (camelCase - must match robot config):
```java
frontLeft               // Motors
backRight               // Motors
feedServo1              // Servos
odo                     // Sensors
```

### OpMode Structure

**LinearOpMode Template** (TeleOp and most Autonomous):
```java
@TeleOp(name="My OpMode", group="Competition")  // or @Autonomous
public class MyOpMode extends LinearOpMode {
    // 1. Hardware and subsystems
    private AuroraManager robotManager;
    private ElapsedTime runtime = new ElapsedTime();
    
    // 2. State variables
    private boolean previousButtonState = false;
    
    @Override
    public void runOpMode() {
        // 3. Initialization
        telemetry.addLine("Initializing...");
        robotManager = new AuroraManager(hardwareMap, telemetry);
        telemetry.update();
        
        // 4. Wait for start
        waitForStart();
        runtime.reset();
        
        // 5. Main loop
        while (opModeIsActive()) {
            // Update subsystems
            robotManager.update(gamepad1, gamepad2);
            
            // Custom logic with edge detection
            if (gamepad1.a && !previousButtonState) {
                // Button just pressed
            }
            previousButtonState = gamepad1.a;
            
            // Telemetry (at end of loop)
            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
        
        // 6. Cleanup
        robotManager.cleanup();
    }
}
```

**Iterative OpMode Template** (Pedro Pathing):
```java
@Autonomous(name="My Auto", group="Pedro Autonomous")
public class MyAuto extends OpMode {
    private Follower follower;
    private PedroAutonomousBuilder autoBuilder;
    
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        autoBuilder = new PedroAutonomousBuilder(follower)
            .addPath(myPath);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    
    @Override
    public void start() {
        autoBuilder.start();
    }
    
    @Override
    public void loop() {
        follower.update();           // ALWAYS call before builder.update()
        autoBuilder.update();
        telemetry.update();
    }
}
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
ElapsedTime moveTimer = new ElapsedTime();
double MAX_MOVE_TIME = 5.0;

moveTimer.reset();
while (opModeIsActive() && !atTarget() && moveTimer.seconds() < MAX_MOVE_TIME) {
    // Movement logic
    updateMotors();
    sleep(10);
}

if (moveTimer.seconds() >= MAX_MOVE_TIME) {
    telemetry.addLine("⚠️ Move timeout!");
    stopMotors();
}
```

### Telemetry Best Practices

**Formatting**:
```java
telemetry.addData("Label", "Value");
telemetry.addData("Number", "%.2f", doubleValue);  // 2 decimal places
telemetry.addLine("Status message");
telemetry.addLine();  // Blank line for spacing
telemetry.update();   // ALWAYS call at end of loop
```

**Icons** (use for visual clarity):
- ✅ Success/Active
- ⚠️ Warnings
- ❌ Errors
- 🎮 Controls
- 🔋 Battery
- 📊 Statistics

---

## Guidelines for Future Changes

### Adding a New OpMode

**Steps**:
1. Create `.java` file in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
2. Add `@TeleOp` or `@Autonomous` annotation
3. Extend `LinearOpMode` (recommended) or `OpMode` (for Pedro)
4. Follow existing structure (init, waitForStart, loop, cleanup)
5. Use `AuroraManager` for TeleOp or existing autonomous framework
6. Test on actual robot before competition

**TeleOp Example**:
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
            telemetry.update();
        }
        
        robotManager.cleanup();
    }
}
```

### Modifying Subsystems

**Before Modifying**:
1. Understand subsystem's role in overall architecture
2. Check dependencies (what calls this class?)
3. Review existing method signatures and contracts
4. Test changes in isolation first

**SmartMechanumDrive Modifications**:
- New drive mode: Add to `DriveMode` enum with max power and description
- Tuning: Adjust `ACCELERATION_LIMIT`, `LOW_VOLTAGE_THRESHOLD`
- Features: Add methods, maintain existing API

**EnhancedDecodeHelper Modifications**:
- New preset: Add to `ShooterConfig.ShooterPreset` enum
- PID tuning: Modify `RPM_KP`, `RPM_KI`, `RPM_KD` constants
- Always test RPM stability after PID changes

### Tuning PID Constants

**Drive PID** (Aurora autonomous):
- Location: `BLUEAutoAurora.java`, `REDAutoAurora.java`, etc.
- Start with P-only, add D for damping, add I for steady-state error
- Test iteratively with small increments

**Shooter RPM PID**:
- Location: `EnhancedDecodeHelper.java`
- Use `ShooterTuningOpMode.java` for live tuning
- Monitor RPM stability with telemetry

**Pedro Pathing PID**:
- Location: `pedroPathing/Constants.java`
- Use Pedro's built-in tuning OpModes
- Adjust follower constants and path constraints

### Debugging Tips

**Common Issues**:
1. **"Can't find motor 'frontLeft'"**
   - Check hardware config names match code exactly
   - Verify spelling and capitalization

2. **Robot drifts during autonomous**
   - Recalibrate odometry offsets (-154mm, 0mm)
   - Check motor directions
   - Verify encoder connections

3. **Shooter RPM unstable**
   - Tune PID gains
   - Check battery voltage (>11.5V recommended)
   - Verify encoder is counting

4. **Gamepad not responding**
   - Check driver mode (single vs dual)
   - Verify gamepad assignment (1 vs 2)
   - Check for button conflicts

5. **OpMode crashes on init**
   - Check hardware initialization error handling
   - Verify all try/catch blocks
   - Check hardware config file

**Debugging Tools**:
- `telemetry.addData()` - Print values to Driver Station
- `RobotLog.dd(TAG, message)` - Log to logcat (view with adb logcat)
- `PerformanceMonitor` - Track loop times, errors
- `ShooterTuningOpMode` - Live shooter calibration

---

## Important Hardware Configuration

### Motor Directions (CRITICAL)
```java
// In SmartMechanumDrive.java
leftFront.setDirection(DcMotor.Direction.REVERSE);
leftBack.setDirection(DcMotor.Direction.REVERSE);
rightFront.setDirection(DcMotor.Direction.FORWARD);
rightBack.setDirection(DcMotor.Direction.FORWARD);
```
**If robot moves incorrectly**, swap REVERSE ↔ FORWARD for affected motors.

### Odometry Calibration (CRITICAL)
```java
// X_OFFSET: Forward pod is 154mm BEHIND robot center
// Y_OFFSET: Strafe pod is at robot center
odometry.setOffsets(-154, 0);  // in millimeters
```
**If localization is wrong**, re-measure offsets from robot center to odometry pods.

### Hardware Names (Must Match Robot Config)
- **Drive Motors**: `frontLeft`, `frontRight`, `backLeft`, `backRight`
- **Shooter**: `shooter` (DC motor with encoder)
- **Feed Servos**: `feedServo1`, `feedServo2` (continuous rotation)
- **Odometry**: `odo` (GoBilda Pinpoint)
- **Light**: `light` (optional RGB indicator servo)

---

## Known Limitations and Constraints

### Build System
1. **Internet Required**: First build MUST have internet access
2. **No Offline Build**: `--offline` flag will fail due to missing dependencies
3. **Build Time**: Initial build takes 2-5 minutes
4. **Gradle Version**: Requires Gradle 8.9 (auto-downloaded)

### Testing
1. **No Automated Tests**: No JUnit, no instrumentation tests
2. **Manual Testing Only**: Must test on actual robot hardware
3. **No CI/CD**: No automated build/test pipelines

### Development Environment
1. **Android Studio Required**: Cannot build with VS Code or IntelliJ IDEA
2. **Android SDK Required**: Must install Android SDK via Android Studio
3. **JDK 17**: Project uses JDK 17 (auto-configured by Gradle)

### Network Dependencies
The build system requires access to:
- `dl.google.com` - Android Gradle Plugin
- `repo1.maven.org` - Maven Central (AndroidX, etc.)
- `mymaven.bylazar.com` - Pedro Pathing libraries
- `repo.dairy.foundation` - Sloth framework

### FTC Competition Rules
1. **SDK Version**: Must use FTC-approved SDK (currently 11.0.0)
2. **Hardware**: Must use FTC-legal hardware (Control Hub, approved motors/sensors)
3. **OpMode Requirements**: Must follow FTC OpMode structure
4. **Inspection**: Robot must pass pre-match inspection

---

## Machine Learning System

**Location**: `util/aurora/RpmLearningSystem.java` + `EnhancedDecodeHelper.java`

The ML system learns optimal shot compensation parameters:
- `shotBoostDelay` - Time after feed servos fire before boost
- `shotBoostDuration` - How long to apply extra power
- `shotBoostPower` - Extra power percentage during boost

**Training Mode**: Enable in `ShooterMLTrainingOpMode.java`
- Fires shots and analyzes RPM recovery
- Adjusts parameters based on performance
- Stores learned values to file

**Production Mode**: Disabled by default in `AURORATeleOp`
- Uses pre-trained values from configuration
- Can save/reset via gamepad controls (GP2: LB=save, RB hold=reset)

**Data Storage**: `/sdcard/FIRST/shooter_learning_data.txt`

---

## Battery Management

**Voltage Compensation**: Enabled by default
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

This is a **competition-ready FTC DECODE robot codebase** with:
- ✅ Well-organized package structure
- ✅ Graceful error handling
- ✅ Dual autonomous frameworks (AURORA + Pedro)
- ✅ Advanced shooter control with ML
- ✅ Flexible driver configurations

**When making changes**:
1. Follow existing naming conventions and structure
2. Test incrementally on actual robot hardware
3. Maintain backward compatibility where possible
4. Document new features in comments and telemetry
5. Consider impact on both TeleOp and Autonomous
6. Always handle errors gracefully (null checks, try/catch)

**Remember**: Prioritize reliability and consistency over experimental features during competition season. Save experimental work for post-season or practice robots.

---

## Quick Reference Card

### File Locations
- **OpModes**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/*.java`
- **AURORA Framework**: `util/aurora/`
- **Pedro Pathing**: `pedroPathing/`
- **Build Config**: `build.gradle`, `build.common.gradle`, `build.dependencies.gradle`

### Build Commands
```bash
./gradlew --version              # Check Gradle version
./gradlew assembleDebug          # Build Robot Controller APK
./gradlew clean                  # Clean build artifacts
```

### Hardware Config Names
- Drive: `frontLeft`, `frontRight`, `backLeft`, `backRight`
- Shooter: `shooter`, `feedServo1`, `feedServo2`, `light`
- Odometry: `odo`

### PID Constants
- Position: KP=0.08, KI=0.001, KD=0.02
- Heading: KP=0.025, KI=0.0005, KD=0.008
- RPM: KP=0.0003, KI=0.00001, KD=0.0001

### Odometry Offsets
- X_OFFSET = -154mm (forward pod behind center)
- Y_OFFSET = 0mm (strafe pod at center)

Good luck in the DECODE season! 🤖🏆
