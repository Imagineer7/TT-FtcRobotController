# Copilot Instructions for FTC Robot Controller (DECODE 2025-2026)

**Version:** 11.0 (SDK) / Aurora System v2  
**Competition Season:** DECODE 2025-2026  
**Last Updated:** January 2025

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Critical Information for AI Agents](#critical-information-for-ai-agents)
3. [Architecture Overview](#architecture-overview)
4. [Hardware Configuration (65+ Devices)](#hardware-configuration)
5. [Package Structure](#package-structure)
6. [Core Systems Reference](#core-systems-reference)
7. [Development Guidelines](#development-guidelines)
8. [Common Tasks](#common-tasks)
9. [Known Issues & Warnings](#known-issues--warnings)
10. [Quick Reference](#quick-reference)

---

## Project Overview

This is an FTC (FIRST Tech Challenge) Robot Controller for the **DECODE (2025-2026)** season, featuring the **Aurora System v2** architecture.

### Key Specifications
- **Platform:** Android (SDK 24+), FTC SDK 11.0
- **Build:** Gradle 7.x, Android Gradle Plugin 8.1.0  
- **Language:** Java 8
- **IDE:** Android Studio Ladybug 2024.2+
- **Hardware:** REV Control Hub + Expansion Hub(s)

### Robot Capabilities
1. **Mecanum Drive** - Field-centric omnidirectional movement
2. **Dual Shooter** - Variable RPM flywheel (2000-3400 RPM)
3. **Indexing System** - Push-based 3-artifact sorting
4. **Shot Planning** - AI-optimized firing sequences
5. **Vision** - Limelight 3A AprilTag localization
6. **Odometry** - GoBILDA Pinpoint precise positioning

---

## Critical Information for AI Agents

### 🚨 DO NOT MODIFY These Files

1. **`AuroraHardwareConfig.java`** 
   - 65+ hardware device mappings
   - Device names MUST match Driver Station config EXACTLY
   - Requires physical robot reconfiguration if changed

2. **`IndexingSystem.java`** (1800+ lines)
   - Complex 7-state machine
   - Push-based mechanical constraints
   - Timing-critical servo operations
   - Battle-tested over months

3. **`ShotPlanner.java` + `PlannerExecutor.java`**
   - Mathematical shot optimization
   - 7 comprehensive test cases
   - Physical constraints encoded

4. **`FiringSequenceCoordinator.java`**
   - 7-gate safety system
   - Manual override detection
   - Cancellation logic

5. **`DecodeHelper.java`**
   - PID-tuned RPM control
   - RPM stability algorithms
   - Safety interlocks

### ⚠️ Modify With Caution
- `ShooterConfig.java` - timing constants (document changes)
- `IndexingConfig.java` - parameters (test thoroughly)
- `GamepadConfig.java` - button mappings
- OpModes in `/opmodes` - generally safe

### ✅ Safe to Modify
- Test OpModes (`/opmode/test`)
- Utility classes (`/util/tool`, `/util/debug`)
- Documentation (`.md` files)

---

## Architecture Overview

```
OpMode Layer (TeleOp/Autonomous)
    ↓
Subsystem Layer (Shooter, IndexingSystem, Drive, Turret, ShotPlanner)
    ↓
Hardware Abstraction (AuroraHardwareConfig)
    ↓
FTC SDK (DcMotor, Servo, Sensors)
```

### Design Principles
1. **Separation of Concerns** - Self-contained subsystems
2. **Single Source of Truth** - Centralized hardware config
3. **State Machines** - Discrete states for complex logic
4. **Safety First** - Multiple gating rules
5. **Testable** - Independent component testing

### Key Decisions

**Push-Based Indexing** (not pull)
- 1st artifact → center
- 2nd artifact → pushes 1st to opposite intake (unless manual mode)
- 3rd artifact → stays in collection intake
- *Why?* Mechanical simplicity, reliability

**Shot Planning Split**
- **ShotPlanner** - Pure logic (WHAT to do)
- **PlannerExecutor** - Physical execution (WHEN/HOW)
- *Why?* Planning runs every loop, execution respects hardware state

**Configuration Objects**
- ShooterConfig, IndexingConfig, GamepadConfig
- *Why?* Easy tuning, no code changes needed

---

## Hardware Configuration

### Complete Device List (65+)

#### Drive System (4 motors)
```java
// Access: hardware.getFrontLeftMotor()
FRONT_LEFT_MOTOR = "Left Front"        // REV HD Hex, REVERSED, BRAKE
FRONT_RIGHT_MOTOR = "Right Front"      // FORWARD, BRAKE
BACK_LEFT_MOTOR = "Left Back"          // REVERSED, BRAKE  
BACK_RIGHT_MOTOR = "Right Back"        // FORWARD, BRAKE
// All: RUN_WITHOUT_ENCODER (for odometry)
```

#### Shooter System (2 motors + servo)
```java
// Access: hardware.getLeftShooterMotor()
LEFT_SHOOTER_MOTOR = "Shooter Front"   // REV HD Hex 20:1, REVERSED
RIGHT_SHOOTER_MOTOR = "Shooter Back"   // FORWARD
LIGHT_SERVO = "RGB Light Back"         // Optional LED
// Mode: RUN_USING_ENCODER, Zero Power: FLOAT
```

#### Turret (1 servo)
```java
// Access: hardware.getTurretServo()
TURRET_SERVO = "Turret Left"           // 0.0-1.0 range, calibrate at 0.5
```

#### Indexing System (2 motors + 8 servos)
```java
// Roller Motors
FRONT_ROLLER_MOTOR = "TopIntakeFront"
BACK_ROLLER_MOTOR = "TopIntakeBack"

// Bottom Intake Servos (assist rollers)
FRONT_BOTTOM_INTAKE_SERVO = "BottomIntakeFront"
BACK_BOTTOM_INTAKE_SERVO = "BottomIntakeBack"

// Transfer Servos (rollers → center)
FRONT_TRANSFER_SERVO = "TransferSystemFront"
BACK_TRANSFER_SERVO = "TransferSystemBack"

// Uptake Servos (center → shooter)
UPTAKE_SERVO_L = "UptakeTransferLeft"
UPTAKE_SERVO_R = "UptakeTransferRight"

// Injector Servos (transfer ↔ center)
INJECTOR_SERVO_LEFT = "InjectorSystemLeft"
INJECTOR_SERVO_RIGHT = "InjectorSystemRight"
```

#### Sensors (10 total)
```java
// Laser Distance (Analog 0-3.3V = 0-1000mm)
FRONT_DISTANCE_SENSOR = "LaserSensorFront"
BACK_DISTANCE_SENSOR = "LaserSensorBack"

// REV 2m Distance
FRONT_LEFT_DISTANCE_SENSOR = "DistSensorLeftFront"
BACK_RIGHT_DISTANCE_SENSOR = "DistSensorRightBack"

// Color Sensors (REV V3, Normalized RGB 0-1)
FRONT_LEFT_COLOR_SENSOR = "ColorSensorLeftFront"
FRONT_RIGHT_COLOR_SENSOR = "ColorSensorRightFront"
BACK_RIGHT_COLOR_SENSOR = "ColorSensorRightBack"
LEFT_RIGHT_COLOR_SENSOR = "ColorSensorLeftBack"
FRONT_CENTER_COLOR_SENSOR = "ColorSensorFront"
BACK_CENTER_COLOR_SENSOR = "ColorSensorBack"

// IMU & Odometry
IMU_SENSOR = "imu"                     // BNO055 or BHI260AP
ODOMETRY_COMPUTER = "OdometryPinpointComputer"  // GoBILDA Pinpoint
```

### Hardware Access Pattern
```java
// Initialize (once in init())
AuroraHardwareConfig hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
hardware.initializeWithOdometry();  // TeleOp
// OR hardware.initialize();  // Autonomous

// Access devices
DcMotor motor = hardware.getFrontLeftMotor();
Servo turret = hardware.getTurretServo();

// Check status
if (hardware.isDriveSystemInitialized()) { /* safe to use */ }
String summary = hardware.getInitializationSummary();
```

---

## Package Structure

### `/opmodes` - Primary OpModes
- `FullSystemTest.java` - Complete integration test
- `BasicMechanumDriveExample.java` - Simple drive
- `IndexingSystemTest.java` - Indexing testing
- `ShooterTestOpMode.java` - RPM testing
- `LocalizationTestOpMode.java` - Odometry validation

### `/util/aurora` - Core System (⚠️ CRITICAL)

**Hardware & Config:**
- `AuroraHardwareConfig.java` - Hardware mappings
- `ShooterConfig.java` - Shooter parameters
- `IndexingConfig.java` - Indexing parameters

**Subsystems:**
- `Shooter.java` - Shooter interface
- `DecodeHelper.java` - RPM control
- `IndexingSystem.java` - Artifact management
- `Turret.java` - Aiming control
- `IntakeController.java` - Intake motors

**Shot Planning:**
- `ShotPlanner.java` - Optimization logic
- `PlannerExecutor.java` - Physical execution
- `ShotPlannerTest.java` - 7 test cases

**Coordination:**
- `FiringSequenceCoordinator.java` - Safety coordinator
- `Artifact.java` - Artifact data class

**Drive:**
- `IntelMechanumDrive.java` - Field-centric drive
- `MechanumDriveSimple.java` - Basic drive

**Utilities:**
- `Localization.java` - Odometry wrapper
- `LimelightVisionHelper.java` - Vision integration
- `GamepadConfig.java` - Input mappings
- `SystemMonitor.java` - Performance tracking

### `/util/tool` - Shared Utilities
- `GoBildaPinpointDriver.java` - Odometry driver
- `FieldMap.java` - Field coordinates

### `/util/debug` - Debug Tools
- `DebugLogger.java` - Enhanced logging

---

## Core Systems Reference

### IndexingSystem (1800+ lines)

**States:**
```java
IDLE, COLLECTING, TRANSFERRING, PUSHING, READY_TO_FIRE, FIRING, ERROR
```

**Usage:**
```java
IndexingSystem indexing = new IndexingSystem(hardware, config, shooter, telemetry);
indexing.enable();

// In loop - CRITICAL
indexing.update();

// Check state
if (indexing.getCurrentState() == SystemState.READY_TO_FIRE) { /* ready */ }

// Get info
int count = indexing.getArtifactCount();
boolean hasCenter = indexing.hasArtifactInCenter();
```

**Firing Integration:**
```java
// Set manual detector (only relevant controls!)
indexing.setManualInputDetector(() -> 
    gamepad2.dpad_up || gamepad2.dpad_down  // Uptake only
);

FiringSequenceCoordinator firing = new FiringSequenceCoordinator(indexing, shooter);

// In loop
firing.update();

// Fire
if (gamepad1.x && firing.canStartFiring()) {
    firing.startFiring();
}
```

### Shooter System

**States:**
```java
IDLE, WARMUP, SPINNING_UP, READY, FIRING, RECOVERY, ERROR
```

**Usage:**
```java
Shooter shooter = new Shooter(hardware, config, telemetry);
shooter.enable();

// In loop
shooter.update();

// Spin up
shooter.spinUpToRPM(ShooterConfig.RPM_HIGH_BASKET);  // 3200 RPM

// Check ready
if (shooter.isReadyToFire()) { /* at target + stable */ }

// Get info
double rpm = shooter.getCurrentRPM();
ShooterState state = shooter.getState();
```

**RPM Presets:**
```java
RPM_HIGH_BASKET = 3200.0    // Long range
RPM_LOW_BASKET = 2800.0     // Medium
RPM_SPECIMEN = 2400.0       // Specimen
RPM_OBSERVATION = 2000.0    // Close
```

### Shot Planning

**Motif Patterns:** PPG, PGP, GPP (P=Purple, G=Green)

**Scoring:** 1st match = +3, 2nd = +2, 3rd = +1 points

**Usage:**
```java
// Set pattern
indexing.setMotifPattern("PPG");

// Auto-runs in indexing.update()

// Get plan
List<Artifact> plan = indexing.getShotPlanner().getShotPlan();

// Check rearrangement
boolean needsRearrange = planner.isRearrangementNeeded();
```

**Rearrangement Rules:**
- ✅ 2 artifacts, executor idle, not manual mode, READY_TO_FIRE
- ❌ 1 artifact (nothing to swap), 3 artifacts (no empty intake), busy, manual mode

### Drive System

```java
IntelMechanumDrive drive = new IntelMechanumDrive(hardware, telemetry);

// Robot-centric
drive.driveRobotCentric(gamepad1.left_stick_y, left_stick_x, right_stick_x);

// Field-centric (requires IMU)
drive.driveFieldCentric(left_stick_y, left_stick_x, right_stick_x, imuHeading);

// Reset field-centric
drive.resetFieldCentric();
```

### Localization

```java
Localization loc = new Localization(hardware, telemetry);

// In loop
loc.update();

// Get position
double x = loc.getX();         // inches
double y = loc.getY();
double heading = loc.getHeading();  // radians

// Reset/set
loc.reset(0, 0, 0);
loc.setPosition(x, y, heading);  // From AprilTag
```

---

## Development Guidelines

### Coding Standards
```java
// Classes: PascalCase
public class IndexingSystem { }

// Methods: camelCase
public void updateShotPlanner() { }

// Constants: UPPER_SNAKE_CASE
public static final double RPM_HIGH_BASKET = 3200.0;

// Fields: camelCase
private SystemState currentState;
```

### OpMode Structure
```java
@TeleOp(name="My OpMode", group="Testing")
public class MyOpMode extends LinearOpMode {
    
    // 1. Declarations
    private AuroraHardwareConfig hardware;
    private IndexingSystem indexing;
    
    @Override
    public void runOpMode() {
        // 2. Initialize
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        waitForStart();
        
        // 3. Main loop
        while (opModeIsActive()) {
            indexing.update();  // CRITICAL
            // ... rest of code
            telemetry.update();
        }
    }
}
```

### Common Patterns

**Button Edge Detection:**
```java
boolean current = gamepad1.x;
if (current && !lastX) {
    performAction();  // Just pressed
}
lastX = current;
```

**State Machine:**
```java
switch (state) {
    case IDLE:
        if (shouldStart()) {
            state = OPERATING;
            startTime = System.currentTimeMillis();
        }
        break;
    case OPERATING:
        if (isComplete()) state = IDLE;
        else if (isTimeout()) state = ERROR;
        break;
    case ERROR:
        handleError();
        if (shouldRecover()) state = IDLE;
        break;
}
```

**Timeout Protection:**
```java
long start = System.currentTimeMillis();
while ((System.currentTimeMillis() - start) < 3000) {
    if (complete()) return true;
}
telemetry.addData("ERROR", "Timeout");
return false;
```

---

## Common Tasks

### Add New OpMode
```java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.util.aurora.*;

@TeleOp(name="My OpMode", group="Testing")
public class MyOpMode extends LinearOpMode {
    private AuroraHardwareConfig hardware;
    
    @Override
    public void runOpMode() {
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Your code
            telemetry.update();
        }
    }
}
```

### Add New Subsystem
```java
package org.firstinspires.ftc.teamcode.util.aurora;

public class MySubsystem {
    private final AuroraHardwareConfig hardware;
    private final Telemetry telemetry;
    private boolean enabled;
    
    public MySubsystem(AuroraHardwareConfig hardware, Telemetry telemetry) {
        this.hardware = hardware;
        this.telemetry = telemetry;
    }
    
    public void enable() { enabled = true; }
    public void disable() { enabled = false; }
    
    public void update() {
        if (!enabled) return;
        // Update logic
    }
    
    public void doSomething() {
        if (!enabled) {
            telemetry.addData("WARN", "Not enabled");
            return;
        }
        // Implementation
    }
}
```

### Tune Shooter RPM
```java
// 1. Modify ShooterConfig.java
public static final double RPM_MY_PRESET = 3000.0;

// 2. Test with ShooterTestOpMode
shooter.spinUpToRPM(RPM_MY_PRESET);
telemetry.addData("Current", shooter.getCurrentRPM());
telemetry.addData("Target", shooter.getTargetRPM());
telemetry.addData("Ready", shooter.isReadyToFire());
```

---

## Known Issues & Warnings

### Critical Issues

**1. IndexingSystem State Corruption**
- **Cause:** Not calling `indexingSystem.update()` every loop
- **Solution:** ALWAYS call in loop, no exceptions

**2. Servo Glitches on Stop**
- **Cause:** Servos not stopped explicitly
- **Solution:**
```java
@Override
public void stop() {
    hardware.getUptakeServoL().setPower(0);
    hardware.getUptakeServoR().setPower(0);
    // ... all servos
    super.stop();
}
```

**3. Hardware Not Initialized**
- **Cause:** Accessing before initialization
- **Solution:**
```java
if (hardware.isDriveSystemInitialized()) {
    // Safe to use
} else {
    telemetry.addData("ERROR", "Not initialized");
}
```

**4. USB Communication Race**
- **Issue:** Rapid disconnect/reconnect causes errors
- **Solution:** Let system auto-recover (built-in)

**5. Shooter RPM Instability**
- **Causes:** Low battery, friction, bad PIDF
- **Solution:**
```java
double voltage = hardware.getVoltageSensor().getVoltage();
if (voltage < 12.0) {
    telemetry.addData("WARN", "Low battery: " + voltage + "V");
}
```

### Key Assumptions

1. **Device names match Driver Station EXACTLY** (case-sensitive)
2. **Update order:** indexing → shooter → firingCoordinator
3. **Max 3 artifacts** (system enforces)
4. **Timing is critical** (don't modify without testing)
5. **Manual mode overrides automation**

### Gotchas

**Java vs Blocks coordinates:**
```java
// Java: Y-forward = NEGATIVE joystick
double forward = -gamepad1.left_stick_y;  // Note negation
```

**LinearOpMode vs OpMode:**
```java
// LinearOpMode: runOpMode() with while loop
@Override
public void runOpMode() {
    waitForStart();
    while (opModeIsActive()) { }
}

// OpMode: loop() called automatically
@Override
public void loop() {
    // Called repeatedly
}
```

---

## Quick Reference

### Essential Init
```java
AuroraHardwareConfig hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
hardware.initializeWithOdometry();

IndexingSystem indexing = new IndexingSystem(hardware, config, shooter, telemetry);
Shooter shooter = new Shooter(hardware, shooterConfig, telemetry);
FiringSequenceCoordinator firing = new FiringSequenceCoordinator(indexing, shooter);

indexing.enable();
shooter.enable();
```

### Essential Loop
```java
while (opModeIsActive()) {
    indexing.update();   // CRITICAL
    shooter.update();
    firing.update();
    
    // Your code
    
    telemetry.update();
}
```

### Debug Logging
```java
import org.firstinspires.ftc.teamcode.util.debug.DebugLogger;

DebugLogger logger = new DebugLogger(telemetry);
logger.info("Initialized");
logger.warn("Low battery");
logger.error("Sensor failed");
logger.debug("RPM: " + rpm);
```

### Build Commands
```bash
# Build APK
./gradlew :FtcRobotController:assembleDebug

# TeamCode only (faster)
./gradlew :TeamCode:compileDebugJavaWithJavac

# Clean
./gradlew clean
```

### Critical Rules

1. ✅ ALWAYS call `.update()` every loop
2. ✅ Check initialization before use
3. ✅ Test with hardware
4. ✅ Document changes
5. ❌ Don't modify core files
6. ❌ Don't skip timeout protection
7. ❌ Don't bypass safety checks
8. ❌ Don't modify timing without testing

---

## Documentation References

### In-Repository
- `FIRING_SYSTEM_USAGE.md` - Complete firing guide
- `SHOT_PLANNER_IMPLEMENTATION.md` - Shot planning details
- `TeamCode/AURORA_SYSTEM_README.md` - System overview
- `TeamCode/AURORA_HARDWARE_CONFIG_GUIDE.md` - Hardware setup
- `TeamCode/INDEXING_SYSTEM_GUIDE.md` - Indexing deep dive

### External
- FTC Docs: https://ftc-docs.firstinspires.org/
- FTC Javadoc: https://javadoc.io/doc/org.firstinspires.ftc
- REV Docs: https://docs.revrobotics.com/
- Game Manual: https://www.firstinspires.org/ftc

---

## Code Review Checklist

When reviewing/generating code:

- [ ] Hardware initialized before use
- [ ] `.update()` called for all subsystems every loop
- [ ] Error handling for hardware failures
- [ ] Telemetry for debugging
- [ ] Button edge detection (not hold)
- [ ] Timeout protection
- [ ] Manual mode respected
- [ ] State machine logic sound
- [ ] Constants used (no magic numbers)
- [ ] Comments for complex logic
- [ ] JavaDoc for public methods
- [ ] No modifications to critical files

### Red Flags
- ❌ Hardware access without init check
- ❌ Missing `.update()` calls
- ❌ Infinite loop without `opModeIsActive()` 
- ❌ Hardcoded timing values
- ❌ Direct servo/motor control bypassing systems
- ❌ Modified device names in AuroraHardwareConfig
- ❌ Removed safety checks
- ❌ Changed scoring algorithms

### Success Indicators
- ✅ Clear initialization
- ✅ Proper update loop
- ✅ Comprehensive telemetry
- ✅ Error handling with logging
- ✅ Consistent naming
- ✅ Comments for non-obvious logic
- ✅ Uses config objects
- ✅ Respects subsystem interfaces
- ✅ Tested with hardware
- ✅ Documentation updated

---

**Remember:** This robot represents months of refinement and testing. Respect the architecture, test thoroughly, and document all changes. When uncertain, ask before modifying core systems.

**Competition:** DECODE 2025-2026  
**Last Updated:** January 2025
