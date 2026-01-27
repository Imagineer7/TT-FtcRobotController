# Copilot Instructions for FTC Robot Controller (DECODE 2025-2026)

**Version:** 11.0 (SDK) / Aurora System v3  
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

This is an FTC (FIRST Tech Challenge) Robot Controller for the **DECODE (2025-2026)** season, featuring the **Aurora System v3** architecture.

### Key Specifications
- **Platform:** Android (SDK 24+), FTC SDK 11.0
- **Build:** Gradle 7.x, Android Gradle Plugin 8.1.0  
- **Language:** Java 8
- **IDE:** Android Studio Ladybug 2024.2+
- **Hardware:** REV Control Hub + Expansion Hub(s)

### Robot Capabilities
1. **Mecanum Drive** - Field-centric omnidirectional movement
2. **Dual Shooter** - Variable RPM flywheel (2000-3400 RPM)
3. **Indexing System V3** - Slot-based artifact management with transactional operations
4. **Shot Planning** - AI-optimized firing sequences with automatic rearrangement
5. **Vision** - Limelight 3A AprilTag localization
6. **Odometry** - GoBILDA Pinpoint precise positioning

---

## Critical Information for AI Agents

### 🚨 DO NOT MODIFY These Files

1. **`AuroraHardwareConfig.java`** 
   - 65+ hardware device mappings
   - Device names MUST match Driver Station config EXACTLY
   - Requires physical robot reconfiguration if changed

2. **`IndexingSystemV3.java`** (1500+ lines)
   - Slot-based state management (CENTER, FRONT, BACK)
   - Transactional operations with atomic commits
   - Sensor fusion via IntakePerception
   - Battle-tested operation system

3. **`ShotPlanner.java` + `ShotPlanningCoordinator.java`**
   - Mathematical shot optimization
   - Automatic rearrangement logic
   - Physical constraints encoded

4. **`BasicFiringHelper.java` + `BasicIndexingHelper.java`**
   - Hardware abstraction layer
   - Timed movement coordination
   - Keep-alive burst firing

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
Subsystem Layer (Shooter, IndexingSystemV3, Drive, Turret, ShotPlanner)
    ↓
Hardware Abstraction (BasicIndexingHelper, BasicFiringHelper, AuroraHardwareConfig)
    ↓
FTC SDK (DcMotor, Servo, Sensors)
```

### Design Principles
1. **Separation of Concerns** - Self-contained subsystems
2. **Single Source of Truth** - SlotLedger for artifact tracking
3. **Transactional Operations** - Atomic commits on success
4. **Sensors as Hints** - Perception layer provides confidence levels, not absolute truth
5. **Safety First** - Watchdog enforcement, manual override detection
6. **Testable** - Independent component testing

### Key Decisions

**Slot-Based Indexing** (V3 Architecture)
- **SlotLedger** - Single source of truth with 3 slots: CENTER, FRONT, BACK
- **Operations as Transactions** - Collect, Transfer, Swap, Fire, Eject
- **Sensor Fusion** - IntakePerception provides confidence levels, not binary states
- **Atomic Commits** - Slot changes only occur on operation success
- *Why?* Prevents state corruption, enables reliable automation, testable

**Artifact Flow**
- 1st artifact → transferred to CENTER immediately (ready to fire)
- 2nd artifact → stays in intake unless shot planner says swap
- 3rd artifact → stays in intake (system full, max 3 artifacts)
- *Why?* Optimal shot planning while maintaining system capacity

**Shot Planning Integration**
- **ShotPlanner** - Pure logic (WHAT to do)
- **ShotPlanningCoordinator** - Execution coordination (WHEN/HOW)
- *Why?* Planning runs every loop, coordinator respects hardware state

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
- `IndexingSystemV3.java` - V3 artifact management (current)
- `IndexingSystemOld.java` - Legacy system (deprecated)
- `Turret.java` - Aiming control
- `IntakeController.java` - Intake motors

**V3 Components (in `/util/aurora/v3`):**
- `IndexingSystemV3.java` - Main controller
- `SlotLedger.java` - Slot state management
- `IntakePerception.java` - Sensor fusion per intake
- `OperationRunner.java` - Operation lifecycle
- `CollectOperation.java` - Collection logic
- `TransferOperation.java` - Transfer logic
- `SwapOperation.java` - Swap logic
- `FireOperation.java` - Firing logic
- `EjectOperation.java` - Ejection logic
- `PrepositionOperation.java` - Preposition logic
- `ArtifactIdentity.java` - Artifact data class
- `ShotPlanningCoordinator.java` - Planning coordinator
- `KeepAliveWatchdog.java` - Safety watchdog

**Helpers:**
- `BasicIndexingHelper.java` - Hardware abstraction for indexing
- `BasicFiringHelper.java` - Hardware abstraction for firing

**Shot Planning:**
- `ShotPlanner.java` - Optimization logic

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

### IndexingSystemV3 (1500+ lines)

**System States:**
```java
IDLE,                  // No artifacts, no operations
COLLECTING,            // Collecting artifact into intake
TRANSFERRING,          // Transferring artifact to center
READY_TO_FIRE,        // Center occupied, shooter ready, prepositioned
FIRING,                // Firing shot
REARRANGING,          // Swapping artifacts for optimal order
EJECTING,             // Clearing artifacts
ERROR                 // System error state
```

**Slot Model (SlotLedger):**
```java
// Three slots: CENTER (ready to fire), FRONT (storage), BACK (storage)
ledger.getCenter()    // Returns ArtifactIdentity or null
ledger.getFront()     // Returns ArtifactIdentity or null
ledger.getBack()      // Returns ArtifactIdentity or null
ledger.getArtifactCount()  // 0-3
ledger.isFull()       // True when 3 artifacts
```

**Basic Usage:**
```java
IndexingSystemV3 indexing = new IndexingSystemV3(hardware, config, shooter, telemetry);
indexing.enable();
indexing.setMotifPattern("PPG");
indexing.setHuntEnabled(true);      // Hunt mode ON (auto-collect)
indexing.setSkipColorDetection(true);  // Fast mode (skip color, ~200ms)

// In loop - CRITICAL: This calls shooter.update() internally!
indexing.update();  // DO NOT call shooter.update() separately!

// Check state
SystemState state = indexing.getCurrentState();
int count = indexing.getArtifactCount();
boolean centerOccupied = indexing.hasArtifactInCenter();
```

**Operation Requests:**
```java
// Manual operations
indexing.requestCollect(SlotLedger.Slot.FRONT);     // Collect from front intake
indexing.requestTransfer(SlotLedger.Slot.FRONT);    // Transfer FRONT → CENTER
indexing.requestSwap(SlotLedger.Slot.BACK);         // Swap BACK ↔ CENTER
indexing.requestFire();                             // Fire single shot
indexing.requestFire(3200.0);                       // Fire with specific RPM
indexing.requestEject(EjectOperation.EjectMode.ALL); // Eject all artifacts

// Check if operation running
boolean busy = indexing.isOperationRunning();  // Checks runner AND hardware
String opName = indexing.getCurrentOperationName();
```

**Burst Firing (Keep-Alive Mode):**
```java
// Start burst firing - shooter stays spun between shots
indexing.requestFire(ShooterConfig.RPM_HIGH_BASKET);
indexing.setFiringButtonHeld(gamepad1.right_trigger > 0.5);  // Track trigger

// In loop
if (indexing.isReadyForNextShot() && !indexing.isOperationRunning()) {
    indexing.fireNextShot();  // Fire subsequent shots without spinup delay
}

// Cancel burst
indexing.cancelBurstFiring();  // Manual override or trigger release
```

**Hunt Mode (Auto-Collection):**
```java
// Hunt mode enables automatic artifact collection
indexing.setHuntEnabled(true);   // Hunt ON - auto-collect when detected
indexing.setHuntEnabled(false);  // Hunt OFF - sleep mode (manual only)

// Hunt eligibility rules (per intake):
// ✅ Hunt mode ON
// ✅ Slot empty (not storing artifact)
// ✅ System not full (< 3 artifacts)
// ✅ No operation running

// When hunt-eligible:
// - Rollers run at collect power
// - Transfer servos jiggle (prevent blind spots)
// - Sensors actively poll
// - Auto-collect on HIGH confidence detection
```

**Skip Color Detection (Fast Mode):**
```java
// Skip mode trades accuracy for speed
indexing.setSkipColorDetection(true);   // Fast mode: ~200ms collection
indexing.setSkipColorDetection(false);  // Full mode: ~1400ms with color

// Skip mode ON:
// - Artifacts collected as UNKNOWN immediately
// - No color sampling at checkpoints
// - Faster collection for time-critical situations
// - Shot planner treats UNKNOWN as neutral

// Skip mode OFF:
// - Full color detection with checkpoint sampling
// - Artifacts classified as PURPLE, GREEN, or UNKNOWN
// - Jiggling if needed to improve sensor visibility
// - More reliable shot planning
```

**Artifact Collection Flow:**
```java
// 1ST ARTIFACT: Auto-transfer to CENTER
// - Collected into intake (FRONT or BACK)
// - Immediately transferred to CENTER
// - Ready to fire

// 2ND ARTIFACT: Storage with optional swap
// - Collected into available intake
// - Shot planner checks if swap needed for optimal order
// - If swap beneficial: automatically swaps with CENTER
// - Otherwise: stays in intake (storage mode)

// 3RD ARTIFACT: Storage only
// - System full (max capacity reached)
// - Stays in intake (storage mode)
// - Rollers run at hold power to retain artifact
// - No more auto-collection until space available
```

**Manual Mode & Watchdog:**
```java
// Manual mode detection (OpMode responsibility)
indexing.setManualModeActive(true);   // Disables automation
indexing.setManualModeActive(false);  // Enables automation

// Watchdog safety (automatic burst cancellation)
indexing.setWatchdogTriggerState(gamepad1.right_trigger > 0.5);
// Watchdog monitors:
// - Trigger release → auto-cancel burst
// - Manual override → auto-cancel burst
// - Operation timeout → safety stop
```

**Telemetry:**
```java
// Add telemetry display (paged output)
indexing.addTelemetry();

// Page 1: Overview (state, slots, operations, shot plan)
// Page 2: Sensors (perception, confidence, raw sensor data)
// Page 3: Statistics (counters, watchdog status)

indexing.nextTelemetryPage();  // Cycle between pages
```

**Perception System (Sensor Fusion):**
```java
// IntakePerception provides confidence levels per intake
// - NONE: No sensors detect artifact
// - LOW: One sensor detects
// - MEDIUM: Two sensors detect
// - HIGH: Three or more sensors detect

// Sensors as hints (not absolute truth):
// - Laser distance (outward-facing)
// - REV 2m ToF (mouth-mounted, with hysteresis)
// - Color sensors x2 (outward + mouth)

// Color checkpoint policy:
// - Checkpoint 1: Collection confirmation window (after 150ms settle)
// - Checkpoint 2: Transfer completion (after 200ms settle)
// - Checkpoint 3: Manual operator override

// Confidence thresholds:
// - Skip mode ON: MEDIUM confidence required (2+ sensors)
// - Skip mode OFF: HIGH confidence required (3+ sensors)
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

// Auto-runs in indexing.update() via ShotPlanningCoordinator

// Check rearrangement status
// V3 coordinator automatically executes swaps when beneficial
```

**Rearrangement Rules (V3):**
- ✅ 2 artifacts total (one in CENTER, one in intake)
- ✅ Swap improves shot plan score
- ✅ Not in manual mode
- ✅ No operation running
- ❌ 1 artifact (nothing to swap)
- ❌ 3 artifacts (no benefit to swap when system full)
- ❌ Manual mode active
- ❌ Operation busy

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
    private IndexingSystemV3 indexing;
    private Shooter shooter;
    
    @Override
    public void runOpMode() {
        // 2. Initialize
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        indexing = new IndexingSystemV3(hardware, config, shooter, telemetry);
        
        indexing.enable();
        shooter.enable();
        
        waitForStart();
        
        // 3. Main loop
        while (opModeIsActive()) {
            // CRITICAL: indexing.update() calls shooter.update() internally
            // DO NOT call shooter.update() separately!
            indexing.update();  // This updates shooter internally
            
            // Handle gamepad inputs
            if (gamepad1.a) indexing.requestCollect(SlotLedger.Slot.FRONT);
            if (gamepad1.x) indexing.requestFire();
            
            // Update telemetry
            indexing.addTelemetry();
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
import org.firstinspires.ftc.teamcode.util.aurora.v3.*;

@TeleOp(name="My OpMode", group="Testing")
public class MyOpMode extends LinearOpMode {
    private AuroraHardwareConfig hardware;
    private IndexingSystemV3 indexing;
    private Shooter shooter;
    
    @Override
    public void runOpMode() {
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        shooter = new Shooter(hardware, new ShooterConfig(), telemetry);
        indexing = new IndexingSystemV3(hardware, new IndexingConfig(), shooter, telemetry);
        
        indexing.enable();
        shooter.enable();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // CRITICAL: Don't call shooter.update() - indexing does it internally
            indexing.update();
            
            // Your code here
            
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

**1. IndexingSystemV3 Update Loop**
- **Cause:** Not calling `indexing.update()` every loop
- **Solution:** ALWAYS call in loop, no exceptions
- **CRITICAL:** `indexing.update()` internally calls `shooter.update()` via `BasicFiringHelper`
- **DO NOT** call `shooter.update()` separately - this causes duplicate updates and shooter pulsing!

**2. Servo Glitches on Stop**
- **Cause:** Servos not stopped explicitly
- **Solution:**
```java
@Override
public void stop() {
    if (indexing != null) indexing.disable();  // Stops all operations
    // ... other cleanup
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

**6. State Corruption from Manual Override**
- **Cause:** Calling `setManualModeActive()` inconsistently
- **Solution:** Always call every loop to track manual input state
- **Note:** V3 uses OpMode-controlled manual mode detection

### Key Assumptions

1. **Device names match Driver Station EXACTLY** (case-sensitive)
2. **Update order:** `indexing.update()` (which internally calls shooter.update())
3. **Max 3 artifacts** (enforced by SlotLedger)
4. **Timing is critical** (don't modify without testing)
5. **Manual mode overrides automation**
6. **Operations are transactional** (atomic commits on success)
7. **Sensors provide hints, not truth** (SlotLedger is source of truth)
8. **Color sampling only at checkpoints** (collection, transfer, manual override)

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

Shooter shooter = new Shooter(hardware, new ShooterConfig(), telemetry);
IndexingSystemV3 indexing = new IndexingSystemV3(hardware, new IndexingConfig(), shooter, telemetry);

indexing.enable();
shooter.enable();

// Optional: Configure indexing
indexing.setMotifPattern("PPG");
indexing.setHuntEnabled(true);      // Auto-collect
indexing.setSkipColorDetection(true);  // Fast mode
```

### Essential Loop
```java
while (opModeIsActive()) {
    // CRITICAL: indexing.update() calls shooter.update() internally
    // DO NOT call shooter.update() separately!
    indexing.update();
    
    // Handle gamepad inputs
    if (gamepad1.a) indexing.requestCollect(SlotLedger.Slot.FRONT);
    if (gamepad1.x) indexing.requestFire();
    
    // Manual mode detection
    boolean manualActive = gamepad2.dpad_up || gamepad2.dpad_down;
    indexing.setManualModeActive(manualActive);
    
    // Watchdog trigger tracking
    indexing.setWatchdogTriggerState(gamepad1.right_trigger > 0.5);
    
    // Add telemetry
    indexing.addTelemetry();
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

1. ✅ ALWAYS call `indexing.update()` every loop
2. ✅ NEVER call `shooter.update()` separately (indexing does it internally)
3. ✅ Check initialization before use
4. ✅ Test with hardware
5. ✅ Document changes
6. ❌ Don't modify core V3 files (IndexingSystemV3, operations, SlotLedger)
7. ❌ Don't skip timeout protection
8. ❌ Don't bypass safety checks
9. ❌ Don't modify timing without testing
10. ❌ Don't access SlotLedger directly (use IndexingSystemV3 API)
11. ❌ Don't sample color outside checkpoints
12. ❌ Don't modify sensor thresholds without calibration

---

## IndexingSystemV3 Deep Dive

### Architecture Principles

**1. Slot-Based State Management**
- **SlotLedger** is the single source of truth
- Three slots: CENTER (ready to fire), FRONT (storage), BACK (storage)
- Each slot contains `ArtifactIdentity` or null
- State changes ONLY via operation commits (never sensor-driven)

**2. Transactional Operations**
- Every physical action is an operation with lifecycle: `start() → update() → commit()`
- Operations start hardware, track completion, commit atomically
- No partial state changes - either full success or rollback
- Single operation at a time (OperationRunner enforces)

**3. Sensors as Hints**
- Sensors provide confidence levels, not absolute truth
- IntakePerception fuses multiple sensors per intake
- Presence confidence: NONE, LOW (1 sensor), MEDIUM (2 sensors), HIGH (3+ sensors)
- SlotLedger state takes precedence over sensor readings

**4. Color Checkpoint Policy**
- Color sampling ONLY at stable checkpoints:
  - Checkpoint 1: Collection (after 150ms settle delay)
  - Checkpoint 2: Transfer (after 200ms settle delay)
  - Checkpoint 3: Manual operator override
- No color sampling during motion or between checkpoints
- Prevents noise from moving artifacts

### Operation Types

**CollectOperation**
- Preconditions: Target slot empty, system not full
- Process: Run rollers → detect presence → wait for settle → sample color (if enabled) → commit
- Commit: Place artifact in target slot with identity
- Duration: ~200ms (skip mode) or ~1400ms (full color detection)

**TransferOperation**
- Preconditions: Source slot occupied, center empty (or post-fire)
- Process: Run transfer servos/injectors → wait for completion → settle
- Commit: Move artifact from source → center, clear source slot
- Post-transfer: Perception reset to prevent false re-detection

**SwapOperation**
- Preconditions: Both slots occupied, 2 artifacts total
- Process: Simultaneous transfer and accept (push-style mechanics)
- Commit: Atomically swap artifact identities between slots
- Use case: Shot planning optimization

**FireOperation**
- Preconditions: Center occupied, prepositioned, shooter ready
- Process: Spin up shooter → preposition artifact → feed → fire
- Commit: Clear center slot, consume artifact from plan
- Keep-alive mode: Shooter stays spun for subsequent shots

**PrepositionOperation**
- Preconditions: Center occupied, not already prepositioned
- Process: Move artifact to firing position via uptake servos
- Commit: Set preposition flag
- Required before firing

**EjectOperation**
- Modes: ALL (all slots), CENTER (center only), SOFTWARE_CLEAR (ledger only)
- Process: Reverse intakes, run shooter at low RPM (physical eject)
- Commit: Clear specified slots
- Emergency operation (no preconditions)

### Hunt Mode Behavior

**When Hunt Mode ON:**
1. Empty intakes become "hunt-eligible" if system not full
2. Hunt-eligible intakes:
   - Run rollers at collect power
   - Run transfer servos in reverse (jiggling to prevent blind spots)
   - Poll sensors continuously
   - Auto-collect when HIGH confidence (or MEDIUM if skip mode)
3. Auto-collection has 1-second cooldown to prevent repeated triggers

**When Hunt Mode OFF (Sleep):**
1. Empty intakes stop rollers and servos
2. Sensors still update but don't trigger collection
3. Storage intakes (with artifacts) maintain hold power
4. Manual operations still work (collect, transfer, fire, eject)

**Hunt Eligibility Rules:**
```java
boolean isHuntEligible = 
    huntEnabled &&           // Hunt mode ON
    slot != CENTER &&        // Only FRONT/BACK
    !ledger.isOccupied(slot) &&  // Slot empty
    !ledger.isFull() &&      // System has capacity
    !runner.isBusy();        // No operation running
```

### Burst Firing Flow

**Phase 1: Initiation**
1. Call `requestFire(rpm)` with desired RPM
2. System enters burst firing mode
3. Watchdog activated for safety monitoring
4. FiringHelper spins up shooter (keep-alive enabled)

**Phase 2: First Shot**
1. Wait for shooter to reach target RPM
2. Preposition artifact (if needed)
3. Feed artifact through uptake (300ms)
4. Shot count increments
5. Ledger clears CENTER immediately

**Phase 3: Subsequent Shots**
1. Check `isReadyForNextShot()` - shooter spun, waiting
2. Check `!isOperationRunning()` - no transfer in progress
3. Call `fireNextShot()` - immediate fire (no spinup delay)
4. Transfer next artifact from intake → CENTER (automatic)
5. Repeat until no more artifacts or trigger released

**Phase 4: Termination**
1. Watchdog detects trigger release → auto-cancel
2. Manual override detected → auto-cancel
3. No more artifacts → end burst
4. OpMode calls `cancelBurstFiring()` → manual stop

**Safety Features:**
- Watchdog monitors trigger state
- Auto-cancel on trigger release
- Auto-cancel on manual override
- Max 5 shots per burst (configurable)
- Deferred transfers prevent conflicts

### Perception System Details

**Sensor Types per Intake:**
1. **Laser Distance** (Analog): 0-3.3V = 0-1000mm, threshold 10cm
2. **REV 2m ToF** (Digital): Distance with hysteresis (enter 7cm, exit 3cm below baseline)
3. **Color Sensor (Outward)**: Next to laser, for settled artifacts
4. **Color Sensor (Mouth)**: Opposite REV sensor, for entering artifacts

**Signal Fusion:**
```java
// Raw signals
frontBlocked = laserDistance < 10cm
mouthOccupied = revDistance < (baseline - hysteresis)
colorSeesArtifact_outward = colorConfidence > threshold
colorSeesArtifact_mouth = colorConfidence > threshold

// Combined confidence
sensorCount = frontBlocked + mouthOccupied + colorOutward + colorMouth
if (sensorCount >= 3) confidence = HIGH
else if (sensorCount == 2) confidence = MEDIUM
else if (sensorCount == 1) confidence = LOW
else confidence = NONE

// Debouncing
fastPresence = anySignal stable for 30ms    // Edge detection
stablePresence = anySignal stable for 100ms // Confirmation
```

**Baseline Calibration:**
- REV sensor calibrates baseline when intake known empty
- Updates every 500ms if 5 consecutive empty readings
- Accounts for environmental changes

### Manual Override System

**OpMode Responsibilities:**
1. Detect manual input (e.g., gamepad2 controls)
2. Call `setManualModeActive(true/false)` every loop
3. Manual mode disables automation but allows operations

**Effects of Manual Mode:**
- Disables automatic transfers
- Disables automatic rearrangement
- Disables shot plan consumption
- Cancels burst firing if active
- Manual operations (collect, transfer, fire) still work

**Example:**
```java
// In OpMode loop
boolean manualActive = 
    gamepad2.dpad_up ||      // Manual uptake
    gamepad2.dpad_down ||    // Manual outtake
    gamepad2.left_bumper ||  // Manual transfer
    gamepad2.right_bumper;   // Manual inject

indexing.setManualModeActive(manualActive);
```

### Common Usage Patterns

**Basic Collection:**
```java
if (gamepad1.a) {
    indexing.requestCollect(SlotLedger.Slot.FRONT);
}
// Auto-transfers to CENTER if 1st artifact
// Auto-checks swap if 2nd artifact
// Stays in intake if 3rd artifact
```

**Manual Transfer:**
```java
if (gamepad1.left_bumper && ledger.isFrontOccupied()) {
    indexing.requestTransfer(SlotLedger.Slot.FRONT);
}
```

**Single Shot:**
```java
if (gamepad1.x) {
    indexing.requestFire();  // Fire once, shooter stops after
}
```

**Burst Fire:**
```java
// Start burst
if (gamepad1.right_trigger > 0.5 && !burstStarted) {
    indexing.requestFire(ShooterConfig.RPM_HIGH_BASKET);
    burstStarted = true;
}

// Track trigger for watchdog
indexing.setWatchdogTriggerState(gamepad1.right_trigger > 0.5);

// Fire subsequent shots
if (burstStarted && indexing.isReadyForNextShot() && !indexing.isOperationRunning()) {
    indexing.fireNextShot();
}

// Detect end
if (gamepad1.right_trigger <= 0.5) {
    burstStarted = false;  // Watchdog auto-cancels
}
```

**Emergency Eject:**
```java
if (gamepad1.back) {
    indexing.requestEject(EjectOperation.EjectMode.ALL);
}
```

**Manual Artifact Injection (Testing):**
```java
// Add phantom artifact for testing/debugging
if (gamepad2.dpad_left) {
    indexing.addManualArtifact(SlotLedger.Slot.FRONT, ColorClass.PURPLE);
}
if (gamepad2.dpad_right) {
    indexing.addManualArtifact(SlotLedger.Slot.BACK, ColorClass.GREEN);
}
```

### Migration from V2 to V3

**Key API Changes:**
```java
// OLD (v2)
IndexingSystem indexing = new IndexingSystem(...);
indexing.setManualInputDetector(() -> gamepad2.dpad_up);
FiringSequenceCoordinator firing = new FiringSequenceCoordinator(...);

// In loop
indexing.update();
shooter.update();
firing.update();

// NEW (v3)
IndexingSystemV3 indexing = new IndexingSystemV3(...);
// No separate firing coordinator

// In loop
indexing.update();  // Handles shooter internally
indexing.setManualModeActive(gamepad2.dpad_up);
indexing.setWatchdogTriggerState(gamepad1.right_trigger > 0.5);
```

**Conceptual Differences:**
| Aspect | V2 (Old) | V3 (New) |
|--------|----------|----------|
| State model | List + refs | SlotLedger (3 slots) |
| Sensor role | Primary tracking | Hints only |
| Operations | State transitions | Explicit operations |
| Artifacts | Pushed between slots | Transferred atomically |
| Color | Continuous sampling | Checkpoint-based |
| Firing | FiringSequenceCoordinator | Integrated into system |
| Manual mode | Detector lambda | OpMode-controlled flag |

---

## Documentation References

### In-Repository
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/README.md` - V3 architecture overview
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/COLOR_CHECKPOINT_POLICY.md` - Color sampling policy
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/MIGRATION_COMPATIBILITY.md` - V2 → V3 migration guide
- `FIRING_SYSTEM_USAGE.md` - Complete firing guide (may reference old system)
- `TeamCode/AURORA_SYSTEM_README.md` - System overview
- `TeamCode/AURORA_HARDWARE_CONFIG_GUIDE.md` - Hardware setup

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
- ❌ Missing `indexing.update()` calls
- ❌ Calling `shooter.update()` separately from indexing
- ❌ Infinite loop without `opModeIsActive()` 
- ❌ Hardcoded timing values
- ❌ Direct servo/motor control bypassing helpers
- ❌ Modified device names in AuroraHardwareConfig
- ❌ Removed safety checks
- ❌ Changed scoring algorithms
- ❌ Direct SlotLedger modification (bypass operations)
- ❌ Color sampling outside checkpoints
- ❌ Manual ledger.set() calls (use operations instead)

### Success Indicators
- ✅ Clear initialization
- ✅ Proper update loop (indexing.update() only, no separate shooter.update())
- ✅ Comprehensive telemetry
- ✅ Error handling with logging
- ✅ Consistent naming
- ✅ Comments for non-obvious logic
- ✅ Uses config objects
- ✅ Respects subsystem interfaces
- ✅ Uses operation requests (not direct hardware control)
- ✅ Manual mode tracking
- ✅ Watchdog trigger updates
- ✅ Tested with hardware
- ✅ Documentation updated

---

**Remember:** This robot represents months of refinement and testing. The V3 indexing system is a complete architectural rewrite with slot-based state management, transactional operations, and sensor fusion. Respect the architecture, test thoroughly, and document all changes. When uncertain, ask before modifying core systems.

**Competition:** DECODE 2025-2026  
**System Version:** Aurora V3 (IndexingSystemV3)  
**Last Updated:** January 2025
