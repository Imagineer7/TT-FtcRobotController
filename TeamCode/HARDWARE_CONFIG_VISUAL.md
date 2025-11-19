# AURORA Hardware Configuration - Visual Guide

## Before vs After: The Problem We Solved

### ❌ BEFORE: Scattered Hardware Mapping

```
┌─────────────────────────────────────────────────────────────────┐
│ AuroraManager.java                                              │
│                                                                 │
│  DcMotor leftFront = hardwareMap.get(DcMotor.class,           │
│                                      "frontLeft");  ◄── 1      │
│  DcMotor rightFront = hardwareMap.get(DcMotor.class,          │
│                                       "frontRight"); ◄── 2     │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ EnhancedDecodeHelper.java                                       │
│                                                                 │
│  shooter = hardwareMap.get(DcMotor.class, "shooter"); ◄── 3   │
│  feedServo1 = hardwareMap.get(CRServo.class,                   │
│                               "servo1");             ◄── 4     │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ BLUEAutoAurora.java                                             │
│                                                                 │
│  odometry = hardwareMap.get(GoBildaPinpointDriver.class,       │
│                             "odo");                 ◄── 5      │
└─────────────────────────────────────────────────────────────────┘

PROBLEM: 
• Device names scattered across 5+ files
• Hard to maintain when names change
• Inconsistent error handling
• Duplicate initialization code
```

### ✅ AFTER: Unified Hardware Configuration

```
┌─────────────────────────────────────────────────────────────────┐
│ AuroraHardwareConfig.java   ◄─── SINGLE SOURCE OF TRUTH        │
│━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━│
│                                                                 │
│  // All device names in ONE place                              │
│  public static final String FRONT_LEFT_MOTOR = "frontLeft";    │
│  public static final String SHOOTER_MOTOR = "shooter";         │
│  public static final String FEED_SERVO_1 = "servo1";           │
│  public static final String ODOMETRY_COMPUTER = "odo";         │
│                                                                 │
│  // Centralized initialization                                 │
│  private void initializeDriveSystem() { ... }                  │
│  private void initializeShooterSystem() { ... }                │
│  private void initializeOdometry() { ... }                     │
│                                                                 │
│  // Public accessors                                           │
│  public DcMotor getFrontLeftMotor() { ... }                    │
│  public DcMotor getShooterMotor() { ... }                      │
│  public CRServo getFeedServo1() { ... }                        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
          ┌───────────────────┴───────────────────┐
          │                                       │
┌─────────▼─────────────┐              ┌─────────▼─────────────┐
│ AuroraManager         │              │ EnhancedDecodeHelper  │
│                       │              │                       │
│ Uses hardware config  │              │ Uses hardware config  │
│ ✓ No hardwareMap.get()│              │ ✓ No hardwareMap.get()│
└───────────────────────┘              └───────────────────────┘

BENEFITS:
• ✓ Change device names in ONE place
• ✓ Consistent error handling
• ✓ Detailed initialization logging
• ✓ Easy to maintain
```

## Component Relationships

```
                    ┌─────────────────────┐
                    │   Your OpMode       │
                    │ (AURORATeleOp, etc) │
                    └──────────┬──────────┘
                               │
                               │ creates
                               │
                    ┌──────────▼──────────┐
                    │   AuroraManager     │
                    │  (System Coordinator)│
                    └──────────┬──────────┘
                               │
                               │ creates internally
                               │
              ┌────────────────▼────────────────┐
              │  AuroraHardwareConfig           │
              │  • Initializes all hardware     │
              │  • Handles errors gracefully    │
              │  • Provides status checking     │
              └────────────────┬────────────────┘
                               │
                               │ initializes
                               │
        ┌──────────────────────┼──────────────────────┐
        │                      │                      │
        ▼                      ▼                      ▼
┌───────────────┐    ┌─────────────────┐    ┌────────────────┐
│ Drive Motors  │    │ Shooter System  │    │ Sensors        │
│ • frontLeft   │    │ • shooter motor │    │ • voltage      │
│ • frontRight  │    │ • servo1        │    │ • odometry     │
│ • backLeft    │    │ • servo2        │    │                │
│ • backRight   │    │ • light         │    │                │
└───────────────┘    └─────────────────┘    └────────────────┘
```

## Hardware Initialization Flow

```
START
  │
  ├─► Create AuroraHardwareConfig
  │     │
  │     └─► Choose initialization mode
  │           │
  │           ├─► initialize()           (for Autonomous)
  │           │     │
  │           │     ├─► initializeDriveSystem()
  │           │     │     ├─► Get frontLeft motor
  │           │     │     ├─► Get frontRight motor
  │           │     │     ├─► Get backLeft motor
  │           │     │     ├─► Get backRight motor
  │           │     │     └─► Configure motors
  │           │     │
  │           │     ├─► initializeShooterSystem()
  │           │     │     ├─► Get shooter motor
  │           │     │     ├─► Get feed servos
  │           │     │     ├─► Try get light (optional)
  │           │     │     └─► Configure shooter
  │           │     │
  │           │     └─► initializeVoltageSensor()
  │           │           └─► Try get sensor (optional)
  │           │
  │           └─► initializeWithOdometry() (for TeleOp)
  │                 │
  │                 ├─► Same as initialize() PLUS:
  │                 │
  │                 └─► initializeOdometry()
  │                       ├─► Get odometry device
  │                       ├─► Set offsets
  │                       ├─► Set encoder directions
  │                       └─► Reset position and IMU
  │
  └─► Check initialization status
        │
        ├─► isDriveSystemInitialized()?
        ├─► isShooterSystemInitialized()?
        └─► isOdometryInitialized()?
              │
              └─► Ready for use!
```

## Error Handling Flow

```
┌─────────────────────────────────────────┐
│ Try to initialize hardware component     │
└──────────────┬──────────────────────────┘
               │
        ┌──────▼──────┐
        │  Success?   │
        └──────┬──────┘
               │
       ┌───────┴───────┐
       │               │
      YES             NO
       │               │
       ▼               ▼
┌────────────┐   ┌──────────────────┐
│ Set flag   │   │ Catch exception  │
│ = true     │   │                  │
│            │   │ Set flag = false │
│ Clear error│   │                  │
│            │   │ Store error msg  │
│ Log success│   │                  │
└────────────┘   │ Log detailed info│
                 │                  │
                 │ Continue init... │
                 └──────────────────┘
                          │
                          ▼
                 ┌────────────────┐
                 │ OpMode can     │
                 │ check status   │
                 │ and handle     │
                 │ gracefully     │
                 └────────────────┘
```

## How to Change Hardware Device Names

```
STEP 1: Open AuroraHardwareConfig.java
┌─────────────────────────────────────────────┐
│ public static final String                  │
│   FRONT_LEFT_MOTOR = "frontLeft";  ◄─ EDIT │
│                                             │
│ Change to match your Driver Station config  │
│                                             │
│ Example: FRONT_LEFT_MOTOR = "motor_fl";    │
└─────────────────────────────────────────────┘

STEP 2: That's it! All OpModes automatically use the new name
┌─────────────────────────────────────────────┐
│ AuroraManager.java        ✓ Updates         │
│ EnhancedDecodeHelper.java ✓ Updates         │
│ All Auto OpModes          ✓ Updates         │
│ All TeleOp OpModes        ✓ Updates         │
└─────────────────────────────────────────────┘
```

## Usage Patterns

### Pattern 1: Using AuroraManager (Recommended)

```
┌────────────────────────────────────────────┐
│ Your OpMode                                │
│                                            │
│ public void runOpMode() {                  │
│                                            │
│   // AuroraManager handles everything     │
│   AuroraManager robotManager =            │
│     new AuroraManager(hardwareMap,         │
│                       telemetry);          │
│                                            │
│   // Hardware config is used internally   │
│   // You don't need to create it!         │
│                                            │
│   while (opModeIsActive()) {              │
│     robotManager.update(gamepad1,          │
│                        gamepad2);          │
│   }                                        │
│ }                                          │
└────────────────────────────────────────────┘
```

### Pattern 2: Direct Hardware Access

```
┌────────────────────────────────────────────┐
│ Your OpMode                                │
│                                            │
│ public void runOpMode() {                  │
│                                            │
│   // Create hardware config               │
│   AuroraHardwareConfig hardware =         │
│     new AuroraHardwareConfig(             │
│       hardwareMap, telemetry);            │
│                                            │
│   // Initialize (with or without odo)     │
│   hardware.initializeWithOdometry();      │
│                                            │
│   // Get hardware components              │
│   DcMotor shooter =                        │
│     hardware.getShooterMotor();           │
│   DcMotor frontLeft =                      │
│     hardware.getFrontLeftMotor();         │
│                                            │
│   // Use hardware...                      │
│   while (opModeIsActive()) {              │
│     shooter.setPower(0.8);                │
│     frontLeft.setPower(gamepad1.left_y);  │
│   }                                        │
│ }                                          │
└────────────────────────────────────────────┘
```

## Status Checking

```
┌────────────────────────────────────────────┐
│ Check initialization status                │
│                                            │
│ if (hardware.isDriveSystemInitialized()) { │
│   // Drive system is ready to use         │
│   DcMotor motor =                          │
│     hardware.getFrontLeftMotor();         │
│ } else {                                   │
│   // Handle error                          │
│   String error =                           │
│     hardware.getDriveInitError();         │
│   telemetry.addLine("Error: " + error);   │
│ }                                          │
│                                            │
│ // Or get complete summary                │
│ String summary =                           │
│   hardware.getInitializationSummary();    │
│ telemetry.addLine(summary);               │
└────────────────────────────────────────────┘
```

## File Organization

```
TeamCode/
│
├── AURORA_HARDWARE_CONFIG_GUIDE.md    ◄── Detailed guide
├── AURORA_SYSTEM_README.md             ◄── System overview
├── HARDWARE_CONFIG_VISUAL.md           ◄── This visual guide
│
└── src/main/java/.../
    │
    ├── AURORATeleOp.java               Uses AuroraManager
    ├── BLUEAutoAurora.java             Uses AuroraManager
    ├── REDAutoAurora.java              Uses AuroraManager
    │
    ├── util/aurora/
    │   ├── AuroraHardwareConfig.java   ◄── ★ NEW ★
    │   │                               Single source of truth
    │   │
    │   ├── AuroraManager.java          ◄── Updated
    │   │                               Uses hardware config
    │   │
    │   ├── EnhancedDecodeHelper.java   ◄── Updated
    │   │                               Uses hardware config
    │   │
    │   └── SmartMechanumDrive.java     Uses motors from config
    │
    └── examples/
        └── AuroraHardwareConfigExample.java  ◄── Tutorial
```

## Quick Reference Card

```
╔═══════════════════════════════════════════════════════════╗
║        AURORA Hardware Config Quick Reference             ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  CREATE CONFIG:                                           ║
║    AuroraHardwareConfig hardware =                        ║
║      new AuroraHardwareConfig(hardwareMap, telemetry);    ║
║                                                           ║
║  INITIALIZE:                                              ║
║    hardware.initialize();              // No odometry     ║
║    hardware.initializeWithOdometry();  // With odometry   ║
║                                                           ║
║  ACCESS HARDWARE:                                         ║
║    hardware.getFrontLeftMotor()                           ║
║    hardware.getShooterMotor()                             ║
║    hardware.getFeedServo1()                               ║
║    hardware.getOdometry()                                 ║
║                                                           ║
║  CHECK STATUS:                                            ║
║    hardware.isDriveSystemInitialized()                    ║
║    hardware.isShooterSystemInitialized()                  ║
║    hardware.getDriveInitError()                           ║
║                                                           ║
║  CHANGE DEVICE NAMES:                                     ║
║    Edit constants in AuroraHardwareConfig.java            ║
║    (Changes apply to all OpModes automatically!)          ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝
```

## Benefits Summary

```
┌──────────────────────────────────────────────────────────┐
│ ✓ SINGLE SOURCE OF TRUTH                                 │
│   All device names in one place                          │
│                                                          │
│ ✓ EASY MAINTENANCE                                       │
│   Change once, affects all OpModes                       │
│                                                          │
│ ✓ BETTER ERROR HANDLING                                  │
│   Detailed logging and status checking                   │
│                                                          │
│ ✓ CONSISTENT INITIALIZATION                              │
│   Same setup across all OpModes                          │
│                                                          │
│ ✓ CLEAR DOCUMENTATION                                    │
│   All hardware documented in one class                   │
│                                                          │
│ ✓ BACKWARD COMPATIBLE                                    │
│   Old code still works, migrate gradually                │
│                                                          │
│ ✓ TYPE SAFE                                              │
│   Compile-time checking of hardware access               │
└──────────────────────────────────────────────────────────┘
```
