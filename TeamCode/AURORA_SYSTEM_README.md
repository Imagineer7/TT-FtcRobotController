# AURORA System Overview

**AURORA** = **A**dvanced **U**nified **R**obot **O**perating & **R**esponse **A**rchitecture

## What is AURORA?

AURORA is a comprehensive robot control system for FTC (FIRST Tech Challenge) robots that provides:
- Unified hardware configuration management
- Advanced drive system with multiple modes
- Intelligent shooter control with ML-based optimization
- Performance monitoring and analytics
- Dual-driver support for competition
- Smart telemetry system
- Semi-autonomous features

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                      OpMode Layer                        │
│  (AURORATeleOp, BLUEAutoAurora, REDAutoAurora)          │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│                  AuroraManager                           │
│        (Central coordination and management)             │
└──────┬──────────────┬──────────────┬────────────────────┘
       │              │               │
┌──────▼────────┐ ┌──▼─────────────┐ ┌▼──────────────────┐
│ AuroraHardware│ │SmartMecanum    │ │EnhancedDecode     │
│ Config        │ │Drive           │ │Helper (Shooter)   │
│               │ │                │ │                   │
│ - Drive Motors│ │ - SPORT Mode   │ │ - RPM Control     │
│ - Shooter     │ │ - NORMAL Mode  │ │ - ML Optimization │
│ - Servos      │ │ - PRECISION    │ │ - Feed Servos     │
│ - Odometry    │ │ - EFFICIENCY   │ │ - Performance Mon │
│ - Sensors     │ │ - Field Rel.   │ │ - Shot Tracking   │
└───────────────┘ └────────────────┘ └───────────────────┘
```

## Core Components

### 1. AuroraHardwareConfig
**Purpose**: Centralized hardware device mapping

**Key Features**:
- Single source of truth for all hardware device names
- Graceful error handling during initialization
- Separate methods for TeleOp (with odometry) and Autonomous
- Detailed status checking and error reporting

**File**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/AuroraHardwareConfig.java`

**Usage**:
```java
AuroraHardwareConfig hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
hardware.initializeWithOdometry();  // For TeleOp
// OR
hardware.initialize();  // For Autonomous
```

### 2. AuroraManager
**Purpose**: Central coordination hub for all robot subsystems

**Key Features**:
- Manages drive system, shooter system, and all subsystems
- Dual-driver mode support (single driver or driver + operator)
- Semi-autonomous action coordination
- System health monitoring
- Battery-based power management
- Emergency stop protocols

**File**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/AuroraManager.java`

**Usage**:
```java
AuroraManager robotManager = new AuroraManager(hardwareMap, telemetry);
robotManager.update(gamepad1, gamepad2);  // Call in main loop
```

### 3. SmartMechanumDrive
**Purpose**: Advanced mecanum drive control with multiple drive modes

**Drive Modes**:
- **SPORT**: Maximum performance (100% power)
- **NORMAL**: Balanced control (75% power)
- **PRECISION**: Fine control (40% power)
- **EFFICIENCY**: Battery saving (50% power)

**Features**:
- Field-relative driving
- Fine movement controls (D-pad)
- Fine rotation controls (bumpers)
- Voltage-based automatic mode switching
- Movement analytics and performance tracking

**File**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/SmartMechanumDrive.java`

### 4. EnhancedDecodeHelper
**Purpose**: Intelligent shooter control system

**Key Features**:
- Configurable shooting presets (Long Range, Short Range, Rapid Fire, etc.)
- PID-based RPM control with recovery mode
- Machine Learning shot optimization (adaptive learning)
- Shot compensation boost system
- Performance monitoring and analytics
- Odometry integration for position tracking
- RGB light indicator for status

**File**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/EnhancedDecodeHelper.java`

### 5. SmartTelemetryManager
**Purpose**: Optimized telemetry display with multiple pages

**Pages**:
1. **Overview**: System status & critical info
2. **Drive**: Movement & navigation systems
3. **Shooter**: Shooting system & performance
4. **Performance**: System metrics & diagnostics
5. **Controls**: Control mapping & help

**Features**:
- Page cycling to avoid information overload
- Automatic page switching based on context
- Optimized for minimal lag
- Color-coded status indicators

**File**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/SmartTelemetryManager.java`

## Quick Start Guide

### For Competition (Recommended)

Use the pre-built AURORA OpModes:

**TeleOp**: `AURORATeleOp`
- Full-featured TeleOp with dual-driver support
- Smart telemetry system
- ML-based shooter optimization
- Field visualization

**Autonomous**: `BLUEAutoAurora` or `REDAutoAurora`
- Odometry-based navigation
- Intelligent movement functions
- Integrated shooter control
- PID-based positioning

### For Custom OpModes

1. **Initialize AURORA System**:
```java
AuroraManager robotManager = new AuroraManager(hardwareMap, telemetry);
```

2. **Main Loop**:
```java
while (opModeIsActive()) {
    robotManager.update(gamepad1, gamepad2);
}
```

3. **Access Subsystems**:
```java
SmartMechanumDrive drive = robotManager.getDriveSystem();
EnhancedDecodeHelper shooter = robotManager.getShooterSystem();
```

## Hardware Configuration

All hardware device names are defined in `AuroraHardwareConfig.java`:

### Required Hardware
- **Drive Motors**: frontLeft, frontRight, backLeft, backRight
- **Shooter Motor**: shooter
- **Feed Servos**: servo1, servo2

### Optional Hardware
- **Light Servo**: light (RGB indicator)
- **Odometry**: odo (goBILDA Pinpoint)
- **Voltage Sensor**: (automatically detected)

### Changing Device Names

To match your robot's configuration, edit the constants in `AuroraHardwareConfig.java`:

```java
// Example: Change motor names
public static final String FRONT_LEFT_MOTOR = "motor_fl";  // Changed from "frontLeft"
```

See `AURORA_HARDWARE_CONFIG_GUIDE.md` for detailed instructions.

## Driver Controls

### Single Driver Mode (Default in code, can toggle)
**Gamepad 1**: All controls
- Movement: Right stick (forward/strafe), Left stick (rotate)
- Fine Movement: D-pad (XY at 20%), Bumpers (rotation at 20%)
- Drive Modes: Left stick button (precision), X (field relative)
- Shooting: A/Y (shoot), B (feed), X (stop), Triggers (manual control)

### Dual Driver Mode
**Gamepad 1 (Driver)**:
- Movement controls only
- Semi-autonomous actions
- Mode switching

**Gamepad 2 (Operator)**:
- Shooting controls
- ML controls (save/reset)
- System monitoring

**Toggle Driver Mode**: Back + D-pad Left (on gamepad1)

## File Structure

```
TeamCode/
├── AURORA_SYSTEM_README.md (this file)
├── AURORA_HARDWARE_CONFIG_GUIDE.md (detailed hardware guide)
│
├── src/main/java/org/firstinspires/ftc/teamcode/
│   ├── AURORATeleOp.java (Main TeleOp)
│   ├── BLUEAutoAurora.java (Blue autonomous)
│   ├── REDAutoAurora.java (Red autonomous)
│   │
│   ├── util/aurora/
│   │   ├── AuroraHardwareConfig.java ★ (Unified hardware config)
│   │   ├── AuroraManager.java (System coordinator)
│   │   ├── SmartMechanumDrive.java (Drive system)
│   │   ├── EnhancedDecodeHelper.java (Shooter system)
│   │   ├── SmartTelemetryManager.java (Telemetry system)
│   │   ├── ShooterConfig.java (Shooter presets)
│   │   ├── PerformanceMonitor.java (Analytics)
│   │   ├── RpmLearningSystem.java (ML system)
│   │   ├── MovementRecorder.java (Recording)
│   │   └── ShooterBoostConfig.java (Shot boost)
│   │
│   └── examples/
│       └── AuroraHardwareConfigExample.java (Tutorial)
```

★ = Recently added unified hardware configuration system

## Key Features

### 1. Unified Hardware Configuration
- **Single Source of Truth**: All device names in one place
- **Easy Maintenance**: Change once, affects all OpModes
- **Graceful Error Handling**: Detailed logging and status checks
- **Documentation**: Complete guide with examples

### 2. Intelligent Drive System
- **Multiple Drive Modes**: Sport, Normal, Precision, Efficiency
- **Field-Relative Driving**: Robot moves relative to field
- **Fine Control**: Precise positioning with D-pad and bumpers
- **Auto Mode Switching**: Battery-aware performance adjustment

### 3. Advanced Shooter System
- **Shooting Presets**: Pre-configured for different ranges
- **PID RPM Control**: Precise speed control with recovery mode
- **ML Optimization**: Learns optimal PID gains from shots
- **Shot Compensation**: Adaptive boost system
- **Performance Tracking**: Detailed analytics and monitoring

### 4. Smart Telemetry
- **Multiple Pages**: Organized information display
- **Context-Aware**: Shows relevant info automatically
- **Optimized**: Minimal lag even with complex systems
- **Color-Coded**: Easy-to-read status indicators

### 5. Dual Driver Support
- **Flexible Control**: Single or dual driver modes
- **Easy Toggle**: Switch modes during match
- **Optimal Division**: Driver focuses on movement, Operator on utilities

## Documentation

- **Hardware Configuration Guide**: `AURORA_HARDWARE_CONFIG_GUIDE.md`
- **This Overview**: `AURORA_SYSTEM_README.md`
- **Example OpMode**: `examples/AuroraHardwareConfigExample.java`
- **Code Comments**: Extensive JavaDoc in all classes

## Troubleshooting

### Common Issues

1. **"Motor not found" errors**
   - Check Driver Station hardware configuration
   - Verify device names match constants in `AuroraHardwareConfig.java`

2. **Robot not responding to gamepad**
   - Check that AuroraManager.update() is called in main loop
   - Verify gamepads are connected

3. **Shooter not reaching target RPM**
   - Check battery voltage (low battery affects performance)
   - Verify shooter motor is properly connected
   - Check ML learning data (may need reset)

4. **Odometry not working**
   - Ensure you called `initializeWithOdometry()` not `initialize()`
   - Check device is named "odo" in Driver Station
   - Verify I2C connection

See `AURORA_HARDWARE_CONFIG_GUIDE.md` for more troubleshooting.

## Best Practices

1. **Always use AuroraHardwareConfig** for hardware mapping
2. **Use AuroraManager** for integrated system control
3. **Test hardware configuration** at start of each practice
4. **Monitor telemetry pages** during operation
5. **Save ML data** after successful shooting sessions
6. **Check battery voltage** regularly during matches

## Version History

### v2.0 (Current) - Unified Hardware Configuration
- Added `AuroraHardwareConfig` for centralized hardware mapping
- Updated all AURORA classes to use unified config
- Added comprehensive documentation
- Created example OpMode
- Improved error handling and logging

### v1.0 - Initial AURORA System
- Core system architecture
- Smart drive system with multiple modes
- Advanced shooter with ML optimization
- Smart telemetry system
- Dual driver support

## Contributing

When adding new hardware components:

1. Add device name constant to `AuroraHardwareConfig.java`
2. Add initialization code in appropriate method
3. Add public accessor method
4. Update documentation
5. Update example OpMode if relevant

## Support

For questions or issues:
1. Check documentation files
2. Review example OpModes
3. Examine code comments
4. Test on robot hardware

## Credits

Developed for FTC Team by the robotics programming team.
AURORA System © 2025
