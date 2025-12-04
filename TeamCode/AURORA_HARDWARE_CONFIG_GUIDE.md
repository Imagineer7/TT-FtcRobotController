# AURORA Unified Hardware Configuration Guide

## Overview

The AURORA system now uses a **unified hardware configuration class** (`AuroraHardwareConfig.java`) that centralizes all hardware device mapping in one place. This makes it much easier to maintain and update robot hardware configurations.

## Benefits

1. **Single Source of Truth**: All hardware device names are defined in one place
2. **Easy Maintenance**: Change a device name once, affects all OpModes
3. **Better Error Handling**: Detailed initialization logging and error messages
4. **Consistent Initialization**: Same hardware setup across all OpModes
5. **Clear Documentation**: All robot hardware is documented in one class

## Quick Start

### For New OpModes

```java
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;

public class MyOpMode extends LinearOpMode {
    private AuroraHardwareConfig hardware;
    
    @Override
    public void runOpMode() {
        // Initialize hardware configuration
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();  // For TeleOp with odometry
        // OR
        hardware.initialize();  // For Autonomous without odometry
        
        // Access hardware components
        DcMotor shooter = hardware.getShooterMotor();
        DcMotor frontLeft = hardware.getFrontLeftMotor();
        GoBildaPinpointDriver odo = hardware.getOdometry();
        
        // ... rest of your OpMode
    }
}
```

### Using with AURORA Subsystems

```java
// Initialize hardware config
AuroraHardwareConfig hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
hardware.initializeWithOdometry();

// Pass to AuroraManager (recommended)
AuroraManager robotManager = new AuroraManager(hardwareMap, telemetry, true);
// AuroraManager automatically creates and uses hardware config internally

// Or use hardware config directly with subsystems
EnhancedDecodeHelper shooter = new EnhancedDecodeHelper(hardware);
SmartMechanumDrive drive = new SmartMechanumDrive(
    hardware.getFrontLeftMotor(),
    hardware.getFrontRightMotor(),
    hardware.getBackLeftMotor(),
    hardware.getBackRightMotor(),
    null,
    hardware.getVoltageSensor()
);
```

## Hardware Device Names

All hardware device names are defined as constants in `AuroraHardwareConfig.java`:

### Drive Motors
- `FRONT_LEFT_MOTOR` = "frontLeft"
- `FRONT_RIGHT_MOTOR` = "frontRight"
- `BACK_LEFT_MOTOR` = "backLeft"
- `BACK_RIGHT_MOTOR` = "backRight"

### Shooter System
- `SHOOTER_MOTOR` = "shooter"
- `FEED_SERVO_1` = "servo1"
- `FEED_SERVO_2` = "servo2"
- `LIGHT_SERVO` = "light" (optional)

### Odometry
- `ODOMETRY_COMPUTER` = "odo"

## Configuring Your Robot

To match your robot's hardware configuration, you only need to change the device names in **ONE place**:

1. Open `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/AuroraHardwareConfig.java`
2. Find the section labeled "HARDWARE DEVICE NAMES"
3. Update the string constants to match your Driver Station configuration:

```java
// Example: If your front left motor is named "motor1" instead of "frontLeft"
public static final String FRONT_LEFT_MOTOR = "motor1";  // Changed from "frontLeft"
```

## Odometry Configuration

Odometry pod offsets are also centralized in `AuroraHardwareConfig.java`:

```java
// Odometry pod offsets (in inches)
private static final double ODOMETRY_X_OFFSET = 4.71;   // Right from center
private static final double ODOMETRY_Y_OFFSET = -6.62;  // Forward from center

// Pod directions
private static final GoBildaPinpointDriver.EncoderDirection FORWARD_POD_DIRECTION = 
    GoBildaPinpointDriver.EncoderDirection.FORWARD;
private static final GoBildaPinpointDriver.EncoderDirection STRAFE_POD_DIRECTION = 
    GoBildaPinpointDriver.EncoderDirection.REVERSED;
```

To change these for your robot:
1. Measure the physical offsets of your odometry pods
2. Update the `ODOMETRY_X_OFFSET` and `ODOMETRY_Y_OFFSET` constants
3. Adjust pod directions if needed

## Initialization Methods

### `initialize()`
Use for **Autonomous OpModes** that don't need continuous odometry tracking:
```java
hardware.initialize();  // No odometry
```

### `initializeWithOdometry()`
Use for **TeleOp OpModes** that need position tracking:
```java
hardware.initializeWithOdometry();  // With odometry
```

## Error Handling

The hardware config provides detailed error information:

```java
hardware.initialize();

// Check initialization status
if (!hardware.isDriveSystemInitialized()) {
    telemetry.addLine("Drive system error: " + hardware.getDriveInitError());
}

if (!hardware.isShooterSystemInitialized()) {
    telemetry.addLine("Shooter system error: " + hardware.getShooterInitError());
}

// Get a complete summary
String summary = hardware.getInitializationSummary();
telemetry.addLine(summary);
```

## Available Hardware Accessors

### Motors
```java
DcMotor frontLeft = hardware.getFrontLeftMotor();
DcMotor frontRight = hardware.getFrontRightMotor();
DcMotor backLeft = hardware.getBackLeftMotor();
DcMotor backRight = hardware.getBackRightMotor();
DcMotor shooter = hardware.getShooterMotor();
```

### Servos
```java
CRServo feedServo1 = hardware.getFeedServo1();
CRServo feedServo2 = hardware.getFeedServo2();
Servo light = hardware.getLightServo();  // May be null if not configured
```

### Sensors
```java
VoltageSensor voltage = hardware.getVoltageSensor();  // May be null
GoBildaPinpointDriver odo = hardware.getOdometry();   // May be null if not initialized
```

### Other
```java
HardwareMap hardwareMap = hardware.getHardwareMap();  // Original hardwareMap if needed
```

## Migration Guide for Existing OpModes

If you have existing OpModes that use direct `hardwareMap.get()` calls:

### Before:
```java
public void runOpMode() {
    DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
    DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    CRServo feedServo1 = hardwareMap.get(CRServo.class, "servo1");
    
    // ... rest of code
}
```

### After:
```java
public void runOpMode() {
    AuroraHardwareConfig hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
    hardware.initialize();
    
    DcMotor shooter = hardware.getShooterMotor();
    DcMotor frontLeft = hardware.getFrontLeftMotor();
    CRServo feedServo1 = hardware.getFeedServo1();
    
    // ... rest of code
}
```

## Using with AuroraManager (Recommended)

The simplest approach is to use `AuroraManager`, which automatically uses `AuroraHardwareConfig`:

```java
@TeleOp(name="My TeleOp")
public class MyTeleOp extends LinearOpMode {
    private AuroraManager robotManager;
    
    @Override
    public void runOpMode() {
        // AuroraManager handles all hardware initialization internally
        robotManager = new AuroraManager(hardwareMap, telemetry);
        
        waitForStart();
        
        while (opModeIsActive()) {
            robotManager.update(gamepad1, gamepad2);
        }
    }
}
```

For autonomous:
```java
@Autonomous(name="My Auto")
public class MyAuto extends LinearOpMode {
    private AuroraManager robotManager;
    
    @Override
    public void runOpMode() {
        // Disable odometry for autonomous (optional)
        robotManager = new AuroraManager(hardwareMap, telemetry, false);
        
        waitForStart();
        
        // Use robotManager.getDriveSystem(), robotManager.getShooterSystem(), etc.
    }
}
```

## Troubleshooting

### "Motor not found" or "Device not found" errors
1. Check your Driver Station hardware configuration
2. Verify device names match the constants in `AuroraHardwareConfig.java`
3. Make sure all cables are properly connected

### Odometry not initializing
1. Ensure you called `initializeWithOdometry()` instead of `initialize()`
2. Check that the odometry device is named "odo" in Driver Station
3. Verify the goBILDA Pinpoint is connected to the correct I2C port

### Wrong motor spinning
1. Check motor directions in `SmartMechanumDrive.java`
2. Verify you have the correct motor plugged into each port
3. Update device names in `AuroraHardwareConfig.java` if needed

## Best Practices

1. **Always use AuroraHardwareConfig** for new OpModes instead of direct hardwareMap.get() calls
2. **Check initialization status** before using hardware components
3. **Update device names in one place** (AuroraHardwareConfig.java) rather than throughout code
4. **Use AuroraManager when possible** for the most integrated experience
5. **Test hardware configuration** at the start of each practice session

## Example: Complete TeleOp OpMode

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager;

@TeleOp(name="Simple AURORA TeleOp", group="Competition")
public class SimpleAURORATeleOp extends LinearOpMode {
    
    private AuroraManager robotManager;
    
    @Override
    public void runOpMode() {
        // Initialize AURORA system (automatically uses AuroraHardwareConfig)
        telemetry.addLine("Initializing AURORA Robot Systems...");
        telemetry.update();
        
        robotManager = new AuroraManager(hardwareMap, telemetry);
        
        telemetry.clear();
        telemetry.addLine("✅ AURORA Robot Ready!");
        telemetry.addLine("");
        telemetry.addLine(robotManager.getHardware().getInitializationSummary());
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update all robot systems
            robotManager.update(gamepad1, gamepad2);
            
            // Optional: Add custom telemetry
            telemetry.addData("System Status", 
                robotManager.isSystemsHealthy() ? "✅ OK" : "⚠️ Warning");
            telemetry.update();
        }
    }
}
```

## Questions?

For more information, see:
- `AuroraHardwareConfig.java` - The hardware configuration class source code
- `AuroraManager.java` - How the hardware config is used in the manager
- `EnhancedDecodeHelper.java` - Example of subsystem using hardware config
