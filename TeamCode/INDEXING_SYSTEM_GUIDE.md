# Aurora Push-Based Artifact Indexing System Guide

## Overview

The Aurora Push-Based Artifact Indexing System is a sophisticated mechanism for managing artifact collection, storage, and firing on the Aurora robot. The system implements a **push-based** indexing approach where artifacts physically push each other into storage positions, respecting mechanical constraints.

## Key Concepts

### Push-Based Indexing

Unlike software-controlled indexing where motors move artifacts independently, this system uses **physical pushing** to position artifacts:

- **Rollers run continuously inward** (main collection mechanism)
- **Intake transfer servos** move artifacts from intakes into center
- **Center transfer servos** complete the transfer AND push artifacts out to intakes
- Artifacts are moved by being pushed by other artifacts
- This creates a deterministic, mechanically-enforced shot order

### Hardware Operation

**Rollers (Motors):**
- Run continuously inward at all times
- Full power during collection mode
- Reduced power (30%) when intake is in storage mode (holding an artifact)

**Intake Transfer Servos:**
- One for front intake, one for back intake
- Transfer artifacts from their intake into the center

**Center Transfer Servos:**
- Two servos (left and right)
- Accept artifacts from intake transfer servos
- Complete the move into center storage
- Push artifacts out of center into empty intakes during indexing

### Three-Artifact Capacity

The system can hold up to 3 artifacts at once:
- 1 in center storage (ready to fire)
- 1 in front intake storage
- 1 in back intake storage

### Indexing Rules

#### First Artifact Collected
1. Goes directly into **center storage**
2. Cannot be moved until pushed by the second artifact
3. Waits in center position

#### Second Artifact Collected
1. Enters center storage
2. **Pushes** the first artifact into the **opposite intake** for storage
3. Remains in center storage
4. Becomes the **forced first shot** (mechanically determined)

#### Third Artifact Collected
1. Stored in the **same intake** it was collected from
2. Does NOT push any other artifact
3. Waits in intake storage

### Artifact Detection

The system uses both distance and color sensors to accurately detect artifacts:

**Detection Criteria:**
- Distance sensor reads < 10cm (adjustable)
- Color is NOT yellow (yellow indicates non-artifact object)

**Color Sensors:**
- 3 sensors per intake (6 total)
- Readings are averaged for accuracy
- Detect purple and green artifacts
- Reject yellow objects that aren't artifacts

### Motif Pattern and Shot Planning

The system uses a **motif pattern** (determined by limelight camera) to optimize shot order:

**Motif Patterns:**
- `PPG` - Purple, Purple, Green
- `PGP` - Purple, Green, Purple  
- `GPP` - Green, Purple, Purple

The limelight camera system sets the motif pattern, which determines the optimal shooting order.

### Early Fire Handling

The system supports firing before reaching full capacity:

**One Artifact:**
- Artifact should already be in center
- Can fire immediately

**Two Artifacts (Smart Rearrangement):**
- System checks motif pattern for desired first shot color
- If center artifact matches desired color → fire it
- If storage artifact matches desired color → rearrange using empty intake
- **Example**: Green in center, Purple in back, pattern wants Purple first
  - Push green to front intake (empty)
  - Pull purple from back to center
  - Now purple is ready to fire first

**Three Artifacts (No Rearrangement):**
- Fire the artifact in center (mechanically forced)
- No empty intake available for rearrangement
- Follow motif pattern for remaining shots

## System Components

### 1. Artifact Class

Represents an individual game artifact with:
- **Color**: PURPLE, GREEN, or UNKNOWN
- **Location**: FRONT_INTAKE, BACK_INTAKE, CENTER_STORAGE, FIRED
- **Collection Order**: 1, 2, or 3
- **Timestamp**: When it was collected

```java
Artifact artifact = new Artifact(
    Artifact.Color.PURPLE,
    Artifact.Location.UNKNOWN,
    1  // Collection order
);
```

### 2. IndexingConfig Class

Centralized configuration for all timing parameters:

```java
IndexingConfig config = new IndexingConfig();

// Intake timing
config.setIntakeRollerTime(0.5);  // seconds
config.setIntakeRollerPower(0.8);

// Transfer timing
config.setTransferServoTime(0.4);
config.setCenterAcceptTime(0.3);

// Push timing
config.setSecondArtifactPushTime(0.6);
config.setStorageIntakeAcceptTime(0.4);

// Firing
config.setFireFeedTime(0.25);
config.setMinShooterSpinupTime(1.0);

// Sensors
config.setArtifactDetectionDistance(2.0);  // inches
config.setSensorDebounceTime(0.05);
```

### 3. IndexingSystem Class

Core state machine managing the indexing process:

**Key Methods:**
- `onArtifactDetected(Artifact, IntakeSource)` - Called when artifact detected
- `onFireSignal()` - Called when fire button pressed
- `update()` - Periodic update (call every loop)
- `reset()` - Reset system to initial state

**State Machine States:**
- `IDLE` - Ready for operations
- `COLLECTING` - Actively collecting artifact
- `TRANSFERRING` - Moving artifact to center
- `PUSHING` - Pushing first artifact to storage
- `READY_TO_FIRE` - Artifact in center, ready to fire
- `FIRING` - Actively firing
- `ERROR` - Error detected

### 4. Shooter Class

Wrapper around DecodeHelper providing clean interface:

```java
Shooter shooter = new Shooter(hardware, config, telemetry);

// Enable and spin up
shooter.enable();
shooter.spinUp(ShooterConfig.ShooterPreset.LONG_RANGE);

// Check if ready
if (shooter.isReadyToFire()) {
    shooter.fire();
}

// Disable when done
shooter.disable();
```

### 5. AuroraHardwareConfig Extensions

New hardware components added:

**Motors:**
- `frontRollerMotor` - Front intake roller (runs continuously inward, slower when in storage mode)
- `backRollerMotor` - Back intake roller (runs continuously inward, slower when in storage mode)

**Servos:**
- `frontTransferServo` - Front intake transfer servo (transfers artifacts from front intake into center)
- `backTransferServo` - Back intake transfer servo (transfers artifacts from back intake into center)
- `transferServoCL` - Center left transfer servo (completes transfer into center, pushes artifacts out to intakes)
- `transferServoCR` - Center right transfer servo (completes transfer into center, pushes artifacts out to intakes)

**Distance Sensors (goBILDA Laser Distance Sensors in Analog Mode):**
- `frontDistanceSensor` (frontDist) - Detect artifacts at front intake
- `backDistanceSensor` (backDist) - Detect artifacts at back intake
- Configured as `AnalogInput` devices (0-3.3V = 0-1000mm)
- Artifact detected when distance < 100mm (10cm, adjustable)

**Color Sensors (REV Color Sensor V3 - 3 per intake, 6 total):**
- Front intake: `frontLeftColorSensor`, `frontRightColorSensor`, `frontCenterColorSensor`
- Back intake: `backRightColorSensor`, `leftRightColorSensor`, `backCenterColorSensor`
- Configured as `NormalizedColorSensor` devices (provides normalized RGB values 0-1)
- Used for accurate color detection (purple/green) and to reject yellow non-artifacts
- Readings from all 3 sensors per intake are averaged for better accuracy

## Hardware Setup

### Device Names in Driver Station

Update these in `AuroraHardwareConfig.java` to match your configuration:

```java
// Intake and Indexing System
public static final String FRONT_ROLLER_MOTOR = "frontRollerMotor";
public static final String BACK_ROLLER_MOTOR = "backRollerMotor";
public static final String FRONT_TRANSFER_SERVO = "frontTransferServo";
public static final String BACK_TRANSFER_SERVO = "backTransferServo";
public static final String TRANSFER_SERVO_CL = "transferServoCL";
public static final String TRANSFER_SERVO_CR = "transferServoCR";

// Distance Sensors
public static final String FRONT_DISTANCE_SENSOR = "frontDist";
public static final String BACK_DISTANCE_SENSOR = "backDist";

// Color Sensors
public static final String FRONT_LEFT_COLOR_SENSOR = "frontLeftColor";
public static final String FRONT_RIGHT_COLOR_SENSOR = "frontRightColor";
public static final String BACK_RIGHT_COLOR_SENSOR = "backRightColor";
public static final String LEFT_RIGHT_COLOR_SENSOR = "leftRightColor";
public static final String FRONT_CENTER_COLOR_SENSOR = "frontCenterColor";
public static final String BACK_CENTER_COLOR_SENSOR = "backCenterColor";
```

### Distance Sensor Configuration

The system uses **goBILDA Laser Distance Sensors in Analog Mode**:

1. **Driver Station Configuration:**
   - Configure sensors as `AnalogInput` devices (not `DistanceSensor`)
   - Name them `frontDist` and `backDist`
   - Connect to analog input ports on the Control Hub

2. **Sensor Calibration:**
   - Sensors output 0-3.3V corresponding to 0-1000mm distance
   - System automatically converts voltage to distance
   - Detection threshold: 100mm (10cm) by default
   - Adjustable via `IndexingConfig.setArtifactDetectionDistanceCm()`

3. **Alternative: Digital Mode:**
   - If using digital mode, sensors must be reconfigured as `DigitalChannel`
   - Adjust potentiometer on sensor for detection distance (25-264mm)
   - Code modifications required in `AuroraHardwareConfig`

### Color Sensor Configuration

The system uses **REV Color Sensor V3** for artifact color detection:

1. **Driver Station Configuration:**
   - Configure sensors as `NormalizedColorSensor` devices (or `REV Color Sensor V3`)
   - Name them according to their position:
     - Front intake: `frontLeftColor`, `frontRightColor`, `frontCenterColor`
     - Back intake: `backRightColor`, `leftRightColor`, `backCenterColor`
   - Connect to I2C ports on the Control Hub

2. **Important I2C Considerations:**
   - Color Sensor V3 and 2m Distance Sensor share the same I2C address
   - **Do not** configure both on the same I2C bus
   - Distribute sensors across available I2C buses

3. **Color Detection:**
   - Sensors provide normalized RGB values (0-1 range)
   - System averages readings from all 3 sensors per intake for accuracy
   - Detects purple (high red+blue, low green) and green (high green, low red+blue)
   - Rejects yellow objects (high red+green, low blue) as non-artifacts

4. **Example Code:**
   ```java
   // Reading color from REV Color Sensor V3
   NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "frontLeftColor");
   NormalizedRGBA colors = sensor.getNormalizedColors();
   
   telemetry.addData("Red", "%.3f", colors.red);    // 0-1 range
   telemetry.addData("Green", "%.3f", colors.green);
   telemetry.addData("Blue", "%.3f", colors.blue);
   ```

### Motor Directions

Configure motor directions in hardware initialization if needed:
- Front roller: Inward (continuous)
- Back roller: Inward (continuous)

Adjust in `AuroraHardwareConfig.initializeIndexingSystem()` if your robot differs.

## Usage Examples

### Basic Setup

```java
// In your OpMode
@Override
public void runOpMode() {
    // Initialize hardware
    AuroraHardwareConfig hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
    hardware.initializeWithOdometry();
    
    // Initialize shooter
    ShooterConfig shooterConfig = new ShooterConfig();
    Shooter shooter = new Shooter(hardware, shooterConfig, telemetry);
    
    // Initialize indexing config
    IndexingConfig indexingConfig = new IndexingConfig();
    
    // Initialize indexing system
    IndexingSystem indexingSystem = new IndexingSystem(
        hardware, 
        indexingConfig, 
        shooter, 
        telemetry
    );
    
    waitForStart();
    
    while (opModeIsActive()) {
        // Update systems
        shooter.update();
        indexingSystem.update();
        
        // Your code here
    }
}
```

### Setting Motif Pattern

```java
// Called by limelight/camera system when pattern is detected
// Pattern can be "PPG", "PGP", or "GPP"
boolean success = indexingSystem.setMotifPattern("PPG");

if (success) {
    // Pattern set successfully
    // System will use this for shot planning and rearrangement
}

// Check if pattern has been set
if (indexingSystem.isMotifPatternSet()) {
    String pattern = indexingSystem.getMotifPattern();
    // Use pattern info...
}
```

### Collecting Artifacts

```java
// When artifact detected at front intake
Artifact artifact = new Artifact(
    Artifact.Color.PURPLE,  // Detected color
    Artifact.Location.UNKNOWN,
    0  // Will be assigned by system
);

boolean success = indexingSystem.onArtifactDetected(
    artifact,
    IndexingSystem.IntakeSource.FRONT
);

if (!success) {
    // Collection failed (system full or busy)
}
```

### Firing Artifacts

```java
// Enable shooter first
shooter.enable();
shooter.spinUp();

// Wait for shooter to be ready
while (!shooter.isReadyToFire() && opModeIsActive()) {
    shooter.update();
    sleep(20);
}

// Fire when ready
if (indexingSystem.onFireSignal()) {
    // Firing started successfully
}
```

### Monitoring Status

```java
// Check system state
IndexingSystem.SystemState state = indexingSystem.getCurrentState();
int artifactCount = indexingSystem.getArtifactCount();
boolean ready = indexingSystem.isReadyToFire();

// Check artifact locations
Artifact inCenter = indexingSystem.getArtifactInCenter();
Artifact inFront = indexingSystem.getArtifactInFrontIntake();
Artifact inBack = indexingSystem.getArtifactInBackIntake();

// Check shot plan
Artifact secondShot = indexingSystem.getPlannedSecondShot();
Artifact thirdShot = indexingSystem.getPlannedThirdShot();

// Display status
telemetry.addData("State", state);
telemetry.addData("Artifacts", artifactCount + "/3");
telemetry.addData("Ready", ready);
```

## Shot Planning

The system automatically plans shots 2 and 3 after artifacts are collected:

### Forced First Shot
- The **second artifact collected** is always the first shot
- This is mechanically enforced by the push-based system
- Software cannot change this

### Planned Shots
- **Second shot**: Planned by software based on strategy
- **Third shot**: Planned by software based on strategy

Default strategy uses collection order, but can be enhanced with:
- Color-based prioritization
- Alliance-specific strategies
- Game situation awareness

### Custom Shot Planning

Extend the `planShots()` method in IndexingSystem:

```java
private void planShots() {
    List<Artifact> available = new ArrayList<>();
    for (Artifact a : artifacts) {
        if (a.getLocation() != Artifact.Location.FIRED && 
            a.getLocation() != Artifact.Location.CENTER_STORAGE) {
            available.add(a);
        }
    }
    
    // Custom strategy: prioritize purple artifacts
    available.sort((a1, a2) -> {
        if (a1.getColor() == Artifact.Color.PURPLE) return -1;
        if (a2.getColor() == Artifact.Color.PURPLE) return 1;
        return a1.getCollectionOrder() - a2.getCollectionOrder();
    });
    
    if (available.size() > 0) plannedSecondShot = available.get(0);
    if (available.size() > 1) plannedThirdShot = available.get(1);
}
```

## Tuning Parameters

### Collection Timing

Adjust these if artifacts aren't collecting reliably:

```java
config.setIntakeRollerTime(0.5);      // Time to run intake (seconds)
config.setIntakeRollerPower(0.8);     // Intake motor power (0-1)
config.setCenterAcceptTime(0.3);      // Time for center to accept
```

### Transfer Timing

Adjust for reliable center transfers:

```java
config.setTransferServoTime(0.4);     // Servo movement time
config.setTransferServoTransferPosition(0.7);  // Transfer position
config.setTransferServoIdlePosition(0.2);      // Idle position
```

### Push Timing

Critical for second artifact push operation:

```java
config.setSecondArtifactPushTime(0.6);        // Push duration
config.setStorageIntakeAcceptTime(0.4);       // Storage accept time
config.setPushStartDelay(0.1);                // Delay before push
```

### Firing Timing

```java
config.setFireFeedTime(0.25);                 // Feed time during fire
config.setMinShooterSpinupTime(1.0);         // Min shooter ready time
config.setPostFireDelay(0.3);                // Delay after firing
```

### Sensor Configuration

```java
config.setArtifactDetectionDistance(2.0);    // Detection distance (inches)
config.setSensorDebounceTime(0.05);          // Debounce time (seconds)
config.setColorConfidenceThreshold(0.6);     // Color detection threshold
```

## Safety Features

### Safety Checks

The system includes safety checks to prevent illegal operations:

```java
config.setEnableSafetyChecks(true);  // Enable/disable safety
```

**Checks include:**
- Cannot collect more than 3 artifacts
- Cannot fire without artifact in center
- Cannot start new operation while busy
- Timeout detection for stuck operations

### Auto Recovery

```java
config.setEnableAutoRecovery(true);  // Enable auto-recovery
```

If enabled, system attempts to recover from error states automatically.

### Emergency Stop

```java
// Stop all indexing operations
indexingSystem.reset();
hardware.stopAllMotors();
```

## Troubleshooting

### Artifacts Not Collecting

**Check:**
1. Intake motors are running (check power levels)
2. Sensors detecting artifacts (check distances)
3. Timing parameters adequate (increase roller time)
4. System not full or busy

### Artifacts Not Transferring to Center

**Check:**
1. Transfer servo moving to correct position
2. Center roller running
3. Timing adequate (increase accept time)
4. No mechanical obstructions

### Push Operation Not Working

**Check:**
1. First artifact is in center before second collection
2. Opposite intake accepting pushed artifact
3. Push timing adequate
4. Mechanical alignment correct

### Firing Not Working

**Check:**
1. Shooter is enabled and ready
2. Artifact is in center storage
3. Minimum spinup time has elapsed
4. Feed mechanism operating correctly

### Sensor Issues

**Check:**
1. Sensors properly connected
2. Detection distance appropriate
3. Color sensors have adequate lighting
4. Debounce time set correctly

## Advanced Topics

### Event-Driven Detection

For automatic artifact detection:

```java
// In your main loop
if (isArtifactAtFront()) {
    Artifact.Color color = detectColorAtFront();
    Artifact artifact = new Artifact(color, Artifact.Location.UNKNOWN, 0);
    indexingSystem.onArtifactDetected(artifact, IntakeSource.FRONT);
}
```

### Integration with Autonomous

```java
// In autonomous, use indexing system for scoring
indexingSystem.onArtifactDetected(preloadArtifact, IntakeSource.FRONT);

// Wait for transfer
while (indexingSystem.getCurrentState() != SystemState.READY_TO_FIRE) {
    indexingSystem.update();
    sleep(20);
}

// Fire when positioned
if (atScoringPosition()) {
    shooter.enable();
    shooter.spinUp();
    // Wait for ready...
    indexingSystem.onFireSignal();
}
```

### Multiple Firing Sequences

```java
// Fire all three artifacts in sequence
for (int i = 0; i < 3; i++) {
    // Wait for ready
    while (!indexingSystem.isReadyToFire() || !shooter.isReadyToFire()) {
        shooter.update();
        indexingSystem.update();
        sleep(20);
    }
    
    // Fire
    indexingSystem.onFireSignal();
    
    // Wait for firing to complete
    while (indexingSystem.getCurrentState() == SystemState.FIRING) {
        indexingSystem.update();
        sleep(20);
    }
}
```

## Best Practices

1. **Always call update()** - Both shooter and indexing system need periodic updates
2. **Check ready states** - Before firing, ensure both systems are ready
3. **Handle errors** - Check return values and error counts
4. **Tune gradually** - Adjust timing parameters incrementally
5. **Test thoroughly** - Test all scenarios (1, 2, 3 artifacts)
6. **Monitor telemetry** - Use debug telemetry during development
7. **Respect constraints** - Don't try to force physically impossible operations

## Example OpMode

See `IndexingSystemExample.java` for a complete working example demonstrating:
- System initialization
- Artifact detection simulation
- Firing control
- Status monitoring
- Error handling

## Support and Contributions

For issues or enhancements:
1. Test with example OpMode first
2. Check configuration parameters
3. Review telemetry output
4. Verify hardware connections
5. Consult this guide

## Version History

### v1.0 - Initial Implementation
- Push-based indexing logic
- Three-artifact capacity
- Early fire handling
- Shot planning
- Hardware integration
- Safety features
- Example OpMode
- Complete documentation
