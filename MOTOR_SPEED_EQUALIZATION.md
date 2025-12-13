# Motor Speed Equalization System

## Overview

The Motor Speed Equalization system is a software-based solution for compensating uneven motor speeds caused by physical factors such as weight distribution, mechanical tolerances, or varying loads. Despite sending equal power to all motors, physical differences can cause motors to move at different speeds, leading to unwanted robot behavior (e.g., rotation during forward movement).

This system uses encoder feedback to measure actual motor speeds in real-time and applies corrections to ensure all motors move at equal speeds.

## Features

- **Real-time Encoder-based Speed Measurement**: Monitors actual motor speeds using encoder deltas
- **Multiple Correction Modes**:
  - **AGGRESSIVE**: Increases power to slower motors (limited by max power)
  - **CONSERVATIVE**: Decreases power to faster motors to match the slowest
  - **ACCELERATION_SYNC**: Synchronizes acceleration/deceleration rates (NEW - best for direction changes)
  - **DISABLED**: No speed equalization (default)
- **Optional Enable/Disable Toggle**: Can be turned on/off during runtime
- **Acceleration Monitoring**: Tracks acceleration rates to detect balance issues
- **Configurable Parameters**: Fine-tune correction behavior for your robot
- **Telemetry Support**: Detailed diagnostics for tuning and debugging

## How It Works

### Aggressive Mode
When a motor is detected to be running slower than others:
1. Calculate the speed difference between the slowest and fastest motors
2. Apply a power boost to the slower motor (up to configured maximum)
3. Other motors maintain their target power
4. Best for: Maximizing speed while compensating for load imbalances

**Example**: If motor "LB" (left back) is 20% slower due to weight, it receives up to 15% extra power to catch up.

### Conservative Mode
When motors are running at different speeds:
1. Find the slowest motor's speed
2. Reduce power to faster motors to match the slowest motor's speed
3. Ensures all motors move at the same rate
4. Best for: Precise movements and maintaining straight lines

**Example**: If three motors run at 100% speed but one at 80%, the three faster motors are slowed down to 80%.

### Acceleration Sync Mode (NEW - Recommended)
**Specifically designed to fix deceleration issues during direction changes:**
1. Continuously monitors acceleration/deceleration rates of all motors
2. Detects when motors are decelerating at different rates
3. Applies corrections to ensure all motors slow down together
4. Prevents rotation caused by uneven deceleration
5. Best for: Quick direction changes, stopping accurately, preventing drift during deceleration

**Example**: During a quick direction change, if the left motors decelerate faster than the right motors (causing unwanted rotation), this mode adds power to the faster-decelerating motors to slow their deceleration, keeping all motors synchronized.

**Why this mode is important**: The most common issue with uneven weight distribution is that during deceleration or direction changes, lighter motors stop faster than heavier motors, causing the robot to rotate. This mode directly addresses that problem.

## Installation

The system is already integrated into `SmartMechanumDrive`. No additional installation is required.

## Usage

### Basic Usage (SmartMechanumDrive)

The simplest way to use motor speed equalization is through the integrated `SmartMechanumDrive` class:

```java
// Create drive system (equalization is disabled by default)
SmartMechanumDrive drive = new SmartMechanumDrive(
    frontLeft, frontRight, backLeft, backRight,
    gamepad1, voltageSensor, odometry
);

// Enable speed equalization
drive.setSpeedEqualizationEnabled(true);

// Set correction mode
drive.setSpeedEqualizationMode(MotorSpeedEqualizer.CorrectionMode.AGGRESSIVE);

// In your loop
drive.update();  // Equalization happens automatically
```

### Advanced Configuration

For fine-tuning, access the equalizer directly:

```java
MotorSpeedEqualizer equalizer = drive.getSpeedEqualizer();

// Configure correction strength
equalizer.setAggressiveBoostFactor(0.15);  // Max 15% boost for slow motors
equalizer.setConservativeReductionFactor(0.15);  // Max 15% reduction for fast motors

// Configure tolerances
equalizer.setSpeedTolerancePercent(5.0);  // Only correct if >5% difference
equalizer.setMinSpeedThreshold(50.0);  // Only correct when moving >50 ticks/sec
equalizer.setAccelerationTolerancePercent(10.0);  // Acceleration variance threshold
```

### Standalone Usage

You can also use `MotorSpeedEqualizer` independently:

```java
// Create equalizer with your motors
DcMotor[] motors = new DcMotor[]{leftFront, rightFront, leftBack, rightBack};
String[] names = new String[]{"LF", "RF", "LB", "RB"};
MotorSpeedEqualizer equalizer = new MotorSpeedEqualizer(motors, names);

// Enable and configure
equalizer.setEnabled(true);
equalizer.setCorrectionMode(MotorSpeedEqualizer.CorrectionMode.AGGRESSIVE);

// In your control loop
double[] targetPowers = new double[]{power1, power2, power3, power4};
double[] correctedPowers = equalizer.update(targetPowers);

// Apply corrected powers
leftFront.setPower(correctedPowers[0]);
rightFront.setPower(correctedPowers[1]);
leftBack.setPower(correctedPowers[2]);
rightBack.setPower(correctedPowers[3]);
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `aggressiveBoostFactor` | 0.15 | Maximum power boost for slow motors (0.0-1.0) |
| `conservativeReductionFactor` | 0.15 | Maximum power reduction for fast motors (0.0-1.0) |
| `accelerationCorrectionFactor` | 0.20 | Maximum power adjustment for acceleration sync (0.0-1.0) |
| `speedTolerancePercent` | 5.0 | Speed difference threshold to trigger correction |
| `minSpeedThreshold` | 50.0 | Minimum speed (ticks/sec) to apply corrections |
| `accelerationTolerancePercent` | 10.0 | Acceleration variance threshold for balance check |
| `updateIntervalSeconds` | 0.05 | Update rate (20Hz) |

## Telemetry

The system provides detailed telemetry for diagnostics:

```java
MotorSpeedEqualizer.EqualizerTelemetryData data = equalizer.getTelemetryData();

// Display motor speeds
for (int i = 0; i < data.speeds.length; i++) {
    telemetry.addData(data.motorNames[i] + " Speed", "%.1f ticks/sec", data.speeds[i]);
    telemetry.addData(data.motorNames[i] + " Correction", "%+.3f", data.corrections[i]);
}

// Display status
telemetry.addData("Mode", data.mode.getName());
telemetry.addData("Corrections Applied", data.correctionCount);
telemetry.addData("Acceleration Balanced", data.accelerationBalanced ? "Yes" : "No");
```

## Example OpMode

See `MotorSpeedEqualizationExample.java` in the `examples` package for a complete working example with:
- Toggle equalization on/off
- Cycle through correction modes
- Real-time telemetry display

## When to Use Each Mode

### ACCELERATION_SYNC Mode (RECOMMENDED - Start Here)
**Use when:**
- ⚠️ **You experience rotation during direction changes** (most common issue)
- Motors decelerate at different rates
- Robot behaves unpredictably when stopping
- Quick direction changes cause unwanted turning
- You want synchronized acceleration and deceleration

**Advantages:**
- **Directly fixes the deceleration problem** that causes rotation
- Works during both acceleration and deceleration
- Especially effective for quick direction changes
- Maintains control during stops
- Does not require motors to be at specific speeds

**Disadvantages:**
- Slightly more complex than speed-based modes
- May take a few cycles to synchronize

**Recommended for**: Most robots with uneven weight distribution, especially if you notice rotation during stops or direction changes.

### AGGRESSIVE Mode
**Use when:**
- You want maximum speed
- Weight distribution causes one side to be slower during steady movement
- You have enough motor headroom (not already at max power)
- Battery life is not a primary concern

**Advantages:**
- Maintains desired speed
- Better for competitive scenarios
- Compensates for heavy loads

**Disadvantages:**
- Increases power consumption
- May not work if slower motors are already at max power
- Does NOT fix deceleration issues

### CONSERVATIVE Mode
**Use when:**
- Precision is more important than speed
- You want to minimize power consumption
- You need guaranteed synchronization
- Battery life is a concern

**Advantages:**
- Lower power consumption
- Always works (can always reduce power)
- More predictable behavior
- Longer battery life

**Disadvantages:**
- Robot moves slower overall
- May not meet speed requirements
- Does NOT fix deceleration issues

### DISABLED Mode
**Use when:**
- Testing baseline performance
- Robot has balanced weight distribution
- Speed equalization causes issues
- Running autonomous with pre-tuned powers

## Tuning Tips

1. **Start with DISABLED**: Test your robot without equalization to understand the baseline behavior

2. **Enable and Observe**: Turn on equalization with default settings and watch telemetry to see which motors need correction

3. **Choose the Right Mode**: 
   - If motors are often at max power → use CONSERVATIVE
   - If motors have headroom → use AGGRESSIVE

4. **Adjust Tolerance**: 
   - If corrections are too frequent → increase `speedTolerancePercent`
   - If corrections are too rare → decrease `speedTolerancePercent`

5. **Fine-tune Correction Strength**:
   - Start with 0.10 (10%) and gradually increase
   - Too high → oscillations and jittery movement
   - Too low → ineffective corrections

6. **Test Different Scenarios**:
   - Forward/backward movement
   - Strafing left/right
   - Rotation
   - Diagonal movements
   - With/without game pieces or manipulators extended

## Troubleshooting

### Corrections Not Being Applied
- Check that equalization is enabled: `setSpeedEqualizationEnabled(true)`
- Verify mode is not DISABLED
- Check if speeds are above `minSpeedThreshold`
- Check if speed variance is above `speedTolerancePercent`

### Robot Jittery or Oscillating
- Reduce correction factors (try 0.10 or lower)
- Increase `speedTolerancePercent` to reduce sensitivity
- Increase `updateIntervalSeconds` to slow down corrections

### One Motor Still Slower
- In AGGRESSIVE mode: Motor may be at max power already
- Switch to CONSERVATIVE mode to synchronize at slower speed
- Check for mechanical issues or motor damage

### Battery Drains Too Fast
- Switch from AGGRESSIVE to CONSERVATIVE mode
- Reduce correction factors
- Consider fixing the root cause (weight distribution) mechanically

## Technical Details

### Speed Calculation
Motor speeds are calculated by measuring encoder position deltas over time:
```
speed (ticks/sec) = (currentPosition - lastPosition) / deltaTime
```

### Acceleration Calculation
Acceleration is calculated as the change in speed over time:
```
acceleration (ticks/sec²) = (currentSpeed - lastSpeed) / deltaTime
```

### Correction Application

**Aggressive Mode:**
```
speedRatio = slowestSpeed / fastestSpeed
boostNeeded = (1.0 - speedRatio) * aggressiveBoostFactor
correctedPower = targetPower + boost
```

**Conservative Mode:**
```
speedRatio = slowestSpeed / currentMotorSpeed
reductionNeeded = (1.0 - speedRatio) * conservativeReductionFactor
correctedPower = targetPower - reduction
```

## Performance Considerations

- **Update Rate**: Default 20Hz (50ms) is a good balance between responsiveness and CPU usage
- **Computation Cost**: Minimal - simple arithmetic operations per motor
- **Memory Usage**: Small - only tracks current and last encoder positions
- **Motor Wear**: No negative impact - corrections are smooth and gradual

## Integration with Existing Systems

The equalization system integrates seamlessly with:
- Acceleration limiting in `SmartMechanumDrive`
- Field-relative driving
- Heading stabilization
- Battery voltage compensation
- All drive modes (Precision, Normal, Sport, etc.)

Corrections are applied **after** all other drive system features, ensuring compatibility.

## Credits

Implemented as part of the AURORA (Advanced Unified Robot Operating & Response Architecture) system for FTC Team.

## License

Copyright (c) 2025 FTC Team. All rights reserved.
