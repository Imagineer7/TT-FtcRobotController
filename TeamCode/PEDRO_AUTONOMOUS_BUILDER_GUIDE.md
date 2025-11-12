# Pedro Autonomous Builder - Complete Guide

## Overview
The `PedroAutonomousBuilder` is a powerful helper class that dramatically simplifies creating complex autonomous routines with Pedro Pathing. It handles state management, action sequencing, position tracking during actions, and shooter integration automatically.

## Key Features

✅ **Automatic State Machine Generation** - No manual state constants or switch statements needed
✅ **Continuous Position Tracking** - Robot position is always tracked, even during actions
✅ **Shooter Integration** - Built-in support for `EnhancedDecodeHelper` shooting actions
✅ **Action Insertion** - Easily add actions between any paths
✅ **Heading Alignment** - Automatic turn-to-heading actions
✅ **Custom Actions** - Lambda support for any custom mechanism control
✅ **Conditional Branching** - Execute different sequences based on conditions
✅ **Wait/Pause** - Simple delay actions
✅ **Progress Tracking** - Built-in step counting and telemetry

## Quick Start

### Basic Usage

```java
// In your OpMode init()
PedroAutonomousBuilder autoBuilder = new PedroAutonomousBuilder(follower)
        .withShooter(shooter)
        .addPath(paths.Path1)
        .addPath(paths.Path2)
        .addPath(paths.Path3);

// In your start()
autoBuilder.start();

// In your loop()
follower.update();  // IMPORTANT: Update follower first
String currentStep = autoBuilder.update();
```

## Available Actions

### 1. Path Following
```java
.addPath(paths.Path1)
```
Follows a PathChain. Automatically waits for completion before next step.

### 2. Shooting Action
```java
.addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE)
```
Fires specified number of shots using EnhancedDecodeHelper. 
- Automatically handles shooter spinup
- Maintains position tracking while shooting
- Waits for all shots to complete

**Parameters:**
- `numShots` - Number of shots to fire
- `preset` - ShooterConfig preset (LONG_RANGE, SHORT_RANGE, RAPID_FIRE, etc.)

### 3. Turn to Heading
```java
.addTurnToHeading(Math.toRadians(90))  // Turn to 90 degrees
```
Turns robot to specific heading (in radians).
- Creates a point turn path
- 2-degree tolerance
- Useful before shooting actions

### 4. Wait/Pause
```java
.addWait(1.5)  // Wait 1.5 seconds
```
Pauses for specified duration (in seconds).
- Robot position continues to be tracked
- Useful for allowing mechanisms to settle

### 5. Custom Action
```java
.addCustomAction("Deploy Intake", (follower, elapsedTime) -> {
    // Your mechanism control code here
    if (elapsedTime < 0.5) {
        intakeServo.setPosition(0.8);
    }
    return elapsedTime > 1.0;  // Complete when true
})
```
Execute custom code with full control.
- Takes a name (for telemetry) and lambda function
- Lambda receives follower (for position) and elapsed time
- Return true when action is complete

### 6. Conditional Branching
```java
.addConditional(
    (follower) -> sampleDetected,  // Condition
    trueBuilder,   // Execute if true
    falseBuilder   // Execute if false
)
```
Choose between different sequences based on runtime conditions.

## Real-World Example

### Competition Autonomous with Scoring

```java
private void buildAutonomousSequence() {
    autoBuilder = new PedroAutonomousBuilder(follower)
            .withShooter(shooter)
            
            // Preload specimen
            .addPath(paths.ToSubmersible)
            .addTurnToHeading(Math.toRadians(270))
            .addWait(0.2)
            .addCustomAction("Score Specimen", (f, t) -> {
                if (t < 0.5) {
                    clawServo.setPosition(CLAW_OPEN);
                } else if (t < 1.0) {
                    slideMotor.setPower(-0.8);  // Retract
                }
                return t > 1.2;
            })
            
            // First sample cycle
            .addPath(paths.ToSample1)
            .addCustomAction("Grab Sample", (f, t) -> {
                intakeServo.setPower(t < 0.8 ? 1.0 : 0.0);
                return t > 1.0;
            })
            
            .addPath(paths.ToBasket)
            .addTurnToHeading(Math.toRadians(45))
            .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)
            
            // Second sample cycle
            .addPath(paths.ToSample2)
            .addWait(0.3)
            .addCustomAction("Grab Sample", (f, t) -> {
                intakeServo.setPower(t < 0.8 ? 1.0 : 0.0);
                return t > 1.0;
            })
            
            .addPath(paths.ToBasket)
            .addTurnToHeading(Math.toRadians(45))
            .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)
            
            // Third sample cycle
            .addPath(paths.ToSample3)
            .addWait(0.3)
            .addCustomAction("Grab Sample", (f, t) -> {
                intakeServo.setPower(t < 0.8 ? 1.0 : 0.0);
                return t > 1.0;
            })
            
            .addPath(paths.ToBasket)
            .addTurnToHeading(Math.toRadians(45))
            .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)
            
            // Park
            .addPath(paths.ToPark)
            .addCustomAction("Stow Mechanisms", (f, t) -> {
                slideMotor.setPower(0);
                clawServo.setPosition(CLAW_CLOSED);
                return t > 0.5;
            });
}
```

## Position Tracking During Actions

**Key Feature:** The robot's position is ALWAYS tracked, even when not following a path.

This is crucial because:
1. The follower continues to update odometry during shooting
2. When resuming path following, position is accurate
3. No drift accumulates during pauses

The builder automatically calls `follower.update()` during ALL action types, ensuring continuous localization.

## Telemetry Integration

The builder provides useful information:

```java
String currentStep = autoBuilder.update();  // "Shooting (2/3)"
int currentIndex = autoBuilder.getCurrentStepIndex();  // 5
int totalSteps = autoBuilder.getTotalSteps();  // 12
boolean finished = autoBuilder.isFinished();  // false
```

Display in your loop:
```java
panelsTelemetry.debug("Current Step", currentStep);
panelsTelemetry.debug("Progress", (currentIndex + 1) + " / " + totalSteps);
panelsTelemetry.debug("Finished", finished);
```

## Advanced Features

### Multi-Stage Custom Actions

```java
.addCustomAction("Complex Sequence", (follower, time) -> {
    if (time < 0.5) {
        // Stage 1: Extend slides
        slideMotor.setPower(1.0);
    } else if (time < 1.0) {
        // Stage 2: Rotate wrist
        slideMotor.setPower(0.2);  // Hold
        wristServo.setPosition(WRIST_SCORE);
    } else if (time < 1.5) {
        // Stage 3: Release
        clawServo.setPosition(CLAW_OPEN);
    } else if (time < 2.0) {
        // Stage 4: Retract
        slideMotor.setPower(-0.8);
        wristServo.setPosition(WRIST_STOW);
    }
    return time > 2.2;  // Complete
})
```

### Sensor-Based Actions

```java
.addCustomAction("Wait for Sample", (follower, time) -> {
    // Check sensor state
    boolean sampleDetected = colorSensor.alpha() > 100;
    
    if (!sampleDetected && time < 3.0) {
        intakeMotor.setPower(1.0);  // Keep intaking
        return false;  // Not complete
    } else {
        intakeMotor.setPower(0.0);  // Stop
        return true;  // Complete when detected or timeout
    }
})
```

### Position-Based Actions

```java
.addCustomAction("Deploy at Target", (follower, time) -> {
    Pose currentPose = follower.getPose();
    double distanceToTarget = Math.hypot(
        currentPose.getX() - targetX,
        currentPose.getY() - targetY
    );
    
    // Deploy when close to target
    if (distanceToTarget < 5.0) {
        deployServo.setPosition(DEPLOY);
        return time > 0.5;
    }
    return false;
})
```

### Conditional Branching Example

```java
// Create alternate paths
PedroAutonomousBuilder leftPath = new PedroAutonomousBuilder(follower)
        .addPath(paths.LeftSample)
        .addPath(paths.LeftBasket);

PedroAutonomousBuilder rightPath = new PedroAutonomousBuilder(follower)
        .addPath(paths.RightSample)
        .addPath(paths.RightBasket);

// Use conditional
autoBuilder = new PedroAutonomousBuilder(follower)
        .addPath(paths.ToDetectionZone)
        .addConditional(
            (f) -> sampleIsLeft,  // Check condition
            leftPath,              // If true
            rightPath              // If false
        )
        .addPath(paths.ToPark);  // Continue after branch
```

## How It Works Internally

### State Machine

The builder creates an internal list of `AutonomousStep` objects:
- Each step has `onStart()`, `update()`, and `getName()` methods
- Steps execute sequentially
- Progress tracked automatically

### Step Types

1. **PathStep** - Wraps PathChain, waits for `!follower.isBusy()`
2. **ShootAction** - Manages shooter spinup and firing sequence
3. **TurnToHeadingAction** - Creates point turn path
4. **WaitAction** - Simple timer
5. **CustomAction** - Executes user lambda
6. **ConditionalStep** - Evaluates condition and executes sub-builder

### Update Cycle

```
Your loop() calls builder.update()
   ↓
Builder calls follower.update()  // Position tracking
   ↓
Builder calls currentStep.update(follower, elapsedTime)
   ↓
Step returns true when complete
   ↓
Builder advances to next step
```

## Migration from Manual State Machine

### Before (Manual):
```java
switch (pathState) {
    case STATE_FOLLOWING_PATH_1:
        if (!follower.isBusy()) {
            pathState = STATE_ACTION_1;
            actionTimer.reset();
        }
        break;
    case STATE_ACTION_1:
        // Action code...
        if (actionTimer.seconds() > 1.0) {
            pathState = STATE_FOLLOWING_PATH_2;
            follower.followPath(paths.Path2);
        }
        break;
    // ... many more cases
}
```

### After (Builder):
```java
autoBuilder = new PedroAutonomousBuilder(follower)
        .addPath(paths.Path1)
        .addCustomAction("Action 1", (f, t) -> t > 1.0)
        .addPath(paths.Path2);

// In loop:
autoBuilder.update();
```

## Best Practices

### 1. Always Update Follower First
```java
@Override
public void loop() {
    follower.update();  // MUST be first
    autoBuilder.update();
}
```

### 2. Keep Custom Actions Simple
Break complex sequences into multiple actions:
```java
.addCustomAction("Extend Slides", (f, t) -> {
    slideMotor.setPower(1.0);
    return t > 0.8;
})
.addCustomAction("Score", (f, t) -> {
    clawServo.setPosition(OPEN);
    return t > 0.3;
})
.addCustomAction("Retract Slides", (f, t) -> {
    slideMotor.setPower(-0.8);
    return t > 0.8;
})
```

### 3. Use Named Constants for Timing
```java
private static final double INTAKE_TIME = 1.0;
private static final double SCORE_TIME = 0.5;

.addCustomAction("Intake", (f, t) -> {
    intake.setPower(1.0);
    return t > INTAKE_TIME;
})
```

### 4. Add Wait Before Shooting
```java
.addPath(paths.ToScoringPosition)
.addTurnToHeading(Math.toRadians(45))
.addWait(0.2)  // Let robot settle
.addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE)
```

### 5. Check Shooter Configuration
```java
// In init()
if (shooter == null) {
    throw new IllegalStateException("Shooter not initialized!");
}
autoBuilder = new PedroAutonomousBuilder(follower)
        .withShooter(shooter);  // Must call this before addShootAction
```

## Debugging Tips

### Log Current Step
```java
String step = autoBuilder.update();
telemetry.addData("Step", step);  // Shows "Shooting (2/3)"
```

### Add Debug Actions
```java
.addCustomAction("Debug Checkpoint 1", (f, t) -> {
    Log.d("Auto", "Reached checkpoint 1 at " + f.getPose());
    return true;  // Complete immediately
})
```

### Test Individual Sequences
```java
// Test just the shooting sequence
autoBuilder = new PedroAutonomousBuilder(follower)
        .withShooter(shooter)
        .addTurnToHeading(Math.toRadians(45))
        .addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE);
```

## Comparison Table

| Feature | Manual State Machine | PedroAutonomousBuilder |
|---------|---------------------|------------------------|
| Code Lines | ~200-300 | ~30-50 |
| State Constants | Manual | Automatic |
| Position Tracking | Manual | Automatic |
| Shooter Integration | Manual | Built-in |
| Action Timing | Manual timers | Built-in |
| Readability | Complex | Simple |
| Maintenance | Difficult | Easy |
| Debugging | Harder | Easier |

## Files

- `PedroAutonomousBuilder.java` - The builder class
- `PedroBuilderAutonomous.java` - Example OpMode using the builder

## Status

✅ **COMPLETE** - Ready to use for autonomous routines!

The builder dramatically simplifies autonomous creation while providing powerful features like automatic position tracking, shooter integration, and flexible action sequencing.

