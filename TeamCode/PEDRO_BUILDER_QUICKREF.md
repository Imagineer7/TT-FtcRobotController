# Pedro Autonomous Builder - Quick Reference

## Setup (In OpMode)

```java
// In init()
PedroAutonomousBuilder autoBuilder = new PedroAutonomousBuilder(follower)
        .withShooter(shooter);

// In start()
autoBuilder.start();

// In loop()
follower.update();  // IMPORTANT: Must be first!
String currentStep = autoBuilder.update();
```

## Actions Quick Reference

| Action | Code | Description |
|--------|------|-------------|
| **Path** | `.addPath(paths.Path1)` | Follow a path |
| **Shoot** | `.addShootAction(3, LONG_RANGE)` | Fire shots |
| **Turn** | `.addTurnToHeading(Math.toRadians(90))` | Turn to heading |
| **Wait** | `.addWait(1.5)` | Pause for seconds |
| **Custom** | `.addCustomAction("Name", (f, t) -> {...})` | Custom code |

## Examples

### Simple Path Sequence
```java
autoBuilder = new PedroAutonomousBuilder(follower)
        .addPath(paths.Path1)
        .addPath(paths.Path2)
        .addPath(paths.Path3);
```

### Path → Shoot → Path
```java
autoBuilder = new PedroAutonomousBuilder(follower)
        .withShooter(shooter)
        .addPath(paths.ToScoringPosition)
        .addTurnToHeading(Math.toRadians(45))
        .addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE)
        .addPath(paths.ToPark);
```

### Custom Action Template
```java
.addCustomAction("Action Name", (follower, elapsedTime) -> {
    // Your code here
    if (elapsedTime < 0.5) {
        // Stage 1
        myServo.setPosition(0.8);
    } else {
        // Stage 2
        myMotor.setPower(0.5);
    }
    return elapsedTime > 1.0;  // Return true when complete
})
```

### Multi-Stage Action
```java
.addCustomAction("Deploy Mechanism", (f, t) -> {
    if (t < 0.3) {
        slideMotor.setPower(1.0);  // Extend
    } else if (t < 0.8) {
        slideMotor.setPower(0.1);  // Hold
        wristServo.setPosition(SCORE);
    } else if (t < 1.2) {
        clawServo.setPosition(OPEN);  // Release
    } else {
        slideMotor.setPower(-0.8);  // Retract
    }
    return t > 1.5;
})
```

## Telemetry

```java
String step = autoBuilder.update();
int current = autoBuilder.getCurrentStepIndex();
int total = autoBuilder.getTotalSteps();
boolean done = autoBuilder.isFinished();

telemetry.addData("Step", step);  // "Shooting (2/3)"
telemetry.addData("Progress", (current + 1) + " / " + total);
telemetry.addData("Done", done);
```

## Common Patterns

### Intake → Drive → Score
```java
.addPath(paths.ToSample)
.addCustomAction("Intake", (f, t) -> {
    intakeMotor.setPower(1.0);
    return t > 1.0;
})
.addPath(paths.ToBasket)
.addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)
```

### Wait for Settle Before Action
```java
.addPath(paths.ToPosition)
.addWait(0.2)  // Let robot settle
.addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE)
```

### Repeated Cycles
```java
// Cycle 1
.addPath(paths.ToSample1)
.addCustomAction("Grab", grabAction)
.addPath(paths.ToBasket)
.addShootAction(1, PRESET)

// Cycle 2
.addPath(paths.ToSample2)
.addCustomAction("Grab", grabAction)
.addPath(paths.ToBasket)
.addShootAction(1, PRESET)

// Cycle 3
.addPath(paths.ToSample3)
.addCustomAction("Grab", grabAction)
.addPath(paths.ToBasket)
.addShootAction(1, PRESET)
```

## Key Benefits

✅ **No manual state machine** - Automatic state management
✅ **Position always tracked** - Even during actions
✅ **Shooter integration** - Built-in shooting support
✅ **Simple syntax** - Clean, readable code
✅ **Easy debugging** - Clear step names in telemetry
✅ **Flexible** - Custom actions for anything

## Important Notes

⚠️ **Always call `follower.update()` before `autoBuilder.update()`**
⚠️ **Call `.withShooter()` before using `.addShootAction()`**
⚠️ **Custom actions must return `true` when complete**
⚠️ **Heading is in radians** - Use `Math.toRadians(degrees)`

## Full Competition Example

```java
private void buildAutonomousSequence() {
    autoBuilder = new PedroAutonomousBuilder(follower)
            .withShooter(shooter)
            
            // Preload
            .addPath(paths.ToSubmersible)
            .addCustomAction("Score Specimen", scoreAction)
            
            // Sample 1
            .addPath(paths.ToSample1)
            .addWait(0.3)
            .addCustomAction("Grab Sample", grabAction)
            .addPath(paths.ToBasket)
            .addTurnToHeading(Math.toRadians(45))
            .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)
            
            // Sample 2
            .addPath(paths.ToSample2)
            .addWait(0.3)
            .addCustomAction("Grab Sample", grabAction)
            .addPath(paths.ToBasket)
            .addTurnToHeading(Math.toRadians(45))
            .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)
            
            // Sample 3
            .addPath(paths.ToSample3)
            .addWait(0.3)
            .addCustomAction("Grab Sample", grabAction)
            .addPath(paths.ToBasket)
            .addTurnToHeading(Math.toRadians(45))
            .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)
            
            // Park
            .addPath(paths.ToPark)
            .addCustomAction("Stow", stowAction);
}
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Position drifts during actions | Ensure `follower.update()` is called first in loop |
| Shooter not firing | Call `.withShooter(shooter)` before `.addShootAction()` |
| Action never completes | Check your return statement returns `true` |
| Robot doesn't move | Verify path is being followed: check `follower.isBusy()` |

---

**For full documentation, see `PEDRO_AUTONOMOUS_BUILDER_GUIDE.md`**

