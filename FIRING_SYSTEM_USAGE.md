# Firing System Usage Guide

## Overview

The IndexingSystem firing implementation provides robust, safe artifact firing with the following capabilities:

- **Automatic firing** controlled by FiringSequenceCoordinator
- **Manual override detection** via pluggable detector functions
- **Intelligent gating rules** that enforce safety conditions
- **Post-fire advancement** to prepare next artifacts
- **Mid-operation cancellation** with safe state preservation
- **Shot plan consumption** for accurate tracking

## Quick Start

### Basic Setup in OpMode

```java
// 1. Create components
IndexingSystem indexingSystem = new IndexingSystem(hardware, config, shooter, telemetry);
FiringSequenceCoordinator firingCoordinator = new FiringSequenceCoordinator(indexingSystem, shooter);

// 2. Configure manual input detection (optional but recommended)
indexingSystem.setManualInputDetector(() -> {
    // Return true if any relevant manual controls are active
    // Only detect index/uptake controls, NOT robot movement
    return gamepad2.dpad_up || gamepad2.dpad_down;  // Uptake servo manual control
});

// 3. Enable systems
indexingSystem.enable();
shooter.enable();

// 4. In your loop()
indexingSystem.update();           // Update indexing system
shooter.update();                  // Update shooter
firingCoordinator.update();        // Coordinate firing

// 5. Start firing when ready
if (gamepad1.x && firingCoordinator.canStartFiring()) {
    firingCoordinator.startFiring();
}

// 6. Emergency stop if needed
if (gamepad1.y) {
    firingCoordinator.cancelFiring();
}
```

## Firing Gating Rules

The firing system enforces **7 critical gating rules** before allowing a fire operation:

| Rule | Condition | Purpose |
|------|-----------|---------|
| 1 | `firingSequenceActive == true` | Coordinator must explicitly enable firing |
| 2 | No manual input active | Manual controls override automation |
| 3 | Artifact in center | Must have something to fire |
| 4 | Artifact pre-positioned | Uptake servos must be ready |
| 5 | Indexing system not busy | No conflicting operations |
| 6 | Shooter ready (RPM + stable) | Shooter at target speed |
| 7 | PlannerExecutor not busy | No rearrangements in progress |

All rules must pass for `onFireSignal()` to return `true`.

## Manual Input Detection

### Default Behavior
By default, no manual input detection is active. The system operates in fully automated mode.

### Configuring Detection

**Important:** Only detect controls that **directly affect the indexing/firing system**. Do NOT detect robot movement controls.

```java
// Example: Detect uptake servo and index controls only
indexingSystem.setManualInputDetector(() -> {
    // Gamepad 2: Shooter/Uptake controls
    boolean uptakeManual = gamepad2.dpad_up || gamepad2.dpad_down;
    
    // Gamepad 1: Manual push/index controls (if applicable)
    boolean indexManual = gamepad1.left_bumper || gamepad1.right_bumper;
    
    return uptakeManual || indexManual;
});
```

**Bad Example (detects too much):**
```java
// DON'T DO THIS - detects robot movement
indexingSystem.setManualInputDetector(() -> {
    return gamepad1.a || gamepad1.b || 
           Math.abs(gamepad1.left_stick_x) > 0.1 ||  // ❌ Robot movement
           Math.abs(gamepad1.left_stick_y) > 0.1;     // ❌ Robot movement
});
```

### When Manual Input is Detected

When manual input becomes active:

- **During FIRING**: Operation is cancelled immediately, artifact preserved in center
- **During TRANSFERRING**: Transfer completes safely, then stops
- **Before firing**: `onFireSignal()` returns false, preventing new fire operations

## Firing Sequence Flow

```
1. FiringSequenceCoordinator.startFiring()
   └─> Sets firingSequenceActive = true
   └─> Notifies IndexingSystem
   └─> Spins up shooter

2. FiringSequenceCoordinator.update() (loop)
   └─> Checks shooter.isReadyToFire()
   └─> Checks indexingSystem.isReadyToFire()
   └─> Calls indexingSystem.onFireSignal()

3. IndexingSystem.onFireSignal()
   └─> Validates all 7 gating rules
   └─> Starts uptake servos (800ms feed time)
   └─> Returns true if firing started

4. IndexingSystem.updateFiring() (loop)
   └─> Monitors feed time completion
   └─> Stops uptake servos after 800ms
   └─> Consumes fired artifact

5. IndexingSystem.attemptPostFireAdvancement()
   └─> Gets next artifact from shot plan
   └─> Transfers next artifact to center
   └─> Pre-positions for next shot
   └─> OR: Stops if manual input detected
   └─> OR: Completes if no more artifacts

6. FiringSequenceCoordinator.update() (loop)
   └─> Repeats from step 2 until done
```

## Artifact Consumption

After a successful fire:

```java
// Artifact state changes
artifactInCenter.location = FIRED;  // Marked as fired
artifactInCenter = null;             // Center slot cleared

// Shot plan updated
shotPlanner.updateShotPlan();        // Re-evaluated
```

**Important:** Artifacts are only consumed on successful completion. Cancellations preserve artifact state.

## Post-Fire Advancement

The system automatically advances the next artifact to center after firing:

```java
// Automatic advancement conditions:
✅ More artifacts remain
✅ No manual input active
✅ Shot plan not empty
✅ Next artifact in intake storage

// When advancement occurs:
1. Get next artifact from shot plan
2. Start transfer to center (uses existing transfer logic)
3. Pre-position uptake servos
4. Ready for next fire signal
```

## Cancellation & Safety

### Mid-Fire Cancellation

```java
// If firingSequenceActive becomes false during firing:
firingCoordinator.cancelFiring();
// Result:
- Uptake servos stop immediately
- Artifact remains in center (NOT consumed)
- System returns to READY_TO_FIRE state
- Shot plan unchanged
```

### Mid-Transfer Cancellation

```java
// If cancelled during post-fire transfer:
- Transfer completes safely (prevents undefined state)
- Artifact reaches center as intended
- System then stops advancement
- Shot plan updated for completed transfer
```

### Manual Override During Firing

```java
// If manual input detected during firing:
- Same behavior as cancellation
- Manual controls take priority
- System enters safe state
- Ready for manual operation
```

## Integration with PlannerExecutor

The firing system **respects planner executor operations**:

```java
// Before firing, checks:
if (plannerExecutor.isBusy()) {
    return false;  // Don't fire during rearrangement
}

// Planner executor operations:
- Push operations
- Artifact rearrangements  
- Intake mode updates

// Are NOT interrupted by firing system
```

## Timing Configuration

All timing uses `ShooterConfig` constants:

| Constant | Default | Purpose |
|----------|---------|---------|
| `UPTAKE_FEED_TIME_MS` | 800ms | Artifact feed duration |
| `UPTAKE_PREPOSITION_TIMEOUT_MS` | 500ms | Pre-position duration |
| `UPTAKE_RETRACT_TIME_MS` | 500ms | Retraction before push |
| `UPTAKE_FEED_POWER` | 0.8 | Feed servo power |
| `UPTAKE_PREPOSITION_POWER` | 0.5 | Pre-position power |

## Telemetry & Debugging

### Debug Output

Enable debug telemetry to see firing system status:

```java
config.setDebugTelemetry(true);

// Output includes:
🔫 Fire signal received
   Gating rule checks
   ✅ All rules passed
   🔄 FIRING STARTED
   ⏱️ FIRING: 0.3s / 0.8s
   ✅ FIRING COMPLETE
   📦 Consumed artifact #2
   🔄 POST-FIRE TRANSFER
```

### Common Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| `onFireSignal()` returns false | Gating rule failed | Check telemetry for which rule |
| Fires but nothing happens | Uptake servos not connected | Check hardware config |
| Cancels immediately | Manual input always true | Review detector function |
| Doesn't advance next | Shot plan empty | Verify planner is running |
| Pre-positioning fails | Artifact not in center | Check artifact location state |

## Advanced Usage

### Custom Shot Sequencing

```java
// Let shot planner determine order
shotPlanner.setMotifPattern("PPG");  // Purple, Purple, Green

// Planner will:
1. Evaluate artifact colors
2. Score against motif pattern
3. Request rearrangement if needed
4. Update shot plan each cycle

// Firing system consumes in plan order
```

### Multi-Artifact Scenarios

```java
// Scenario: 3 artifacts collected
// - Artifact #1 (Green) in center
// - Artifact #2 (Purple) in front intake
// - Artifact #3 (Purple) in back intake

// Shot plan (PPG pattern): [#2, #3, #1]
// System will:
1. Request rearrangement (#2 to center)
2. Fire #2
3. Advance #3 to center
4. Fire #3
5. Advance #1 to center
6. Fire #1
```

### Error Recovery

```java
// If firing times out or errors:
indexingSystem.getCurrentState() == SystemState.ERROR

// Auto-recovery (if enabled):
config.setEnableAutoRecovery(true);
// System automatically resets to IDLE after 1 second

// Manual recovery:
indexingSystem.reset();  // Full system reset
firingCoordinator.reset();  // Coordinator reset
```

## API Reference

### IndexingSystem Firing Methods

```java
// Core firing
boolean onFireSignal()                          // Start firing (called by coordinator)
void setFiringSequenceActive(boolean active)    // Set firing sequence state
boolean isFiringSequenceActive()                // Check firing sequence state

// Manual input detection
void setManualInputDetector(Supplier<Boolean>)  // Configure detector
void clearManualInputDetector()                 // Remove detector

// Status checks
boolean isReadyToFire()                         // Check if ready
boolean isOperationInProgress()                 // Check if busy
SystemState getCurrentState()                   // Get current state
```

### FiringSequenceCoordinator Methods

```java
// Control
boolean canStartFiring()     // Check if all conditions met
boolean startFiring()        // Start firing sequence
void completeFiring()        // Complete sequence
void cancelFiring()          // Cancel sequence
void emergencyStop()         // Emergency stop all systems

// Status
boolean isFiringActive()     // Check if firing
int getCurrentShotNumber()   // Get shot count
long getElapsedTime()        // Get elapsed time

// Configuration
void setFiringTimeout(long)  // Set timeout (default 30s)
```

## Best Practices

1. **Always configure manual input detection** for teleop modes
2. **Use FiringSequenceCoordinator** instead of calling `onFireSignal()` directly
3. **Check `canStartFiring()`** before starting sequences
4. **Monitor telemetry** during development for gating rule feedback
5. **Test cancellation scenarios** to verify safe state preservation
6. **Respect PlannerExecutor** - don't fire during rearrangements
7. **Use shot planner** for intelligent artifact ordering

## Example: Complete Teleop Integration

```java
@TeleOp(name="Full System Test")
public class FullSystemTest extends LinearOpMode {
    
    private IndexingSystem indexingSystem;
    private Shooter shooter;
    private FiringSequenceCoordinator firingCoordinator;
    
    @Override
    public void runOpMode() {
        // Initialize
        indexingSystem = new IndexingSystem(hardware, config, shooter, telemetry);
        firingCoordinator = new FiringSequenceCoordinator(indexingSystem, shooter);
        
        // Configure manual input detection
        indexingSystem.setManualInputDetector(() -> {
            // Gamepad 2: Uptake manual control
            return gamepad2.dpad_up || gamepad2.dpad_down;
        });
        
        // Enable systems
        indexingSystem.enable();
        shooter.enable();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update all systems
            indexingSystem.update();
            shooter.update();
            firingCoordinator.update();
            
            // Gamepad 1 controls
            if (gamepad1.x && firingCoordinator.canStartFiring()) {
                firingCoordinator.startFiring();  // Start firing
            }
            
            if (gamepad1.y) {
                firingCoordinator.cancelFiring();  // Emergency stop
            }
            
            // Gamepad 2: Shooter presets and manual override
            if (gamepad2.right_bumper) {
                shooter.spinUp();  // Manual shooter spin-up
            }
            
            // Manual uptake control (overrides automation)
            if (gamepad2.dpad_up) {
                // Manual input detected - automation yields
            }
            
            telemetry.update();
        }
    }
}
```

## Testing Checklist

- [ ] Fire single artifact successfully
- [ ] Fire all 3 artifacts in sequence
- [ ] Cancel firing mid-operation
- [ ] Manual override during firing
- [ ] Manual override during transfer
- [ ] Post-fire advancement works
- [ ] Empty shot plan handled
- [ ] No artifacts in center handled
- [ ] Shooter not ready handled
- [ ] PlannerExecutor busy handled
- [ ] Timeout protection works
- [ ] State transitions are clean
- [ ] Artifact consumption is accurate
- [ ] Shot plan updates correctly
