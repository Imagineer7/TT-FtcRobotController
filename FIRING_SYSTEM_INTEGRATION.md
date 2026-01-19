# Firing System Integration Guide

This document explains how to integrate the IndexingSystem firing logic with your OpMode and FiringSequenceCoordinator.

## Overview

The firing system consists of three coordinated components:

1. **IndexingSystem** - Executes the physical firing operation and manages artifacts
2. **FiringSequenceCoordinator** - Coordinates firing sequences and checks readiness
3. **OpMode** - Detects manual input and reports it to IndexingSystem

## Manual Input Detection (Required)

The firing system **requires** manual input detection to safely yield control when the driver manually operates indexing/uptake systems. Without this, automated firing could conflict with manual controls.

### Why Manual Input Detection?

- **Safety**: Prevents automated actions during manual servo control
- **User Control**: Allows driver to override automation at any time
- **State Consistency**: Avoids leaving the robot in undefined mechanical states

### Implementation Pattern

Your OpMode must call `indexingSystem.setManualInputActive()` every loop cycle to report the current manual input state.

```java
// In your OpMode's main loop or handleGamepadInputs() method:

// Detect if driver is manually controlling indexing/uptake systems
boolean manualIndexingControl = false;

if (manualServoMode) {
    // Check if operator is actively using servo controls
    manualIndexingControl = gamepad1.right_bumper ||      // Injector servos
                           gamepad1.left_trigger > 0.1 || // Uptake servos
                           gamepad1.right_trigger > 0.1;  // Transfer servos
}

// Report manual input state to IndexingSystem
indexingSystem.setManualInputActive(manualIndexingControl);
```

### What Counts as Manual Input?

**DO detect as manual input:**
- Active servo control (triggers/bumpers pressed)
- Continuous manual operations (holding buttons)
- Direct hardware overrides

**DO NOT detect as manual input:**
- Discrete commands (button presses for fire, collect)
- Robot driving (mecanum drive controls)
- Camera/vision system controls
- Configuration changes

The key distinction: **continuous override** vs **discrete command**.

### Example: IndexingSystemTest.java

See `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/IndexingSystemTest.java` for a complete implementation example around line 574.

## Firing Gating Rules

The `onFireSignal()` method enforces these hard requirements:

1. **firingSequenceActive == true** (coordinator controls this)
2. **Manual input NOT active** (detected by OpMode)
3. **Center artifact exists**
4. **Center artifact pre-positioned** (uptake servos ready)
5. **Indexing system NOT busy** (no operation in progress)
6. **Shooter ready** (at target RPM and stabilized)

If ANY condition fails, firing returns `false` immediately.

## FiringSequenceCoordinator Integration

The coordinator should call `onFireSignal()` when ready to fire:

```java
// In FiringSequenceCoordinator.update():
if (firingSequenceActive) {
    // Check readiness
    if (indexingSystem.isReadyToFire() && !indexingSystem.isOperationInProgress()) {
        boolean fired = indexingSystem.onFireSignal();
        if (fired) {
            currentShotNumber++;
            // Firing in progress - will complete automatically
        }
    }
}
```

The coordinator does NOT need to:
- Check manual input (IndexingSystem handles this)
- Pre-position artifacts (IndexingSystem handles this)
- Manage artifact advancement (IndexingSystem handles this)

## Artifact Consumption

After a successful fire:

1. Artifact marked as `Location.FIRED` in artifacts list
2. Shot plan automatically filters out FIRED artifacts
3. Center slot cleared (`artifactInCenter = null`)
4. Artifact count decremented (via `getArtifactCount()`)

## Post-Fire Advancement

After firing completes, the system automatically:

1. **Checks if advancement should occur:**
   - Artifacts remain in storage?
   - Manual input NOT active?
   
2. **If yes, advances next artifact:**
   - Uses shot plan to determine next artifact
   - Transfers from storage to center
   - Pre-positions for next shot
   
3. **If no, goes to IDLE:**
   - Waits for next operation
   - Preserves system state

## Mid-Transfer Cancellation

If manual input becomes active during artifact transfer:

- **Transfer completes** (already started, must finish safely)
- Artifact location updated to CENTER
- Artifact pre-positioned
- **Firing does NOT occur** (onFireSignal checks manual input)
- System remains in READY_TO_FIRE state

This ensures the robot never leaves artifacts in undefined positions.

## Timing Configuration

All firing timing uses `ShooterConfig` constants:

```java
ShooterConfig.UPTAKE_FEED_TIME_MS        // 800ms - Time to feed artifact
ShooterConfig.UPTAKE_FEED_POWER          // 0.8 - Feed servo power
ShooterConfig.UPTAKE_PREPOSITION_TIMEOUT_MS  // 500ms - Pre-position time
```

**Never use hardcoded timing values** in your OpMode or coordinator.

## State Machine Flow

```
READY_TO_FIRE
    ↓ [onFireSignal() called, all gates pass]
FIRING (uptake servos running for UPTAKE_FEED_TIME_MS)
    ↓ [firing complete]
completeFiring()
    ↓ [consume artifact, check advancement]
    ├─ [no more artifacts OR manual input active]
    │   → IDLE
    └─ [artifacts remain AND no manual input]
        → TRANSFERRING (advancing next artifact)
            → READY_TO_FIRE (ready for next shot)
```

## Testing Checklist

Before deploying:

- [ ] Compile successfully
- [ ] Manual input detection implemented in OpMode
- [ ] Test fire with coordinator enabled
- [ ] Test manual servo override during firing sequence
- [ ] Test artifact advancement after firing
- [ ] Test mid-transfer manual cancellation
- [ ] Verify no hardcoded timing values
- [ ] Check all gating rules enforce correctly

## Troubleshooting

### Firing returns false immediately

Check these in order:
1. Is `firingSequenceActive` true? (coordinator's responsibility)
2. Is manual input active? (check OpMode implementation)
3. Is artifact in center? (check `artifactInCenter != null`)
4. Is artifact pre-positioned? (check `uptakeServoPrePositionedForCurrentArtifact`)
5. Is system busy? (check `operationInProgress`)
6. Is shooter ready? (check `shooter.isReadyToFire()`)

### Artifacts don't advance after firing

- Check manual input is not incorrectly reported as active
- Verify shot plan is not empty after consumption
- Check storage references are correct

### Manual override doesn't work

- Ensure `setManualInputActive()` is called every loop
- Check the boolean detection logic in your OpMode
- Verify manual input detection doesn't include discrete commands

## See Also

- `IndexingSystem.java` - Core firing implementation
- `FiringSequenceCoordinator.java` - Sequence coordination
- `IndexingSystemTest.java` - Example OpMode with manual input detection
- `ShooterConfig.java` - Timing constants
