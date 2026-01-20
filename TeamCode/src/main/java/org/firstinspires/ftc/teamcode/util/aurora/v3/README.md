# Indexing System V3 - Design Documentation

## Overview

The V3 Indexing System is a complete rewrite of the indexing subsystem, implementing a slot-based, transaction-oriented architecture that treats sensors as hints rather than truth.

## Design Principles

### 1. Slot Ledger as Single Source of Truth

The system maintains exactly 3 slots:
- **CENTER** - Ready to fire position
- **FRONT** - Front intake storage
- **BACK** - Back intake storage

Each slot holds an `ArtifactIdentity` or null (empty).

**Key Rule:** Slots are updated ONLY by operation commit, never by sensors.

### 2. Operations as Transactions

Every physical action is an operation with a lifecycle:

```
start() → update()* → commit() → complete
                  ↓
              cancel() → abort
```

Operations:
- Start hardware via BasicIndexingHelper/BasicFiringHelper
- Track state and check completion
- Commit slot changes atomically when done
- Never leave system in half-completed state

### 3. Sensors as Hints, Not Truth

Sensors provide:
- **Entry detection** - triggers "first detect" event
- **Confirmation window** - validate presence after delay
- **Diagnostics** - jam detection, telemetry

Sensors do NOT:
- Remove artifacts from tracking
- Override committed slot state
- Drive state transitions after commit

### 4. Model-First Tracking

Once an artifact is committed to a slot:
- It stays until explicitly removed (fire/eject/clear)
- Sensor disagreement logged but ignored
- Only operator actions remove artifacts

## Core Components

### ArtifactIdentity

Represents artifact metadata:

```java
public class ArtifactIdentity {
    ColorClass colorClass;      // GREEN, PURPLE, UNKNOWN
    double colorConfidence;     // 0.0 to 1.0
    ClassificationSource source; // COLOR_SENSOR, INFERRED, OPERATOR
    int sequenceId;             // Unique identifier
    long timestamp;             // Collection time
}
```

**Immutability:** All updates create new instances.

**Color Updates:** Only allowed if confidence significantly stronger (0.15+ improvement) or OPERATOR override.

### SlotLedger

Single source of truth for slot state:

```java
public class SlotLedger {
    ArtifactIdentity centerSlot;
    ArtifactIdentity frontSlot;
    ArtifactIdentity backSlot;
    
    // Queries
    int getArtifactCount();
    boolean isFull();
    Slot getFirstEmptySlot();
    Slot getOccupiedIntakeSlot();
    
    // Mutations (operation-only)
    void set(Slot slot, ArtifactIdentity artifact);
    void swap(Slot slot1, Slot slot2);
    void clearAll();
}
```

**Access Pattern:** Operations read slots freely, but only modify at commit time.

### IndexingOperation Interface

All operations implement this interface:

```java
public interface IndexingOperation {
    boolean start();           // Start hardware
    boolean update();          // Check completion (called every loop)
    void commit();             // Apply slot changes
    boolean cancel();          // Abort if cancellable
    boolean isComplete();      // Check if done
    boolean isSuccess();       // Check if succeeded
    String getOperationName(); // For logging
    String getStatusMessage(); // For telemetry
}
```

### RejectReason Enum

Structured rejection reasons:

```java
public enum RejectReason {
    SYSTEM_FULL,
    SYSTEM_BUSY,
    SLOT_OCCUPIED,
    SLOT_EMPTY,
    HARDWARE_BUSY,
    SHOOTER_NOT_READY,
    NOT_PREPOSITIONED,
    MANUAL_INPUT_DETECTED,
    // ... etc
}
```

## Planned Operations

### Collection Operations

**CollectFromFront / CollectFromBack**
- Preconditions: Target slot empty, system not full
- Hardware: Run intake rollers, bottom servos, transfer servos
- Commit: Place new artifact in target slot with initial color classification

### Transfer Operations

**TransferToCenter**
- Preconditions: Source slot occupied, center empty (or post-fire)
- Hardware: BasicIndexingHelper.transferXIntakeToCenterTimed()
- Commit: Move artifact from source slot to center, clear source

### Swap Operations

**SwapCenterWithFront / SwapCenterWithBack**
- Preconditions: Both slots occupied, exactly 2 artifacts total
- Hardware: Simultaneous transfer and accept operations (push-style)
- Commit: Swap artifact identities between slots atomically

### Firing Operations

**FireCenter**
- Preconditions: Center occupied, pre-positioned, shooter ready, 7 gating rules pass
- Hardware: BasicFiringHelper.startFiring()
- Commit: Clear center slot, consume artifact

**PrepositionCenter**
- Preconditions: Center occupied, not already prepositioned
- Hardware: BasicIndexingHelper.prePositionArtifacts()
- Commit: Set preposition flag (tracked separately from slots)

### Utility Operations

**EjectAll / EjectSlot**
- Preconditions: None (emergency operation)
- Hardware: Reverse all intakes, run shooter at low RPM
- Commit: Clear all slots (EjectAll) or specific slot (EjectSlot)

**Clear**
- Preconditions: Operator confirmation
- Hardware: None
- Commit: Clear all slots (software-only reset)

## Perception Layer (Planned)

### IntakePerception Class

Per-intake sensor fusion:

```java
public class IntakePerception {
    // Derived signals
    boolean frontBlocked;         // Laser distance < 10cm
    boolean mouthOccupied;        // REV 2m delta from baseline with hysteresis
    boolean colorSeesArtifact;    // Color confidence high for GREEN/PURPLE
    boolean artifactHint;         // Any signal indicates presence
    
    // Confidence
    PresenceConfidence confidence; // LOW, MEDIUM, HIGH
    
    // Color classification
    ColorObservation getBestColorObs(); // Fuse mouth + outward sensors
}
```

**Hysteresis Example:**
- REV 2m baseline: 25cm
- Enter threshold: 18cm (artifact entering)
- Exit threshold: 22cm (artifact leaving)
- Prevents oscillation

**Debounce:**
- Signal must be stable for 100ms before triggering

### Color Checkpoints

Classify color only at stable points:
1. During intake confirm window (after 100ms delay)
2. Immediately after transfer completes (settled)

Never use "live" color while artifact is moving.

## Shot Planning Integration

### Existing ShotPlanner

Keep the existing ShotPlanner.java (already correct):
- Runs every loop
- Generates shot plan from slot state
- Outputs desiredCenterArtifact if rearrangement needed

### V3 Integration

```java
// In IndexingSystemV3.update()
shotPlanner.updateShotPlan(ledger.getCenter(), ledger.getFront(), ledger.getBack());

if (shotPlanner.getDesiredCenterArtifact() != null) {
    // Check if rearrangement operation is allowed
    if (canExecuteRearrangement()) {
        SwapOperation swap = new SwapOperation(...);
        operationRunner.start(swap);
    }
}
```

### Shot Plan Consumption

During firing:
```java
// FireCenter operation
void commit() {
    ArtifactIdentity firedArtifact = ledger.getCenter();
    ledger.clearCenter();
    
    // Advance next artifact if available
    List<Artifact> shotPlan = shotPlanner.getShotPlan();
    if (shotPlan.size() > 1) {
        ArtifactIdentity nextDesired = shotPlan.get(1);  // Now first
        Slot sourceSlot = ledger.findSlot(nextDesired);
        if (sourceSlot != null) {
            queueTransferOperation(sourceSlot, Slot.CENTER);
        }
    }
}
```

## Operation Runner

Enforces single operation at a time:

```java
public class OperationRunner {
    IndexingOperation currentOperation;
    
    boolean start(IndexingOperation op) {
        if (currentOperation != null) {
            return false;  // Busy
        }
        
        if (op.start()) {
            currentOperation = op;
            return true;
        }
        return false;
    }
    
    void update() {
        if (currentOperation == null) return;
        
        if (!currentOperation.update()) {
            // Operation complete
            if (currentOperation.isSuccess()) {
                currentOperation.commit();
            }
            currentOperation = null;
        }
    }
    
    boolean isBusy() {
        return currentOperation != null;
    }
}
```

## Gating Rules

Before firing (7 rules from spec):

1. ✅ `firingSequenceActive == true` (coordinator enabled)
2. ✅ `!isManualInputActive()` (no manual override)
3. ✅ `ledger.isCenterOccupied()` (artifact present)
4. ✅ `isPrepositioned` (artifact positioned for firing)
5. ✅ `!operationRunner.isBusy()` (not busy)
6. ✅ `shooter.isReadyToFire()` (RPM stable)
7. ✅ `!plannerExecutor.isBusy()` (not rearranging)

All must pass. Failure → RejectReason logged.

## Telemetry Structure

### Slot State
```
CENTER: ArtifactIdentity{seq=1, color=PURPLE, conf=0.85, source=COLOR_SENSOR}
FRONT:  ArtifactIdentity{seq=2, color=GREEN, conf=0.90, source=COLOR_SENSOR}
BACK:   EMPTY
```

### Operation Status
```
Operation: TransferToCenter
Status: TRANSFERRING (elapsed 1200ms / 2500ms timeout)
Source: FRONT → CENTER
```

### Perception Diagnostics (per intake)
```
Front Intake:
  frontBlocked: true (8cm)
  mouthOccupied: true (16cm, baseline 25cm)
  colorSeesArtifact: true (PURPLE, conf=0.82)
  artifactHint: true
  presenceConfidence: HIGH
```

### Reject Reasons
```
❌ FireCenter REJECTED: NOT_PREPOSITIONED
   Artifact in center but not yet positioned for firing
```

## Testing Strategy

### Unit Tests

**SlotLedger:**
- Test slot get/set/clear
- Test swap atomicity
- Test derived queries (count, empty slots, etc.)

**ArtifactIdentity:**
- Test immutability
- Test color update rules (confidence threshold)
- Test factory methods

**Operations:**
- Mock BasicIndexingHelper/BasicFiringHelper
- Test precondition checking
- Test commit behavior
- Test timeout detection

### Integration Tests

**Full Workflow:**
1. Collect artifact from front → verify slot state
2. Transfer to center → verify slot state
3. Fire → verify center cleared
4. Post-fire advancement → verify next artifact moved

**Rearrangement:**
1. Collect 2 artifacts (green center, purple front)
2. Set motif PPG
3. Verify swap requested
4. Execute swap → verify slots swapped

### Hardware Tests

**OpMode: IndexingSystemV3Test**
- Manual trigger operations via gamepad
- Display slot state + operation status on telemetry
- Test with real artifacts and sensors

## Implementation Status

### ✅ Completed (Phase 1)
- [x] ArtifactIdentity data class
- [x] SlotLedger class
- [x] IndexingOperation interface
- [x] RejectReason enum

### 🚧 In Progress (Phase 2-7)
- [ ] IntakePerception (sensor fusion)
- [ ] BaseOperation abstract class
- [ ] Specific operations (Collect, Transfer, Swap, Fire, etc.)
- [ ] OperationRunner
- [ ] IndexingSystemV3 main controller
- [ ] Integration with ShotPlanner
- [ ] Testing and validation

## Migration Path

The V3 system will coexist with the old system initially:

1. Implement V3 in separate package (util.aurora.v3)
2. Create IndexingSystemV3 with same public API as IndexingSystemOld
3. Test V3 in dedicated OpModes
4. Once validated, switch production OpModes to V3
5. Deprecate IndexingSystemOld

## Key Differences from Old System

| Aspect | Old System | V3 System |
|--------|-----------|-----------|
| State model | Artifact list + location refs | Slot ledger only |
| Sensor role | Primary tracking | Hints + diagnostics |
| Operations | State machine transitions | Explicit operations |
| Commit timing | During operation | At completion |
| Artifact removal | Sensor-driven (bugs) | Explicit only |
| Color updates | Anytime | Only at checkpoints |
| Concurrency | Multiple state flags | Single operation runner |

## References

- [INDEXING_SYSTEM_SPECIFICATION.md](../../INDEXING_SYSTEM_SPECIFICATION.md)
- [Indexing Rewrite Notes (Sensors + Tracking Assumptions).md](../Indexing%20Rewrite%20Notes%20(Sensors%20+%20Tracking%20Assumptions).md)
- BasicIndexingHelper.java (HAL for hardware)
- BasicFiringHelper.java (HAL for firing)
- ShotPlanner.java (existing planner logic)
