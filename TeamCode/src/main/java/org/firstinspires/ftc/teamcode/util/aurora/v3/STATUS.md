# Indexing System V3 - Implementation Status

## 🎯 Project Goals

Build a reliable, testable indexing system that:
- Uses slot-based tracking (CENTER/FRONT/BACK) as single source of truth
- Treats sensors as hints (never auto-removes committed artifacts)
- Implements operations as atomic transactions
- Integrates with existing BasicIndexingHelper and BasicFiringHelper
- Works with existing ShotPlanner for shot optimization

## ✅ Completed (Phases 1-2)

### Phase 1: Core Data Structures

**Files Created:**
- ✅ `ArtifactIdentity.java` - Immutable artifact metadata (color, confidence, source, sequence ID)
- ✅ `SlotLedger.java` - Single source of truth for 3 slots with atomic operations
- ✅ `IndexingOperation.java` - Operation interface defining lifecycle
- ✅ `RejectReason.java` - 30+ structured rejection reasons
- ✅ `README.md` - Complete architecture documentation

**Key Features:**
- Immutable artifact identity with smart color update rules
- Slot-based model (no separate artifact lists)
- Clear operation contract (start/update/commit/cancel)
- Telemetry-friendly error messages

### Phase 2: Perception Layer

**Files Created:**
- ✅ `IntakePerception.java` - Multi-sensor fusion per intake

**Key Features:**
- Fuses 4 sensors per intake (laser distance, REV 2m ToF, 2x color sensors)
- Hysteresis on REV sensor (prevents oscillation)
- Debounce on all signals (100ms stability requirement)
- Presence confidence levels (NONE/LOW/MEDIUM/HIGH)
- Auto-calibrating REV sensor baseline
- Rich structured telemetry

**Derived Signals:**
- `frontBlocked` - outward laser distance sensor
- `mouthOccupied` - REV 2m with hysteresis
- `colorSeesArtifact` - dual color sensor confidence
- `artifactHint` - combined presence (any sensor active)
- `bestColorClass` - fused color classification

## 🚧 Remaining Work (Phases 3-7)

### Phase 3: Operation Framework (Next Priority)

**Need to Create:**
- `BaseOperation.java` - Abstract base class with common lifecycle logic
- `OperationRunner.java` - Enforces single operation at a time, manages lifecycle

**Key Responsibilities:**
- Common timeout tracking and detection
- Standard telemetry generation
- Lifecycle state management (NOT_STARTED, RUNNING, COMPLETE, FAILED)
- Automatic commit on success

### Phase 4: Core Operations

**Need to Create:**
- `CollectOperation.java` - Collect from front or back intake
- `TransferOperation.java` - Transfer from intake to center
- `SwapOperation.java` - Push-style swap between center and intake
- `PrepositionOperation.java` - Position center artifact for firing
- `FireOperation.java` - Fire center artifact via BasicFiringHelper
- `EjectOperation.java` - Clear artifacts (emergency)

**Each Operation Must:**
1. Check preconditions in start()
2. Orchestrate BasicIndexingHelper/BasicFiringHelper
3. Track completion via helper status checks
4. Commit slot changes only when complete
5. Provide rich telemetry

### Phase 5: Shot Planning Integration

**Need to Create/Modify:**
- Shot plan consumption logic in FireOperation
- Rearrangement coordination (SwapOperation triggered by planner)
- Confidence-aware planning extensions

**Integration Points:**
- Existing ShotPlanner already generates plans
- Need to consume plan during firing sequence
- Need to trigger rearrangement when planner requests

### Phase 6: Main Controller

**Need to Create:**
- `IndexingSystemV3.java` - Main controller class

**Must Implement:**
- Public API matching existing IndexingSystem (for drop-in replacement)
- Operation request handling with gating rules
- Manual override detection and cancellation
- IntakePerception instances (front + back)
- OperationRunner lifecycle
- Integration with BasicIndexingHelper/BasicFiringHelper
- Integration with Shooter subsystem
- 7-gate firing precondition checks

**Key Methods:**
```java
void enable()
void disable()
void update()  // MUST call every loop
void reset()
boolean onFireSignal()
boolean setMotifPattern(String pattern)
void setManualInputDetector(Supplier<Boolean> detector)
int getArtifactCount()
boolean isReadyToFire()
// ... plus telemetry methods
```

### Phase 7: Testing & Validation

**Need to Create:**
- Unit tests for SlotLedger
- Unit tests for operations (with mocked helpers)
- Integration test OpMode (`IndexingSystemV3Test.java`)
- Hardware validation procedures

**Testing Strategy:**
1. Unit test slot ledger operations (swap, clear, queries)
2. Unit test operations in isolation (mock helpers)
3. Integration test full workflow (collect → transfer → fire)
4. Hardware test with real sensors and actuators
5. Shot planning integration test (rearrangement scenarios)

## 🔄 Current Architecture State

### Data Flow (Completed Layers)

```
Sensors (Hardware)
    ↓
IntakePerception (sensor fusion)
    ↓
[artifactHint, presenceConfidence, colorClass]
    ↓
??? IndexingSystemV3 (TO BE BUILT)
    ↓
??? OperationRunner (TO BE BUILT)
    ↓
??? Concrete Operations (TO BE BUILT)
    ↓
BasicIndexingHelper / BasicFiringHelper (existing HAL)
    ↓
Hardware Actuators
```

### State Management (Completed)

```
SlotLedger (single source of truth)
    ├─ CENTER slot: ArtifactIdentity?
    ├─ FRONT slot: ArtifactIdentity?
    └─ BACK slot: ArtifactIdentity?

Operations (to be built)
    ├─ Read slots freely
    ├─ Modify only at commit time
    └─ Never partial updates
```

## 📋 Next Steps for Completion

### Immediate Priority (Phase 3)

1. **Create BaseOperation abstract class**
   - Common timeout logic
   - Standard telemetry generation
   - State tracking (NOT_STARTED, RUNNING, COMPLETE, FAILED)
   - Helper method for elapsed time

2. **Create OperationRunner**
   - Enforce single operation at a time
   - Call operation.update() every loop
   - Auto-commit on success
   - Track current operation

### Example: CollectOperation (Phase 4 Sample)

```java
public class CollectOperation extends BaseOperation {
    private final SlotLedger ledger;
    private final IntakePerception perception;
    private final BasicIndexingHelper helper;
    private final SlotLedger.Slot targetSlot;
    private final int sequenceId;
    
    private ArtifactIdentity collectedArtifact;
    
    @Override
    public boolean start() {
        // Check preconditions
        if (ledger.isFull()) return false;
        if (ledger.isOccupied(targetSlot)) return false;
        
        // Read color at checkpoint
        collectedArtifact = ArtifactIdentity.createFromSensor(
            perception.getBestColorClass(),
            perception.getBestColorConfidence(),
            sequenceId
        );
        
        // Start hardware (intake rollers + transfer)
        helper.runXIntakeTimed(true, 1.0, 800);  // X = Front or Back
        
        return true;
    }
    
    @Override
    public boolean update() {
        // Check completion
        return helper.isXIntakeBusy();  // true = still running
    }
    
    @Override
    public void commit() {
        // Apply slot change
        ledger.set(targetSlot, collectedArtifact);
    }
}
```

## 🎓 Design Principles (Reference)

1. **Slot Ledger = Truth**: Never modify slots mid-operation
2. **Sensors = Hints**: Never auto-remove committed artifacts
3. **Operations = Transactions**: Atomic start → commit pattern
4. **One Operation at a Time**: OperationRunner enforces serialization
5. **Helper Orchestration**: Don't reimplement timing, use helpers
6. **Structured Telemetry**: Clear reject reasons and state snapshots

## 📚 Documentation References

- [v3/README.md](./README.md) - Full architecture documentation
- [../../INDEXING_SYSTEM_SPECIFICATION.md](../../INDEXING_SYSTEM_SPECIFICATION.md) - Original spec
- [../Indexing Rewrite Notes.md](../Indexing%20Rewrite%20Notes%20(Sensors%20+%20Tracking%20Assumptions).md) - Design notes

## 🚀 Estimated Completion

**Completed:** Phases 1-2 (~40% of implementation)
**Remaining:** Phases 3-7 (~60% of implementation)

**Time Estimates:**
- Phase 3 (Operation Framework): ~2-3 hours
- Phase 4 (Core Operations): ~4-6 hours (6 operations)
- Phase 5 (Shot Planning): ~1-2 hours
- Phase 6 (Main Controller): ~3-4 hours
- Phase 7 (Testing): ~2-3 hours

**Total Remaining:** ~12-18 hours of focused development

## ✨ Key Advantages of V3 Design

1. **Reliability**: Model-first approach prevents sensor-driven bugs
2. **Testability**: Clear operation boundaries, mockable helpers
3. **Maintainability**: Structured reject reasons, rich telemetry
4. **Safety**: Transaction model prevents partial updates
5. **Flexibility**: Easy to add new operations or sensor types
6. **Performance**: Minimal computational overhead (slot-based queries)

---

**Status:** 2 of 7 phases complete. Foundation is solid. Ready for operation implementation.

**Last Updated:** January 2026
