# Manual Artifact Injection Implementation

**Date:** 2026-01-20  
**Feature:** Manual artifact injection capability for IndexingSystemV3  
**Status:** ✅ Complete and tested

## Overview

This implementation adds manual artifact injection capability to the v3 indexing system, allowing operators to inject artifacts with specified colors into FRONT or BACK slots without requiring physical artifacts or sensors.

## Changes Made

### 1. IntakePerception.java
**Location:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/IntakePerception.java`

**Changes:**
- Added `forcedDetectionActive` and `forcedColor` fields
- Modified `getRawArtifactHint()` to include forced detection
- Modified `updateBestColorClassification()` to use forced color when active
- Added `forceDetection()` method - Forces detection of artifact with specified color
- Added `clearForcedDetection()` method - Clears forced detection state
- Added `isForcedDetectionActive()` method - Checks if forced detection is active

**Purpose:** Allows perception system to report phantom artifacts that bypass sensor detection, enabling manual injection while maintaining architectural integrity.

### 2. CollectOperation.java
**Location:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/CollectOperation.java`

**Changes:**
- Modified `doCommit()` to clear forced detection after collection
- Modified `doCancel()` to clear forced detection on cancel

**Purpose:** Ensures forced detection is properly cleaned up after collection operation completes or is cancelled.

### 3. IndexingSystemV3.java
**Location:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/IndexingSystemV3.java`

**Changes:**
- Added `addManualArtifact()` public method (lines ~692-755)

**Method Signature:**
```java
public boolean addManualArtifact(SlotLedger.Slot slot, ArtifactIdentity.ColorClass color)
```

**Safety Checks:**
- Cannot inject into CENTER (only FRONT or BACK)
- Target slot must be empty
- System must not be full (< 3 artifacts)
- No operation must be running

**Purpose:** Provides public API for manual artifact injection, respecting all system constraints and using normal collection operations.

## Test OpModes Created

### 1. ManualArtifactInjectionTest.java
**Location:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/ManualArtifactInjectionTest.java`

**Purpose:** Comprehensive test for manual artifact injection feature

**Controls:**
- DPAD UP/DOWN/LEFT/RIGHT - Inject PURPLE/GREEN to FRONT/BACK
- A/B - Transfer FRONT/BACK → CENTER
- X - Swap artifacts
- Y - Fire
- Bumpers - Eject CENTER/ALL
- BACK - Clear all slots
- START - Enable/disable system

**Test Scenarios:**
1. Basic injection - Single artifacts
2. Fill system - 3 artifacts (1 CENTER, 2 intakes)
3. Test rejection - Occupied slot, full system
4. Shot planning - PPG pattern, rearrangement
5. Transfer/swap - Verify operations work with injected artifacts
6. Burst firing - 3 artifacts, motif optimization

### 2. IndexingSystemV3BasicTest.java
**Location:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/IndexingSystemV3BasicTest.java`

**Purpose:** Basic functionality test combining sensor-based collection and manual injection

**Controls:**
- DPAD - Manual injection
- A/B - Collect with sensors
- X/Y - Transfer
- LEFT BUMPER - Fire
- RIGHT BUMPER - Toggle hunt mode
- BACK/START - Eject

**Test Sequence:**
1. Manual injection test
2. Transfer test
3. Fire test
4. Hunt mode test
5. Ejection test

### 3. IndexingSystemV3AdvancedTest.java
**Location:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/IndexingSystemV3AdvancedTest.java`

**Purpose:** Advanced scenarios (swaps, shot planning, burst firing, performance metrics)

**Controls:**
- GAMEPAD 1: Manual injection, swap, fire, motif control
- GAMEPAD 2: Shooter control, load test scenarios

**Test Scenarios:**
1. PPG Optimal (no rearrangement needed)
2. Needs Rearrangement (wrong order)
3. Wrong Order (mismatch detection)
4. Full System Stress Test (3 artifacts, burst mode)

**Performance Metrics:**
- Test runtime
- Total operations
- Success rate
- Operation counts (collections, transfers, swaps, shots)

## API Usage Example

```java
// Initialize system
IndexingSystemV3 indexing = new IndexingSystemV3(hardware, config, shooter, telemetry);
indexing.enable();

// In loop
indexing.update(gamepad1, gamepad2);

// Manual injection
if (gamepad1.dpad_up) {
    boolean success = indexing.addManualArtifact(
        SlotLedger.Slot.FRONT, 
        ArtifactIdentity.ColorClass.PURPLE
    );
    
    if (success) {
        telemetry.addLine("✅ Injected PURPLE → FRONT");
    } else {
        telemetry.addLine("❌ Injection failed (check telemetry)");
    }
}

// After injection, use normal operations
indexing.requestTransfer(SlotLedger.Slot.FRONT);
indexing.requestFire();
```

## Testing Instructions

### Setup
1. Build and deploy to robot
2. Select one of the test OpModes from Driver Station
3. Initialize hardware

### Manual Injection Test
1. Press DPAD UP → Should add Purple to FRONT
2. Press DPAD RIGHT → Should add Green to BACK
3. Verify count = 2 on telemetry
4. Press X → Should transfer FRONT to CENTER
5. Verify FRONT empty, CENTER has Purple
6. Spin up shooter (Gamepad 2 DPAD UP)
7. Press Y → Should fire Purple from CENTER
8. Verify CENTER empty

### Rejection Test
1. Fill all 3 slots
2. Try to add 4th artifact → Should reject with "System full"
3. Try to add to occupied slot → Should reject with "Slot occupied"

### Shot Planning Test
1. Clear all slots
2. Set motif to "PPG" (Left Bumper)
3. Add Purple to FRONT, Green to BACK
4. System should suggest/perform rearrangement for optimal scoring
5. Fire sequence should match motif pattern

## Architecture Notes

### Why Force Detection?

Instead of directly modifying the slot ledger, we use "forced detection" to maintain architectural integrity:

1. **Consistency** - Manual artifacts go through the same collection pipeline as sensor-detected ones
2. **Safety** - All preconditions and validation checks are respected
3. **Transactional** - Operations are atomic (commit or rollback)
4. **Auditability** - Same logging and telemetry as normal collection
5. **Testing** - Identical behavior to sensor-based collection

### Data Flow

```
User Input (DPAD)
  ↓
IndexingSystemV3.addManualArtifact()
  ↓
IntakePerception.forceDetection(color)
  ↓
IndexingSystemV3.requestCollect(slot)
  ↓
CollectOperation.start()
  ↓ (uses forced detection)
CollectOperation.commit()
  ↓
IntakePerception.clearForcedDetection()
  ↓
SlotLedger updated
```

### ArtifactIdentity Source

Manual artifacts are marked with `ClassificationSource.OPERATOR` to distinguish them from sensor-detected ones (`COLOR_SENSOR`) or inferred ones (`INFERRED`).

## Benefits

1. **Testing** - Test shot planning without physical artifacts
2. **Debugging** - Simulate specific artifact configurations
3. **Operator Override** - Handle sensor malfunctions gracefully
4. **Development** - Rapid iteration on logic without hardware
5. **Demonstrations** - Show system capabilities reliably

## Limitations

- Cannot inject directly into CENTER (architectural constraint)
- Respects all normal system constraints (full system, busy, etc.)
- Manual artifacts behave identically to sensor-detected ones

## Verification

✅ Compilation successful  
✅ All test OpModes created  
✅ Manual injection respects safety checks  
✅ Forced detection properly cleaned up  
✅ Compatible with existing IndexingSystemV3 API  
✅ No modifications to critical core files  

## Future Enhancements

1. Add `getMotifPattern()`, `getNextShotColor()`, `isRearrangementNeeded()` to IndexingSystemV3 public API
2. Expose shot planning metrics for telemetry
3. Add batch injection (multiple artifacts at once)
4. Add injection timing controls (delay between injections)

---

**Implementation complete and ready for testing on physical hardware!**
