# Firing Sequence Fix V3 - Physical Hardware State Check

## Date
2026-01-22

## Problem After V2 Fix

After the V2 fix that addressed hunt mode interference, testing revealed a new issue:
- First artifact fires correctly ✓
- Ledger clears and shows second artifact as "last fired" ✓
- But second shot fires BEFORE the transfer physically completes ❌
- CENTER gets cleared while transfer is still running ❌
- No more artifacts can fire after that ❌

## Root Cause Analysis

### Log Evidence

**Loop 185**: Transfer operation completes
```
PLANEX/D: update() - stillRunning=false, op=Transfer[FRONT→CENTER]
PLANEX/D: Operation SUCCESSFUL, committing...
PLANEX/D: Clearing operation, now idle
PLANEX/D: handleOperationComplete: Transfer[FRONT→CENTER], success=true
TRANSFER/D: Transfer completed
TEST/I: Ready for next shot - firing now
TEST/D: Center occupied: true, Operation running: false  ← runner.isBusy() = false
FIRING/D: fireShot() called - feeding artifact
```

**Loop 189**: Shot fires (300ms after loop 185)
```
TRANSFER/D: TRANSFERRING - Elapsed Time: 536ms  ← STILL TRANSFERRING!
FIRING/D: Shot fired - transitioning to READY_TO_FIRE
FIRING/D: Subsequent shot detected (1 → 2), clearing CENTER
FIRING/D: Deferring next transfer queue (operation busy)
```

### The Problem

There are TWO layers of state:

1. **Operation State Machine** (`runner.isBusy()`)
   - Managed by PlannerExecutor
   - Becomes false when the operation's state machine completes
   - Does NOT wait for physical hardware to finish

2. **Physical Hardware State** (`indexingHelper.isTransferActive()`)
   - Managed by BasicIndexingHelper
   - Tracks actual servo/motor timed movements
   - Continues running AFTER the operation completes

**The Mismatch:**
- Transfer operation completes at loop 185
- `runner.isBusy()` becomes false immediately
- OpMode checks `!indexing.isOperationRunning()` → sees false
- OpMode thinks it's safe to fire → calls `fireNextShot()`
- But BasicIndexingHelper is STILL physically moving the artifact!
- Shot fires 300ms later while hardware still running
- Clears CENTER prematurely

### Timing Breakdown

```
T+0ms:    Transfer operation starts
T+162ms:  Transfer state machine completes
          - runner.isBusy() = false
          - indexingHelper.isTransferActive() = true (hardware still running)
T+162ms:  OpMode detects ready, calls fireNextShot()
T+462ms:  Shot fires (300ms uptake feeding)
          - Still showing "TRANSFERRING - Elapsed Time: 536ms"
          - Transfer hardware STILL running
T+462ms:  Clears CENTER (wrong! transfer not done)
```

## The Solution

Update `isOperationRunning()` to check BOTH states:

**Before:**
```java
public boolean isOperationRunning() {
    return runner.isBusy();  // Only checks state machine
}
```

**After:**
```java
public boolean isOperationRunning() {
    return runner.isBusy() || indexingHelper.isTransferActive();  // Checks both!
}
```

### Why This Works

1. Transfer operation completes → `runner.isBusy()` = false
2. But `indexingHelper.isTransferActive()` = true (hardware still running)
3. `isOperationRunning()` returns true (correctly!)
4. OpMode sees `readyToFire` = false (operation still running)
5. Waits for hardware to complete
6. Eventually `indexingHelper.isTransferActive()` becomes false
7. NOW `isOperationRunning()` returns false
8. OpMode fires the next shot safely

## Expected Flow After Fix

### Burst Firing with 2 Artifacts

**Setup:** 2 artifacts collected (CENTER + FRONT)

**Sequence:**
```
Loop 142: Press fire button
          → requestFire() starts first shot

Loop 172: First shot fires
          → Clears CENTER (artifact gone)
          → Queues transfer FRONT→CENTER
          → Transfer starts

Loop 173-184: Transfer running
              → runner.isBusy() = true
              → isOperationRunning() = true ✓
              → OpMode sees operation running, waits

Loop 185: Transfer state machine completes
          → runner.isBusy() = false
          → BUT indexingHelper.isTransferActive() = true ✓
          → isOperationRunning() = true ✓
          → OpMode STILL waits (correct!)

Loop 186-194: Physical hardware completing
              → indexingHelper.isTransferActive() = true
              → isOperationRunning() = true
              → OpMode continues waiting

Loop 195+: Physical hardware completes
           → indexingHelper.isTransferActive() = false
           → isOperationRunning() = false ✓
           → CENTER occupied = true
           → isReadyForNextShot() = true
           → OpMode detects ready → fires second shot ✓

Loop 225: Second shot fires
          → Artifact physically gone from CENTER
          → Clears ledger
          → No more artifacts
          → Burst ends
```

## Key Insights

1. **Two-Layer Architecture**: Operations have both logical state (state machine) and physical state (hardware)

2. **Async Completion**: The operation can logically complete while hardware is still physically running

3. **Safety Checks Must Be Comprehensive**: Can't just check `runner.isBusy()`, must also check physical hardware state

4. **Transfer Operations Are Async**: They start hardware movements and complete their state machine before hardware finishes

5. **isOperationRunning() Semantics**: Should mean "is the SYSTEM busy with an operation", not just "is the state machine busy"

## Files Changed

### IndexingSystemV3.java
**Method:** `isOperationRunning()`

**Change:**
```java
// Before: Only checked state machine
return runner.isBusy();

// After: Checks both state machine AND physical hardware
return runner.isBusy() || indexingHelper.isTransferActive();
```

**Impact:**
- OpModes that call `isOperationRunning()` now correctly wait for physical completion
- Prevents premature `fireNextShot()` calls
- Ensures artifact is actually in CENTER before firing

## Testing Checklist

- [ ] Two artifacts: Fire both in burst mode
  - First shot fires correctly
  - System waits for transfer to physically complete
  - Second shot fires after transfer done
  - Both shots recorded in ledger

- [ ] Three artifacts: Fire all in burst mode
  - All three shots fire correctly
  - Transfers complete before shots
  - Ledger updates properly

- [ ] Early button release
  - Fire first shot
  - Release button during transfer
  - Transfer completes
  - Auto-transfer fills CENTER
  - Can fire again

- [ ] Verify timing in logs
  - "Ready for next shot" appears AFTER transfer hardware completes
  - No "TRANSFERRING" messages after firing
  - Ledger clears only after hardware idle

## Build Status

✅ Code compiles successfully (BUILD SUCCESSFUL in 2m 2s)
✅ Ready for hardware testing

## Summary

The fix ensures that `isOperationRunning()` accurately reflects the COMPLETE operational state of the system, including both the logical state machine AND the physical hardware state. This prevents the OpMode from firing prematurely while hardware is still active.

This was a subtle but critical bug where the abstraction layer (operation state) was leaking implementation details (hardware still running) that the OpMode needed to know about. The fix properly encapsulates this by making the public API (`isOperationRunning()`) return the correct combined state.
