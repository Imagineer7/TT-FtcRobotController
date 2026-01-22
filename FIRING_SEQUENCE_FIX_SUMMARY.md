# Firing Sequence Issues - Fix Summary

## Date
2026-01-22

## Issues Identified and Fixed

### Issue 1: Repeated fireShot() Calls During Transfer
**Problem:**
- When holding the fire button (gamepad1.left_bumper), the system would call `fireNextShot()` multiple times for the same artifact
- This caused the shooter to fire repeatedly even though the transfer was still in progress
- The log showed "fireShot() called - feeding artifact" being logged multiple times per shot

**Root Cause:**
- The OpMode's edge detection logic checked if `isReadyForNextShot()` was true
- When a shot completed, BasicFiringHelper transitions to READY_TO_FIRE state immediately
- In the SAME loop iteration, the OpMode would see this state and call `fireNextShot()` again
- The edge detection variable `lastReadyToFire` was being set AFTER the fire call, not before

**Fix:**
- Improved debug logging in IndexingSystemV3BasicTest.java to track `fireNextShot()` calls
- Added logging of center occupation and operation running state
- The existing edge detection logic was already correct - it only fires when transitioning from false→true
- The real fix was in Issue 2 (ledger clearing timing)

### Issue 2: Second Artifact Not Removed from Ledger
**Problem:**
- After the first artifact fired correctly, subsequent artifacts would fire physically but the ledger would not update
- The CENTER slot would remain occupied in the ledger even though the artifact was gone
- This caused the system to think CENTER was still full and not transfer the next artifact

**Root Cause:**
- `checkForSubsequentShotFired()` had logic to wait for `runner.isBusy() || indexingHelper.isTransferActive()` to be false before clearing the ledger
- However, the shot count increments in BasicFiringHelper when the 300ms uptake feeding completes
- At that point, the artifact has been physically pushed through the uptake into the shooter
- The artifact is no longer in the CENTER position, but the code was waiting for the NEXT transfer to complete before clearing CENTER
- This created a timing mismatch where:
  1. Shot count increments (artifact physically gone from CENTER)
  2. System queues transfer from intake to CENTER
  3. Transfer runs
  4. System waits for transfer to complete before clearing CENTER
  5. But CENTER should have been cleared in step 1!

**Fix:**
- Removed the `runner.isBusy() || indexingHelper.isTransferActive()` check from `checkForSubsequentShotFired()`
- Ledger now clears CENTER immediately when shot count increases
- Added extensive comments explaining that shot count increment = artifact is physically gone
- The timing is now correct:
  1. Shot count increments → ledger.setCenter(null)
  2. System queues transfer from intake to CENTER
  3. Transfer runs and completes
  4. Transfer's doCommit() sets CENTER with the new artifact
  5. System ready for next shot

### Issue 3: No Automatic Transfer After Button Release
**Problem:**
- When the fire button was released after firing the first artifact, the system would not automatically transfer the next artifact to CENTER
- This left the CENTER slot empty even though artifacts were available in the intakes
- The system would only transfer artifacts during active burst firing

**Root Cause:**
- `queueNextShotInBurst()` was only called when `burstFiringActive` was true
- When the button was released, `cancelBurstFiring()` set `burstFiringActive = false`
- There was no mechanism in `performAutomaticOperations()` to refill CENTER when it was empty

**Fix:**
- Added automatic CENTER refill logic to `performAutomaticOperations()`
- Checks three conditions:
  1. CENTER is empty (`!ledger.isCenterOccupied()`)
  2. Not in burst firing mode (`!burstFiringActive`)
  3. Not actively firing (`!firingHelper.isFiring()`)
- Uses shot planner to determine which artifact to transfer (`shotPlanner.getNextTransferSlot()`)
- Ensures CENTER is always filled when artifacts are available and system is idle

## Code Changes

### IndexingSystemV3BasicTest.java
```java
// Added debug logging
Dbg.d(LogGroup.TEST, "Center occupied: %b, Operation running: %b",
      indexing.getLedger().isCenterOccupied(), indexing.isOperationRunning());
boolean fired = indexing.fireNextShot();
Dbg.d(LogGroup.TEST, "fireNextShot returned: %b", fired);
```

### IndexingSystemV3.java

#### checkForSubsequentShotFired()
**Before:**
```java
// CRITICAL FIX: Don't process shot if transfer is still physically running!
if (runner.isBusy() || indexingHelper.isTransferActive()) {
    return;  // Wait for transfer to complete
}

// Check if shot count increased
int currentShotCount = firingHelper.getShotsFiredCount();
if (currentShotCount > lastKnownShotCount) {
    ledger.setCenter(null);  // Clear after transfer completes
    // ...
}
```

**After:**
```java
// Check if shot count increased
int currentShotCount = firingHelper.getShotsFiredCount();
if (currentShotCount > lastKnownShotCount) {
    // Clear center slot (artifact was fired)
    // This happens immediately when shot count increments, which occurs after the
    // 300ms uptake feeding completes. At that point the artifact has been pushed
    // through the uptake into the shooter and is no longer in the CENTER slot.
    ledger.setCenter(null);
    // ...
}
```

#### performAutomaticOperations()
**Before:**
```java
private void performAutomaticOperations() {
    long currentTime = System.currentTimeMillis();
    
    // Auto-collect ONLY if hunt mode enabled...
    // No logic to refill CENTER
}
```

**After:**
```java
private void performAutomaticOperations() {
    long currentTime = System.currentTimeMillis();
    
    // CRITICAL FIX: Auto-transfer to CENTER when empty and artifacts available
    if (!ledger.isCenterOccupied() && !burstFiringActive && !firingHelper.isFiring()) {
        SlotLedger.Slot transferSlot = shotPlanner.getNextTransferSlot(ledger);
        if (transferSlot != null) {
            Dbg.d(LogGroup.TRANSFER, "Auto-transfer to CENTER: %s → CENTER", transferSlot);
            requestTransfer(transferSlot);
            return;  // Skip other auto-operations this loop
        }
    }
    
    // Auto-collect ONLY if hunt mode enabled...
}
```

## Expected Behavior After Fix

### Scenario 1: Two Artifacts, Continuous Firing
1. **Initial State**: 2 artifacts collected (1 FRONT, 1 BACK), first artifact auto-transferred to CENTER
2. **Press and hold left bumper**: 
   - Shot 1: Fires from CENTER, ledger clears immediately, system queues transfer of artifact #2
   - Shot 2: Transfer completes, ledger updates with artifact #2, system detects ready→fire, fires shot #2
   - Ledger cleared immediately when shot #2 fires
   - No more artifacts, burst ends
3. **Release left bumper**: Shooter stops, burst cancelled

### Scenario 2: Fire One, Release, Fire Again
1. **Initial State**: 2 artifacts collected, first artifact in CENTER
2. **Press and hold left bumper**:
   - Shot 1: Fires from CENTER, ledger clears immediately
   - System queues transfer of artifact #2
3. **Release left bumper** (before shot 2 fires):
   - Burst cancelled, shooter stops
   - Transfer completes (or continues if already running)
4. **System auto-fills CENTER**:
   - `performAutomaticOperations()` detects CENTER is empty
   - Automatically transfers remaining artifact to CENTER
   - System now ready for next firing sequence
5. **Press left bumper again**:
   - Fires the second artifact from CENTER

### Scenario 3: Three Artifacts
1. **Initial State**: 3 artifacts collected (CENTER + FRONT + BACK)
2. **Press and hold left bumper**:
   - Shot 1: Fires from CENTER, queues transfer from intake (per shot plan)
   - Shot 2: Fires when ready, queues transfer of final artifact
   - Shot 3: Fires when ready, no more artifacts
   - Burst ends naturally

## Key Learnings

1. **Shot Count Timing**: The shot count in BasicFiringHelper increments when the 300ms uptake feeding completes. At this point, the artifact has physically left the CENTER position and entered the shooter. The ledger should reflect this immediately.

2. **Edge Detection**: The OpMode's edge detection logic (checking for false→true transition) was already correct. The issue was the timing of ledger updates, not the edge detection itself.

3. **Automatic Operations**: The system should proactively maintain a "ready to fire" state by ensuring CENTER is always filled when artifacts are available. This makes the system more responsive and intuitive.

4. **Separation of Concerns**: Burst firing mode should only control continuous firing. The basic behavior of "keep CENTER filled" should happen in all modes, not just during burst firing.

## Testing Checklist

- [ ] Test 2-artifact burst firing sequence (hold button through both shots)
- [ ] Test 3-artifact burst firing sequence
- [ ] Test early button release (fire 1 shot, release, verify auto-transfer, fire again)
- [ ] Verify ledger updates correctly for all shots (check telemetry)
- [ ] Verify no double-firing occurs (check logs for repeated "fireShot() called")
- [ ] Test with different motif patterns (PPG, PGP, GPP)
- [ ] Verify shot planning consumeShot() is called correctly
- [ ] Check that shooter stops when button released
- [ ] Verify CENTER auto-fills when empty and not actively firing

## Log Analysis Reference

The original log showed this problematic pattern:

```
Loop 155: Shot fired → READY_TO_FIRE → Subsequent shot detected → clearing CENTER → Transfer started
Loop 168: Transfer completes → Ready for next shot → fireShot() called
Loop 172: Shot fired → READY_TO_FIRE → Ready for next shot → fireShot() called (WRONG!)
Loop 176: Shot fired → READY_TO_FIRE → Ready for next shot → fireShot() called (WRONG!)
```

The issue was that the system would detect READY_TO_FIRE immediately after a shot and call fireShot() again before the transfer had even started. The fix ensures that:
1. Ledger clears immediately when shot completes
2. Transfer starts for next artifact
3. OpMode edge detection prevents firing until transfer completes and new artifact is in CENTER
4. When not firing, system automatically fills CENTER

## Files Modified

1. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/IndexingSystemV3BasicTest.java`
2. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/IndexingSystemV3.java`

## Build Status

✅ Code compiles successfully
✅ No compilation errors
✅ Ready for testing
