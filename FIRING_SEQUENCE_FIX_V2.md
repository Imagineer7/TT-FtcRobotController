# Firing Sequence Fix V2 - Complete Analysis and Solution

## Date
2026-01-22

## Problem Evolution

### Initial Problem (Original Log)
- First shot fired correctly
- Second shot: Physical firing happened, but ledger didn't update
- Repeated `fireShot()` calls during transfer

### After First Fix Attempt
- System stopped firing altogether
- Hunt mode auto-collection interfered with burst firing
- CENTER was cleared immediately after being filled by transfer

## Root Cause Analysis

### The Complete Flow (What Was Happening)

**Loop 152**: First shot fires
- Shot count increments (0→1)
- `checkForSubsequentShotFired()` detects it
- Clears CENTER immediately ✓
- Queues transfer from FRONT to CENTER

**Loop 164**: Transfer completes
- Transfer's `doCommit()` sets CENTER with new artifact
- System detects ready to fire
- OpMode calls `fireNextShot()`
- Second shot starts firing

**Loop 166**: Hunt mode auto-collects
- While transfer was running (152-164), hunt mode saw empty FRONT
- Started a collection operation from FRONT
- Collection runs in parallel with the firing

**Loop 168**: THE PROBLEM
- Collection completes successfully
- `handleOperationComplete()` → `handlePostCollection()`
- `handlePostCollection()` sees artifactCount=2
- Treats it as "2nd normal collection" → doesn't transfer
- Meanwhile, `checkForSubsequentShotFired()` runs
- Detects shot (1→2), clears CENTER
- **But CENTER was just filled by the transfer!**
- Result: CENTER empty, no more artifacts to fire

### Why the First Fix Failed

My initial fix cleared the ledger immediately (correct) but didn't account for:
1. Hunt mode auto-collecting during burst firing
2. `handlePostCollection()` being called when collection completes during burst
3. The conflict between burst firing's transfer logic and normal collection logic

## The Correct Solution

### Key Principles

1. **Ledger Timing**: Clear CENTER immediately when shot count increments (artifact is physically gone)
2. **Transfer Deferral**: If operations are running when shot fires, defer the transfer queue
3. **Context Separation**: Burst firing has its own transfer logic; don't let normal collection logic interfere
4. **Auto-Transfer**: When not in burst mode, automatically fill CENTER from available artifacts

### Implementation Details

#### 1. checkForSubsequentShotFired() - Immediate Clear with Deferred Queue

```java
private void checkForSubsequentShotFired() {
    if (!burstFiringActive) return;
    
    int currentShotCount = firingHelper.getShotsFiredCount();
    if (currentShotCount > lastKnownShotCount) {
        // CRITICAL: Clear CENTER immediately when shot fires
        // The artifact has been physically pushed into the shooter
        ledger.setCenter(null);
        
        // Update tracking
        lastKnownShotCount = currentShotCount;
        // ... other tracking updates ...
        
        // Queue next transfer ONLY if no operations running
        // This prevents conflicts with ongoing transfers or collections
        if (!runner.isBusy() && !indexingHelper.isTransferActive()) {
            queueNextShotInBurst();
        } else {
            Dbg.d(LogGroup.FIRING, "Deferring next transfer queue (operation busy)");
            deferredTransferNeeded = true;  // Will queue when operation completes
        }
    }
}
```

**Why this works:**
- Clears ledger immediately (physically accurate)
- Doesn't queue transfer if operation busy (prevents conflicts)
- Sets flag to queue later (ensures transfer happens eventually)

#### 2. handleOperationComplete() - Skip Collection Logic During Burst

```java
private void handleOperationComplete() {
    // ... operation type checks ...
    
    if (lastOp instanceof CollectOperation) {
        totalCollections++;
        
        // IMPORTANT: Skip handlePostCollection if in burst firing mode
        // Burst logic manages its own transfers
        if (!burstFiringActive) {
            handlePostCollection();
        } else {
            Dbg.d(LogGroup.INDEXING, "Skipping handlePostCollection (burst firing active)");
        }
        
    } else if (lastOp instanceof TransferOperation) {
        totalTransfers++;
        
        // If we deferred a transfer during burst firing, queue it now
        if (deferredTransferNeeded && burstFiringActive) {
            Dbg.d(LogGroup.FIRING, "Queuing deferred transfer");
            queueNextShotInBurst();
            deferredTransferNeeded = false;
        }
    }
    // ... other operation types ...
}
```

**Why this works:**
- Collections during burst don't trigger `handlePostCollection()`
- Prevents the "2nd artifact" logic from running during burst
- Deferred transfers are queued when transfer completes
- Normal collections (not during burst) still work correctly

#### 3. performAutomaticOperations() - Auto-Transfer for Non-Burst

```java
private void performAutomaticOperations() {
    long currentTime = System.currentTimeMillis();
    
    // Auto-transfer to CENTER when empty (not in burst firing mode)
    // Burst firing handles its own transfers via queueNextShotInBurst()
    if (!ledger.isCenterOccupied() && !burstFiringActive && !firingHelper.isFiring()) {
        // CENTER is empty and we're not actively firing - try to fill it
        SlotLedger.Slot transferSlot = shotPlanner.getNextTransferSlot(ledger);
        if (transferSlot != null) {
            Dbg.d(LogGroup.TRANSFER, "Auto-transfer to CENTER: %s → CENTER", transferSlot);
            requestTransfer(transferSlot);
            return;  // Skip other auto-operations this loop
        }
    }
    
    // ... hunt mode auto-collect logic ...
}
```

**Why this works:**
- Ensures CENTER is filled when button released (not in burst mode)
- Doesn't interfere with burst firing (checks !burstFiringActive)
- Uses shot planner to select correct artifact

## Flow Diagrams

### Correct Burst Firing Flow

```
Loop 152: Shot 1 fires
  └─ checkForSubsequentShotFired()
     ├─ Detects count 0→1
     ├─ Clears CENTER immediately ✓
     ├─ Checks: runner.isBusy()? NO
     └─ Queues transfer FRONT→CENTER

Loop 164: Transfer completes
  └─ handleOperationComplete()
     ├─ TransferOperation completed
     ├─ Resets perception
     └─ No deferred transfer (wasn't set)
  └─ OpMode detects ready → fireNextShot()

Loop 166: Hunt collects (during second shot)
  └─ Auto-collect starts from FRONT
  └─ Collection operation running...

Loop 168: Collection completes
  └─ handleOperationComplete()
     ├─ CollectOperation completed
     ├─ Checks: burstFiringActive? YES
     └─ Skips handlePostCollection() ✓
  └─ checkForSubsequentShotFired()
     ├─ Detects count 1→2
     ├─ Clears CENTER immediately ✓
     ├─ Checks: runner.isBusy()? NO (collection just finished)
     └─ Queues transfer FRONT→CENTER

Loop 180: Transfer completes
  └─ CENTER filled with next artifact
  └─ Ready to fire shot 3
```

### Handling Deferred Transfers

```
Loop 152: Shot fires while transfer running
  └─ checkForSubsequentShotFired()
     ├─ Clears CENTER ✓
     ├─ Checks: runner.isBusy()? YES (transfer from shot 1)
     └─ Sets deferredTransferNeeded = true

Loop 164: Transfer completes
  └─ handleOperationComplete()
     ├─ TransferOperation completed
     ├─ Checks: deferredTransferNeeded? YES
     ├─ Calls queueNextShotInBurst() ✓
     └─ Sets deferredTransferNeeded = false

Loop 165+: New transfer runs
  └─ Artifact moves to CENTER
  └─ Ready for next shot
```

### Auto-Transfer When Not Firing

```
Loop 216: Button released
  └─ handleFire()
     ├─ Detects button release
     └─ Calls cancelBurstFiring()
        ├─ Sets burstFiringActive = false
        └─ Stops shooter

Loop 217+: Automatic operations
  └─ performAutomaticOperations()
     ├─ Checks: !centerOccupied? YES
     ├─ Checks: !burstFiringActive? YES
     ├─ Checks: !isFiring()? YES
     └─ Calls requestTransfer() ✓
        └─ Transfers remaining artifact to CENTER

Loop 230+: Button pressed again
  └─ CENTER has artifact
  └─ Can fire immediately
```

## Testing Expectations

### Test 1: Two Artifacts, Continuous Hold
**Setup**: 2 artifacts (FRONT + BACK), first auto-transferred to CENTER

**Expected:**
1. Press and hold left bumper
2. Shot 1 fires from CENTER
3. Ledger clears immediately
4. Transfer queues from intake
5. Transfer completes, CENTER filled
6. Shot 2 fires from CENTER
7. Ledger clears immediately
8. No more artifacts, burst ends
9. Shooter stays spinning (button still held)
10. Release button, shooter stops

**Log markers to verify:**
- "Subsequent shot detected (0 → 1), clearing CENTER"
- "Starting operation: Transfer[...]"
- "Subsequent shot detected (1 → 2), clearing CENTER"
- No "Skipping handlePostCollection (burst firing active)" during the sequence

### Test 2: Fire One, Release, Fire Again
**Setup**: 2 artifacts (FRONT + BACK), first in CENTER

**Expected:**
1. Press left bumper → Shot 1 fires
2. Release bumper before shot 2
3. Burst cancelled, shooter stops
4. `performAutomaticOperations()` detects CENTER empty
5. Auto-transfers remaining artifact to CENTER
6. Press left bumper again → Shot 2 fires

**Log markers to verify:**
- "Fire button released - canceling firing sequence"
- "Auto-transfer to CENTER: [FRONT or BACK] → CENTER"
- Second press shows "Starting firing with SHORT_RANGE preset"

### Test 3: Hunt Mode Collecting During Burst
**Setup**: 1 artifact in CENTER, 1 in intake, hunt mode ON

**Expected:**
1. Press and hold left bumper
2. Shot 1 fires, clears CENTER
3. Transfer starts from intake
4. Hunt mode detects artifact, starts collection
5. Collection completes → skips handlePostCollection
6. Transfer completes, CENTER filled
7. Shot 2 fires correctly

**Log markers to verify:**
- "Auto-collect FRONT (confidence=HIGH)" during burst
- "Skipping handlePostCollection (burst firing active)"
- "Subsequent shot detected" happens AFTER "Skipping handlePostCollection"
- Transfer completes and fills CENTER
- Next shot fires successfully

## Key Changes Summary

### Files Modified
1. `IndexingSystemV3.java` - Core burst firing logic
2. `IndexingSystemV3BasicTest.java` - Debug logging

### New Fields
- `deferredTransferNeeded` - Tracks when transfer should be queued after operations complete

### Modified Methods
- `checkForSubsequentShotFired()` - Immediate clear, deferred queue
- `handleOperationComplete()` - Skip handlePostCollection during burst, process deferred transfers
- `performAutomaticOperations()` - Auto-transfer when not in burst mode

### Logic Flow
1. Shot fires → clear ledger immediately
2. If busy → defer transfer
3. When idle → queue transfer
4. Collection during burst → skip normal logic
5. Not in burst → auto-fill CENTER

## Lessons Learned

1. **Timing Matters**: Physical hardware state (artifact gone) vs logical state (ledger) must sync
2. **Context Separation**: Different modes (burst vs normal) need different logic paths
3. **Operation Conflicts**: Multiple automatic systems (hunt, burst, post-collection) can conflict
4. **Deferred Actions**: When busy, defer and process later rather than blocking or skipping
5. **Testing with Hunt Mode**: Hunt mode's automatic behavior adds complexity to burst firing

## Build Status
✅ Compiles successfully
✅ Ready for hardware testing
