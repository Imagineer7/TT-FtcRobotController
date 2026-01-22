# BasicFiringHelper & BasicIndexingHelper - Migration Guide

**Date:** January 20, 2026  
**Affected Classes:** `BasicFiringHelper`, `BasicIndexingHelper`  
**Version:** Aurora System v2

---

## Overview

Recent updates to the `BasicFiringHelper` and `BasicIndexingHelper` classes have simplified the API and improved functionality. This guide explains what changed and how to update your code.

---

## Summary of Changes

### 1. BasicIndexingHelper Constructor Simplified

**BEFORE:**
```java
// Old constructor took 3 parameters
BasicIndexingHelper indexingHelper = new BasicIndexingHelper(hardware, indexingConfig, telemetry);
```

**AFTER:**
```java
// New constructor takes 2 parameters (IndexingConfig removed)
BasicIndexingHelper indexingHelper = new BasicIndexingHelper(hardware, telemetry);
```

**Why?** The `IndexingConfig` parameter was unnecessary for basic manual control. The helper class directly controls hardware without needing configuration parameters.

### 2. BasicIndexingHelper No Longer Has enable() / disable() Methods

**BEFORE:**
```java
indexingHelper = new BasicIndexingHelper(hardware, telemetry);
indexingHelper.enable();  // ❌ No longer exists

// ... later ...
indexingHelper.disable();  // ❌ No longer exists
```

**AFTER:**
```java
indexingHelper = new BasicIndexingHelper(hardware, telemetry);
// No enable() call needed - ready to use immediately

// ... later ...
// No disable() call needed - just stop using it
```

**Why?** BasicIndexingHelper is a stateless helper that provides direct hardware control. There's no internal state machine that needs to be enabled/disabled. You can start/stop motors directly without managing an enabled state.

### 3. BasicFiringHelper Now Supports External Fire Commands

**NEW FEATURE:** Keep-alive mode with external firing control

**BEFORE (Old Behavior):**
```java
// Would fire once then automatically keep firing
firingHelper.startFiring(3200, "HIGH", true);
```

**AFTER (New Behavior):**
```java
// Fire first shot, then wait for explicit fire commands
firingHelper.startFiring(3200, "HIGH", true);  // Spin up and fire once

// In your loop, when you want to fire another shot:
if (firingHelper.isReadyForNextShot()) {
    if (gamepad1.a) {  // User presses button
        firingHelper.fireShot();  // Explicitly fire next shot
    }
}

// Stop when done
if (gamepad1.b) {
    firingHelper.cancelFiring();
}
```

**Why?** The old behavior kept firing automatically, which didn't allow for external control. The new behavior fires once, then waits in `READY_TO_FIRE` state for explicit commands via `fireShot()`, giving OpModes full control over when shots fire.

---

## Dead-Man Switch Implementation

The new `fireShot()` method enables **dead-man switch control** - a safety pattern where releasing a trigger immediately stops the action:

```java
// Track button state
boolean currentTrigger = gamepad1.right_trigger > 0.5;

if (currentTrigger && !lastTrigger) {
    // Just pressed - start firing with keep-alive
    firingHelper.startFiring(3200, "HIGH", true);
}

// While held, auto-fire when ready
if (currentTrigger && firingHelper.isReadyForNextShot()) {
    firingHelper.fireShot();  // Fire automatically while held
}

// Just released - STOP immediately (dead-man switch)
if (!currentTrigger && lastTrigger) {
    firingHelper.cancelFiring();
}

lastTrigger = currentTrigger;
```

**Benefits:**
- ✅ Immediate stop on button release (safety)
- ✅ Continuous firing while held
- ✅ Responsive control
- ✅ Predictable behavior

---

## Migration Steps

### Step 1: Update BasicIndexingHelper Initialization

**Find code like this:**
```java
IndexingConfig indexingConfig = new IndexingConfig();
BasicIndexingHelper indexingHelper = new BasicIndexingHelper(hardware, indexingConfig, telemetry);
indexingHelper.enable();
```

**Replace with:**
```java
BasicIndexingHelper indexingHelper = new BasicIndexingHelper(hardware, telemetry);
// No enable() needed - ready to use immediately
```

**Also remove:**
```java
indexingHelper.disable();  // Remove this line from cleanup/stop
```

### Step 2: Update BasicFiringHelper Usage (If Using Keep-Alive)

**Find code like this:**
```java
// Old: Would auto-fire continuously
firingHelper.startFiring(3200, "HIGH", true);
```

**Replace with:**
```java
// New: Requires explicit fireShot() calls
firingHelper.startFiring(3200, "HIGH", true);  // Fires once, then waits

// In your loop - add this:
if (firingHelper.isReadyForNextShot()) {
    if (/* your fire condition */) {
        firingHelper.fireShot();  // Trigger next shot
    }
}
```

### Step 3: Implement Dead-Man Switch (Recommended)

For safe, responsive control, implement dead-man switch pattern:

```java
// Add field to track button state
private boolean lastFireButton = false;

// In your loop:
boolean currentFireButton = gamepad1.right_trigger > 0.5;

// Pressed - start
if (currentFireButton && !lastFireButton) {
    firingHelper.startFiring(3200, "HIGH", true);
}

// While held - auto-fire
if (currentFireButton && firingHelper.isReadyForNextShot()) {
    firingHelper.fireShot();
}

// Released - STOP
if (!currentFireButton && lastFireButton) {
    firingHelper.cancelFiring();
}

lastFireButton = currentFireButton;
```

---

## Example Code Comparisons

### Before: Old Pattern
```java
@TeleOp(name="Old Pattern", group="Testing")
public class OldPattern extends LinearOpMode {
    @Override
    public void runOpMode() {
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        shooter = new Shooter(hardware, new ShooterConfig(), telemetry);
        shooter.enable();
        
        // OLD: 3 parameters, enable() required
        IndexingConfig indexingConfig = new IndexingConfig();
        indexingHelper = new BasicIndexingHelper(hardware, indexingConfig, telemetry);
        indexingHelper.enable();  // ❌ No longer exists
        
        firingHelper = new BasicFiringHelper(shooter, indexingHelper, hardware, telemetry);
        firingHelper.setEnabled(true);
        
        waitForStart();
        
        while (opModeIsActive()) {
            shooter.update();
            indexingHelper.update();
            firingHelper.update();
            
            // OLD: Would auto-fire continuously
            if (gamepad1.x) {
                firingHelper.startFiring(3200, "HIGH", true);
            }
        }
        
        indexingHelper.disable();  // ❌ No longer exists
    }
}
```

### After: New Pattern
```java
@TeleOp(name="New Pattern", group="Testing")
public class NewPattern extends LinearOpMode {
    private boolean lastX = false;
    
    @Override
    public void runOpMode() {
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        shooter = new Shooter(hardware, new ShooterConfig(), telemetry);
        shooter.enable();
        
        // NEW: 2 parameters, no enable() needed
        indexingHelper = new BasicIndexingHelper(hardware, telemetry);
        
        firingHelper = new BasicFiringHelper(shooter, indexingHelper, hardware, telemetry);
        firingHelper.setEnabled(true);
        
        waitForStart();
        
        while (opModeIsActive()) {
            shooter.update();
            indexingHelper.update();
            firingHelper.update();
            
            // NEW: Explicit control with fireShot()
            boolean currentX = gamepad1.x;
            if (currentX && !lastX) {
                firingHelper.startFiring(3200, "HIGH", true);  // Fires once
            }
            
            // Auto-fire while held (dead-man switch)
            if (currentX && firingHelper.isReadyForNextShot()) {
                firingHelper.fireShot();  // Explicit fire
            }
            
            // Release to stop
            if (!currentX && lastX) {
                firingHelper.cancelFiring();
            }
            
            lastX = currentX;
        }
        
        // No disable() needed
    }
}
```

---

## API Reference

### BasicIndexingHelper

#### Constructor
```java
// OLD
BasicIndexingHelper(AuroraHardwareConfig hardware, IndexingConfig config, Telemetry telemetry)

// NEW
BasicIndexingHelper(AuroraHardwareConfig hardware, Telemetry telemetry)
```

#### Removed Methods
- ❌ `enable()` - No longer needed
- ❌ `disable()` - No longer needed

#### Unchanged Methods
All motor/servo control methods remain the same:
- ✅ `setFrontRollerPower(double power)`
- ✅ `setFrontRollerTimed(double power, long durationMs)`
- ✅ `isFrontRollerBusy()`
- ✅ `update()` - Still required in loop
- ✅ ... all other control methods

### BasicFiringHelper

#### New Methods
```java
// Fire a single shot (only works in READY_TO_FIRE state)
boolean fireShot()

// Check if ready for next shot
boolean isReadyForNextShot()
```

#### Changed Behavior
```java
// startFiring() with keep-alive=true now requires explicit fireShot() calls
boolean startFiring(double rpm, String presetName, boolean keepAlive)

// Fires once, then:
// - If keepAlive=false: stops (old behavior unchanged)
// - If keepAlive=true: waits in READY_TO_FIRE for fireShot() calls (NEW)
```

#### Unchanged Methods
- ✅ `update()` - Still required in loop
- ✅ `cancelFiring()` - Still works the same
- ✅ `isFiring()` - Still works the same
- ✅ `startFiring(double rpm)` - Single shot mode unchanged
- ✅ All ejection methods unchanged

---

## Common Patterns

### Pattern 1: Single Shot (No Keep-Alive)
```java
// Unchanged - works exactly as before
if (gamepad1.a) {
    firingHelper.startFiring(3200, "HIGH", false);  // Fires once and stops
}
```

### Pattern 2: Keep-Alive with Manual Triggers
```java
if (gamepad1.x) {
    firingHelper.startFiring(3200, "HIGH", true);  // Start keep-alive
}

if (gamepad1.a && firingHelper.isReadyForNextShot()) {
    firingHelper.fireShot();  // Fire another shot
}

if (gamepad1.b) {
    firingHelper.cancelFiring();  // Stop shooter
}
```

### Pattern 3: Dead-Man Switch (Recommended)
```java
boolean trigger = gamepad1.right_trigger > 0.5;

if (trigger && !lastTrigger) {
    firingHelper.startFiring(3200, "HIGH", true);
}

if (trigger && firingHelper.isReadyForNextShot()) {
    firingHelper.fireShot();
}

if (!trigger && lastTrigger) {
    firingHelper.cancelFiring();
}

lastTrigger = trigger;
```

### Pattern 4: Rate-Limited Auto-Fire
```java
private long lastShotTime = 0;
private static final long SHOT_DELAY = 500; // 500ms between shots

// ...

if (gamepad1.right_trigger > 0.5) {
    if (!firingHelper.isFiring()) {
        firingHelper.startFiring(3200, "HIGH", true);
    }
    
    if (firingHelper.isReadyForNextShot()) {
        long now = System.currentTimeMillis();
        if (now - lastShotTime >= SHOT_DELAY) {
            firingHelper.fireShot();
            lastShotTime = now;
        }
    }
} else {
    if (firingHelper.isFiring()) {
        firingHelper.cancelFiring();
    }
}
```

---

## Troubleshooting

### Compilation Error: "cannot be applied to given types"
**Error:**
```
constructor BasicIndexingHelper in class BasicIndexingHelper cannot be applied to given types;
  required: AuroraHardwareConfig,Telemetry
  found:    AuroraHardwareConfig,IndexingConfig,Telemetry
```

**Fix:** Remove the `IndexingConfig` parameter:
```java
// Before
indexingHelper = new BasicIndexingHelper(hardware, indexingConfig, telemetry);

// After
indexingHelper = new BasicIndexingHelper(hardware, telemetry);
```

### Compilation Error: "cannot find symbol: enable()"
**Error:**
```
cannot find symbol
  symbol:   method enable()
  location: variable indexingHelper of type BasicIndexingHelper
```

**Fix:** Remove the `enable()` and `disable()` calls:
```java
// Before
indexingHelper.enable();   // Remove this
// ...
indexingHelper.disable();  // Remove this

// After
// (nothing needed)
```

### Firing Keeps Running Automatically
**Problem:** Shooter fires continuously without button presses

**Fix:** Add explicit `fireShot()` calls and dead-man switch:
```java
// Replace automatic firing with explicit control
if (firingHelper.isReadyForNextShot() && gamepad1.a) {
    firingHelper.fireShot();
}

// Add release detection to stop
if (!gamepad1.a && lastA) {
    firingHelper.cancelFiring();
}
```

---

## Testing Checklist

After migration, verify:

- [ ] Code compiles without errors
- [ ] BasicIndexingHelper works (motors/servos respond)
- [ ] Single shot mode still works (`keepAlive=false`)
- [ ] Keep-alive mode requires explicit `fireShot()` calls
- [ ] Dead-man switch stops immediately on button release
- [ ] `isReadyForNextShot()` returns true between shots
- [ ] `cancelFiring()` stops the shooter
- [ ] Telemetry shows correct states

---

## Impact Assessment

### Who Is Affected?

1. **OpModes using BasicIndexingHelper**
   - Must update constructor call
   - Must remove `enable()`/`disable()` calls
   - ⚠️ **Breaking change** - requires code modification

2. **OpModes using BasicFiringHelper with keep-alive mode**
   - Must add explicit `fireShot()` calls for follow-up shots
   - Should implement dead-man switch for safety
   - ⚠️ **Behavior change** - requires code modification

3. **OpModes using BasicFiringHelper single-shot mode**
   - ✅ **No changes needed** - compatible as-is

4. **Full IndexingSystem users**
   - ✅ **Not affected** - IndexingSystem unchanged

### Breaking vs Non-Breaking Changes

**Breaking Changes (require code updates):**
- ❌ `BasicIndexingHelper` constructor signature changed
- ❌ `BasicIndexingHelper.enable()` removed
- ❌ `BasicIndexingHelper.disable()` removed
- ❌ `BasicFiringHelper` keep-alive behavior changed

**Non-Breaking Changes (backward compatible):**
- ✅ New `fireShot()` method added
- ✅ New `isReadyForNextShot()` method added
- ✅ Single-shot mode behavior unchanged
- ✅ All IndexingHelper motor/servo methods unchanged

---

## Benefits of New Design

### 1. Simplified Initialization
- Fewer parameters to manage
- No enable/disable state to track
- Ready to use immediately

### 2. Better Control
- Explicit firing gives fine-grained control
- Dead-man switch for safety
- Rate limiting possible
- Predictable behavior

### 3. More Flexible
- Can implement various firing patterns
- Easy to add custom logic between shots
- Better suited for autonomous sequences

### 4. Safer
- Dead-man switch prevents runaway firing
- Explicit control reduces unexpected behavior
- Clear state transitions

---

## Reference OpModes

### Updated Example Files

1. **`DeadManSwitchExample.java`** ✅
   - Demonstrates dead-man switch pattern
   - Shows right/left trigger control
   - Includes rate limiting
   - **RECOMMENDED STARTING POINT**

2. **`BasicFiringHelperExample.java`** ✅
   - Shows basic keep-alive usage
   - Demonstrates explicit fire control
   - Simple button mapping

### Example OpModes Location
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/
├── DeadManSwitchExample.java        (✅ Dead-man switch pattern)
├── BasicFiringHelperExample.java    (✅ Basic keep-alive pattern)
└── [Your OpModes]                   (⚠️ Need updates)
```

---

## Quick Migration Checklist

- [ ] Find all `new BasicIndexingHelper(...)` calls
- [ ] Remove `IndexingConfig` parameter from constructor
- [ ] Remove all `indexingHelper.enable()` calls
- [ ] Remove all `indexingHelper.disable()` calls
- [ ] Find all `startFiring(..., true)` calls (keep-alive mode)
- [ ] Add `fireShot()` calls for follow-up shots
- [ ] Add `isReadyForNextShot()` checks
- [ ] Implement dead-man switch pattern (recommended)
- [ ] Test firing behavior
- [ ] Verify button release stops firing

---

## Support

If you encounter issues during migration:

1. **Check compiler errors** - Follow error messages, they point to exact issues
2. **Review example OpModes** - `DeadManSwitchExample.java` shows correct pattern
3. **Read Troubleshooting section** - Common problems and solutions above
4. **Test incrementally** - Fix one OpMode at a time
5. **Refer to API Reference** - Method signatures and behavior documented above

---

## Document History

- **January 20, 2026** - Initial migration guide created
- Covers changes to BasicIndexingHelper and BasicFiringHelper
- Documents dead-man switch implementation pattern

---

**Remember:** These changes improve safety and control. The new explicit firing pattern prevents unexpected behavior and gives you full control over when shots fire. Take time to understand the dead-man switch pattern - it's the recommended approach for competition use.
