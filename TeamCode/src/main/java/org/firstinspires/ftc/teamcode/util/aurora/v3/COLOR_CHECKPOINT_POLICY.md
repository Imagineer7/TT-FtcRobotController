# Color Classification Policy

## Problem

Color sensors can produce noisy readings, especially when artifacts are moving. We need a clear policy for **when** color classification happens and **how** it's enforced.

## Policy: Color Sampling Windows

Color classification is **only performed at stable checkpoints**. These are well-defined moments when:
1. The artifact is physically settled (not moving)
2. We've waited for sensor readings to stabilize
3. We commit the color reading to the artifact identity

## Checkpoint Definitions

### Checkpoint 1: Collection Confirmation Window

**When:** After initial artifact detection during collection operation

**Trigger:** `IntakePerception.getEdgeDetected()` transitions from false → true

**Window:**
1. Edge detected (fast, 30ms debounce)
2. Wait for `COLOR_CLASSIFICATION_DELAY` (default: 150ms) to let artifact settle
3. Sample color from both sensors (outward + mouth)
4. Select best confidence reading
5. Commit color on operation completion

**Code Location:** `CollectOperation.start()` and `CollectOperation.update()`

```java
// In CollectOperation
if (edgeDetected && !colorSampled) {
    if (System.currentTimeMillis() - edgeDetectTime > COLOR_CLASSIFICATION_DELAY) {
        // Sample color now - artifact has settled
        ColorClass color = perception.getBestColorClass();
        double confidence = perception.getBestColorConfidence();
        collectedArtifact = ArtifactIdentity.createFromSensor(color, confidence, sequenceId);
        colorSampled = true;
    }
}
```

**Enforcement:**
- `IntakePerception.getBestColorClass()` can be called anytime, but operations only call it during checkpoints
- Outside checkpoints, last cached color is returned (doesn't re-sample)

### Checkpoint 2: Transfer Completion

**When:** Immediately after transfer operation completes

**Trigger:** `TransferOperation.isComplete()` returns true

**Window:**
1. Transfer hardware completes (injectors/uptake finish)
2. Wait for `TRANSFER_SETTLE_DELAY` (default: 200ms) for artifact to settle in new location
3. Sample color from center color sensors
4. Update artifact identity if confidence improves
5. Commit update on next operation cycle

**Code Location:** `TransferOperation.commit()`

```java
// In TransferOperation.commit()
// Artifact has moved to center - resample color
ColorClass newColor = centerPerception.getBestColorClass();
double newConfidence = centerPerception.getBestColorConfidence();

// Update artifact with potentially improved color reading
ArtifactIdentity updated = artifact.withUpdatedColor(newColor, newConfidence, 
                                                     ClassificationSource.COLOR_SENSOR);
ledger.setCenter(updated);
```

**Enforcement:**
- Transfer operations explicitly wait for settle delay before committing
- No other code can trigger color resampling during transfer

### Checkpoint 3: Manual Operator Override

**When:** Operator explicitly sets color via manual control

**Trigger:** Gamepad input (e.g., DPAD LEFT = purple, DPAD RIGHT = green)

**Window:** Immediate (no delay needed - operator decision)

**Code Location:** `IndexingSystemV3.handleManualColorOverride()`

```java
// In IndexingSystemV3
if (gamepad.dpad_left) {
    // Operator says artifact is PURPLE
    ArtifactIdentity updated = artifact.withUpdatedColor(
        ColorClass.PURPLE, 
        1.0,  // Full confidence
        ClassificationSource.OPERATOR
    );
    ledger.set(slot, updated);  // Via controlled setter
}
```

**Enforcement:**
- Only accessible through main controller
- Uses `ClassificationSource.OPERATOR` which always wins in update policy

## Non-Checkpoints (Color NOT Sampled)

These are times when color classification is **explicitly disabled**:

1. **During hardware motion:** While rollers/servos are running
2. **Between checkpoints:** Artifact sitting in slot with committed color
3. **During rearrangement:** While swap operation is executing
4. **During firing:** While uptake is feeding artifact

## Enforcement Mechanisms

### 1. Perception Layer (IntakePerception)

```java
public class IntakePerception {
    private ColorClass cachedColorClass;
    private double cachedColorConfidence;
    private boolean samplingEnabled;  // NEW: gate for sampling
    
    /**
     * Enable color sampling (call at checkpoint start)
     */
    public void enableColorSampling() {
        samplingEnabled = true;
    }
    
    /**
     * Disable color sampling (call after checkpoint)
     */
    public void disableColorSampling() {
        samplingEnabled = false;
    }
    
    /**
     * Get best color class
     * Only re-samples if sampling is enabled, otherwise returns cached
     */
    public ColorClass getBestColorClass() {
        if (samplingEnabled) {
            // Re-sample from sensors
            updateBestColorClassification();
        }
        // Return cached value
        return cachedColorClass;
    }
}
```

### 2. Operation Layer

Operations explicitly control sampling windows:

```java
public class CollectOperation extends BaseOperation {
    @Override
    public boolean start() {
        // Disable sampling initially
        perception.disableColorSampling();
        
        // Start hardware
        helper.runXIntakeTimed(...);
        return true;
    }
    
    @Override
    public boolean update() {
        // Check if edge detected and delay passed
        if (edgeDetected && !colorSampled) {
            if (elapsedSinceEdge() > COLOR_CLASSIFICATION_DELAY) {
                // Open sampling window
                perception.enableColorSampling();
                
                // Sample NOW
                ColorClass color = perception.getBestColorClass();
                double confidence = perception.getBestColorConfidence();
                collectedArtifact = ArtifactIdentity.createFromSensor(...);
                
                // Close sampling window
                perception.disableColorSampling();
                colorSampled = true;
            }
        }
        
        return helper.isXIntakeBusy();
    }
}
```

### 3. Config Parameters

Add to `IndexingConfig.java`:

```java
/** Delay after edge detection before sampling color (milliseconds) */
private long colorClassificationDelay = 150;

/** Delay after transfer completion before sampling color (milliseconds) */
private long transferSettleDelay = 200;

/** Enable strict checkpoint enforcement (disable for debugging) */
private boolean strictCheckpointEnforcement = true;
```

## Unknown Color Handling

If color classification fails at a checkpoint:

**Policy:**
1. If no confident reading after delay → commit as `UNKNOWN`
2. `UNKNOWN` artifacts are still tracked and can fire
3. Shot planner treats `UNKNOWN` as neutral (doesn't match any pattern)
4. Next checkpoint may upgrade `UNKNOWN` → known if sensor improves

**Example:**
```java
// After delay, still no confident reading
if (confidence < UNKNOWN_UPGRADE_THRESHOLD) {
    collectedArtifact = ArtifactIdentity.createUnknown(sequenceId);
}
```

## Validation

To verify checkpoint enforcement:

1. **Telemetry logging:**
```java
telemetry.addData("Color Checkpoint", "COLLECTING (delay=" + elapsed + "ms)");
telemetry.addData("Sampling Enabled", perception.isSamplingEnabled());
```

2. **Assertion checks (debug mode):**
```java
// In perception.getBestColorClass()
if (!samplingEnabled && debugMode) {
    telemetry.addData("⚠️ WARNING", "Color sampled outside checkpoint!");
}
```

3. **Test scenarios:**
- Collect artifact → verify color sampled exactly once
- Transfer artifact → verify color resampled after settle
- Move artifact while in slot → verify color NOT resampled

## Summary

**Checkpoints (color IS sampled):**
1. Collection: after edge + delay
2. Transfer: after completion + settle
3. Manual: immediate operator override

**Non-checkpoints (color NOT sampled):**
- During motion
- Between checkpoints
- During rearrangement/firing

**Enforcement:**
- Perception layer gates sampling with `samplingEnabled` flag
- Operations explicitly open/close sampling windows
- Config parameters control delays
- Telemetry logs checkpoint events

This ensures color classification happens only when physically meaningful, preventing noise from moving artifacts.
