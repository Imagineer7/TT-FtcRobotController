# Skip Color Detection Feature - Indexing System V3

## Overview

The Skip Color Detection feature allows the Indexing System V3 to collect artifacts much faster by bypassing the color detection process. When enabled, artifacts are collected immediately as `UNKNOWN` after presence is detected, reducing collection time from ~1400ms to ~200ms.

## Feature Details

### Default Behavior
- **Default State**: ON (skip color detection enabled)
- **Reason**: Prioritizes speed over color accuracy for fast-paced gameplay

### Collection Modes

#### Fast Collection Mode (Skip ON - Default)
- **Speed**: ~200ms per artifact
- **Color**: All artifacts collected as `UNKNOWN`
- **Process**: Detect presence → Wait 150ms (hardware settle) → Stop hardware → Collect as UNKNOWN
- **Use Case**: Rapid collection during matches where color doesn't matter or will be determined later

#### Full Detection Mode (Skip OFF)
- **Speed**: ~1400ms per artifact (includes jiggling for low confidence)
- **Color**: Full color classification (PURPLE, GREEN, or UNKNOWN)
- **Process**: Complete 5-state sampling machine with jiggling
- **Use Case**: When accurate color classification is required for shot planning

### States Bypassed in Fast Mode

When skip mode is ON, the following color detection states are bypassed:
1. **SETTLING** (50ms) - Artifact stabilization
2. **SAMPLING_INITIAL** (150ms) - Initial color sampling
3. **JIGGLING** (900ms) - Mechanical rotation to improve sensor view
4. **SETTLING_AFTER_JIGGLE** (50ms) - Post-jiggle stabilization
5. **SAMPLING_AFTER_JIGGLE** (150ms) - Re-sampling after jiggle

Only **WAITING_HARDWARE_DELAY** (150ms) is used in fast mode to allow artifact to settle physically.

## API Reference

### IndexingSystemV3 Methods

```java
// Set skip mode
indexing.setSkipColorDetection(true);   // Enable fast mode (skip color)
indexing.setSkipColorDetection(false);  // Enable full detection mode

// Toggle skip mode
boolean newState = indexing.toggleSkipColorDetection();

// Check current mode
boolean isSkipping = indexing.isSkipColorDetection();
```

### CollectOperation Constructor

```java
CollectOperation(
    SlotLedger ledger,
    IntakePerception perception,
    BasicIndexingHelper helper,
    IndexingConfig config,
    SlotLedger.Slot targetSlot,
    int sequenceId,
    boolean skipColorDetection,  // NEW PARAMETER
    Telemetry telemetry
)
```

## Test OpMode Controls

### IndexingSystemV3BasicTest
- **LEFT BUMPER** - Toggle fast collect mode (skip color detection)
- Status displayed on telemetry when toggled

### IndexingSystemV3AdvancedTest
- **RIGHT TRIGGER** - Toggle fast collect mode (skip color detection)
- Status displayed on telemetry when toggled

## Implementation Details

### Changes to CollectOperation.java

1. **New Field**: `skipColorDetection` (boolean)
2. **Constructor Parameter**: Added `skipColorDetection` parameter
3. **Fast Path Logic**: 
   ```java
   if (skipColorDetection && samplingState == WAITING_HARDWARE_DELAY) {
       // Wait for hardware checkpoint (150ms)
       // Stop hardware
       // Create UNKNOWN artifact
       // Skip to COMPLETE state
   }
   ```
4. **Telemetry**: Added "Skip Color Detection" field

### Changes to IndexingSystemV3.java

1. **New State Field**: `skipColorDetection` (default: true)
2. **API Methods**: 
   - `setSkipColorDetection(boolean enabled)`
   - `toggleSkipColorDetection()`
   - `isSkipColorDetection()`
3. **requestCollect()**: Passes skip flag to CollectOperation
4. **Logging**: Debug logs for mode changes

## Performance Comparison

| Mode | Time | Color Accuracy | States Executed |
|------|------|----------------|-----------------|
| Fast (Skip ON) | ~200ms | None (UNKNOWN) | 1 state (WAITING_HARDWARE_DELAY) |
| Full (Skip OFF) | ~400-1400ms | High (with jiggle) | 5-6 states (complete sampling) |

**Speed Improvement**: 85% faster (200ms vs 1400ms for full detection with jiggle)

## Use Cases

### When to Enable Skip Mode (Default)
- ✅ Autonomous period (collect as many as possible)
- ✅ Rapid collection phase
- ✅ When color classification happens at transfer checkpoint
- ✅ When motif pattern isn't critical
- ✅ Hunt mode with auto-collection

### When to Disable Skip Mode
- ❌ Need accurate color at collection point
- ❌ Testing color sensor accuracy
- ❌ Debugging color classification issues
- ❌ Shot planning requires color before transfer

## Testing Recommendations

### Functional Testing
1. **Enable Fast Mode**:
   - Press LEFT BUMPER (BasicTest) or RIGHT TRIGGER (AdvancedTest)
   - Collect artifacts using gamepad A (FRONT) or B (BACK)
   - Verify artifacts appear as UNKNOWN in slot ledger
   - Verify collection completes in ~200ms

2. **Disable Fast Mode**:
   - Toggle mode again
   - Collect artifacts
   - Verify artifacts have color classification (PURPLE/GREEN/UNKNOWN)
   - Verify collection takes longer (~400-1400ms)

3. **Toggle During Operation**:
   - Test mode changes during active collection
   - Verify system handles mode changes gracefully

### Performance Testing
1. Time 10 collections in Fast Mode → Average should be ~200ms
2. Time 10 collections in Full Mode → Average should be 400-1400ms
3. Calculate throughput improvement

### Integration Testing
1. **With Hunt Mode**:
   - Enable both hunt and fast collect
   - Verify auto-collections are rapid
   
2. **With Shot Planning**:
   - Test with motif patterns (PPG, PGP, GPP)
   - Verify UNKNOWN artifacts can still be used for shots
   - Check if color is updated at transfer checkpoint

3. **With Burst Firing**:
   - Collect multiple artifacts in fast mode
   - Fire rapid shots
   - Verify system keeps up with fast collection rate

## Troubleshooting

### Artifacts Still Taking Long Time
- Check if skip mode is actually enabled: `indexing.isSkipColorDetection()`
- Verify no other operations blocking collection
- Check hardware delays in IndexingConfig

### Color Detection Not Working When Disabled
- Verify sensors are functional (color sensor test)
- Check IntakePerception sensor fusion
- Review color checkpoint policy

### Mode Toggle Not Working
- Check button edge detection logic
- Verify telemetry shows mode changes
- Check debug logs (LogGroup.INTAKE)

## Future Enhancements

Potential improvements for future versions:
1. **Hybrid Mode**: Fast collection + color update at transfer checkpoint
2. **Confidence Threshold**: Skip only if initial confidence is very low
3. **Adaptive Mode**: Auto-switch based on time constraints
4. **Per-Intake Settings**: Different modes for FRONT and BACK intakes

## Related Documentation

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/README.md` - V3 System Overview
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/v3/COLOR_CHECKPOINT_POLICY.md` - Color Detection Details
- `FIRING_SYSTEM_USAGE.md` - Integration with firing system

## Version History

- **v3.1** (2026-01-23): Initial implementation of skip color detection feature
  - Added skipColorDetection flag to CollectOperation
  - Added API methods to IndexingSystemV3
  - Updated test OpModes with toggle controls
  - Default: ON (skip enabled)
