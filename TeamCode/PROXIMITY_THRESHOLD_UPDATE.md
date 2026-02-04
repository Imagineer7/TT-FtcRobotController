# Proximity Detection Threshold Update - February 3, 2026

## Summary
Updated proximity detection threshold from 5cm to 7cm across all IntakePerception sensor fusion logic.

## Change Details

### Threshold Updated
- **Old Value:** 5.0 cm
- **New Value:** 7.0 cm
- **Detection Rule:** Distance ≤ 7cm AND > 0.1cm = Artifact Detected

### Files Modified

#### 1. IntakePerception.java
**Location:** Line ~101

**Change:**
```java
// OLD
private static final double PROXIMITY_THRESHOLD_CM = 5.0;

// NEW
private static final double PROXIMITY_THRESHOLD_CM = 7.0;  // 7cm or less = artifact detected
```

**Impact:**
- Both left and right REV Color V3 proximity sensors now detect artifacts at ≤7cm
- Presence confidence calculation includes these proximity detections
- More lenient detection allows earlier artifact sensing

#### 2. SensorMappingDiagnostic.java
**Location:** Line ~345

**Change:**
Added visual detection status indicator based on 7cm threshold:
```java
// Shows proximity distance with detection status
String status = (distanceCm <= 7.0 && distanceCm > 0.1) ? " ✓ DETECTED" : " ✗ CLEAR";
telemetry.addData("  Proximity", String.format(Locale.US, "%.1f cm", distanceCm) + status);
```

**Display Example:**
- `Proximity: 4.2 cm ✓ DETECTED` (artifact present)
- `Proximity: 9.5 cm ✗ CLEAR` (no artifact)

## Presence Confidence Calculation

The presence confidence system counts detections from 5 possible sensors per intake:

1. **Confirmation sensor** (goBILDA laser distance, <10cm)
2. **Left proximity** (REV Color V3, ≤7cm) ← Updated threshold
3. **Right proximity** (REV Color V3, ≤7cm) ← Updated threshold
4. **Left color confidence** (REV Color V3 color detection)
5. **Right color confidence** (REV Color V3 color detection)

**Confidence Levels:**
- **NONE:** 0 sensors detect
- **LOW:** 1 sensor detects
- **MEDIUM:** 2 sensors detect
- **HIGH:** 3+ sensors detect

## Testing Recommendations

1. **Test detection range:**
   - Place artifact at various distances (5cm, 7cm, 9cm)
   - Verify detection triggers at 7cm
   - Verify detection clears beyond 7cm

2. **Test with SensorMappingDiagnostic:**
   - Watch proximity values and status indicators
   - Confirm ✓/✗ symbols match actual presence

3. **Test with IntakePerceptionTest:**
   - Verify presence confidence increases with proximity
   - Check that LOW/MEDIUM/HIGH levels trigger appropriately

4. **Test in full system:**
   - Verify automatic collection triggers at correct distances
   - Confirm no false positives from distant objects

## Rationale for 7cm Threshold

- **Gain increased to 150:** Better sensitivity at distance
- **Earlier detection:** Allows more time for collection logic
- **Physical robot constraints:** Matches actual sensor positioning and artifact approach angles
- **Tested observations:** Based on real sensor data showing reliable detection at 7cm

## Rollback Instructions

If 7cm threshold causes false positives:
1. Revert `PROXIMITY_THRESHOLD_CM` back to 5.0
2. Or adjust to intermediate value (e.g., 6.0)
3. Update SensorMappingDiagnostic detection status check to match

**Location:** `IntakePerception.java` line ~101
