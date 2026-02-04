# IndexingSystemV3 Performance Optimization - Test Checklist

**Date:** January 2025  
**Purpose:** Verify performance optimizations maintain functionality while improving latency

---

## Pre-Test Setup

### Hardware Requirements
- [ ] Robot fully assembled with all 65+ devices
- [ ] REV Control Hub + Expansion Hub connected
- [ ] Driver Station connected and configured
- [ ] Battery fully charged (>12.5V)
- [ ] All sensors calibrated and functional

### Software Requirements
- [ ] Latest code deployed to robot
- [ ] Performance monitoring enabled: `indexing.setPerformanceMonitoringEnabled(true)`
- [ ] Dbg logging configured (LogLevel.INFO minimum)
- [ ] Test artifacts available (purple/green samples)

---

## Phase 1: Baseline Measurement (BEFORE)

### 1.1 Performance Test (No Optimizations)

**Test:** Run PerformanceTestOpMode for 2 minutes

**Procedure:**
1. Deploy code with optimizations DISABLED (if possible) OR use previous version
2. Start "Performance Test" OpMode
3. Press START to begin 2-minute test
4. Let robot run idle (no gamepad input)
5. Record metrics from telemetry page 3

**Expected Results (Before Optimizations):**
```
Loop Time: avg=30-40ms max=80-120ms
Loop Period: avg=30-40ms
Sections:
  perception: avg=8-15ms
  indexingHelper: avg=3-8ms
  firingHelper: avg=3-10ms
  operations: avg=1-3ms
  shotPlanner: avg=1-2ms
```

**Recorded Results:**
- [ ] Loop avg: ________ms
- [ ] Loop max: ________ms
- [ ] Loop period avg: ________ms
- [ ] Perception avg: ________ms
- [ ] IndexingHelper avg: ________ms
- [ ] FiringHelper avg: ________ms

---

## Phase 2: Post-Optimization Measurement (AFTER)

### 2.1 Performance Test (With Optimizations)

**Test:** Run PerformanceTestOpMode for 2 minutes with optimizations

**Procedure:**
1. Deploy code with optimizations ENABLED (current code)
2. Start "Performance Test" OpMode
3. Press dpad_up to enable performance monitoring
4. Press START to begin 2-minute test
5. Let robot run idle (no gamepad input)
6. Record metrics from telemetry page 3

**Expected Results (After Optimizations):**
```
Loop Time: avg=20-28ms max=40-60ms (30-40% improvement)
Loop Period: avg=20-28ms
Sections:
  perception: avg=5-9ms (40% faster)
  indexingHelper: avg=2-4ms (50% faster)
  firingHelper: avg=3-10ms (unchanged - necessary PID)
  operations: avg=1-3ms (unchanged - already efficient)
  shotPlanner: avg=1-2ms (unchanged - already efficient)
```

**Recorded Results:**
- [ ] Loop avg: ________ms (Expected: 20-28ms)
- [ ] Loop max: ________ms (Expected: 40-60ms)
- [ ] Loop period avg: ________ms
- [ ] Perception avg: ________ms (Expected: 5-9ms, ~40% faster)
- [ ] IndexingHelper avg: ________ms (Expected: 2-4ms, ~50% faster)
- [ ] FiringHelper avg: ________ms

**Improvement Calculation:**
- Loop avg improvement: ______% (Target: >20%)
- Loop max improvement: ______% (Target: >30%)
- Perception improvement: ______% (Target: >30%)
- IndexingHelper improvement: ______% (Target: >40%)

**Pass Criteria:**
- [ ] Loop avg reduced by >20%
- [ ] Loop max reduced by >30%
- [ ] No increase in any section timings
- [ ] No errors in logs

---

## Phase 3: Functional Testing

### 3.1 Basic Operations Test

**Test:** All core operations work correctly

**Procedure (Hunt Mode ON):**
1. Start IndexingSystemV3BasicTest or similar OpMode
2. Enable hunt mode
3. Present artifact to front intake
4. Verify auto-collection
5. Verify auto-transfer to CENTER
6. Present artifact to back intake
7. Verify auto-collection
8. Check slot ledger shows 2 artifacts

**Pass Criteria:**
- [ ] Front intake auto-collects artifact
- [ ] Artifact auto-transfers to CENTER
- [ ] Back intake auto-collects artifact
- [ ] Ledger shows correct artifacts (2 total)
- [ ] No false detections
- [ ] No missed detections
- [ ] Collection time ~200ms (skip mode) or ~1400ms (full color)

### 3.2 Color Detection Test

**Test:** Color classification accuracy maintained

**Procedure:**
1. Set skip mode OFF: `indexing.setSkipColorDetection(false)`
2. Collect purple artifact
3. Check ledger shows PURPLE with confidence >70%
4. Collect green artifact
5. Check ledger shows GREEN with confidence >70%
6. Collect unknown artifact
7. Check ledger shows UNKNOWN

**Pass Criteria:**
- [ ] Purple detected as PURPLE (confidence >70%)
- [ ] Green detected as GREEN (confidence >70%)
- [ ] Unknown/mixed detected as UNKNOWN
- [ ] No color classification regressions
- [ ] Checkpoint sampling still works (3 checkpoints)

### 3.3 Manual Operations Test

**Test:** Manual control still responsive

**Procedure:**
1. Press gamepad button to request collect (front)
2. Verify collection starts immediately
3. Press button to request transfer
4. Verify transfer starts immediately
5. Press button to request fire
6. Verify shooter spins up and fires

**Pass Criteria:**
- [ ] Collection responds within 1 loop (~20-30ms)
- [ ] Transfer responds within 1 loop
- [ ] Fire responds within 1 loop (spinup takes normal time)
- [ ] No input lag noticeable to driver
- [ ] Immediate motor/servo response

### 3.4 Burst Firing Test (Keep-Alive)

**Test:** Keep-alive burst firing works correctly

**Procedure:**
1. Load 3 artifacts (system full)
2. Hold fire button (gamepad1.right_trigger)
3. Verify first shot fires
4. Verify shooter stays spinning
5. Verify second shot fires
6. Verify third shot fires
7. Release fire button
8. Verify shooter stops

**Pass Criteria:**
- [ ] First shot fires after RPM stable
- [ ] Shooter stays spinning between shots
- [ ] Second shot fires ~200ms after first (no spinup delay)
- [ ] Third shot fires ~200ms after second
- [ ] Shooter stops after trigger release
- [ ] Watchdog auto-cancels burst on release
- [ ] No ledger corruption (CENTER cleared after each shot)

### 3.5 Shot Planning Test

**Test:** Automatic rearrangement works

**Procedure:**
1. Set motif pattern: `indexing.setMotifPattern("PPG")`
2. Collect green artifact (goes to CENTER)
3. Collect purple artifact (goes to FRONT/BACK)
4. Check shot planner shows "Rearrangement Needed"
5. Verify automatic swap occurs (purple → CENTER, green → storage)
6. Check shot plan shows optimal order

**Pass Criteria:**
- [ ] Shot planner detects suboptimal order
- [ ] Automatic swap operation executes
- [ ] Purple artifact moves to CENTER
- [ ] Green artifact moves to storage
- [ ] Shot plan score improves
- [ ] No operation conflicts

### 3.6 Hunt Mode Stress Test

**Test:** Hunt mode with rapid artifact presentation

**Procedure:**
1. Enable hunt mode
2. Rapidly present artifacts to both intakes (alternating)
3. Verify system handles rapid collection
4. Check for false detections
5. Check for missed detections
6. Verify system fills to 3 artifacts max

**Pass Criteria:**
- [ ] System handles rapid presentation (no crashes)
- [ ] No false detections (empty → artifact)
- [ ] No missed detections (artifact → empty)
- [ ] Cooldown works (1-second between auto-collects)
- [ ] System stops at 3 artifacts (capacity)
- [ ] Hunt-eligible intakes stop when full

---

## Phase 4: Edge Cases & Safety

### 4.1 Manual Override Test

**Test:** Manual mode cancels automation

**Procedure:**
1. Start burst firing (hold trigger)
2. Activate manual mode (gamepad2 controls)
3. Verify burst cancels immediately
4. Verify manual controls work
5. Release manual mode
6. Verify automation resumes

**Pass Criteria:**
- [ ] Manual mode detected correctly
- [ ] Burst firing cancels on manual activation
- [ ] Manual controls responsive
- [ ] Automation resumes after manual release
- [ ] No watchdog false triggers

### 4.2 System Full Test

**Test:** System behavior at capacity (3 artifacts)

**Procedure:**
1. Collect 3 artifacts (system full)
2. Present 4th artifact to intake
3. Verify no collection attempt
4. Verify hunt mode stops for full intakes
5. Fire one shot
6. Verify hunt mode resumes

**Pass Criteria:**
- [ ] System correctly detects full capacity
- [ ] No collection when full
- [ ] Hunt mode disables for full intakes
- [ ] Storage intakes run at hold power
- [ ] Hunt mode resumes after space available

### 4.3 Sensor Error Handling

**Test:** Graceful degradation on sensor failure

**Procedure:**
1. Disconnect one sensor (safely)
2. Verify system continues operating
3. Check for error messages in telemetry
4. Reconnect sensor
5. Verify system recovers

**Pass Criteria:**
- [ ] No crash on sensor disconnect
- [ ] Perception uses remaining sensors
- [ ] Error logged to telemetry/Dbg
- [ ] System recovers on reconnect
- [ ] No permanent state corruption

### 4.4 Operation Timeout Test

**Test:** Timeout protection works

**Procedure:**
1. Start collection operation
2. Physically block artifact (prevent completion)
3. Wait for timeout (~3-5 seconds)
4. Verify operation times out gracefully
5. Verify system returns to IDLE

**Pass Criteria:**
- [ ] Operation times out after configured duration
- [ ] No infinite busy state
- [ ] System returns to IDLE
- [ ] Ledger not corrupted
- [ ] Next operation can start

---

## Phase 5: Regression Testing

### 5.1 Existing OpMode Compatibility

**Test:** Existing OpModes work without changes

**Procedure:**
1. Run each existing OpMode:
   - [ ] FullSystemTest
   - [ ] BasicMechanumDriveExample
   - [ ] IndexingSystemExample
   - [ ] IndexingSystemV3BasicTest
   - [ ] IndexingSystemV3AdvancedTest
2. Verify no errors
3. Verify expected behavior

**Pass Criteria:**
- [ ] All OpModes compile
- [ ] All OpModes run without crashes
- [ ] No unexpected behavior changes
- [ ] Telemetry displays correctly

### 5.2 Sensor Fusion Accuracy

**Test:** Presence detection accuracy

**Procedure:**
1. Test with different artifact types:
   - Solid artifacts (no holes)
   - Artifacts with holes
   - Partial obstructions
2. Verify confidence levels correct:
   - NONE (0 sensors)
   - LOW (1 sensor)
   - MEDIUM (2 sensors)
   - HIGH (3+ sensors)

**Pass Criteria:**
- [ ] Solid artifacts: HIGH confidence
- [ ] Holed artifacts: MEDIUM+ confidence
- [ ] Partial: LOW-MEDIUM confidence
- [ ] No false NONE readings with artifact present
- [ ] No false HIGH readings without artifact

---

## Phase 6: Driver Experience

### 6.1 Perceived Lag Test

**Test:** Driver input responsiveness

**Procedure:**
1. Have driver operate robot normally
2. Test drive controls (mecanum)
3. Test indexing controls (collect, transfer, fire)
4. Test shot planning controls
5. Ask driver: "Do you notice any lag?"

**Pass Criteria:**
- [ ] Driver reports immediate response to inputs
- [ ] No perceptible delay between button press and action
- [ ] Motors/servos respond within same loop
- [ ] Firing feels immediate (after spinup)
- [ ] Driver satisfaction: "No lag noticed"

### 6.2 Real Match Simulation

**Test:** 2-minute match scenario

**Procedure:**
1. Run full 2-minute simulated match
2. Driver performs typical match actions:
   - Collect multiple artifacts
   - Fire sequences
   - Drive movements
   - Emergency stops
3. Monitor loop times throughout
4. Check for any spikes or degradation

**Pass Criteria:**
- [ ] Loop times stay consistent throughout match
- [ ] No degradation over time
- [ ] No spikes during operations
- [ ] System remains responsive at end of match
- [ ] Driver reports smooth operation

---

## Phase 7: Final Validation

### 7.1 Performance Regression Check

**Test:** No unintended side effects

**Checklist:**
- [ ] Loop times improved (verified in Phase 2)
- [ ] All operations work (verified in Phase 3)
- [ ] Safety systems intact (verified in Phase 4)
- [ ] No new errors in logs
- [ ] No memory leaks (long-duration test)
- [ ] No thermal issues (motors/servos)

### 7.2 Code Review Checklist

**Verification:**
- [ ] Sensor caching implemented correctly
- [ ] Write-on-change implemented correctly
- [ ] No direct setPower calls bypassing cache
- [ ] Performance monitor can be disabled
- [ ] All comments accurate and up-to-date
- [ ] No debug logging left in hot paths

### 7.3 Documentation Check

**Verification:**
- [ ] PERFORMANCE_OPTIMIZATION_REPORT.md complete
- [ ] This test checklist complete
- [ ] Code comments explain optimizations
- [ ] No misleading comments
- [ ] Public API documented

---

## Test Results Summary

### Performance Improvements

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Loop Avg | ___ms | ___ms | ___%  |
| Loop Max | ___ms | ___ms | ___%  |
| Perception | ___ms | ___ms | ___%  |
| IndexingHelper | ___ms | ___ms | ___%  |

### Functional Tests

| Test | Pass | Fail | Notes |
|------|------|------|-------|
| Basic Operations | ☐ | ☐ | |
| Color Detection | ☐ | ☐ | |
| Manual Operations | ☐ | ☐ | |
| Burst Firing | ☐ | ☐ | |
| Shot Planning | ☐ | ☐ | |
| Hunt Mode | ☐ | ☐ | |
| Manual Override | ☐ | ☐ | |
| System Full | ☐ | ☐ | |
| Sensor Errors | ☐ | ☐ | |
| Operation Timeout | ☐ | ☐ | |

### Overall Assessment

**Performance Target Met:** ☐ Yes ☐ No (Target: <20ms avg loop time)

**Functional Regressions:** ☐ None ☐ Minor ☐ Major

**Ready for Competition:** ☐ Yes ☐ No ☐ With Reservations

**Issues Found:**
1. _____________________________________________________
2. _____________________________________________________
3. _____________________________________________________

**Recommendations:**
_____________________________________________________________
_____________________________________________________________
_____________________________________________________________

---

**Tested By:** _____________________  
**Date:** _____________________  
**Robot Serial:** _____________________  
**Code Version:** _____________________  

**Sign-Off:**
- [ ] Performance Engineer
- [ ] Lead Programmer
- [ ] Drive Team Captain
- [ ] Coach

