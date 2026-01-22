# Aurora Indexing System - Technical Specification v2.0

**DECODE Season 2025-2026**  
**Document Type:** Source of Truth for Expected Behavior  
**Last Updated:** January 19, 2026  
**Intended Audience:** Software Team, Future Developers, System Maintainers

---

## Document Purpose

This document describes the **intended behavior** of the Aurora Indexing System if all implementation were perfect. It serves as the authoritative reference for:
- How artifacts should move through the robot
- When operations are allowed to occur
- What guarantees each subsystem provides
- How conflicts and failures should be handled

This is **NOT** a description of current bugs or implementation quirks. This is the **design specification** that the code attempts to implement.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Key Concepts & Terminology](#2-key-concepts--terminology)
3. [Artifact State Model](#3-artifact-state-model)
4. [Hardware Architecture](#4-hardware-architecture)
5. [Primary Workflows](#5-primary-workflows)
6. [Shot Planning System](#6-shot-planning-system)
7. [Rules & Invariants](#7-rules--invariants)
8. [Concurrency, Priority, and Overrides](#8-concurrency-priority-and-overrides)
9. [Failure Handling & Recovery](#9-failure-handling--recovery)
10. [Interfaces & Contract Summary](#10-interfaces--contract-summary)
11. [Trace Examples](#11-trace-examples)

---

## 1. System Overview

### 1.1 Responsibilities

The **IndexingSystem** is responsible for:
1. **Collecting** artifacts from intakes (front and back)
2. **Transferring** artifacts to center storage
3. **Storing** artifacts in defined physical locations(the intakes can be used a stroage locations)
4. **Planning** optimal shot order based on motif patterns
5. **Pre-positioning** artifacts for rapid firing
6. **Firing** artifacts through the shooter
7. **Post-fire advancement** of next artifact to center
8. **Ejecting** artifacts when needed

### 1.2 Dependencies

**Hardware Dependencies:**
- 2 DC Motors (front/back rollers) for intake - These are the main devices for pulling artifacts into the robot from the field.
- 10 CR Servos (bottom intake, transfer, injector, uptake) - for artifact movement and positioning.
- 4 Color sensors (REV V3) for artifact detection - used to determine artifact color with a stabilization delay.
- 2 Laser distance sensors (analog) for presence detection - primary method for detecting artifact presence. These sensors face the intake openings out into the field.
- 2 REV 2m distance sensors (optional, enhanced detection) - provide additional detection capability. These sensors face perpendicularly to the laser sensors to the other side of the intakes.

**Software Dependencies:**
- **Shooter** - Flywheel subsystem providing RPM control and ready status
- **BasicIndexingHelper** - Low-level hardware control with timed operations
- **BasicFiringHelper** - Non-blocking firing sequences
- **ShotPlanner** - Pure logic for determining optimal shot order
- **PlannerExecutor** - Physical execution of rearrangements
- **FiringSequenceCoordinator** - High-level firing orchestration
- **IndexingConfig** - Centralized timing and threshold parameters

### 1.3 Integration Context

**TeleOp Mode:**
- Driver controls trigger collection/firing
- Manual override capability for direct servo control
- Real-time telemetry and status display
- Vision-based motif pattern detection

**Autonomous Mode:**
- Pre-programmed sequences
- Vision-based motif pattern detection
- Automatic shot planning and execution

---

## 2. Key Concepts & Terminology

### 2.1 Artifact

An **Artifact** is a game piece (a hallow ball like object with holes) with:
- **Color**: PURPLE, GREEN, or UNKNOWN
- **Location**: Physical position in robot (FRONT_INTAKE, BACK_INTAKE, CENTER_STORAGE, FIRED, UNKNOWN)
- **Collection Order**: Sequence number (1, 2, or 3) indicating when artifact entered system

Artifacts are **immutable** - location changes create new Artifact instances.

### 2.2 Physical Locations

```
┌─────────────────────────────────────────┐
│         SHOOTER (Flywheel)              │
└──────────────┬──────────────────────────┘
               │ Uptake Servos (feed up)
               │
      ┌────────▼────────┐
      │  CENTER STORAGE │ ← Artifact ready to fire
      └────────┬────────┘
               │ Injector Servos (push in/out)
       ┌───────┴───────┐
       │               │
┌──────▼──────┐ ┌─────▼───────┐
│ FRONT INTAKE│ │ BACK INTAKE │
│   (Storage) │ │  (Storage)  │
└─────────────┘ └─────────────┘
   │                  │
   │ Transfer Servos  │
   │                  │
┌──▼──────────┐ ┌────▼────────┐
│Front Rollers│ │Back Rollers │
│  (Collect)  │ │  (Collect)  │
└─────────────┘ └─────────────┘
```

### 2.3 System States

**SystemState Enum:**
- `IDLE` - No operations in progress, system ready
- `COLLECTING` - Actively pulling artifact into intake
- `TRANSFERRING` - Moving artifact from intake/storage to center
- `PUSHING` - Pushing first artifact to storage, second to center
- `READY_TO_FIRE` - Artifact in center, pre-positioned, ready
- `FIRING` - Actively feeding artifact to shooter
- `ERROR` - Recoverable error detected

### 2.4 "Pre-positioned"

An artifact in CENTER_STORAGE is **pre-positioned** when:
1. Uptake servos are powered **UP** (toward shooter) at a set power for a set time before stopping.
2. Artifact is positioned right before shooter flywheels
3. Pre-positioning duration has completed (300-500ms)
4. `uptakeServoPrePosCurrentArtifact` flag is `true`

**Purpose:** Reduce firing latency by eliminating uptake servo travel time.

### 2.5 "Shot Plan"

An **ordered list** of artifacts to fire, determined by:
1. Current artifact locations
2. Motif pattern (PPG, PGP, or GPP)
3. Mechanical rearrangement constraints

Updated **every loop cycle** by ShotPlanner.

### 2.6 "Busy" / Operation in Progress

`operationInProgress` flag indicates:
- System is actively executing a state machine operation
- New commands will be rejected (except manual overrides)
- Timeout protection is active
- State transitions are ongoing

### 2.7 "Centered"

An artifact is **centered** when its location is `CENTER_STORAGE`. This is the **only** location from which firing can occur.

### 2.8 Intake Modes

**COLLECTION Mode:**
- Rollers run at full power (1.0)
- Intake is actively collecting artifacts

**STORAGE Mode:**
- Rollers run at reduced power (0.65)
- Intake holds an artifact in place
- Prevents artifact from falling out

---

## 3. Artifact State Model

### 3.1 Location State Diagram

```
UNKNOWN (detected)
    │
    ├─ (intake rollers) ──→ FRONT_INTAKE or BACK_INTAKE
    │
    ├─ (transfer servos) ──→ CENTER_STORAGE
    │
    ├─ (uptake servos) ─────→ FIRED
    │
    └─ (ejection) ──────────→ FIRED
```

### 3.2 Location Transitions

| From State     | To State       | Trigger                        | Hardware Actions                          |
|----------------|----------------|--------------------------------|-------------------------------------------|
| UNKNOWN        | FRONT_INTAKE   | Color delay complete           | Rollers run                               |
| UNKNOWN        | BACK_INTAKE    | Color delay complete           | Rollers run                               |
| FRONT_INTAKE   | CENTER_STORAGE | Transfer commanded             | Transfer servos + Injector servos         |
| BACK_INTAKE    | CENTER_STORAGE | Transfer commanded             | Transfer servos + Injector servos         |
| CENTER_STORAGE | FRONT_INTAKE   | Push operation (rearrangement) | Injector reverse, transfer accept         |
| CENTER_STORAGE | BACK_INTAKE    | Push operation (rearrangement) | Injector reverse, transfer accept         |
| CENTER_STORAGE | FIRED          | Firing sequence                | Uptake servos feed after shooter is ready |
| FRONT_INTAKE   | FIRED          | Ejection                       | Rollers reverse, Transfer Reverse         |
| BACK_INTAKE    | FIRED          | Ejection                       | Rollers reverse, Transfer Reverse         |

### 3.3 How State Becomes "Truth"

The system uses a **hybrid approach**:

**Sensor-Based Detection (Initial):**
- Artifact first detected by distance sensors (< 10cm)
- Color detected after 100ms stabilization delay
- Creates artifact with UNKNOWN location initially

**Command-Based Tracking (Operational):**
- Once collection starts, location is set by software
- Hardware timing enforces physical constraints
- Sensors provide validation, not primary tracking

**Location Updates:**
- Immutable artifact pattern: `artifact.withLocation(newLocation)`
- Location changes occur at **end of timed operation**
- Storage references updated atomically with artifact list

**State Validation:**
- `artifactInCenter`, `artifactInFrontIntake`, `artifactInBackIntake` references MUST match artifact list locations
- Planner validates state consistency before planning
- Invalid states trigger fallback to default order

---

## 4. Hardware Architecture

### 4.1 Actuator Types and Roles

#### DC Motors (2)
- **Front Roller Motor** (`TopIntakeFront`)
  - Role: Pull artifacts into front intake
  - Modes: COLLECTION (1.0 power), STORAGE (0.65 power), EJECT (-0.8 power)
  
- **Back Roller Motor** (`TopIntakeBack`)
  - Role: Pull artifacts into back intake
  - Modes: COLLECTION (1.0 power), STORAGE (0.65 power), EJECT (-0.8 power)

#### CR Servos (10)

**Bottom Intake Servos (2):**
- `BottomIntakeFront`, `BottomIntakeBack`
- Role: Assist rollers in pulling artifacts
- Used during collection only

**Transfer Servos (2):**
- `TransferSystemFront`, `TransferSystemBack`
- Role: Move artifacts from intake → center storage
- Bidirectional: Normal (in), Reverse (accepts from center)

**Injector Servos (2):**
- `InjectorSystemLeft`, `InjectorSystemRight`
- Role: Complete artifact movement into center, or push out of center
- Direction depends on intake source

**Uptake Servos (2):**
- `UptakeTransferLeft`, `UptakeTransferRight`
- Role: Feed artifacts from center → shooter
- Modes: PRE-POSITION (power up), FEED (1.0 power up), RETRACT (-0.8 power down)

### 4.2 Sensor Suite

#### Color Sensors (4 REV V3)
- `ColorSensorFront`, `ColorSensorBack`, `ColorSensorLeftFront`, `ColorSensorRightBack`
- Provide normalized RGB values (0.0-1.0)
- Used for color detection with pattern-based scoring
- 100ms stabilization delay before reading

#### Laser Distance Sensors (2 Analog)
- `LaserSensorFront`, `LaserSensorBack`
- Range: 0-1000mm (0-3.3V analog)
- Threshold: < 100mm indicates artifact presence
- Primary detection mechanism

#### REV 2m Distance Sensors (2, Optional)
- `DistSensorLeftFront`, `DistSensorRightBack`
- Enhanced detection capability
- Baseline: 25cm, Threshold: 18cm
- Weight: 40% (combined with laser 60%)

### 4.3 Timing Constants (from IndexingConfig)

| Operation                  | Duration (ms) | Purpose                                    |
|----------------------------|---------------|--------------------------------------------|
| Intake Roller Time         | 800-Infinity  | Collect artifact into intake               |
| Transfer Servo Time        | 1200          | Move artifact toward center                |
| Center Accept Time         | 800           | Injector completes centering               |
| Push Time                  | 2500          | Push artifact to opposite intake           |
| Fire Feed Time             | 300           | Uptake feed duration                       |
| Pre-position Time          | 300-500       | Uptake pre-positioning                     |
| Uptake Retract Time        | 400           | Retract before push                        |
| Color Detection Delay      | 100           | Sensor stabilization                       |
| Operation Timeout          | 4000          | Maximum time for any operation             |

---

## 5. Primary Workflows

### 5.1 Collection Workflow

**Trigger:** Sensor detects artifact presence (distance < 10cm)

**Steps:**

1. **Initial Detection** (`IDLE` state)
   - Distance sensor triggers
   - Start 100ms color stabilization delay
   - Store pending artifact with detected color

2. **Delay Completion** (`IDLE` → `COLLECTING`)
   - After 100ms, confirm artifact still present
   - If absent, cancel pending artifact
   - If present, start collection

3. **Collection** (`COLLECTING` state)
   - Set intake to COLLECTION mode (full power)
   - Start BasicIndexingHelper transfer sequence(only if it's the first artifact)
   - Duration: 800ms

4. **Collection Complete** (`COLLECTING` → next state)
   - Determine artifact number (1st, 2nd, or 3rd)
   - Route to appropriate next workflow

**Branching Based on Artifact Count:**

**1st Artifact:**
```
COLLECTING → TRANSFERRING → CENTER_STORAGE → READY_TO_FIRE
```

**2nd Artifact (Manual Push Mode OFF, and if the first artifact is not the desired first shot):**
```
COLLECTING → PUSHING → [1st to opposite intake, 2nd to center] → READY_TO_FIRE
```

**2nd Artifact (Manual Push Mode ON):**
```
COLLECTING → STORAGE in collection intake → READY_TO_FIRE
```

**3rd Artifact:**
```
COLLECTING → STORAGE in collection intake → READY_TO_FIRE
```

### 5.2 Transfer to Center Workflow

**Trigger:** `startTransferToCenter(artifact)` called

**Preconditions:**
- Artifact is in FRONT_INTAKE or BACK_INTAKE location
- Center slot is empty OR this is a post-fire transfer
- System is not busy with another operation

**Steps:**

1. **Start Transfer** (`→ TRANSFERRING`)
   - Mark operation in progress
   - Start operation timer
   - Track which artifact is being transferred
   - Execute transfer hardware sequence

2. **Hardware Sequence** (via BasicIndexingHelper)
   - **Phase 1 (0-1200ms):** Transfer servos move artifact toward center
   - **Phase 2 (1200-2000ms):** Injector servos complete centering
   - Rollers may run to assist

3. **Transfer Complete** (`TRANSFERRING → READY_TO_FIRE`)
   - Update artifact location to CENTER_STORAGE
   - Set `artifactInCenter` reference
   - Clear source intake reference
   - Stop all transfer/injector servos
   - Reset uptake pre-position flag (new artifact needs positioning)
   - Update intake modes (source now in COLLECTION mode)

4. **Post-Transfer Actions**
   - If first artifact: Increment collection order counter
   - If post-fire transfer: Do NOT increment counter
   - State → READY_TO_FIRE

**MUST Guarantee:**
- After completion, exactly one artifact is in CENTER_STORAGE
- All servos are stopped
- operationInProgress flag is cleared

### 5.3 Push Operation Workflow (2nd Artifact Indexing)

**Trigger:** 2nd artifact collected, manual push mode OFF

**Purpose:** Move current center artifact to opposite intake, move desired artifact to center, but only if rearrangement is needed (current artifact is not desired first shot).

**Preconditions:**
- Exactly 2 artifacts in system
- 1st artifact in CENTER_STORAGE
- 2nd artifact just collected
- Manual push mode is OFF

**Implementation Strategy:**
A push operation requires TWO **simultaneous** `BasicIndexingHelper` calls with specific timing:
1. **Transfer to Center** - Move desired artifact from its intake into center (duration: ~2500ms default)
2. **Accept from Center** - Timed transfer servo on opposite intake to pull the pushed artifact in (duration: slightly longer than transfer, ~2600-2700ms)

The **desired artifact is determined by the shot planner** - it's the next artifact in the optimal shot sequence. The opposite intake is where the current center artifact will be pushed. Both operations must start at the exact same time to ensure perfect synchronization.

**Steps:**

1. **Start Push** (`→ PUSHING`)
   - Mark push operation in progress
   - Determine which intake contains the desired artifact (based on shot plan)
   - Identify opposite intake (where current center artifact will be pushed)
   - **IMMEDIATELY** update storage references:
     - Move current center artifact to opposite intake (software model)
     - Move desired artifact from its intake to center (software model)
     - Update intake modes to STORAGE mode (prevent false detection)

2. **Execute Both Calls Simultaneously**
   - **Call 1: Transfer to Center** (start first or same time as Call 2)
     - If desired artifact in FRONT: `helper.transferFrontIntakeToCenterTimed()`
     - If desired artifact in BACK: `helper.transferBackIntakeToCenterTimed()`
     - Duration: 2500ms (default, includes un-pre-position + transfer + pre-position)
   
   - **Call 2: Opposite Intake Accept** (start at exact same time as Call 1)
     - If pushed artifact going to FRONT: `helper.setFrontTransferTimed(-1.0, 2600ms)` or higher
     - If pushed artifact going to BACK: `helper.setBackTransferTimed(-1.0, 2600ms)` or higher
     - Duration: 100-200ms longer than transfer operation
     - Servo continues accepting/pulling after transfer completes to ensure artifact fully seated

3. **Monitor Completion**
   - Check `helper.isTransferActive()` in loop
   - When false, transfer operation complete
   - Accept servo may still be running (that's OK, it terminates naturally)
   - Verify pushed artifact is fully in opposite intake

4. **Complete Push** (`PUSHING → READY_TO_FIRE`)
   - Both transfer and accept operations complete
   - Desired artifact now in CENTER_STORAGE
   - Previous center artifact in opposite intake (stored)
   - Uptake servos pre-positioned for next shot
   - Clear push operation flag

**MUST Guarantee:**
- Both calls start at the exact same moment
- Opposite intake transfer servo runs 100-200ms longer than transfer operation
- Desired artifact physically moved to center
- Previous center artifact physically moved to opposite intake
- Uptake pre-positioning restored for firing
- Software state matches physical reality
- No artifacts lost or duplicated
- Perfect synchronization prevents servo conflicts

### 5.4 Pre-positioning Workflow

**Trigger:** Artifact in center, system IDLE or READY_TO_FIRE, no operations in progress, and artifact not yet pre-positioned

**Purpose:** Position artifact near shooter for rapid firing. **Automatically handled by BasicIndexingHelper during transfer and push operations**, but can be manually triggered if needed.

**Preconditions:**
- `artifactInCenter` is not null
- System state is IDLE or READY_TO_FIRE
- operationInProgress is false
- uptakeServoPrePositionedForCurrentArtifact is false

**Automatic Pre-positioning:**
- `BasicIndexingHelper.prePositionArtifacts()` is called automatically at the end of transfer and push sequences
- Duration: 600ms per BasicIndexingHelper constants (PREPOSITION_DURATION_MS)
- Power: 1.0 (full forward) per BasicIndexingHelper constants (PREPOSITION_POWER)
- No manual intervention usually required

**Manual Pre-positioning (if needed):**
- Call `helper.prePositionArtifacts()` directly
- This method:
  - Calls `helper.setUptakeTimed(1.0, 600ms)` internally
  - Runs uptake servos forward for 600ms
  - Positions artifact against flywheels for firing
  - Returns immediately (non-blocking)
- In main loop, call `helper.update()` to process timed movement
- Check completion with `helper.isUptakeBusy()`

**Steps (Manual Trigger):**

1. **Initiate Pre-positioning**
   - Verify artifact in center and system ready
   - Call `helper.prePositionArtifacts()`
   - Operation queued and non-blocking

2. **Monitor Completion**
   - Check `helper.isUptakeBusy()` in loop
   - When false, pre-positioning complete
   - Artifact positioned against flywheels

3. **Ready for Firing**
   - Artifact held in pre-positioned state by servo power
   - Can fire immediately
   - Set `uptakeServoPrePositionedForCurrentArtifact` flag in IndexingSystem

**MUST Guarantee:**
- Pre-positioning completes during transfer/push automatically (no manual call needed)
- Manual pre-positioning available via `prePositionArtifacts()` if required
- Artifact properly positioned against flywheels
- Uptake servos maintain position until next operation
- No manual timing management needed (BasicIndexingHelper handles all durations)

### 5.5 Firing Workflow

**Trigger:** `onFireSignal()` called by FiringSequenceCoordinator

**Preconditions (7 Gating Rules):**

1. ✅ `firingSequenceActive` is true (coordinator enabled)
2. ✅ No manual input detected (intelligent override check)
3. ✅ Artifact exists in center (`artifactInCenter != null`)
4. ✅ Artifact is pre-positioned (`uptakeServoPrePositionedForCurrentArtifact == true`)
5. ✅ System not busy (`operationInProgress == false`)
6. ✅ Shooter ready (`shooter.isReadyToFire() == true`)
7. ✅ PlannerExecutor not busy (`plannerExecutor.isBusy() == false`)

**Steps:**

1. **Start Firing** (`→ FIRING`)
   - Mark operation in progress
   - Clear pre-position flags
   - Delegate to BasicFiringHelper
   - BasicFiringHelper.startFiring(targetRPM, "AUTO", false)

2. **BasicFiringHelper Execution**
   - Verify shooter at target RPM (already done by gating rule 6)
   - Power uptake servos at full power (1.0)
   - Duration: 300ms
   - Auto-stop after feeding

3. **Monitor Completion** (`FIRING` state update loop)
   - Check `firingHelper.isFiring()` each loop
   - When false: Firing complete

4. **Complete Firing** (`FIRING → post-fire`)
   - Consume fired artifact:
     - Remove from artifacts list
     - Clear `artifactInCenter`
   - Attempt post-fire advancement

5. **Post-Fire Advancement**
   - Check for remaining artifacts in storage
   - If found: Transfer next to center
   - If none: Reset system to IDLE

**MUST Guarantee:**
- Artifact successfully fired (removed from tracking)
- Shooter maintains RPM throughout
- Next artifact advanced if available
- System ready for next shot or collection

### 5.6 Post-Fire Advancement Workflow

**Trigger:** Firing complete, center is empty

**Purpose:** Automatically advance next artifact to center

**Decision Tree:**

```
Are there artifacts remaining?
│
├─ NO → Reset system to IDLE
│        - Clear all state
│        - Reset collection counter
│        - Disable auto-detection temporarily
│        - Ready for new collection cycle
│
└─ YES → Determine source and transfer
         │
         ├─ Front intake has desired next artifact?
         │   └─ Transfer from front → center
         │
         ├─ Back intake has desired next artifact?
         │   └─ Transfer from back → center
         │
         └─ None in storage?
                └─ Stay in READY_TO_FIRE
                   (artifact already centered)
```

**Steps:**

1. **Check Inventory**
   - Query `artifactInFrontIntake`
   - Query `artifactInBackIntake`

2. **Select Source**
   - Prefer front intake (arbitrary, consistent ordering) unless following shot plan
   - Fall back to back intake

3. **Execute Transfer**
   - Mark as post-fire transfer (don't increment counter)
   - Use standard transfer workflow
   - Result: Next artifact centered and pre-positioned

**MUST Guarantee:**
- Only transfer if source artifact exists
- Don't increment collection counter (already collected)
- System returns to READY_TO_FIRE with artifact centered

---

## 6. Shot Planning System

### 6.1 Architecture

**Two-Component Design:**

1. **ShotPlanner** (Pure Logic)
   - Runs **every loop cycle**
   - Determines WHAT to do
   - Outputs desired state (shot plan)
   - Does NOT touch hardware

2. **PlannerExecutor** (Physical Execution)
   - Executes **when idle**
   - Determines WHEN/HOW to do it
   - Respects hardware state
   - Enforces timeouts

**Separation Rationale:**
- Planning can run continuously without hardware risk
- Execution respects ongoing operations
- Planner can update plan during execution
- Executor can fail safely without breaking planning

### 6.2 Shot Planning Algorithm

**Inputs:**
- Current artifact locations (center, front, back)
- Motif pattern (PPG, PGP, GPP)
- Manual push mode flag

**Skip Conditions (Planning Disabled):**
- Manual push mode is ON
- Only 1 artifact (can't rearrange)
- 3 artifacts (mechanically locked)
- All(regardless of count) artifacts are green or purple (no benefit)

**Outputs:**
- `shotPlan` - Ordered list of artifacts to fire
- `desiredCenterArtifact` - Artifact that should be centered (if rearrangement needed)

**Algorithm Steps:**

1. **Validate State**
   - Ensure artifact references match locations
   - Count active (non-fired) artifacts

2. **Check Skip Conditions**
   - If skipped: Use default physical order
   - Return false (planning skipped)

3. **Generate Feasible Orders**
   - **1 artifact:** Only current order (no rearrangement)
   - **2 artifacts:** Current order + Swapped order
   - **3 artifacts:** Only current order (locked)

4. **Score Each Order**
   - Compare to motif pattern
   - Scoring: 1st match = +3, 2nd = +2, 3rd = +1

5. **Select Best Order**
   - Highest score wins
   - Tie-break: Prefer no rearrangement

6. **Output Results**
   - Set `shotPlan` to best order
   - If best order's 1st ≠ current center: Set `desiredCenterArtifact`

**Example:**

```
Current State:
- Center: Green #2
- Front: Purple #1
Motif: PPG (want Purple first)

Feasible Orders:
1. [Green #2, Purple #1]  → Score: 0 (no matches)
2. [Purple #1, Green #2]  → Score: 5 (Purple=+3, Green=+2)

Best: Order 2
Rearrangement Needed: Yes
desiredCenterArtifact: Purple #1
```

### 6.3 Rearrangement Execution

**Conditions for Execution:**
- PlannerExecutor is IDLE
- Exactly 2 artifacts
- Manual push mode OFF
- ShotPlanner output: `desiredCenterArtifact != null`
- Current center ≠ desired center

**Execution Steps:**

1. **Request Rearrangement**
   - Planner calls `executor.requestRearrangement(desiredCenter)`
   - Executor stores pending request if idle

2. **IndexingSystem Checks Execution**
   - Each loop: Check `executor.getPendingDesiredCenter()`
   - Validate: Not busy, 2 artifacts, not manual mode, READY_TO_FIRE

3. **Execute Rearrangement**
   - Call `executor.startRearrangement()`
   - Perform **manual push operation**:
     - Push current center → opposite intake
     - Pull desired artifact → center
   - On completion: Call `executor.completeRearrangement()`

4. **Timeout Enforcement**
   - Max time: 4000ms (from IndexingConfig)
   - If timeout: `executor.abortOperation()`
   - Failed executor enters FAILED state
   - Rearrangement lockout enabled until artifact count changes

**MUST Guarantee:**
- Rearrangement only occurs when mechanically possible
- Timeout prevents infinite operations
- Failed rearrangement doesn't break system
- Lockout prevents repeated failures

---

## 7. Rules & Invariants

### 7.1 Hard Safety Rules (MUST NEVER Violate)

#### R1: Maximum Artifact Count
**MUST:** System never tracks more than 3 artifacts simultaneously  
**Enforcement:** Reject new detections if `getArtifactCount() >= 3`

#### R2: Unique Center Slot
**MUST:** At most one artifact in CENTER_STORAGE at any time  
**Enforcement:** Transfer only when center empty OR post-fire transfer

#### R3: No Firing Without Pre-positioning
**MUST:** Never fire unless `uptakeServoPrePositionedForCurrentArtifact == true`  
**Enforcement:** Gating rule #4 in `onFireSignal()`

#### R4: No Pre-positioning During Operations
**MUST:** Never start pre-positioning while `operationInProgress == true`  
**Enforcement:** State machine checks before starting pre-position

#### R5: Uptake Retraction Before Push
**MUST:** Always retract uptake servos before push operation  
**Enforcement:** `startSecondArtifactIndexing()` calls `retractUptakeServos()` immediately

#### R6: Shooter RPM Ready Before Firing
**MUST:** Shooter at target RPM and stabilized before firing  
**Enforcement:** Gating rule #6, checks `shooter.isReadyToFire()`

#### R7: No Collection During Firing
**MUST:** Never start collection while system is FIRING  
**Enforcement:** operationInProgress flag blocks new collection

### 7.2 Mechanical Constraints

#### M1: Push-Based Mechanics
**Physical Reality:** First artifact in center cannot be removed except by:
1. Firing (uptake servos feed up)
2. Push operation (second artifact pushes first out horizontally)
3. Ejection (reverse all systems)

#### M2: Storage Capacity
**Physical Reality:** Each intake can hold maximum 1 artifact

#### M3: Rearrangement Limits
**Physical Reality:**
- 1 artifact: No rearrangement possible (nothing to swap)
- 2 artifacts: Rearrangement possible (use empty intake as intermediate)
- 3 artifacts: No rearrangement possible (no empty intake)

### 7.3 State Consistency Rules

#### S1: Location Reference Consistency
**MUST:** Storage references match artifact list locations  
**Validation:** ShotPlanner validates before planning  
**Example:**
```java
for (Artifact a : artifacts) {
    if (a.getLocation() == CENTER_STORAGE) {
        assert a == artifactInCenter;
    }
}
```

#### S2: Collection Order Uniqueness
**MUST:** Each artifact has unique collection order (1, 2, or 3)  
**Enforcement:** `nextCollectionOrder` increments after each collection

#### S3: Fired Artifacts Excluded
**MUST:** Artifacts with location FIRED are excluded from counts and planning  
**Implementation:** `getArtifactCount()` filters by location != FIRED

### 7.4 Timing Invariants

#### T1: Operation Timeout
**MUST:** Every operation completes or times out within 4000ms  
**Enforcement:** `updatePushing()`, `updateTransferring()`, etc. check timeout

#### T2: Pre-positioning Duration
**MUST:** Pre-positioning runs for 300-500ms, then completes  
**Enforcement:** `updateUptakeServoTimeout()` stops servos after duration

#### T3: Firing Duration
**MUST:** Firing sequence completes within 300ms feed time  
**Enforcement:** BasicFiringHelper times uptake servo feed

### 7.5 Mutual Exclusion Rules

#### E1: Single Operation at a Time
**MUST:** Only one operation (collect, transfer, push, fire) active at a time  
**Enforcement:** `operationInProgress` flag

#### E2: Planner vs. Executor
**MUST:** Executor only runs when idle, not during other operations  
**Enforcement:** `canExecuteRearrangement()` checks operationInProgress

#### E3: Manual Override Priority
**MUST:** Manual input cancels automated operations  
**Enforcement:** Firing cancellation check runs before state machine

---

## 8. Concurrency, Priority, and Overrides

### 8.1 Operation Priority Hierarchy

**Priority Order (Highest to Lowest):**

1. **Emergency Stop** - Immediate halt of all systems
2. **Manual Override** - Driver direct control
3. **Firing Cancellation** - Mid-fire abort
4. **Active Operation** - Ongoing state machine operation
5. **Rearrangement** - Automated optimization
6. **Pre-positioning** - Background readiness
7. **Auto-Detection** - Sensor-triggered collection

### 8.2 Manual Override Behavior

**Detection Method:**
```java
isManualInputActive() {
    if (manualInputDetector != null) {
        return manualInputDetector.get();  // Custom logic
    }
    return false;  // No manual input
}
```

**OpMode Provides Detector:**
```java
indexingSystem.setManualInputDetector(() -> 
    gamepad2.dpad_up || gamepad2.dpad_down  // Only relevant controls
);
```

**Override Actions:**

**During TRANSFERRING:**
- Cancel transfer immediately
- Return to READY_TO_FIRE
- Artifact remains in center (don't consume)

**During FIRING:**
- Cancel firing via `firingHelper.cancelFiring()`
- Return to READY_TO_FIRE
- Artifact remains in center

**During Pre-positioning:**
- Stop uptake servos
- Clear pre-position flags
- Prevent automatic restart

**MUST Guarantee:**
- Manual override always takes precedence
- System returns to safe, known state
- No artifacts are lost or duplicated

### 8.3 FiringSequenceCoordinator Integration

**Role:** High-level orchestrator between IndexingSystem and Shooter

**Responsibilities:**
1. Set `firingSequenceActive` flag (enables firing)
2. Coordinate shooter spin-up
3. Call `indexingSystem.onFireSignal()` when ready
4. Track shot counts and timing
5. Detect timeout and cancel if needed

**Communication Flow:**
```
FiringSequenceCoordinator
        ↓ (sets flag)
IndexingSystem.firingSequenceActive = true
        ↓ (enables gating rule #1)
IndexingSystem.onFireSignal() allowed
        ↓ (all gates pass)
BasicFiringHelper.startFiring()
        ↓
Uptake servos feed artifact
```

**Cancellation Protocol:**
1. Coordinator sets `firingSequenceActive = false`
2. IndexingSystem detects in `checkFiringCancellation()`
3. If mid-transfer or mid-fire: Cancel operation
4. Return to READY_TO_FIRE state

### 8.4 Conflict Resolution

**Scenario 1: Collection During Firing**
- **Conflict:** Sensor detects artifact while system is FIRING
- **Resolution:** `operationInProgress` flag blocks new collection
- **Outcome:** Detection ignored, fired artifact completes first

**Scenario 2: Manual Input During Rearrangement**
- **Conflict:** Driver takes manual control during automated push
- **Resolution:** Rearrangement continues (already committed)
- **Outcome:** Manual input takes effect after push completes
- **Rationale:** Aborting mid-push leaves artifacts in undefined locations

**Scenario 3: Multiple Rearrangement Requests**
- **Conflict:** Planner requests rearrangement while executor busy
- **Resolution:** `executor.requestRearrangement()` returns false (ignored)
- **Outcome:** Second request discarded, first completes

**Scenario 4: Timeout During Operation**
- **Conflict:** Operation exceeds 4000ms timeout
- **Resolution:** Force transition to ERROR or IDLE state
- **Outcome:** Current operation aborted, telemetry logged
- **Recovery:** System reset may be required

---

## 9. Failure Handling & Recovery

### 9.1 Sensor Disagreement

**Scenario:** Color sensor and distance sensor disagree on artifact presence

**Expected Behavior:**
1. **Color Not Detected:** Artifact assumed UNKNOWN color, collection proceeds
2. **Distance Fluctuates:** 100ms debounce delay stabilizes reading
3. **Color Changes:** Use color detected during initial 100ms window
4. **Both Fail:** Artifact rejected (not added to tracking)

**Recovery:** Operator removes artifact, system returns to IDLE

### 9.2 Artifact Missing

**Scenario:** Artifact expected in storage but sensor reports empty(should not be enforced as sensors may not detect an artifact in storage)

**Expected Behavior:**
1. **During Transfer:** Operation times out (4000ms)
2. **System Transitions:** ERROR state
3. **Telemetry:** Log "Artifact lost during transfer"
4. **Recovery:** Manual system reset (`indexingSystem.reset()`)

**Prevention:** Storage mode intake power (0.65) holds artifacts

### 9.3 Jam Detection

**Scenario:** Servo cannot complete movement (physical obstruction)

**Expected Behavior:**
1. **Timeout Trigger:** Operation exceeds 4000ms
2. **State Transition:** ERROR state
3. **All Servos Stop:** Hardware disabled
4. **Telemetry:** "Operation timeout - possible jam"

**Recovery Steps:**
1. Driver activates ejection sequence
2. All intakes reverse
3. Shooter runs low speed (1200 RPM)
4. Uptake servos push forward
5. Artifacts ejected from all locations
6. System reset after ejection

### 9.4 Timeout Handling

**Operation Timeout (4000ms):**
- Applies to: COLLECTING, TRANSFERRING, PUSHING, FIRING
- Action: Transition to ERROR state, log error
- Recovery: Requires manual reset

**Uptake Pre-position Timeout (30000ms):**
- Applies to: Continuous pre-positioning
- Action: Auto-stop uptake servos, clear flags
- Recovery: Automatic, system stays in READY_TO_FIRE

**Firing Sequence Timeout (30000ms):**
- Applies to: Overall firing sequence from FiringSequenceCoordinator
- Action: Cancel firing, stop shooter
- Recovery: Automatic, system returns to IDLE

### 9.5 Rearrangement Failure

**Scenario:** Rearrangement times out or fails

**Expected Behavior:**
1. **PlannerExecutor State:** FAILED
2. **Rearrangement Lockout:** Enabled (prevents retry)
3. **Firing Continues:** System fires artifacts in current order
4. **Lockout Reset:** When artifact count changes

**Rationale:** Failed rearrangement shouldn't prevent firing. Suboptimal shot order is better than no shots.

### 9.6 State Resynchronization

**When Needed:**
- After error recovery
- After manual intervention
- After power cycle (not persisted)

**Resync Protocol:**
1. Call `indexingSystem.reset()`
2. Clears all artifact tracking
3. Resets counters to initial state
4. Restarts hardware (rollers)
5. Enables auto-detection
6. Returns to IDLE state

**What is Lost:**
- All artifact memory
- Collection orders
- Shot plans

**What is Preserved:**
- Hardware configuration
- Timing parameters
- Motif pattern

---

## 10. Interfaces & Contract Summary

### 10.1 IndexingSystem Public API

#### Initialization & Lifecycle

```java
void enable()
```
**Contract:**
- Starts hardware operation (rollers running)
- Must be called before operations
- **Guarantees:** System ready to detect artifacts

```java
void disable()
```
**Contract:**
- Stops all motors and servos immediately
- **Guarantees:** All hardware powered off

```java
void reset()
```
**Contract:**
- Clears all state, artifacts, counters
- Restarts hardware
- **Guarantees:** System returns to initial IDLE state

#### Core Operations

```java
void update()
```
**Contract:**
- **MUST** be called every loop cycle
- Updates state machine, timers, hardware
- **Guarantees:** System progresses through states correctly

```java
boolean onArtifactFirstDetected(IntakeSource source)
```
**Contract:**
- Called when sensor first detects artifact
- Starts 100ms color stabilization delay
- **Preconditions:** Artifact count < 3, not busy
- **Returns:** true if delay started, false if rejected
- **Guarantees:** If true, collection will start after delay

```java
boolean onFireSignal()
```
**Contract:**
- Called by FiringSequenceCoordinator to fire artifact
- Checks 7 gating rules before proceeding
- **Preconditions:** See section 5.5 (7 gating rules)
- **Returns:** true if firing started, false if rejected
- **Guarantees:** If true, artifact will be fired within 300ms

```java
boolean onManualPush()
```
**Contract:**
- Manually trigger rearrangement (push operation)
- **Preconditions:** Exactly 2 artifacts, not busy, READY_TO_FIRE
- **Returns:** true if push started, false if rejected
- **Guarantees:** If true, artifacts will swap positions

#### Configuration

```java
boolean setMotifPattern(String pattern)
```
**Contract:**
- Sets shot planning pattern (PPG, PGP, GPP)
- **Preconditions:** pattern is valid
- **Returns:** true if set, false if invalid
- **Guarantees:** ShotPlanner uses new pattern next cycle

```java
void setManualInputDetector(Supplier<Boolean> detector)
```
**Contract:**
- Provides custom manual override detection
- **Preconditions:** detector is not null
- **Guarantees:** Manual input detected via detector

```java
void setFiringSequenceActive(boolean active)
```
**Contract:**
- Called by FiringSequenceCoordinator to enable/disable firing
- **Guarantees:** Gating rule #1 uses this flag

#### State Queries

```java
SystemState getCurrentState()
```
**Contract:**
- Returns current state machine state
- **Guarantees:** Accurate representation of system state

```java
int getArtifactCount()
```
**Contract:**
- Returns number of active (non-fired) artifacts
- **Guarantees:** Count ∈ [0, 3]

```java
boolean isReadyToFire()
```
**Contract:**
- Checks if artifact in center, pre-positioned, and ready
- **Returns:** true only if firing is mechanically possible
- **Guarantees:** If true, `onFireSignal()` can succeed (modulo other gates)

```java
boolean isOperationInProgress()
```
**Contract:**
- Checks if state machine is busy
- **Returns:** true if operation active
- **Guarantees:** If true, new operations will be rejected

### 10.2 ShotPlanner Interface

```java
boolean updateShotPlan(List<Artifact> artifacts, 
                       Artifact center, 
                       Artifact front, 
                       Artifact back)
```
**Contract:**
- Called every loop cycle
- Updates shot plan based on current state
- **Returns:** true if planning succeeded, false if skipped
- **Guarantees:** `shotPlan` always contains valid order

```java
List<Artifact> getShotPlan()
```
**Contract:**
- Returns current shot plan (ordered list)
- **Guarantees:** List is immutable (defensive copy)

```java
Artifact getDesiredCenterArtifact()
```
**Contract:**
- Returns artifact that should be centered for optimal order
- **Returns:** null if no rearrangement needed
- **Guarantees:** Only non-null when rearrangement is beneficial

### 10.3 PlannerExecutor Interface

```java
boolean requestRearrangement(Artifact desiredCenter)
```
**Contract:**
- Request rearrangement operation
- **Preconditions:** Executor is IDLE
- **Returns:** true if accepted, false if busy or locked out
- **Guarantees:** If true, request stored as pending

```java
boolean update(int artifactCount)
```
**Contract:**
- Called every loop to check timeout and reset lockout
- **Returns:** true if rearrangement in progress
- **Guarantees:** Timeout enforced, lockout reset on count change

```java
boolean startRearrangement()
```
**Contract:**
- Begin executing rearrangement
- **Preconditions:** Pending request exists, executor IDLE
- **Returns:** true if started
- **Guarantees:** State changes to REARRANGING

```java
void completeRearrangement()
```
**Contract:**
- Mark rearrangement successful
- **Guarantees:** Executor returns to IDLE, pending cleared

```java
void abortOperation()
```
**Contract:**
- Abort failed operation
- **Guarantees:** Executor enters FAILED, lockout enabled

### 10.4 Shooter Interface

```java
boolean isReadyToFire()
```
**Contract:**
- Checks if shooter at target RPM and stabilized
- **Returns:** true only if flywheel ready
- **Guarantees:** If true, artifact will successfully launch

```java
boolean spinUp()
```
**Contract:**
- Start spinning flywheel to target RPM
- **Returns:** true if started
- **Guarantees:** Shooter will reach target within timeout

```java
double getCurrentRPM()
```
**Contract:**
- Returns average RPM of both motors
- **Guarantees:** Real-time measurement

### 10.5 BasicFiringHelper Interface

```java
void startFiring(double targetRPM, String preset, boolean keepAlive)
```
**Contract:**
- Start firing sequence (verify RPM → feed → complete)
- **Preconditions:** Shooter enabled, target RPM set
- **Guarantees:** Feed servos powered for 300ms

```java
boolean isFiring()
```
**Contract:**
- Check if firing sequence active
- **Returns:** true while feeding
- **Guarantees:** Returns false when feeding complete

```java
void cancelFiring()
```
**Contract:**
- Abort active firing sequence
- **Guarantees:** Servos stopped immediately

### 10.6 FiringSequenceCoordinator Interface

```java
boolean canStartFiring()
```
**Contract:**
- Comprehensive readiness check
- Checks: artifacts, shooter ready, indexing ready, not busy
- **Returns:** true only if all conditions met
- **Guarantees:** If true, `startFiring()` will succeed

```java
boolean startFiring()
```
**Contract:**
- Initiate automated firing sequence
- Sets `firingSequenceActive` flag
- **Returns:** true if started
- **Guarantees:** Fires all artifacts or times out

```java
void update()
```
**Contract:**
- Coordinate shooter and indexing
- Call `indexingSystem.onFireSignal()` when ready
- **Guarantees:** Progression through sequence or timeout

```java
void cancelFiring()
```
**Contract:**
- Stop firing sequence
- Clears `firingSequenceActive` flag
- **Guarantees:** IndexingSystem receives cancellation signal

---

## 11. Trace Examples

### 11.1 Normal Collection and Fire (Single Artifact)

**Initial State:**
```
Artifacts: []
Center: null
Front: null
Back: null
State: IDLE
```

**Sequence:**

```
T+0ms: Sensor detects artifact at front intake
       → onArtifactFirstDetected(FRONT) called
       → 100ms color delay starts
       → frontPendingArtifact = Artifact{color=PURPLE, order=1}

T+100ms: Delay complete, artifact still present
         → onArtifactDetected() called internally
         → State: IDLE → COLLECTING
         → operationInProgress = true
         → Rollers + servos activated
         → artifacts.add(PURPLE #1)

T+900ms: Collection complete (800ms elapsed)
         → State: COLLECTING → TRANSFERRING
         → Transfer sequence starts

T+2900ms: Transfer complete (2000ms elapsed)
          → PURPLE #1 moved to CENTER_STORAGE
          → artifactInCenter = PURPLE #1
          → State: TRANSFERRING → READY_TO_FIRE
          → operationInProgress = false

T+2900ms: Pre-positioning starts (automatic)
          → Uptake servos powered UP (0.35)
          → uptakeServoPrePositioned = true

T+3400ms: Pre-positioning complete (500ms elapsed)
          → uptakeServoPrePositionedForCurrentArtifact = true
          → System ready to fire

T+5000ms: FiringSequenceCoordinator.startFiring() called
          → firingSequenceActive = true
          → Shooter already at 3200 RPM (spun up earlier)

T+5010ms: canStartFiring() returns true
          → Calls indexingSystem.onFireSignal()
          → All 7 gating rules pass ✓
          → State: READY_TO_FIRE → FIRING
          → BasicFiringHelper.startFiring(3200, "AUTO", false)

T+5010ms: Uptake servos feed at full power (1.0)

T+5310ms: Feed complete (300ms elapsed)
          → BasicFiringHelper.isFiring() returns false
          → State: FIRING → completeFiringOperation()
          → PURPLE #1 marked as FIRED
          → artifacts.remove(PURPLE #1)
          → artifactInCenter = null

T+5310ms: Post-fire advancement
          → No artifacts remaining
          → resetAfterFiringComplete() called
          → State: → IDLE
          → nextCollectionOrder = 1
          → Ready for new collection
```

**Final State:**
```
Artifacts: []
Center: null
State: IDLE
```

### 11.2 Rearrangement: Wrong Order to Optimal Order

**Initial State:**
```
Artifacts: [GREEN #1 (CENTER), PURPLE #2 (FRONT)]
Motif Pattern: PPG (want Purple first)
Center: GREEN #1
Front: PURPLE #2
Back: null
State: READY_TO_FIRE
Manual Push Mode: OFF
```

**Sequence:**

```
T+0ms: ShotPlanner.updateShotPlan() runs (every loop)
       → Active artifacts: 2
       → Skip conditions: None apply
       → Generate feasible orders:
         1. [GREEN #1, PURPLE #2] (current)
         2. [PURPLE #2, GREEN #1] (swapped)
       → Score orders:
         1. Score = 0 (no matches to PPG)
         2. Score = 5 (P match=+3, P skip, G match=+2)
       → Best order: [PURPLE #2, GREEN #1]
       → Rearrangement needed: YES
       → desiredCenterArtifact = PURPLE #2

T+10ms: PlannerExecutor.requestRearrangement(PURPLE #2)
        → Executor state: IDLE ✓
        → Request accepted
        → pendingDesiredCenter = PURPLE #2

T+20ms: IndexingSystem.updatePlannerExecutor()
        → Check conditions:
          - artifactCount == 2 ✓
          - executor.getPendingDesiredCenter() != null ✓
          - executor.isIdle() ✓
          - operationInProgress == false ✓
          - manualPushMode == false ✓
          - currentState == READY_TO_FIRE ✓
        → Conditions met, execute rearrangement

T+20ms: executeRearrangement() called
        → executor.startRearrangement()
        → Executor state: IDLE → REARRANGING
        → State: READY_TO_FIRE → PUSHING
        → operationInProgress = true
        → retractUptakeServos() (400ms)
        → Update storage references:
          - GREEN #1: CENTER → BACK_INTAKE (software)
          - artifactInBackIntake = GREEN #1
          - artifactInCenter = null

T+420ms: Uptake retraction complete
         → executePushHardware() called
         → Injector servos REVERSE (push GREEN out)
         → Back transfer servos ACCEPT (pull GREEN in)
         → Duration: 2500ms

T+2920ms: Push complete (2500ms elapsed)
          → completePushOperation() called
          → wasPlannedRearrangement = true
          → PURPLE #2 moved to CENTER_STORAGE
          → artifactInCenter = PURPLE #2
          → GREEN #1 in BACK_INTAKE (already set)
          → executor.completeRearrangement()
          → Executor state: REARRANGING → IDLE
          → State: PUSHING → READY_TO_FIRE
          → operationInProgress = false

T+2920ms: Pre-positioning starts
          → Uptake servos UP (0.35)
          → uptakeServoPrePositioned = true

T+3420ms: Pre-positioning complete
          → uptakeServoPrePositionedForCurrentArtifact = true
          → Ready to fire PURPLE first (optimal order)
```

**Final State:**
```
Artifacts: [PURPLE #2 (CENTER), GREEN #1 (BACK)]
Shot Plan: [PURPLE #2, GREEN #1] (matches PPG)
Center: PURPLE #2
Back: GREEN #1
State: READY_TO_FIRE
```

**Score:** First shot will be PURPLE (3 points), second GREEN (2 points) = **5 points total**

### 11.3 Jam Recovery: Timeout During Transfer

**Initial State:**
```
Artifacts: [PURPLE #1 (FRONT)]
State: TRANSFERRING
Operation Start: T+0ms
```

**Sequence:**

```
T+0ms: Transfer started (front → center)
       → Transfer servos activated
       → Injector servos activated
       → operationInProgress = true
       → operationStartTime = currentTime

T+1500ms: Normal timing would complete at 2000ms
          → Servo physically jammed (obstruction)
          → Artifact stuck in transfer mechanism

T+4000ms: Operation timeout (4000ms elapsed)
          → update() detects timeout:
            elapsed = currentTime - operationStartTime
            if (elapsed > config.getOperationTimeoutMs())
          → setError("Operation timeout during TRANSFERRING")
          → State: TRANSFERRING → ERROR
          → operationInProgress = false
          → Telemetry: "⚠️ ERROR: Operation timeout - possible jam"

T+4010ms: Driver sees error on telemetry
          → Manually inspects robot
          → Physical obstruction found and removed

T+5000ms: Driver commands ejection
          → BasicFiringHelper.startEjection()
          → Front rollers: -0.8 power (reverse)
          → Back rollers: -0.8 power (reverse)
          → Shooter: 1200 RPM (low speed)
          → Uptake servos: 1.0 power (push forward)

T+6000ms: Ejection runs for 1000ms
          → Artifact ejected from system
          → Driver stops ejection

T+6100ms: Driver commands reset
          → indexingSystem.reset()
          → Clears all artifacts
          → State: ERROR → IDLE
          → nextCollectionOrder = 1
          → Restart hardware (rollers)
          → System ready for new collection
```

**Final State:**
```
Artifacts: []
State: IDLE
Error Count: 1
Last Error: "Operation timeout during TRANSFERRING"
```

**Recovery Complete:** System functional, ready for new collection

---

## Appendices

### Appendix A: State Machine Diagram

```
┌─────┐
│IDLE │ ◄─────────────────┐
└──┬──┘                    │
   │ artifact detected     │ no artifacts after fire
   │ (after 100ms delay)   │
   ▼                       │
┌──────────┐               │
│COLLECTING│               │
└─────┬────┘               │
      │ collection complete│
      │ (800ms)            │
      ▼                    │
┌─────────────┐            │
│TRANSFERRING │            │
└──────┬──────┘            │
       │ transfer complete │
       │ (2000ms)          │
       ▼                   │
 ┌────────────┐            │
 │  PUSHING   │ ◄──┐       │
 └─────┬──────┘    │       │
       │ push      │ 2nd   │
       │ complete  │ artifact
       │ (2500ms)  │       │
       ▼           │       │
┌──────────────┐   │       │
│READY_TO_FIRE │───┘       │
└──────┬───────┘           │
       │ onFireSignal()    │
       │ (7 gates pass)    │
       ▼                   │
   ┌───────┐               │
   │FIRING │               │
   └───┬───┘               │
       │ feed complete     │
       │ (300ms)           │
       │                   │
       ├─ artifacts remain → back to READY_TO_FIRE
       │                     (post-fire transfer)
       │
       └─ no artifacts ─────┘
```

### Appendix B: Hardware Timing Summary

| Hardware Component              | Action           | Duration (ms) | Power       |
|---------------------------------|------------------|---------------|-------------|
| Front/Back Roller Motors        | Collection       | 800           | 1.0         |
| Front/Back Roller Motors        | Storage Hold     | Continuous    | 0.65        |
| Front/Back Roller Motors        | Ejection         | 800           | -0.8        |
| Bottom Intake Servos            | Assist           | 800           | 1.0         |
| Transfer Servos                 | To Center        | 1200          | -1.0        |
| Transfer Servos                 | Accept from Ctr  | 2500          | 1.0         |
| Injector Servos                 | Complete Center  | 800           | varies      |
| Injector Servos                 | Push Out         | 2500          | reverse     |
| Uptake Servos                   | Pre-position     | 300-500       | 0.35        |
| Uptake Servos                   | Feed (Fire)      | 300           | 1.0         |
| Uptake Servos                   | Retract          | 400           | -0.8        |
| Uptake Servos                   | Eject            | 1000          | 1.0         |

### Appendix C: Gating Rules Quick Reference

**7 Gating Rules for Firing (`onFireSignal()`):**

1. ✅ `firingSequenceActive == true`
2. ✅ `isManualInputActive() == false`
3. ✅ `artifactInCenter != null`
4. ✅ `uptakeServoPrePositionedForCurrentArtifact == true`
5. ✅ `operationInProgress == false`
6. ✅ `shooter.isReadyToFire() == true`
7. ✅ `plannerExecutor.isBusy() == false`

**All 7 MUST be true** for firing to proceed.

### Appendix D: Configuration Parameters

**From IndexingConfig:**
- `intakeRollerTime`: 0.8s
- `transferServoTime`: 1.2s
- `centerAcceptTime`: 0.8s
- `secondArtifactPushTime`: 2.5s
- `fireFeedTime`: 1.0s (unused, BasicFiringHelper uses 0.3s)
- `operationTimeout`: 4.0s
- `colorDetectionDelay`: 0.1s
- `artifactDetectionDistance`: 10.0 cm

**From ShooterConfig:**
- `uptakePrePositionTime`: 0.3-0.5s
- `uptakeRetractTime`: 0.4s
- `feedTime`: 0.2s (BasicFiringHelper uses 0.3s)

### Appendix E: Artifact Locations Summary

| Location         | Meaning                                      | Max Count |
|------------------|----------------------------------------------|-----------|
| UNKNOWN          | Detected but not yet located                 | N/A       |
| FRONT_INTAKE     | Stored in front intake                       | 1         |
| BACK_INTAKE      | Stored in back intake                        | 1         |
| CENTER_STORAGE   | Centered, ready to fire                      | 1         |
| FIRED            | Consumed (removed from system)               | ∞         |

**Total Active Artifacts:** ≤ 3 (excluding FIRED)

---

## Document Revision History

| Version | Date         | Author         | Changes                                    |
|---------|--------------|----------------|--------------------------------------------|
| 1.0     | 2025-01-19   | Software Team  | Initial specification based on code audit  |
| 2.0     | 2026-01-19   | AI Agent       | Comprehensive rewrite with all subsystems  |

---

## Document Status

**Status:** ✅ **APPROVED** - Source of Truth for Expected Behavior

**Next Review:** Before competition season changes or major refactor

**Contact:** FTC Team Software Lead

---

**END OF SPECIFICATION**
