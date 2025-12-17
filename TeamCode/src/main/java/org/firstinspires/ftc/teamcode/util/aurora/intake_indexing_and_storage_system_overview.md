# Intake, Indexing, and Storage System Overview

## System Architecture

> **Critical Mechanical Constraint — Push-Based Transfers**:
> - The **central transfer wheels do not contact artifacts when they are in center storage**.
> - Artifacts can only be moved **out of center storage if another artifact physically pushes them**.
> - Therefore, **center ↔ intake transfers require a second artifact** to act as a pusher.
> - The system cannot pull an artifact out of center storage on its own.

This constraint fundamentally shapes the indexing order and explains why collection order determines storage locations.

### Dual Intake System
The robot uses **two independent intakes**:
- **Front intake** driven by a DC motor
- **Back intake** driven by a DC motor

Each intake includes a **servo-driven transfer mechanism**. The purpose of this mechanism is to move collected artifacts from the intake into the internal transfer system, ultimately feeding them into the shooter or routing them to storage within the robot.

---

### Central Transfer System
At the center of the robot is a **transfer wheel assembly** consisting of **four wheels**, divided into two synchronized sets:
- **Left-side transfer wheels** (belt-linked)
- **Right-side transfer wheels** (belt-linked)

Each side spins as a single unit, with both wheels on that side rotating in the same direction. The central transfer system serves two purposes:
1. Moving artifacts into the shooter feed position (center storage)
2. Routing artifacts through the robot to the opposite intake for temporary storage

---

## Sensor Configuration (Per Intake)
Each intake is equipped with **four sensors** to detect artifact presence and identity:
- **2× REV 2m Distance Sensors** (left and right sides)
- **1× REV Color Sensor V3** (center-facing outward to detect artifact color upon intake)
- **1× goBILDA Distance Sensor** (mounted adjacent to the color sensor, same orientation)

A combination of distance changes and color detection is used to reliably determine when an artifact has been collected.

---

## Intake and Indexing Sequence

> **Core Indexing Constraints and Invariants**:
> - Transfers are **push-based only**; center storage artifacts cannot move unless pushed by another artifact.
> - If the **front intake is being used as storage**, it cannot intake new artifacts.
> - If **center storage is occupied**, artifacts **cannot be transferred between intakes** unless a push event occurs during intake.
> - The system must index **only based on what it has already collected and where those artifacts are physically stored**.
> - The system **cannot choose the first shot once indexing is complete**.

> **Fundamental Push-Based Indexing Rule**:
> - **1st artifact collected** → placed into *center storage*
> - **2nd artifact collected** → enters center and **pushes the 1st artifact out** into the *opposite intake* for storage
> - **3rd artifact collected** → stored in the *same intake where it was collected*

Under this model, the **second artifact collected will always remain in center storage** and therefore becomes the **forced first shot once indexing is complete**.

> **Core Indexing Constraints and Invariants**:
> - If the **front intake is being used as storage**, it cannot intake new artifacts.
> - If **center storage is occupied**, artifacts **cannot be transferred between intakes** (front ↔ back).
> - The system must index **only based on what it has already collected and where those artifacts are physically stored**.
> - The system **cannot choose the first shot**. The first shot is *forced* by the order of collection.

> **Fundamental Rule of Operation**:
> - **1st artifact collected** → routed to the *opposite intake* for storage
> - **2nd artifact collected** → routed to *center storage* (**this will always be the first shot**)
> - **3rd artifact collected** → stored in the *same intake where it was collected*

The indexing system’s job is therefore **not** to optimize the first shot, but to **plan the second and third shots** using the fixed storage layout created by the intake order.

> **Important Storage Constraint**: If the **front intake is being used as storage**, it is no longer able to collect new artifacts. Therefore, once either intake enters storage mode, the **next collected artifact must be routed to center storage**, if available.

> The following sequence assumes **Indexing Mode is enabled**.

### Step 1: Artifact Detection and Classification
- Front and back intake rollers spin inward to collect artifacts.
- When an artifact is detected via sensor input (distance and/or color change):
  - The color sensor identifies the artifact color.
  - The robot evaluates:
    - The required shooting pattern (e.g., **P–P–G**, **P–G–P**, **G–P–P**)
    - What artifacts are already stored
    - The current storage locations of those artifacts

This decision-making process must complete in **under one second**.

---

### Step 2: Intelligent Artifact Routing
Based on the desired shooting pattern and current storage state, the robot routes the collected artifact to the appropriate location.

**Example Scenario**:
- Shooting pattern: **Purple – Purple – Green (P–P–G)**
- Robot initially empty

1. **Green artifact collected (front intake)**
   - Green is intended for the *last* shot.
   - Artifact is routed to the **back intake** for storage.
   - Back intake transfer servo is disabled.
   - Back intake roller runs at low speed to retain the artifact.
   - Sensors switch to *retention mode* (monitoring artifact presence rather than detecting new ones).

2. **Purple artifact collected (front intake)**
   - Purple is needed for the *first* shot.
   - Artifact is routed directly to **center storage**, ready for the shooter.

3. **Second purple artifact collected (front intake)**
   - Robot storage capacity is now full:
     - Back intake: Green
     - Center: Purple
     - Front intake: Purple
   - Front intake enters **storage mode**, similar to the back intake.

At this point, the robot is fully indexed and ready to shoot.

---

### Step 3: Shot Order Planning and Transfer Sequencing
Once the robot is full, **indexing stops**. The robot must shoot with whatever artifact is currently in the center.

The robot computes the optimal transfer sequence to fire artifacts in the correct order:

1. **First shot**
   - The purple artifact already in center storage is fired.

2. **Second shot**
   - Purple artifact stored in the front intake is transferred to center storage.
   - Second shot is fired.

3. **Third shot**
   - Green artifact stored in the back intake is transferred to center storage.
   - Third shot is fired.

This sequencing calculation must complete in **under one second**.

---

### Step 4: Shooting Execution
- Robot aligns with the goal (driver-controlled or autonomous).
- Shooter is already spun up prior to firing.
- Upon receiving the shoot command:
  1. First artifact is fired
  2. Second artifact is fired
  3. Third artifact is fired

After each shot:
- The software updates its internal record (e.g., list or array) to remove the fired artifact.
- The robot always maintains an accurate understanding of:
  - Which artifacts it has
  - Their colors
  - Their current storage locations

---

## Cycle Reset and Recovery
- Once all stored artifacts have been fired:
  - Intakes resume normal operation immediately
  - Indexing becomes active again as soon as storage locations are empty

The entire intake–index–shoot cycle then repeats.

---

## Tunable Parameters
The system must support configurable timing constants, including:
- Transfer servo activation durations
- Transfer wheel run times
- Retention roller speeds

These values should be adjustable to ensure reliable artifact movement and retention across varying conditions.

---

## Indexing Philosophy

The indexing system is intentionally **order-constrained** rather than fully flexible. This simplifies mechanical routing and improves reliability.

Key principles:
- The **first shot is deterministic** and always corresponds to the **second artifact collected**.
- Indexing logic focuses on **second- and third-shot planning only**.
- No assumptions are made about future intakes once the center is filled.
- The robot never attempts illegal transfers (intake-to-intake with center occupied).

This philosophy ensures fast (<1s), predictable behavior that respects mechanical limits.

---

## Design Goals Summary
- Deterministic and fast decision-making (< 1 second)
- Reliable artifact detection and color identification
- Flexible routing to support multiple shooting patterns
- Accurate internal state tracking
- Fully tunable for on-field optimization

---

## Additional Indexing Example Scenarios (Push-Based, Physically Valid)

All scenarios below strictly obey the **push-based indexing axiom**:
- Artifacts in **center storage cannot move unless another artifact pushes them**
- No center → intake transfer without a pusher
- Storage intakes cannot intake
- All routing emerges from physical flow, not arbitrary software choice

---

### Scenario A: Single Artifact, Early Fire

**Collection**: Front intake = **G** (robot initially empty)

1. **First artifact: Green**
   - Routed into **center storage**.

**Fire Signal Issued**:
- No second artifact exists to push.

**Action**:
1. Feed **Green** directly from center into shooter
2. Fire **Green**

**Result**:
- Valid firing with one artifact
- No transfers required

---

### Scenario B: Two Artifacts Collected, Normal Indexing

**Collection**: Front intake = **G**, Front intake = **P**

1. **First artifact: Green**
   - Placed into **center storage**.

2. **Second artifact: Purple**
   - Enters center and **pushes Green out**
   - Green routed to **back intake storage** (opposite intake)
   - Purple remains in **center storage**

**Storage Result**:
- Center: Purple (forced first shot once full)
- Back intake: Green

---

### Scenario C: Two Artifacts Collected, Early Fire

**Collection**: Front intake = **G**, Front intake = **P**

**State Before Fire**:
- Center: Purple
- Back intake: Green

**Fire Signal Issued**:
- No third artifact exists to act as a pusher

**Action**:
1. Feed **Purple** from center → shooter → fire
2. **Green cannot move** (no pusher)

**Result**:
- Only one artifact can be fired
- Remaining artifact stays stored until another artifact is collected

---

### Scenario D: Three Artifacts, Same Intake Collection

**Collection**: Front intake = **G–P–G**

1. **First: Green** → center
2. **Second: Purple** → pushes Green → back intake
3. **Third: Green** → stays in **front intake storage**

**Final Storage**:
- Center: Purple
- Front intake: Green
- Back intake: Green

**Shot Planning (Once Full)**:
1. Fire **Purple**
2. Next artifact is determined by which intake feeds center first
3. Fire last artifact

---

### Scenario E: Mixed Intake Collection (Valid)

**Collection**: Front intake = **P**, Front intake = **G**, Front intake = **G**

1. **First: Purple** → center
2. **Second: Green** → pushes Purple → back intake
3. **Third: Green** → front intake storage

**Final Storage**:
- Center: Green
- Front intake: Green
- Back intake: Purple

---

### Scenario F: Full Robot, Fire Sequence

**State**:
- Center: Artifact #2
- Front intake: Artifact #3
- Back intake: Artifact #1

**Fire Signal Issued**:
1. Feed **center artifact** into shooter → fire
2. Intake feed introduces next artifact, pushing stored artifact into center
3. Fire second
4. Repeat for third

All movement remains **push-driven** and mechanically valid.

---

These scenarios reflect the **only physically possible behaviors** of the system. Any sequence not representable here is **mechanically impossible** and therefore excluded from software handling.

