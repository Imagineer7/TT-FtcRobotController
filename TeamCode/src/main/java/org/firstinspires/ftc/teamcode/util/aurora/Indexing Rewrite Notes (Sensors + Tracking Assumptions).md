# Indexing Rewrite Notes (Sensors + Tracking Assumptions)

# Indexing Rewrite Notes (Sensors + Tracking Assumptions)

## Context / Current Hardware Reality

* Sensors are **not fully reliable** for confirming artifact presence in storage because artifacts have **holes**.
* We are **not fixing** this sensor limitation in the immediate rewrite.
* Future plan: add **multiple offset sensors** so at least one will see an artifact reliably.

## Existing Low-Level Execution Helpers (Already Built + Tested)

The new indexing system’s execution layer will use existing tested helpers:

### BasicIndexingHelper

* Low-level hardware control
* Runs timed operations for indexing/collection/transfer/etc.

### BasicFiringHelper

* Non-blocking firing sequences
* Intended to handle firing actions without blocking the main loop

Design goal:

* The rewritten indexing system should treat these as the **hardware abstraction layer (HAL)** and focus higher-level logic on:

    * slot tracking
    * operations/transactions
    * shot planning integration
    * gating/safety

## Current Sensor Suite Per Intake

Each intake currently has:

### 1) Main “forward-facing” laser distance sensor

* Faces outward/in front of the intake.
* Reads **far range when empty**.
* Reads **near/shorter distance when an artifact is blocking** the intake opening.

### 2) REV 2m infrared ToF distance sensor (NOT a beam-break)

* Positioned **perpendicular** to the main forward sensor.
* Oriented so it measures distance across the intake mouth to the **opposite side wall**.
* Typical empty reading is the opposite wall distance (~25 cm).
* When an artifact enters the mouth volume, the reading usually drops (artifact interrupts the sensor’s view).
* Important: REV 2m can indicate **“something entered/left the entrance zone”**, but cannot reliably say “artifact is fully stored inside the intake.”

### 3) Color sensors (2x per intake)

* Color sensors report raw RGB values.
* If nothing is in front of the sensor, RGB values are typically low.
* We already have a score-based color classifier with:

    * predicted color class
    * confidence score

Mounting:

* **Mouth color sensor**: mounted opposite the REV 2m (REV 2m looks at this wall).
* **Outward color sensor**: mounted next to the outward-facing distance sensor, also facing outward.

## Key System Assumption (Major Design Advantage)

* **We rarely, if ever, release artifacts after collecting them.**
* When we do release artifacts, it is through an **intentional operator action** (fire/eject) and we know it’s happening.

This allows the indexing system to be **model-first**:

* Once an artifact is committed to tracking, it stays tracked until explicitly consumed.

## Core Design Rule: Model Truth vs Sensor Truth

### Model-first invariant

* After an artifact is committed into a slot, it must remain tracked until removed by an explicit action.

### Allowed ways artifacts can leave the model

Artifacts may only be removed from the internal model by:

* **FIRE_CENTER** (consumes center artifact)
* **EJECT_* operations** (explicit clear)
* **MANUAL_CLEAR / operator reset** (explicit acknowledgment)
* (Optional) **FAULT_CLEAR** only with explicit operator confirmation

### Explicit non-goal

* Sensors must **never** “auto-delete” artifacts from tracking (holes/occlusion/sensor noise).
* Sensor disagreement after commit is treated as:

    * warning/telemetry
    * confidence reduction
    * NOT an automatic state correction

## Sensor Usage Strategy (Pre-offset-sensor era)

### What sensors ARE good for

* **Entry/Boundary detection**: “something is at/entering the intake mouth.”
* Starting the “first detected → delay → confirm” workflow.
* Advisory diagnostics (jam, mouth blocked unexpectedly).

### What sensors are NOT trusted for (yet)

* Determining whether an artifact is definitely inside storage.
* Proving push/swap success with certainty.

## Recommended Sensor Fusion Concepts

### Convert sensors into zone booleans (derived signals)

Maintain derived booleans per intake:

* `frontBlocked` (from forward-facing distance sensor)
* `mouthOccupied` (from REV 2m distance sensor)

These are derived using:

* thresholding
* hysteresis (two thresholds)
* debounce (time stability)

### Add color-sensor-based presence hints (RGB + confidence)

Per intake, there are **two color sensors** that report raw RGB (low values usually mean “nothing present”):

* **Mouth color sensor**: mounted opposite the REV 2m (REV 2m looks at this wall).
* **Outward color sensor**: mounted next to the outward-facing distance sensor, also facing outward.

We already have a **score-based color classification** (purple/green) with a **confidence score**.

Use this as an additional *presence hint*:

* If `frontBlocked` is false (outward distance says empty) **but** color confidence is strongly **PURPLE or GREEN**, we may still have an artifact present (distance sensor may be missing due to geometry/holes/angles).

Important rule:

* Color confidence may **increase** presence confidence or trigger “first detect,” but should **not** be used to remove artifacts from tracking after commit.

Suggested derived signals:

* `colorSeesArtifact_outward = (colorConfidenceOutward >= threshold && colorClass in {PURPLE,GREEN})`
* `colorSeesArtifact_mouth   = (colorConfidenceMouth   >= threshold && colorClass in {PURPLE,GREEN})`

Then a combined presence hint:

* `artifactHint = frontBlocked || mouthOccupied || colorSeesArtifact_outward || colorSeesArtifact_mouth`

### Event detection (edges)

Once stable booleans exist, edges can be used:

* `mouthOccupied: false → true` = artifact entered mouth zone (candidate “first detect”)
* `mouthOccupied: true → false` = artifact left mouth zone

### Confirm presence logic

During the “100ms stabilization” window:

* Confirm still present if: `artifactHint` is true

## Future-proof Abstraction for Rewrite

Even before new sensors are added, the rewrite should expose a consistent intake interface:

* `getMouthOccupied()`
* `getFrontBlocked()`
* `getIntakeEvent()` (ENTER / EXIT / NONE)
* `getPresenceConfidence()` (LOW / MED / HIGH)

When offset sensors are added later, only the internal computation changes — not the indexing logic.

---

# Color + “What’s Where” (Shot Planning Requirements)

## Requirement

* The indexing system must always know **what artifact color/type is in each location** so the planner can build correct shot plans.

## Slot Ledger as the Single Source of Truth

Use a slot-truth model:

* `CENTER`
* `FRONT`
* `BACK`

Each slot holds an **ArtifactIdentity** (or null).

### ArtifactIdentity fields (suggested)

* `colorClass`: `GREEN | PURPLE | UNKNOWN`
* `colorConfidence`: `0..1`
* `source`: `COLOR_SENSOR | INFERRED | OPERATOR`
* `timestamp` or `sequenceId` (helps debugging)

Key principle:

* **UNKNOWN is valid** (planner can still operate safely).

## Color Checkpoints (when to classify)

Color classification should happen at reliable, repeatable checkpoints:

* During the intake “first detect → wait 100ms → confirm” window
* Immediately after transfers complete (artifact settled)

Avoid depending on continuous “live color” while artifact is moving.

## Dual Color Sensors per Intake: Fuse Observations

Per intake there are two color sensors:

* Outward color sensor (next to outward distance sensor)
* Mouth color sensor (opposite REV 2m)

Each produces a score-based classification + confidence.

Create:

* `ColorObservation outwardObs`
* `ColorObservation mouthObs`
* `bestObs = maxConfidence(outwardObs, mouthObs)` (or weighted combine)

Store `bestObs` into the slot’s ArtifactIdentity during commit.

## Post-commit rule

* Color sensing may **increase confidence** or upgrade UNKNOWN → known color.
* Avoid random color “flips” unless new confidence is significantly stronger.
* Sensors must **never remove** artifacts from tracking.

## Swap / Push rule

During Swap/Push:

* ArtifactIdentity moves with the slot (swap the identities)
* Do not depend on re-sensing to decide the new slot identity

## Planner integration: Confidence-aware planning

Shot planner should consider both:

* `colorClass`
* `colorConfidence`

Suggested safety behavior:

* Prefer firing artifacts with high confidence (e.g., >= 0.8)
* If only UNKNOWN / low-confidence artifacts exist, plan conservative actions or try to increase certainty

## Manual override support (optional but useful)

Allow an explicit operator action:

* `ManualSetSlotColor(slot, GREEN/PURPLE)`
* This should be tagged `source=OPERATOR` and logged

---

## Practical Diagnostics / Telemetry to Add

Per intake, log and display:

* raw REV 2m distance
* raw forward sensor distance
* derived `mouthOccupied` / `frontBlocked`
* derived presence confidence
* last edge timestamps (ENTER/EXIT)
* any warnings:

    * mouth blocked too long
    * front blocked but mouth not occupied (hovering / not grabbing)
    * repeated edges while rollers running (possible chatter)

## Spec Note to Carry Forward (Suggested wording)

> After an artifact is committed into a slot, it shall remain tracked until consumed by an explicit Fire/Eject operation or cleared by an explicit operator/diagnostic reset. Sensors may not remove artifacts from tracking, and sensor disagreement after commit is treated as advisory only.
