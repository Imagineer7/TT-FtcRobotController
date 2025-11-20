# FTC DECODE - Game Overview for Programmers

This document explains the DECODE game from a **programming and robot design** perspective. It focuses on what your code needs to handle, not just the game rules.

---

## 🎮 Game Basics

### Field Layout

```
┌─────────────────────────────────────────┐
│                                         │
│  GOAL (Blue)              GOAL (Red)    │
│  ┌─────┐                  ┌─────┐      │
│  │ C O │                  │ C O │      │  C = CLASSIFIED section
│  └─────┘                  └─────┘      │  O = OVERFLOW section
│                                         │
│         BASE (Blue)  BASE (Red)         │
│            ┌───┐      ┌───┐            │
│            │   │      │   │            │
│            └───┘      └───┘            │
│                                         │
│  RAMP PATTERN AREA                      │
│  (16 colored tiles)                     │
│                                         │
│  LAUNCH LINE ═══════════════════        │
│  (starting position)                    │
└─────────────────────────────────────────┘
```

### Key Game Elements

**ARTIFACTS** (5-inch foam balls)
- Purple for one alliance, green for the other
- Each robot starts with 2 preloaded
- More available around the field
- Soft, easy to handle with simple mechanisms

**GOAL Structures**
- Two sections: CLASSIFIED (higher points) and OVERFLOW (lower points)
- Scoring: 6 points per artifact in GOAL
- Height: ~24 inches off ground (requires shooter or lift)

**RAMP PATTERN**
- 4×4 grid of colored tiles
- Teams move tiles to match the MOTIF (pattern shown at start)
- Score multipliers for completing rows/columns
- Advanced strategy, not required for basic scoring

**BASE**
- Safe zone for parking
- Endgame bonus: 3 points for robot parked in BASE

**LAUNCH LINE**
- Starting position for autonomous
- Robot must completely leave the line for 3 points

---

## ⏱️ Match Phases

### 1. AUTONOMOUS (30 seconds)

**What Happens:**
- Robots run pre-programmed code (no driver input)
- Critical for early scoring advantage
- Sets up positioning for driver-controlled period

**Programming Tasks:**
- Navigate off LAUNCH LINE (required for 3 points)
- Shoot preloaded artifacts (12 points if both score)
- Optional: Navigate to BASE for parking setup
- Optional: Use vision to align with GOAL

**Points Available:**
- Leave LAUNCH LINE: 3 points
- 2 preloaded artifacts: 12 points
- Total: **15 points** (baseline strategy)

**Code Requirements:**
- Movement system (time-based, encoder-based, or odometry)
- Shooter control with timing
- Collision avoidance (don't hit field elements or alliance partner)

---

### 2. DRIVER-CONTROLLED (2 minutes)

**What Happens:**
- Drivers control robots with gamepads
- Main scoring period
- Strategy: cycles of intake → shoot → repeat

**Programming Tasks:**
- Smooth drive controls (mecanum or tank)
- Shooter with variable power settings
- Intake mechanism (if your robot has one)
- Indexer to hold multiple artifacts
- Telemetry to help drivers (RPM, artifact count, etc.)

**Points Available:**
- 6 points per artifact × many cycles
- Competitive robots score 50-100+ points in this phase

**Code Requirements:**
- Responsive gamepad controls
- State management (shooting vs intaking)
- Safety limits (prevent damage from overdriving)

---

### 3. ENDGAME (last 30 seconds)

**What Happens:**
- Final scoring push
- Teams position for parking bonuses
- RAMP PATTERN completion (advanced)

**Programming Tasks:**
- Quick navigation to BASE
- Final artifact launches
- Optional: Climb or park on elevated structures

**Points Available:**
- Park in BASE: 3 points
- RAMP PATTERN completion: Variable multipliers
- Last-second artifacts: 6 points each

**Code Requirements:**
- Fast, reliable movement to BASE
- Option to switch to "park mode" via gamepad button

---

## 🤖 Robot Design Considerations (For Programmers)

### Minimum Viable Robot (MVP)

**Hardware:**
- Mecanum or tank drive
- Single flywheel shooter
- Basic artifact guide/holder

**Code:**
- Drive controls (StarterDrive)
- Shooter with one preset (StarterShooter)
- Simple autonomous (StarterAutoSkeleton)

**Expected Score:** 15-30 points per match

---

### Competitive Robot

**Hardware:**
- Mecanum drive with odometry
- Dual-flywheel shooter with adjustable angle
- Powered intake (roller or claw)
- Indexer (holds 3-4 artifacts)
- Vision camera (AprilTag alignment)

**Code:**
- Advanced drive (field-relative, auto-align)
- Multi-preset shooter with distance detection
- State machine for intake → index → shoot cycles
- Vision-guided autonomous
- Pedro Pathing or custom path following

**Expected Score:** 70-120 points per match

---

## 📊 Scoring Priorities (Programming Impact)

### High-Value, Low-Complexity
1. **Leave LAUNCH LINE** (3 pts) - Drive forward 24+ inches
2. **Shoot preloaded artifacts** (12 pts) - Just needs shooter + timer

### Medium-Value, Medium-Complexity
3. **TeleOp artifact cycles** (6 pts each) - Needs intake + shooter
4. **Park in BASE** (3 pts) - Drive to zone in endgame

### High-Value, High-Complexity
5. **RAMP PATTERN** (variable) - Tile manipulation, needs arm/gripper
6. **Vision-guided shooting** - Camera, AprilTag detection, alignment
7. **Optimized autonomous** - Odometry, path planning, sensor fusion

**Recommended Focus for Beginners:** Items 1-4 above. Get consistent 30+ point matches before tackling advanced features.

---

## 🎯 Programming Challenges

### Challenge 1: Shooter Consistency
**Problem:** Artifacts don't land in GOAL consistently

**Solutions:**
- Use encoder-based RPM control (not just motor power)
- Compensate for battery voltage drop
- Add RPM telemetry to find optimal speeds
- Consider fixed shooting positions (less variability)

**Code Pattern:**
```java
// Bad: Fixed power (varies with battery)
shooter.setPower(0.8);

// Good: Target RPM with feedback
shooter.setTargetRPM(2780);
while (!shooter.isAtSpeed()) {
    shooter.update();  // PID loop adjusts power
}
```

---

### Challenge 2: Autonomous Positioning
**Problem:** Robot doesn't end up where expected

**Solutions:**
- Start with time-based movements (simple but inaccurate)
- Upgrade to encoder-based (better accuracy)
- Add IMU for heading control (prevents drift)
- Use odometry for precise positioning (GoBilda Pinpoint)

**Code Pattern:**
```java
// Time-based (beginner)
drive(0.5, 0, 0);
sleep(1000);

// Encoder-based (intermediate)
driveDistance(24.0);  // inches

// Odometry-based (advanced)
driveToPosition(36, 24, 90);  // x, y, heading
```

---

### Challenge 3: Driver Feedback
**Problem:** Drivers don't know robot state during match

**Solutions:**
- Use telemetry (Driver Station screen)
- Add gamepad rumble (requires OpMode code)
- Use LED strips (requires hardware + wiring)
- Audio cues (requires speaker)

**Code Pattern:**
```java
// Clear, organized telemetry
telemetry.addLine("━━━━ SHOOTER ━━━━");
telemetry.addData("Status", shooter.isAtSpeed() ? "✅ READY" : "⏳ Spinning");
telemetry.addData("RPM", "%.0f / %.0f", currentRPM, targetRPM);
telemetry.update();

// Gamepad rumble feedback
if (shooter.isAtSpeed()) {
    gamepad1.rumble(200);  // Short pulse when ready
}
```

---

## 🔄 Typical Match Flow (Code Perspective)

### Autonomous Phase (30s)
```
Init → waitForStart()
  ↓
Drive off LAUNCH LINE (3 pts)
  ↓
Spin up shooter
  ↓
Fire artifact 1 (6 pts)
  ↓
Fire artifact 2 (6 pts)
  ↓
[Optional] Drive toward BASE
  ↓
Stop motors, end autonomous
```

### Driver-Controlled Phase (2min)
```
Loop while opModeIsActive():
  ↓
Read gamepad inputs
  ↓
Update drive system
  ↓
Update shooter system
  ↓
[If has intake] Update intake
  ↓
Display telemetry
  ↓
Check for endgame (timer > 90s)
```

### Endgame (last 30s)
```
If timer > 90s:
  ↓
Flash LED / Rumble gamepad (alert driver)
  ↓
[Optional] Auto-navigate to BASE
  ↓
Park in BASE (3 pts)
  ↓
Match ends at 120s
```

---

## 📐 Field Measurements (For Navigation)

| Element | Dimension | Notes |
|---------|-----------|-------|
| Field | 12 ft × 12 ft | 144 in × 144 in |
| LAUNCH LINE to GOAL | ~36-48 in | Varies by starting position |
| GOAL height | ~24 in | Plan shooter angle accordingly |
| BASE zone | ~24 in × 24 in | Parking area |
| ARTIFACT diameter | 5 inches | Soft foam, easy to compress |
| Tile size | 24 in × 24 in | Standard FTC foam tiles |

**Odometry Tip:** Use inches as your standard unit (easier than cm for FTC field measurements).

---

## 🎓 Learning Progression

### Week 1-2: Get Moving
- Deploy StarterTeleOp
- Drive around practice field
- Test basic controls

### Week 3-4: Add Shooting
- Calibrate shooter RPM
- Practice artifact loading
- Test firing consistency

### Week 5-6: Basic Autonomous
- Leave LAUNCH LINE
- Shoot preloaded artifacts
- Measure and tune drive distances

### Week 7-8: Cycle Efficiency
- Add intake mechanism
- Optimize shooting positions
- Practice driver coordination

### Week 9+: Advanced Features
- Add odometry for precise positioning
- Implement vision (AprilTag alignment)
- Try path-following libraries (Pedro Pathing)
- Work on RAMP PATTERN strategies

---

## 🏆 Scoring Goals by Experience Level

### Rookie Team (First Year)
- **Autonomous:** 15 points (leave line + 2 preloaded)
- **TeleOp:** 10-15 points (2-3 manual cycles)
- **Total:** 25-30 points per match
- **Goal:** Consistent autonomous, reliable shooting

### Sophomore Team (Second Year)
- **Autonomous:** 15-21 points (+ vision alignment)
- **TeleOp:** 30-50 points (5-8 cycles with intake)
- **Total:** 50-70 points per match
- **Goal:** Fast cycles, good driver coordination

### Veteran Team (3+ Years)
- **Autonomous:** 20-30 points (optimized paths, multi-artifact)
- **TeleOp:** 60-90 points (10+ cycles, strategic positioning)
- **Total:** 90-120 points per match
- **Goal:** Consistent high scores, advanced strategies

---

## 💡 Strategy Tips for Programmers

1. **Reliability > Speed**
   - Scoring 15 points every match > 40 points occasionally
   - Focus on consistent autonomous first

2. **Test, Test, Test**
   - Run autonomous 20+ times before competition
   - Practice with low battery (voltage affects shooter)
   - Test with different artifact conditions (old/new foam)

3. **Plan for Failures**
   - What if shooter jams?
   - What if alliance partner bumps you?
   - What if odometry drifts?
   - Graceful degradation in code prevents 0-point matches

4. **Use Telemetry Wisely**
   - Drivers can't read paragraphs during match
   - Use colors/symbols: ✅ ❌ ⚠️ ⏳
   - Show only critical info (RPM, artifact count, timer)

5. **Version Control**
   - Save working code before making changes
   - Tag code versions for each competition
   - Document what works and what doesn't

---

## 📚 Additional Resources

- **Game Manual:** Official rules (60+ pages, very detailed)
- **Game Q&A Forum:** Ask rule clarifications
- **CAD Models:** Field elements for simulation
- **AprilTag Library:** Vision-based localization
- **Pedro Pathing:** Path-following library for smooth autonomous

**Remember:** The best robot is the one that works reliably, not the one with the most features!

Good luck in the DECODE season! 🎉
