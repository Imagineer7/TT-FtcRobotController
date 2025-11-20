# FTC DECODE Starter Kit - Complete Summary

This document provides a comprehensive overview of the FTC DECODE Starter Kit repository structure, design decisions, and usage guidelines.

---

## 📋 Repository Purpose

**Primary Goal:** Provide a beginner-friendly, well-documented robot control framework for FTC teams participating in the 2025-2026 DECODE season.

**Target Audience:**
- Rookie teams with no prior FTC programming experience
- Teams transitioning from Blocks to Java
- Mentors teaching FTC programming concepts

**Design Philosophy:**
1. **Clarity over performance** - Code must be readable and understandable
2. **Simplicity over features** - Include only essential functionality
3. **Documentation over brevity** - Explain WHY, not just WHAT
4. **Gradual complexity** - Clear upgrade paths to advanced features

---

## 🏗️ Architecture Overview

### Based on AURORA Framework

This starter kit is derived from the **AURORA (Advanced Unified Robot Operating & Response Architecture)** framework used by FTC Team TT (Imagineer7). However, it has been significantly simplified:

**What was simplified:**
- ❌ Machine learning systems (RpmLearningSystem)
- ❌ Advanced performance monitoring
- ❌ Web dashboards
- ❌ Complex autonomous frameworks (Aurora Lightning)
- ❌ Field-relative driving modes
- ❌ Multi-driver coordination

**What was kept:**
- ✅ Subsystem coordination pattern (StarterRobotManager)
- ✅ Unified drive system (StarterDrive)
- ✅ Basic shooter control (StarterShooter)
- ✅ Graceful error handling
- ✅ Clear telemetry displays

**Why simplified?**
- Reduces cognitive load for beginners
- Focuses on fundamental concepts
- Provides clearer learning progression
- Still production-ready for competition

---

## 📁 File Structure Explained

```
StarterKit/
├── .github/
│   └── copilot-instructions.md      # Coding guidelines for contributors
│
├── docs/
│   ├── README-DECODE-Overview.md    # Game mechanics from programming perspective
│   ├── Sloth-Quickstart.md          # Sloth framework integration guide
│   └── ADB-Setup-Guide.md           # Android Studio & deployment instructions
│
├── TeamCode/
│   ├── build.gradle                 # Module config with Sloth plugin
│   └── src/main/java/.../teamcode/
│       ├── core/                    # Core robot framework
│       │   ├── StarterRobotManager.java  # Subsystem coordinator
│       │   ├── StarterDrive.java         # Tank/mecanum drive system
│       │   └── StarterShooter.java       # Flywheel shooter control
│       │
│       ├── teleop/
│       │   └── StarterTeleOp.java        # Driver-controlled OpMode
│       │
│       └── auto/
│           └── StarterAutoSkeleton.java  # Autonomous template
│
├── gradle/                          # Gradle wrapper (build system)
│
├── build.gradle                     # Root build configuration
├── build.common.gradle              # Shared Android settings
├── build.dependencies.gradle        # Dependencies (FTC SDK + Sloth)
├── settings.gradle                  # Multi-module project structure
├── gradle.properties                # Build optimization settings
│
├── README.md                        # Main project documentation
├── INSTALLATION.md                  # Setup instructions
├── LICENSE                          # MIT License
└── STARTER_KIT_SUMMARY.md          # This file
```

---

## 🎯 Key Components

### 1. StarterRobotManager (`core/StarterRobotManager.java`)

**Purpose:** Central coordinator for all robot subsystems.

**Responsibilities:**
- Initialize drive and shooter subsystems
- Handle hardware initialization errors gracefully
- Coordinate subsystem updates in main loop
- Manage driver mode (single vs dual driver)

**Key Features:**
- **Graceful degradation:** Robot continues working even if hardware fails to initialize
- **Clear error messages:** Shows exactly which hardware is missing/misconfigured
- **Extensible:** Easy to add new subsystems following same pattern

**Usage Pattern:**
```java
// In OpMode
StarterRobotManager robot = new StarterRobotManager(hardwareMap, telemetry);

// In loop
while (opModeIsActive()) {
    robot.update(gamepad1, gamepad2);
}

// On stop
robot.stop();
```

---

### 2. StarterDrive (`core/StarterDrive.java`)

**Purpose:** Unified drive system supporting tank and mecanum configurations.

**Supported Drive Types:**
- **TANK:** Traditional left/right arcade control
- **MECANUM:** Omnidirectional with strafing

**Key Features:**
- Single API for both drive types (easy to switch)
- Power limiting for safety (default 80%)
- Brake mode for precise control
- Encoder support for autonomous

**Motor Configuration:**
```java
// Standard motor names (must match robot config)
frontLeft, frontRight, backLeft, backRight

// Standard directions (adjust if needed)
leftFront.setDirection(REVERSE);
leftBack.setDirection(REVERSE);
rightFront.setDirection(FORWARD);
rightBack.setDirection(FORWARD);
```

**Usage Pattern:**
```java
// Create drive system
StarterDrive drive = new StarterDrive(hardwareMap, DriveType.MECANUM);

// In loop
double axial = -gamepad1.left_stick_y;   // Forward/back
double lateral = gamepad1.left_stick_x;  // Strafe
double yaw = gamepad1.right_stick_x;     // Rotation
drive.drive(axial, lateral, yaw);
```

---

### 3. StarterShooter (`core/StarterShooter.java`)

**Purpose:** Simple flywheel shooter for launching ARTIFACTS.

**Features:**
- RPM-based speed control (not just motor power)
- Two presets: SHORT_RANGE and SAFE_TEST
- Automatic feed servo timing
- Button-based controls

**Shooting Presets:**
```java
SHORT_RANGE: 100% power, 2780 RPM - Fast close shots
SAFE_TEST:   50% power, 1390 RPM  - Slow testing mode
```

**Hardware Requirements:**
```java
// Required in robot config
"shooter"   - DC motor with encoder (flywheel)
"feedServo" - Continuous rotation servo (feeder)
```

**Usage Pattern:**
```java
// Create shooter
StarterShooter shooter = new StarterShooter(hardwareMap);

// In loop
shooter.update(gamepad1);  // Handles Y=spin, A=fire

// Or manual control
shooter.spinUp();
if (shooter.isAtSpeed()) {
    shooter.fireOnce();
}
```

---

### 4. StarterTeleOp (`teleop/StarterTeleOp.java`)

**Purpose:** Driver-controlled OpMode for 2-minute match period.

**Control Scheme:**
```
Gamepad 1 (Single Driver Mode):
├── Left Stick Y:   Forward/Backward
├── Left Stick X:   Strafe (mecanum only)
├── Right Stick X:  Rotation
├── Y Button:       Spin up/stop shooter
├── A Button:       Fire artifact
└── X Button:       Change shooter preset
```

**Features:**
- Clear visual telemetry with status icons
- Real-time RPM and state display
- System health monitoring
- Automatic cleanup on stop

---

### 5. StarterAutoSkeleton (`auto/StarterAutoSkeleton.java`)

**Purpose:** Template for autonomous period (30 seconds).

**Default Strategy:**
1. Leave LAUNCH LINE (+3 points)
2. Position for shooting
3. Shoot 2 preloaded artifacts (+12 points)
4. Drive toward BASE (optional)

**Movement Type:** Time-based (simple but inaccurate)

**Upgrade Paths:**
- Encoder-based movement (better accuracy)
- IMU-based heading (prevents drift)
- Odometry (precise positioning)
- Pedro Pathing (smooth curves)

---

## 🔧 Dependencies

### Core Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| FTC SDK | 11.0.0 | Robot Controller base |
| Android SDK | 34 | Android app platform |
| Sloth Framework | 0.2.4 | Advanced control patterns |
| SlothBoard | 0.2.4+0.4.17 | Enhanced telemetry |

### Build Tools

| Tool | Version | Purpose |
|------|---------|---------|
| Gradle | 8.9 | Build automation |
| Android Gradle Plugin | 8.7.0 | Android app building |
| Java | 8+ | Programming language |

### Optional Dependencies (Commented Out)

| Dependency | Version | Purpose |
|------------|---------|---------|
| Pedro Pathing | 2.0.4 | Path following for autonomous |

---

## 📖 Documentation Structure

### Main Documentation

**README.md** - Project overview and quick start
- DECODE game summary
- Hardware requirements
- Installation steps
- Customization guide
- Troubleshooting

**INSTALLATION.md** - Detailed setup instructions
- FtcRobotController module setup
- Two installation methods
- Verification steps
- Troubleshooting

### Technical Guides

**docs/README-DECODE-Overview.md** - Game mechanics for programmers
- Field layout and game elements
- Match phases and scoring
- Robot design considerations
- Common programming challenges
- Code patterns and solutions

**docs/Sloth-Quickstart.md** - Sloth framework integration
- What Sloth is and why it's used
- Gradle configuration explained
- Feature-by-feature usage guide
- Troubleshooting
- Learning progression

**docs/ADB-Setup-Guide.md** - Deployment and debugging
- ADB fundamentals
- Android Studio setup
- Connection methods (USB + wireless)
- Deployment walkthrough
- Logcat debugging
- Common issues and solutions

### Contributor Guides

**.github/copilot-instructions.md** - Coding guidelines
- Repository philosophy
- Coding standards
- Documentation requirements
- Feature addition checklist
- Common scenarios

---

## 🎓 Learning Progression

### Week 1-2: Get Moving
**Goal:** Drive the robot in TeleOp

**Tasks:**
- Deploy StarterTeleOp to robot
- Test drive controls on practice field
- Verify all motors move correctly
- Practice driver coordination

**Success Criteria:** Robot drives smoothly and predictably

---

### Week 3-4: Add Shooting
**Goal:** Score artifacts in TeleOp

**Tasks:**
- Test shooter with SAFE_TEST preset
- Practice loading artifacts
- Calibrate SHORT_RANGE preset
- Test firing accuracy

**Success Criteria:** Consistent artifact launching into GOAL

---

### Week 5-6: Basic Autonomous
**Goal:** Score 15+ points in autonomous

**Tasks:**
- Run StarterAutoSkeleton
- Measure and tune drive times
- Test shooting from starting position
- Verify leaves LAUNCH LINE completely

**Success Criteria:** Autonomous runs reliably every match

---

### Week 7-8: Optimize TeleOp
**Goal:** Fast artifact cycles

**Tasks:**
- Add intake mechanism (optional)
- Practice driver efficiency
- Optimize shooting positions
- Add telemetry for drivers

**Success Criteria:** 5+ artifact cycles per match

---

### Week 9+: Advanced Features
**Goal:** Competition-level performance

**Tasks:**
- Add odometry (GoBilda Pinpoint)
- Implement vision (AprilTag)
- Try Pedro Pathing
- Work on RAMP PATTERN strategies

**Success Criteria:** Consistent 60+ point matches

---

## 🎯 Scoring Potential

### Baseline (Starter Kit Default)
- **Autonomous:** 15 points (leave line + 2 preloaded)
- **TeleOp:** 12-18 points (2-3 manual cycles)
- **Total:** 27-33 points per match
- **Win Rate:** 30-40% against rookie competition

### With Intake Added
- **Autonomous:** 15 points (unchanged)
- **TeleOp:** 30-42 points (5-7 cycles)
- **Total:** 45-57 points per match
- **Win Rate:** 60-70% against rookie competition

### With Advanced Features
- **Autonomous:** 21-27 points (optimized path + vision)
- **TeleOp:** 60-90 points (10+ cycles, strategic positioning)
- **Total:** 81-117 points per match
- **Win Rate:** Competitive at regional level

---

## 🔐 Safety Features

### Hardware Protection

1. **Power Limiting:**
   ```java
   maxPower = 0.8;  // 80% max to prevent overheating
   ```

2. **Brake Mode:**
   ```java
   motor.setZeroPowerBehavior(BRAKE);  // Stop quickly
   ```

3. **Encoder Usage:**
   ```java
   motor.setMode(RUN_USING_ENCODER);  // Consistent speed
   ```

### Software Protection

1. **Null Checks:**
   ```java
   if (drive == null) return;  // Skip if hardware failed
   ```

2. **Timeout Protection:**
   ```java
   while (opModeIsActive() && timer.seconds() < 3.0) {
       // Autonomous with timeout
   }
   ```

3. **OpMode Active Checks:**
   ```java
   while (opModeIsActive()) {
       // Stop immediately when STOP pressed
   }
   ```

---

## 🛠️ Customization Guide

### Change Drive Type

**File:** `StarterRobotManager.java`, line ~83

```java
// Tank drive
driveSystem = new StarterDrive(hardwareMap, DriveType.TANK);

// Mecanum drive
driveSystem = new StarterDrive(hardwareMap, DriveType.MECANUM);
```

### Adjust Shooter Speed

**File:** `StarterShooter.java`, line ~44

```java
// Modify existing preset
SHORT_RANGE(1.0, 2780, "Short Range"),

// Or add new preset
LONG_RANGE(0.85, 4000, "Long Range"),
```

### Add New Subsystem

1. Create `StarterIntake.java` in `core/` package
2. Follow same pattern as `StarterShooter.java`
3. Add to `StarterRobotManager.java`:
   ```java
   private StarterIntake intake;
   
   // In initializeSystems()
   try {
       intake = new StarterIntake(hardwareMap);
   } catch (Exception e) {
       // Handle error
   }
   
   // In update()
   if (intake != null) {
       intake.update(gamepad);
   }
   ```

### Modify Autonomous

**File:** `StarterAutoSkeleton.java`

**Time-based (current):**
```java
drive.drive(0.5, 0, 0);
sleep(1000);  // Drive 1 second
```

**Encoder-based (upgrade):**
```java
driveDistance(24.0);  // Drive 24 inches
```

**Odometry-based (advanced):**
```java
driveToPosition(36, 24, 90);  // x, y, heading
```

---

## 🧪 Testing Recommendations

### Pre-Competition Checklist

**Hardware:**
- [ ] All motors respond correctly
- [ ] Encoders counting properly
- [ ] Shooter reaches target RPM
- [ ] Feed servo feeds reliably
- [ ] Battery fully charged (12.5V+)

**Software:**
- [ ] TeleOp drives smoothly
- [ ] Shooter presets calibrated
- [ ] Autonomous runs 20+ times successfully
- [ ] Emergency stop works (gamepad STOP button)
- [ ] Telemetry displays correctly

**Team:**
- [ ] Drivers practiced controls
- [ ] Quick hardware config changes
- [ ] Backup USB cable available
- [ ] Code deployed to Robot Controller
- [ ] Driver Station paired

### Testing Procedure

1. **Bench Test (No Field):**
   - Deploy code
   - Test each subsystem individually
   - Verify telemetry displays

2. **Field Test (Practice Field):**
   - Run TeleOp, test all controls
   - Run autonomous 10+ times
   - Measure actual drive distances
   - Test from different starting positions

3. **Competition Simulation:**
   - Full 2.5 minute matches
   - Switch drivers
   - Practice between-match troubleshooting
   - Time code deployment process

---

## 📈 Upgrade Roadmap

### Phase 1: Baseline (Current)
- ✅ Basic drive (tank/mecanum)
- ✅ Simple shooter (2 presets)
- ✅ Time-based autonomous
- ✅ TeleOp with telemetry

**Scoring Potential:** 27-33 points

---

### Phase 2: Intake System
- ⬜ Add intake subsystem
- ⬜ Integrate with StarterRobotManager
- ⬜ Add intake controls to TeleOp
- ⬜ Test artifact cycling

**Scoring Potential:** 45-57 points

---

### Phase 3: Encoder-Based Autonomous
- ⬜ Add encoder-based movement methods
- ⬜ Implement precise distance driving
- ⬜ Add IMU for heading control
- ⬜ Improve autonomous consistency

**Scoring Potential:** 50-65 points

---

### Phase 4: Vision System
- ⬜ Add camera hardware
- ⬜ Implement AprilTag detection
- ⬜ Auto-align with GOAL
- ⬜ Vision-guided autonomous

**Scoring Potential:** 70-90 points

---

### Phase 5: Odometry & Path Following
- ⬜ Install GoBilda Pinpoint or dead wheels
- ⬜ Calibrate odometry
- ⬜ Integrate Pedro Pathing
- ⬜ Create smooth autonomous paths

**Scoring Potential:** 90-120 points

---

## 🤝 Contributing

This is an open-source learning resource. Contributions welcome!

**How to Contribute:**
1. Fork the repository
2. Create feature branch: `git checkout -b feature/intake-system`
3. Follow coding guidelines in `.github/copilot-instructions.md`
4. Test on real hardware
5. Submit pull request with documentation

**What to Contribute:**
- Bug fixes
- Additional subsystems (intake, arm, lift)
- Improved documentation
- Example autonomous strategies
- Video tutorials

---

## 📞 Support

**For Issues:**
- Check [docs/ADB-Setup-Guide.md](docs/ADB-Setup-Guide.md) for deployment issues
- Review [INSTALLATION.md](INSTALLATION.md) for setup problems
- Search FTC forums: https://ftcforum.firstinspires.org/

**For Questions:**
- FTC Discord: #programming channel
- Chief Delphi: FTC subforum
- Reddit: /r/FTC

**For Updates:**
- Star this repository for notifications
- Follow for season updates
- Check for new releases

---

## 📄 License

MIT License - See [LICENSE](LICENSE) for details.

Based on AURORA architecture by FTC Team TT (Imagineer7).
Uses FIRST Tech Challenge SDK (BSD 3-Clause License).

---

**Ready to build your DECODE robot?** Start with [README.md](README.md)! 🚀

Good luck in the 2025-2026 FTC DECODE season! 🎉
