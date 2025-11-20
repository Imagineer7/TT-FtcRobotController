# FTC DECODE Starter Kit 🤖

**A beginner-friendly robot control framework for the 2025-2026 FIRST Tech Challenge DECODE season**

This starter kit provides a clean, well-documented codebase based on the competition-tested AURORA architecture. It's designed to help rookie teams get a robot driving and scoring within their first practice session, while providing clear upgrade paths to advanced features.

---

## 🎮 What is DECODE?

DECODE is the 2025-2026 FTC game where two alliances compete to:

1. **Score ARTIFACTS** (5-inch foam balls) into GOAL structures
   - Purple artifacts for one alliance, green for the other
   - 6 points per artifact in GOAL
   - Extra points for CLASSIFIED vs OVERFLOW sections

2. **Build RAMP PATTERNS** using colored tiles
   - Match the MOTIF pattern shown at field start
   - Score multipliers for completing patterns

3. **Navigate Strategically** through three phases:
   - **AUTONOMOUS** (30 sec): Pre-programmed robot actions
   - **DRIVER-CONTROLLED** (2 min): Human-operated gameplay
   - **ENDGAME** (30 sec): Final scoring rush and positioning

### What This Starter Kit Does

Out of the box, this robot can:
- ✅ **Drive** using mecanum or tank drive configurations
- ✅ **Leave LAUNCH LINE** in autonomous (+3 points)
- ✅ **Shoot preloaded ARTIFACTS** into GOAL (+12 points for 2 artifacts)
- ✅ **Driver-controlled operation** with clear gamepad controls

**Total baseline score: ~15 points** - enough to win against teams with no autonomous!

For complete game rules and programming context, see [docs/README-DECODE-Overview.md](docs/README-DECODE-Overview.md).

---

## 🚀 Quick Start (First-Time Setup)

### ⚠️ Important First Step

This starter kit requires the **FtcRobotController** module from the official FTC SDK. See [INSTALLATION.md](INSTALLATION.md) for detailed setup instructions.

**Quick option:** Clone the official FTC SDK and copy this starter kit's TeamCode into it.

### Prerequisites

1. **Android Studio Ladybug (2024.2) or later**
   - Download from: https://developer.android.com/studio
   - Includes Android SDK, emulators, and build tools

2. **Robot Controller Device**
   - REV Control Hub (recommended) or
   - Android phone with USB debugging enabled

3. **Internet Connection**
   - Required for first build to download dependencies

### Installation Steps

1. **Set up the project**
   - **Recommended:** Follow [INSTALLATION.md](INSTALLATION.md) for complete instructions
   - **Quick option:** Clone FTC SDK, copy this starter kit's code

2. **Open in Android Studio**
   - File → Open → Select the project root directory
   - Wait for Gradle sync to complete (2-5 minutes first time)

3. **Connect your Robot Controller**
   - Enable Developer Options and USB Debugging
   - Connect via USB cable or wireless
   - See [docs/ADB-Setup-Guide.md](docs/ADB-Setup-Guide.md) for detailed instructions

4. **Deploy and Run**
   - Click the green "Run" button in Android Studio
   - Select your Robot Controller device
   - App installs automatically

5. **Configure Hardware**
   - Open `StarterRobotManager.java`
   - Verify motor/servo names match your robot configuration
   - Default names: `frontLeft`, `frontRight`, `backLeft`, `backRight`, `shooter`, `feedServo`

6. **Test Drive**
   - Connect Driver Station to Robot Controller
   - Select "Starter TeleOp" OpMode
   - Press START and test controls!

---

## 📦 What's Included

### Core Framework (`TeamCode/src/main/java/.../core/`)

- **StarterRobotManager.java** - Central coordinator for all subsystems
  - Graceful error handling (robot works even if hardware fails)
  - Single and dual driver modes
  - Clear initialization messages

- **StarterDrive.java** - Unified drive system
  - Supports tank (2-side) and mecanum (4-wheel holonomic) configurations
  - Power limiting and motor direction management
  - Easy switching between drive types

- **StarterShooter.java** - Simple flywheel shooter
  - RPM-based speed control
  - Two presets: SHORT_RANGE and SAFE_TEST
  - Automatic feed servo timing

### OpModes

- **StarterTeleOp.java** (`teleop/`) - Driver-controlled operation
  - Left stick: Forward/backward + strafe
  - Right stick: Rotation
  - Y button: Spin up shooter
  - A button: Fire artifact
  - X button: Change shooter preset

- **StarterAutoSkeleton.java** (`auto/`) - Autonomous template
  - Leave LAUNCH LINE (required for points)
  - Spin up and shoot preloaded artifacts
  - Drive toward BASE (optional positioning)
  - Heavily commented with expansion suggestions

### Build Configuration

- **Gradle 8.9** with Android Gradle Plugin 8.7.0
- **FTC SDK 11.0.0** (DECODE season)
- **Sloth Framework 0.2.4** (advanced control patterns)
- Optional: Pedro Pathing support (commented out by default)

---

## 🛠️ Hardware Requirements

### Minimum Viable Robot (Can Score ~15 Points)

**Drivetrain:**
- 4× DC motors (REV HD Hex or equivalent)
- 4× Mecanum wheels OR standard wheels for tank drive
- REV Control Hub or Robot Controller phone

**Shooter System:**
- 1× DC motor with encoder (flywheel)
- 1× Continuous rotation servo (feeder)
- Basic ARTIFACT holder/guide

**Power:**
- 12V battery (fully charged: ~13V)
- Power distribution hub

**Control:**
- REV Driver Station phone
- 1-2 Gamepads (Logitech F310 or Xbox controllers)

### Recommended Competitive Robot (Can Score 50+ Points)

**Add to Minimum:**
- Intake mechanism (motorized roller/claw)
- Indexer (holds 2-4 artifacts)
- Improved shooter (dual flywheels, adjustable angle)
- Odometry system (GoBilda Pinpoint or dead wheels)
- Camera for AprilTag vision (helps with autonomous alignment)
- Arm/lift for RAMP PATTERN building

---

## 📚 Documentation

| Document | Purpose |
|----------|---------|
| [INSTALLATION.md](INSTALLATION.md) | Complete installation and setup instructions |
| [README-DECODE-Overview.md](docs/README-DECODE-Overview.md) | Game rules explained for programmers |
| [Sloth-Quickstart.md](docs/Sloth-Quickstart.md) | Sloth framework setup and usage |
| [ADB-Setup-Guide.md](docs/ADB-Setup-Guide.md) | Android Debug Bridge and deployment |
| [copilot-instructions.md](.github/copilot-instructions.md) | Guidelines for contributing code |

---

## 🎯 Customization Guide

### Change Drive Type (Tank ↔ Mecanum)

Edit `StarterRobotManager.java`, line ~83:
```java
driveSystem = new StarterDrive(
    hardwareMap, 
    StarterDrive.DriveType.MECANUM  // Change to TANK for tank drive
);
```

### Adjust Shooter Speed

Edit `StarterShooter.java`, preset definitions:
```java
SHORT_RANGE(1.0, 2780, "Short Range"),  // Change power or RPM
SAFE_TEST(0.5, 1390, "Safe Test")       // Lower speed for practice
```

Or create new presets:
```java
LONG_RANGE(0.85, 4000, "Long Range - High arc shots"),
```

### Modify Autonomous Strategy

Edit `StarterAutoSkeleton.java`:
- Adjust drive times in `leaveStartLine()`
- Add strafing in `positionForShot()`
- Change shooter preset in `shootPreloadedArtifacts()`
- Add vision-based alignment (requires camera)

### Hardware Names

If your robot configuration uses different names, update these files:
- **StarterRobotManager.java** (lines 92-95): Motor names
- **StarterShooter.java** (lines 61-62): Shooter motor and servo names

---

## 🔧 Troubleshooting

### "Can't find motor 'frontLeft'"
- Check hardware configuration file on Robot Controller
- Verify spelling and capitalization match exactly
- Make sure all motors are properly connected

### Robot drives in wrong direction
- Open `StarterDrive.java`
- Find `configureMotors()` method
- Swap `FORWARD` ↔ `REVERSE` for problematic motors

### Shooter not spinning
- Check "shooter" motor name in hardware config
- Verify motor is plugged into correct port
- Test with lower power preset (SAFE_TEST)

### Gradle sync fails
- Check internet connection (required for dependencies)
- Delete `.gradle/` folder and retry
- Update Android Studio to latest version

### Can't deploy to Robot Controller
- See [ADB-Setup-Guide.md](docs/ADB-Setup-Guide.md)
- Check USB debugging is enabled
- Try wireless ADB connection
- Restart Robot Controller device

---

## 🎓 Learning Path

1. **Week 1-2: Basics**
   - Get robot driving in TeleOp
   - Test shooter with SAFE_TEST preset
   - Practice with Driver Station

2. **Week 3-4: Autonomous**
   - Run StarterAutoSkeleton
   - Tune drive times for your field position
   - Add shooting to autonomous

3. **Week 5-6: Enhancements**
   - Add intake mechanism
   - Improve shooter accuracy
   - Learn encoder-based movement

4. **Week 7+: Advanced**
   - Add odometry (GoBilda Pinpoint)
   - Implement vision (AprilTag alignment)
   - Try Pedro Pathing for smooth autonomous

---

## 🤝 Contributing & Support

This is a starter template - make it your own! Some ideas:

- Add new subsystems (intake, arm, lift)
- Create additional autonomous strategies
- Improve telemetry displays
- Add gamepad rumble feedback
- Document your improvements

For contribution guidelines, see [.github/copilot-instructions.md](.github/copilot-instructions.md).

---

## 📄 License

This starter kit is provided under the MIT License. See [LICENSE](LICENSE) for details.

Based on the AURORA architecture developed by FTC Team TT (Imagineer7) for the 2025-2026 DECODE season.

---

## 🏆 Season Goals

- **Rookie Teams**: 20-30 points per match (autonomous + basic TeleOp)
- **Intermediate Teams**: 50-70 points per match (refined autonomous + intake)
- **Advanced Teams**: 100+ points per match (vision, odometry, complex strategies)

**Good luck in the DECODE season!** 🎉
