# FTC DECODE Starter Kit - Copilot Instructions

## ⚠️ IMPORTANT: Philosophy of This Repository

**This is a TEACHING repository, not a competition repository.**

The primary goal is to help rookie FTC teams learn robot programming through clear, well-documented code. Every decision prioritizes:

1. **Clarity over performance** - Code should be easy to understand
2. **Simplicity over features** - Only include what beginners need
3. **Documentation over brevity** - Comments explain "why", not just "what"
4. **Gradual complexity** - Clear upgrade paths to advanced features

---

## Repository Overview

### Quick Facts
- **Purpose**: Beginner-friendly FTC robot starter kit for DECODE season
- **SDK Version**: FTC SDK 11.0.0
- **Language**: Java
- **Build System**: Gradle 8.9 with Android Gradle Plugin 8.7.0
- **Framework**: Sloth 0.2.4 (configured but optional)
- **Target Audience**: Rookie teams with no prior FTC experience

### Architecture Pattern

This starter kit is based on the **AURORA architecture** from the parent TT-FtcRobotController repository, but heavily simplified:

- **StarterRobotManager** - Simplified version of AuroraManager (no ML, no advanced monitoring)
- **StarterDrive** - Unified tank/mecanum drive (no field-relative, no adaptive modes)
- **StarterShooter** - Basic flywheel shooter (no ML learning, just two presets)

**Key Difference:** Parent repo is competition-optimized. This repo is learning-optimized.

---

## Build and Testing

### Build Commands
```bash
# Verify Gradle
./gradlew --version

# Build Robot Controller APK
./gradlew assembleDebug

# Clean build artifacts
./gradlew clean
```

### No Automated Tests
This repository intentionally **does not include** automated testing infrastructure:
- No JUnit tests
- No instrumentation tests
- No CI/CD pipelines

**Why:** Testing frameworks add complexity that overwhelms beginners. Manual testing on actual hardware is more valuable for learning FTC robotics.

### Manual Testing Process
1. Build APK
2. Deploy to Robot Controller
3. Test with Driver Station
4. Monitor telemetry for issues

---

## File Structure

```
StarterKit/
├── .github/
│   └── copilot-instructions.md      # This file
├── docs/
│   ├── README-DECODE-Overview.md    # Game rules for programmers
│   ├── Sloth-Quickstart.md          # Framework setup guide
│   └── ADB-Setup-Guide.md           # Android Studio deployment
├── TeamCode/
│   └── src/main/java/.../teamcode/
│       ├── core/                    # Core framework classes
│       │   ├── StarterRobotManager.java
│       │   ├── StarterDrive.java
│       │   └── StarterShooter.java
│       ├── teleop/
│       │   └── StarterTeleOp.java
│       └── auto/
│           └── StarterAutoSkeleton.java
├── build.gradle                     # Root build config
├── build.common.gradle              # Shared Android config
├── build.dependencies.gradle        # Dependencies (FTC SDK + Sloth)
└── README.md                        # Main project documentation
```

---

## Coding Guidelines

### Naming Conventions

**Classes** (PascalCase):
```java
StarterRobotManager    // Manager classes
StarterDrive           // Subsystems
StarterTeleOp          // OpModes
```

**Methods** (camelCase):
```java
update()              // Actions
getCurrentRPM()       // Getters
setMaxPower()         // Setters
isAtSpeed()           // Boolean checks (is/has prefix)
```

**Variables** (camelCase):
```java
currentPower          // State
maxSpeed              // Limits
shooterRunning        // Boolean flags
```

**Constants** (UPPER_SNAKE_CASE):
```java
MAX_POWER             // Limits
FIRE_DURATION         // Timing
COUNTS_PER_REV        // Hardware specs
```

---

### Documentation Standards

**Every class must have:**
1. Purpose statement (what it does)
2. Key features (how it works)
3. How to use (basic usage pattern)

**Example:**
```java
/**
 * StarterShooter - Simple flywheel shooter for launching ARTIFACTS
 * 
 * Purpose:
 * - Launch preloaded ARTIFACTS into the GOAL
 * - Control flywheel speed with presets
 * - Simple one-button firing
 * 
 * Hardware Requirements:
 * Your robot hardware config must have:
 * - "shooter" - DC motor with encoder (flywheel)
 * - "feedServo" - Continuous rotation servo (feeds artifacts)
 * 
 * How to Use:
 * 1. Call spinUp() to start the flywheel
 * 2. Wait until isAtSpeed() returns true
 * 3. Press fire button (handled in update())
 */
```

**Complex logic must explain WHY:**
```java
// Bad comment:
frontLeftPower = axial + lateral + yaw;  // Calculate power

// Good comment:
// Calculate power for each wheel
// This math comes from vector analysis of mecanum wheel geometry:
// Each wheel contributes to all three movement types based on its roller angle
frontLeftPower = axial + lateral + yaw;
```

---

### Hardware Configuration

**Standard motor names** (must match robot config):
- `frontLeft`, `frontRight`, `backLeft`, `backRight` - Drive motors
- `shooter` - Flywheel motor
- `feedServo` - Feed servo

**Motor directions:**
```java
// Standard configuration (adjust if needed)
frontLeft.setDirection(DcMotor.Direction.REVERSE);
backLeft.setDirection(DcMotor.Direction.REVERSE);
frontRight.setDirection(DcMotor.Direction.FORWARD);
backRight.setDirection(DcMotor.Direction.FORWARD);
```

**Motor configuration pattern:**
```java
// Always use encoders for better control
motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Brake mode for safety (robot stops quickly)
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// Exception: Flywheel uses FLOAT (coast to maintain speed)
shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
```

---

### OpMode Structure

**LinearOpMode template:**
```java
@TeleOp(name="My OpMode", group="Competition")  // or @Autonomous
public class MyOpMode extends LinearOpMode {
    // 1. Declare subsystems
    private StarterRobotManager robot;
    private ElapsedTime timer = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        // 2. Initialize
        telemetry.addLine("Initializing...");
        robot = new StarterRobotManager(hardwareMap, telemetry);
        telemetry.update();
        
        // 3. Wait for start
        waitForStart();
        timer.reset();
        
        // 4. Main loop
        while (opModeIsActive()) {
            // Update subsystems
            robot.update(gamepad1, gamepad2);
            
            // Custom logic here
            
            // Telemetry
            telemetry.addData("Timer", timer.seconds());
            telemetry.update();
        }
        
        // 5. Cleanup
        robot.stop();
    }
}
```

---

### Error Handling Pattern

**Always use graceful degradation:**
```java
// Initialize subsystem with error handling
try {
    shooter = new StarterShooter(hardwareMap);
    shooterInitialized = true;
    telemetry.addLine("✅ Shooter initialized");
} catch (Exception e) {
    shooter = null;
    shooterInitialized = false;
    telemetry.addLine("❌ Shooter FAILED: " + e.getMessage());
    telemetry.addLine("   Check hardware config: shooter, feedServo");
}
```

**Why:** Robot should still work (drive-only) if shooter hardware fails. Helps teams debug configuration issues without complete failures.

---

### Telemetry Best Practices

**Use visual hierarchy:**
```java
telemetry.addLine("╔════════════════════════════════════╗");
telemetry.addLine("║   FTC DECODE - DRIVER CONTROL      ║");
telemetry.addLine("╚════════════════════════════════════╝");
telemetry.addLine();

telemetry.addLine("━━━━ SHOOTER SYSTEM ━━━━");
telemetry.addData("Status", shooter.getStateString());
telemetry.addData("RPM", "%.0f / %.0f", currentRPM, targetRPM);
```

**Use icons for quick status:**
- ✅ Success / Ready
- ❌ Error / Failed
- ⚠️ Warning / Partial
- ⏳ In Progress / Waiting
- 🎯 Target / Goal

---

## Adding New Features

### When to Add a Feature

**YES - Add if:**
- Helps beginners learn a fundamental concept
- Solves a common problem for rookie teams
- Has clear, simple documentation
- Doesn't require understanding advanced topics

**NO - Don't add if:**
- Only useful for advanced competition strategies
- Requires deep understanding of complex algorithms
- Makes existing code harder to understand
- Better suited for an "advanced" branch/example

### Feature Addition Checklist

When adding a new feature:

- [ ] Keep API simple (few methods, clear names)
- [ ] Add detailed javadoc comments
- [ ] Explain WHY the feature exists
- [ ] Show example usage in comments
- [ ] Update relevant documentation
- [ ] Test on actual hardware (no automated tests available)
- [ ] Verify error messages are helpful

---

## Sloth Framework Guidelines

### Current Usage

Sloth is **configured but minimally used** in the default code:
- Build plugin: Active (processes annotations)
- Runtime library: Included
- Dashboard: Available but not required

**Intentional:** Lets beginners start with familiar patterns, then adopt Sloth features gradually.

### When Adding Sloth Features

- Document in `docs/Sloth-Quickstart.md`
- Mark as "Advanced" if not beginner-friendly
- Provide non-Sloth alternative for comparison
- Explain benefits clearly

---

## Documentation Requirements

### All Documentation Must:

1. **Assume zero FTC experience**
   - Define acronyms on first use
   - Explain context before diving into details
   - Use analogies to familiar concepts

2. **Be practical, not theoretical**
   - Show code examples
   - Explain common pitfalls
   - Provide troubleshooting steps

3. **Have clear structure**
   - Use headers for navigation
   - Include table of contents for long docs
   - Separate basic and advanced sections

4. **Stay current**
   - Update for SDK changes
   - Fix outdated examples
   - Keep consistent with codebase

---

## Common Scenarios

### Scenario: Team wants to add intake mechanism

**Good approach:**
1. Create `StarterIntake.java` in `core/` package
2. Follow same pattern as `StarterShooter.java`:
   - Simple API: `extend()`, `retract()`, `collect()`, `stop()`
   - Clear javadoc with hardware requirements
   - Error handling with telemetry messages
3. Add to `StarterRobotManager.java` with graceful error handling
4. Update `StarterTeleOp.java` with intake controls
5. Document control scheme in comments

**Bad approach:**
- Complex state machine
- Machine learning for pickup optimization
- Integration with vision system
- Multi-page documentation

---

### Scenario: Team's robot drives backward when pushing forward

**Solution in docs:**
```markdown
### Robot drives in wrong direction

1. Open `StarterDrive.java`
2. Find `configureMotors()` method (around line 123)
3. For motors that move wrong way, swap:
   - `FORWARD` → `REVERSE`
   - `REVERSE` → `FORWARD`
4. Rebuild and test
```

---

### Scenario: Team wants better autonomous

**Guidance in docs:**
```markdown
### Upgrade Path for Autonomous

**Level 1 (Current):** Time-based movement
- Drive forward X seconds
- Simple but inaccurate

**Level 2 (Easy upgrade):** Encoder-based movement
- Add `driveDistance()` method using motor encoders
- More accurate, still simple code

**Level 3 (Intermediate):** IMU-based heading
- Add gyro to maintain straight lines
- Prevents drift during long movements

**Level 4 (Advanced):** Odometry + Path Following
- Add GoBilda Pinpoint or dead wheels
- Use Pedro Pathing library
- Competition-level precision
```

---

## Version Control Best Practices

### Branch Strategy

**main** - Stable, tested code for beginners
**advanced-features** - Optional complex features
**experimental** - Work in progress, may break

### Commit Messages

**Good:**
```
Add intake subsystem with simple extend/retract API

- Created StarterIntake.java following subsystem pattern
- Added intake controls to TeleOp (B button)
- Updated documentation with hardware requirements
- Tested on physical robot: extends/retracts reliably
```

**Bad:**
```
Updated code
```

---

## Release Process

### Before Each Release

1. Test on physical robot (all OpModes)
2. Verify documentation is current
3. Check all links work
4. Review for beginner-friendly language
5. Tag version: `v1.0.0`, `v1.1.0`, etc.

### Season Updates

- **Start of season:** Update to latest FTC SDK
- **Mid-season:** Only bug fixes (don't break working code)
- **Off-season:** Major feature additions, refactoring

---

## Contributing Guidelines

### For AI Assistants (Copilot, etc.)

When generating code for this repository:

1. **Prioritize clarity** - Readable code > clever code
2. **Document thoroughly** - Explain WHY, not just WHAT
3. **Keep it simple** - Beginner-friendly patterns only
4. **Test manually** - No automated tests available
5. **Graceful errors** - Never crash, always provide helpful messages

### For Human Contributors

1. **Ask first** - Open issue before major changes
2. **Match style** - Follow existing patterns exactly
3. **Document** - Update all affected docs
4. **Test hardware** - Must work on real robot
5. **Think rookie** - Would a beginner understand this?

---

## Resources

### Parent Repository (Advanced Version)
- Repository: `Imagineer7/TT-FtcRobotController`
- Branch: `master`
- Use for reference, not for copying complex features

### FTC Resources
- **SDK Docs:** https://ftctechnh.github.io/ftc_app/doc/javadoc/
- **Game Manual:** Official FTC game rules
- **FTC Discord:** Community support

### Framework Resources
- **Sloth Framework:** https://github.com/Dairy-Foundation/Sloth
- **Pedro Pathing:** https://github.com/Pedro-Pathing/pedro-pathing

---

## Quick Reference

### Add New Subsystem
1. Create class in `core/` package
2. Add to `StarterRobotManager.java`
3. Add controls to `StarterTeleOp.java`
4. Update README.md hardware requirements
5. Test on robot

### Add New OpMode
1. Create in `teleop/` or `auto/` package
2. Extend `LinearOpMode`
3. Add `@TeleOp` or `@Autonomous` annotation
4. Follow standard OpMode structure
5. Comment control scheme

### Update Documentation
1. Edit relevant `.md` file in `docs/`
2. Keep consistent formatting
3. Update table of contents if needed
4. Check all links work
5. Spell check

---

**Remember: This repository exists to teach, not to win competitions. Every change should make learning easier, not harder!** 🎓

Good luck coaching teams in the DECODE season! 🤖
