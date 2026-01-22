# Dbg Logging Utility - Usage Guide

## Quick Start

### 1. Basic Setup (in your OpMode)

```java
@TeleOp(name="My OpMode", group="Testing")
public class MyOpMode extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        // Configure logging (do this once in init)
        Dbg.setGlobalPrefix("SnowRover");
        Dbg.setContext("TeleOp", "INIT");
        Dbg.setGlobalLevel(LogLevel.DEBUG);
        
        // Optional: enable loop counter
        Dbg.setIncludeLoopCount(true);
        Dbg.resetLoop();
        
        // Optional: enable Panels telemetry output (INFO and above)
        Dbg.setEnablePanelsTelemetry(true);
        Dbg.setMaxTelemetryLines(5);
        
        // Initialize hardware...
        Dbg.i(LogGroup.HARDWARE, "Initializing hardware...");
        
        waitForStart();
        Dbg.setContext("TeleOp", "RUN");
        
        while (opModeIsActive()) {
            Dbg.incrementLoop();  // Increment if using loop counter
            
            // Your code with logging...
            
            telemetry.update();
        }
    }
    
    @Override
    public void stop() {
        Dbg.setContext("TeleOp", "STOP");
        Dbg.i(LogGroup.SYSTEM, "OpMode stopping");
        Dbg.reset();  // Clean up
        super.stop();
    }
}
```

### 2. Basic Logging

```java
// Simple message
Dbg.d(LogGroup.DRIVE, "Drive enabled");

// Formatted message (printf-style)
Dbg.d(LogGroup.DRIVE, "Speed: %.2f, Heading: %.1f°", speed, heading);

// Different levels
Dbg.t(LogGroup.SENSORS, "Raw sensor: %d", raw);      // TRACE
Dbg.d(LogGroup.DRIVE, "Position: %.2f", pos);        // DEBUG
Dbg.i(LogGroup.AUTO, "Starting autonomous");         // INFO
Dbg.w(LogGroup.HARDWARE, "Low voltage: %.1fV", v);   // WARN
Dbg.e(LogGroup.INTAKE, "Motor not found: %s", name); // ERROR

// With exceptions
try {
    // risky code
} catch (Exception e) {
    Dbg.e(LogGroup.HARDWARE, e, "Failed to initialize motor");
}
```

### 3. Using Custom Groups

```java
// Predefined groups (recommended)
Dbg.d(LogGroup.INDEX, "Artifact count: %d", count);
Dbg.d(LogGroup.INTAKE, "State: %s", state);
Dbg.d(LogGroup.TRANSFER, "Transfer active");
Dbg.d(LogGroup.FIRE, "RPM: %.0f", rpm);
Dbg.d(LogGroup.DRIVE, "Mecanum speed: %.2f", speed);
Dbg.d(LogGroup.VISION, "AprilTag ID: %d", tagId);
Dbg.d(LogGroup.AUTO, "Step: %d", step);
Dbg.d(LogGroup.SENSORS, "Distance: %.1fmm", distance);

// Or use custom string groups
Dbg.d("CLIMBER", "Arm position: %d", position);
Dbg.d("CUSTOM", "My subsystem active");
```

## Advanced Features

### Spam Control

#### Rate Limiting (everyMs)
Log at most once per time period - perfect for high-frequency loops:

```java
// In your main loop - logs at most once per second
Dbg.everyMs(LogGroup.INTAKE, LogLevel.INFO, "status", 1000,
            "Artifacts: %d, State: %s", count, state);

// Multiple rate-limited logs with different keys
Dbg.everyMs(LogGroup.SENSORS, LogLevel.DEBUG, "distance_front", 500,
            "Front: %.1fmm", frontDistance);
Dbg.everyMs(LogGroup.SENSORS, LogLevel.DEBUG, "distance_back", 500,
            "Back: %.1fmm", backDistance);
```

#### One-Time Logging (once)
Log exactly once per OpMode run - useful for warnings:

```java
// Deprecation warning
Dbg.once(LogGroup.AUTO, LogLevel.WARN, "old_path",
         "Using deprecated autonomous path - update to PathV2");

// First-time detection
if (artifactDetected && !wasDetected) {
    Dbg.once(LogGroup.VISION, LogLevel.INFO, "first_artifact",
             "First artifact detected at position: %.1f", pos);
}
```

#### Counted Logging (counted)
Log every Nth occurrence - useful for sampling:

```java
// Log every 10th sensor reading
Dbg.counted(LogGroup.SENSORS, LogLevel.DEBUG, "odometry", 10,
            "Odom: X=%.1f Y=%.1f H=%.1f", x, y, heading);

// Log every 50 loops
Dbg.counted(LogGroup.SYSTEM, LogLevel.INFO, "heartbeat", 50,
            "Loop %d, voltage %.1fV", loopCount, voltage);
```

### Level Control

#### Global Level
```java
// Set minimum level for ALL groups
Dbg.setGlobalLevel(LogLevel.TRACE);  // Most verbose
Dbg.setGlobalLevel(LogLevel.DEBUG);  // Development
Dbg.setGlobalLevel(LogLevel.INFO);   // Default
Dbg.setGlobalLevel(LogLevel.WARN);   // Competition (quieter)
Dbg.setGlobalLevel(LogLevel.ERROR);  // Only errors
```

#### Per-Group Level
```java
// Show all TRACE logs for DRIVE, but ERROR only for VISION
Dbg.setGlobalLevel(LogLevel.INFO);              // Default
Dbg.setGroupLevel(LogGroup.DRIVE, LogLevel.TRACE);  // Override for DRIVE
Dbg.setGroupLevel(LogGroup.VISION, LogLevel.ERROR); // Override for VISION

// Now:
Dbg.t(LogGroup.DRIVE, "Will be logged");    // TRACE >= TRACE ✓
Dbg.d(LogGroup.VISION, "Will NOT be logged"); // DEBUG < ERROR ✗
Dbg.i(LogGroup.SENSORS, "Will be logged");  // INFO >= INFO ✓
```

#### Enable/Disable Groups
```java
// Completely disable a group (overrides level)
Dbg.setGroupEnabled(LogGroup.VISION, false);

// Now ALL vision logs are suppressed, regardless of level
Dbg.e(LogGroup.VISION, "Even errors are hidden!");  // Not logged

// Re-enable
Dbg.setGroupEnabled(LogGroup.VISION, true);
```

### Context and Prefixes

#### Global Prefix (Robot Name)
```java
Dbg.setGlobalPrefix("SnowRover");
// Logs: [SnowRover] DRIVE/D: Speed: 0.75
```

#### OpMode Context
```java
Dbg.setContext("AutoRed", "INIT");
// Logs: [SnowRover][AutoRed:INIT] DRIVE/D: Speed: 0.75

Dbg.setContext("AutoRed", "RUN");
// Logs: [SnowRover][AutoRed:RUN] DRIVE/D: Moving to position
```

#### Loop Counter
```java
Dbg.setIncludeLoopCount(true);
Dbg.resetLoop();

while (opModeIsActive()) {
    Dbg.incrementLoop();
    // Logs: [SnowRover][TeleOp:RUN][123] DRIVE/D: Speed: 0.75
}
```

#### Timestamp
```java
Dbg.setIncludeTimestamp(true);
// Logs: [SnowRover][TeleOp:RUN][123][1737500123456] DRIVE/D: Speed: 0.75
```

## Migration from System.out.println

### Before (System.out)
```java
System.out.println("IndexingSystem: Current state: " + state);
System.out.println("Artifact count: " + count);
System.out.println("WARNING: Low voltage: " + voltage + "V");
```

### After (Dbg)
```java
Dbg.d(LogGroup.INDEX, "Current state: %s", state);
Dbg.i(LogGroup.INDEX, "Artifact count: %d", count);
Dbg.w(LogGroup.HARDWARE, "Low voltage: %.1fV", voltage);
```

### Benefits
- ✅ Filterable by group and level
- ✅ Consistent formatting
- ✅ No string concatenation (faster)
- ✅ Proper log levels
- ✅ Can be disabled at runtime
- ✅ Integrates with FTC RobotLog

## Subsystem Integration

### Example: IndexingSystem
```java
public class IndexingSystem {
    private static final LogGroup LOG_GROUP = LogGroup.INDEX;
    
    public void update() {
        // Rate-limited status
        Dbg.everyMs(LOG_GROUP, LogLevel.DEBUG, "update_status", 1000,
                    "State: %s, Count: %d, Center: %b",
                    currentState, artifactCount, hasArtifactInCenter);
        
        switch (currentState) {
            case COLLECTING:
                Dbg.t(LOG_GROUP, "Collecting - sensor: %.1fmm", distance);
                if (artifactDetected()) {
                    Dbg.i(LOG_GROUP, "Artifact detected!");
                    transitionTo(SystemState.TRANSFERRING);
                }
                break;
                
            case TRANSFERRING:
                if (transferComplete) {
                    Dbg.d(LOG_GROUP, "Transfer complete, count: %d", artifactCount);
                    transitionTo(SystemState.IDLE);
                }
                break;
                
            case ERROR:
                Dbg.e(LOG_GROUP, "Error state: %s", errorMessage);
                break;
        }
    }
    
    private void transitionTo(SystemState newState) {
        Dbg.i(LOG_GROUP, "State: %s -> %s", currentState, newState);
        currentState = newState;
    }
}
```

### Example: Shooter
```java
public class Shooter {
    private static final LogGroup LOG_GROUP = LogGroup.FIRE;
    
    public void spinUpToRPM(double targetRPM) {
        Dbg.i(LOG_GROUP, "Spinning up to %.0f RPM", targetRPM);
        this.targetRPM = targetRPM;
        state = ShooterState.SPINNING_UP;
    }
    
    public void update() {
        double currentRPM = getCurrentRPM();
        
        // Sample RPM every 10 loops
        Dbg.counted(LOG_GROUP, LogLevel.TRACE, "rpm_sample", 10,
                    "RPM: %.0f / %.0f", currentRPM, targetRPM);
        
        if (state == ShooterState.SPINNING_UP) {
            if (isAtTargetRPM()) {
                Dbg.i(LOG_GROUP, "Ready to fire - RPM stable at %.0f", currentRPM);
                state = ShooterState.READY;
            } else if (isTimeout()) {
                Dbg.w(LOG_GROUP, "Spin-up timeout - current: %.0f, target: %.0f",
                      currentRPM, targetRPM);
            }
        }
    }
}
```

## Logcat Filtering

### View All Logs
```bash
adb logcat -s TeamDbg:*
```

### Filter by Level
```bash
# Warnings and errors only
adb logcat -s TeamDbg:W

# Info and above (suppress debug/trace)
adb logcat -s TeamDbg:I
```

### Filter by Group (grep)
```bash
# All INDEX logs
adb logcat -s TeamDbg:* | grep "INDEX/"

# All DRIVE debug logs
adb logcat -s TeamDbg:* | grep "DRIVE/D"

# Multiple groups
adb logcat -s TeamDbg:* | grep -E "(INDEX|INTAKE|TRANSFER)/"
```

### Filter by OpMode
```bash
# All logs from AutoRed
adb logcat -s TeamDbg:* | grep "\[AutoRed"

# All RUN phase logs
adb logcat -s TeamDbg:* | grep ":RUN\]"
```

### Complex Filters
```bash
# INDEX errors during RUN phase
adb logcat -s TeamDbg:* | grep ":RUN\]" | grep "INDEX/E"

# All warnings from TeleOp, excluding VISION
adb logcat -s TeamDbg:* | grep "\[TeleOp" | grep "/W" | grep -v "VISION"
```

## Configuration Presets

### Development/Testing
```java
public static void setupDevelopment() {
    Dbg.setGlobalLevel(LogLevel.DEBUG);
    Dbg.setIncludeLoopCount(true);
    Dbg.setGroupLevel(LogGroup.SENSORS, LogLevel.TRACE);  // Extra verbose
}
```

### Competition
```java
public static void setupCompetition() {
    Dbg.setGlobalLevel(LogLevel.INFO);  // Less noise
    Dbg.setIncludeLoopCount(false);
    Dbg.setGroupEnabled(LogGroup.TEST, false);  // Disable test logs
}
```

### Debugging Specific Subsystem
```java
public static void setupDebugIndexing() {
    Dbg.setGlobalLevel(LogLevel.WARN);  // Quiet everything else
    Dbg.setGroupLevel(LogGroup.INDEX, LogLevel.TRACE);    // Max detail
    Dbg.setGroupLevel(LogGroup.INTAKE, LogLevel.DEBUG);   // Some detail
    Dbg.setGroupLevel(LogGroup.TRANSFER, LogLevel.DEBUG); // Some detail
}
```

## Best Practices

### 1. Choose Appropriate Levels
- **TRACE**: Very detailed, inner loop, variable dumps
- **DEBUG**: Detailed flow, state changes, calculations
- **INFO**: Major events, initialization, user actions
- **WARN**: Recoverable issues, deprecated usage, unusual conditions
- **ERROR**: Failures, exceptions, critical problems

### 2. Use Meaningful Keys for Spam Control
```java
// Good: Descriptive, unique per call site
Dbg.everyMs(LogGroup.INTAKE, LogLevel.INFO, "artifact_status", 1000, ...);
Dbg.once(LogGroup.AUTO, LogLevel.WARN, "deprecated_path_v1", ...);

// Bad: Generic, could conflict
Dbg.everyMs(LogGroup.INTAKE, LogLevel.INFO, "status", 1000, ...);
Dbg.once(LogGroup.AUTO, LogLevel.WARN, "warning", ...);
```

### 3. Don't Log in Tight Inner Loops
```java
// Bad: Will spam or waste CPU even when disabled
for (int i = 0; i < 1000; i++) {
    Dbg.d(LogGroup.SYSTEM, "Processing %d", i);
}

// Good: Use counted or conditional
for (int i = 0; i < 1000; i++) {
    Dbg.counted(LogGroup.SYSTEM, LogLevel.TRACE, "process_loop", 100,
                "Processing %d", i);
}

// Or only log important events
for (int i = 0; i < 1000; i++) {
    if (important[i]) {
        Dbg.d(LogGroup.SYSTEM, "Important at index %d", i);
    }
}
```

### 4. Log State Transitions
```java
// Always log state changes at INFO or higher
private void setState(State newState) {
    if (this.state != newState) {
        Dbg.i(LogGroup.INDEX, "State: %s -> %s", this.state, newState);
        this.state = newState;
    }
}
```

### 5. Use Format Strings
```java
// Good: Efficient, no allocation if filtered
Dbg.d(LogGroup.DRIVE, "Position: x=%.2f y=%.2f", x, y);

// Bad: Always allocates strings
Dbg.d(LogGroup.DRIVE, "Position: x=" + x + " y=" + y);
```

## Troubleshooting

### "I don't see my logs"
1. Check log level: `Dbg.setGlobalLevel(LogLevel.TRACE)`
2. Check group enabled: `Dbg.setGroupEnabled(LogGroup.DRIVE, true)`
3. Check logcat filter: `adb logcat -s TeamDbg:*`
4. Dump config: `Dbg.dumpConfig()`

### "Too many logs"
1. Increase global level: `Dbg.setGlobalLevel(LogLevel.INFO)`
2. Disable noisy groups: `Dbg.setGroupEnabled(LogGroup.SENSORS, false)`
3. Use rate limiting: `Dbg.everyMs(...)` instead of `Dbg.d(...)`

### "Logs from wrong OpMode"
1. Make sure you call `Dbg.setContext()` in each OpMode
2. Call `Dbg.reset()` in `stop()` to clear state

## Performance Notes

- **Zero allocation when disabled**: If a log is filtered out by level or group, no strings are formatted
- **Thread-safe**: All spam control uses atomic operations
- **Efficient**: Uses `StringBuilder` for prefixes, `String.format` only when needed
- **FTC-optimized**: Uses RobotLog directly, no intermediate layers

## Summary

Replace this:
```java
System.out.println("IndexingSystem: State=" + state + " count=" + count);
```

With this:
```java
Dbg.d(LogGroup.INDEX, "State=%s count=%d", state, count);
```

And get:
- Filterable logs by level and subsystem
- Spam control built-in
- Proper FTC integration
- Better performance
- More maintainable code

---

**Need help?** Check `Dbg.dumpConfig()` to see current settings, or view the JavaDoc in `Dbg.java`.
