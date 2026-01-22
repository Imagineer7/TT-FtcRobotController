// ============================================================
// Dbg Quick Reference Card
// ============================================================

// SETUP (once in OpMode init)
Dbg.setGlobalPrefix("RobotName");
Dbg.setContext("TeleOp", "INIT");
Dbg.setGlobalLevel(LogLevel.DEBUG);
Dbg.setIncludeLoopCount(true);
Dbg.resetLoop();

// OPTIONAL: Enable Panels telemetry output (INFO/WARN/ERROR → Dashboard)
Dbg.setEnablePanelsTelemetry(true);  // Enable
Dbg.setMaxTelemetryLines(5);         // Limit lines

// IN LOOP
while (opModeIsActive()) {
    Dbg.incrementLoop();
    // ... your code ...
}

// CLEANUP (at end of runOpMode)
Dbg.reset();

// ============================================================
// BASIC LOGGING
// ============================================================

// Simple message
Dbg.d(LogGroup.DRIVE, "Motor started");

// Formatted (printf-style)
Dbg.d(LogGroup.DRIVE, "Speed: %.2f, Heading: %.1f°", speed, heading);

// Different levels
Dbg.t(LogGroup.SENSORS, "Raw: %d", raw);         // TRACE (most verbose)
Dbg.d(LogGroup.DRIVE, "Position: %.2f", pos);    // DEBUG
Dbg.i(LogGroup.AUTO, "Starting auto");           // INFO
Dbg.w(LogGroup.HARDWARE, "Low voltage: %.1fV");  // WARNin
Dbg.e(LogGroup.INTAKE, "Motor not found");       // ERROR (highest)

// With exception
try { ... } catch (Exception e) {
    Dbg.e(LogGroup.HARDWARE, e, "Failed to initialize");
}

// ============================================================
// SPAM CONTROL
// ============================================================

// Rate limit (max once per 1000ms)
Dbg.everyMs(LogGroup.INTAKE, LogLevel.INFO, "status", 1000,
            "Artifacts: %d", count);

// Log only once per OpMode run
Dbg.once(LogGroup.AUTO, LogLevel.WARN, "deprecated",
         "Using old path");

// Log every Nth call (every 10th)
Dbg.counted(LogGroup.SENSORS, LogLevel.DEBUG, "odom", 10,
            "X=%.2f Y=%.2f", x, y);

// ============================================================
// FILTERING
// ============================================================

// Global level (affects all groups)
Dbg.setGlobalLevel(LogLevel.INFO);    // Default
Dbg.setGlobalLevel(LogLevel.TRACE);   // Most verbose
Dbg.setGlobalLevel(LogLevel.WARN);    // Competition (quiet)

// Per-group level (overrides global)
Dbg.setGroupLevel(LogGroup.INDEX, LogLevel.TRACE);  // Extra verbose
Dbg.setGroupLevel(LogGroup.VISION, LogLevel.ERROR); // Quiet

// Disable entire group
Dbg.setGroupEnabled(LogGroup.VISION, false);

// ============================================================
// PREDEFINED GROUPS
// ============================================================

// CORE ROBOT / RUNTIME
LogGroup.OPMODE     // OpMode lifecycle, init/start/stop, mode transitions
LogGroup.LOOP       // Loop timing, cycle rate, overruns, heartbeat
LogGroup.SAFETY     // Deadman switch, inhibit reasons, interlocks

// AURORA / GAME-SPECIFIC SYSTEM
LogGroup.AURORA     // General Aurora coordination (high-level)
LogGroup.ARTIFACT   // Artifact identity/tracking/ledger changes
LogGroup.INDEXING   // Indexing state machine, slot occupancy
LogGroup.INTAKE     // Intake roller logic, collection triggers
LogGroup.TRANSFER   // Moving artifacts between locations
LogGroup.FIRING     // Firing sequences, feed timing
LogGroup.EJECT      // Eject/reject flows
LogGroup.SHOTPLAN   // Shot plan creation/consumption
LogGroup.PLANNEREX  // Planner/executor coordination
LogGroup.KEEPALIVE  // Watchdog + keep-alive integration

// MECHANISMS
LogGroup.SHOOTER    // Shooter control, RPM/ready checks
LogGroup.TURRET     // Turret targeting/limits
LogGroup.DRIVE      // Drivetrain commands
LogGroup.LOCALIZE   // Localization math, pose updates
LogGroup.PATHING    // PedroPathing / autonomous builder

// SENSORS / VISION
LogGroup.SENSORS    // Raw sensor reads, calibration, sanity checks
LogGroup.VISION     // Limelight and perception

// UTILITIES / DEBUGGING
LogGroup.MONITOR    // SystemMonitor panel summaries
LogGroup.DBG        // General debugging
LogGroup.TEST       // Test OpModes and harness logs

// LEGACY (use new groups instead)
LogGroup.AUTO       // Autonomous (→ use OPMODE + PATHING)
LogGroup.HARDWARE   // Hardware init (→ use OPMODE)
LogGroup.SYSTEM     // General system (→ use OPMODE or AURORA)

// Or use custom string
Dbg.d("CLIMBER", "Position: %d", pos);

// ============================================================
// PANELS TELEMETRY OUTPUT
// ============================================================

// Enable Panels output (INFO/WARN/ERROR → FTC Dashboard)
Dbg.setEnablePanelsTelemetry(true);
Dbg.setMaxTelemetryLines(5);

// Only INFO and above go to Panels
Dbg.d(LogGroup.SENSORS, "Sensor: %d", val);  // logcat only (DEBUG)
Dbg.i(LogGroup.INDEX, "Artifact detected");   // logcat + Panels (INFO)
Dbg.w(LogGroup.HARDWARE, "Low voltage!");     // logcat + Panels (WARN)
Dbg.e(LogGroup.DRIVE, "Motor disconnected");  // logcat + Panels (ERROR)

// Disable for competition (performance)
Dbg.setEnablePanelsTelemetry(false);

// Toggle at runtime
boolean panelsEnabled = !panelsEnabled;
Dbg.setEnablePanelsTelemetry(panelsEnabled);

// ============================================================
// LOGCAT COMMANDS
// ============================================================

// View all logs
adb logcat -s TeamDbg:*

// Clear first
adb logcat -c && adb logcat -s TeamDbg:*

// Filter by level
adb logcat -s TeamDbg:W              // Warnings & errors
adb logcat -s TeamDbg:I              // Info & above

// Filter by group (grep)
adb logcat -s TeamDbg:* | grep "INDEX/"
adb logcat -s TeamDbg:* | grep -E "(INDEX|INTAKE|FIRE)/"

// Filter by OpMode
adb logcat -s TeamDbg:* | grep "\[TeleOp"

// Find errors
adb logcat -s TeamDbg:* | grep "/E:"

// Save to file
adb logcat -s TeamDbg:* > log.txt

// ============================================================
// SUBSYSTEM PATTERN
// ============================================================

public class MySubsystem {
    private static final LogGroup LOG_GROUP = LogGroup.INDEX;

    public void update() {
        // Rate-limited status
        Dbg.everyMs(LOG_GROUP, LogLevel.DEBUG, "status", 1000,
                    "State: %s, Count: %d", state, count);

        switch (state) {
            case IDLE:
                Dbg.t(LOG_GROUP, "Waiting for input");
                break;
            case ACTIVE:
                Dbg.d(LOG_GROUP, "Processing...");
                break;
            case ERROR:
                Dbg.e(LOG_GROUP, "Error: %s", errorMsg);
                break;
        }
    }

    private void setState(State newState) {
        // Always log state transitions
        Dbg.i(LOG_GROUP, "State: %s -> %s", state, newState);
        state = newState;
    }
}

// ============================================================
// CONFIGURATION PRESETS
// ============================================================

// Development (all enabled, verbose)
Dbg.setGlobalLevel(LogLevel.DEBUG);
Dbg.setIncludeLoopCount(true);
Dbg.setEnablePanelsTelemetry(true);

// Competition (minimal, critical only)
Dbg.setGlobalLevel(LogLevel.INFO);
Dbg.setGroupEnabled(LogGroup.TEST, false);
Dbg.setGroupEnabled(LogGroup.LOOP, false);
Dbg.setGroupEnabled(LogGroup.MONITOR, false);
Dbg.setGroupEnabled(LogGroup.DBG, false);
Dbg.setEnablePanelsTelemetry(false);

// Aurora Recommended Defaults (starter)
// Enabled by default (INFO+)
Dbg.setGlobalLevel(LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.OPMODE, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.SAFETY, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.INDEXING, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.FIRING, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.SHOTPLAN, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.PLANNEREX, LogLevel.INFO);

// Rate-limited (DEBUG) - use with everyMs()
Dbg.setGroupLevel(LogGroup.DRIVE, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.LOCALIZE, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.SENSORS, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.VISION, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.KEEPALIVE, LogLevel.DEBUG);

// Off by default (enable when debugging)
Dbg.setGroupEnabled(LogGroup.LOOP, false);
Dbg.setGroupEnabled(LogGroup.MONITOR, false);
Dbg.setGroupEnabled(LogGroup.DBG, false);
Dbg.setGroupEnabled(LogGroup.TEST, false);

// Debug specific subsystem
Dbg.setGlobalLevel(LogLevel.WARN);                         // Quiet everything
Dbg.setGroupLevel(LogGroup.INDEXING, LogLevel.TRACE);      // Except INDEXING
Dbg.setGroupLevel(LogGroup.ARTIFACT, LogLevel.TRACE);      // And ARTIFACT
Dbg.setGroupLevel(LogGroup.PLANNEREX, LogLevel.DEBUG);     // And PLANNEREX

// Debug firing sequence issues
Dbg.setGlobalLevel(LogLevel.WARN);                         // Quiet
Dbg.setGroupLevel(LogGroup.FIRING, LogLevel.TRACE);        // Firing details
Dbg.setGroupLevel(LogGroup.SHOOTER, LogLevel.DEBUG);       // Shooter RPM
Dbg.setGroupLevel(LogGroup.SHOTPLAN, LogLevel.DEBUG);      // Plan creation

// ============================================================
// DEBUGGING THE LOGGER
// ============================================================

Dbg.dumpConfig();  // Print current configuration to logcat

// ============================================================
// MIGRATION
// ============================================================

// Before:
System.out.println("State: " + state + ", count: " + count);

// After:
Dbg.d(LogGroup.INDEX, "State: %s, count: %d", state, count);

// Benefits: Filterable, faster, consistent, FTC-integrated

// ============================================================
// FILES
// ============================================================

// Core library:
//   /util/debug/Dbg.java          - Main class
//   /util/debug/LogLevel.java     - Enum (TRACE..ERROR)
//   /util/debug/LogGroup.java     - Enum (INDEX, DRIVE, etc)

// Documentation:
//   TeamCode/DBG_LOGGING_GUIDE.md - Full guide
//   TeamCode/DBG_IMPLEMENTATION_SUMMARY.md - Summary

// Example:
//   /opmodes/test/DbgLoggingExample.java - Interactive demo

// ============================================================
