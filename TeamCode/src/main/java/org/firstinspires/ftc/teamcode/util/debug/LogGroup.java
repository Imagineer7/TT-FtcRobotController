package org.firstinspires.ftc.teamcode.util.debug;

/**
 * Predefined log groups for FTC subsystems.
 * Use these to categorize logs by robot component.
 *
 * Organized by functional area for the Aurora System v2.
 * Custom groups can also be created using strings in Dbg methods.
 *
 * Suggested defaults:
 * - Enabled (INFO+): OPMODE, SAFETY, INDEXING, FIRING, SHOTPLAN, PLANNEREX
 * - Rate-limited (DEBUG): DRIVE, LOCALIZE, SENSORS, VISION, KEEPALIVE
 * - Off by default: LOOP, MONITOR, DBG, TEST
 */
public enum LogGroup {
    // ==================== Core Robot / Runtime ====================
    OPMODE("OPMODE"),      // OpMode lifecycle, init/start/stop, mode transitions
    LOOP("LOOP"),          // Loop timing, cycle rate, overruns, heartbeat
    SAFETY("SAFETY"),      // Deadman switch, inhibit reasons, interlocks

    // ==================== Aurora / Game-Specific System ====================
    AURORA("AURORA"),      // General Aurora package coordination (high-level)
    ARTIFACT("ARTIFACT"),  // Artifact identity/tracking/ledger changes
    INDEXING("INDEX"),     // Indexing state machine, slot occupancy changes
    INTAKE("INTAKE"),      // Intake roller logic, collection triggers
    TRANSFER("TRANSFER"),  // Moving artifacts between locations
    FIRING("FIRING"),      // Firing sequences, feed timing
    EJECT("EJECT"),        // Eject/reject flows
    SHOTPLAN("SHOTPLAN"),  // Shot plan creation/consumption
    PLANNEREX("PLANEX"),   // Planner/executor coordination, operation runner
    KEEPALIVE("KEEPALIVE"),// Watchdog + keep-alive integration

    // ==================== Mechanisms ====================
    SHOOTER("SHOOTER"),    // Shooter control, RPM/ready checks
    TURRET("TURRET"),      // Turret targeting/limits
    DRIVE("DRIVE"),        // Drivetrain commands
    LOCALIZE("LOCALIZE"),  // Localization math, pose updates
    PATHING("PATHING"),    // PedroPathing / autonomous path builder

    // ==================== Sensors / Vision ====================
    SENSORS("SENSORS"),    // Raw sensor reads, calibration, sanity checks
    VISION("VISION"),      // Limelight and perception

    // ==================== Utilities / Debugging ====================
    MONITOR("MONITOR"),    // SystemMonitor panel summaries
    DBG("DBG"),            // General debugging
    TEST("TEST"),          // Test OpModes and harness logs

    // ==================== Legacy / Compatibility ====================
    AUTO("AUTO"),          // Autonomous (deprecated, use OPMODE + PATHING)
    HARDWARE("HW"),        // Hardware initialization (use OPMODE)
    SYSTEM("SYS");         // General system (use OPMODE or AURORA)

    private final String tag;

    LogGroup(String tag) {
        this.tag = tag;
    }

    public String getTag() {
        return tag;
    }

    @Override
    public String toString() {
        return tag;
    }
}
