package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;
import org.firstinspires.ftc.teamcode.util.debug.LogLevel;

/**
 * Comprehensive example demonstrating all features of the Dbg logging utility.
 *
 * This OpMode shows:
 * 1. Basic logging at all levels
 * 2. Spam control (rate limiting, once, counted)
 * 3. Runtime configuration
 * 4. Per-group filtering
 * 5. Context and prefix management
 *
 * USAGE:
 * - Run this OpMode and view logs with: adb logcat -s TeamDbg:*
 * - Press buttons to see different log levels and spam control
 * - Use dpad to change log levels at runtime
 *
 * @author FTC Team 26581 Tundra Tech
 */
@TeleOp(name="Dbg Logging Demo", group="Test")
@Disabled
public class DbgLoggingExample extends LinearOpMode {

    // Edge detection for buttons
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastStart = false;

    // Demo state
    private int demoCounter = 0;
    private boolean indexingEnabled = true;
    private boolean driveEnabled = true;
    private boolean panelsTelemetryEnabled = false;

    @Override
    public void runOpMode() {
        // ==================== INITIALIZATION ====================

        // Configure logging
        telemetry.addData("Status", "Configuring Dbg logger...");
        telemetry.update();

        Dbg.setGlobalPrefix("DemoBot");
        Dbg.setContext("DbgDemo", "INIT");
        Dbg.setGlobalLevel(LogLevel.DEBUG);
        Dbg.setIncludeLoopCount(true);
        Dbg.resetLoop();

        // Optional: Enable telemetry output (INFO and above only)
        Dbg.setEnablePanelsTelemetry(false);  // Start disabled, toggle with Start button
        Dbg.setMaxTelemetryLines(5);

        // Optional: configure specific groups
        Dbg.setGroupLevel(LogGroup.SENSORS, LogLevel.TRACE);  // Extra verbose
        Dbg.setGroupLevel(LogGroup.VISION, LogLevel.WARN);    // Less noisy

        // Log initialization
        Dbg.i(LogGroup.OPMODE, "OpMode initializing");
        Dbg.d(LogGroup.HARDWARE, "Configuring hardware...");

        // Demonstrate all log levels during init
        Dbg.t(LogGroup.OPMODE, "TRACE: Most verbose, for deep debugging");
        Dbg.d(LogGroup.OPMODE, "DEBUG: Detailed information");
        Dbg.i(LogGroup.OPMODE, "INFO: General information");
        Dbg.w(LogGroup.OPMODE, "WARN: Warning message");
        Dbg.e(LogGroup.OPMODE, "ERROR: Error message (not a real error!)");

        // Show formatted logging
        double voltage = 12.8;
        int artifactCount = 0;
        Dbg.d(LogGroup.HARDWARE, "Battery voltage: %.1fV", voltage);
        Dbg.d(LogGroup.INDEXING, "Initial artifact count: %d", artifactCount);

        // Demonstrate once-only logging
        Dbg.once(LogGroup.OPMODE, LogLevel.INFO, "init_once",
                 "This message appears only once per OpMode run");
        Dbg.once(LogGroup.OPMODE, LogLevel.INFO, "init_once",
                 "This duplicate won't appear");

        // Dump configuration for debugging
        Dbg.dumpConfig();

        telemetry.addData("Status", "Configured! Check logcat");
        telemetry.addData("Logcat", "adb logcat -s TeamDbg:*");
        telemetry.addLine();
        telemetry.addData("Controls", "See messages below");
        updateTelemetry();

        Dbg.i(LogGroup.OPMODE, "Waiting for start...");
        waitForStart();

        // ==================== START ====================

        Dbg.setContext("DbgDemo", "RUN");
        Dbg.i(LogGroup.OPMODE, "OpMode started!");

        // ==================== MAIN LOOP ====================

        while (opModeIsActive()) {
            Dbg.incrementLoop();  // Increment loop counter

            // Handle controls
            handleControls();

            // Demonstrate different logging patterns
            demonstrateLogging();

            // Update telemetry
            updateTelemetry();
        }

        // Cleanup when OpMode ends
        Dbg.setContext("DbgDemo", "STOP");
        Dbg.i(LogGroup.OPMODE, "OpMode stopping, ran %d loops", demoCounter);
        Dbg.reset();
    }

    /**
     * Handle gamepad controls for runtime configuration.
     */
    private void handleControls() {
        // A: Log all levels
        if (gamepad1.a && !lastA) {
            Dbg.t(LogGroup.TEST, "TRACE level message");
            Dbg.d(LogGroup.TEST, "DEBUG level message");
            Dbg.i(LogGroup.TEST, "INFO level message");
            Dbg.w(LogGroup.TEST, "WARN level message");
            Dbg.e(LogGroup.TEST, "ERROR level message");
        }
        lastA = gamepad1.a;

        // B: Demonstrate formatted logging
        if (gamepad1.b && !lastB) {
            double speed = Math.random();
            int count = (int)(Math.random() * 10);
            String state = "ACTIVE";

            Dbg.d(LogGroup.DRIVE, "Speed: %.3f m/s", speed);
            Dbg.d(LogGroup.INDEXING, "Artifacts: %d, State: %s", count, state);
            Dbg.d(LogGroup.SENSORS, "X: %.2f, Y: %.2f, H: %.1f°",
                  Math.random() * 100, Math.random() * 100, Math.random() * 360);
        }
        lastB = gamepad1.b;

        // X: Demonstrate exception logging
        if (gamepad1.x && !lastX) {
            try {
                throw new RuntimeException("Simulated exception for demo");
            } catch (Exception e) {
                Dbg.e(LogGroup.HARDWARE, e, "Caught exception during operation");
            }
        }
        lastX = gamepad1.x;

        // Y: Test once-only logging
        if (gamepad1.y && !lastY) {
            Dbg.once(LogGroup.TEST, LogLevel.INFO, "button_y",
                     "This message from Y button appears only once");
        }
        lastY = gamepad1.y;

        // Dpad Up: Decrease global level (more logs)
        if (gamepad1.dpad_up && !lastDpadUp) {
            LogLevel current = Dbg.getGlobalLevel();
            LogLevel newLevel = decreaseLevel(current);
            Dbg.setGlobalLevel(newLevel);
            Dbg.i(LogGroup.OPMODE, "Log level: %s -> %s (more logs)", current, newLevel);
        }
        lastDpadUp = gamepad1.dpad_up;

        // Dpad Down: Increase global level (fewer logs)
        if (gamepad1.dpad_down && !lastDpadDown) {
            LogLevel current = Dbg.getGlobalLevel();
            LogLevel newLevel = increaseLevel(current);
            Dbg.setGlobalLevel(newLevel);
            Dbg.i(LogGroup.OPMODE, "Log level: %s -> %s (fewer logs)", current, newLevel);
        }
        lastDpadDown = gamepad1.dpad_down;

        // Dpad Left: Toggle INDEX group
        if (gamepad1.dpad_left && !lastDpadLeft) {
            indexingEnabled = !indexingEnabled;
            Dbg.setGroupEnabled(LogGroup.INDEXING, indexingEnabled);
            Dbg.i(LogGroup.OPMODE, "INDEX group %s", indexingEnabled ? "ENABLED" : "DISABLED");
        }
        lastDpadLeft = gamepad1.dpad_left;

        // Dpad Right: Toggle DRIVE group
        if (gamepad1.dpad_right && !lastDpadRight) {
            driveEnabled = !driveEnabled;
            Dbg.setGroupEnabled(LogGroup.DRIVE, driveEnabled);
            Dbg.i(LogGroup.OPMODE, "DRIVE group %s", driveEnabled ? "ENABLED" : "DISABLED");
        }
        lastDpadRight = gamepad1.dpad_right;

        // Left Bumper: Dump configuration
        if (gamepad1.left_bumper && !lastLeftBumper) {
            Dbg.i(LogGroup.OPMODE, "Dumping configuration...");
            Dbg.dumpConfig();
        }
        lastLeftBumper = gamepad1.left_bumper;

        // Right Bumper: Reset spam control state
        if (gamepad1.right_bumper && !lastRightBumper) {
            Dbg.i(LogGroup.OPMODE, "Resetting spam control (once/counted/everyMs)");
            Dbg.reset();
            Dbg.setContext("DbgDemo", "RUN");  // Restore context
        }
        lastRightBumper = gamepad1.right_bumper;

        // Start: Toggle Panels telemetry output
        if (gamepad1.start && !lastStart) {
            panelsTelemetryEnabled = !panelsTelemetryEnabled;
            Dbg.setEnablePanelsTelemetry(panelsTelemetryEnabled);
            Dbg.i(LogGroup.OPMODE, "Panels telemetry %s",
                  panelsTelemetryEnabled ? "ENABLED" : "DISABLED");
        }
        lastStart = gamepad1.start;
    }

    /**
     * Demonstrate different logging patterns every loop.
     */
    private void demonstrateLogging() {
        demoCounter++;

        // 1. Rate-limited logging (once per second)
        Dbg.everyMs(LogGroup.OPMODE, LogLevel.INFO, "heartbeat", 1000,
                    "Loop %d, Counter: %d", Dbg.getGlobalLevel(), demoCounter);

        // 2. Counted logging (every 50 loops)
        Dbg.counted(LogGroup.SENSORS, LogLevel.DEBUG, "sensor_sample", 50,
                    "Sampled sensor reading: %.2f", Math.random() * 1000);

        // 3. Multiple rate-limited logs (different keys, different rates)
        Dbg.everyMs(LogGroup.INDEXING, LogLevel.DEBUG, "index_status", 2000,
                    "Indexing: enabled=%b, artifacts=%d",
                    indexingEnabled, (int)(Math.random() * 4));

        Dbg.everyMs(LogGroup.DRIVE, LogLevel.DEBUG, "drive_status", 3000,
                    "Drive: enabled=%b, speed=%.2f",
                    driveEnabled, Math.random());

        // 4. Conditional logging (only when something interesting happens)
        if (demoCounter % 100 == 0) {
            Dbg.i(LogGroup.OPMODE, "Milestone: %d loops completed", demoCounter);
        }

        // 5. Simulated subsystem logging
        if (indexingEnabled) {
            // This would spam without rate limiting
            Dbg.everyMs(LogGroup.INDEXING, LogLevel.TRACE, "index_trace", 5000,
                        "Index trace: detailed internal state");
        }

        if (driveEnabled) {
            // Counted sampling
            Dbg.counted(LogGroup.DRIVE, LogLevel.TRACE, "drive_trace", 100,
                        "Drive trace: motor positions");
        }

        // 6. Simulated sensor readings (very noisy without control)
        Dbg.counted(LogGroup.SENSORS, LogLevel.TRACE, "color_sensor", 200,
                    "Color: R=%.2f G=%.2f B=%.2f",
                    Math.random(), Math.random(), Math.random());

        // 7. Simulated vision processing
        Dbg.everyMs(LogGroup.VISION, LogLevel.DEBUG, "vision_update", 5000,
                    "AprilTag detected: ID=%d, distance=%.1fcm",
                    (int)(Math.random() * 10), Math.random() * 100);
    }

    /**
     * Update telemetry with controls and status.
     */
    private void updateTelemetry() {
        telemetry.addData("Loop", demoCounter);
        telemetry.addData("Log Level", Dbg.getGlobalLevel());
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A", "Log all levels");
        telemetry.addData("B", "Formatted logging");
        telemetry.addData("X", "Exception logging");
        telemetry.addData("Y", "Once-only log");
        telemetry.addLine();
        telemetry.addData("Dpad ↑", "More logs (decrease level)");
        telemetry.addData("Dpad ↓", "Fewer logs (increase level)");
        telemetry.addData("Dpad ←", "Toggle INDEX (" + (indexingEnabled ? "ON" : "OFF") + ")");
        telemetry.addData("Dpad →", "Toggle DRIVE (" + (driveEnabled ? "ON" : "OFF") + ")");
        telemetry.addLine();
        telemetry.addData("LB", "Dump config");
        telemetry.addData("RB", "Reset spam control");
        telemetry.addData("Start", "Toggle Panels (" + (panelsTelemetryEnabled ? "ON" : "OFF") + ")");
        telemetry.addLine();

        telemetry.addLine("=== STATUS ===");
        telemetry.addData("INDEX group", indexingEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("DRIVE group", driveEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Panels output", panelsTelemetryEnabled ? "ENABLED" : "DISABLED");
        telemetry.addLine();

        telemetry.addLine("View logs:");
        telemetry.addData("Command", "adb logcat -s TeamDbg:*");

        telemetry.update();
    }

    /**
     * Decrease log level (more verbose).
     */
    private LogLevel decreaseLevel(LogLevel current) {
        switch (current) {
            case ERROR: return LogLevel.WARN;
            case WARN: return LogLevel.INFO;
            case INFO: return LogLevel.DEBUG;
            case DEBUG: return LogLevel.TRACE;
            case TRACE: return LogLevel.TRACE;  // Already minimum
            default: return LogLevel.DEBUG;
        }
    }

    /**
     * Increase log level (less verbose).
     */
    private LogLevel increaseLevel(LogLevel current) {
        switch (current) {
            case TRACE: return LogLevel.DEBUG;
            case DEBUG: return LogLevel.INFO;
            case INFO: return LogLevel.WARN;
            case WARN: return LogLevel.ERROR;
            case ERROR: return LogLevel.ERROR;  // Already maximum
            default: return LogLevel.INFO;
        }
    }
}
