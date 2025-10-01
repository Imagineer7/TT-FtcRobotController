/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Enhanced TeleOp demonstration with all advanced systems
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AuroraManager; //AURORA - Advanced Unified Robot Operating & Response Architecture
import org.firstinspires.ftc.teamcode.util.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.SmartMechanumDrive;
import org.firstinspires.ftc.teamcode.util.SmartTelemetryManager;

/**
 * AURORATeleOp - Enhanced TeleOp with dual-driver mode support and smart telemetry
 * <p>
 * Control Scheme:
 * <p>
 * Single Driver Mode (Default):
 * Gamepad 1: All controls
 * - Movement: Left stick (forward/strafe), Right stick (rotate)
 * - Fine Movement: D-pad (fine XY at 20% power), Bumpers (fine rotation at 20% power)
 * - Drive modes: Left stick button (precision), X (field relative)
 * - Semi-auto: Y (basket+shoot), B (precision), A (retreat), triggers (rotate/align)
 * - Shooting: A (single), Y (continuous), Right trigger (manual), B (feed), X (stop)
 * - System: Back + D-pad Left (toggle driver mode), D-pad Right (cycle telemetry pages)
 * <p>
 * Dual Driver Mode:
 * Gamepad 1 (Driver - Movement & Semi-auto):
 * - Movement: Left stick (forward/strafe), Right stick (rotate)
 * - Fine Movement: D-pad (fine XY at 20% power), Bumpers (fine rotation at 20% power)
 * - Drive modes: Left stick button (precision), X (field relative)
 * - Semi-auto: Y (basket+shoot), B (precision), A (retreat), triggers (rotate/align)
 * - System: Back + D-pad Left (toggle driver mode), Back (cancel semi-auto), D-pad Right (cycle telemetry)
 * <p>
 * Gamepad 2 (Operator - Shooting only):
 * - Range Selection: Right bumper (long range), Left bumper (short range)
 * - A: Single shot (uses selected range)
 * - Y: Continuous shooting (rapid fire)
 * - Right trigger: Manual shooter power control
 * - B: Manual feed servo control / Reset stats
 * - X: Emergency stop shooting
 * <p>
 * Smart Telemetry Pages:
 * - Overview: System status & critical info
 * - Drive: Movement & navigation systems
 * - Shooter: Shooting system & performance
 * - Performance: System metrics & diagnostics
 * - Controls: Control mapping & help
 * <p>
 * Emergency Stop: Both Start buttons (gamepad1.start && gamepad2.start)
 */
@TeleOp(name="AURORA Enhanced TeleOp", group="Competition")
public class AURORATeleOp extends LinearOpMode {

    private AuroraManager robotManager;
    private SmartTelemetryManager smartTelemetry;
    private final ElapsedTime runtime = new ElapsedTime();

    // Control state tracking
    private boolean prevPrecisionToggle = false;
    private boolean prevStatsReset = false;

    // Emergency stop tracking
    private boolean emergencyStop = false;

    @Override
    public void runOpMode() {
        // Initialize the robot systems
        telemetry.addLine("Initializing AURORA Enhanced Robot Systems...");
        telemetry.update();

        // Initialize the unified robot system manager
        robotManager = new AuroraManager(hardwareMap, telemetry);

        // Initialize the smart telemetry manager
        smartTelemetry = new SmartTelemetryManager(telemetry, robotManager);

        telemetry.clear();
        telemetry.addLine("✅ AURORA Enhanced TeleOp Ready!");
        telemetry.addLine("");
        telemetry.addData("Driver Mode", robotManager.getDriverModeString());
        telemetry.addLine("");
        telemetry.addLine("=== SMART TELEMETRY SYSTEM ===");
        telemetry.addLine("📊 5 Focused telemetry pages:");
        telemetry.addLine("• Overview: System status");
        telemetry.addLine("• Drive: Movement systems");
        telemetry.addLine("• Shooter: Shooting performance");
        telemetry.addLine("• Performance: Diagnostics");
        telemetry.addLine("• Controls: Button mapping");
        telemetry.addLine("");
        telemetry.addLine("=== KEY CONTROLS ===");
        telemetry.addLine("• X (GP1): Cycle telemetry pages");
        telemetry.addLine("• Back + D-pad Left: Toggle driver mode");
        telemetry.addLine("• Y (GP1): Quick precision toggle");
        telemetry.addLine("• B (GP2): Reset performance stats");
        telemetry.addLine("• Start + Start: Emergency stop");
        telemetry.addLine("");
        telemetry.addLine("⚡ Optimized for minimal lag!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Update all robot systems
            robotManager.update(gamepad1, gamepad2);

            // Handle special controls
            handleStatReset();
            handlePrecisionToggle();
            handleEmergencyStop();

            // Handle telemetry page cycling through smart telemetry manager
            smartTelemetry.handlePageCycling(gamepad1.x);

            // Update smart telemetry display
            smartTelemetry.updateDisplay(emergencyStop);
        }

        // Graceful shutdown when OpMode ends
        if (robotManager.getShooterSystem() != null) {
            robotManager.getShooterSystem().reset();
        }

        telemetry.clear();
        telemetry.addLine("AURORA Enhanced TeleOp stopped safely");
        telemetry.addLine("Smart telemetry system optimized performance!");
        telemetry.update();
    }

    /**
     * Handle performance stats reset
     */
    private void handleStatReset() {
        boolean currentB = gamepad2.b;
        if (currentB && !prevStatsReset) {
            if (robotManager.getDriveSystem() != null) {
                robotManager.getDriveSystem().resetAnalytics();
            }
            if (robotManager.getGlobalMonitor() != null) {
                robotManager.getGlobalMonitor().reset();
            }
            // Briefly show performance page to confirm reset
            smartTelemetry.showPerformance();
        }
        prevStatsReset = currentB;
    }

    /**
     * Handle quick precision mode toggle
     */
    private void handlePrecisionToggle() {
        boolean currentY = gamepad1.y;
        if (currentY && !prevPrecisionToggle) {
            // Toggle quick precision mode
            if (robotManager.getDriveSystem() != null) {
                // Toggle between PRECISION and NORMAL modes
                if (robotManager.getDriveSystem().getCurrentMode() == SmartMechanumDrive.DriveMode.PRECISION) {
                    robotManager.getDriveSystem().setCurrentMode(SmartMechanumDrive.DriveMode.NORMAL);
                } else {
                    robotManager.getDriveSystem().setCurrentMode(SmartMechanumDrive.DriveMode.PRECISION);
                }
                // Briefly show drive page to confirm mode change
                smartTelemetry.showDrive();
            }
        }
        prevPrecisionToggle = currentY;
    }

    /**
     * Handle emergency stop
     */
    private void handleEmergencyStop() {
        boolean currentStart1 = gamepad1.start;
        boolean currentStart2 = gamepad2.start;

        if (currentStart1 && currentStart2) {
            emergencyStop = true;
        }

        if (emergencyStop) {
            // Engage emergency stop procedures
            if (robotManager.getShooterSystem() != null) {
                robotManager.getShooterSystem().emergencyStop();
            }

            // Use the AuroraManager's emergency stop which handles all systems
            robotManager.emergencyStopAll();

            // Smart telemetry will automatically show emergency status
        }

        // Cancel emergency stop if Back is pressed
        if (gamepad1.back || gamepad2.back) {
            emergencyStop = false;
        }
    }
}
