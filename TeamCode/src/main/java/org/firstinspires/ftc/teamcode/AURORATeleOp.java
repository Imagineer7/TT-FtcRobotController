/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Enhanced TeleOp demonstration with all advanced systems
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager; //AURORA - Advanced Unified Robot Operating & Response Architecture
import org.firstinspires.ftc.teamcode.util.aurora.RobotLiftController;
import org.firstinspires.ftc.teamcode.util.aurora.SmartMechanumDrive;
import org.firstinspires.ftc.teamcode.util.aurora.SmartTelemetryManager;

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
 * Gamepad 2 (Operator - Shooting + ML Controls + Lift):
 * - Left trigger: Warmup mode (spins shooter at 65% RPM to save power)
 * - A: Single shot (uses selected range)
 * - Y: Continuous shooting (rapid fire)
 * - Right trigger: Manual shooter power control
 * - B: Manual feed servo control / Reset stats
 * - X: Emergency stop shooting
 * - Left Bumper: Save ML learning data
 * - Right Bumper (hold 2s): Reset ML to defaults
 * - Right Stick Y: Manual lift control (up/down)
 * - D-pad Up: Move lift to HIGH position
 * - D-pad Right: Move lift to MID position
 * - D-pad Down: Move lift to LOW position
 * - D-pad Left: Move lift to GROUND position
 * <p>
 * Smart Telemetry Pages:
 * - Overview: System status & critical info
 * - Drive: Movement & navigation systems
 * - Shooter: Shooting system & performance
 * - Performance: System metrics & diagnostics (includes ML data)
 * - Controls: Control mapping & help
 * <p>
 * Emergency Stop: Both Start buttons (gamepad1.start && gamepad2.start)
 */
@TeleOp(name="AURORA Enhanced TeleOp", group="Competition")
@Configurable // Panels
public class AURORATeleOp extends LinearOpMode {

    private AuroraManager robotManager;
    private SmartTelemetryManager smartTelemetry;
    private RobotLiftController liftController;
    private final ElapsedTime runtime = new ElapsedTime();

    // Panels field visualization
    private FieldManager panelsField;
    private static final Style robotStyle = new Style("", "#00FF00", 0.75);
    private static final Style headingStyle = new Style("", "#00FF00", 0.75);
    private static final double ROBOT_RADIUS = 9.0;

    // Control state tracking
    private boolean prevPrecisionToggle = false;
    private boolean prevStatsReset = false;
    private boolean prevMlSave = false;
    private double mlResetButtonHoldStart = 0;
    private boolean mlResetButtonHeld = false;

    // Lift control state tracking
    private boolean prevLiftHighButton = false;
    private boolean prevLiftMidButton = false;
    private boolean prevLiftLowButton = false;
    private boolean prevLiftGroundButton = false;

    // Emergency stop tracking
    private boolean emergencyStop = false;

    private boolean cycleTelemetry = false;

    @Override
    public void runOpMode() {
        // Initialize the robot systems
        telemetry.addLine("Initializing AURORA Robot Systems...");
        telemetry.update();

        // Initialize Panels field visualization
        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        // Initialize the unified robot system manager
        robotManager = new AuroraManager(hardwareMap, telemetry);

        // Initialize lift controller
        liftController = new RobotLiftController(hardwareMap, telemetry);
        liftController.initialize();

        // Initialize the smart telemetry manager (pass lift controller for telemetry display)
        smartTelemetry = new SmartTelemetryManager(telemetry, robotManager, liftController);

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
        telemetry.addLine("• LB (GP2): Save ML learning data");
        telemetry.addLine("• RB (GP2, hold 2s): Reset ML data");
        telemetry.addLine("• Start + Start: Emergency stop");
        telemetry.addLine("");
        telemetry.addLine("=== LIFT CONTROLS (GP2) ===");
        telemetry.addLine("• Right Stick Y: Manual lift control");
        telemetry.addLine("• D-pad Up: Move to HIGH position");
        telemetry.addLine("• D-pad Right: Move to MID position");
        telemetry.addLine("• D-pad Down: Move to LOW position");
        telemetry.addLine("• D-pad Left: Move to GROUND position");
        telemetry.addLine("");
        telemetry.addLine("⚡ Optimized for minimal lag!");
        telemetry.addLine("🤖 ML System: Auto-learning PID gains");
        telemetry.addLine("🎯 Lift: Auto PID gravity compensation");
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
            handleMlControls();
            handleLiftControls();
            handleEmergencyStop();

            cycleTelemetry = gamepad1.x || gamepad2.dpad_left;

            // Handle telemetry page cycling through smart telemetry manager
            smartTelemetry.handlePageCycling(cycleTelemetry);

            // Update Panels field visualization with odometry data
            updateFieldVisualization();

            // Add debug information about gamepad inputs for troubleshooting
            //addGamepadDebugInfo();

            // Update smart telemetry display
            smartTelemetry.updateDisplay(emergencyStop);
        }

        // Save ML data on exit
        if (robotManager.getShooterSystem() != null) {
            robotManager.getShooterSystem().getRpmLearning().saveLearningData();
        }

        // Graceful shutdown when OpMode ends
        if (robotManager.getShooterSystem() != null) {
            robotManager.getShooterSystem().reset();
        }

        // Stop lift safely
        if (liftController != null) {
            liftController.stop();
        }

        telemetry.clear();
        telemetry.addLine("AURORA Enhanced TeleOp stopped safely");
        telemetry.addLine("Smart telemetry system optimized performance!");
        telemetry.addLine("🤖 ML learning data saved automatically");
        telemetry.addLine("🎯 Lift system stopped safely");
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
                    robotManager.getDriveSystem().setCurrentMode(SmartMechanumDrive.DriveMode.SPORT);
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

            // Emergency stop lift
            if (liftController != null) {
                liftController.emergencyStop();
            }

            // Smart telemetry will automatically show emergency status
        }

        // Cancel emergency stop if Back is pressed
        if (gamepad1.back || gamepad2.back) {
            emergencyStop = false;
            if (liftController != null) {
                liftController.clearEmergencyStop();
            }
        }
    }

    /**
     * Update Panels field visualization with current robot position from odometry
     */
    private void updateFieldVisualization() {
        if (robotManager.getShooterSystem() != null) {
            Pose2D position = robotManager.getShooterSystem().getPosition();
            if (position != null) {
                // Convert Pose2D to coordinates for drawing
                double x = position.getX(DistanceUnit.INCH);
                double y = position.getY(DistanceUnit.INCH);
                double heading = position.getHeading(AngleUnit.RADIANS);

                // Draw robot on field
                drawRobot(x, y, heading);
            }
        }

        // Update the field display
        panelsField.update();
    }

    /**
     * Draw robot on Panels field with heading indicator
     */
    private void drawRobot(double x, double y, double heading) {
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) {
            return;
        }

        panelsField.setStyle(robotStyle);
        panelsField.moveCursor(x, y);
        panelsField.circle(ROBOT_RADIUS);

        // Draw heading line
        double headingLength = ROBOT_RADIUS;
        double x2 = x + Math.cos(heading) * headingLength;
        double y2 = y + Math.sin(heading) * headingLength;

        panelsField.setStyle(headingStyle);
        panelsField.moveCursor(x, y);
        panelsField.line(x2, y2);
    }

    /**
     * Add debug information about gamepad inputs
     * This method outputs the current state of the gamepad inputs to telemetry
     * to help troubleshoot any issues with gamepad control.
     */
    private void addGamepadDebugInfo() {
        // Log gamepad 1 inputs
        telemetry.addData("GP1 - Left Stick (X,Y)", "%.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("GP1 - Right Stick (X,Y)", "%.2f, %.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("GP1 - Triggers (LT,RT)", "%.2f, %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("GP1 - Bumpers (LB,RB)", "%s, %s", gamepad1.left_bumper, gamepad1.right_bumper);
        telemetry.addData("GP1 - D-pad (U,D,L,R)", "%s, %s, %s, %s", gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
        telemetry.addData("GP1 - Buttons (A,B,X,Y)", "%s, %s, %s, %s", gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);
        telemetry.addData("GP1 - Start, Back", "%s, %s", gamepad1.start, gamepad1.back);

        // Log gamepad 2 inputs
        telemetry.addData("GP2 - Left Stick (X,Y)", "%.2f, %.2f", gamepad2.left_stick_x, gamepad2.left_stick_y);
        telemetry.addData("GP2 - Right Stick (X,Y)", "%.2f, %.2f", gamepad2.right_stick_x, gamepad2.right_stick_y);
        telemetry.addData("GP2 - Triggers (LT,RT)", "%.2f, %.2f", gamepad2.left_trigger, gamepad2.right_trigger);
        telemetry.addData("GP2 - Bumpers (LB,RB)", "%s, %s", gamepad2.left_bumper, gamepad2.right_bumper);
        telemetry.addData("GP2 - D-pad (U,D,L,R)", "%s, %s, %s, %s", gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right);
        telemetry.addData("GP2 - Buttons (A,B,X,Y)", "%s, %s, %s, %s", gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y);
        telemetry.addData("GP2 - Start, Back", "%s, %s", gamepad2.start, gamepad2.back);

        // Update the telemetry with the gamepad debug info
        telemetry.update();
    }

    /**
     * Handle ML system controls (save and reset)
     */
    private void handleMlControls() {
        if (robotManager.getShooterSystem() == null) return;

        // Handle ML save button (Left Bumper)
        boolean currentMlSave = gamepad2.left_bumper;
        if (currentMlSave && !prevMlSave) {
            // Save ML data
            boolean saved = robotManager.getShooterSystem().getRpmLearning().saveLearningData();
            if (saved) {
                // Briefly show performance page to confirm save
                smartTelemetry.showPerformance();
            }
        }
        prevMlSave = currentMlSave;

        // Handle ML reset button (Right Bumper - must hold for 2 seconds)
        boolean currentMlReset = gamepad2.right_bumper;
        double currentTime = runtime.seconds();

        if (currentMlReset) {
            if (!mlResetButtonHeld) {
                // Just started holding button
                mlResetButtonHeld = true;
                mlResetButtonHoldStart = currentTime;
            } else {
                // Continue holding - check if held long enough
                double holdTime = currentTime - mlResetButtonHoldStart;
                if (holdTime >= 2.0) {
                    // Reset ML system
                    robotManager.getShooterSystem().getRpmLearning().resetLearning();
                    mlResetButtonHeld = false; // Prevent repeated resets
                    // Briefly show performance page to confirm reset
                    smartTelemetry.showPerformance();
                }
            }
        } else {
            mlResetButtonHeld = false;
        }

        // Pass ML controls to shooter system
        double holdTime = mlResetButtonHeld ? (currentTime - mlResetButtonHoldStart) : 0;
        robotManager.getShooterSystem().handleMlControls(
            currentMlSave && !prevMlSave,  // Save on button press edge
            currentMlReset,                // Reset button state
            holdTime                       // How long reset button has been held
        );
    }

    /**
     * Handle lift controls (gamepad2 in dual driver mode, gamepad1 in single driver mode)
     * Manual control: Right stick Y
     * Preset positions: D-pad buttons
     */
    private void handleLiftControls() {
        if (liftController == null) return;

        // In dual driver mode, gamepad2 controls lift
        // In single driver mode, gamepad1 controls everything (but we'll still use gamepad2 for lift to avoid conflicts)

        // Manual lift control from right stick Y (inverted so up = positive)
        double liftInput = -gamepad2.right_stick_y;

        // Handle preset position buttons with edge detection to prevent repeated commands
        boolean currentHighButton = gamepad2.dpad_up;
        boolean currentMidButton = gamepad2.dpad_right;
        boolean currentLowButton = gamepad2.dpad_down;
        boolean currentGroundButton = gamepad2.dpad_left;

        // Move to high position (D-pad Up)
        if (currentHighButton && !prevLiftHighButton) {
            liftController.moveToHigh();
        }

        // Move to mid position (D-pad Right)
        if (currentMidButton && !prevLiftMidButton) {
            liftController.moveToMid();
        }

        // Move to low position (D-pad Down)
        if (currentLowButton && !prevLiftLowButton) {
            liftController.moveToLow();
        }

        // Move to ground position (D-pad Left)
        if (currentGroundButton && !prevLiftGroundButton) {
            liftController.moveToGround();
        }

        // Update previous button states
        prevLiftHighButton = currentHighButton;
        prevLiftMidButton = currentMidButton;
        prevLiftLowButton = currentLowButton;
        prevLiftGroundButton = currentGroundButton;

        // Cancel auto-position if driver manually moves stick
        if (Math.abs(liftInput) > 0.1 && liftController.isPositionControlActive()) {
            liftController.cancelPositionControl();
        }

        // Update lift controller with current input
        liftController.update(liftInput);
    }
}
