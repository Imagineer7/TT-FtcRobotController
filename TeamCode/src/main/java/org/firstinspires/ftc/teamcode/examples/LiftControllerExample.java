/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Example OpMode demonstrating RobotLiftController usage
 */

package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.RobotLiftController;

/**
 * Example OpMode showing how to use the RobotLiftController
 *
 * Controls:
 * - Left Stick Y: Manual lift control
 * - D-pad Up: Move to high position
 * - D-pad Right: Move to mid position
 * - D-pad Left: Move to low position
 * - D-pad Down: Move to ground position
 * - X: Show basic telemetry
 * - Y: Show detailed telemetry
 * - A: Show calibration telemetry
 * - B: Reset performance stats
 */
@TeleOp(name="Lift Controller Example", group="Examples")
public class LiftControllerExample extends LinearOpMode {

    private RobotLiftController liftController;
    private int telemetryMode = 0;  // 0=basic, 1=detailed, 2=calibration

    @Override
    public void runOpMode() {
        // Initialize lift controller
        telemetry.addLine("Initializing Lift Controller Example...");
        telemetry.update();

        liftController = new RobotLiftController(hardwareMap, telemetry);
        if (!liftController.initialize()) {
            telemetry.addLine("❌ Failed to initialize lift controller!");
            telemetry.addLine("Check motor names: robotLiftLeft & robotLiftRight");
            telemetry.update();
            return;
        }

        telemetry.addLine("✅ Ready to start!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("• Left Stick Y: Manual control");
        telemetry.addLine("• D-pad: Preset positions");
        telemetry.addLine("• X/Y/A: Change telemetry mode");
        telemetry.addLine("• B: Reset stats");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get manual control input (invert if needed for your setup)
            double liftInput = -gamepad1.left_stick_y;

            // Handle preset position buttons
            if (gamepad1.dpad_up) {
                liftController.moveToHigh();
            } else if (gamepad1.dpad_right) {
                liftController.moveToMid();
            } else if (gamepad1.dpad_left) {
                liftController.moveToLow();
            } else if (gamepad1.dpad_down) {
                liftController.moveToGround();
            }

            // Cancel auto-position if driver moves stick
            if (Math.abs(liftInput) > 0.1 && liftController.isPositionControlActive()) {
                liftController.cancelPositionControl();
            }

            // Update lift controller
            liftController.update(liftInput);

            // Handle telemetry mode changes
            if (gamepad1.x) {
                telemetryMode = 0;  // Basic
            } else if (gamepad1.y) {
                telemetryMode = 1;  // Detailed
            } else if (gamepad1.a) {
                telemetryMode = 2;  // Calibration
            }

            // Reset performance stats
            if (gamepad1.b) {
                liftController.resetPerformanceStats();
            }

            // Display telemetry based on mode
            telemetry.clear();

            switch (telemetryMode) {
                case 0:
                    liftController.addTelemetry();
                    telemetry.addLine();
                    telemetry.addLine("Press Y for detailed telemetry");
                    telemetry.addLine("Press A for calibration telemetry");
                    break;

                case 1:
                    liftController.addDetailedTelemetry();
                    telemetry.addLine();
                    telemetry.addLine("Press X for basic telemetry");
                    telemetry.addLine("Press A for calibration telemetry");
                    break;

                case 2:
                    liftController.addCalibrationTelemetry();
                    telemetry.addLine();
                    telemetry.addLine("Press X for basic telemetry");
                    telemetry.addLine("Press Y for detailed telemetry");
                    break;
            }

            telemetry.addLine();
            telemetry.addData("Status", liftController.getStatusString());

            // Show PID status
            if (liftController.isGravityPidActive()) {
                telemetry.addLine();
                telemetry.addLine("🤖 Auto Gravity PID: ACTIVE");
                telemetry.addData("Learned Power", "%.4f", liftController.getGravityCompensationPower());
            }

            telemetry.update();
        }

        // Cleanup
        liftController.stop();
        telemetry.clear();
        telemetry.addLine("Lift Controller Example stopped");
        telemetry.update();
    }
}

