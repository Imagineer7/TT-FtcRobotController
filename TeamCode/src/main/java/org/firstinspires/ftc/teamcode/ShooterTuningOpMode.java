/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Shooter tuning OpMode with live parameter adjustment and learning telemetry
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.EnhancedDecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;

/**
 * ShooterTuningOpMode - Live tuning and learning telemetry for shooter
 *
 * Controls:
 * - A: Fire shot (hold for continuous)
 * - B: Toggle learning on/off
 * - X: Save current parameters
 * - Y: Reset to defaults
 *
 * D-Pad Up/Down: Adjust boost power (+/- 1%)
 * D-Pad Left/Right: Adjust boost duration (+/- 10ms)
 * Left Bumper: Decrease boost delay (-5ms)
 * Right Bumper: Increase boost delay (+5ms)
 */
@TeleOp(name = "Shooter Tuning", group = "Tuning")
public class ShooterTuningOpMode extends LinearOpMode {

    private EnhancedDecodeHelper shooter;
    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;

    @Override
    public void runOpMode() {
        // Initialize shooter system with odometry disabled for tuning
        shooter = new EnhancedDecodeHelper(hardwareMap, false);

        // Display initialization status
        telemetry.addLine("=== SHOOTER TUNING MODE ===");
        telemetry.addLine();

        if (shooter.hasSavedConfig()) {
            telemetry.addLine("✓ Loaded saved boost parameters");
        } else {
            telemetry.addLine("○ Using default parameters");
        }

        telemetry.addLine();
        telemetry.addLine("Ready to tune!");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        // Set shooter to long range preset
        ShooterConfig.ShooterPreset currentPreset = ShooterConfig.ShooterPreset.SHORT_RANGE;

        while (opModeIsActive()) {
            // === CONTROL INPUTS ===

            // Shooting control (A button)
            boolean aPressed = gamepad1.a;
            shooter.handleShootButton(aPressed, currentPreset);

            // Toggle learning (B button)
            boolean bPressed = gamepad1.b;
            if (bPressed && !lastBState) {
                shooter.setLearningEnabled(!shooter.isLearningEnabled());
            }

            // Save parameters (X button)
            boolean xPressed = gamepad1.x;
            if (xPressed && !lastXState) {
                boolean saved = shooter.saveBoostParametersToFile();
                telemetry.speak(saved ? "Parameters saved" : "Save failed");
            }

            // Reset to defaults (Y button)
            boolean yPressed = gamepad1.y;
            if (yPressed && !lastYState) {
                boolean reset = shooter.resetSavedConfig();
                telemetry.speak(reset ? "Reset to defaults" : "Reset failed");
            }

            // Adjust boost power (D-pad Up/Down)
            if (gamepad1.dpad_up) {
                shooter.adjustBoostPower(0.01); // +1%
            }
            if (gamepad1.dpad_down) {
                shooter.adjustBoostPower(-0.01); // -1%
            }

            // Adjust boost duration (D-pad Left/Right)
            if (gamepad1.dpad_left) {
                shooter.adjustBoostDuration(-0.010); // -10ms
            }
            if (gamepad1.dpad_right) {
                shooter.adjustBoostDuration(0.010); // +10ms
            }

            // Adjust boost delay (Bumpers)
            if (gamepad1.left_bumper) {
                shooter.adjustBoostDelay(-0.005); // -5ms
            }
            if (gamepad1.right_bumper) {
                shooter.adjustBoostDelay(0.005); // +5ms
            }

            // === TELEMETRY ===

            telemetry.addLine("=== SHOOTER TUNING ===");
            telemetry.addLine();

            // Shooter status
            telemetry.addData("Status", shooter.isShooterRunning() ? "🔴 RUNNING" : "⚪ STOPPED");
            telemetry.addData("RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
            telemetry.addData("Ready", shooter.isShooterReady() ? "✓ YES" : "○ NO");
            telemetry.addLine();

            // Learning status
            String learningStatus = shooter.isLearningEnabled() ? "✓ ENABLED" : "✗ DISABLED";
            telemetry.addData("Learning", learningStatus);
            telemetry.addData("Shots Analyzed", shooter.getShotsAnalyzed());
            telemetry.addLine();

            // Boost parameters
            telemetry.addLine("--- Boost Parameters ---");
            double[] params = shooter.getBoostParametersArray();
            telemetry.addData("Delay", "%.3f s (%.0f ms)", params[0], params[0] * 1000);
            telemetry.addData("Duration", "%.3f s (%.0f ms)", params[1], params[1] * 1000);
            telemetry.addData("Power", "%.2f (%.0f%%)", params[2], params[2] * 100);
            telemetry.addLine();

            // Controls reminder
            telemetry.addLine("--- Controls ---");
            telemetry.addData("A", "Shoot");
            telemetry.addData("B", "Toggle Learning");
            telemetry.addData("X", "Save Parameters");
            telemetry.addData("Y", "Reset to Defaults");
            telemetry.addLine();
            telemetry.addData("D-Pad ↑↓", "Boost Power");
            telemetry.addData("D-Pad ←→", "Boost Duration");
            telemetry.addData("Bumpers", "Boost Delay");

            telemetry.update();

            // Save button states
            lastAState = aPressed;
            lastBState = bPressed;
            lastXState = xPressed;
            lastYState = yPressed;
        }

        // Auto-save parameters at end of session
        if (shooter.getShotsAnalyzed() > 0) {
            shooter.saveBoostParametersToFile();
        }
    }
}

