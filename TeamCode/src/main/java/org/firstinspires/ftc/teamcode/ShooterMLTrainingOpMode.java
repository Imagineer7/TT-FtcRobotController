/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Shooter Machine Learning Training OpMode
 * Use this to train the shooter parameters, then use trained values in competition
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.util.aurora.EnhancedDecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterBoostConfig;

/**
 * ShooterMLTrainingOpMode - Train the shooter machine learning system
 *
 * CONTROLS:
 * - RIGHT TRIGGER: Shoot (hold to fire continuously)
 * - DPAD UP: Increase boost power by 1%
 * - DPAD DOWN: Decrease boost power by 1%
 * - DPAD RIGHT: Increase boost duration by 5ms
 * - DPAD LEFT: Decrease boost duration by 5ms
 * - Y: Reset learning to defaults
 * - X: Save learned parameters
 * - A: Toggle auto-learning on/off
 * - B: Test rapid fire (10 shots)
 *
 * TRAINING PROCESS:
 * 1. Start this OpMode
 * 2. Fire shots using RIGHT TRIGGER
 * 3. Watch telemetry - system learns optimal parameters automatically
 * 4. After 20+ shots, parameters will be fine-tuned
 * 5. Press X to save learned parameters
 * 6. Use saved parameters in competition OpMode with learning disabled
 */
@TeleOp(name="Shooter ML Training", group="Training")
public class ShooterMLTrainingOpMode extends LinearOpMode {

    private EnhancedDecodeHelper shooter;
    private Gamepad previousGamepad1 = new Gamepad();
    private boolean autoLearning = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Shooter ML Training...");
        telemetry.update();

        // Initialize shooter with learning ENABLED
        shooter = new EnhancedDecodeHelper(hardwareMap, false); // No odometry needed for training
        shooter.setLearningEnabled(true); // Enable learning mode

        // Try to load any existing learned parameters as starting point
        boolean loadedExisting = shooter.loadBoostParametersFromFile();
        if (loadedExisting) {
            telemetry.addData("Status", "Loaded existing parameters as baseline");
        } else {
            telemetry.addData("Status", "Starting with default parameters");
        }
        telemetry.update();

        telemetry.addData("Status", "Ready to train!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("RT = Shoot | Y = Reset | X = Save");
        telemetry.addLine("A = Toggle Learning | B = Test 10 shots");
        telemetry.addLine("DPAD = Manual adjustments");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle shooting
            boolean shootButton = gamepad1.right_trigger > 0.5;
            shooter.handleShootButton(shootButton, ShooterConfig.ShooterPreset.LONG_RANGE);

            // Update shooter control (RPM maintenance)
            // This is already handled inside handleShootButton

            // Handle manual parameter adjustments (for fine-tuning if needed)
            handleManualAdjustments();

            // Handle control buttons
            handleControlButtons();

            // Display comprehensive training telemetry
            displayTrainingTelemetry();

            // Store previous gamepad state for edge detection
            previousGamepad1.copy(gamepad1);

            sleep(10); // Small delay for stability
        }

        // Auto-save on stop
        boolean saved = shooter.saveBoostParametersToFile();
        telemetry.addData("Status", saved ? "Parameters saved!" : "Save failed");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Handle manual parameter adjustments with DPAD
     */
    private void handleManualAdjustments() {
        // Boost power adjustments
        if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
            shooter.adjustBoostPower(0.01); // +1%
        }
        if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
            shooter.adjustBoostPower(-0.01); // -1%
        }

        // Boost duration adjustments
        if (gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            shooter.adjustBoostDuration(0.005); // +5ms
        }
        if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
            shooter.adjustBoostDuration(-0.005); // -5ms
        }
    }

    /**
     * Handle control buttons
     */
    private void handleControlButtons() {
        // Reset learning
        if (gamepad1.y && !previousGamepad1.y) {
            shooter.resetLearning();
            telemetry.addData("Action", "Learning reset to defaults");
            telemetry.update();
            sleep(500);
        }

        // Save parameters
        if (gamepad1.x && !previousGamepad1.x) {
            boolean saved = shooter.saveBoostParametersToFile();
            telemetry.addData("Action", saved ? "Parameters SAVED!" : "Save FAILED!");
            telemetry.update();
            sleep(1000);
        }

        // Toggle auto-learning
        if (gamepad1.a && !previousGamepad1.a) {
            autoLearning = !autoLearning;
            shooter.setLearningEnabled(autoLearning);
            telemetry.addData("Action", "Auto-learning " + (autoLearning ? "ON" : "OFF"));
            telemetry.update();
            sleep(500);
        }

        // Test rapid fire
        if (gamepad1.b && !previousGamepad1.b) {
            telemetry.addData("Action", "Starting 10-shot test...");
            telemetry.update();
            shooter.startAutoShootSmart(10, ShooterConfig.ShooterPreset.LONG_RANGE);
        }
    }

    /**
     * Display comprehensive training telemetry
     */
    private void displayTrainingTelemetry() {
        telemetry.addData("=== TRAINING MODE ===", "");
        telemetry.addLine();

        // Learning status
        telemetry.addData("Auto-Learning", autoLearning ? "ENABLED" : "DISABLED");
        telemetry.addData("Shots Analyzed", shooter.getShotsAnalyzed());
        telemetry.addData("Config Exists", ShooterBoostConfig.configExists() ? "YES" : "NO");
        telemetry.addLine();

        // Current RPM status
        telemetry.addData("=== RPM Status ===", "");
        telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("RPM Error", "%.0f", shooter.getTargetRPM() - shooter.getCurrentRPM());
        telemetry.addData("Shooter Ready", shooter.isShooterReady() ? "YES" : "NO");
        telemetry.addLine();

        // Learned parameters - Main
        telemetry.addData("=== Learned Params ===", "");
        telemetry.addData("Boost", shooter.getLearningTelemetry());
        telemetry.addData("Timing", shooter.getDetailedLearningTelemetry());
        telemetry.addLine();

        // Shot trajectory (if tracking)
        if (shooter.isShooterRunning()) {
            telemetry.addData("=== Shot Tracking ===", "");
            telemetry.addData("Trajectory", shooter.getTrajectoryTelemetry());
            telemetry.addLine();
        }

        // PID Learning (from RpmLearningSystem)
        telemetry.addData("=== PID Learning ===", "");
        telemetry.addData("Status", shooter.getRpmLearningTelemetry());
        telemetry.addData("Performance", shooter.getRpmPerformanceMetrics());
        telemetry.addLine();

        // Performance stats
        telemetry.addData("=== Performance ===", "");
        telemetry.addData("Shots Fired", shooter.getTotalShots());
        telemetry.addData("Avg Spinup", "%.2fs", shooter.getAverageSpinupTime());
        telemetry.addData("Success Rate", "%.1f%%", shooter.getSuccessRate());
        telemetry.addLine();

        // Instructions
        telemetry.addData("=== Controls ===", "");
        telemetry.addLine("RT=Shoot | Y=Reset | X=Save | A=Toggle");
        telemetry.addLine("UP/DN=Power | LT/RT=Duration | B=Test");

        telemetry.update();
    }
}

