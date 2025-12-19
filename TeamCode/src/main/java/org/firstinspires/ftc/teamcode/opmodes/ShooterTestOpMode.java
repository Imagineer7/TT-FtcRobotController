package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.aurora.DecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;

/**
 * Shooter Test OpMode - Tests DecodeHelper shooter system
 *
 * This OpMode provides comprehensive testing for the dual-motor friction flywheel shooter.
 *
 * CONTROLS:
 *
 * PRESET SHOOTING:
 * - Y Button: Long Range Shooting (4400 RPM)
 * - B Button: Mid Range Shooting (3500 RPM)
 * - A Button: Short Range Shooting (2800 RPM)
 * - Left Trigger: Warmup Mode (65% of target RPM)
 *
 * MANUAL CONTROL:
 * - DPAD UP: Increase target RPM by 100
 * - DPAD DOWN: Decrease target RPM by 100
 * - DPAD LEFT: Decrease target RPM by 500
 * - DPAD RIGHT: Increase target RPM by 500
 * - X Button: Stop shooter (IDLE)
 *
 * TESTING:
 * - Left Bumper: Spin up to current target
 * - Right Bumper: Enable warmup mode
 * - Right Trigger: Manual fire (when ready)
 * - Back Button: Clear error state
 *
 * TELEMETRY:
 * - Current state and RPM data
 * - Target RPM and percentage achieved
 * - Motor sync status
 * - Ready/Firing indicators
 * - Time until next shot allowed
 */
@TeleOp(name = "🎯 Shooter Test", group = "Testing")
public class ShooterTestOpMode extends LinearOpMode {

    private DecodeHelper shooter;
    private DcMotor leftShooterMotor;
    private DcMotor rightShooterMotor;

    // Manual target tracking
    private double manualTargetRPM = 3500;

    // Control mode
    private enum ControlMode {
        PRESET,  // Using preset buttons
        MANUAL   // Using manual RPM adjustment
    }
    private ControlMode controlMode = ControlMode.PRESET;

    // Button state tracking
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevX = false;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevRightTrigger = false;
    private boolean prevBack = false;

    @Override
    public void runOpMode() {
        telemetry.addLine("🎯 Shooter Test OpMode");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        try {
            // Initialize motors (shooter1 and shooter2 as configured in hardware)
            leftShooterMotor = hardwareMap.get(DcMotor.class, "shooter1");
            rightShooterMotor = hardwareMap.get(DcMotor.class, "shooter2");

            // Initialize DecodeHelper
            shooter = new DecodeHelper(leftShooterMotor, rightShooterMotor, telemetry);

            telemetry.addLine("✅ Hardware initialized successfully!");
            telemetry.addLine();
            telemetry.addLine("Ready to start!");
            telemetry.addLine("Press PLAY to begin testing");

        } catch (Exception e) {
            telemetry.addLine("❌ ERROR: Failed to initialize hardware");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.update();
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // Handle control inputs
            handlePresetControls();
            handleManualControls();
            handleTestingControls();

            // Update shooter system
            shooter.update();

            // Display telemetry
            displayTelemetry();
            telemetry.update();

            // Small delay to prevent excessive updates
            sleep(20);
        }

        // Stop shooter on exit
        shooter.disableShooter();
        telemetry.addLine("🛑 OpMode stopped - shooter disabled");
        telemetry.update();
    }

    /**
     * Handle preset shooting controls
     */
    private void handlePresetControls() {
        boolean anyPresetActive = false;

        // Y Button - Long Range
        if (gamepad1.y) {
            shooter.handleShootButton(true, ShooterConfig.ShooterPreset.LONG_RANGE);
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
        }
        // B Button - Mid Range
        else if (gamepad1.b) {
            shooter.handleShootButton(true, ShooterConfig.ShooterPreset.MID_RANGE);
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
        }
        // A Button - Short Range
        else if (gamepad1.a) {
            shooter.handleShootButton(true, ShooterConfig.ShooterPreset.SHORT_RANGE);
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
        }

        // If no preset button held, release
        if (!anyPresetActive && controlMode == ControlMode.PRESET) {
            shooter.handleShootButton(false, null);
        }

        // Left Trigger - Warmup Mode
        boolean warmupActive = gamepad1.left_trigger > 0.3;
        ShooterConfig.ShooterPreset warmupPreset = shooter.getActivePreset() != null ?
                shooter.getActivePreset() : ShooterConfig.ShooterPreset.LONG_RANGE;
        shooter.handleWarmupButton(warmupActive, warmupPreset);
    }

    /**
     * Handle manual RPM adjustment controls
     */
    private void handleManualControls() {
        // DPAD UP - Increase by 100
        if (gamepad1.dpad_up && !prevDpadUp) {
            manualTargetRPM += 100;
            manualTargetRPM = Math.min(manualTargetRPM, 6000);
            controlMode = ControlMode.MANUAL;
        }
        prevDpadUp = gamepad1.dpad_up;

        // DPAD DOWN - Decrease by 100
        if (gamepad1.dpad_down && !prevDpadDown) {
            manualTargetRPM -= 100;
            manualTargetRPM = Math.max(manualTargetRPM, 0);
            controlMode = ControlMode.MANUAL;
        }
        prevDpadDown = gamepad1.dpad_down;

        // DPAD RIGHT - Increase by 500
        if (gamepad1.dpad_right && !prevDpadRight) {
            manualTargetRPM += 500;
            manualTargetRPM = Math.min(manualTargetRPM, 6000);
            controlMode = ControlMode.MANUAL;
        }
        prevDpadRight = gamepad1.dpad_right;

        // DPAD LEFT - Decrease by 500
        if (gamepad1.dpad_left && !prevDpadLeft) {
            manualTargetRPM -= 500;
            manualTargetRPM = Math.max(manualTargetRPM, 0);
            controlMode = ControlMode.MANUAL;
        }
        prevDpadLeft = gamepad1.dpad_left;

        // X Button - Stop shooter
        if (gamepad1.x && !prevX) {
            shooter.disableShooter();
            controlMode = ControlMode.MANUAL;
        }
        prevX = gamepad1.x;
    }

    /**
     * Handle testing controls
     */
    private void handleTestingControls() {
        // Left Bumper - Spin up to target
        if (gamepad1.left_bumper && !prevLeftBumper) {
            if (controlMode == ControlMode.MANUAL) {
                shooter.setTargetRPM(manualTargetRPM);
                shooter.spinUp();
            }
        }
        prevLeftBumper = gamepad1.left_bumper;

        // Right Bumper - Enable warmup
        if (gamepad1.right_bumper && !prevRightBumper) {
            if (controlMode == ControlMode.MANUAL) {
                shooter.setTargetRPM(manualTargetRPM);
                shooter.enableWarmup();
            }
        }
        prevRightBumper = gamepad1.right_bumper;

        // Right Trigger - Manual fire
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        if (rightTriggerPressed && !prevRightTrigger) {
            if (shooter.isReady()) {
                shooter.fire();
            }
        }
        prevRightTrigger = rightTriggerPressed;

        // Back Button - Clear error
        if (gamepad1.back && !prevBack) {
            shooter.clearError();
        }
        prevBack = gamepad1.back;
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry() {
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("🎯 SHOOTER TEST OPMODE");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine();

        // Control mode
        telemetry.addLine("CONTROL MODE: " + controlMode);
        if (controlMode == ControlMode.MANUAL) {
            telemetry.addLine("Manual Target: " + String.format("%.0f RPM", manualTargetRPM));
        }
        telemetry.addLine();

        // Shooter status
        telemetry.addLine("SHOOTER STATUS:");
        telemetry.addLine(shooter.getStatusString());
        telemetry.addLine();

        // RPM Details
        telemetry.addLine("RPM DETAILS:");
        telemetry.addData("  Left Motor", "%.0f RPM", shooter.getLeftRPM());
        telemetry.addData("  Right Motor", "%.0f RPM", shooter.getRightRPM());
        telemetry.addData("  Average", "%.0f RPM", shooter.getAverageRPM());
        telemetry.addData("  Sync Error", "%.0f RPM", shooter.getRPMSyncError());
        telemetry.addData("  Progress", "%.1f%%", shooter.getRPMPercentage());
        telemetry.addLine();

        // State information
        telemetry.addLine("STATE INFORMATION:");
        telemetry.addData("  State", shooter.getState().toString());
        telemetry.addData("  Ready", shooter.isReady() ? "✅ YES" : "❌ NO");
        telemetry.addData("  At Target", shooter.isAtTargetRPM() ? "✅ YES" : "❌ NO");

        if (shooter.getActivePreset() != null) {
            telemetry.addData("  Active Preset", shooter.getActivePreset().getName());
        }

        if (shooter.getState() == DecodeHelper.ShooterState.SPINNING_UP) {
            telemetry.addData("  Spin-up Time", "%.2f s", shooter.getSpinUpTime() / 1000.0);
        }

        long nextShotTime = shooter.getTimeUntilNextShot();
        if (nextShotTime > 0) {
            telemetry.addData("  Next Shot In", "%.2f s", nextShotTime / 1000.0);
        }
        telemetry.addLine();

        // Controls reference
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  Y/B/A: Long/Mid/Short Range");
        telemetry.addLine("  LT: Warmup | RT: Manual Fire");
        telemetry.addLine("  DPAD: Adjust Manual RPM");
        telemetry.addLine("  LB: Spin Up | RB: Warmup");
        telemetry.addLine("  X: Stop | Back: Clear Error");
    }
}

