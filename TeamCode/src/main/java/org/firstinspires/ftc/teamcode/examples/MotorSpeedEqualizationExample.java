/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Example demonstrating Motor Speed Equalization usage
 */

package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.MotorSpeedEqualizer;
import org.firstinspires.ftc.teamcode.util.aurora.SmartMechanumDrive;

/**
 * Motor Speed Equalization Example
 *
 * This OpMode demonstrates how to use the motor speed equalization feature
 * to compensate for uneven motor speeds caused by weight distribution.
 *
 * Controls:
 * - Gamepad1 Left Stick: Forward/backward and strafe
 * - Gamepad1 Right Stick X: Rotation
 * - Gamepad1 A: Toggle speed equalization ON/OFF
 * - Gamepad1 B: Cycle through correction modes (DISABLED, AGGRESSIVE, CONSERVATIVE)
 * - Gamepad1 X: Reset to DISABLED mode
 *
 * The telemetry will display:
 * - Current equalization status (enabled/disabled)
 * - Current correction mode
 * - Motor speeds (encoder ticks per second)
 * - Applied corrections to each motor
 * - Total corrections applied since start
 */
@TeleOp(name="Motor Speed Equalization Example", group="Examples")
@Disabled  // Remove this line to enable the OpMode
public class MotorSpeedEqualizationExample extends LinearOpMode {

    private AuroraHardwareConfig hardware;
    private SmartMechanumDrive drive;

    // Control state tracking
    private boolean prevToggleButton = false;
    private boolean prevModeButton = false;
    private boolean prevDisableButton = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();

        if (!hardware.isDriveSystemInitialized()) {
            telemetry.addLine("ERROR: Drive system failed to initialize!");
            telemetry.addLine(hardware.getDriveInitError());
            telemetry.update();
            return;
        }

        // Create drive system
        drive = new SmartMechanumDrive(
            hardware.getFrontLeftMotor(),
            hardware.getFrontRightMotor(),
            hardware.getBackLeftMotor(),
            hardware.getBackRightMotor(),
            gamepad1,
            hardware.getVoltageSensor(),
            hardware.getOdometry()
        );

        // Configure speed equalization
        // Start with it disabled - driver can enable it with A button
        drive.setSpeedEqualizationEnabled(false);
        drive.setSpeedEqualizationMode(MotorSpeedEqualizer.CorrectionMode.AGGRESSIVE);

        // Optional: Fine-tune equalization parameters
        MotorSpeedEqualizer equalizer = drive.getSpeedEqualizer();
        equalizer.setAggressiveBoostFactor(0.15);  // 15% max boost for slow motors
        equalizer.setConservativeReductionFactor(0.15);  // 15% max reduction for fast motors
        equalizer.setSpeedTolerancePercent(5.0);  // Only correct if >5% speed difference
        equalizer.setMinSpeedThreshold(50.0);  // Only correct when moving faster than 50 ticks/sec

        telemetry.addLine("✓ Initialization Complete!");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick: Movement");
        telemetry.addLine("  Right Stick X: Rotation");
        telemetry.addLine("  A: Toggle equalization ON/OFF");
        telemetry.addLine("  B: Cycle correction modes");
        telemetry.addLine("  X: Disable equalization");
        telemetry.addLine("");
        telemetry.addLine("Press Start to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle control inputs
            handleControls();

            // Update drive system (this includes speed equalization)
            drive.setDriveInputs(
                -gamepad1.left_stick_y,   // Forward/backward (inverted)
                gamepad1.left_stick_x,     // Strafe
                gamepad1.right_stick_x     // Rotation
            );
            drive.update();

            // Display telemetry
            updateTelemetry();

            // Give some breathing room
            sleep(10);
        }
    }

    /**
     * Handle button controls for equalization
     */
    private void handleControls() {
        // A button: Toggle equalization on/off
        boolean toggleButton = gamepad1.a;
        if (toggleButton && !prevToggleButton) {
            boolean currentState = drive.isSpeedEqualizationEnabled();
            drive.setSpeedEqualizationEnabled(!currentState);
            
            // Provide feedback
            if (!currentState) {
                gamepad1.rumble(200);  // Short rumble when enabling
            } else {
                gamepad1.rumble(100);  // Shorter rumble when disabling
            }
        }
        prevToggleButton = toggleButton;

        // B button: Cycle through correction modes
        boolean modeButton = gamepad1.b;
        if (modeButton && !prevModeButton) {
            drive.cycleSpeedEqualizationMode();
            gamepad1.rumble(150);  // Medium rumble when changing modes
        }
        prevModeButton = modeButton;

        // X button: Disable equalization immediately
        boolean disableButton = gamepad1.x;
        if (disableButton && !prevDisableButton) {
            drive.setSpeedEqualizationEnabled(false);
            drive.setSpeedEqualizationMode(MotorSpeedEqualizer.CorrectionMode.DISABLED);
            gamepad1.rumble(300);  // Long rumble for disable
        }
        prevDisableButton = disableButton;
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.addLine("=== Motor Speed Equalization ===");
        telemetry.addLine("");

        // Status
        boolean enabled = drive.isSpeedEqualizationEnabled();
        MotorSpeedEqualizer.CorrectionMode mode = drive.getSpeedEqualizationMode();
        
        telemetry.addLine("Status: " + (enabled ? "✓ ENABLED" : "✗ DISABLED"));
        telemetry.addLine("Mode: " + mode.getName());
        telemetry.addLine("  " + mode.getDescription());
        telemetry.addLine("");

        // Get detailed telemetry data
        MotorSpeedEqualizer.EqualizerTelemetryData eqData = drive.getSpeedEqualizationTelemetry();

        // Motor speeds
        telemetry.addLine("Motor Speeds (ticks/sec):");
        for (int i = 0; i < eqData.speeds.length; i++) {
            String correction = "";
            if (Math.abs(eqData.corrections[i]) > 0.001) {
                correction = String.format(" [%+.3f]", eqData.corrections[i]);
            }
            telemetry.addData("  " + eqData.motorNames[i], 
                "%.1f%s", eqData.speeds[i], correction);
        }
        telemetry.addLine("");

        // Acceleration status
        telemetry.addLine("Acceleration: " + 
            (eqData.accelerationBalanced ? "✓ Balanced" : "⚠ Unbalanced"));
        telemetry.addLine("");

        // Statistics
        telemetry.addData("Corrections Applied", eqData.correctionCount);
        telemetry.addLine("");

        // Drive info
        telemetry.addLine("--- Drive System ---");
        telemetry.addData("Drive Mode", drive.getCurrentMode().getName());
        telemetry.addData("Battery", "%.1fV", drive.getCurrentVoltage());
        telemetry.addLine("");

        // Controls reminder
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A: Toggle ON/OFF | B: Cycle Mode | X: Disable");

        telemetry.update();
    }
}
