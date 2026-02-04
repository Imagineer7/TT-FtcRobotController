package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.GamepadConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IntelMechanumDrive;
import org.firstinspires.ftc.teamcode.util.aurora.DecodeHelper;

/**
 * GamepadConfigExample - Demonstration of GamepadConfig with AURORA subsystems
 *
 * This OpMode shows how to use GamepadConfig for centralized control mapping
 * with dual gamepad mode (one driver, one operator).
 *
 * Controls:
 *
 * GAMEPAD 1 (DRIVER):
 *   - Left Stick X: Rotation
 *   - Right Stick Y: Forward/Backward
 *   - Right Stick X: Strafe
 *   - Left Bumper: Toggle slow mode
 *   - Right Bumper: Toggle field-centric mode
 *   - Y Button: Reset IMU heading
 *
 * GAMEPAD 2 (OPERATOR):
 *   - Y Button (Hold): Fire shots (Long Range)
 *   - A Button (Hold): Fire shots (Short Range)
 *   - B Button (Hold): Fire shots (Mid Range)
 *   - Left Trigger (Hold): Warmup mode
 *   - Right Trigger: Manual shooter power (if needed)
 */
@TeleOp(name = "GamepadConfig Example", group = "Examples")
@Disabled
public class GamepadConfigExample extends LinearOpMode {

    private AuroraHardwareConfig hardware;
    private GamepadConfig controls;
    private IntelMechanumDrive drive;
    private DecodeHelper shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        telemetry.addLine("🤖 Initializing AURORA System with GamepadConfig...");
        telemetry.update();

        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        // Initialize GamepadConfig in DUAL_GAMEPAD mode
        controls = new GamepadConfig(gamepad1, gamepad2);

        // Initialize subsystems with GamepadConfig
        drive = new IntelMechanumDrive(hardware, controls);
        shooter = new DecodeHelper(hardware, telemetry);

        telemetry.addLine("✅ Ready to operate!");
        telemetry.addLine("");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("GAMEPAD 1 (DRIVER):");
        telemetry.addLine("  Left Stick X: Rotation");
        telemetry.addLine("  Right Stick: Forward/Strafe");
        telemetry.addLine("  Bumpers: Mode toggles");
        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 2 (OPERATOR):");
        telemetry.addLine("  Y: Long Range Shot");
        telemetry.addLine("  A: Short Range Shot");
        telemetry.addLine("  B: Mid Range Shot");
        telemetry.addLine("  Left Trigger: Warmup");
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ═══════════════════════════════════════════════════════════
            // DRIVER CONTROLS (Gamepad 1)
            // ═══════════════════════════════════════════════════════════

            // Handle mode toggles
            if (controls.isSlowModePressed()) {
                drive.toggleSlowMode();
            }

            if (controls.isFieldCentricPressed()) {
                drive.toggleDriveMode();
            }

            if (controls.isResetHeadingPressed()) {
                drive.resetHeading();
            }

            // Main drive loop (automatically reads from GamepadConfig)
            drive.drive();

            // ═══════════════════════════════════════════════════════════
            // OPERATOR CONTROLS (Gamepad 2)
            // ═══════════════════════════════════════════════════════════

            // Handle shooter controls (automatically processes all shoot buttons and warmup)
            boolean shotFired = shooter.handleGamepadShootControls(controls);

            // Update shooter subsystem
            shooter.update();

            // ═══════════════════════════════════════════════════════════
            // TELEMETRY
            // ═══════════════════════════════════════════════════════════

            updateTelemetry(shotFired);
        }
    }

    /**
     * Update telemetry with current system status
     */
    private void updateTelemetry(boolean shotFired) {
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("🤖 AURORA SYSTEM STATUS");
        telemetry.addLine("═══════════════════════════════════════");

        // Drive status
        telemetry.addLine("");
        telemetry.addLine("🎮 DRIVE (Gamepad 1):");
        telemetry.addData("  Mode", "%s | %s",
                drive.getDriveMode(), drive.getSpeedMode());
        telemetry.addData("  Heading", "%.1f°", drive.getCurrentHeading());
        telemetry.addData("  Input", "A:%.2f L:%.2f Y:%.2f",
                controls.getAxial(), controls.getLateral(), controls.getYaw());

        // Shooter status
        telemetry.addLine("");
        telemetry.addLine("🎯 SHOOTER (Gamepad 2):");
        telemetry.addData("  State", shooter.getState());
        telemetry.addData("  Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("  Current RPM", "%.0f (%.1f%%)",
                shooter.getAverageRPM(), shooter.getRPMPercentage());
        telemetry.addData("  Ready", shooter.isReady() ? "✅ YES" : "⏳ NO");

        if (shotFired) {
            telemetry.addLine("");
            telemetry.addLine("💥 SHOT FIRED!");
        }

        // Control mode info
        telemetry.addLine("");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addData("Control Mode", controls.getControlMode());
        telemetry.addData("Drive Sensitivity", "%.2f", controls.getDriveSensitivity());

        telemetry.update();
    }
}

