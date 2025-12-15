package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IntelMechanumDrive;

/**
 * BasicMechanumDriveExample - Simple TeleOp using IntelMechanumDrive
 *
 * This OpMode demonstrates how to use the IntelMechanumDrive class with AuroraHardwareConfig
 *
 * Controls:
 * - Left Stick Y: Forward/Backward
 * - Triggers: Strafe (RT = Right, LT = Left)
 * - Right Stick X: Rotation
 * - D-Pad: Fine control movement
 * - Left Bumper: Toggle slow mode
 * - Right Bumper: Toggle field-centric mode
 * - Y Button: Reset IMU heading (for field-centric)
 */
@TeleOp(name = "Basic Mecanum Drive Example", group = "Examples")
public class BasicMechanumDriveExample extends LinearOpMode {

    private AuroraHardwareConfig hardware;
    private IntelMechanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        telemetry.addLine("🤖 Initializing Basic Mecanum Drive...");
        telemetry.update();

        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();  // Initialize without odometry for simplicity

        // Initialize drive system with gamepad1
        drive = new IntelMechanumDrive(hardware, gamepad1);

        telemetry.addLine("✅ Ready to drive!");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick Y: Forward/Backward");
        telemetry.addLine("  Triggers: Strafe");
        telemetry.addLine("  Right Stick X: Rotation");
        telemetry.addLine("  Left Bumper: Toggle Slow Mode");
        telemetry.addLine("  Right Bumper: Toggle Field-Centric");
        telemetry.addLine("  Y: Reset Heading");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle mode toggles
            if (gamepad1.left_bumper) {
                drive.toggleSlowMode();
                sleep(200);  // Debounce
            }

            if (gamepad1.right_bumper) {
                drive.toggleDriveMode();
                sleep(200);  // Debounce
            }

            if (gamepad1.y) {
                drive.resetHeading();
                sleep(200);  // Debounce
            }

            // Main drive loop
            drive.drive();

            // Telemetry
            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("🎮 DRIVE STATUS");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addData("Drive Mode", drive.getDriveMode());
        telemetry.addData("Speed Mode", drive.getSpeedMode());
        telemetry.addData("Heading", "%.1f°", drive.getCurrentHeading());
        telemetry.addLine("");
        telemetry.addLine("Motor Powers:");
        telemetry.addData("  Front Left", "%.2f", drive.getFrontLeftPower());
        telemetry.addData("  Front Right", "%.2f", drive.getFrontRightPower());
        telemetry.addData("  Back Left", "%.2f", drive.getBackLeftPower());
        telemetry.addData("  Back Right", "%.2f", drive.getBackRightPower());
        telemetry.update();
    }
}

