/* Copyright (c) 2025 FTC Team #26581 Tundra Tech. All rights reserved.
 *
 * Example OpMode demonstrating AuroraLimelightHelper usage
 */

package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.aurora.vision.AuroraLimelightHelper;
import org.firstinspires.ftc.teamcode.util.aurora.vision.AuroraLimelightHelper.Pipeline;
import org.firstinspires.ftc.teamcode.util.aurora.vision.AuroraLimelightHelper.Config;

/**
 * Example OpMode demonstrating the AuroraLimelightHelper class.
 *
 * This shows:
 * - Basic initialization with default settings
 * - Advanced configuration with Config builder
 * - Position retrieval with freshness checking
 * - Integration with localization correction
 * - Proper shutdown procedures
 */
@TeleOp(name = "Aurora Limelight Example", group = "Examples")
@Disabled
public class AuroraLimelightExample extends LinearOpMode {

    private AuroraLimelightHelper limelight;

    // For localization correction example
    private double currentRobotX = 0;
    private double currentRobotY = 0;
    private double currentRobotHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ================================================
        // OPTION 1: Basic initialization (default settings)
        // ================================================
        limelight = AuroraLimelightHelper.getInstance();
        limelight.initialize(hardwareMap, telemetry);

        // ================================================
        // OPTION 2: Initialize with specific pipeline
        // ================================================
        // limelight = AuroraLimelightHelper.getInstance();
        // limelight.initialize(hardwareMap, telemetry, Pipeline.APRILTAG_3D);

        // ================================================
        // OPTION 3: Full configuration with builder
        // ================================================
        // limelight = AuroraLimelightHelper.getInstance();
        // Config config = new Config()
        //         .setLimelightName("limelight")         // Hardware map name
        //         .setPipeline(Pipeline.APRILTAG_3D)     // MegaTag localization
        //         .setMaxDataAgeMs(1000)                 // 1 second max staleness
        //         .setEnableTelemetry(true)              // Show debug info
        //         .setAutoStart(true);                   // Start polling immediately
        // limelight.initialize(hardwareMap, telemetry, config);

        telemetry.addData("Status", "Limelight initialized");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Limelight data (call every loop iteration)
            limelight.updateWithStatus(); // Use update() if you don't need status info

            // ================================================
            // EXAMPLE: Check for fresh position and use it
            // ================================================
            if (limelight.hasValidPosition()) {
                // Data is fresh (less than configured max age)
                Pose3D pose = limelight.getLastBotPose();

                // Use for localization correction
                correctLocalization(pose);

                telemetry.addData("Position Valid", "YES - Age: %dms",
                        limelight.getLastPositionAgeMs());
            } else {
                telemetry.addData("Position Valid", "NO - Using dead reckoning");
            }

            // ================================================
            // EXAMPLE: Get position only if very fresh (500ms)
            // ================================================
            Pose3D freshPose = limelight.getFreshBotPose(500);
            if (freshPose != null) {
                telemetry.addData("Fresh Pose", "X: %.2f, Y: %.2f",
                        freshPose.getPosition().x,
                        freshPose.getPosition().y);
            }

            // ================================================
            // EXAMPLE: Manual position access
            // ================================================
            double x = limelight.getX();
            double y = limelight.getY();
            double heading = limelight.getHeadingDegrees();

            // ================================================
            // EXAMPLE: Access raw targeting data
            // ================================================
            double tx = limelight.getTx(); // Horizontal offset to target
            double ty = limelight.getTy(); // Vertical offset to target

            // ================================================
            // EXAMPLE: Pipeline switching at runtime
            // ================================================
            if (gamepad1.a) {
                limelight.switchPipeline(Pipeline.APRILTAG_3D);
            }
            if (gamepad1.b) {
                limelight.switchPipeline(Pipeline.NEURAL_DETECTOR);
            }

            // Add telemetry
            limelight.addTelemetry(); // Full telemetry
            // limelight.addCompactTelemetry(); // Or use compact version

            telemetry.update();
        }

        // IMPORTANT: Shutdown when done
        limelight.shutdown();
        // OR: AuroraLimelightHelper.resetInstance();
    }

    /**
     * Example method showing how to use Limelight data for localization correction.
     * This would integrate with your Pedro Pathing or other localization system.
     */
    private void correctLocalization(Pose3D limelightPose) {
        if (limelightPose == null || limelightPose.getPosition() == null) {
            return;
        }

        // Get Limelight's position estimate
        double llX = limelightPose.getPosition().x;
        double llY = limelightPose.getPosition().y;
        double llHeading = limelightPose.getOrientation().getYaw();

        // Calculate error between current estimate and Limelight
        double errorX = llX - currentRobotX;
        double errorY = llY - currentRobotY;
        double errorHeading = llHeading - currentRobotHeading;

        // Apply correction (you might want to use a filter here)
        // This is a simple example - real implementation would be smoother
        double correctionFactor = 0.3; // How much to trust Limelight vs odometry

        currentRobotX += errorX * correctionFactor;
        currentRobotY += errorY * correctionFactor;
        currentRobotHeading += errorHeading * correctionFactor;

        telemetry.addData("Correction Applied", "dX: %.2f, dY: %.2f",
                errorX * correctionFactor, errorY * correctionFactor);
    }
}

