package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroAutonomousBuilder;
import org.firstinspires.ftc.teamcode.util.aurora.vision.AuroraAprilTagLocalizer;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

/**
 * Example OpMode demonstrating AprilTag-based starting pose detection
 *
 * This example shows how to:
 * 1. Initialize the AprilTag localizer
 * 2. Configure the builder to use AprilTag detection for starting pose
 * 3. Set a fallback default pose if AprilTag detection fails
 * 4. Initialize the starting pose before running autonomous
 */
@Disabled
@Autonomous(name = "AprilTag Start Pose Example", group = "Examples")
public class AprilTagStartPoseExample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Pedro Pathing follower
        Follower follower = Constants.createFollower(hardwareMap);

        // Initialize AprilTag localizer
        AuroraAprilTagLocalizer aprilTagLocalizer = new AuroraAprilTagLocalizer(
            hardwareMap,
            telemetry,
            FieldMap.Alliance.BLUE,  // Change to RED for red alliance
            "Webcam 1"               // Your webcam name from robot config
        );

        // Create autonomous builder
        PedroAutonomousBuilder builder = new PedroAutonomousBuilder(follower)
            // Set the AprilTag localizer
            .withAprilTagLocalizer(aprilTagLocalizer)

            // Enable AprilTag detection for starting pose
            .setUseAprilTagForStart(true)

            // Set fallback pose if AprilTag detection fails
            // This will be used if no AprilTags are detected or timeout occurs
            .setDefaultStartPose(9, 63, Math.toRadians(0))  // Example: Blue side starting position

            // Optional: Configure detection parameters
            .setAprilTagDetectionTimeout(3.0)       // Wait up to 3 seconds for AprilTag
            .setMinAprilTagConfidence(0.6);         // Require 60% confidence minimum

        // Display configuration to driver
        telemetry.addData("Status", "Initialized");
        telemetry.addData("AprilTag Detection", "Enabled");
        telemetry.addData("Default Pose", "x=9, y=63, heading=0°");
        telemetry.addData("Detection Timeout", "3.0s");
        telemetry.addData("Min Confidence", "60%");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Initialize starting pose (will attempt AprilTag detection)
        telemetry.addData("Status", "Detecting AprilTags...");
        telemetry.update();

        Pose startPose = builder.initializeStartingPose();

        // Display the detected/default pose
        telemetry.addData("Status", "Ready");
        telemetry.addData("Start Pose", String.format("x=%.1f, y=%.1f, h=%.1f°",
            startPose.getX(),
            startPose.getY(),
            Math.toDegrees(startPose.getHeading())));

        // Check if AprilTag was used
        if (aprilTagLocalizer.hasValidPosition()) {
            telemetry.addData("Pose Source", "AprilTag Detection");
            telemetry.addData("Confidence", String.format("%.1f%%",
                aprilTagLocalizer.getPositionConfidence() * 100));
        } else {
            telemetry.addData("Pose Source", "Default Pose (No AprilTag)");
        }
        telemetry.update();

        // Add your autonomous paths here
        // builder.addPath(yourPath1)
        //        .addPath(yourPath2)
        //        ...

        // Start autonomous execution
        // builder.start();

        // Main loop
        // while (opModeIsActive() && !builder.isFinished()) {
        //     follower.update();
        //     String currentStep = builder.update();
        //
        //     telemetry.addData("Current Step", currentStep);
        //     telemetry.addData("Step", builder.getCurrentStepIndex() + "/" + builder.getTotalSteps());
        //     telemetry.update();
        // }

        sleep(5000); // Keep display visible for 5 seconds
    }
}

