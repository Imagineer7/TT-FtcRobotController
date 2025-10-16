/* Copyright (c) 2024 Dryw Wade. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Position Test Auto - AprilTag Localization Testing Mode
 *
 * This mode continuously reports robot position using FTC SDK AprilTag localization
 * without any motor control. Designed for manual testing by dragging the robot
 * around to verify positioning calculations and zone detection.
 *
 * Based on FTC's ConceptAprilTagLocalization sample.
 */
@Autonomous(name="Position Test Auto", group="Testing")
public class PositionTestAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * Camera position and orientation on the robot.
     *
     * For rear-facing camera setup:
     * - Position: Camera offset from robot center (x, y, z in inches)
     * - Orientation: Yaw=180° (facing backward), Pitch=-90° (horizontal), Roll=0°
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            19.5, 26.8, 0, 0); // Final calibration: X offset -3 (22.5-3=19.5), Y offset +3 (23.8+3=26.8)
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 0, 0); // Rear-facing camera (180° yaw)

    // Vision system components
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Field mapping system
    private FieldMap fieldMap;

    // Timing and statistics
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime lastDetectionTime = new ElapsedTime();
    private int totalDetections = 0;
    private int detectionsThisSecond = 0;
    private ElapsedTime detectionRateTimer = new ElapsedTime();

    // Current robot position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private boolean hasValidPosition = false;
    private String lastDetectedTagName = "None";
    private int lastDetectedTagId = -1;

    // Debug information for raw detection data
    private double rawX = 0.0;
    private double rawY = 0.0;
    private double rawZ = 0.0;
    private double rawYaw = 0.0;
    private double rawPitch = 0.0;
    private double rawRoll = 0.0;

    // AprilTag position data from SDK
    private double tagX = 0.0;
    private double tagY = 0.0;
    private double tagZ = 0.0;
    private double tagYaw = 0.0;
    private double tagPitch = 0.0;
    private double tagRoll = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Position Test Auto...");
        telemetry.update();

        // Initialize vision and field mapping systems
        initAprilTag();
        fieldMap = new FieldMap(FieldMap.Alliance.RED); // Default to RED for testing

        telemetry.addData("Status", "Ready for start");
        telemetry.addData("Mode", "Position testing - NO MOTOR CONTROL");
        telemetry.addData("Vision", visionPortal != null ? "Initialized" : "FAILED");
        telemetry.addData("Camera", "Rear-facing (180° yaw)");
        telemetry.addData("Instructions", "Manually move robot to test positioning");
        telemetry.addData(">", "Touch START to begin testing");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Position Test Auto started - drag robot around!");
            telemetry.update();

            runtime.reset();
            lastDetectionTime.reset();
            detectionRateTimer.reset();

            // Main testing loop
            while (opModeIsActive()) {
                try {
                    // Process AprilTag detections
                    processAprilTagDetections();

                    // Update detection rate statistics
                    updateDetectionStats();

                    // Display comprehensive telemetry
                    displayTelemetry();

                } catch (Exception e) {
                    telemetry.addData("ERROR", e.getMessage());
                    telemetry.update();
                }

                // Run at 20Hz to balance responsiveness and CPU usage
                sleep(50);
            }

            // Cleanup
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }

    /**
     * Initialize the AprilTag processor with rear-facing camera configuration.
     */
    private void initAprilTag() {
        try {
            // Create the AprilTag processor with lens intrinsics for better accuracy
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawTagOutline(true)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setCameraPose(cameraPosition, cameraOrientation)
                    // Use C920 640x480 calibration values from official SDK
                    .setLensIntrinsics(622.001, 622.001, 319.803, 241.251)
                    .build();

            // Create the vision portal with optimized settings for C290
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera)
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
                // Set camera resolution optimized for AprilTag detection
                builder.setCameraResolution(new android.util.Size(640, 480));
            } else {
                // Use built-in camera if webcam not available
                builder.setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK);
            }

            // Add the AprilTag processor
            builder.addProcessor(aprilTag);

            // Build the Vision Portal
            visionPortal = builder.build();

            // Apply camera settings for better AprilTag detection
            if (USE_WEBCAM) {
                applyCameraSettings();
            }

            telemetry.addData("Vision", "Initialized successfully with C290 optimization");
        } catch (Exception e) {
            telemetry.addData("Vision Error", e.getMessage());
            visionPortal = null;
            aprilTag = null;
        }
    }

    /**
     * Apply optimal camera settings for the Logitech C290 webcam for AprilTag detection.
     */
    private void applyCameraSettings() {
        try {
            // Wait for camera to be ready
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }

            // Get camera controls
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            // Apply settings optimized for AprilTag detection on C290
            if (exposureControl != null) {
                // Set manual exposure to reduce motion blur and improve tag detection
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(10, TimeUnit.MILLISECONDS); // Fast exposure for reduced blur
            }

            if (gainControl != null) {
                // Set moderate gain for good contrast without too much noise
                gainControl.setGain(100); // Moderate gain value
            }

            telemetry.addData("C290 Camera Settings", "Applied - Exposure: 10ms, Gain: 100");

        } catch (Exception e) {
            telemetry.addData("Camera Settings Error", e.getMessage());
        }
    }

    /**
     * Process AprilTag detections and update robot position.
     */
    private void processAprilTagDetections() {
        if (aprilTag == null) return;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (!currentDetections.isEmpty()) {
            // Process the first valid detection (with metadata and robotPose)
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.robotPose != null &&
                    !detection.metadata.name.contains("Obelisk")) {

                    // Update robot position from SDK calculation
                    robotX = detection.robotPose.getPosition().x;
                    robotY = detection.robotPose.getPosition().y;
                    robotHeading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    // Update raw detection data for debugging
                    rawX = detection.robotPose.getPosition().x;
                    rawY = detection.robotPose.getPosition().y;
                    rawZ = detection.robotPose.getPosition().z;
                    rawYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    rawPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                    rawRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);

                    // Update AprilTag field position data from metadata
                    if (detection.metadata.fieldPosition != null) {
                        tagX = detection.metadata.fieldPosition.get(0); // X coordinate
                        tagY = detection.metadata.fieldPosition.get(1); // Y coordinate
                        tagZ = detection.metadata.fieldPosition.get(2); // Z coordinate
                    }
                    if (detection.metadata.fieldOrientation != null) {
                        // For now, just store the quaternion components directly
                        // Most AprilTags face forward (0 yaw) so we'll use simple values
                        tagYaw = 0.0;  // Default orientation for most tags
                        tagPitch = 0.0;
                        tagRoll = 0.0;

                        // If you need actual quaternion conversion, you'd need to implement
                        // quaternion-to-euler conversion manually
                    }

                    // Update tracking variables
                    hasValidPosition = true;
                    lastDetectedTagName = detection.metadata.name;
                    lastDetectedTagId = detection.id;
                    lastDetectionTime.reset();

                    // Update statistics
                    totalDetections++;
                    detectionsThisSecond++;

                    // Only process the first valid detection
                    break;
                }
            }
        }
    }

    /**
     * Update detection rate statistics.
     */
    private void updateDetectionStats() {
        // Reset detection rate counter every second
        if (detectionRateTimer.seconds() >= 1.0) {
            detectionRateTimer.reset();
            detectionsThisSecond = 0;
        }
    }

    /**
     * Display comprehensive telemetry including position, zones, and debug info.
     */
    private void displayTelemetry() {
        telemetry.addData("=== POSITION TEST MODE ===", "");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Mode", "TESTING - No motor control");
        telemetry.addLine();

        // Vision system status
        telemetry.addData("Vision System", visionPortal != null ? "ACTIVE" : "FAILED");
        telemetry.addData("Camera Config", "Rear-facing (180° yaw)");

        // Detection statistics
        telemetry.addData("Total Detections", totalDetections);
        telemetry.addData("Detection Rate", "%d/sec", detectionsThisSecond);
        telemetry.addData("Last Detection", "%.1f sec ago", lastDetectionTime.seconds());
        telemetry.addLine();

        // Current position data
        if (hasValidPosition && lastDetectionTime.seconds() < 2.0) {
            telemetry.addData("=== ROBOT POSITION ===", "");
            telemetry.addData("Last Detected Tag", "%s (ID %d)", lastDetectedTagName, lastDetectedTagId);
            telemetry.addData("Robot X", "%.1f inches", robotX);
            telemetry.addData("Robot Y", "%.1f inches", robotY);
            telemetry.addData("Robot Heading", "%.1f degrees", robotHeading);

            // Distance from field center
            double distanceFromCenter = Math.sqrt(robotX * robotX + robotY * robotY);
            telemetry.addData("Distance from Center", "%.1f inches", distanceFromCenter);

            // Angle to field center
            double angleToCenter = Math.atan2(-robotY, -robotX) * 180.0 / Math.PI;
            double headingErrorToCenter = normalizeAngle(angleToCenter - robotHeading);
            telemetry.addData("Angle to Center", "%.1f degrees", angleToCenter);
            telemetry.addData("Heading Error to Center", "%.1f degrees", headingErrorToCenter);
            telemetry.addLine();

            // Zone detection using FieldMap
            displayZoneAnalysis();

        } else {
            telemetry.addData("=== DETECTION STATUS ===", "");
            telemetry.addData("AprilTag Status", "NO VALID TAG DETECTED");
            telemetry.addData("Position", "UNKNOWN");
            telemetry.addData("Note", "Move robot to see AprilTags");
        }

        // Debug information for raw detection data
        telemetry.addLine();
        telemetry.addData("=== RAW DETECTION DATA ===", "");
        telemetry.addData("Raw X", "%.3f", rawX);
        telemetry.addData("Raw Y", "%.3f", rawY);
        telemetry.addData("Raw Z", "%.3f", rawZ);
        telemetry.addData("Raw Yaw", "%.3f", rawYaw);
        telemetry.addData("Raw Pitch", "%.3f", rawPitch);
        telemetry.addData("Raw Roll", "%.3f", rawRoll);

        // AprilTag position data
        telemetry.addLine();
        telemetry.addData("=== APRILTAG POSITION DATA ===", "");
        telemetry.addData("Tag X", "%.3f", tagX);
        telemetry.addData("Tag Y", "%.3f", tagY);
        telemetry.addData("Tag Z", "%.3f", tagZ);
        telemetry.addData("Tag Yaw", "%.3f", tagYaw);
        telemetry.addData("Tag Pitch", "%.3f", tagPitch);
        telemetry.addData("Tag Roll", "%.3f", tagRoll);

        // Position error analysis
        telemetry.addLine();
        telemetry.addData("=== POSITION ERROR ANALYSIS ===", "");
        if (hasValidPosition) {
            double expectedX = 0.0; // Expected when robot at field center
            double expectedY = 0.0;
            double errorX = robotX - expectedX;
            double errorY = robotY - expectedY;
            double totalError = Math.sqrt(errorX * errorX + errorY * errorY);

            telemetry.addData("Expected Position", "%.1f, %.1f", expectedX, expectedY);
            telemetry.addData("Position Error X", "%.1f inches", errorX);
            telemetry.addData("Position Error Y", "%.1f inches", errorY);
            telemetry.addData("Total Position Error", "%.1f inches", totalError);

            // Calculate what camera offset might fix this
            telemetry.addData("Suggested Camera X Offset", "%.1f inches", -errorX);
            telemetry.addData("Suggested Camera Y Offset", "%.1f inches", -errorY);
        }

        telemetry.addLine();
        telemetry.addData("Instructions", "Drag robot around to test positioning");
        telemetry.addData("Controls", "D-pad Up/Down: Resume/Stop camera");

        // Handle camera controls
        if (gamepad1.dpad_down && visionPortal != null) {
            visionPortal.stopStreaming();
            telemetry.addData("Camera", "STREAMING STOPPED");
        } else if (gamepad1.dpad_up && visionPortal != null) {
            visionPortal.resumeStreaming();
            telemetry.addData("Camera", "STREAMING RESUMED");
        }

        telemetry.update();
    }

    /**
     * Display detailed zone analysis using FieldMap.
     */
    private void displayZoneAnalysis() {
        telemetry.addData("=== ZONE ANALYSIS ===", "");

        // Get comprehensive zone information
        String zoneInfo = fieldMap.getZoneInfo(robotX, robotY);
        telemetry.addData("Current Zones", zoneInfo);

        // Individual zone checks for detailed feedback
        if (fieldMap.isInLaunchZone(robotX, robotY)) {
            if (fieldMap.isInLongRangeLaunchZone(robotX, robotY)) {
                telemetry.addData("Launch Zone", "LONG RANGE");
            } else if (fieldMap.isInShortRangeLaunchZone(robotX, robotY)) {
                telemetry.addData("Launch Zone", "SHORT RANGE");
            }
        }

        if (fieldMap.isInParkingZone(robotX, robotY)) {
            if (fieldMap.isInRedParkingZone(robotX, robotY)) {
                telemetry.addData("Parking Zone", "RED");
            } else if (fieldMap.isInBlueParkingZone(robotX, robotY)) {
                telemetry.addData("Parking Zone", "BLUE");
            }
        }

        if (fieldMap.isInLoadingZone(robotX, robotY)) {
            if (fieldMap.isInRedLoadingZone(robotX, robotY)) {
                telemetry.addData("Loading Zone", "RED");
            } else if (fieldMap.isInBlueLoadingZone(robotX, robotY)) {
                telemetry.addData("Loading Zone", "BLUE");
            }
        }

        // Alliance area detection
        if (fieldMap.isInRedAllianceArea(robotX, robotY)) {
            telemetry.addData("Alliance Area", "RED");
        } else if (fieldMap.isInBlueAllianceArea(robotX, robotY)) {
            telemetry.addData("Alliance Area", "BLUE");
        }

        // Distance to key field elements
        FieldMap.FieldPosition redGoal = fieldMap.getLocation("RED_GOAL");
        FieldMap.FieldPosition blueGoal = fieldMap.getLocation("BLUE_GOAL");
        if (redGoal != null && blueGoal != null) {
            double distToRedGoal = Math.sqrt(Math.pow(robotX - redGoal.x, 2) + Math.pow(robotY - redGoal.y, 2));
            double distToBlueGoal = Math.sqrt(Math.pow(robotX - blueGoal.x, 2) + Math.pow(robotY - blueGoal.y, 2));
            telemetry.addData("Distance to Red Goal", "%.1f inches", distToRedGoal);
            telemetry.addData("Distance to Blue Goal", "%.1f inches", distToBlueGoal);
        }
    }

    /**
     * Normalize angle to [-180, 180] degrees.
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
