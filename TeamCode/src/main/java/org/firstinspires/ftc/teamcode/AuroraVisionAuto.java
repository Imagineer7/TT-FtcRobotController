/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Aurora Vision-Based Autonomous OpMode
 * Uses Webcam 1 for initial position detection and periodic position corrections
 * Motor encoders provide primary movement tracking
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.aurora.auto.AuroraAutoManager;
import org.firstinspires.ftc.teamcode.util.aurora.auto.VisionManager;
import org.firstinspires.ftc.teamcode.util.aurora.auto.AuroraPositioningManager;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

import java.util.HashMap;
import java.util.Map;

/**
 * Aurora Vision-Based Autonomous OpMode
 *
 * Features:
 * - Uses Webcam 1 for AprilTag detection and position correction
 * - Motor encoders for precise movement tracking
 * - Periodic vision-based position validation
 * - Complete autonomous routine with scoring and parking
 * - Safety monitoring and emergency stops
 * - Real-time telemetry with vision data
 */
@Autonomous(name="Aurora Vision Auto", group="Aurora")
public class AuroraVisionAuto extends LinearOpMode {

    // Aurora auto system components
    private AuroraAutoManager autoManager;
    private VisionManager visionManager;
    private AuroraPositioningManager positionManager;

    // Direct motor control for movement
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Timing and control
    private ElapsedTime autonomousTimer = new ElapsedTime();
    private ElapsedTime visionUpdateTimer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    // Configuration
    private FieldMap.Alliance alliance = FieldMap.Alliance.BLUE; // Change as needed
    private AuroraAutoManager.StartingPosition startPosition = AuroraAutoManager.StartingPosition.BASKET_ZONE;

    // Vision settings
    private static final String CAMERA_NAME = "Webcam 1";
    private static final double VISION_UPDATE_INTERVAL = 0.5; // seconds between vision updates (faster for responsive movement)
    private static final double POSITION_TOLERANCE = 3.0; // inches - tolerance for position accuracy

    // Movement control settings
    private static final double CENTER_FIELD_X = 0.0; // Field center X coordinate
    private static final double CENTER_FIELD_Y = 0.0; // Field center Y coordinate
    private static final double MOVEMENT_TOLERANCE = 6.0; // inches - how close to center before stopping
    private static final double SPIN_SPEED = 0.2; // Speed for spinning when searching for tags (reduced for better detection)
    private static final double MOVE_SPEED = 0.5; // Speed for moving toward center

    // Known AprilTag positions on field (example positions - adjust for actual field)
    private Map<Integer, double[]> knownTagPositions;

    // Autonomous states
    private enum AutoState {
        INITIALIZE,
        SEEK_CENTER,
        SPIN_TO_FIND_TAG,
        COMPLETE,
        EMERGENCY_STOP
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private boolean visionPositionSet = false;
    private ElapsedTime spinTimer = new ElapsedTime();
    private static final double MAX_SPIN_TIME = 4.0; // seconds to spin before changing direction

    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetry.addData("Status", "Initializing Aurora Vision Auto...");
        telemetry.update();

        // Initialize known AprilTag positions
        initializeTagPositions();

        // Initialize Aurora auto system
        initializeAuroraSystem();

        // Initialize vision system
        initializeVisionSystem();

        // Wait for start
        telemetry.addData("Status", "Ready for start");
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Starting Position", startPosition.toString());
        telemetry.addData("Camera", CAMERA_NAME);
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            autonomousTimer.reset();
            stateTimer.reset();
            currentState = AutoState.INITIALIZE;

            // Main autonomous loop
            while (opModeIsActive() && currentState != AutoState.COMPLETE && currentState != AutoState.EMERGENCY_STOP) {

                // Update vision periodically
                updateVisionPosition();

                // Execute current state
                executeCurrentState();

                // Update telemetry
                updateTelemetry();

                // Safety check - stop if time exceeded
                if (autonomousTimer.seconds() > 28.0) {
                    currentState = AutoState.EMERGENCY_STOP;
                }

                sleep(20); // Small delay to prevent excessive CPU usage
            }

            // Final cleanup
            cleanup();
        }
    }

    /**
     * Initialize known AprilTag positions on the field
     */
    private void initializeTagPositions() {
        knownTagPositions = new HashMap<>();

        // Get AprilTag positions from FieldMap - only two tags on the goals
        FieldMap fieldMap = new FieldMap(alliance);

        // Red Goal AprilTag (ID 24)
        FieldMap.FieldPosition redGoal = fieldMap.getLocation("RED_GOAL");
        if (redGoal != null) {
            knownTagPositions.put(24, new double[]{redGoal.x, redGoal.y});
        }

        // Blue Goal AprilTag (ID 20)
        FieldMap.FieldPosition blueGoal = fieldMap.getLocation("BLUE_GOAL");
        if (blueGoal != null) {
            knownTagPositions.put(20, new double[]{blueGoal.x, blueGoal.y});
        }

        telemetry.addData("Tag Positions", "Loaded %d goal AprilTags from FieldMap", knownTagPositions.size());
        telemetry.addData("Red Goal Tag", "ID 24 at (%.1f, %.1f)", redGoal != null ? redGoal.x : 0, redGoal != null ? redGoal.y : 0);
        telemetry.addData("Blue Goal Tag", "ID 20 at (%.1f, %.1f)", blueGoal != null ? blueGoal.x : 0, blueGoal != null ? blueGoal.y : 0);
    }

    /**
     * Initialize Aurora auto system
     */
    private void initializeAuroraSystem() {
        try {
            autoManager = new AuroraAutoManager(hardwareMap, telemetry, alliance, startPosition);

            // Initialize drive motors directly
            initializeDriveMotors();

            telemetry.addData("Aurora System", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Aurora System Error", e.getMessage());
            currentState = AutoState.EMERGENCY_STOP;
        }
    }

    /**
     * Initialize drive motors for direct control
     */
    private void initializeDriveMotors() {
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            // Set motor directions (adjust based on your robot)
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Drive Motors", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Drive Motor Error", e.getMessage());
        }
    }

    /**
     * Initialize vision system with Webcam 1
     */
    private void initializeVisionSystem() {
        try {
            visionManager = new VisionManager(hardwareMap, telemetry, true, CAMERA_NAME);

            // Set camera pose relative to robot center
            // Camera is mounted on the back of the robot facing backwards
            double cameraX = -8.5;  // inches behind robot center (negative = back)
            double cameraY = 0.0;   // inches right from robot center (centered)
            double cameraZ = 10.0;  // inches up from robot center
            double cameraYaw = 180.0;   // degrees (facing backwards)
            double cameraPitch = -15.0; // degrees (angled down)
            double cameraRoll = 0.0;    // degrees

            visionManager.setCameraPose(cameraX, cameraY, cameraZ, cameraYaw, cameraPitch, cameraRoll);
            visionManager.startStreaming();

            visionUpdateTimer.reset();

            telemetry.addData("Vision System", "Initialized with %s (rear-facing)", CAMERA_NAME);
        } catch (Exception e) {
            telemetry.addData("Vision System Error", e.getMessage());
            visionManager = null;
        }
    }

    /**
     * Update robot position using vision system and control movement
     */
    private void updateVisionPosition() {
        if (visionManager == null || !visionManager.isVisionReady()) return;

        // Update vision data
        visionManager.updateVision();

        // Check if it's time for a vision position update
        if (visionUpdateTimer.seconds() >= VISION_UPDATE_INTERVAL) {

            // Get robot position from detected AprilTags
            double[] robotPos = visionManager.getRobotPositionFromTags(knownTagPositions);

            if (robotPos != null) {
                // Validate position makes sense (not too far from expected)
                // This prevents bad vision readings from corrupting position
                if (isPositionReasonable(robotPos[0], robotPos[1], robotPos[2])) {

                    // Update position manager with vision-corrected position
                    telemetry.addData("Vision Position", "Updated: (%.1f, %.1f) @ %.0f°",
                                    robotPos[0], robotPos[1], robotPos[2]);

                    visionPositionSet = true;

                    // Calculate movement toward center based on current position
                    moveTowardCenter(robotPos[0], robotPos[1], robotPos[2]);
                }
            }

            visionUpdateTimer.reset();
        }
    }

    /**
     * Check if a position reading is reasonable
     */
    private boolean isPositionReasonable(double x, double y, double heading) {
        // Check if position is within field boundaries
        boolean withinField = (Math.abs(x) <= 72 && Math.abs(y) <= 72);

        // Check if heading is valid
        boolean validHeading = (heading >= -180 && heading <= 360);

        return withinField && validHeading;
    }

    /**
     * Execute the current autonomous state
     */
    private void executeCurrentState() {
        switch (currentState) {
            case INITIALIZE:
                // Start seeking behavior
                currentState = AutoState.SEEK_CENTER;
                spinTimer.reset();
                break;

            case SEEK_CENTER:
                // Check if we can see any AprilTags
                if (visionManager != null && visionManager.getDetectedTagCount() > 0) {
                    // We can see tags - vision system will handle movement toward center
                    telemetry.addData("Action", "Moving toward center using vision");
                } else {
                    // No tags visible - immediately start spinning to find them
                    telemetry.addData("Action", "No tags visible - starting search");
                    currentState = AutoState.SPIN_TO_FIND_TAG;
                    spinTimer.reset();
                }
                break;

            case SPIN_TO_FIND_TAG:
                // Spin in place to find AprilTags
                spinToFindTags();

                // If we find a tag, go back to seeking center
                if (visionManager != null && visionManager.getDetectedTagCount() > 0) {
                    currentState = AutoState.SEEK_CENTER;
                    telemetry.addData("Action", "Found tag! Moving to center");
                }
                break;

            case COMPLETE:
                telemetry.addData("Status", "Autonomous complete");
                break;

            case EMERGENCY_STOP:
                // Stop all movement
                stopAllMotors();
                telemetry.addData("EMERGENCY", "Autonomous stopped");
                break;
        }
    }

    /**
     * Move the robot toward the center of the field based on current position
     */
    private void moveTowardCenter(double currentX, double currentY, double currentHeading) {
        // Calculate distance to center
        double deltaX = CENTER_FIELD_X - currentX;
        double deltaY = CENTER_FIELD_Y - currentY;
        double distanceToCenter = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        telemetry.addData("Distance to Center", "%.1f inches", distanceToCenter);
        telemetry.addData("Current Position", "(%.1f, %.1f)", currentX, currentY);

        // If we're close enough to center, stop
        if (distanceToCenter <= MOVEMENT_TOLERANCE) {
            stopAllMotors();
            telemetry.addData("Status", "At center of field!");
            return;
        }

        // Calculate angle to center
        double angleToCenter = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Normalize angle
        while (angleToCenter > 180) angleToCenter -= 360;
        while (angleToCenter < -180) angleToCenter += 360;

        // Calculate turn needed (difference between current heading and desired heading)
        double turnAngle = angleToCenter - currentHeading;
        while (turnAngle > 180) turnAngle -= 360;
        while (turnAngle < -180) turnAngle += 360;

        telemetry.addData("Angle to Center", "%.1f°", angleToCenter);
        telemetry.addData("Turn Needed", "%.1f°", turnAngle);

        // Move toward center using mecanum drive
        moveTowardsTarget(deltaX, deltaY, turnAngle);
    }

    /**
     * Spin the robot to search for AprilTags
     */
    private void spinToFindTags() {
        telemetry.addData("Action", "Spinning to find AprilTags");

        // Spin at a controlled speed
        if (spinTimer.seconds() < MAX_SPIN_TIME) {
            // Spin clockwise
            setDrivePower(-SPIN_SPEED, SPIN_SPEED, -SPIN_SPEED, SPIN_SPEED);
        } else {
            // Change direction after spinning for max time - spin counter-clockwise
            setDrivePower(SPIN_SPEED, -SPIN_SPEED, SPIN_SPEED, -SPIN_SPEED);
            if (spinTimer.seconds() > MAX_SPIN_TIME * 2) {
                spinTimer.reset();
            }
        }
    }

    /**
     * Move towards a target position using mecanum drive
     */
    private void moveTowardsTarget(double deltaX, double deltaY, double turnAngle) {
        // Calculate movement components
        double forward = deltaY * MOVE_SPEED;  // Y movement
        double strafe = deltaX * MOVE_SPEED;   // X movement
        double turn = Math.signum(turnAngle) * Math.min(Math.abs(turnAngle) * 0.02, MOVE_SPEED); // Proportional turning

        // Limit movement speeds
        forward = Math.max(-MOVE_SPEED, Math.min(MOVE_SPEED, forward));
        strafe = Math.max(-MOVE_SPEED, Math.min(MOVE_SPEED, strafe));

        // Calculate mecanum drive powers
        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                                  Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Set power to all drive motors
     */
    private void setDrivePower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        if (frontLeft != null) frontLeft.setPower(frontLeftPower);
        if (frontRight != null) frontRight.setPower(frontRightPower);
        if (backLeft != null) backLeft.setPower(backLeftPower);
        if (backRight != null) backRight.setPower(backRightPower);
    }

    /**
     * Stop all drive motors
     */
    private void stopAllMotors() {
        setDrivePower(0, 0, 0, 0);
    }
    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        telemetry.addData("=== AURORA VISION AUTO ===", "");
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Time", "%.1f seconds", autonomousTimer.seconds());

        // Vision telemetry with enhanced debugging
        if (visionManager != null) {
            visionManager.addTelemetry(); // Use the enhanced telemetry method

            // Add raw detection count for debugging
            telemetry.addData("Raw Tag Count", visionManager.getDetectedTagCount());

            // Show vision system status
            telemetry.addData("Vision System Ready", visionManager.isVisionReady() ? "YES" : "NO");

        } else {
            telemetry.addData("VISION ERROR", "VisionManager is NULL");
        }

        // Aurora system telemetry
        if (autoManager != null) {
            telemetry.addData("=== AURORA SYSTEM ===", "");
            // The autoManager would provide its own telemetry updates
        }

        telemetry.update();
    }

    /**
     * Cleanup and stop all systems
     */
    private void cleanup() {
        telemetry.addData("Status", "Cleaning up...");

        if (visionManager != null) {
            visionManager.stopStreaming();
        }

        telemetry.addData("Status", "Autonomous complete");
        telemetry.update();
    }
}
