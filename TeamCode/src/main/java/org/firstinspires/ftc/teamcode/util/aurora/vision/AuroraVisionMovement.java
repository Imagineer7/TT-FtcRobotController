package org.firstinspires.ftc.teamcode.util.aurora.vision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

/**
 * Aurora Vision Movement Controller - Real-time movement with vision corrections
 *
 * Provides the same movement capabilities as AuroraVisionAuto.java:
 * - Non-blocking movement toward targets
 * - Real-time vision-based position corrections
 * - Mecanum drive control with precision
 * - Automatic tag searching when vision is lost
 * - Smooth movement integration during autonomous
 */
public class AuroraVisionMovement {
    private AuroraVisionPositioning visionPositioning;
    private Telemetry telemetry;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Movement state
    private boolean isMoving;
    private double targetX, targetY;
    private double movementTolerance;
    private double maxMoveSpeed;
    private double maxTurnSpeed;

    // Search behavior
    private boolean isSearching;
    private long searchStartTime;
    private double searchDirection; // 1.0 for clockwise, -1.0 for counter-clockwise
    private static final long SEARCH_DIRECTION_CHANGE_TIME = 4000; // 4 seconds

    // Movement parameters matching AuroraVisionAuto
    private static final double DEFAULT_MOVE_SPEED = 0.5;
    private static final double DEFAULT_TURN_SPEED = 0.3;
    private static final double DEFAULT_SEARCH_SPEED = 0.2;
    private static final double DEFAULT_TOLERANCE = 6.0; // inches

    /**
     * Initialize vision movement controller
     */
    public AuroraVisionMovement(HardwareMap hardwareMap, Telemetry telemetry,
                               FieldMap.Alliance alliance, String cameraName) {
        this.telemetry = telemetry;

        // Initialize vision positioning
        visionPositioning = new AuroraVisionPositioning(hardwareMap, telemetry, alliance, cameraName);

        // Initialize drive motors
        initializeDriveMotors(hardwareMap);

        // Initialize movement state
        isMoving = false;
        isSearching = false;
        movementTolerance = DEFAULT_TOLERANCE;
        maxMoveSpeed = DEFAULT_MOVE_SPEED;
        maxTurnSpeed = DEFAULT_TURN_SPEED;
        searchDirection = 1.0; // Start searching clockwise

        telemetry.addData("VisionMovement", "Initialized successfully");
    }

    /**
     * Initialize drive motors
     */
    private void initializeDriveMotors(HardwareMap hardwareMap) {
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
     * Start vision system
     */
    public void startVision() {
        visionPositioning.startVision();
    }

    /**
     * Stop vision system
     */
    public void stopVision() {
        visionPositioning.stopVision();
    }

    /**
     * Update movement system - call this frequently in your main loop
     * This is NON-BLOCKING and handles all movement logic
     */
    public void update() {
        // Update vision positioning
        visionPositioning.updatePosition();

        // Handle movement based on current state
        if (isMoving) {
            updateMovementToTarget();
        } else if (isSearching) {
            updateSearchBehavior();
        } else if (!visionPositioning.hasValidPosition() && visionPositioning.getDetectedTagCount() == 0) {
            // No tags visible and no movement target - start searching
            startSearching();
        }
    }

    /**
     * Move toward a target position - NON-BLOCKING
     */
    public void moveToTarget(double x, double y) {
        targetX = x;
        targetY = y;
        isMoving = true;
        isSearching = false;

        telemetry.addData("VisionMovement", "Moving to target (%.1f, %.1f)", x, y);
    }

    /**
     * Move toward center of field
     */
    public void moveToCenter() {
        moveToTarget(0.0, 0.0);
    }

    /**
     * Stop all movement
     */
    public void stopMovement() {
        isMoving = false;
        isSearching = false;
        setDrivePower(0, 0, 0, 0);

        telemetry.addData("VisionMovement", "Movement stopped");
    }

    /**
     * Update movement toward target using vision positioning
     */
    private void updateMovementToTarget() {
        if (!visionPositioning.hasValidPosition()) {
            // No valid position - start searching for tags
            startSearching();
            return;
        }

        // Calculate distance to target
        double distance = visionPositioning.getDistanceToTarget(targetX, targetY);

        // Check if we've reached the target
        if (distance <= movementTolerance) {
            stopMovement();
            telemetry.addData("VisionMovement", "Target reached!");
            return;
        }

        // Calculate movement needed
        double currentX = visionPositioning.getCurrentX();
        double currentY = visionPositioning.getCurrentY();
        double currentHeading = visionPositioning.getCurrentHeading();

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        // Calculate angle to target
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Calculate turn needed
        double turnNeeded = angleToTarget - currentHeading;
        while (turnNeeded > 180) turnNeeded -= 360;
        while (turnNeeded < -180) turnNeeded += 360;

        // Move toward target using mecanum drive (same logic as AuroraVisionAuto)
        moveTowardsTarget(deltaX, deltaY, turnNeeded);

        telemetry.addData("Target Distance", "%.1f inches", distance);
        telemetry.addData("Turn Needed", "%.1f degrees", turnNeeded);
    }

    /**
     * Move towards target using mecanum drive - same as AuroraVisionAuto
     */
    private void moveTowardsTarget(double deltaX, double deltaY, double turnAngle) {
        // Calculate movement components
        double forward = deltaY * maxMoveSpeed;  // Y movement
        double strafe = deltaX * maxMoveSpeed;   // X movement
        double turn = Math.signum(turnAngle) * Math.min(Math.abs(turnAngle) * 0.02, maxTurnSpeed);

        // Limit movement speeds
        forward = Math.max(-maxMoveSpeed, Math.min(maxMoveSpeed, forward));
        strafe = Math.max(-maxMoveSpeed, Math.min(maxMoveSpeed, strafe));

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
     * Start searching for AprilTags
     */
    private void startSearching() {
        isSearching = true;
        isMoving = false;
        searchStartTime = System.currentTimeMillis();

        telemetry.addData("VisionMovement", "Searching for AprilTags...");
    }

    /**
     * Update search behavior - spin to find AprilTags
     */
    private void updateSearchBehavior() {
        // If we found tags, stop searching
        if (visionPositioning.getDetectedTagCount() > 0) {
            isSearching = false;
            stopMovement();
            telemetry.addData("VisionMovement", "AprilTags found! Stopping search");
            return;
        }

        // Change search direction periodically
        long searchTime = System.currentTimeMillis() - searchStartTime;
        if (searchTime > SEARCH_DIRECTION_CHANGE_TIME) {
            searchDirection *= -1.0; // Reverse direction
            searchStartTime = System.currentTimeMillis();
        }

        // Spin in place to search for tags
        double searchSpeed = DEFAULT_SEARCH_SPEED * searchDirection;
        setDrivePower(-searchSpeed, searchSpeed, -searchSpeed, searchSpeed);

        telemetry.addData("Search Direction", searchDirection > 0 ? "Clockwise" : "Counter-clockwise");
    }

    /**
     * Set power to all drive motors
     */
    private void setDrivePower(double frontLeftPower, double frontRightPower,
                              double backLeftPower, double backRightPower) {
        if (frontLeft != null) frontLeft.setPower(frontLeftPower);
        if (frontRight != null) frontRight.setPower(frontRightPower);
        if (backLeft != null) backLeft.setPower(backLeftPower);
        if (backRight != null) backRight.setPower(backRightPower);
    }

    /**
     * Set movement parameters
     */
    public void setMovementTolerance(double tolerance) {
        this.movementTolerance = tolerance;
    }

    public void setMaxMoveSpeed(double speed) {
        this.maxMoveSpeed = Math.max(0.1, Math.min(1.0, speed));
    }

    public void setMaxTurnSpeed(double speed) {
        this.maxTurnSpeed = Math.max(0.1, Math.min(1.0, speed));
    }

    /**
     * Get current robot position
     */
    public double[] getCurrentPosition() {
        return visionPositioning.getCurrentPosition();
    }

    /**
     * Check if robot has a valid position
     */
    public boolean hasValidPosition() {
        return visionPositioning.hasValidPosition();
    }

    /**
     * Check if robot is currently moving
     */
    public boolean isMoving() {
        return isMoving;
    }

    /**
     * Check if robot is searching for tags
     */
    public boolean isSearching() {
        return isSearching;
    }

    /**
     * Get number of detected AprilTags
     */
    public int getDetectedTagCount() {
        return visionPositioning.getDetectedTagCount();
    }

    /**
     * Check if vision system is ready
     */
    public boolean isVisionReady() {
        return visionPositioning.isVisionReady();
    }

    /**
     * Set manual camera exposure and gain
     */
    public boolean setManualExposure(int exposureMS, int gain) {
        return visionPositioning.setManualExposure(exposureMS, gain);
    }

    /**
     * Add telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== VISION MOVEMENT ===", "");
        telemetry.addData("Moving", isMoving ? "YES" : "NO");
        telemetry.addData("Searching", isSearching ? "YES" : "NO");

        if (isMoving) {
            telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);
            if (hasValidPosition()) {
                double distance = visionPositioning.getDistanceToTarget(targetX, targetY);
                telemetry.addData("Distance to Target", "%.1f inches", distance);
            }
        }

        // Add vision positioning telemetry
        visionPositioning.addTelemetry();
    }

    /**
     * Cleanup and close systems
     */
    public void close() {
        stopMovement();
        visionPositioning.close();
    }
}
