package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.aurora.auto.AuroraLocalizationManager;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

/**
 * Aurora Center Navigation Auto
 *
 * Uses the new Aurora AprilTag localization system to autonomously navigate to field center (0,0).
 * Demonstrates real-time position feedback, vision-guided navigation, and Aurora integration.
 *
 * Features:
 * - AprilTag-guided navigation with 10-inch accuracy
 * - Real-time position correction using calibrated C290 camera
 * - Smooth movement with PID control
 * - Automatic fallback to encoder-only mode if vision is lost
 * - Dynamic speed adjustment based on distance to target
 */
@Autonomous(name="Aurora Navigate to Center", group="Aurora")
public class AuroraCenterNavigationAuto extends LinearOpMode implements AuroraLocalizationManager.AuroraPositionUpdateListener {

    // Aurora localization system
    private AuroraLocalizationManager localizationManager;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Navigation state
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private AuroraLocalizationManager.PositionSource currentSource = AuroraLocalizationManager.PositionSource.UNKNOWN;

    // Navigation targets
    private static final double TARGET_X = 0.0;  // Field center
    private static final double TARGET_Y = 0.0;  // Field center
    private static final double TARGET_HEADING = 0.0;  // Face toward blue alliance

    // Navigation parameters
    private static final double POSITION_TOLERANCE = 3.0;  // inches
    private static final double HEADING_TOLERANCE = 5.0;   // degrees
    private static final double MAX_DRIVE_SPEED = 0.6;
    private static final double MIN_DRIVE_SPEED = 0.15;
    private static final double MAX_TURN_SPEED = 0.4;

    // PID controllers (simple proportional control)
    private static final double KP_DRIVE = 0.03;  // Proportional gain for driving
    private static final double KP_STRAFE = 0.03; // Proportional gain for strafing
    private static final double KP_TURN = 0.02;   // Proportional gain for turning

    // Navigation state machine
    private enum NavigationState {
        INITIALIZING,
        WAITING_FOR_VISION,
        NAVIGATING,
        ARRIVED,
        FAILED
    }
    private NavigationState currentState = NavigationState.INITIALIZING;

    // Timing
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime navigationTimer = new ElapsedTime();

    // Performance tracking
    private int positionUpdates = 0;
    private double initialDistance = 0.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Initialize Aurora localization system
        initializeAuroraLocalization();

        // Wait for start
        telemetry.addData("Status", "Aurora Center Navigation Ready");
        telemetry.addData("Target", "Field Center (0, 0)");
        telemetry.addData("Camera", "Calibrated C290 with 10in accuracy");
        telemetry.addData(">", "Touch START to begin navigation");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            navigationTimer.reset();
            currentState = NavigationState.WAITING_FOR_VISION;
            stateTimer.reset();

            // Main navigation loop
            while (opModeIsActive() && currentState != NavigationState.ARRIVED && currentState != NavigationState.FAILED) {
                // Update localization
                localizationManager.updateLocalization();

                // Run navigation state machine
                runNavigationStateMachine();

                // Display status
                displayNavigationStatus();

                // Run at 20Hz
                sleep(50);
            }

            // Final state
            handleFinalState();

            // Cleanup
            localizationManager.shutdown();
        }
    }

    /**
     * Initialize drive motors
     */
    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions (adjust based on your robot)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Hardware", "Drive motors initialized");
    }

    /**
     * Initialize Aurora localization system
     */
    private void initializeAuroraLocalization() {
        localizationManager = new AuroraLocalizationManager(
            hardwareMap,
            telemetry,
            FieldMap.Alliance.RED,  // Change to BLUE as needed
            "Webcam 1"
        );

        localizationManager.setAuroraListener(this);
        telemetry.addData("Aurora", "Localization system initialized");
    }

    /**
     * Run the navigation state machine
     */
    private void runNavigationStateMachine() {
        switch (currentState) {
            case WAITING_FOR_VISION:
                handleWaitingForVision();
                break;

            case NAVIGATING:
                handleNavigating();
                break;

            default:
                break;
        }
    }

    /**
     * Handle waiting for vision state
     */
    private void handleWaitingForVision() {
        // Wait for reliable position data
        if (positionUpdates >= 3 && currentSource != AuroraLocalizationManager.PositionSource.UNKNOWN) {
            // Calculate initial distance for progress tracking
            initialDistance = Math.sqrt(robotX * robotX + robotY * robotY);

            currentState = NavigationState.NAVIGATING;
            stateTimer.reset();
            telemetry.addData("Navigation", "Starting navigation from (%.1f, %.1f)", robotX, robotY);
        } else if (stateTimer.seconds() > 10.0) {
            // Timeout waiting for vision
            currentState = NavigationState.FAILED;
            telemetry.addData("Error", "Vision system timeout");
        }
    }

    /**
     * Handle navigation state
     */
    private void handleNavigating() {
        // Calculate distance to target
        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Calculate heading error
        double targetHeading = Math.atan2(deltaY, deltaX) * 180.0 / Math.PI;
        double headingError = normalizeAngle(targetHeading - robotHeading);

        // Check if we've arrived
        if (distanceToTarget <= POSITION_TOLERANCE && Math.abs(headingError) <= HEADING_TOLERANCE) {
            currentState = NavigationState.ARRIVED;
            stopRobot();
            return;
        }

        // Check for timeout
        if (navigationTimer.seconds() > 30.0) {
            currentState = NavigationState.FAILED;
            stopRobot();
            return;
        }

        // Calculate drive powers
        calculateAndApplyDrivePowers(deltaX, deltaY, headingError, distanceToTarget);
    }

    /**
     * Calculate and apply drive powers for navigation
     */
    private void calculateAndApplyDrivePowers(double deltaX, double deltaY, double headingError, double distanceToTarget) {
        // Convert field-relative movement to robot-relative
        double robotHeadingRad = Math.toRadians(robotHeading);
        double robotDeltaX = deltaX * Math.cos(-robotHeadingRad) - deltaY * Math.sin(-robotHeadingRad);
        double robotDeltaY = deltaX * Math.sin(-robotHeadingRad) + deltaY * Math.cos(-robotHeadingRad);

        // Calculate base drive powers using proportional control
        double drivePower = robotDeltaX * KP_DRIVE;
        double strafePower = robotDeltaY * KP_STRAFE;
        double turnPower = headingError * KP_TURN;

        // Apply speed scaling based on distance (slow down as we approach)
        double speedScale = Math.min(1.0, distanceToTarget / 12.0); // Start slowing at 12 inches
        speedScale = Math.max(0.3, speedScale); // Minimum 30% speed

        drivePower *= speedScale;
        strafePower *= speedScale;

        // Limit powers
        drivePower = Range.clip(drivePower, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
        strafePower = Range.clip(strafePower, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
        turnPower = Range.clip(turnPower, -MAX_TURN_SPEED, MAX_TURN_SPEED);

        // Apply minimum speed threshold
        if (Math.abs(drivePower) < MIN_DRIVE_SPEED && Math.abs(drivePower) > 0.01) {
            drivePower = Math.signum(drivePower) * MIN_DRIVE_SPEED;
        }
        if (Math.abs(strafePower) < MIN_DRIVE_SPEED && Math.abs(strafePower) > 0.01) {
            strafePower = Math.signum(strafePower) * MIN_DRIVE_SPEED;
        }

        // Calculate individual motor powers (mecanum drive)
        double frontLeftPower = drivePower + strafePower + turnPower;
        double frontRightPower = drivePower - strafePower - turnPower;
        double backLeftPower = drivePower - strafePower + turnPower;
        double backRightPower = drivePower + strafePower - turnPower;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                  Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Apply powers to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Stop the robot
     */
    private void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Handle final state
     */
    private void handleFinalState() {
        stopRobot();

        if (currentState == NavigationState.ARRIVED) {
            telemetry.addData("=== NAVIGATION COMPLETE ===", "");
            telemetry.addData("Status", "Successfully reached field center!");
            telemetry.addData("Final Position", "X: %.1f, Y: %.1f", robotX, robotY);
            telemetry.addData("Final Distance", "%.1f inches from target",
                Math.sqrt(robotX * robotX + robotY * robotY));
            telemetry.addData("Navigation Time", "%.1f seconds", navigationTimer.seconds());
        } else {
            telemetry.addData("=== NAVIGATION FAILED ===", "");
            telemetry.addData("Status", "Could not reach field center");
            telemetry.addData("Current Position", "X: %.1f, Y: %.1f", robotX, robotY);
            telemetry.addData("Distance Remaining", "%.1f inches",
                Math.sqrt(robotX * robotX + robotY * robotY));
        }

        telemetry.update();
        sleep(5000); // Display results for 5 seconds
    }

    /**
     * Aurora position update callback
     */
    @Override
    public void onPositionUpdated(double x, double y, double heading, AuroraLocalizationManager.PositionSource source) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeading = heading;
        this.currentSource = source;
        this.positionUpdates++;
    }

    /**
     * Position source change callback
     */
    @Override
    public void onPositionSourceChanged(AuroraLocalizationManager.PositionSource oldSource,
                                      AuroraLocalizationManager.PositionSource newSource) {
        telemetry.addData("Position Source Changed", "%s → %s", oldSource, newSource);
    }

    /**
     * Display comprehensive navigation status
     */
    private void displayNavigationStatus() {
        telemetry.addData("=== AURORA CENTER NAVIGATION ===", "");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Position Updates", positionUpdates);
        telemetry.addLine();

        // Current position
        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("Current", "X: %.1f, Y: %.1f, H: %.1f°", robotX, robotY, robotHeading);
        telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.1f°", TARGET_X, TARGET_Y, TARGET_HEADING);

        double distanceToTarget = Math.sqrt((TARGET_X - robotX) * (TARGET_X - robotX) +
                                          (TARGET_Y - robotY) * (TARGET_Y - robotY));
        telemetry.addData("Distance to Target", "%.1f inches", distanceToTarget);
        telemetry.addData("Position Source", currentSource.toString());
        telemetry.addLine();

        // Navigation progress
        if (initialDistance > 0) {
            double progress = Math.max(0, (initialDistance - distanceToTarget) / initialDistance) * 100;
            telemetry.addData("Progress", "%.1f%%", progress);
        }

        // Aurora system status
        AuroraLocalizationManager.LocalizationStatus status = localizationManager.getStatus();
        telemetry.addData("=== AURORA STATUS ===", "");
        telemetry.addData("Vision Active", status.hasVision ? "YES" : "NO");
        telemetry.addData("Vision Confidence", "%.2f", status.visionConfidence);

        if (status.visionStatus != null) {
            telemetry.addData("Vision Success Rate", "%.1f%%",
                status.visionStatus.getSuccessRate() * 100);
        }

        telemetry.update();
    }

    /**
     * Normalize angle to [-180, 180] degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
