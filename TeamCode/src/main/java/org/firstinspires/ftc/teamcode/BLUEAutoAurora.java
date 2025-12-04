/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * AURORA Autonomous OpMode with Intelligent Odometry-Based Navigation
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager;
import org.firstinspires.ftc.teamcode.util.aurora.SmartMechanumDrive;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

/**
 * AURORA Autonomous - Intelligent Odometry-Based Autonomous OpMode
 *
 * Features:
 * - Precise odometry-based movement control
 * - Robot-relative movement commands (moveLeft, moveRight, moveForward, moveBackward)
 * - Absolute field-relative positioning
 * - Intelligent heading tracking with IMU + Odometry fusion
 * - Automatic course correction during movement
 * - PID-based position control
 * - Easy-to-use movement functions for autonomous sequences
 *
 * Movement Functions:
 * - moveLeft(inches) - Move left relative to robot orientation
 * - moveRight(inches) - Move right relative to robot orientation
 * - moveForward(inches) - Move forward relative to robot orientation
 * - moveBackward(inches) - Move backward relative to robot orientation
 * - turnToAngle(degrees) - Turn to absolute field angle
 * - turnRelative(degrees) - Turn relative to current heading
 * - moveToPosition(x, y, heading) - Move to absolute field position
 * - strafeToPosition(x, y) - Strafe to position while maintaining heading
 *
 * Usage Example:
 * <pre>
 * // Move forward 24 inches
 * moveForward(24);
 *
 * // Strafe left 12 inches
 * moveLeft(12);
 *
 * // Turn to face 90 degrees (field heading)
 * turnToAngle(90);
 *
 * // Turn 45 degrees clockwise from current heading
 * turnRelative(-45);
 * </pre>
 */
@Autonomous(name="BLUE AutoAurora", group="Timed Autonomous")
public class BLUEAutoAurora extends LinearOpMode {

    // Robot systems
    private AuroraManager robotManager;
    GoBildaPinpointDriver odometry; // Declare OpMode member for the Odometry Computer
    private SmartMechanumDrive driveSystem;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();

    // Movement control parameters
    private static final double POSITION_TOLERANCE = 1.0; // inches
    private static final double HEADING_TOLERANCE = 2.0; // degrees
    private static final double MAX_MOVE_TIME = 5.0; // seconds
    private static final double MIN_POWER = 0.15; // Minimum motor power
    private static final double MAX_POWER = 0.6; // Maximum motor power for autonomous
    private static final double SETTLING_TIME = 0.3; // seconds to remain at target before completing
    private static final int STABILITY_CHECKS = 3; // Number of consecutive checks within tolerance required

    // Smooth motion profile parameters
    private static final double ACCELERATION_DISTANCE = 6.0; // inches to accelerate over
    private static final double DECELERATION_DISTANCE = 8.0; // inches to decelerate over
    private static final double ROTATION_ACCELERATION = 20.0; // degrees to accelerate over
    private static final double ROTATION_DECELERATION = 30.0; // degrees to decelerate over
    private static final boolean USE_SMOOTH_MOTION = true; // Enable smooth acceleration/deceleration

    // PID Constants for position control
    private static final double KP_POSITION = 0.08; // Proportional gain for position
    private static final double KI_POSITION = 0.001; // Integral gain for position
    private static final double KD_POSITION = 0.02; // Derivative gain for position

    // PID Constants for heading control
    private static final double KP_HEADING = 0.025; // Proportional gain for heading
    private static final double KI_HEADING = 0.0005; // Integral gain for heading
    private static final double KD_HEADING = 0.008; // Derivative gain for heading

    // PID state variables
    private double positionErrorIntegral = 0;
    private double lastPositionError = 0;
    private double headingErrorIntegral = 0;
    private double lastHeadingError = 0;

    // Movement state tracking
    private Pose2D targetPosition;
    private Pose2D startPosition;
    private double initialHeading = 0;

    // Emergency stop flag
    private boolean emergencyStop = false;

    //odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo"); init later

    @Override
    public void runOpMode() {
        // Initialize robot systems
        telemetry.addLine("Initializing AURORA Autonomous Systems...");
        telemetry.update();

        // Initialize AuroraManager without odometry (we'll get it from hardware config)
        robotManager = new AuroraManager(hardwareMap, telemetry, false);
        driveSystem = robotManager.getDriveSystem();

        // Get odometry from the hardware config (already initialized if available)
        odometry = robotManager.getHardware().getOdometry();
        
        if (odometry == null) {
            // Try to initialize odometry if it wasn't initialized by hardware config
            initializeOdometry();
        } else {
            telemetry.addLine("✅ Using odometry from hardware config");
            telemetry.update();
        }

        // Set drive mode to precision for autonomous
        if (driveSystem != null) {
            driveSystem.setCurrentMode(SmartMechanumDrive.DriveMode.NORMAL);
        }

        telemetry.clear();
        telemetry.addLine("✅ AURORA Autonomous Ready!");
        telemetry.addLine("");
        telemetry.addLine("Odometry Status: " + (odometry != null ? odometry.getDeviceStatus() : "NOT INITIALIZED"));
        telemetry.addLine("");
        telemetry.addLine("Press START to begin autonomous sequence");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();

        // Record initial heading
        if (odometry != null) {
            odometry.update();
            initialHeading = odometry.getHeading(AngleUnit.DEGREES);
        }

        // Run the autonomous sequence with continuous loop
        while (opModeIsActive() && !emergencyStop) {
            runAutonomousSequence();

            // Break after sequence completes once (unless you want it to loop)
            break;
        }

        // Stop all systems
        stopRobot();

        if (emergencyStop) {
            telemetry.addLine("⚠️ EMERGENCY STOP ACTIVATED!");
        } else {
            telemetry.addLine("✅ Autonomous Complete!");
        }
        telemetry.update();
    }

    /**
     * Initialize the Pinpoint Odometry Computer
     */
    private void initializeOdometry() {
        try {
            telemetry.addLine("Attempting to get odometry device 'odo'...");
            telemetry.update();

            odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

            telemetry.addLine("Device found! Configuring...");
            telemetry.update();

            // Set odometry pod offsets (adjust these for your robot configuration)
            // X offset: how far sideways from center is the X (forward) pod (left is positive)
            // Y offset: how far forward from center is the Y (strafe) pod (forward is positive)
            odometry.setOffsets(-154, 0, DistanceUnit.MM); // Example values from SensorGoBildaPinpointExample

            // Set encoder resolution (use goBILDA pods or custom resolution)
            odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            // Set encoder directions
            // X (forward) pod should increase when robot moves forward
            // Y (strafe) pod should increase when robot moves left
            odometry.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            // Reset position and recalibrate IMU
            telemetry.addLine("Resetting position and calibrating IMU...");
            telemetry.update();
            odometry.resetPosAndIMU();

            telemetry.addLine("✅ Odometry initialized successfully");
            telemetry.addData("Device Version", odometry.getDeviceVersion());
            telemetry.addData("Device Status", odometry.getDeviceStatus());
            telemetry.addData("X Offset (mm)", odometry.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y Offset (mm)", odometry.getYOffset(DistanceUnit.MM));
            telemetry.addData("Yaw Scalar", odometry.getYawScalar());

        } catch (IllegalArgumentException e) {
            odometry = null;
            telemetry.addLine("⚠️ Odometry device 'odo' not found in hardware configuration!");
            telemetry.addLine("");
            telemetry.addLine("To fix this:");
            telemetry.addLine("1. Go to Robot Configuration on the Driver Station");
            telemetry.addLine("2. Add an I2C device named 'odo'");
            telemetry.addLine("3. Set device type to 'goBILDA Pinpoint'");
            telemetry.addLine("");
            telemetry.addLine("Error: " + e.getMessage());
        } catch (Exception e) {
            odometry = null;
            telemetry.addLine("⚠️ Odometry initialization failed!");
            telemetry.addLine("Error type: " + e.getClass().getSimpleName());
            telemetry.addLine("Error: " + e.getMessage());
            telemetry.addLine("");
            telemetry.addLine("Autonomous will run without odometry feedback");
        }
        telemetry.update();
    }

    /**
     * Main autonomous sequence
     */
    private void runAutonomousSequence() {
        telemetry.addLine("Starting Autonomous Sequence...");
        telemetry.update();

        // Start shooter at FULL POWER while moving to save time
        startShooter(ShooterConfig.ShooterPreset.SHORT_RANGE);

        // Move back from goal zone (odometry-based with timeout fallback)
        moveBackward(28, 1.3);
        sleep(300);

        // Fire 3 shots - shooter should already be at speed
        telemetry.addLine("Scoring...");
        telemetry.update();
        fireMultipleShots(3, ShooterConfig.ShooterPreset.SHORT_RANGE, false, 10.0);
        sleep(500);

        // Move out of launch zone
        moveRight(12, 1.6);
        sleep(300);

        telemetry.addLine("✅ Sequence Complete!");
        telemetry.update();
    }

    // ========================================================================================
    // MOVEMENT FUNCTIONS - Use these to build your autonomous sequence
    // ========================================================================================

    /**
     * Move left (strafe) relative to robot's current orientation
     * @param inches Distance to move in inches
     */
    public void moveLeft(double inches) {
        moveLeft(inches, MAX_MOVE_TIME);
    }

    /**
     * Move left (strafe) relative to robot's current orientation with custom timeout
     * @param inches Distance to move in inches
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    public void moveLeft(double inches, double timeoutSeconds) {
        moveRobotRelative(inches, 0, getCurrentHeading(), timeoutSeconds);
    }

    /**
     * Move right (strafe) relative to robot's current orientation
     * @param inches Distance to move in inches
     */
    public void moveRight(double inches) {
        moveRight(inches, MAX_MOVE_TIME);
    }

    /**
     * Move right (strafe) relative to robot's current orientation with custom timeout
     * @param inches Distance to move in inches
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    public void moveRight(double inches, double timeoutSeconds) {
        moveRobotRelative(-inches, 0, getCurrentHeading(), timeoutSeconds);
    }

    /**
     * Move forward relative to robot's current orientation
     * @param inches Distance to move in inches
     */
    public void moveForward(double inches) {
        moveForward(inches, MAX_MOVE_TIME);
    }

    /**
     * Move forward relative to robot's current orientation with custom timeout
     * @param inches Distance to move in inches
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    public void moveForward(double inches, double timeoutSeconds) {
        moveRobotRelative(0, inches, getCurrentHeading(), timeoutSeconds);
    }

    /**
     * Move backward relative to robot's current orientation
     * @param inches Distance to move in inches
     */
    public void moveBackward(double inches) {
        moveBackward(inches, MAX_MOVE_TIME);
    }

    /**
     * Move backward relative to robot's current orientation with custom timeout
     * @param inches Distance to move in inches
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    public void moveBackward(double inches, double timeoutSeconds) {
        moveRobotRelative(0, -inches, getCurrentHeading(), timeoutSeconds);
    }

    /**
     * Turn to an absolute field heading
     * @param targetDegrees Target heading in degrees (0-360)
     */
    public void turnToAngle(double targetDegrees) {
        turnToAngle(targetDegrees, MAX_MOVE_TIME);
    }

    /**
     * Turn to an absolute field heading with custom timeout
     * @param targetDegrees Target heading in degrees (0-360)
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    public void turnToAngle(double targetDegrees, double timeoutSeconds) {
        if (odometry == null || driveSystem == null || !opModeIsActive() || emergencyStop) return;

        telemetry.addData("Action", "Turning to " + targetDegrees + "°");
        telemetry.update();

        resetPID();
        moveTimer.reset();

        // Calculate total angle to turn
        odometry.update();
        double startHeading = getCurrentHeading();
        double totalAngle = Math.abs(normalizeAngle(targetDegrees - startHeading));

        while (opModeIsActive() && !emergencyStop) {
            // TIMEOUT CHECK FIRST - This overrides everything else
            if (moveTimer.seconds() >= timeoutSeconds) {
                stopRobot(); // FORCE STOP motors immediately
                telemetry.addData("Status", "⚠️ Turn timeout!");
                telemetry.update();
                sleep(100); // Brief pause to ensure motors stop
                break;
            }

            odometry.update();

            double currentHeading = getCurrentHeading();
            double headingError = normalizeAngle(targetDegrees - currentHeading);

            // Check if we've reached the target
            if (Math.abs(headingError) < HEADING_TOLERANCE) {
                telemetry.addData("Status", "✅ Heading reached!");
                telemetry.update();
                break;
            }

            // Calculate rotation power using PID
            double rotationPower = calculateHeadingPID(headingError);

            // Apply smooth motion profile
            double smoothMultiplier = calculateSmoothRotationProfile(Math.abs(headingError), totalAngle);
            rotationPower *= smoothMultiplier;

            // Apply power to motors
            driveSystem.setDriveInputs(0, 0, rotationPower);
            driveSystem.update();

            // Telemetry
            telemetry.addData("Current Heading", "%.1f°", currentHeading);
            telemetry.addData("Target Heading", "%.1f°", targetDegrees);
            telemetry.addData("Error", "%.1f°", headingError);
            telemetry.addData("Timer", "%.2f / %.2f sec", moveTimer.seconds(), timeoutSeconds);
            telemetry.addData("Base Power", "%.2f", rotationPower / smoothMultiplier);
            telemetry.addData("Smooth Multiplier", "%.2f", smoothMultiplier);
            telemetry.addData("Final Power", "%.2f", rotationPower);
            telemetry.update();

            // Small delay to prevent loop overrun
            sleep(10);
        }

        stopRobot();
    }

    /**
     * Turn relative to current heading
     * @param degrees Degrees to turn (positive = counterclockwise, negative = clockwise)
     */
    public void turnRelative(double degrees) {
        turnRelative(degrees, MAX_MOVE_TIME);
    }

    /**
     * Turn relative to current heading with custom timeout
     * @param degrees Degrees to turn (positive = counterclockwise, negative = clockwise)
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    public void turnRelative(double degrees, double timeoutSeconds) {
        double currentHeading = getCurrentHeading();
        double targetHeading = normalizeAngle(currentHeading + degrees);
        turnToAngle(targetHeading, timeoutSeconds);
    }

    /**
     * Move to an absolute field position
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @param targetHeading Target heading in degrees
     */
    public void moveToPosition(double targetX, double targetY, double targetHeading) {
        moveToPosition(targetX, targetY, targetHeading, MAX_MOVE_TIME);
    }

    /**
     * Move to an absolute field position with custom timeout
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @param targetHeading Target heading in degrees
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    public void moveToPosition(double targetX, double targetY, double targetHeading, double timeoutSeconds) {
        if (odometry == null || driveSystem == null || !opModeIsActive() || emergencyStop) return;

        telemetry.addData("Action", String.format("Moving to (%.1f, %.1f) @ %.1f°", targetX, targetY, targetHeading));
        telemetry.update();

        resetPID();
        moveTimer.reset();
        ElapsedTime settlingTimer = new ElapsedTime();
        int stabilityCounter = 0;

        // Calculate total distance for motion profiling
        odometry.update();
        Pose2D startPos = odometry.getPosition();
        double startX = startPos.getX(DistanceUnit.INCH);
        double startY = startPos.getY(DistanceUnit.INCH);
        double startHeading = getCurrentHeading();
        double totalDistance = Math.sqrt(Math.pow(targetX - startX, 2) + Math.pow(targetY - startY, 2));
        double totalAngle = Math.abs(normalizeAngle(targetHeading - startHeading));

        while (opModeIsActive() && !emergencyStop) {
            // TIMEOUT CHECK FIRST - This overrides everything else
            if (moveTimer.seconds() >= timeoutSeconds) {
                stopRobot(); // FORCE STOP motors immediately
                telemetry.addData("Status", "⚠️ Movement timeout!");
                telemetry.update();
                sleep(100); // Brief pause to ensure motors stop
                break;
            }

            odometry.update();
            Pose2D currentPos = odometry.getPosition();

            double currentX = currentPos.getX(DistanceUnit.INCH);
            double currentY = currentPos.getY(DistanceUnit.INCH);
            double currentHeading = getCurrentHeading();

            // Calculate errors
            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;
            double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            double headingError = normalizeAngle(targetHeading - currentHeading);

            // Check if we're within tolerance
            boolean atTarget = distance < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE;

            if (atTarget) {
                stabilityCounter++;
                // Require multiple consecutive checks at target to confirm stability
                if (stabilityCounter >= STABILITY_CHECKS) {
                    // Reset PID integrals when settled to prevent drift
                    positionErrorIntegral = 0;
                    headingErrorIntegral = 0;
                    telemetry.addData("Status", "✅ Target reached!");
                    telemetry.update();
                    break;
                }
            } else {
                stabilityCounter = 0;
                settlingTimer.reset();
            }


            // Convert field-relative errors to robot-relative
            double angleToTarget = Math.atan2(deltaY, deltaX);
            double headingRad = Math.toRadians(currentHeading);
            double robotRelativeAngle = angleToTarget - headingRad;

            // Calculate motor powers using PID
            double positionPower = calculatePositionPID(distance);

            // Apply smooth motion profile to position
            double smoothMultiplier = calculateSmoothMotionProfile(distance, totalDistance);
            positionPower *= smoothMultiplier;

            double forwardPower = positionPower * Math.cos(robotRelativeAngle);
            double strafePower = positionPower * Math.sin(robotRelativeAngle);

            // Calculate rotation power with smooth profile
            double rotationPower = calculateHeadingPID(headingError);
            double rotationMultiplier = calculateSmoothRotationProfile(Math.abs(headingError), totalAngle);
            rotationPower *= rotationMultiplier;

            // Apply powers
            driveSystem.setDriveInputs(forwardPower, strafePower, rotationPower);
            driveSystem.update();

            // Telemetry
            telemetry.addData("Current", "X: %.1f, Y: %.1f, H: %.1f°", currentX, currentY, currentHeading);
            telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.1f°", targetX, targetY, targetHeading);
            telemetry.addData("Distance", "%.1f in", distance);
            telemetry.addData("Heading Error", "%.1f°", headingError);
            telemetry.addData("Stability", "%d/%d", stabilityCounter, STABILITY_CHECKS);
            telemetry.addData("Timer", "%.2f / %.2f sec", moveTimer.seconds(), timeoutSeconds);
            telemetry.addData("Smooth Mult", "%.2f", smoothMultiplier);
            telemetry.addData("Powers", "F: %.2f, S: %.2f, R: %.2f", forwardPower, strafePower, rotationPower);
            telemetry.update();

            // Small delay to prevent loop overrun
            sleep(10);
        }

        stopRobot();
    }

    /**
     * Strafe to a field position while maintaining current heading
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     */
    public void strafeToPosition(double targetX, double targetY) {
        double currentHeading = getCurrentHeading();
        moveToPosition(targetX, targetY, currentHeading);
    }

    /**
     * Strafe to a field position while maintaining current heading with custom timeout
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    public void strafeToPosition(double targetX, double targetY, double timeoutSeconds) {
        double currentHeading = getCurrentHeading();
        moveToPosition(targetX, targetY, currentHeading, timeoutSeconds);
    }

    // ========================================================================================
    // SHOOTER FUNCTIONS - Use these to control the shooter during autonomous
    // ========================================================================================

    /**
     * Start the shooter at a specific preset
     * @param preset The shooting preset to use (LONG_RANGE, SHORT_RANGE, RAPID_FIRE, PRECISION, BATTERY_SAVER)
     */
    public void startShooter(ShooterConfig.ShooterPreset preset) {
        if (robotManager == null || robotManager.getShooterSystem() == null || emergencyStop) return;

        telemetry.addData("Shooter", "Starting - " + preset.getName());
        telemetry.update();

        robotManager.getShooterSystem().getConfig().setPreset(preset);
        robotManager.getShooterSystem().startShooter();
    }

    /**
     * Start the shooter at long range preset (default)
     */
    public void startShooter() {
        startShooter(ShooterConfig.ShooterPreset.LONG_RANGE);
    }

    /**
     * Stop the shooter
     */
    public void stopShooter() {
        if (robotManager == null || robotManager.getShooterSystem() == null) return;

        telemetry.addData("Shooter", "Stopping");
        telemetry.update();

        robotManager.getShooterSystem().stopShooter();
    }

    /**
     * Fire a single shot
     * Waits for shooter to be ready, then fires one shot
     *
     * @param preset The shooting preset to use
     * @return true if shot was fired successfully
     */
    public boolean fireSingleShot(ShooterConfig.ShooterPreset preset) {
        if (robotManager == null || robotManager.getShooterSystem() == null || emergencyStop) return false;

        telemetry.addData("Shooter", "Firing single shot - " + preset.getName());
        telemetry.update();

        // Set preset and start shooter if not running
        robotManager.getShooterSystem().getConfig().setPreset(preset);
        if (!robotManager.getShooterSystem().isShooterRunning()) {
            robotManager.getShooterSystem().startShooter();
        }

        // Wait for shooter to be ready
        ElapsedTime waitTimer = new ElapsedTime();
        while (opModeIsActive() && !emergencyStop && waitTimer.seconds() < 3.0) {
            if (robotManager.getShooterSystem().isShooterReady()) {
                break;
            }
            sleep(20);
        }

        // Fire the shot
        if (robotManager.getShooterSystem().isShooterReady()) {
            boolean shotFired = robotManager.getShooterSystem().fireSingleShot();

            // Wait for shot to complete
            ElapsedTime shotTimer = new ElapsedTime();
            while (opModeIsActive() && !emergencyStop && shotTimer.seconds() < 1.0) {
                if (!robotManager.getShooterSystem().isShooting()) {
                    break;
                }
                sleep(20);
            }

            return shotFired;
        }

        return false;
    }

    /**
     * Fire a single shot with long range preset
     */
    public boolean fireSingleShot() {
        return fireSingleShot(ShooterConfig.ShooterPreset.LONG_RANGE);
    }

    /**
     * Fire multiple shots in rapid succession
     *
     * @param numShots Number of shots to fire
     * @param preset The shooting preset to use
     * @param keepRunning If true, keeps shooter running after shots complete
     */
    public void fireMultipleShots(int numShots, ShooterConfig.ShooterPreset preset, boolean keepRunning) {
        fireMultipleShots(numShots, preset, keepRunning, 10.0); // Default 10 second timeout
    }

    /**
     * Fire multiple shots in rapid succession with timeout
     *
     * @param numShots Number of shots to fire
     * @param preset The shooting preset to use
     * @param keepRunning If true, keeps shooter running after shots complete
     * @param timeoutSeconds Maximum time allowed for shooting sequence
     */
    public void fireMultipleShots(int numShots, ShooterConfig.ShooterPreset preset, boolean keepRunning, double timeoutSeconds) {
        if (robotManager == null || robotManager.getShooterSystem() == null || emergencyStop) return;

        // STOP DRIVE MOTORS IMMEDIATELY - keep them stopped during shooting
        stopRobot();

        telemetry.addData("Shooter", "Firing %d shots - %s", numShots, preset.getName());
        telemetry.update();

        ElapsedTime shootTimer = new ElapsedTime();

        // Set preset and start shooter
        robotManager.getShooterSystem().getConfig().setPreset(preset);
        if (!robotManager.getShooterSystem().isShooterRunning()) {
            robotManager.getShooterSystem().startShooter();
        }

        int shotsFired = 0;
        double shotInterval = robotManager.getShooterSystem().getConfig().getShotInterval();

        // Fire shots with timeout protection
        ElapsedTime shotAttemptTimer = new ElapsedTime();
        double maxTimePerShot = 5.0; // Max 5 seconds per shot attempt (increased to allow recovery)

        while (opModeIsActive() && !emergencyStop && shotsFired < numShots && shootTimer.seconds() < timeoutSeconds) {
            shotAttemptTimer.reset();

            telemetry.addData("🎯 Preparing Shot", String.format("%d/%d", shotsFired + 1, numShots));
            telemetry.update();

            // Wait for shooter to be ready (with per-shot timeout)
            while (opModeIsActive() && !robotManager.getShooterSystem().isShooterReady() && shotAttemptTimer.seconds() < maxTimePerShot) {
                stopRobot(); // Keep drive motors stopped
                telemetry.addData("🔄 Status", "Waiting for ready...");
                telemetry.addData("RPM", "%.0f", robotManager.getShooterSystem().getCurrentRPM());
                telemetry.addData("Ready?", robotManager.getShooterSystem().isShooterReady() ? "YES" : "NO");
                telemetry.addData("Shots", "%d/%d", shotsFired, numShots);
                telemetry.addData("Total Timer", "%.1f / %.1f sec", shootTimer.seconds(), timeoutSeconds);
                telemetry.addData("Shot Wait", "%.1f / %.1f sec", shotAttemptTimer.seconds(), maxTimePerShot);
                telemetry.update();
                sleep(100);
            }

            // Check if we timed out waiting for ready
            if (!robotManager.getShooterSystem().isShooterReady()) {
                telemetry.addData("⚠️ Shot " + (shotsFired + 1), "Timed out waiting for ready");
                telemetry.addData("Final RPM", "%.0f", robotManager.getShooterSystem().getCurrentRPM());
                telemetry.addData("Was Ready?", "NO");
                telemetry.update();
                sleep(1000); // Give time to read the error
                break;
            }

            telemetry.addData("✅ Shooter Ready", "Firing shot " + (shotsFired + 1) + "...");
            telemetry.update();

            // Fire the shot
            if (robotManager.getShooterSystem().fireSingleShot()) {
                shotsFired++;
                telemetry.addData("🔥 Shot " + shotsFired + "/" + numShots, "STARTED");
                telemetry.update();

                // Wait for the shot to complete (feed servos to finish)
                // CRITICAL: Must keep calling fireSingleShot() to update the shooter state!
                ElapsedTime shotCompleteTimer = new ElapsedTime();
                while (opModeIsActive() && shotCompleteTimer.seconds() < 1.0) {
                    // Keep updating the shooter - this is what actually stops the servos!
                    robotManager.getShooterSystem().fireSingleShot();
                    stopRobot(); // Keep drive motors stopped

                    if (!robotManager.getShooterSystem().isShooting()) {
                        break;
                    }

                    telemetry.addData("🔥 Shot " + shotsFired + "/" + numShots, String.format("Feeding... %.1fs", shotCompleteTimer.seconds()));
                    telemetry.update();
                    sleep(20);
                }

                telemetry.addData("✅ Shot " + shotsFired + "/" + numShots, "COMPLETE");
                telemetry.update();
                sleep(200); // Brief pause to read status

                // Wait for shot interval before next shot (only if more shots remaining)
                if (shotsFired < numShots) {
                    telemetry.addData("⏳ Waiting", String.format("Next shot in %.1fs...", shotInterval));
                    telemetry.update();

                    ElapsedTime intervalTimer = new ElapsedTime();
                    while (opModeIsActive() && intervalTimer.seconds() < shotInterval) {
                        stopRobot(); // Keep drive motors stopped
                        telemetry.addData("⏳ Interval Wait", String.format("%.1f / %.1f sec", intervalTimer.seconds(), shotInterval));
                        telemetry.addData("Shots Complete", String.format("%d/%d", shotsFired, numShots));
                        telemetry.addData("RPM", "%.0f", robotManager.getShooterSystem().getCurrentRPM());
                        telemetry.update();
                        sleep(100);
                    }

                    telemetry.addData("✅ Interval Complete", "Ready for shot " + (shotsFired + 1));
                    telemetry.update();
                }
            } else {
                // Failed to fire, wait a bit and try again
                telemetry.addData("⚠️ Failed", "Shot " + (shotsFired + 1) + " did not fire, retrying...");
                telemetry.addData("Shooter Ready?", robotManager.getShooterSystem().isShooterReady() ? "YES" : "NO");
                telemetry.addData("RPM", "%.0f", robotManager.getShooterSystem().getCurrentRPM());
                telemetry.update();
                sleep(500);
            }
        }

        // FORCE STOP shooter immediately
        if (!keepRunning) {
            robotManager.getShooterSystem().stopShooter();
        }

        // Stop drive motors in case they were moving during shooting
        stopRobot();

        if (shootTimer.seconds() >= timeoutSeconds) {
            telemetry.addData("Shooter", "⚠️ Timeout! Fired %d/%d shots", shotsFired, numShots);
        } else {
            telemetry.addData("Shooter", "✅ Complete! Fired %d shots", shotsFired);
        }
        telemetry.update();
        sleep(300);
    }

    /**
     * Fire multiple shots with long range preset
     *
     * @param numShots Number of shots to fire
     * @param keepRunning If true, keeps shooter running after shots complete
     */
    public void fireMultipleShots(int numShots, boolean keepRunning) {
        fireMultipleShots(numShots, ShooterConfig.ShooterPreset.LONG_RANGE, keepRunning);
    }

    /**
     * Fire multiple shots and stop shooter when done
     *
     * @param numShots Number of shots to fire
     */
    public void fireMultipleShots(int numShots) {
        fireMultipleShots(numShots, ShooterConfig.ShooterPreset.LONG_RANGE, false);
    }

    /**
     * Wait for shooter to reach target RPM
     * Useful when you want to pre-spin the shooter before moving
     *
     * @param maxWaitTime Maximum time to wait in seconds
     * @return true if shooter is ready, false if timeout
     */
    public boolean waitForShooterReady(double maxWaitTime) {
        if (robotManager == null || robotManager.getShooterSystem() == null || emergencyStop) return false;

        ElapsedTime waitTimer = new ElapsedTime();

        while (opModeIsActive() && !emergencyStop && waitTimer.seconds() < maxWaitTime) {
            if (robotManager.getShooterSystem().isShooterReady()) {
                telemetry.addData("Shooter", "Ready!");
                telemetry.update();
                return true;
            }

            telemetry.addData("Shooter", "Spinning up... %.0f RPM", robotManager.getShooterSystem().getCurrentRPM());
            telemetry.update();

            sleep(50);
        }

        telemetry.addData("Shooter", "Timeout waiting for ready");
        telemetry.update();
        return false;
    }

    /**
     * Start shooter in warmup mode (lower RPM to save battery)
     * Useful at the start of autonomous to have shooter ready quickly
     *
     * @param preset The shooting preset (warmup will run at 65% of target RPM)
     */
    public void startShooterWarmup(ShooterConfig.ShooterPreset preset) {
        if (robotManager == null || robotManager.getShooterSystem() == null || emergencyStop) return;

        telemetry.addData("Shooter", "Starting warmup - " + preset.getName());
        telemetry.update();

        robotManager.getShooterSystem().getConfig().setPreset(preset);
        // Use the handleWarmupButton method with a simulated button press
        robotManager.getShooterSystem().handleWarmupButton(true, preset);
    }

    /**
     * Start shooter in warmup mode with long range preset
     */
    public void startShooterWarmup() {
        startShooterWarmup(ShooterConfig.ShooterPreset.LONG_RANGE);
    }

    /**
     * Check if shooter is currently running
     *
     * @return true if shooter is spinning
     */
    public boolean isShooterRunning() {
        if (robotManager == null || robotManager.getShooterSystem() == null) return false;
        return robotManager.getShooterSystem().isShooterRunning();
    }

    /**
     * Check if shooter is ready to fire
     *
     * @return true if shooter has reached target RPM and is stable
     */
    public boolean isShooterReady() {
        if (robotManager == null || robotManager.getShooterSystem() == null) return false;
        return robotManager.getShooterSystem().isShooterReady();
    }

    /**
     * Get current shooter RPM
     *
     * @return Current RPM of the shooter
     */
    public double getShooterRPM() {
        if (robotManager == null || robotManager.getShooterSystem() == null) return 0;
        return robotManager.getShooterSystem().getCurrentRPM();
    }

    // ========================================================================================
    // HELPER FUNCTIONS - Internal movement control
    // ========================================================================================

    /**
     * Move robot relative to its current position
     * @param strafeInches Strafe distance (positive = left, negative = right)
     * @param forwardInches Forward distance (positive = forward, negative = backward)
     * @param maintainHeading Heading to maintain during movement
     * @param timeoutSeconds Maximum time allowed for this movement
     */
    private void moveRobotRelative(double strafeInches, double forwardInches, double maintainHeading, double timeoutSeconds) {
        if (odometry == null || driveSystem == null || !opModeIsActive() || emergencyStop) return;

        // Get starting position
        odometry.update();
        startPosition = odometry.getPosition();

        // Calculate target position in field coordinates
        double startX = startPosition.getX(DistanceUnit.INCH);
        double startY = startPosition.getY(DistanceUnit.INCH);
        double headingRad = Math.toRadians(maintainHeading);

        // Transform robot-relative movement to field-relative
        double fieldDeltaX = forwardInches * Math.cos(headingRad) - strafeInches * Math.sin(headingRad);
        double fieldDeltaY = forwardInches * Math.sin(headingRad) + strafeInches * Math.cos(headingRad);

        double targetX = startX + fieldDeltaX;
        double targetY = startY + fieldDeltaY;

        // Move to the calculated position
        moveToPosition(targetX, targetY, maintainHeading, timeoutSeconds);
    }

    /**
     * Calculate PID output for position control
     * @param error Current position error in inches
     * @return Motor power output
     */
    private double calculatePositionPID(double error) {
        // Proportional term
        double pTerm = KP_POSITION * error;

        // Integral term (with anti-windup and dead zone)
        // Don't accumulate integral when very close to target to prevent windup
        if (Math.abs(error) > POSITION_TOLERANCE * 0.5) {
            positionErrorIntegral += error;
            if (Math.abs(positionErrorIntegral) > 50) {
                positionErrorIntegral = Math.signum(positionErrorIntegral) * 50;
            }
        } else {
            // Decay integral when close to target
            positionErrorIntegral *= 0.8;
        }
        double iTerm = KI_POSITION * positionErrorIntegral;

        // Derivative term
        double dTerm = KD_POSITION * (error - lastPositionError);
        lastPositionError = error;

        // Calculate total output
        double output = pTerm + iTerm + dTerm;

        // Clamp output to valid range
        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

        // Apply minimum power threshold, but reduce it when very close to target
        double minPowerThreshold = MIN_POWER;
        if (Math.abs(error) < POSITION_TOLERANCE * 2) {
            // Scale down minimum power when close to target
            minPowerThreshold = MIN_POWER * 0.5;
        }

        if (Math.abs(output) > 0 && Math.abs(output) < minPowerThreshold) {
            output = Math.signum(output) * minPowerThreshold;
        }

        return output;
    }

    /**
     * Calculate smooth motion profile multiplier based on distance traveled and remaining
     * This creates a trapezoidal velocity profile: accelerate -> cruise -> decelerate
     *
     * @param distanceRemaining Distance remaining to target (inches)
     * @param totalDistance Total distance of the movement (inches)
     * @return Power multiplier between 0.0 and 1.0
     */
    private double calculateSmoothMotionProfile(double distanceRemaining, double totalDistance) {
        if (!USE_SMOOTH_MOTION) {
            return 1.0; // No smoothing, use full PID output
        }

        double distanceTraveled = totalDistance - distanceRemaining;
        double multiplier = 1.0;

        // Acceleration phase: ramp up from MIN_POWER to MAX_POWER
        if (distanceTraveled < ACCELERATION_DISTANCE) {
            // Smooth acceleration curve (ease-in)
            double progress = distanceTraveled / ACCELERATION_DISTANCE;
            // Use smooth curve (quadratic ease-in)
            multiplier = MIN_POWER / MAX_POWER + (1.0 - MIN_POWER / MAX_POWER) * (progress * progress);
        }
        // Deceleration phase: ramp down from MAX_POWER to MIN_POWER
        else if (distanceRemaining < DECELERATION_DISTANCE) {
            // Smooth deceleration curve (ease-out)
            double progress = distanceRemaining / DECELERATION_DISTANCE;
            // Use smooth curve (quadratic ease-out)
            multiplier = MIN_POWER / MAX_POWER + (1.0 - MIN_POWER / MAX_POWER) * (progress * progress);

            // Never go below minimum viable multiplier
            multiplier = Math.max(multiplier, MIN_POWER / MAX_POWER);
        }
        // Cruise phase: full speed
        else {
            multiplier = 1.0;
        }

        return multiplier;
    }

    /**
     * Calculate smooth motion profile for rotation
     *
     * @param angleRemaining Angle remaining to target (degrees)
     * @param totalAngle Total angle of the rotation (degrees)
     * @return Power multiplier between 0.0 and 1.0
     */
    private double calculateSmoothRotationProfile(double angleRemaining, double totalAngle) {
        if (!USE_SMOOTH_MOTION) {
            return 1.0; // No smoothing
        }

        double angleTraveled = totalAngle - angleRemaining;
        double multiplier = 1.0;

        // Acceleration phase
        if (angleTraveled < ROTATION_ACCELERATION) {
            double progress = angleTraveled / ROTATION_ACCELERATION;
            multiplier = MIN_POWER / MAX_POWER + (1.0 - MIN_POWER / MAX_POWER) * (progress * progress);
        }
        // Deceleration phase
        else if (angleRemaining < ROTATION_DECELERATION) {
            double progress = angleRemaining / ROTATION_DECELERATION;
            multiplier = MIN_POWER / MAX_POWER + (1.0 - MIN_POWER / MAX_POWER) * (progress * progress);
            multiplier = Math.max(multiplier, MIN_POWER / MAX_POWER);
        }
        // Cruise phase
        else {
            multiplier = 1.0;
        }

        return multiplier;
    }

    /**
     * Calculate PID output for heading control
     * @param error Current heading error in degrees
     * @return Rotation power output
     */
    private double calculateHeadingPID(double error) {
        // Proportional term
        double pTerm = KP_HEADING * error;

        // Integral term (with anti-windup and dead zone)
        // Don't accumulate integral when very close to target
        if (Math.abs(error) > HEADING_TOLERANCE * 0.5) {
            headingErrorIntegral += error;
            if (Math.abs(headingErrorIntegral) > 100) {
                headingErrorIntegral = Math.signum(headingErrorIntegral) * 100;
            }
        } else {
            // Decay integral when close to target
            headingErrorIntegral *= 0.8;
        }
        double iTerm = KI_HEADING * headingErrorIntegral;

        // Derivative term
        double dTerm = KD_HEADING * (error - lastHeadingError);
        lastHeadingError = error;

        // Calculate total output
        double output = pTerm + iTerm + dTerm;

        // Clamp output to valid range
        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

        // Apply minimum power threshold, reduced when close to target
        double minPowerThreshold = MIN_POWER;
        if (Math.abs(error) < HEADING_TOLERANCE * 2) {
            minPowerThreshold = MIN_POWER * 0.5;
        }

        if (Math.abs(output) > 0 && Math.abs(output) < minPowerThreshold) {
            output = Math.signum(output) * minPowerThreshold;
        }

        return output;
    }

    /**
     * Reset PID controllers
     */
    private void resetPID() {
        positionErrorIntegral = 0;
        lastPositionError = 0;
        headingErrorIntegral = 0;
        lastHeadingError = 0;
    }

    /**
     * Get current robot heading with odometry + IMU fusion
     * @return Current heading in degrees
     */
    private double getCurrentHeading() {
        if (odometry != null) {
            return odometry.getHeading(AngleUnit.DEGREES);
        }
        return 0;
    }

    /**
     * Normalize angle to -180 to 180 range
     * @param angle Angle in degrees
     * @return Normalized angle
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Stop all robot movement
     */
    private void stopRobot() {
        if (driveSystem != null) {
            driveSystem.setDriveInputs(0, 0, 0);
            driveSystem.update();
        }
    }

    /**
     * Get current robot position
     * @return Current position as Pose2D
     */
    public Pose2D getCurrentPosition() {
        if (odometry != null) {
            odometry.update();
            return odometry.getPosition();
        }
        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

    /**
     * Wait for a specified duration while maintaining position
     * @param seconds Time to wait in seconds
     */
    public void holdPosition(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        Pose2D holdPos = getCurrentPosition();
        double holdHeading = getCurrentHeading();

        while (opModeIsActive() && timer.seconds() < seconds) {
            moveToPosition(
                holdPos.getX(DistanceUnit.INCH),
                holdPos.getY(DistanceUnit.INCH),
                holdHeading
            );
        }
        stopRobot();
    }
}

