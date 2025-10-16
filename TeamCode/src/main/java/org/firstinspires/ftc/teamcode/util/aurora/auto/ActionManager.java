package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.PathPlanningSystem;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

// Used to manage and execute robot movements and actions during autonomous mode
// Contains methods for using the mechanum drive train and other actuators
//Works directly with AuroraPositioningManager for precise movement but only executes actions when called from the AuroraAutoManager
public class ActionManager {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private AuroraPositioningManager positionManager;

    // Drive train motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private boolean driveInitialized = false;

    // Actuator collections
    private Map<String, DcMotor> motors;
    private Map<String, Servo> servos;

    // Movement control
    private ElapsedTime moveTimer;
    private boolean isMoving = false;
    private double maxPower = 0.8;
    private double minPower = 0.15;

    // PID constants for movement
    private double kP_drive = 0.05;
    private double kI_drive = 0.0;
    private double kD_drive = 0.01;
    private double kP_turn = 0.03;
    private double kI_turn = 0.0;
    private double kD_turn = 0.005;

    // Path planning
    private PathPlanningSystem pathPlanner;
    private List<PathPlanningSystem.Point> currentPath;
    private int currentPathIndex = 0;

    /**
     * Initialize action manager
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param positionManager Position manager for feedback
     */
    public ActionManager(HardwareMap hardwareMap, Telemetry telemetry, AuroraPositioningManager positionManager) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.positionManager = positionManager;

        // Initialize collections
        motors = new HashMap<>();
        servos = new HashMap<>();
        moveTimer = new ElapsedTime();

        // Initialize path planner
        pathPlanner = new PathPlanningSystem();

        initializeDriveTrain();
        initializeActuators();

        telemetry.addData("ActionManager", "Initialized");
    }

    /**
     * Initialize mecanum drive train
     */
    private void initializeDriveTrain() {
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

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

            driveInitialized = true;
            telemetry.addData("ActionManager", "Drive train initialized");

        } catch (Exception e) {
            telemetry.addData("ActionManager Error", "Drive train not found: " + e.getMessage());
            driveInitialized = false;
        }
    }

    /**
     * Initialize additional actuators
     */
    private void initializeActuators() {
        // Try to initialize common motors
        String[] motorNames = {"arm", "intake", "shooter", "lift"};
        for (String name : motorNames) {
            try {
                DcMotor motor = hardwareMap.get(DcMotor.class, name);
                motors.put(name, motor);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("ActionManager", "Initialized motor: " + name);
            } catch (Exception e) {
                // Motor not found, continue
            }
        }

        // Try to initialize common servos
        String[] servoNames = {"servo1", "servo2"};
        for (String name : servoNames) {
            try {
                Servo servo = hardwareMap.get(Servo.class, name);
                servos.put(name, servo);
                telemetry.addData("ActionManager", "Initialized servo: " + name);
            } catch (Exception e) {
                // Servo not found, continue
            }
        }
    }

    /**
     * Drive robot with mechanum movement
     * @param drive Forward/backward power (-1 to 1)
     * @param strafe Left/right power (-1 to 1)
     * @param turn Rotation power (-1 to 1)
     */
    public void mechanumDrive(double drive, double strafe, double turn) {
        if (!driveInitialized) return;

        // Calculate wheel powers for mechanum drive
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        // Normalize powers to stay within -1 to 1
        double maxPower = Math.max(Math.abs(frontLeftPower),
                         Math.max(Math.abs(frontRightPower),
                         Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Move to target position using PID control with vision assistance
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     * @param targetHeading Target heading in degrees
     * @param timeout Maximum time for movement in seconds
     * @return True if movement completed successfully
     */
    public boolean moveToPosition(double targetX, double targetY, double targetHeading, double timeout) {
        if (!driveInitialized || !positionManager.isPositionValid()) return false;

        positionManager.setTarget(targetX, targetY, targetHeading);
        moveTimer.reset();
        isMoving = true;

        while (isMoving && moveTimer.seconds() < timeout) {
            // Non-blocking position update (includes vision corrections)
            positionManager.updatePosition();

            // Calculate errors
            double distanceToTarget = positionManager.getDistanceToTarget();
            double angleToTarget = positionManager.getAngleToTarget();
            double headingError = positionManager.getHeadingError();

            // Check if at target
            if (positionManager.isAtTarget(1.0, 5.0)) {
                stopDriving();
                isMoving = false;
                return true;
            }

            // Calculate drive powers using PID with field-relative movement
            double fieldRelativeAngle = Math.toRadians(angleToTarget);
            double robotRelativeAngle = fieldRelativeAngle - Math.toRadians(positionManager.getCurrentHeading());

            double drivePower = Math.cos(robotRelativeAngle) * distanceToTarget * kP_drive;
            double strafePower = Math.sin(robotRelativeAngle) * distanceToTarget * kP_drive;
            double turnPower = headingError * kP_turn;

            // Limit powers
            drivePower = Math.max(-maxPower, Math.min(maxPower, drivePower));
            strafePower = Math.max(-maxPower, Math.min(maxPower, strafePower));
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

            // Apply minimum power if moving
            if (Math.abs(drivePower) > 0 && Math.abs(drivePower) < minPower) {
                drivePower = Math.signum(drivePower) * minPower;
            }
            if (Math.abs(strafePower) > 0 && Math.abs(strafePower) < minPower) {
                strafePower = Math.signum(strafePower) * minPower;
            }

            mechanumDrive(drivePower, strafePower, turnPower);

            try {
                Thread.sleep(20); // 50Hz update rate
            } catch (InterruptedException e) {
                break;
            }
        }

        stopDriving();
        isMoving = false;
        return false; // Timeout reached
    }

    /**
     * Move to target with vision-assisted navigation (non-blocking variant)
     * Call this repeatedly until it returns true
     */
    public boolean moveToPositionNonBlocking(double targetX, double targetY, double targetHeading, double timeout) {
        if (!driveInitialized || !positionManager.isPositionValid()) return false;

        // Initialize movement on first call
        if (!isMoving) {
            positionManager.setTarget(targetX, targetY, targetHeading);
            moveTimer.reset();
            isMoving = true;
            return false; // Still moving
        }

        // Check timeout
        if (moveTimer.seconds() >= timeout) {
            stopDriving();
            isMoving = false;
            return true; // Complete (timeout)
        }

        // Non-blocking position update
        positionManager.updatePosition();

        // Calculate errors
        double distanceToTarget = positionManager.getDistanceToTarget();
        double angleToTarget = positionManager.getAngleToTarget();
        double headingError = positionManager.getHeadingError();

        // Check if at target
        if (positionManager.isAtTarget(1.0, 5.0)) {
            stopDriving();
            isMoving = false;
            return true; // Complete (success)
        }

        // Calculate and apply drive powers
        double fieldRelativeAngle = Math.toRadians(angleToTarget);
        double robotRelativeAngle = fieldRelativeAngle - Math.toRadians(positionManager.getCurrentHeading());

        double drivePower = Math.cos(robotRelativeAngle) * distanceToTarget * kP_drive;
        double strafePower = Math.sin(robotRelativeAngle) * distanceToTarget * kP_drive;
        double turnPower = headingError * kP_turn;

        // Limit and apply powers
        drivePower = Math.max(-maxPower, Math.min(maxPower, drivePower));
        strafePower = Math.max(-maxPower, Math.min(maxPower, strafePower));
        turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

        mechanumDrive(drivePower, strafePower, turnPower);

        return false; // Still moving
    }

    /**
     * Follow a path of waypoints
     * @param waypoints List of points to follow
     * @param timeout Maximum time for entire path in seconds
     * @return True if path completed successfully
     */
    public boolean followPath(List<PathPlanningSystem.Point> waypoints, double timeout) {
        if (waypoints == null || waypoints.isEmpty()) return false;

        moveTimer.reset();
        currentPath = waypoints;
        currentPathIndex = 0;

        while (currentPathIndex < waypoints.size() && moveTimer.seconds() < timeout) {
            PathPlanningSystem.Point target = waypoints.get(currentPathIndex);

            // Move to current waypoint
            if (moveToPosition(target.x, target.y, 0, 5.0)) {
                currentPathIndex++;
            } else {
                return false; // Failed to reach waypoint
            }
        }

        return currentPathIndex >= waypoints.size();
    }

    /**
     * Stop all drive motors
     */
    public void stopDriving() {
        if (driveInitialized) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        isMoving = false;
    }

    /**
     * Set motor power
     * @param motorName Name of motor
     * @param power Power level (-1 to 1)
     */
    public void setMotorPower(String motorName, double power) {
        DcMotor motor = motors.get(motorName);
        if (motor != null) {
            motor.setPower(power);
        }
    }

    /**
     * Set servo position
     * @param servoName Name of servo
     * @param position Position (0 to 1)
     */
    public void setServoPosition(String servoName, double position) {
        Servo servo = servos.get(servoName);
        if (servo != null) {
            servo.setPosition(Math.max(0, Math.min(1, position)));
        }
    }

    /**
     * Run motor to position using encoders
     * @param motorName Name of motor
     * @param targetPosition Target encoder position
     * @param power Motor power (0 to 1)
     * @param timeout Maximum time in seconds
     * @return True if position reached
     */
    public boolean runMotorToPosition(String motorName, int targetPosition, double power, double timeout) {
        DcMotor motor = motors.get(motorName);
        if (motor == null) return false;

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(power));

        ElapsedTime timer = new ElapsedTime();
        while (motor.isBusy() && timer.seconds() < timeout) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return !motor.isBusy();
    }

    /**
     * Perform complex action sequence
     * @param actionName Name of predefined action
     * @return True if action completed successfully
     */
    public boolean performAction(String actionName) {
        switch (actionName.toLowerCase()) {
            case "grab":
                return performGrabAction();
            case "release":
                return performReleaseAction();
            case "park":
                return performParkAction();
            default:
                telemetry.addData("ActionManager", "Unknown action: " + actionName);
                return false;
        }
    }

    /**
     * Perform grab action
     */
    private boolean performGrabAction() {
        // Close claw
        setServoPosition("claw", 0.0);

        // Wait for claw to close
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            return false;
        }

        // Lift arm slightly
        return runMotorToPosition("arm", 200, 0.5, 2.0);
    }

    /**
     * Perform release action
     */
    private boolean performReleaseAction() {
        // Open claw
        setServoPosition("claw", 1.0);

        // Wait for release
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            return false;
        }

        return true;
    }

    /**
     * Perform park action
     */
    private boolean performParkAction() {
        // Lower arm
        runMotorToPosition("arm", 0, 0.3, 3.0);

        // Retract any extensions
        setServoPosition("wrist", 0.5);

        return true;
    }

    /**
     * Set maximum drive power
     * @param maxPower Maximum power (0 to 1)
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.max(0.1, Math.min(1.0, maxPower));
    }

    /**
     * Set PID constants for drive
     * @param kP Proportional constant
     * @param kI Integral constant
     * @param kD Derivative constant
     */
    public void setDrivePID(double kP, double kI, double kD) {
        this.kP_drive = kP;
        this.kI_drive = kI;
        this.kD_drive = kD;
    }

    /**
     * Set PID constants for turning
     * @param kP Proportional constant
     * @param kI Integral constant
     * @param kD Derivative constant
     */
    public void setTurnPID(double kP, double kI, double kD) {
        this.kP_turn = kP;
        this.kI_turn = kI;
        this.kD_turn = kD;
    }

    /**
     * Check if robot is currently moving
     * @return True if movement is active
     */
    public boolean isMoving() {
        return isMoving;
    }

    /**
     * Check if vision system is available and working
     */
    public boolean isVisionAvailable() {
        return positionManager.isVisionAvailable();
    }

    /**
     * Get number of detected AprilTags
     */
    public int getDetectedTagCount() {
        return positionManager.getDetectedTagCount();
    }

    /**
     * Set camera exposure for better AprilTag detection
     */
    public boolean setManualExposure(int exposureMS, int gain) {
        return positionManager.setManualExposure(exposureMS, gain);
    }

    /**
     * Get current robot position from positioning system
     */
    public double[] getCurrentPosition() {
        return positionManager.getCurrentPosition();
    }

    /**
     * Add action telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== ACTION DATA ===", "");
        telemetry.addData("Drive Initialized", driveInitialized ? "YES" : "NO");
        telemetry.addData("Is Moving", isMoving ? "YES" : "NO");
        telemetry.addData("Max Power", "%.2f", maxPower);
        telemetry.addData("Motors Available", motors.size());
        telemetry.addData("Servos Available", servos.size());

        // Vision integration status
        if (positionManager != null) {
            telemetry.addData("Vision Available", isVisionAvailable() ? "YES" : "NO");
            if (isVisionAvailable()) {
                telemetry.addData("AprilTags Detected", getDetectedTagCount());
            }
        }

        // Show motor powers if driving
        if (driveInitialized) {
            telemetry.addData("Drive Powers", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                frontLeft.getPower(), frontRight.getPower(),
                backLeft.getPower(), backRight.getPower());
        }
    }

    /**
     * Close action manager and cleanup resources
     */
    public void close() {
        stopDriving();
        if (positionManager != null) {
            positionManager.close();
        }
    }
}
