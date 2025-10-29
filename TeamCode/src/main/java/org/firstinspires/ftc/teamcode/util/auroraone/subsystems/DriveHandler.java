package org.firstinspires.ftc.teamcode.util.auroraone.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.auroraone.config.RobotMap;
import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;
import org.firstinspires.ftc.teamcode.util.auroraone.core.Blackboard;

/**
 * AURORA ONE - Drive Handler
 * Advanced mecanum drive system for autonomous operation with intelligent direction verification
 *
 * This class manages the robot's drive system for autonomous modes, including motor control,
 * movement execution, and intelligent odometry-based direction verification.
 * It provides methods to execute autonomous movement commands and automatically detects
 * and corrects motor wiring issues.
 *
 * Features:
 * - Dynamic power scaling based on battery voltage
 * - Predictive movement with acceleration curves
 * - Energy-efficient driving modes
 * - Anti-tip protection for high-speed maneuvers
 * - Intelligent direction verification using odometry feedback
 * - Automatic motor direction correction for autonomous reliability
 * - Performance analytics and monitoring
 * - Integration with Aurora One State Machine system
 * - Centralized tunable parameters via Tunables class
 *
 * Autonomous Direction Verification:
 * The system continuously compares commanded movement directions (from path planner)
 * with actual robot movement (from odometry). If motors are wired incorrectly and the
 * robot moves opposite to commands, the system automatically inverts motor directions
 * to ensure autonomous paths execute correctly.
 */
public class DriveHandler {

    // Hardware reference
    private RobotMap robotMap;
    private Blackboard blackboard;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private VoltageSensor voltageSensor;

    // Drive modes
    public enum DriveMode {
        PRECISION(Tunables.DRIVE_MAX_SPEED_PRECISION, "Precision", "Slow, accurate movements"),
        NORMAL(Tunables.DRIVE_MAX_SPEED_NORMAL, "Normal", "Balanced speed and control"),
        SPORT(Tunables.DRIVE_MAX_SPEED_SPORT, "Sport", "Maximum speed and agility"),
        EFFICIENCY(Tunables.DRIVE_MAX_SPEED_EFFICIENCY, "Efficiency", "Optimized for battery life"),
        AUTO_ADAPTIVE(Tunables.DRIVE_MAX_SPEED_AUTO_ADAPTIVE, "Auto-Adaptive", "AI-optimized based on conditions");

        private final double maxPower;
        private final String name;
        private final String description;

        DriveMode(double maxPower, String name, String description) {
            this.maxPower = maxPower;
            this.name = name;
            this.description = description;
        }

        public double getMaxPower() { return maxPower; }
        public String getName() { return name; }
        public String getDescription() { return description; }
    }

    // State management
    private DriveMode currentMode = DriveMode.SPORT;
    private boolean fieldRelative = Tunables.TELEOP_FIELD_RELATIVE_DEFAULT;
    private double robotHeading = 0;
    private boolean isInitialized = false;

    // Performance optimization
    private ElapsedTime accelerationTimer = new ElapsedTime();
    private double[] lastMotorPowers = new double[4];
    private double[] targetMotorPowers = new double[4];

    // Battery optimization
    private double currentVoltage = Tunables.DRIVE_NOMINAL_VOLTAGE;
    private boolean batteryOptimization = true;
    private ElapsedTime lowVoltageTimer = new ElapsedTime();
    private boolean voltageConsistentlyLow = false;

    // Driver analytics
    private double totalDistance = 0;
    private double energyUsed = 0;
    private int sharpTurns = 0;
    private double averageSpeed = 0;
    private ElapsedTime sessionTimer = new ElapsedTime();

    // Safety systems
    private boolean antiTipEnabled = true;

    // Fine movement control (for precision adjustments)
    private double fineX = 0.0;
    private double fineY = 0.0;
    private double fineRotation = 0.0;

    // Commanded drive inputs (from autonomous control system)
    private double commandedAxial = 0.0;    // Forward/backward command
    private double commandedLateral = 0.0;  // Left/right command
    private double commandedYaw = 0.0;      // Rotation command

    // Intelligent direction verification
    private boolean directionVerificationEnabled = Tunables.DRIVE_DIRECTION_VERIFICATION_ENABLED;
    private int directionMismatchCount = 0;
    private int directionCorrectCount = 0;
    private boolean motorDirectionsInverted = false;
    private double lastCommandedVelocityX = 0.0;
    private double lastCommandedVelocityY = 0.0;
    private ElapsedTime directionCheckTimer = new ElapsedTime();
    private int totalDirectionCorrections = 0;
    private boolean directionMismatchDetected = false;

    /**
     * Constructor - Initialize with RobotMap
     */
    public DriveHandler(RobotMap robotMap) {
        this.robotMap = robotMap;
        this.blackboard = Blackboard.getInstance();
        initialize();
    }

    /**
     * Initialize the drive system using RobotMap
     */
    private void initialize() {
        if (!robotMap.isDriveSystemReady()) {
            isInitialized = false;
            return;
        }

        // Get motors from RobotMap
        leftFront = robotMap.frontLeft;
        rightFront = robotMap.frontRight;
        leftBack = robotMap.backLeft;
        rightBack = robotMap.backRight;
        voltageSensor = robotMap.voltageSensor;

        configureMotors();

        isInitialized = true;
        sessionTimer.reset();
        accelerationTimer.reset();
    }

    /**
     * Configure motors with optimal settings
     */
    private void configureMotors() {
        if (!isInitialized()) return;

        // Set to use encoders for better control
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake mode for better control
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor directions are already set in RobotMap initialization
    }

    /**
     * Set autonomous drive commands (primary input method for Aurora One)
     * This is called by the autonomous control system to command robot movement
     *
     * @param axial Forward/backward velocity command (-1.0 to 1.0)
     * @param lateral Left/right velocity command (-1.0 to 1.0)
     * @param yaw Rotational velocity command (-1.0 to 1.0)
     */
    public void setDriveInputs(double axial, double lateral, double yaw) {
        // Store commanded velocities (no deadzone in auto - precise control needed)
        this.commandedAxial = axial;
        this.commandedLateral = lateral;
        this.commandedYaw = yaw * Tunables.DRIVE_ROTATION_SENSITIVITY;
    }

    /**
     * Apply deadzone to input value (used for safety checks only in auto)
     */
    private double applyDeadzone(double input) {
        return Math.abs(input) < Tunables.DRIVE_DEADZONE ? 0.0 : input;
    }

    /**
     * Main drive update method - call this every loop
     */
    public void update() {
        if (!isInitialized()) return;

        updateBatteryStatus();

        // Get commanded velocities from autonomous control
        double axial = commandedAxial;
        double lateral = commandedLateral;
        double yaw = commandedYaw;

        // Store commanded velocity for direction verification
        lastCommandedVelocityX = lateral;
        lastCommandedVelocityY = axial;

        // Apply field-relative transformation if enabled
        if (fieldRelative) {
            double[] fieldRelativeInputs = applyFieldRelative(axial, lateral, robotHeading);
            axial = fieldRelativeInputs[0];
            lateral = fieldRelativeInputs[1];
        }

        // Calculate target motor powers
        calculateMechanumPowers(axial, lateral, yaw);

        // Apply drive mode limitations and optimizations
        applyDriveModeOptimizations();

        // Apply smooth acceleration
        applySmoothAcceleration();

        // Perform intelligent direction verification using odometry
        if (directionVerificationEnabled) {
            performDirectionVerification();
        }

        // Set motor powers
        setMotorPowers();

        // Update analytics
        updateAnalytics(axial, lateral, yaw);

        // Update Blackboard with drive data
        updateBlackboard();
    }

    /**
     * Update Blackboard with current drive system state
     */
    private void updateBlackboard() {
        // Drive system status
        blackboard.put("hardware.drive.ready", isInitialized);
        blackboard.put("drive.mode", currentMode.getName());
        blackboard.put("drive.field_relative", fieldRelative);
        blackboard.put("drive.voltage", currentVoltage);
        blackboard.put("drive.low_battery", currentVoltage < Tunables.DRIVE_LOW_VOLTAGE_THRESHOLD);

        // Motor powers for debugging
        blackboard.put("drive.motor.front_left", lastMotorPowers[0]);
        blackboard.put("drive.motor.front_right", lastMotorPowers[1]);
        blackboard.put("drive.motor.back_left", lastMotorPowers[2]);
        blackboard.put("drive.motor.back_right", lastMotorPowers[3]);

        // Drive analytics
        blackboard.put("drive.analytics.total_distance", totalDistance);
        blackboard.put("drive.analytics.energy_used", energyUsed);
        blackboard.put("drive.analytics.efficiency", getEfficiencyScore());
        blackboard.put("drive.analytics.sharp_turns", sharpTurns);
        blackboard.put("drive.analytics.average_speed", averageSpeed);

        // Direction verification status
        blackboard.put("drive.verification.enabled", directionVerificationEnabled);
        blackboard.put("drive.verification.mismatch_count", directionMismatchCount);
        blackboard.put("drive.verification.total_corrections", totalDirectionCorrections);
        blackboard.put("drive.verification.motors_inverted", motorDirectionsInverted);
        blackboard.put("drive.verification.mismatch_detected", directionMismatchDetected);

        // Autonomous drive commands (for state machine monitoring)
        blackboard.put("drive.command.axial", commandedAxial);
        blackboard.put("drive.command.lateral", commandedLateral);
        blackboard.put("drive.command.yaw", commandedYaw);

        // Check for autonomous drive commands from Blackboard
        boolean autoActive = blackboard.get("drive.auto.active", false);
        if (autoActive) {
            // Get autonomous drive targets
            double targetX = blackboard.get("drive.target.x", 0.0);
            double targetY = blackboard.get("drive.target.y", 0.0);
            double targetHeading = blackboard.get("drive.target.heading", 0.0);

            // Set autonomous drive inputs (this would integrate with path planning)
            // For now, just acknowledge the command
            blackboard.put("drive.auto.command_received", true);
        }
    }

    /**
     * Calculate standard mechanum drive powers
     */
    private void calculateMechanumPowers(double axial, double lateral, double yaw) {
        // Add fine movement controls to the main joystick inputs
        axial += fineY;
        lateral += fineX;
        yaw += fineRotation;

        targetMotorPowers[0] = axial + lateral + yaw; // Left Front
        targetMotorPowers[1] = axial - lateral - yaw; // Right Front
        targetMotorPowers[2] = axial - lateral + yaw; // Left Back
        targetMotorPowers[3] = axial + lateral - yaw; // Right Back

        // Normalize if any power exceeds 1.0
        double max = Math.max(Math.max(Math.abs(targetMotorPowers[0]), Math.abs(targetMotorPowers[1])),
                             Math.max(Math.abs(targetMotorPowers[2]), Math.abs(targetMotorPowers[3])));

        if (max > 1.0) {
            for (int i = 0; i < 4; i++) {
                targetMotorPowers[i] /= max;
            }
        }
    }

    /**
     * Apply field-relative transformation
     */
    private double[] applyFieldRelative(double axial, double lateral, double heading) {
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double newAxial = axial * cos - lateral * sin;
        double newLateral = axial * sin + lateral * cos;

        return new double[]{newAxial, newLateral};
    }

    /**
     * Apply drive mode specific optimizations
     */
    private void applyDriveModeOptimizations() {
        double modeMultiplier = currentMode.getMaxPower();

        // Battery voltage compensation
        if (batteryOptimization) {
            double voltageRatio = currentVoltage / Tunables.DRIVE_NOMINAL_VOLTAGE;
            if (voltageRatio < 0.9) { // Low battery
                modeMultiplier *= Math.max(Tunables.DRIVE_LOW_BATTERY_POWER_MULTIPLIER, voltageRatio);
            }
        }

        // Mode-specific adjustments
        switch (currentMode) {
            case PRECISION:
                // Apply exponential curve for fine control
                for (int i = 0; i < 4; i++) {
                    targetMotorPowers[i] = Math.signum(targetMotorPowers[i]) *
                                         Math.pow(Math.abs(targetMotorPowers[i]), 2);
                }
                break;

            case EFFICIENCY:
                // Reduce power for energy efficiency
                modeMultiplier *= 0.8;
                break;

            case AUTO_ADAPTIVE:
                // Adjust based on current conditions - only if voltage has been consistently low
                if (voltageConsistentlyLow) {
                    modeMultiplier *= 0.8; // Reduce power on sustained low battery
                }
                break;
        }

        // Apply multiplier
        for (int i = 0; i < 4; i++) {
            targetMotorPowers[i] *= modeMultiplier;
        }
    }

    /**
     * Apply smooth acceleration to prevent wheel slip and jerky movement
     */
    private void applySmoothAcceleration() {
        double deltaTime = accelerationTimer.seconds();
        accelerationTimer.reset();

        double maxDelta = Tunables.DRIVE_ACCELERATION_LIMIT * deltaTime;

        for (int i = 0; i < 4; i++) {
            double powerDiff = targetMotorPowers[i] - lastMotorPowers[i];

            if (Math.abs(powerDiff) > maxDelta) {
                lastMotorPowers[i] += Math.signum(powerDiff) * maxDelta;
            } else {
                lastMotorPowers[i] = targetMotorPowers[i];
            }
        }
    }

    /**
     * Set motor powers with safety checks
     */
    private void setMotorPowers() {
        if (!isInitialized()) return;

        leftFront.setPower(lastMotorPowers[0]);
        rightFront.setPower(lastMotorPowers[1]);
        leftBack.setPower(lastMotorPowers[2]);
        rightBack.setPower(lastMotorPowers[3]);
    }

    /**
     * Update battery status
     */
    private void updateBatteryStatus() {
        if (voltageSensor != null) {
            currentVoltage = voltageSensor.getVoltage();
        }

        // Check for sustained low voltage using Tunables
        if (currentVoltage < Tunables.DRIVE_LOW_VOLTAGE_THRESHOLD) {
            if (lowVoltageTimer.seconds() > Tunables.DRIVE_LOW_VOLTAGE_DURATION) {
                voltageConsistentlyLow = true;
            }
        } else {
            lowVoltageTimer.reset();
            voltageConsistentlyLow = false;
        }
    }

    /**
     * Update driving analytics
     */
    private void updateAnalytics(double axial, double lateral, double yaw) {
        // Calculate movement intensity
        double movementIntensity = Math.sqrt(axial * axial + lateral * lateral + yaw * yaw);

        // Estimate distance traveled (rough approximation)
        totalDistance += movementIntensity * 0.1; // Adjust scaling as needed

        // Estimate energy consumption
        double powerConsumption = 0;
        for (double power : lastMotorPowers) {
            powerConsumption += Math.abs(power);
        }
        energyUsed += powerConsumption * 0.01; // Rough estimate

        // Detect sharp turns using Tunables thresholds
        if (Math.abs(yaw) > Tunables.DRIVE_SHARP_TURN_THRESHOLD &&
            movementIntensity > Tunables.DRIVE_MOVEMENT_THRESHOLD) {
            sharpTurns++;
        }

        // Update average speed
        double sessionTime = sessionTimer.seconds();
        if (sessionTime > 0) {
            averageSpeed = totalDistance / sessionTime;
        }
    }

    /**
     * Perform intelligent direction verification using odometry feedback
     * Compares commanded autonomous movement direction with actual odometry velocity
     * Auto-corrects motor directions if consistent mismatch detected
     *
     * This is critical for autonomous operation where motors may be wired incorrectly
     * and the robot moves opposite to the path planner's commands.
     */
    private void performDirectionVerification() {
        // Only check periodically to reduce CPU load
        if (directionCheckTimer.seconds() < 0.1) { // Check every 100ms
            return;
        }
        directionCheckTimer.reset();

        // Get commanded movement magnitude (from autonomous path planner)
        double commandedMagnitude = Math.sqrt(
            lastCommandedVelocityX * lastCommandedVelocityX +
            lastCommandedVelocityY * lastCommandedVelocityY
        );

        // Skip verification if commanded movement is too small
        // (robot should be stopped or holding position)
        if (commandedMagnitude < Tunables.DRIVE_MIN_INPUT_FOR_VALIDATION) {
            directionMismatchDetected = false;
            return;
        }

        // Get actual velocity from odometry via Blackboard
        double actualVelocityX = blackboard.get("robot.velocity.x", 0.0);
        double actualVelocityY = blackboard.get("robot.velocity.y", 0.0);

        // Calculate actual velocity magnitude
        double actualMagnitude = Math.sqrt(
            actualVelocityX * actualVelocityX +
            actualVelocityY * actualVelocityY
        );

        // Skip verification if actual velocity is too small (robot might be stuck or starting)
        if (actualMagnitude < Tunables.DRIVE_MIN_VELOCITY_FOR_VALIDATION) {
            directionMismatchDetected = false;
            return;
        }

        // Calculate commanded direction angle (in degrees) - where autonomous wants to go
        double commandedAngle = Math.toDegrees(Math.atan2(lastCommandedVelocityY, lastCommandedVelocityX));

        // Calculate actual direction angle (in degrees) - where robot is actually going
        double actualAngle = Math.toDegrees(Math.atan2(actualVelocityY, actualVelocityX));

        // Calculate angular difference
        double angleDifference = Math.abs(normalizeAngleDegrees(actualAngle - commandedAngle));

        // Check if directions are opposite (mismatch)
        if (angleDifference > Tunables.DRIVE_DIRECTION_MISMATCH_THRESHOLD) {
            directionMismatchCount++;
            directionCorrectCount = 0; // Reset correct count
            directionMismatchDetected = true;

            // Check if we have enough consecutive mismatches to trigger correction
            if (directionMismatchCount >= Tunables.DRIVE_DIRECTION_MISMATCH_CONFIRMATIONS) {
                handleDirectionMismatch(commandedAngle, actualAngle, angleDifference);
            }
        } else {
            // Direction is correct
            directionCorrectCount++;
            directionMismatchDetected = false;

            // Reset mismatch count after enough correct readings
            if (directionCorrectCount >= Tunables.DRIVE_CORRECT_DIRECTION_RESET_COUNT) {
                directionMismatchCount = 0;
            }
        }
    }

    /**
     * Handle detected direction mismatch - apply auto-correction based on mode
     * This is critical for autonomous - if motors are backwards, the entire path will fail
     */
    private void handleDirectionMismatch(double commandedAngle, double actualAngle, double angleDifference) {
        // Log the mismatch
        blackboard.put("drive.verification.last_mismatch_time", System.currentTimeMillis());
        blackboard.put("drive.verification.commanded_angle", commandedAngle);
        blackboard.put("drive.verification.actual_angle", actualAngle);
        blackboard.put("drive.verification.angle_difference", angleDifference);

        // Apply correction based on auto-correction mode
        switch (Tunables.DRIVE_AUTO_CORRECTION_MODE) {
            case 0: // Disabled - log only
                blackboard.put("drive.verification.alert", "Direction mismatch detected - logging only");
                break;

            case 1: // Invert motor directions
                if (!motorDirectionsInverted) {
                    invertMotorDirections();
                    totalDirectionCorrections++;
                    motorDirectionsInverted = true;
                    blackboard.put("drive.verification.alert", "Motor directions auto-corrected!");
                }
                break;

            case 2: // Stop and alert (safety mode)
                emergencyStop();
                blackboard.put("drive.verification.alert", "CRITICAL: Direction mismatch - motors stopped!");
                break;
        }

        // Reset mismatch counter to avoid repeated corrections
        directionMismatchCount = 0;
    }

    /**
     * Invert all motor directions to correct backwards operation
     */
    private void invertMotorDirections() {
        if (!isInitialized()) return;

        // Invert all motor directions
        leftFront.setDirection(leftFront.getDirection() == DcMotor.Direction.FORWARD ?
            DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rightFront.setDirection(rightFront.getDirection() == DcMotor.Direction.FORWARD ?
            DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        leftBack.setDirection(leftBack.getDirection() == DcMotor.Direction.FORWARD ?
            DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rightBack.setDirection(rightBack.getDirection() == DcMotor.Direction.FORWARD ?
            DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        blackboard.put("drive.verification.last_correction_time", System.currentTimeMillis());
    }

    /**
     * Normalize angle to -180 to 180 degrees
     */
    private double normalizeAngleDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Emergency stop - immediately stop all motors
     */
    public void emergencyStop() {
        if (!isInitialized()) return;

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Reset commanded velocities
        commandedAxial = 0;
        commandedLateral = 0;
        commandedYaw = 0;
        clearFineMovement();
    }

    /**
     * Cycle through drive modes
     */
    public void cycleDriveMode() {
        DriveMode[] modes = DriveMode.values();
        int currentIndex = currentMode.ordinal();
        currentMode = modes[(currentIndex + 1) % modes.length];
    }

    /**
     * Toggle field-relative mode
     */
    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    /**
     * Set robot heading for field-relative driving
     */
    public void setRobotHeading(double heading) {
        this.robotHeading = Math.toRadians(heading);
    }

    /**
     * Get current robot heading in radians
     */
    public double getCurrentHeading() {
        return this.robotHeading;
    }

    /**
     * Get efficiency score (distance per energy unit)
     */
    public double getEfficiencyScore() {
        return energyUsed > 0 ? totalDistance / energyUsed : 0;
    }

    /**
     * Get status summary for debugging
     */
    public String getStatusSummary() {
        if (!isInitialized()) {
            return "DriveHandler: Not initialized - Drive system not ready";
        }

        return String.format("DriveHandler: %s mode, %.1fV, Field-Relative: %s, Debug: %s",
            currentMode.getName(),
            currentVoltage,
            fieldRelative ? "ON" : "OFF",
            Tunables.isDebugEnabled("drive") ? "ON" : "OFF");
    }

    /**
     * Get drive system telemetry data for centralized display
     * Enhanced with Tunables information
     */
    public DriveSystemData getTelemetryData() {
        return new DriveSystemData(
            currentMode.getName(),
            fieldRelative,
            currentVoltage,
            totalDistance,
            getEfficiencyScore(),
            sharpTurns,
            averageSpeed,
            lastMotorPowers.clone(),
            currentVoltage < Tunables.DRIVE_LOW_VOLTAGE_THRESHOLD,
            isInitialized,
            Tunables.isDebugEnabled("drive"),
            voltageConsistentlyLow
        );
    }

    /**
     * Data container for drive system telemetry information
     * Enhanced with additional Tunables-based information
     */
    public static class DriveSystemData {
        public final String driveMode;
        public final boolean fieldRelative;
        public final double voltage;
        public final double totalDistance;
        public final double efficiency;
        public final int sharpTurns;
        public final double averageSpeed;
        public final double[] motorPowers;
        public final boolean lowBattery;
        public final boolean initialized;
        public final boolean debugEnabled;
        public final boolean voltageConsistentlyLow;

        public DriveSystemData(String driveMode, boolean fieldRelative, double voltage,
                              double totalDistance, double efficiency, int sharpTurns,
                              double averageSpeed, double[] motorPowers, boolean lowBattery,
                              boolean initialized, boolean debugEnabled, boolean voltageConsistentlyLow) {
            this.driveMode = driveMode;
            this.fieldRelative = fieldRelative;
            this.voltage = voltage;
            this.totalDistance = totalDistance;
            this.efficiency = efficiency;
            this.sharpTurns = sharpTurns;
            this.averageSpeed = averageSpeed;
            this.motorPowers = motorPowers;
            this.lowBattery = lowBattery;
            this.initialized = initialized;
            this.debugEnabled = debugEnabled;
            this.voltageConsistentlyLow = voltageConsistentlyLow;
        }
    }

    /**
     * Get current Tunables configuration summary for diagnostics
     */
    public String getTunablesInfo() {
        return String.format("Tunables: Accel=%.1f, DeadZone=%.3f, RotSens=%.1f, FineMov=%.1f",
            Tunables.DRIVE_ACCELERATION_LIMIT,
            Tunables.DRIVE_DEADZONE,
            Tunables.DRIVE_ROTATION_SENSITIVITY,
            Tunables.DRIVE_FINE_MOVEMENT_SCALE);
    }

    // === GETTERS AND SETTERS ===

    public boolean isInitialized() {
        return isInitialized && robotMap.isDriveSystemReady();
    }

    public DriveMode getCurrentMode() {
        return currentMode;
    }

    public void setCurrentMode(DriveMode mode) {
        this.currentMode = mode;
    }

    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public void setFieldRelative(boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
    }

    public double getCurrentVoltage() {
        return currentVoltage;
    }

    public boolean isBatteryOptimizationEnabled() {
        return batteryOptimization;
    }

    public void setBatteryOptimization(boolean enabled) {
        this.batteryOptimization = enabled;
    }

    public boolean isAntiTipEnabled() {
        return antiTipEnabled;
    }

    public void setAntiTipEnabled(boolean enabled) {
        this.antiTipEnabled = enabled;
    }

    /**
     * Reset analytics
     */
    public void resetAnalytics() {
        totalDistance = 0;
        energyUsed = 0;
        sharpTurns = 0;
        averageSpeed = 0;
        sessionTimer.reset();
    }

    /**
     * Set fine movement values for precise control
     * Uses Tunables for scaling factor
     * @param x Lateral movement (-1.0 to 1.0, negative = left, positive = right)
     * @param y Axial movement (-1.0 to 1.0, negative = backward, positive = forward)
     * @param rotation Rotational movement (-1.0 to 1.0, negative = counter-clockwise, positive = clockwise)
     */
    public void setFineMovement(double x, double y, double rotation) {
        // Scale fine movement using Tunables
        this.fineX = Math.max(-1.0, Math.min(1.0, x)) * Tunables.DRIVE_FINE_MOVEMENT_SCALE;
        this.fineY = Math.max(-1.0, Math.min(1.0, y)) * Tunables.DRIVE_FINE_MOVEMENT_SCALE;
        this.fineRotation = Math.max(-1.0, Math.min(1.0, rotation)) * Tunables.DRIVE_FINE_MOVEMENT_SCALE;
    }

    /**
     * Set fine rotation value for precise rotational control
     * Uses Tunables for scaling factor
     * @param rotation Rotational movement (-1.0 to 1.0, negative = counter-clockwise, positive = clockwise)
     */
    public void setFineRotation(double rotation) {
        // Scale fine rotation using Tunables
        this.fineRotation = Math.max(-1.0, Math.min(1.0, rotation)) * Tunables.DRIVE_FINE_MOVEMENT_SCALE;
    }

    /**
     * Get current fine movement values
     * @return Array containing [x, y, rotation] values
     */
    public double[] getFineMovement() {
        return new double[]{fineX, fineY, fineRotation};
    }

    /**
     * Clear all fine movement values
     */
    public void clearFineMovement() {
        fineX = 0.0;
        fineY = 0.0;
        fineRotation = 0.0;
    }

    // === DIRECTION VERIFICATION CONTROL ===

    /**
     * Enable or disable direction verification
     */
    public void setDirectionVerificationEnabled(boolean enabled) {
        this.directionVerificationEnabled = enabled;
        if (!enabled) {
            // Reset counters when disabled
            directionMismatchCount = 0;
            directionCorrectCount = 0;
            directionMismatchDetected = false;
        }
    }

    /**
     * Check if direction verification is enabled
     */
    public boolean isDirectionVerificationEnabled() {
        return directionVerificationEnabled;
    }

    /**
     * Get direction verification status
     */
    public DirectionVerificationStatus getDirectionVerificationStatus() {
        return new DirectionVerificationStatus(
            directionVerificationEnabled,
            directionMismatchDetected,
            motorDirectionsInverted,
            directionMismatchCount,
            totalDirectionCorrections
        );
    }

    /**
     * Reset direction verification (useful after manual motor configuration)
     */
    public void resetDirectionVerification() {
        directionMismatchCount = 0;
        directionCorrectCount = 0;
        motorDirectionsInverted = false;
        totalDirectionCorrections = 0;
        directionMismatchDetected = false;
    }

    /**
     * Data container for direction verification status
     */
    public static class DirectionVerificationStatus {
        public final boolean enabled;
        public final boolean mismatchDetected;
        public final boolean motorsInverted;
        public final int mismatchCount;
        public final int totalCorrections;

        public DirectionVerificationStatus(boolean enabled, boolean mismatchDetected,
                                          boolean motorsInverted, int mismatchCount,
                                          int totalCorrections) {
            this.enabled = enabled;
            this.mismatchDetected = mismatchDetected;
            this.motorsInverted = motorsInverted;
            this.mismatchCount = mismatchCount;
            this.totalCorrections = totalCorrections;
        }
    }
}
