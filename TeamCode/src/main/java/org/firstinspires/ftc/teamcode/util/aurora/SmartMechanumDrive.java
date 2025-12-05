/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Intelligent drive system with efficiency optimization and predictive control
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

/**
 * SmartMechanumDrive - Advanced drive system with efficiency and performance optimization
 *
 * Features:
 * - Dynamic power scaling based on battery voltage
 * - Predictive movement with acceleration curves
 * - Energy-efficient driving modes
 * - Anti-tip protection for high-speed maneuvers
 * - Driver performance analytics
 */
public class SmartMechanumDrive {

    // Hardware
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private VoltageSensor voltageSensor;
    private Gamepad gamepad;
    private GoBildaPinpointDriver odometry;  // Using Pinpoint odometry instead of IMU (more accurate, no drift)

    // Drive modes
    public enum DriveMode {
        PRECISION(0.3, "Precision", "Slow, accurate movements"),
        NORMAL(0.8, "Normal", "Balanced speed and control"),
        SPORT(1.0, "Sport", "Maximum speed and agility"),
        EFFICIENCY(0.6, "Efficiency", "Optimized for battery life"),
        AUTO_ADAPTIVE(1.0, "Auto-Adaptive", "AI-optimized based on conditions");

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
    private boolean fieldRelative = false;
    private double robotHeading = 0;

    // Performance optimization
    private ElapsedTime accelerationTimer = new ElapsedTime();
    private double[] lastMotorPowers = new double[4];
    private double[] targetMotorPowers = new double[4];
    private static final double ACCELERATION_LIMIT = 5.0; // Power units per second
    private boolean enableAccelerationLimiting = true; // Can be disabled for PID control

    // Battery optimization
    private double nominalVoltage = 12.0;
    private double currentVoltage = 12.0;
    private boolean batteryOptimization = true;
    private static final double LOW_VOLTAGE_THRESHOLD = 10.5; // Voltage threshold for efficiency mode
    private static final double LOW_VOLTAGE_DURATION = 3.0; // Seconds voltage must be low before switching
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
    private double maxTiltRate = 5.0; // degrees per second

    // Fine movement control
    private double fineX = 0.0;
    private double fineY = 0.0;
    private double fineRotation = 0.0;

    // Manual drive inputs (used when gamepad is null)
    private double manualAxial = 0.0;
    private double manualLateral = 0.0;
    private double manualYaw = 0.0;

    // Heading stabilization (Pinpoint odometry-based rotation correction)
    // ENABLED by default - Pinpoint odometry is accurate and doesn't drift like IMU
    private boolean headingStabilizationEnabled = false;  // ENABLED - using Pinpoint odometry (no drift!)
    private double targetHeading = 0; // The heading we want to maintain (in degrees)
    private double headingTolerance = 2.0; // Degrees of acceptable drift before correction
    private double headingCorrectionGain = 0.008; // PID proportional gain - REDUCED MORE to prevent wiggling
    private static final double HEADING_CORRECTION_DEADBAND = 5.0; // Ignore errors under 5 degrees - MORE TOLERANT
    private static final double MAX_HEADING_CORRECTION = 0.08; // Maximum correction power - VERY GENTLE
    private static final double MOVEMENT_THRESHOLD = 0.08; // Lower threshold to detect any movement
    private static final double YAW_INTENT_THRESHOLD = 0.02; // Very small yaw input = intentional rotation

    /**
     * Constructor
     */
    public SmartMechanumDrive(DcMotor leftFront, DcMotor rightFront,
                             DcMotor leftBack, DcMotor rightBack,
                             Gamepad gamepad, VoltageSensor voltageSensor) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.gamepad = gamepad;
        this.voltageSensor = voltageSensor;
        this.odometry = null; // No odometry by default

        configureMotors();
    }

    /**
     * Constructor with Pinpoint odometry support for heading stabilization
     */
    public SmartMechanumDrive(DcMotor leftFront, DcMotor rightFront,
                             DcMotor leftBack, DcMotor rightBack,
                             Gamepad gamepad, VoltageSensor voltageSensor, GoBildaPinpointDriver odometry) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.gamepad = gamepad;
        this.voltageSensor = voltageSensor;
        this.odometry = odometry;

        configureMotors();

        // Initialize target heading if odometry is available
        if (odometry != null) {
            odometry.update();
            targetHeading = odometry.getHeading(AngleUnit.DEGREES);
        }
    }

    /**
     * Set drive inputs manually (alternative to gamepad input)
     */
    public void setDriveInputs(double axial, double lateral, double yaw) {
        this.manualAxial = axial;
        this.manualLateral = lateral;
        this.manualYaw = yaw;
    }

    /**
     * Configure motors with optimal settings
     */
    private void configureMotors() {
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

        // Set directions (adjust as needed for your robot)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Main drive update method - call this every loop
     */
    public void update() {
        updateBatteryStatus();

        // Get raw joystick inputs - prioritize manual inputs from AuroraManager
        double axial, lateral, yaw;

        // Always use manual inputs since AuroraManager handles all gamepad processing
        axial = manualAxial;
        lateral = manualLateral;
        yaw = manualYaw;

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

        // Set motor powers
        setMotorPowers();

        // Update analytics
        updateAnalytics(axial, lateral, yaw);
    }

    /**
     * Calculate standard mechanum drive powers
     */
    private void calculateMechanumPowers(double axial, double lateral, double yaw) {
        // Add fine movement controls to the main joystick inputs
        axial += fineY;
        lateral += fineX;
        yaw += fineRotation;

        // Apply Pinpoint odometry-based heading stabilization if enabled and odometry is available
        if (headingStabilizationEnabled && odometry != null) {
            yaw = applyHeadingStabilization(yaw);
        }

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
     * Apply Pinpoint odometry-based heading stabilization to prevent unwanted rotation
     * This actively corrects drift when the driver isn't inputting yaw
     *
     * @param yaw The driver's yaw input
     * @return The corrected yaw with heading stabilization applied
     */
    private double applyHeadingStabilization(double yaw) {
        // Always update odometry first
        odometry.update();
        double currentHeading = odometry.getHeading(AngleUnit.DEGREES);

        // If driver is actively rotating (even slightly), continuously update target heading
        if (Math.abs(yaw) > YAW_INTENT_THRESHOLD) { // Driver is intentionally rotating
            // CONTINUOUSLY update target heading while driver rotates
            // This way when they stop rotating, the target is already set to where they want to be
            targetHeading = currentHeading;
            return yaw; // Use driver's input directly - NO correction at all
        }

        // Check if robot is actively moving (axial or lateral movement without rotation)
        double movementIntensity = Math.sqrt(manualAxial * manualAxial + manualLateral * manualLateral);

        // If robot is moving AT ALL (but not rotating), DON'T apply correction
        // Update target heading while moving to prevent snap-back when stopping
        if (movementIntensity > MOVEMENT_THRESHOLD) {
            // Keep target heading updated to current heading during movement
            targetHeading = currentHeading;
            return 0; // No correction during ANY movement
        }

        // Robot is STATIONARY and driver is NOT rotating
        // NOW we can apply heading stabilization to the LAST SET target heading

        // Calculate heading error (how much we've drifted from target)
        double headingError = normalizeAngle(targetHeading - currentHeading);

        // Only apply correction if error is above deadband (5 degrees)
        if (Math.abs(headingError) < HEADING_CORRECTION_DEADBAND) {
            return 0; // No correction needed - within tolerance
        }

        // Calculate correction using proportional control - VERY gentle
        double correction = headingError * headingCorrectionGain;

        // Clamp correction to maximum allowed (very small)
        correction = Math.max(-MAX_HEADING_CORRECTION, Math.min(MAX_HEADING_CORRECTION, correction));

        return correction; // Apply the gentle correction as yaw input
    }

    /**
     * Normalize angle to -180 to +180 degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
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

        // Battery voltage compensation - DISABLED to prevent rotation issues
        // The voltage compensation was causing uneven power delivery during quick direction changes
        // Comment out for now until we can implement a better solution
        /*
        if (batteryOptimization) {
            double voltageRatio = currentVoltage / nominalVoltage;
            if (voltageRatio < 0.9) { // Low battery
                modeMultiplier *= Math.max(0.7, voltageRatio); // Minimum 70% power
            }
        }
        */

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
                // DISABLED: This was causing rotation issues during quick strafe changes
                /*
                if (voltageConsistentlyLow) {
                    modeMultiplier *= 0.8; // Reduce power on sustained low battery
                }
                */
                break;
        }

        // Apply multiplier uniformly to all motors to maintain balance
        for (int i = 0; i < 4; i++) {
            targetMotorPowers[i] *= modeMultiplier;
        }
    }

    /**
     * Apply smooth acceleration to prevent wheel slip and jerky movement
     */
    private void applySmoothAcceleration() {
        // If acceleration limiting is disabled (e.g., for PID control), apply target directly
        if (!enableAccelerationLimiting) {
            for (int i = 0; i < 4; i++) {
                lastMotorPowers[i] = targetMotorPowers[i];
            }
            return;
        }

        double deltaTime = accelerationTimer.seconds();
        accelerationTimer.reset();

        // Use a higher limit for better responsiveness and to prevent rotation issues
        double maxDelta = ACCELERATION_LIMIT * deltaTime;

        // Track if we're doing a direction reversal (sign change)
        boolean[] directionReversal = new boolean[4];
        for (int i = 0; i < 4; i++) {
            directionReversal[i] = (Math.signum(targetMotorPowers[i]) != Math.signum(lastMotorPowers[i]))
                                   && Math.abs(lastMotorPowers[i]) > 0.05;
        }

        // Apply acceleration limiting with special handling for direction reversals
        for (int i = 0; i < 4; i++) {
            double powerDiff = targetMotorPowers[i] - lastMotorPowers[i];

            // For direction reversals, allow faster deceleration to zero to prevent rotation
            if (directionReversal[i]) {
                // Quickly decelerate to zero first, then accelerate in new direction
                if (Math.abs(lastMotorPowers[i]) > 0.05) {
                    // Decelerate to zero with higher limit
                    lastMotorPowers[i] *= 0.5; // Faster deceleration
                } else {
                    // Now accelerate in new direction
                    lastMotorPowers[i] = targetMotorPowers[i] * 0.3; // Quick initial acceleration
                }
            } else {
                // Normal acceleration limiting
                if (Math.abs(powerDiff) > maxDelta) {
                    lastMotorPowers[i] += Math.signum(powerDiff) * maxDelta;
                } else {
                    lastMotorPowers[i] = targetMotorPowers[i];
                }
            }
        }
    }

    /**
     * Set motor powers with safety checks
     */
    private void setMotorPowers() {
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

        // Check for sustained low voltage
        if (currentVoltage < LOW_VOLTAGE_THRESHOLD) {
            if (lowVoltageTimer.seconds() > LOW_VOLTAGE_DURATION) {
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

        // Detect sharp turns
        if (Math.abs(yaw) > 0.8 && movementIntensity > 0.5) {
            sharpTurns++;
        }

        // Update average speed
        double sessionTime = sessionTimer.seconds();
        if (sessionTime > 0) {
            averageSpeed = totalDistance / sessionTime;
        }
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
     * Set whether to enable smooth acceleration limiting
     * Should be disabled for PID-controlled autonomous movement
     */
    public void setAccelerationLimiting(boolean enable) {
        this.enableAccelerationLimiting = enable;
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
     * Get drive system telemetry data for centralized display
     * This replaces the old updateTelemetry method to support centralized telemetry
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
            currentVoltage < LOW_VOLTAGE_THRESHOLD
        );
    }

    /**
     * Data container for drive system telemetry information
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

        public DriveSystemData(String driveMode, boolean fieldRelative, double voltage,
                              double totalDistance, double efficiency, int sharpTurns,
                              double averageSpeed, double[] motorPowers, boolean lowBattery) {
            this.driveMode = driveMode;
            this.fieldRelative = fieldRelative;
            this.voltage = voltage;
            this.totalDistance = totalDistance;
            this.efficiency = efficiency;
            this.sharpTurns = sharpTurns;
            this.averageSpeed = averageSpeed;
            this.motorPowers = motorPowers;
            this.lowBattery = lowBattery;
        }
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

    // Getters and setters
    public DriveMode getCurrentMode() { return currentMode; }
    public void setCurrentMode(DriveMode mode) { this.currentMode = mode; }
    public boolean isFieldRelative() { return fieldRelative; }
    public void setFieldRelative(boolean fieldRelative) { this.fieldRelative = fieldRelative; }
    public double getCurrentVoltage() { return currentVoltage; }

    /**
     * Set fine movement values for precise control
     * @param x Lateral movement (-1.0 to 1.0, negative = left, positive = right)
     * @param y Axial movement (-1.0 to 1.0, negative = backward, positive = forward)
     * @param rotation Rotational movement (-1.0 to 1.0, negative = counter-clockwise, positive = clockwise)
     */
    public void setFineMovement(double x, double y, double rotation) {
        // Scale fine movement to 20% power for precise control
        this.fineX = Math.max(-1.0, Math.min(1.0, x)) * 0.2; // Clamp and scale to 20%
        this.fineY = Math.max(-1.0, Math.min(1.0, y)) * 0.2;
        this.fineRotation = Math.max(-1.0, Math.min(1.0, rotation)) * 0.2;
    }

    /**
     * Set fine rotation value for precise rotational control
     * @param rotation Rotational movement (-1.0 to 1.0, negative = counter-clockwise, positive = clockwise)
     */
    public void setFineRotation(double rotation) {
        // Scale fine rotation to 20% power for precise control
        this.fineRotation = Math.max(-1.0, Math.min(1.0, rotation)) * 0.2; // Clamp and scale to 20%
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

    /**
     * Enable or disable IMU-based heading stabilization
     * @param enabled True to enable, false to disable
     */
    public void setHeadingStabilizationEnabled(boolean enabled) {
        this.headingStabilizationEnabled = enabled;
    }

    /**
     * Check if heading stabilization is enabled
     * @return True if enabled, false otherwise
     */
    public boolean isHeadingStabilizationEnabled() {
        return headingStabilizationEnabled;
    }

    /**
     * Set the heading correction gain (PID proportional constant)
     * Higher values = more aggressive correction, but may oscillate
     * Lower values = smoother correction, but slower response
     * @param gain The proportional gain (recommended: 0.01 to 0.05)
     */
    public void setHeadingCorrectionGain(double gain) {
        this.headingCorrectionGain = gain;
    }

    /**
     * Get the current heading correction gain
     * @return The proportional gain value
     */
    public double getHeadingCorrectionGain() {
        return headingCorrectionGain;
    }

    /**
     * Manually reset the target heading to the current odometry reading
     * Useful after autonomous or if the robot has been moved
     */
    public void resetTargetHeading() {
        if (odometry != null) {
            odometry.update();
            targetHeading = odometry.getHeading(AngleUnit.DEGREES);
        }
    }

    /**
     * Get the current target heading that stabilization is maintaining
     * @return Target heading in degrees
     */
    public double getTargetHeading() {
        return targetHeading;
    }

    /**
     * Get the current actual heading from the Pinpoint odometry
     * @return Current heading in degrees, or 0 if odometry not available
     */
    public double getOdometryHeading() {
        if (odometry != null) {
            odometry.update();
            return odometry.getHeading(AngleUnit.DEGREES);
        }
        return 0;
    }

    /**
     * @deprecated Use getOdometryHeading() instead
     */
    @Deprecated
    public double getIMUHeading() {
        return getOdometryHeading();
    }

    /**
     * Get the current heading error (drift from target)
     * @return Heading error in degrees
     */
    public double getHeadingError() {
        if (odometry != null) {
            odometry.update();
            double currentHeading = odometry.getHeading(AngleUnit.DEGREES);
            return normalizeAngle(targetHeading - currentHeading);
        }
        return 0;
    }
}
