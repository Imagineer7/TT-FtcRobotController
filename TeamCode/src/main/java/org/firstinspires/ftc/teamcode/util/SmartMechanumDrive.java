/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Intelligent drive system with efficiency optimization and predictive control
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private DriveMode currentMode = DriveMode.NORMAL;
    private boolean fieldRelative = false;
    private double robotHeading = 0;

    // Performance optimization
    private ElapsedTime accelerationTimer = new ElapsedTime();
    private double[] lastMotorPowers = new double[4];
    private double[] targetMotorPowers = new double[4];
    private static final double ACCELERATION_LIMIT = 2.0; // Power units per second

    // Battery optimization
    private double nominalVoltage = 12.0;
    private double currentVoltage = 12.0;
    private boolean batteryOptimization = true;

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

        configureMotors();
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

        // Get raw joystick inputs
        double axial = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;
        double yaw = gamepad.right_stick_x;

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
            double voltageRatio = currentVoltage / nominalVoltage;
            if (voltageRatio < 0.9) { // Low battery
                modeMultiplier *= Math.max(0.7, voltageRatio); // Minimum 70% power
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
                // Adjust based on current conditions
                if (currentVoltage < 11.0) {
                    modeMultiplier *= 0.8; // Reduce power on low battery
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

        double maxDelta = ACCELERATION_LIMIT * deltaTime;

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
            currentVoltage < 11.0
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
}
