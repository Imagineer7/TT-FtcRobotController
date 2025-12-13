/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Configurable shooter parameters for different game scenarios
 */

package org.firstinspires.ftc.teamcode.util.aurora;

/**
 * ShooterConfig - Centralized configuration for shooter parameters
 *
 * This allows easy tuning without recompiling and supports multiple presets
 */
public class ShooterConfig {

    // Preset configurations for different scenarios
    public enum ShooterPreset {
        LONG_RANGE("Long Range", 0.85, 4400, 1.0, 1.2),
        SHORT_RANGE("Short Range", 1.0, 2800, 0.25, 0.8),  // Increased from 2300 to 2800 RPM
        RAPID_FIRE("Rapid Fire", 0.80, 3900, 0.3, 0.7),    // Balanced speed/consistency
        PRECISION("Precision", 0.90, 4800, 2.0, 1.5),
        BATTERY_SAVER("Battery Saver", 0.65, 3000, 1.8, 1.0);

        private final String name;
        private final double power;
        private final double targetRPM;
        private final double shotInterval;
        private final double spinupTime;

        ShooterPreset(String name, double power, double targetRPM, double shotInterval, double spinupTime) {
            this.name = name;
            this.power = power;
            this.targetRPM = targetRPM;
            this.shotInterval = shotInterval;
            this.spinupTime = spinupTime;
        }

        public String getName() { return name; }
        public double getPower() { return power; }
        public double getTargetRPM() { return targetRPM; }
        public double getShotInterval() { return shotInterval; }
        public double getSpinupTime() { return spinupTime; }
    }

    // Current configuration
    private ShooterPreset currentPreset = ShooterPreset.LONG_RANGE;

    // Advanced parameters
    private double feedPower = 1.0;
    private double feedTime = 0.2;
    private double rpmTolerance = 75;
    private double shootingRpmTolerance = 50; // Back to 50 RPM for consistency
    private double rpmStabilityTime = 0.25;
    private boolean useRpmSpinup = true;
    private double maxSpinupTime = 3.0;

    // Warmup mode parameters
    private double warmupRpmPercentage = 0.65; // Warmup runs at 65% of target RPM to save power
    private double warmupSpinupTime = 0.8; // Faster spinup time for warmup mode

    // Battery voltage compensation
    private boolean batteryCompensation = true;
    private double nominalVoltage = 12.0;
    private double minVoltage = 10.5;

    /**
     * Set the current shooting preset
     */
    public void setPreset(ShooterPreset preset) {
        this.currentPreset = preset;
    }

    /**
     * Get current preset
     */
    public ShooterPreset getPreset() {
        return currentPreset;
    }

    /**
     * Get power with optional battery compensation
     */
    public double getPower(double batteryVoltage) {
        double basePower = currentPreset.getPower();

        if (batteryCompensation && batteryVoltage > 0) {
            // More aggressive compensation for battery voltage drop
            double voltageRatio = nominalVoltage / Math.max(batteryVoltage, minVoltage);

            // Apply more aggressive scaling when voltage drops significantly
            if (batteryVoltage < nominalVoltage * 0.9) {
                // Below 90% of nominal voltage, use more aggressive compensation
                voltageRatio = Math.pow(voltageRatio, 0.8); // Less aggressive curve
                basePower = Math.min(1.0, basePower * voltageRatio * 1.1); // Extra 10% boost
            } else {
                basePower = Math.min(1.0, basePower * voltageRatio);
            }
        }

        return basePower;
    }

    // Getters for current preset values
    public double getTargetRPM() { return currentPreset.getTargetRPM(); }
    public double getShotInterval() { return currentPreset.getShotInterval(); }
    public double getSpinupTime() { return currentPreset.getSpinupTime(); }

    // Getters/Setters for advanced parameters
    public double getFeedPower() { return feedPower; }
    public void setFeedPower(double feedPower) { this.feedPower = feedPower; }

    public double getFeedTime() { return feedTime; }
    public void setFeedTime(double feedTime) { this.feedTime = feedTime; }

    public double getRpmTolerance() { return rpmTolerance; }
    public void setRpmTolerance(double rpmTolerance) { this.rpmTolerance = rpmTolerance; }

    public double getShootingRpmTolerance() { return shootingRpmTolerance; }
    public void setShootingRpmTolerance(double shootingRpmTolerance) { this.shootingRpmTolerance = shootingRpmTolerance; }

    public boolean isUseRpmSpinup() { return useRpmSpinup; }
    public void setUseRpmSpinup(boolean useRpmSpinup) { this.useRpmSpinup = useRpmSpinup; }

    public boolean isBatteryCompensation() { return batteryCompensation; }
    public void setBatteryCompensation(boolean batteryCompensation) { this.batteryCompensation = batteryCompensation; }

    public double getWarmupRpmPercentage() { return warmupRpmPercentage; }
    public void setWarmupRpmPercentage(double warmupRpmPercentage) { this.warmupRpmPercentage = warmupRpmPercentage; }

    public double getWarmupSpinupTime() { return warmupSpinupTime; }
    public void setWarmupSpinupTime(double warmupSpinupTime) { this.warmupSpinupTime = warmupSpinupTime; }

    public double getWarmupTargetRPM() { return currentPreset.getTargetRPM() * warmupRpmPercentage; }
}
