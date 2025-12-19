/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Configurable shooter parameters for different game scenarios
 */

package org.firstinspires.ftc.teamcode.util.aurora;

/**
 * ShooterConfig - Centralized configuration for shooter parameters
 *
 * This allows easy tuning without recompiling and supports multiple presets.
 *
 * Hardware Specifications:
 * - 2x DC Motors with encoders (1:1 gear ratio)
 * - Maximum RPM: 6000
 * - Friction flywheel launch mechanism
 */
public class ShooterConfig {

    // ═══════════════════════════════════════════════════════════════════════
    // PRESET DEFINITIONS - EDIT THESE FOR EASY TUNING
    // ═══════════════════════════════════════════════════════════════════════
    // Format: NAME, POWER, TARGET_RPM, SHOT_INTERVAL (seconds), SPINUP_TIME (seconds)

    /**
     * Preset configurations for different shooting ranges
     *
     * Each preset defines the shooter parameters for a specific distance range.
     * The shotInterval and spinupTime parameters determine the firing mode:
     * - Normal Mode: Uses recovery time (spinupTime > 0) for accuracy
     * - Rapid Fire Mode: No recovery (spinupTime = 0) for maximum fire rate
     *
     * To enable rapid fire mode, set spinupTime to 0.0 when holding the shoot button.
     */
    public enum ShooterPreset {
        LONG_RANGE("Long Range", 0.85, 4400, 1.0, 1.2),
        MID_RANGE("Mid Range", 0.75, 3500, 0.6, 0.8),
        SHORT_RANGE("Short Range", 1.0, 2800, 0.25, 0.8);

        private final String name;
        private final double power;
        private final double targetRPM;
        private final double shotInterval;      // Min time between shots (seconds)
        private final double spinupTime;        // Recovery/spinup time (seconds)

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

        /** Get shot interval in milliseconds */
        public long getShotIntervalMs() {
            return (long)(shotInterval * 1000);
        }

        /** Get spinup time in milliseconds */
        public long getSpinupTimeMs() {
            return (long)(spinupTime * 1000);
        }

        /** Check if this is rapid fire mode (no recovery) */
        public boolean isRapidFire() {
            return spinupTime <= 0;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CURRENT CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════

    /** Current active preset */
    private ShooterPreset currentPreset = ShooterPreset.LONG_RANGE;

    // ═══════════════════════════════════════════════════════════════════════
    // ADVANCED PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Feed servo power */
    private double feedPower = 1.0;

    /** Feed servo activation time (seconds) */
    private double feedTime = 0.2;

    /** RPM tolerance for "at target" status */
    private double rpmTolerance = 75;

    /** Stricter RPM tolerance during shooting */
    private double shootingRpmTolerance = 50;

    /** Time RPM must be stable before shooting (seconds) */
    private double rpmStabilityTime = 0.25;

    /** Use RPM-based spinup (vs time-based) */
    private boolean useRpmSpinup = true;

    /** Maximum spinup time before timeout (seconds) */
    private double maxSpinupTime = 3.0;

    // Warmup mode parameters
    /** Warmup runs at this percentage of target RPM */
    private double warmupRpmPercentage = 0.65;

    /** Faster spinup time for warmup mode */
    private double warmupSpinupTime = 0.8;

    // Battery voltage compensation
    /** Enable battery voltage compensation */
    private boolean batteryCompensation = true;

    /** Nominal battery voltage */
    private double nominalVoltage = 12.0;

    /** Minimum safe battery voltage */
    private double minVoltage = 10.5;


    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE SPECIFICATIONS (STATIC CONSTANTS)
    // ═══════════════════════════════════════════════════════════════════════

    /** Maximum motor RPM (hardware limit) */
    public static final double MAX_RPM = 6000.0;

    /** Minimum safe RPM for launching */
    public static final double MIN_LAUNCH_RPM = 1000.0;

    /** Motor encoder ticks per revolution (adjust for your motors) */
    public static final double TICKS_PER_REVOLUTION = 28.0;

    /** Motor gear ratio (output/input) */
    public static final double GEAR_RATIO = 1.0;

    // ═══════════════════════════════════════════════════════════════════════
    // PID CONTROL PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** PID coefficient: Proportional gain */
    public static final double PID_KP = 0.0001;

    /** PID coefficient: Integral gain */
    public static final double PID_KI = 0.00001;

    /** PID coefficient: Derivative gain */
    public static final double PID_KD = 0.0;

    /** PID coefficient: Feedforward gain (velocity feedforward) */
    public static final double PID_KF = 0.00015;

    /** Maximum integral accumulation (anti-windup) */
    public static final double MAX_INTEGRAL = 0.3;

    /** Minimum power output (prevent stalling) */
    public static final double MIN_POWER = 0.0;

    /** Maximum power output */
    public static final double MAX_POWER = 1.0;

    // ═══════════════════════════════════════════════════════════════════════
    // RPM SYNCHRONIZATION PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Maximum acceptable RPM difference between motors */
    public static final double MAX_RPM_SYNC_ERROR = 100.0;

    /** Sync correction factor (how aggressively to correct differences) */
    public static final double SYNC_CORRECTION_FACTOR = 0.5;

    /** Emergency stop if sync error exceeds this value */
    public static final double EMERGENCY_SYNC_ERROR = 500.0;

    // ═══════════════════════════════════════════════════════════════════════
    // TIMING PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Measurement update rate (milliseconds) */
    public static final long UPDATE_RATE_MS = 20;

    /** Default minimum time between shots (milliseconds) */
    public static final long MIN_SHOT_INTERVAL_MS = 500;

    /** Default maximum spin-up time (milliseconds) */
    public static final long MAX_SPINUP_TIME_MS = 3000;

    /** Time to maintain target RPM before allowing shot (milliseconds) */
    public static final long RPM_STABILIZATION_TIME_MS = 250;

    /** Default RPM tolerance for "at target" status */
    public static final double RPM_TOLERANCE = 50.0;

    /** Warm-up mode spin-up percentage (65% of target) */
    public static final double WARMUP_PERCENTAGE = 0.65;

    // ═══════════════════════════════════════════════════════════════════════
    // SAFETY PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Enable safety checks (RPM limits, sync error, etc.) */
    public static final boolean ENABLE_SAFETY_CHECKS = true;

    /**
     * Motor direction configuration
     * Adjust these if motors spin the wrong direction
     * For counter-rotating flywheels, one should be FORWARD, one REVERSE
     */
    public static final boolean REVERSE_LEFT_MOTOR = false;
    public static final boolean REVERSE_RIGHT_MOTOR = true;

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS - PRESET MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════

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

    /**
     * Get power without battery compensation (uses preset value directly)
     */
    public double getPower() {
        return currentPreset.getPower();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS - CURRENT PRESET VALUES
    // ═══════════════════════════════════════════════════════════════════════

    public double getTargetRPM() { return currentPreset.getTargetRPM(); }
    public double getShotInterval() { return currentPreset.getShotInterval(); }
    public double getSpinupTime() { return currentPreset.getSpinupTime(); }
    public long getShotIntervalMs() { return currentPreset.getShotIntervalMs(); }
    public long getSpinupTimeMs() { return currentPreset.getSpinupTimeMs(); }

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS/SETTERS - ADVANCED PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    public double getFeedPower() { return feedPower; }
    public void setFeedPower(double feedPower) { this.feedPower = feedPower; }

    public double getFeedTime() { return feedTime; }
    public void setFeedTime(double feedTime) { this.feedTime = feedTime; }

    public double getRpmTolerance() { return rpmTolerance; }
    public void setRpmTolerance(double rpmTolerance) { this.rpmTolerance = rpmTolerance; }

    public double getShootingRpmTolerance() { return shootingRpmTolerance; }
    public void setShootingRpmTolerance(double shootingRpmTolerance) {
        this.shootingRpmTolerance = shootingRpmTolerance;
    }

    public double getRpmStabilityTime() { return rpmStabilityTime; }
    public void setRpmStabilityTime(double rpmStabilityTime) {
        this.rpmStabilityTime = rpmStabilityTime;
    }

    public boolean isUseRpmSpinup() { return useRpmSpinup; }
    public void setUseRpmSpinup(boolean useRpmSpinup) { this.useRpmSpinup = useRpmSpinup; }

    public double getMaxSpinupTime() { return maxSpinupTime; }
    public void setMaxSpinupTime(double maxSpinupTime) { this.maxSpinupTime = maxSpinupTime; }

    public boolean isBatteryCompensation() { return batteryCompensation; }
    public void setBatteryCompensation(boolean batteryCompensation) {
        this.batteryCompensation = batteryCompensation;
    }

    public double getWarmupRpmPercentage() { return warmupRpmPercentage; }
    public void setWarmupRpmPercentage(double warmupRpmPercentage) {
        this.warmupRpmPercentage = warmupRpmPercentage;
    }

    public double getWarmupSpinupTime() { return warmupSpinupTime; }
    public void setWarmupSpinupTime(double warmupSpinupTime) {
        this.warmupSpinupTime = warmupSpinupTime;
    }

    public double getWarmupTargetRPM() {
        return currentPreset.getTargetRPM() * warmupRpmPercentage;
    }

    public double getNominalVoltage() { return nominalVoltage; }
    public void setNominalVoltage(double nominalVoltage) { this.nominalVoltage = nominalVoltage; }

    public double getMinVoltage() { return minVoltage; }
    public void setMinVoltage(double minVoltage) { this.minVoltage = minVoltage; }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Validate if an RPM value is within safe operating limits
     */
    public static boolean isRPMValid(double rpm) {
        return rpm >= 0 && rpm <= MAX_RPM;
    }

    /**
     * Clamp RPM to safe operating range
     */
    public static double clampRPM(double rpm) {
        return Math.max(0, Math.min(MAX_RPM, rpm));
    }

    /**
     * Get preset by name
     */
    public static ShooterPreset getPresetByName(String name) {
        for (ShooterPreset preset : ShooterPreset.values()) {
            if (preset.getName().equalsIgnoreCase(name)) {
                return preset;
            }
        }
        return null;
    }

    /**
     * Calculate warm-up RPM for a given target (65% of target)
     */
    public static double getWarmupRPM(double targetRPM) {
        return targetRPM * WARMUP_PERCENTAGE;
    }
}
