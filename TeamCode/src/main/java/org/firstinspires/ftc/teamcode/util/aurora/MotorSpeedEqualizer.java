/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Motor Speed Equalization System
 * Compensates for uneven motor speeds caused by weight distribution
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * MotorSpeedEqualizer - Encoder-based motor speed correction
 *
 * This system uses encoder feedback to measure actual motor speeds and
 * applies corrections to ensure all motors move at equal speeds despite
 * uneven weight distribution or other physical factors.
 *
 * Features:
 * - Real-time encoder-based speed measurement
 * - Three correction modes: Aggressive, Conservative, and Acceleration Sync
 * - Optional enable/disable toggle
 * - Acceleration monitoring and synchronization
 *
 * Correction Modes:
 * - AGGRESSIVE: Increases power to slower motors (limited by max power)
 * - CONSERVATIVE: Decreases power to faster motors to match slowest
 * - ACCELERATION_SYNC: Synchronizes acceleration/deceleration rates (recommended for direction changes)
 * - DISABLED: No speed equalization applied
 */
public class MotorSpeedEqualizer {

    /**
     * Correction mode enum
     */
    public enum CorrectionMode {
        DISABLED("Disabled", "No speed equalization"),
        AGGRESSIVE("Aggressive", "Boost slower motors"),
        CONSERVATIVE("Conservative", "Match slowest motor"),
        ACCELERATION_SYNC("Acceleration Sync", "Synchronize acceleration/deceleration rates");

        private final String name;
        private final String description;

        CorrectionMode(String name, String description) {
            this.name = name;
            this.description = description;
        }

        public String getName() { return name; }
        public String getDescription() { return description; }
    }

    // Motors to equalize
    private DcMotor[] motors;
    private String[] motorNames;

    // Correction settings
    private CorrectionMode correctionMode = CorrectionMode.DISABLED;
    private boolean enabled = false;

    // Speed tracking
    private int[] lastEncoderPositions;
    private double[] currentSpeeds;  // In encoder ticks per second
    private double[] lastAccelerations; // In encoder ticks per second squared
    private ElapsedTime speedTimer = new ElapsedTime();
    private ElapsedTime accelerationTimer = new ElapsedTime();

    // Correction parameters
    private double aggressiveBoostFactor = 0.15; // Max 15% power boost for slow motors
    private double conservativeReductionFactor = 0.15; // Max 15% power reduction for fast motors
    private double speedTolerancePercent = 5.0; // 5% speed difference tolerance
    private double minSpeedThreshold = 50.0; // Minimum speed (ticks/sec) to apply correction
    private double accelerationTolerancePercent = 10.0; // 10% acceleration difference tolerance
    private double accelerationCorrectionFactor = 0.20; // Max 20% power adjustment for acceleration sync

    // Constants for power thresholds
    private static final double MIN_POWER_THRESHOLD = 0.01; // Minimum power to consider motor as moving
    private static final double MIN_ACCELERATION_THRESHOLD = 10.0; // Minimum acceleration for balance check
    private static final double DECELERATION_DETECTION_THRESHOLD = -10.0; // Threshold to detect deceleration

    // Update rate limiting
    private double updateIntervalSeconds = 0.05; // Update every 50ms (20Hz)
    private ElapsedTime updateTimer = new ElapsedTime();

    // Correction tracking for telemetry
    private double[] appliedCorrections;
    private int correctionCount = 0;

    /**
     * Constructor
     *
     * @param motors Array of motors to equalize (typically 4 for mecanum drive)
     * @param motorNames Names of motors for debugging/telemetry
     */
    public MotorSpeedEqualizer(DcMotor[] motors, String[] motorNames) {
        if (motors == null || motors.length == 0) {
            throw new IllegalArgumentException("Motors array cannot be null or empty");
        }
        if (motorNames == null || motorNames.length != motors.length) {
            throw new IllegalArgumentException("Motor names must match motor count");
        }

        this.motors = motors;
        this.motorNames = motorNames;

        // Initialize tracking arrays
        int count = motors.length;
        lastEncoderPositions = new int[count];
        currentSpeeds = new double[count];
        lastAccelerations = new double[count];
        appliedCorrections = new double[count];

        // Initialize encoder positions
        for (int i = 0; i < count; i++) {
            lastEncoderPositions[i] = motors[i].getCurrentPosition();
        }

        speedTimer.reset();
        accelerationTimer.reset();
        updateTimer.reset();
    }

    /**
     * Update motor speeds and apply corrections
     * Call this method every loop iteration
     *
     * @param targetPowers The desired power levels for each motor (-1.0 to 1.0)
     * @return Corrected power levels for each motor
     */
    public double[] update(double[] targetPowers) {
        if (!enabled || correctionMode == CorrectionMode.DISABLED) {
            // Return target powers unchanged
            return targetPowers.clone();
        }

        // Rate limit updates
        if (updateTimer.seconds() < updateIntervalSeconds) {
            return targetPowers.clone();
        }
        updateTimer.reset();

        // Measure current motor speeds
        measureSpeeds();

        // Calculate and apply corrections
        double[] correctedPowers = applyCorrections(targetPowers);

        return correctedPowers;
    }

    /**
     * Measure current motor speeds using encoder deltas
     */
    private void measureSpeeds() {
        double deltaTime = speedTimer.seconds();
        speedTimer.reset();

        if (deltaTime < 0.001) {
            return; // Avoid division by zero
        }

        // Calculate speeds for each motor
        for (int i = 0; i < motors.length; i++) {
            int currentPosition = motors[i].getCurrentPosition();
            int deltaPosition = currentPosition - lastEncoderPositions[i];

            // Calculate speed in ticks per second
            double newSpeed = deltaPosition / deltaTime;

            // Store old speed for acceleration calculation
            double oldSpeed = currentSpeeds[i];
            currentSpeeds[i] = newSpeed;

            // Calculate acceleration (change in speed)
            double accelerationDeltaTime = accelerationTimer.seconds();
            if (accelerationDeltaTime > 0.001) {
                lastAccelerations[i] = (newSpeed - oldSpeed) / accelerationDeltaTime;
            }

            // Update last position
            lastEncoderPositions[i] = currentPosition;
        }

        accelerationTimer.reset();
    }

    /**
     * Apply speed corrections based on selected mode
     */
    private double[] applyCorrections(double[] targetPowers) {
        double[] correctedPowers = targetPowers.clone();

        // Calculate absolute speeds (ignore direction)
        double[] absSpeeds = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            absSpeeds[i] = Math.abs(currentSpeeds[i]);
        }

        // Find min and max speeds
        double minSpeed = Double.MAX_VALUE;
        double maxSpeed = 0;
        int slowestMotorIndex = 0;
        int fastestMotorIndex = 0;

        for (int i = 0; i < absSpeeds.length; i++) {
            if (absSpeeds[i] < minSpeed) {
                minSpeed = absSpeeds[i];
                slowestMotorIndex = i;
            }
            if (absSpeeds[i] > maxSpeed) {
                maxSpeed = absSpeeds[i];
                fastestMotorIndex = i;
            }
        }

        // Only apply correction if speeds are above minimum threshold
        if (maxSpeed < minSpeedThreshold) {
            // Motors are too slow, no correction needed
            for (int i = 0; i < motors.length; i++) {
                appliedCorrections[i] = 0;
            }
            return correctedPowers;
        }

        // Calculate speed variance
        double speedDifference = maxSpeed - minSpeed;
        double speedVariancePercent = (speedDifference / maxSpeed) * 100.0;

        // Only apply correction if variance exceeds tolerance
        if (speedVariancePercent < speedTolerancePercent) {
            // Speeds are close enough, no correction needed
            for (int i = 0; i < motors.length; i++) {
                appliedCorrections[i] = 0;
            }
            return correctedPowers;
        }

        // Apply correction based on mode
        switch (correctionMode) {
            case AGGRESSIVE:
                applyAggressiveCorrection(correctedPowers, absSpeeds, slowestMotorIndex, maxSpeed);
                break;

            case CONSERVATIVE:
                applyConservativeCorrection(correctedPowers, absSpeeds, minSpeed);
                break;

            case ACCELERATION_SYNC:
                applyAccelerationSyncCorrection(correctedPowers);
                break;

            case DISABLED:
            default:
                // No correction
                for (int i = 0; i < motors.length; i++) {
                    appliedCorrections[i] = 0;
                }
                break;
        }

        correctionCount++;
        return correctedPowers;
    }

    /**
     * Aggressive correction: Boost slower motors
     */
    private void applyAggressiveCorrection(double[] powers, double[] absSpeeds, int slowestIndex, double maxSpeed) {
        // For each motor, calculate how much slower it is compared to the fastest
        for (int i = 0; i < motors.length; i++) {
            if (i == slowestIndex && Math.abs(powers[i]) > MIN_POWER_THRESHOLD) {
                // This is the slowest motor - apply boost
                double speedRatio = absSpeeds[i] / maxSpeed;
                double boostNeeded = (1.0 - speedRatio) * aggressiveBoostFactor;

                // Apply boost in the same direction as current power
                double boost = Math.signum(powers[i]) * boostNeeded;
                powers[i] = Math.max(-1.0, Math.min(1.0, powers[i] + boost));

                appliedCorrections[i] = boost;
            } else {
                appliedCorrections[i] = 0;
            }
        }
    }

    /**
     * Conservative correction: Slow down faster motors
     */
    private void applyConservativeCorrection(double[] powers, double[] absSpeeds, double minSpeed) {
        // For each motor, calculate how much faster it is compared to the slowest
        for (int i = 0; i < motors.length; i++) {
            if (absSpeeds[i] > minSpeed && Math.abs(powers[i]) > MIN_POWER_THRESHOLD) {
                // This motor is faster - reduce its power
                double speedRatio = minSpeed / absSpeeds[i];
                double reductionNeeded = (1.0 - speedRatio) * conservativeReductionFactor;

                // Apply reduction in the opposite direction
                double reduction = Math.signum(powers[i]) * reductionNeeded;
                powers[i] = Math.max(-1.0, Math.min(1.0, powers[i] - reduction));

                appliedCorrections[i] = -reduction;
            } else {
                appliedCorrections[i] = 0;
            }
        }
    }

    /**
     * Acceleration synchronization correction: Ensure all motors accelerate/decelerate at same rate
     * This is especially important during direction changes and deceleration
     */
    private void applyAccelerationSyncCorrection(double[] powers) {
        // Check if motors are moving fast enough to apply corrections
        // During very slow speeds, acceleration measurements are unreliable
        double maxAbsSpeed = 0;
        for (int i = 0; i < motors.length; i++) {
            double absSpeed = Math.abs(currentSpeeds[i]);
            if (absSpeed > maxAbsSpeed) {
                maxAbsSpeed = absSpeed;
            }
        }
        
        // Use a lower threshold for acceleration sync since we care about deceleration too
        if (maxAbsSpeed < minSpeedThreshold * 0.5) {
            // Motors moving too slowly, acceleration data unreliable
            for (int i = 0; i < motors.length; i++) {
                appliedCorrections[i] = 0;
            }
            return;
        }

        // Calculate absolute accelerations
        double[] absAccelerations = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            absAccelerations[i] = lastAccelerations[i]; // Keep sign for deceleration detection
        }

        // Find min and max accelerations
        double minAccel = Double.MAX_VALUE;
        double maxAccel = -Double.MAX_VALUE;
        int slowestAcceleratingMotorIndex = 0;
        int fastestAcceleratingMotorIndex = 0;

        for (int i = 0; i < absAccelerations.length; i++) {
            if (absAccelerations[i] < minAccel) {
                minAccel = absAccelerations[i];
                slowestAcceleratingMotorIndex = i;
            }
            if (absAccelerations[i] > maxAccel) {
                maxAccel = absAccelerations[i];
                fastestAcceleratingMotorIndex = i;
            }
        }

        // Detect if we're in acceleration or deceleration phase
        boolean isDecelerating = false;
        int deceleratingCount = 0;
        for (int i = 0; i < motors.length; i++) {
            if (lastAccelerations[i] < DECELERATION_DETECTION_THRESHOLD) {
                deceleratingCount++;
            }
        }
        isDecelerating = deceleratingCount >= motors.length / 2; // At least half motors decelerating

        // Check if accelerations are significantly different
        double accelDifference = Math.abs(maxAccel - minAccel);
        if (accelDifference < MIN_ACCELERATION_THRESHOLD) {
            // Accelerations are too small or already balanced
            for (int i = 0; i < motors.length; i++) {
                appliedCorrections[i] = 0;
            }
            return;
        }

        // Apply corrections to synchronize acceleration/deceleration
        if (isDecelerating) {
            // DECELERATION: Make faster-decelerating motors slow down less aggressively
            // This prevents some wheels from stopping before others during direction changes
            for (int i = 0; i < motors.length; i++) {
                if (lastAccelerations[i] < minAccel && Math.abs(powers[i]) > MIN_POWER_THRESHOLD) {
                    // This motor is decelerating faster than the slowest - need to reduce braking
                    double accelRatio = lastAccelerations[i] / minAccel;
                    double correction = (1.0 - accelRatio) * accelerationCorrectionFactor;
                    
                    // Apply correction to slow down the deceleration (add power in current direction)
                    double sign = Math.signum(currentSpeeds[i]); // Use speed sign, not power
                    powers[i] = Math.max(-1.0, Math.min(1.0, powers[i] + (sign * correction)));
                    
                    appliedCorrections[i] = sign * correction;
                } else {
                    appliedCorrections[i] = 0;
                }
            }
        } else {
            // ACCELERATION: Make slower-accelerating motors speed up faster
            // This ensures all motors reach target speed together
            for (int i = 0; i < motors.length; i++) {
                if (lastAccelerations[i] < maxAccel && Math.abs(powers[i]) > MIN_POWER_THRESHOLD) {
                    // This motor is accelerating slower - boost it
                    double accelRatio = lastAccelerations[i] / maxAccel;
                    double boostNeeded = (1.0 - accelRatio) * accelerationCorrectionFactor;
                    
                    // Apply boost in the same direction as current power
                    double boost = Math.signum(powers[i]) * boostNeeded;
                    powers[i] = Math.max(-1.0, Math.min(1.0, powers[i] + boost));
                    
                    appliedCorrections[i] = boost;
                } else {
                    appliedCorrections[i] = 0;
                }
            }
        }
    }

    /**
     * Check if acceleration is balanced across motors
     * @return true if all motors have similar acceleration rates
     */
    public boolean isAccelerationBalanced() {
        if (!enabled || motors.length < 2) {
            return true;
        }

        double[] absAccelerations = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            absAccelerations[i] = Math.abs(lastAccelerations[i]);
        }

        // Find min and max accelerations
        double minAccel = Double.MAX_VALUE;
        double maxAccel = 0;
        for (double accel : absAccelerations) {
            if (accel < minAccel) minAccel = accel;
            if (accel > maxAccel) maxAccel = accel;
        }

        if (maxAccel < MIN_ACCELERATION_THRESHOLD) {
            // Accelerations too small to measure reliably
            return true;
        }

        // Calculate variance
        double accelVariancePercent = ((maxAccel - minAccel) / maxAccel) * 100.0;
        return accelVariancePercent < accelerationTolerancePercent;
    }

    // ========================================================================================
    // CONFIGURATION METHODS
    // ========================================================================================

    /**
     * Enable or disable speed equalization
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            correctionCount = 0;
        }
    }

    /**
     * Check if speed equalization is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Set correction mode
     */
    public void setCorrectionMode(CorrectionMode mode) {
        this.correctionMode = mode;
        correctionCount = 0;
    }

    /**
     * Get current correction mode
     */
    public CorrectionMode getCorrectionMode() {
        return correctionMode;
    }

    /**
     * Cycle to next correction mode
     */
    public void cycleCorrectionMode() {
        CorrectionMode[] modes = CorrectionMode.values();
        int currentIndex = correctionMode.ordinal();
        correctionMode = modes[(currentIndex + 1) % modes.length];
        correctionCount = 0;
    }

    /**
     * Set aggressive boost factor (0.0 to 1.0)
     */
    public void setAggressiveBoostFactor(double factor) {
        this.aggressiveBoostFactor = Math.max(0.0, Math.min(1.0, factor));
    }

    /**
     * Set conservative reduction factor (0.0 to 1.0)
     */
    public void setConservativeReductionFactor(double factor) {
        this.conservativeReductionFactor = Math.max(0.0, Math.min(1.0, factor));
    }

    /**
     * Set speed tolerance percentage
     */
    public void setSpeedTolerancePercent(double percent) {
        this.speedTolerancePercent = Math.max(0.0, percent);
    }

    /**
     * Set minimum speed threshold for applying corrections
     */
    public void setMinSpeedThreshold(double threshold) {
        this.minSpeedThreshold = Math.max(0.0, threshold);
    }

    /**
     * Set acceleration tolerance percentage
     */
    public void setAccelerationTolerancePercent(double percent) {
        this.accelerationTolerancePercent = Math.max(0.0, percent);
    }

    /**
     * Set acceleration correction factor (0.0 to 1.0)
     * Controls how aggressively to correct acceleration/deceleration imbalances
     */
    public void setAccelerationCorrectionFactor(double factor) {
        this.accelerationCorrectionFactor = Math.max(0.0, Math.min(1.0, factor));
    }

    /**
     * Get acceleration correction factor
     */
    public double getAccelerationCorrectionFactor() {
        return accelerationCorrectionFactor;
    }

    // ========================================================================================
    // TELEMETRY / DIAGNOSTICS
    // ========================================================================================

    /**
     * Get current motor speeds (ticks per second)
     */
    public double[] getCurrentSpeeds() {
        return currentSpeeds.clone();
    }

    /**
     * Get current motor accelerations (ticks per second squared)
     */
    public double[] getCurrentAccelerations() {
        return lastAccelerations.clone();
    }

    /**
     * Get applied corrections for each motor
     */
    public double[] getAppliedCorrections() {
        return appliedCorrections.clone();
    }

    /**
     * Get total number of corrections applied
     */
    public int getCorrectionCount() {
        return correctionCount;
    }

    /**
     * Get motor names
     */
    public String[] getMotorNames() {
        return motorNames.clone();
    }

    /**
     * Get telemetry data for display
     */
    public EqualizerTelemetryData getTelemetryData() {
        return new EqualizerTelemetryData(
            enabled,
            correctionMode,
            currentSpeeds.clone(),
            lastAccelerations.clone(),
            appliedCorrections.clone(),
            motorNames.clone(),
            correctionCount,
            isAccelerationBalanced()
        );
    }

    /**
     * Data container for telemetry
     */
    public static class EqualizerTelemetryData {
        public final boolean enabled;
        public final CorrectionMode mode;
        public final double[] speeds;
        public final double[] accelerations;
        public final double[] corrections;
        public final String[] motorNames;
        public final int correctionCount;
        public final boolean accelerationBalanced;

        public EqualizerTelemetryData(boolean enabled, CorrectionMode mode,
                                     double[] speeds, double[] accelerations,
                                     double[] corrections, String[] motorNames,
                                     int correctionCount, boolean accelerationBalanced) {
            this.enabled = enabled;
            this.mode = mode;
            this.speeds = speeds;
            this.accelerations = accelerations;
            this.corrections = corrections;
            this.motorNames = motorNames;
            this.correctionCount = correctionCount;
            this.accelerationBalanced = accelerationBalanced;
        }
    }
}
