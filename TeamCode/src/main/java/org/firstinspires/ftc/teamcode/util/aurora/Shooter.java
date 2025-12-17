package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter - Interface for the Aurora shooter system
 *
 * This class provides a clean, simplified interface to the shooter subsystem for use
 * by the IndexingSystem and other components. It wraps the DecodeHelper class and
 * provides only the methods needed for artifact firing operations.
 *
 * Key Responsibilities:
 * - Check if shooter is ready to fire
 * - Verify shooter is spinning at target RPM
 * - Provide shooter state information
 * - Abstract away complex shooter implementation details
 */
public class Shooter {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final DecodeHelper decodeHelper;
    private final ShooterConfig config;
    private final Telemetry telemetry;

    private boolean enabled;
    private long lastSpinupTime;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new Shooter instance
     * @param hardware The Aurora hardware configuration
     * @param config The shooter configuration
     * @param telemetry The telemetry system for logging
     */
    public Shooter(AuroraHardwareConfig hardware, ShooterConfig config, Telemetry telemetry) {
        this.decodeHelper = new DecodeHelper(hardware, telemetry);
        this.config = config;
        this.telemetry = telemetry;
        this.enabled = false;
        this.lastSpinupTime = 0;
    }

    /**
     * Create a Shooter with existing DecodeHelper instance
     * @param decodeHelper The DecodeHelper instance to wrap
     * @param config The shooter configuration
     * @param telemetry The telemetry system for logging
     */
    public Shooter(DecodeHelper decodeHelper, ShooterConfig config, Telemetry telemetry) {
        this.decodeHelper = decodeHelper;
        this.config = config;
        this.telemetry = telemetry;
        this.enabled = false;
        this.lastSpinupTime = 0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update the shooter state (call periodically from OpMode loop)
     */
    public void update() {
        decodeHelper.update();
    }

    /**
     * Start spinning up the shooter to target RPM
     * @return true if spinup started successfully
     */
    public boolean spinUp() {
        if (!enabled) {
            if (telemetry != null) {
                telemetry.addLine("Cannot spin up: shooter disabled");
            }
            return false;
        }

        decodeHelper.spinUp();
        lastSpinupTime = System.currentTimeMillis();
        return true;
    }

    /**
     * Start spinning up with a specific preset
     * @param preset The shooter preset to use
     * @return true if spinup started successfully
     */
    public boolean spinUp(ShooterConfig.ShooterPreset preset) {
        if (!enabled) {
            if (telemetry != null) {
                telemetry.addLine("Cannot spin up: shooter disabled");
            }
            return false;
        }

        decodeHelper.setPreset(preset);
        decodeHelper.spinUp();
        lastSpinupTime = System.currentTimeMillis();
        return true;
    }

    /**
     * Enable warm-up mode (lower RPM for reduced latency)
     */
    public void enableWarmup() {
        decodeHelper.enableWarmup();
    }

    /**
     * Stop the shooter
     */
    public void stop() {
        decodeHelper.disableShooter();
        enabled = false;
    }

    /**
     * Enable the shooter for operation
     */
    public void enable() {
        enabled = true;
    }

    /**
     * Disable the shooter
     */
    public void disable() {
        stop();
    }

    /**
     * Fire an artifact (if ready)
     * @return true if firing started successfully
     */
    public boolean fire() {
        if (!enabled) {
            if (telemetry != null) {
                telemetry.addLine("Cannot fire: shooter disabled");
            }
            return false;
        }

        if (!isReadyToFire()) {
            if (telemetry != null) {
                telemetry.addLine("Cannot fire: shooter not ready");
            }
            return false;
        }

        return decodeHelper.fire();
    }

    /**
     * Set the target RPM for the shooter
     * @param rpm Target RPM (revolutions per minute)
     */
    public void setTargetRPM(double rpm) {
        decodeHelper.setTargetRPM(rpm);
    }

    /**
     * Set the shooter preset
     * @param preset The preset to use
     */
    public void setPreset(ShooterConfig.ShooterPreset preset) {
        config.setPreset(preset);
        decodeHelper.setPreset(preset);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATE QUERY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if shooter is ready to fire
     * @return true if at target RPM and stable
     */
    public boolean isReadyToFire() {
        if (!enabled) {
            return false;
        }

        // Check if shooter is at target RPM
        if (!decodeHelper.isAtTargetRPM()) {
            return false;
        }

        // Check if minimum spinup time has elapsed
        long timeSinceSpinup = System.currentTimeMillis() - lastSpinupTime;
        if (timeSinceSpinup < config.getPreset().getSpinupTimeMs()) {
            return false;
        }

        // Check shooter state
        return decodeHelper.isReady();
    }

    /**
     * Check if shooter is currently spinning
     * @return true if shooter motors are running
     */
    public boolean isRunning() {
        return enabled && !decodeHelper.isIdle();
    }

    /**
     * Check if shooter is at target RPM
     * @return true if current RPM matches target within tolerance
     */
    public boolean isAtTargetRPM() {
        return decodeHelper.isAtTargetRPM();
    }

    /**
     * Check if shooter is currently firing
     * @return true if in firing state
     */
    public boolean isFiring() {
        return decodeHelper.isFiring();
    }

    /**
     * Check if shooter is in error state
     * @return true if error detected
     */
    public boolean isError() {
        return decodeHelper.isError();
    }

    /**
     * Check if shooter is enabled
     * @return true if enabled for operation
     */
    public boolean isEnabled() {
        return enabled;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS INFORMATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get current shooter RPM (average of both motors)
     * @return Current RPM
     */
    public double getCurrentRPM() {
        return decodeHelper.getAverageRPM();
    }

    /**
     * Get target RPM
     * @return Target RPM
     */
    public double getTargetRPM() {
        return decodeHelper.getTargetRPM();
    }

    /**
     * Get current shooter state
     * @return The shooter state
     */
    public DecodeHelper.ShooterState getState() {
        return decodeHelper.getState();
    }

    /**
     * Get time until next shot is allowed
     * @return Time in milliseconds
     */
    public long getTimeUntilNextShot() {
        return decodeHelper.getTimeUntilNextShot();
    }

    /**
     * Get RPM as percentage of target
     * @return Percentage (0.0 to 1.0+)
     */
    public double getRPMPercentage() {
        return decodeHelper.getRPMPercentage();
    }

    /**
     * Get synchronization error between left and right motors
     * @return RPM difference between motors
     */
    public double getSyncError() {
        return decodeHelper.getRPMSyncError();
    }

    /**
     * Clear any error state
     */
    public void clearError() {
        decodeHelper.clearError();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // DIRECT ACCESS (for advanced use)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get the underlying DecodeHelper instance for advanced control
     * @return The DecodeHelper instance
     */
    public DecodeHelper getDecodeHelper() {
        return decodeHelper;
    }

    /**
     * Get the shooter configuration
     * @return The ShooterConfig instance
     */
    public ShooterConfig getConfig() {
        return config;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS SUMMARY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get a summary string of shooter status
     * @return Status summary
     */
    public String getStatusSummary() {
        return String.format("Shooter: %s | RPM: %.0f/%.0f (%.0f%%) | %s", 
            enabled ? "ON" : "OFF",
            getCurrentRPM(),
            getTargetRPM(),
            getRPMPercentage() * 100,
            isReadyToFire() ? "READY" : "NOT READY"
        );
    }
}
