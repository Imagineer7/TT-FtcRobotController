package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * Lightweight performance monitor for FTC OpModes.
 * Tracks loop times, voltage, and other critical metrics.
 * Provides haptic feedback via gamepad vibration for warnings.
 *
 * @author Aurora System v2
 * @version 1.0
 */
public class PerformanceMonitor {

    // Thresholds
    private static final double VOLTAGE_CRITICAL = 11.5;  // Critical battery
    private static final double VOLTAGE_WARNING = 12.0;   // Low battery
    private static final long LOOP_TIME_WARNING_MS = 50;  // 20 Hz minimum
    private static final long LOOP_TIME_CRITICAL_MS = 100; // 10 Hz minimum

    // Vibration patterns (milliseconds)
    private static final int VIBRATE_SHORT = 100;
    private static final int VIBRATE_MEDIUM = 300;
    private static final int VIBRATE_LONG = 500;
    private static final long VIBRATE_COOLDOWN_MS = 3000; // 3 seconds between warnings

    // Spike filtering - prevent vibration on quick drops
    private static final long SUSTAINED_WARNING_THRESHOLD_MS = 400; // Must persist for 400ms

    // State tracking
    private final Telemetry telemetry;
    private long currentLoopTime;
    private long loopStartTime;
    private double lastVoltage;
    private boolean enabled = true;

    // Warning state
    private WarningLevel currentWarning = WarningLevel.NONE;
    private long lastVibrationTime = 0;
    private String lastWarningMessage = "";

    // Spike filtering state
    private WarningLevel sustainedWarningLevel = WarningLevel.NONE;
    private long warningStartTime = 0;
    private boolean warningAcknowledged = false;

    // Statistics
    private long loopCount = 0;
    private long totalLoopTime = 0;
    private long maxLoopTime = 0;
    private double minVoltage = Double.MAX_VALUE;

    /**
     * Warning severity levels
     */
    public enum WarningLevel {
        NONE,      // No warnings
        INFO,      // Information only
        WARNING,   // Performance degradation
        CRITICAL   // Immediate attention needed
    }

    /**
     * Create a new performance monitor
     * @param telemetry Telemetry for status display
     */
    public PerformanceMonitor(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.loopStartTime = System.currentTimeMillis();
    }

    /**
     * Enable or disable the monitor
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Check if monitor is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Mark the start of a loop cycle
     * Call this at the beginning of your loop
     */
    public void startLoop() {
        if (!enabled) return;
        loopStartTime = System.currentTimeMillis();
    }

    /**
     * Mark the end of a loop cycle and perform checks
     * Call this at the end of your loop
     *
     * @param voltage Current battery voltage
     * @param gamepad1 Primary gamepad for haptic feedback
     * @param gamepad2 Secondary gamepad for haptic feedback
     */
    public void endLoop(double voltage, Gamepad gamepad1, Gamepad gamepad2) {
        if (!enabled) return;

        currentLoopTime = System.currentTimeMillis();
        long loopDuration = currentLoopTime - loopStartTime;

        // Update statistics
        loopCount++;
        totalLoopTime += loopDuration;
        maxLoopTime = Math.max(maxLoopTime, loopDuration);
        lastVoltage = voltage;
        minVoltage = Math.min(minVoltage, voltage);

        // Check for warnings
        checkWarnings(loopDuration, voltage);

        // Spike filtering: Track if warning has been sustained
        if (currentWarning != WarningLevel.NONE) {
            // If this is a new warning or warning level changed
            if (currentWarning != sustainedWarningLevel) {
                // Start tracking this warning
                sustainedWarningLevel = currentWarning;
                warningStartTime = currentLoopTime;
                warningAcknowledged = false;
            } else {
                // Same warning is continuing - check if sustained long enough
                long warningDuration = currentLoopTime - warningStartTime;

                // Only vibrate if warning has persisted AND hasn't been acknowledged yet
                if (warningDuration >= SUSTAINED_WARNING_THRESHOLD_MS && !warningAcknowledged) {
                    // Check cooldown since last vibration
                    long timeSinceLastVibration = currentLoopTime - lastVibrationTime;
                    if (timeSinceLastVibration >= VIBRATE_COOLDOWN_MS) {
                        sendVibration(gamepad1, gamepad2);
                        lastVibrationTime = currentLoopTime;
                        warningAcknowledged = true; // Don't vibrate again for same sustained warning
                    }
                }
            }
        } else {
            // No warning - reset tracking
            sustainedWarningLevel = WarningLevel.NONE;
            warningStartTime = 0;
            warningAcknowledged = false;
        }
    }

    /**
     * Check for warning conditions
     */
    private void checkWarnings(long loopDuration, double voltage) {
        currentWarning = WarningLevel.NONE;
        lastWarningMessage = "";

        // Check voltage (highest priority)
        if (voltage <= VOLTAGE_CRITICAL) {
            currentWarning = WarningLevel.CRITICAL;
            lastWarningMessage = String.format(Locale.US, "CRITICAL: Battery %.1fV", voltage);
            return;
        } else if (voltage <= VOLTAGE_WARNING) {
            currentWarning = WarningLevel.WARNING;
            lastWarningMessage = String.format(Locale.US, "WARNING: Battery %.1fV", voltage);
        }

        // Check loop time
        if (loopDuration >= LOOP_TIME_CRITICAL_MS) {
            if (currentWarning.ordinal() < WarningLevel.CRITICAL.ordinal()) {
                currentWarning = WarningLevel.CRITICAL;
                lastWarningMessage = String.format(Locale.US, "CRITICAL: Loop %dms (%.1f Hz)",
                    loopDuration, 1000.0 / loopDuration);
            }
        } else if (loopDuration >= LOOP_TIME_WARNING_MS) {
            if (currentWarning.ordinal() < WarningLevel.WARNING.ordinal()) {
                currentWarning = WarningLevel.WARNING;
                lastWarningMessage = String.format(Locale.US, "WARNING: Loop %dms (%.1f Hz)",
                    loopDuration, 1000.0 / loopDuration);
            }
        }
    }

    /**
     * Send haptic feedback based on warning level
     */
    private void sendVibration(Gamepad gamepad1, Gamepad gamepad2) {
        switch (currentWarning) {
            case CRITICAL:
                // Three long pulses
                vibratePattern(gamepad1, gamepad2,
                    VIBRATE_LONG, VIBRATE_SHORT,
                    VIBRATE_LONG, VIBRATE_SHORT,
                    VIBRATE_LONG);
                break;

            case WARNING:
                // Two medium pulses
                vibratePattern(gamepad1, gamepad2,
                    VIBRATE_MEDIUM, VIBRATE_SHORT,
                    VIBRATE_MEDIUM);
                break;

            case INFO:
                // Single short pulse
                vibratePattern(gamepad1, gamepad2, VIBRATE_SHORT);
                break;

            case NONE:
            default:
                // No vibration
                break;
        }
    }

    /**
     * Helper to vibrate gamepads with a pattern
     * Supports up to 5 duration values (on-off-on-off-on)
     */
    private void vibratePattern(Gamepad gamepad1, Gamepad gamepad2, int... durations) {
        if (durations.length == 0) return;

        // Gamepad.rumble() only supports simple patterns
        // For complex patterns, we use rumbleBlips()
        if (durations.length == 1) {
            // Single pulse
            if (gamepad1 != null) gamepad1.rumble(durations[0]);
            if (gamepad2 != null) gamepad2.rumble(durations[0]);
        } else if (durations.length == 3) {
            // Two pulses (on-off-on)
            if (gamepad1 != null) gamepad1.rumbleBlips(2);
            if (gamepad2 != null) gamepad2.rumbleBlips(2);
        } else if (durations.length == 5) {
            // Three pulses (on-off-on-off-on)
            if (gamepad1 != null) gamepad1.rumbleBlips(3);
            if (gamepad2 != null) gamepad2.rumbleBlips(3);
        }
    }

    /**
     * Get current warning level
     */
    public WarningLevel getCurrentWarning() {
        return currentWarning;
    }

    /**
     * Get sustained warning level (only changes after threshold)
     */
    public WarningLevel getSustainedWarning() {
        return sustainedWarningLevel;
    }

    /**
     * Get how long the current warning has been active (milliseconds)
     */
    public long getWarningDuration() {
        if (sustainedWarningLevel == WarningLevel.NONE) {
            return 0;
        }
        return currentLoopTime - warningStartTime;
    }

    /**
     * Check if warning has been sustained long enough to trigger vibration
     */
    public boolean isWarningSustained() {
        return getWarningDuration() >= SUSTAINED_WARNING_THRESHOLD_MS;
    }

    /**
     * Get last warning message
     */
    public String getLastWarningMessage() {
        return lastWarningMessage;
    }

    /**
     * Get current loop time in milliseconds
     */
    public long getCurrentLoopTime() {
        return currentLoopTime - loopStartTime;
    }

    /**
     * Get average loop time in milliseconds
     */
    public double getAverageLoopTime() {
        if (loopCount == 0) return 0;
        return (double) totalLoopTime / loopCount;
    }

    /**
     * Get max loop time in milliseconds
     */
    public long getMaxLoopTime() {
        return maxLoopTime;
    }

    /**
     * Get current loop frequency in Hz
     */
    public double getCurrentLoopFrequency() {
        long currentDuration = currentLoopTime - loopStartTime;
        if (currentDuration == 0) return 0;
        return 1000.0 / currentDuration;
    }

    /**
     * Get average loop frequency in Hz
     */
    public double getAverageLoopFrequency() {
        double avgTime = getAverageLoopTime();
        if (avgTime == 0) return 0;
        return 1000.0 / avgTime;
    }

    /**
     * Get last voltage reading
     */
    public double getLastVoltage() {
        return lastVoltage;
    }

    /**
     * Get minimum voltage recorded
     */
    public double getMinVoltage() {
        return minVoltage == Double.MAX_VALUE ? 0 : minVoltage;
    }

    /**
     * Get total loop count
     */
    public long getLoopCount() {
        return loopCount;
    }

    /**
     * Display performance metrics to telemetry
     *
     * @param detailed If true, show detailed statistics
     */
    public void displayTelemetry(boolean detailed) {
        if (!enabled) {
            telemetry.addData("⚡ Performance", "Monitor Disabled");
            return;
        }

        // Current status
        String statusIcon = getStatusIcon();
        telemetry.addData(statusIcon + " Performance",
            String.format(Locale.US, "%.1f Hz | %.1fV",
                getCurrentLoopFrequency(),
                lastVoltage));

        // Warning message if present
        if (currentWarning != WarningLevel.NONE) {
            String warningPrefix = "⚠️ Alert";

            // Show if warning is sustained or still in spike detection period
            if (sustainedWarningLevel != WarningLevel.NONE) {
                long duration = getWarningDuration();
                if (duration < SUSTAINED_WARNING_THRESHOLD_MS) {
                    // Warning detected but not sustained yet
                    warningPrefix = String.format(Locale.US, "⏱️ Alert (%dms)", duration);
                } else if (warningAcknowledged) {
                    // Warning sustained and vibration sent
                    warningPrefix = "⚠️ Alert (vibrated)";
                }
            }

            telemetry.addData(warningPrefix, lastWarningMessage);
        }

        // Detailed statistics
        if (detailed) {
            telemetry.addData("  Loop Time",
                String.format(Locale.US, "Cur: %dms | Avg: %.1fms | Max: %dms",
                    getCurrentLoopTime(),
                    getAverageLoopTime(),
                    maxLoopTime));

            telemetry.addData("  Voltage",
                String.format(Locale.US, "Cur: %.2fV | Min: %.2fV",
                    lastVoltage,
                    getMinVoltage()));

            telemetry.addData("  Loop Count", loopCount);
        }
    }

    /**
     * Get status icon based on current warning level
     */
    private String getStatusIcon() {
        switch (currentWarning) {
            case CRITICAL: return "🔴";
            case WARNING: return "🟡";
            case INFO: return "🔵";
            case NONE:
            default: return "🟢";
        }
    }

    /**
     * Reset all statistics
     */
    public void reset() {
        loopCount = 0;
        totalLoopTime = 0;
        maxLoopTime = 0;
        minVoltage = Double.MAX_VALUE;
        currentWarning = WarningLevel.NONE;
        lastWarningMessage = "";
        loopStartTime = System.currentTimeMillis();

        // Reset spike filtering state
        sustainedWarningLevel = WarningLevel.NONE;
        warningStartTime = 0;
        warningAcknowledged = false;
    }

    /**
     * Get a summary string of current performance
     */
    public String getSummary() {
        return String.format(Locale.US, "Performance: %.1f Hz avg, %.1fV min, %d loops, %dms max",
            getAverageLoopFrequency(),
            getMinVoltage(),
            loopCount,
            maxLoopTime);
    }
}
