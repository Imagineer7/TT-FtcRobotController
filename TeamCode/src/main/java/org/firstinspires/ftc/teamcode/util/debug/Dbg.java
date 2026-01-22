package org.firstinspires.ftc.teamcode.util.debug;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Locale;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Dbg - A lightweight, FTC-optimized logging utility.
 *
 * Features:
 * - Hierarchical log levels (TRACE, DEBUG, INFO, WARN, ERROR)
 * - Per-group filtering (enable/disable specific subsystems)
 * - Contextual prefixes (robot name, opmode, phase)
 * - Spam control (rate limiting, once-only, counted)
 * - Zero-allocation when disabled
 * - Integration with FTC RobotLog
 *
 * Usage:
 * <pre>
 * // Setup (in OpMode init)
 * Dbg.setGlobalPrefix("SnowRover");
 * Dbg.setContext("TeleOp", "INIT");
 * Dbg.setGlobalLevel(LogLevel.DEBUG);
 *
 * // Basic logging
 * Dbg.d(LogGroup.DRIVE, "Speed: %.2f", speed);
 * Dbg.w(LogGroup.SENSORS, "Low voltage: %.1fV", voltage);
 * Dbg.e(LogGroup.HARDWARE, "Motor '%s' not found", motorName);
 *
 * // Spam control
 * Dbg.everyMs(LogGroup.INTAKE, LogLevel.INFO, "status", 1000,
 *             "Artifacts: %d", count);
 * Dbg.once(LogGroup.AUTO, LogLevel.WARN, "deprecated",
 *          "Using deprecated path");
 * </pre>
 *
 * @author FTC Team
 * @version 1.0
 */
public class Dbg {

    // ==================== Configuration ====================

    private static volatile LogLevel globalLevel = LogLevel.INFO;
    private static volatile String globalPrefix = "";
    private static volatile String contextOpMode = "";
    private static volatile String contextPhase = "";
    private static volatile boolean includeTimestamp = false;
    private static volatile boolean includeLoopCount = false;
    private static volatile int loopCount = 0;

    // Telemetry output (optional)
    private static volatile boolean enablePanelsTelemetry = false;
    private static volatile boolean enableFtcTelemetry = false;
    private static volatile Telemetry ftcTelemetry = null;
    private static volatile int maxTelemetryLines = 10;
    private static final ConcurrentHashMap<String, String> lastTelemetryMessages = new ConcurrentHashMap<>();

    // Per-group configuration
    private static final ConcurrentHashMap<String, LogLevel> groupLevels = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, Boolean> groupEnabled = new ConcurrentHashMap<>();

    // Spam control state
    private static final ConcurrentHashMap<String, AtomicLong> lastLogTime = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, Boolean> onceKeys = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, AtomicInteger> countKeys = new ConcurrentHashMap<>();

    // FTC RobotLog tag
    private static final String ROBOT_LOG_TAG = "TeamDbg";

    // ==================== Global Configuration ====================

    /**
     * Set the global minimum log level.
     * Only messages at this level or higher will be logged.
     *
     * @param level Minimum level (TRACE, DEBUG, INFO, WARN, ERROR)
     */
    public static void setGlobalLevel(LogLevel level) {
        globalLevel = level;
        RobotLog.ii(ROBOT_LOG_TAG, "Global log level set to: " + level);
    }

    /**
     * Get the current global log level.
     */
    public static LogLevel getGlobalLevel() {
        return globalLevel;
    }

    /**
     * Set a global prefix for all log messages (e.g., robot name).
     *
     * @param prefix Robot identifier (e.g., "SnowRover", "TestBot")
     */
    public static void setGlobalPrefix(String prefix) {
        globalPrefix = (prefix == null) ? "" : prefix;
    }

    /**
     * Set the current context (OpMode name and phase).
     * Call this in your OpMode's init() and at phase transitions.
     *
     * @param opMode OpMode name (e.g., "TeleOp", "AutoRed")
     * @param phase Current phase ("INIT", "START", "RUN", "STOP")
     */
    public static void setContext(String opMode, String phase) {
        contextOpMode = (opMode == null) ? "" : opMode;
        contextPhase = (phase == null) ? "" : phase;
    }

    /**
     * Enable or disable timestamp in log prefix.
     * Timestamps are in milliseconds since epoch.
     */
    public static void setIncludeTimestamp(boolean include) {
        includeTimestamp = include;
    }

    /**
     * Enable or disable loop counter in log prefix.
     * Call incrementLoop() at the start of your OpMode loop.
     */
    public static void setIncludeLoopCount(boolean include) {
        includeLoopCount = include;
    }

    /**
     * Increment the loop counter. Call once per OpMode loop iteration.
     */
    public static void incrementLoop() {
        loopCount++;
    }

    /**
     * Reset the loop counter (call when OpMode starts).
     */
    public static void resetLoop() {
        loopCount = 0;
    }

    /**
     * Reset all context and state (call in OpMode stop()).
     */
    public static void reset() {
        contextOpMode = "";
        contextPhase = "";
        loopCount = 0;
        lastLogTime.clear();
        onceKeys.clear();
        countKeys.clear();
        lastTelemetryMessages.clear();
    }

    // ==================== Telemetry Configuration ====================

    /**
     * Enable or disable Panels telemetry output.
     * When enabled, logs at INFO level and above will be sent to Panels.
     *
     * @param enable true to enable Panels output
     */
    public static void setEnablePanelsTelemetry(boolean enable) {
        enablePanelsTelemetry = enable;
    }

    /**
     * Enable or disable FTC telemetry output.
     * When enabled, logs at INFO level and above will be sent to the provided telemetry.
     *
     * @param telemetry FTC Telemetry object from OpMode
     * @param enable true to enable FTC telemetry output
     */
    public static void setEnableFtcTelemetry(Telemetry telemetry, boolean enable) {
        ftcTelemetry = telemetry;
        enableFtcTelemetry = enable;
    }

    /**
     * Set maximum number of lines to keep in telemetry output.
     * Older messages will be replaced by newer ones.
     *
     * @param maxLines Maximum lines (default: 10)
     */
    public static void setMaxTelemetryLines(int maxLines) {
        maxTelemetryLines = Math.max(1, maxLines);
    }

    // ==================== Group Configuration ====================

    /**
     * Set the minimum log level for a specific group.
     * Overrides the global level for this group only.
     *
     * @param group Log group (INDEX, DRIVE, etc.)
     * @param level Minimum level for this group
     */
    public static void setGroupLevel(String group, LogLevel level) {
        groupLevels.put(group, level);
    }

    public static void setGroupLevel(LogGroup group, LogLevel level) {
        setGroupLevel(group.getTag(), level);
    }

    /**
     * Enable or disable a log group entirely.
     * When disabled, no logs from this group will be emitted regardless of level.
     *
     * @param group Log group
     * @param enabled true to enable, false to disable
     */
    public static void setGroupEnabled(String group, boolean enabled) {
        groupEnabled.put(group, enabled);
    }

    public static void setGroupEnabled(LogGroup group, boolean enabled) {
        setGroupEnabled(group.getTag(), enabled);
    }

    /**
     * Check if a group is enabled.
     */
    public static boolean isGroupEnabled(String group) {
        return groupEnabled.getOrDefault(group, true);
    }

    /**
     * Clear all group-specific settings.
     */
    public static void clearGroupSettings() {
        groupLevels.clear();
        groupEnabled.clear();
    }

    // ==================== Core Logging Methods ====================

    /**
     * TRACE level log (most verbose).
     */
    public static void t(String group, String message) {
        log(LogLevel.TRACE, group, message);
    }

    public static void t(LogGroup group, String message) {
        log(LogLevel.TRACE, group.getTag(), message);
    }

    public static void t(String group, String format, Object... args) {
        log(LogLevel.TRACE, group, format, args);
    }

    public static void t(LogGroup group, String format, Object... args) {
        log(LogLevel.TRACE, group.getTag(), format, args);
    }

    /**
     * DEBUG level log.
     */
    public static void d(String group, String message) {
        log(LogLevel.DEBUG, group, message);
    }

    public static void d(LogGroup group, String message) {
        log(LogLevel.DEBUG, group.getTag(), message);
    }

    public static void d(String group, String format, Object... args) {
        log(LogLevel.DEBUG, group, format, args);
    }

    public static void d(LogGroup group, String format, Object... args) {
        log(LogLevel.DEBUG, group.getTag(), format, args);
    }

    /**
     * INFO level log.
     */
    public static void i(String group, String message) {
        log(LogLevel.INFO, group, message);
    }

    public static void i(LogGroup group, String message) {
        log(LogLevel.INFO, group.getTag(), message);
    }

    public static void i(String group, String format, Object... args) {
        log(LogLevel.INFO, group, format, args);
    }

    public static void i(LogGroup group, String format, Object... args) {
        log(LogLevel.INFO, group.getTag(), format, args);
    }

    /**
     * WARN level log.
     */
    public static void w(String group, String message) {
        log(LogLevel.WARN, group, message);
    }

    public static void w(LogGroup group, String message) {
        log(LogLevel.WARN, group.getTag(), message);
    }

    public static void w(String group, String format, Object... args) {
        log(LogLevel.WARN, group, format, args);
    }

    public static void w(LogGroup group, String format, Object... args) {
        log(LogLevel.WARN, group.getTag(), format, args);
    }

    /**
     * ERROR level log (highest severity).
     */
    public static void e(String group, String message) {
        log(LogLevel.ERROR, group, message);
    }

    public static void e(LogGroup group, String message) {
        log(LogLevel.ERROR, group.getTag(), message);
    }

    public static void e(String group, String format, Object... args) {
        log(LogLevel.ERROR, group, format, args);
    }

    public static void e(LogGroup group, String format, Object... args) {
        log(LogLevel.ERROR, group.getTag(), format, args);
    }

    /**
     * Log an exception with a message.
     */
    public static void e(String group, Throwable throwable, String message) {
        if (shouldLog(LogLevel.ERROR, group)) {
            String fullMessage = buildPrefix(LogLevel.ERROR, group) + message;
            RobotLog.ee(ROBOT_LOG_TAG, throwable, fullMessage);
        }
    }

    public static void e(LogGroup group, Throwable throwable, String message) {
        e(group.getTag(), throwable, message);
    }

    // ==================== Spam Control ====================

    /**
     * Log at most once per specified period (in milliseconds).
     * Useful for high-frequency updates that would spam the log.
     *
     * Example: Log intake status at most once per second
     * <pre>
     * Dbg.everyMs(LogGroup.INTAKE, LogLevel.INFO, "status", 1000,
     *             "Count: %d, State: %s", count, state);
     * </pre>
     *
     * @param group Log group
     * @param level Log level
     * @param key Unique key for this log site (use meaningful name)
     * @param periodMs Minimum milliseconds between logs
     * @param format Format string
     * @param args Format arguments
     */
    public static void everyMs(String group, LogLevel level, String key, long periodMs,
                               String format, Object... args) {
        if (!shouldLog(level, group)) return;

        String compositeKey = group + ":" + key;
        AtomicLong last = lastLogTime.computeIfAbsent(compositeKey, k -> new AtomicLong(0));

        long now = System.currentTimeMillis();
        long lastTime = last.get();

        if (now - lastTime >= periodMs) {
            if (last.compareAndSet(lastTime, now)) {
                log(level, group, format, args);
            }
        }
    }

    public static void everyMs(LogGroup group, LogLevel level, String key, long periodMs,
                               String format, Object... args) {
        everyMs(group.getTag(), level, key, periodMs, format, args);
    }

    /**
     * Log only once per OpMode run.
     * Useful for deprecation warnings or one-time initialization messages.
     *
     * Example:
     * <pre>
     * Dbg.once(LogGroup.AUTO, LogLevel.WARN, "oldPath",
     *          "Using deprecated autonomous path");
     * </pre>
     *
     * @param group Log group
     * @param level Log level
     * @param key Unique key for this log site
     * @param format Format string
     * @param args Format arguments
     */
    public static void once(String group, LogLevel level, String key,
                            String format, Object... args) {
        if (!shouldLog(level, group)) return;

        String compositeKey = group + ":" + key;
        if (onceKeys.putIfAbsent(compositeKey, Boolean.TRUE) == null) {
            log(level, group, format, args);
        }
    }

    public static void once(LogGroup group, LogLevel level, String key,
                            String format, Object... args) {
        once(group.getTag(), level, key, format, args);
    }

    /**
     * Log once every N calls.
     * Useful for sampling high-frequency events.
     *
     * Example: Log every 10th sensor reading
     * <pre>
     * Dbg.counted(LogGroup.SENSORS, LogLevel.DEBUG, "distance", 10,
     *             "Distance: %.1f mm", distance);
     * </pre>
     *
     * @param group Log group
     * @param level Log level
     * @param key Unique key for this log site
     * @param everyN Log on every Nth call (1 = every call, 10 = every 10th)
     * @param format Format string
     * @param args Format arguments
     */
    public static void counted(String group, LogLevel level, String key, int everyN,
                               String format, Object... args) {
        if (!shouldLog(level, group)) return;
        if (everyN <= 0) return;

        String compositeKey = group + ":" + key;
        AtomicInteger counter = countKeys.computeIfAbsent(compositeKey, k -> new AtomicInteger(0));

        int count = counter.incrementAndGet();
        if (count % everyN == 0) {
            log(level, group, format, args);
        }
    }

    public static void counted(LogGroup group, LogLevel level, String key, int everyN,
                               String format, Object... args) {
        counted(group.getTag(), level, key, everyN, format, args);
    }

    // ==================== Internal Implementation ====================

    /**
     * Check if a log at this level and group should be emitted.
     * Fast-path: returns false immediately if filtered out (zero allocation).
     */
    private static boolean shouldLog(LogLevel level, String group) {
        // Check if group is explicitly disabled
        if (!isGroupEnabled(group)) {
            return false;
        }

        // Check group-specific level, fall back to global
        LogLevel minLevel = groupLevels.getOrDefault(group, globalLevel);
        return level.shouldLog(minLevel);
    }

    /**
     * Core logging method - all public methods route here.
     */
    private static void log(LogLevel level, String group, String format, Object... args) {
        if (!shouldLog(level, group)) {
            return; // Fast exit - no string formatting
        }

        String message = (args.length > 0)
            ? String.format(Locale.US, format, args)
            : format;

        String fullMessage = buildPrefix(level, group) + message;

        // Emit to FTC RobotLog
        emitToRobotLog(level, fullMessage);
    }

    /**
     * Build the log prefix based on current configuration.
     * Format: [PREFIX][OPMODE:PHASE][LOOP][TIMESTAMP] GROUP/LEVEL:
     */
    private static String buildPrefix(LogLevel level, String group) {
        StringBuilder sb = new StringBuilder(64);

        if (!globalPrefix.isEmpty()) {
            sb.append('[').append(globalPrefix).append(']');
        }

        if (!contextOpMode.isEmpty() || !contextPhase.isEmpty()) {
            sb.append('[');
            if (!contextOpMode.isEmpty()) {
                sb.append(contextOpMode);
            }
            if (!contextPhase.isEmpty()) {
                if (!contextOpMode.isEmpty()) sb.append(':');
                sb.append(contextPhase);
            }
            sb.append(']');
        }

        if (includeLoopCount) {
            sb.append('[').append(loopCount).append(']');
        }

        if (includeTimestamp) {
            sb.append('[').append(System.currentTimeMillis()).append(']');
        }

        sb.append(' ').append(group).append('/').append(level.getShortName()).append(": ");

        return sb.toString();
    }

    /**
     * Emit to FTC RobotLog based on level.
     * Uses appropriate RobotLog method for each level.
     * Also sends to telemetry if enabled (INFO and above only).
     */
    private static void emitToRobotLog(LogLevel level, String message) {
        // Always emit to RobotLog
        switch (level) {
            case TRACE:
            case DEBUG:
                RobotLog.dd(ROBOT_LOG_TAG, message);
                break;
            case INFO:
                RobotLog.ii(ROBOT_LOG_TAG, message);
                break;
            case WARN:
                RobotLog.ww(ROBOT_LOG_TAG, message);
                break;
            case ERROR:
                RobotLog.ee(ROBOT_LOG_TAG, message);
                break;
        }

        // Also emit to telemetry if enabled (INFO and above only)
        if (level.getPriority() >= LogLevel.INFO.getPriority()) {
            emitToTelemetry(level, message);
        }
    }

    /**
     * Emit to telemetry outputs (Panels and/or FTC).
     * Only called for INFO, WARN, and ERROR levels.
     */
    private static void emitToTelemetry(LogLevel level, String message) {
        // Panels telemetry
        if (enablePanelsTelemetry) {
            try {
                // Get Panels telemetry instance and add data
                TelemetryManager panelsTelem = PanelsTelemetry.INSTANCE.getTelemetry();
                if (panelsTelem != null) {
                    panelsTelem.addData(level.getShortName(), message);
                    panelsTelem.update();
                }
            } catch (Exception e) {
                // Silently fail if Panels not available
            }
        }

        // FTC telemetry
        if (enableFtcTelemetry && ftcTelemetry != null) {
            try {
                // Store in map to limit lines
                String key = level.getShortName() + "_" + System.currentTimeMillis();
                lastTelemetryMessages.put(key, message);

                // Limit to max lines
                if (lastTelemetryMessages.size() > maxTelemetryLines) {
                    // Remove oldest entry
                    String oldestKey = lastTelemetryMessages.keySet().iterator().next();
                    lastTelemetryMessages.remove(oldestKey);
                }

                // Add all messages to telemetry
                ftcTelemetry.addLine("=== DEBUG LOGS ===");
                for (String msg : lastTelemetryMessages.values()) {
                    ftcTelemetry.addLine(msg);
                }
            } catch (Exception e) {
                // Silently fail if telemetry not available
            }
        }
    }

    // ==================== Utility Methods ====================

    /**
     * Print current configuration to log (for debugging the logger itself).
     */
    public static void dumpConfig() {
        RobotLog.ii(ROBOT_LOG_TAG, "=== Dbg Configuration ===");
        RobotLog.ii(ROBOT_LOG_TAG, "Global Level: " + globalLevel);
        RobotLog.ii(ROBOT_LOG_TAG, "Global Prefix: '" + globalPrefix + "'");
        RobotLog.ii(ROBOT_LOG_TAG, "Context: " + contextOpMode + ":" + contextPhase);
        RobotLog.ii(ROBOT_LOG_TAG, "Include Timestamp: " + includeTimestamp);
        RobotLog.ii(ROBOT_LOG_TAG, "Include Loop Count: " + includeLoopCount);
        RobotLog.ii(ROBOT_LOG_TAG, "Current Loop: " + loopCount);
        RobotLog.ii(ROBOT_LOG_TAG, "Panels Telemetry: " + enablePanelsTelemetry);
        RobotLog.ii(ROBOT_LOG_TAG, "FTC Telemetry: " + enableFtcTelemetry);
        RobotLog.ii(ROBOT_LOG_TAG, "Max Telemetry Lines: " + maxTelemetryLines);

        if (!groupLevels.isEmpty()) {
            RobotLog.ii(ROBOT_LOG_TAG, "Group Levels:");
            for (String group : groupLevels.keySet()) {
                RobotLog.ii(ROBOT_LOG_TAG, "  " + group + " -> " + groupLevels.get(group));
            }
        }

        if (!groupEnabled.isEmpty()) {
            RobotLog.ii(ROBOT_LOG_TAG, "Group Enabled:");
            for (String group : groupEnabled.keySet()) {
                RobotLog.ii(ROBOT_LOG_TAG, "  " + group + " -> " + groupEnabled.get(group));
            }
        }

        RobotLog.ii(ROBOT_LOG_TAG, "========================");
    }

    /**
     * Helper to bind gamepad toggles for common logging controls.
     * Call this in your OpMode loop to allow runtime control.
     *
     * Example usage:
     * <pre>
     * if (gamepad1.back) {
     *     Dbg.toggleLoggingControls(gamepad1);
     * }
     * </pre>
     *
     * Controls:
     * - dpad_up: Increase global level
     * - dpad_down: Decrease global level
     * - left_bumper: Toggle INDEX group
     * - right_bumper: Toggle DRIVE group
     *
     * NOTE: Implement your own edge detection to avoid multiple toggles!
     */
    public static String getControlsHelp() {
        return "Dbg Controls: " +
               "dpad_up/down=level | " +
               "LB=toggle INDEX | " +
               "RB=toggle DRIVE";
    }

    // Private constructor to prevent instantiation
    private Dbg() {}
}
