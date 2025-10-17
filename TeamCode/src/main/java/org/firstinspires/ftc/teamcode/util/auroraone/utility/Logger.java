package org.firstinspires.ftc.teamcode.util.auroraone.utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;
import org.firstinspires.ftc.teamcode.util.auroraone.core.Blackboard;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.List;
import java.util.Queue;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * AURORA ONE - Advanced Logging System
 *
 * This class is responsible for logging events, errors, and other significant occurrences within the robot's operation.
 * It provides methods to record log entries, categorize them by severity, and store them for later review.
 * It includes functionality to display logs in real-time, export them for analysis, and integrate with the Blackboard system.
 *
 * Features:
 * - Multi-level logging (TRACE, DEBUG, INFO, WARN, ERROR, FATAL)
 * - Thread-safe concurrent logging
 * - Integration with FTC Telemetry system
 * - Blackboard integration for system-wide log monitoring
 * - Performance metrics and statistics
 * - Configurable log retention and filtering
 * - Real-time log streaming
 * - Subsystem-specific logging channels
 * - Log export and analysis capabilities
 */
public class Logger {

    // Singleton instance
    private static volatile Logger instance;
    private static final Object lock = new Object();

    // Log levels
    public enum LogLevel {
        TRACE(0, "TRACE", "🔍"),
        DEBUG(1, "DEBUG", "🐛"),
        INFO(2, "INFO", "ℹ️"),
        WARN(3, "WARN", "⚠️"),
        ERROR(4, "ERROR", "❌"),
        FATAL(5, "FATAL", "💀");

        private final int priority;
        private final String name;
        private final String emoji;

        LogLevel(int priority, String name, String emoji) {
            this.priority = priority;
            this.name = name;
            this.emoji = emoji;
        }

        public int getPriority() { return priority; }
        public String getName() { return name; }
        public String getEmoji() { return emoji; }
    }

    // Core logging components
    private final Queue<LogEntry> logBuffer;
    private final CopyOnWriteArrayList<LogListener> listeners;
    private final ConcurrentHashMap<String, SubsystemLogger> subsystemLoggers;
    private final AtomicLong logCounter;

    // Configuration
    private volatile LogLevel currentLogLevel = LogLevel.INFO;
    private volatile int maxBufferSize = 1000;
    private volatile boolean enableTelemetryOutput = true;
    private volatile boolean enableBlackboardIntegration = true;
    private volatile boolean enablePerformanceLogging = false;

    // Blackboard and Telemetry integration
    private Blackboard blackboard;
    private Telemetry telemetry;

    // Performance tracking
    private final ConcurrentHashMap<LogLevel, AtomicLong> levelCounts;
    private final ConcurrentHashMap<String, AtomicLong> subsystemCounts;
    private final SimpleDateFormat timestampFormat;
    private long sessionStartTime;

    /**
     * Private constructor for singleton pattern
     */
    private Logger() {
        this.logBuffer = new ConcurrentLinkedQueue<>();
        this.listeners = new CopyOnWriteArrayList<>();
        this.subsystemLoggers = new ConcurrentHashMap<>();
        this.logCounter = new AtomicLong(0);
        this.levelCounts = new ConcurrentHashMap<>();
        this.subsystemCounts = new ConcurrentHashMap<>();
        this.timestampFormat = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US);
        this.sessionStartTime = System.currentTimeMillis();

        // Initialize level counters
        for (LogLevel level : LogLevel.values()) {
            levelCounts.put(level, new AtomicLong(0));
        }

        // Get Blackboard instance if available
        try {
            this.blackboard = Blackboard.getInstance();
        } catch (Exception e) {
            // Blackboard not available during early initialization
            this.blackboard = null;
        }

        info("Logger", "Aurora One Logger initialized");
    }

    /**
     * Get singleton instance
     */
    public static Logger getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new Logger();
                }
            }
        }
        return instance;
    }

    /**
     * Set telemetry reference for output
     */
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        info("Logger", "Telemetry integration enabled");
    }

    // === PRIMARY LOGGING METHODS ===

    /**
     * Log trace level message
     */
    public void trace(String subsystem, String message) {
        log(LogLevel.TRACE, subsystem, message, null);
    }

    /**
     * Log debug level message
     */
    public void debug(String subsystem, String message) {
        log(LogLevel.DEBUG, subsystem, message, null);
    }

    /**
     * Log info level message
     */
    public void info(String subsystem, String message) {
        log(LogLevel.INFO, subsystem, message, null);
    }

    /**
     * Log warning level message
     */
    public void warn(String subsystem, String message) {
        log(LogLevel.WARN, subsystem, message, null);
    }

    /**
     * Log error level message
     */
    public void error(String subsystem, String message) {
        log(LogLevel.ERROR, subsystem, message, null);
    }

    /**
     * Log error level message with exception
     */
    public void error(String subsystem, String message, Throwable throwable) {
        log(LogLevel.ERROR, subsystem, message, throwable);
    }

    /**
     * Log fatal level message
     */
    public void fatal(String subsystem, String message) {
        log(LogLevel.FATAL, subsystem, message, null);
    }

    /**
     * Log fatal level message with exception
     */
    public void fatal(String subsystem, String message, Throwable throwable) {
        log(LogLevel.FATAL, subsystem, message, throwable);
    }

    // === ADVANCED LOGGING METHODS ===

    /**
     * Log formatted message
     */
    public void logf(LogLevel level, String subsystem, String format, Object... args) {
        try {
            String message = String.format(Locale.US, format, args);
            log(level, subsystem, message, null);
        } catch (Exception e) {
            // Fallback if formatting fails
            log(LogLevel.ERROR, "Logger", "Failed to format log message: " + format, e);
        }
    }

    /**
     * Log with custom timestamp
     */
    public void logWithTimestamp(LogLevel level, String subsystem, String message, long timestamp) {
        if (!shouldLog(level)) return;

        LogEntry entry = new LogEntry(
            logCounter.incrementAndGet(),
            timestamp,
            level,
            subsystem,
            message,
            null,
            Thread.currentThread().getName()
        );

        processLogEntry(entry);
    }

    /**
     * Log performance metrics
     */
    public void logPerformance(String subsystem, String operation, long durationMs) {
        if (enablePerformanceLogging) {
            logf(LogLevel.DEBUG, subsystem, "Performance: %s took %dms", operation, durationMs);

            // Store in Blackboard for monitoring
            if (blackboard != null) {
                blackboard.put("performance." + subsystem + "." + operation + ".duration", durationMs);
            }
        }
    }

    /**
     * Log system state change
     */
    public void logStateChange(String subsystem, String fromState, String toState) {
        info(subsystem, String.format("State change: %s → %s", fromState, toState));

        // Update Blackboard with state information
        if (blackboard != null) {
            blackboard.put("system.state." + subsystem + ".current", toState);
            blackboard.put("system.state." + subsystem + ".previous", fromState);
            blackboard.put("system.state." + subsystem + ".last_change", System.currentTimeMillis());
        }
    }

    // === CORE LOGGING IMPLEMENTATION ===

    /**
     * Main logging method
     */
    private void log(LogLevel level, String subsystem, String message, Throwable throwable) {
        if (!shouldLog(level)) return;

        LogEntry entry = new LogEntry(
            logCounter.incrementAndGet(),
            System.currentTimeMillis(),
            level,
            subsystem,
            message,
            throwable,
            Thread.currentThread().getName()
        );

        processLogEntry(entry);
    }

    /**
     * Check if log level should be processed
     */
    private boolean shouldLog(LogLevel level) {
        return level.getPriority() >= currentLogLevel.getPriority();
    }

    /**
     * Process and distribute log entry
     */
    private void processLogEntry(LogEntry entry) {
        // Add to buffer
        addToBuffer(entry);

        // Update statistics
        updateStatistics(entry);

        // Notify listeners
        notifyListeners(entry);

        // Output to telemetry if enabled
        if (enableTelemetryOutput && telemetry != null) {
            outputToTelemetry(entry);
        }

        // Update Blackboard if enabled
        if (enableBlackboardIntegration && blackboard != null) {
            updateBlackboard(entry);
        }
    }

    /**
     * Add entry to circular buffer
     */
    private void addToBuffer(LogEntry entry) {
        logBuffer.offer(entry);

        // Maintain buffer size
        while (logBuffer.size() > maxBufferSize) {
            logBuffer.poll();
        }
    }

    /**
     * Update logging statistics
     */
    private void updateStatistics(LogEntry entry) {
        levelCounts.get(entry.level).incrementAndGet();
        subsystemCounts.computeIfAbsent(entry.subsystem, k -> new AtomicLong(0)).incrementAndGet();
    }

    /**
     * Notify all registered listeners
     */
    private void notifyListeners(LogEntry entry) {
        for (LogListener listener : listeners) {
            try {
                listener.onLogEntry(entry);
            } catch (Exception e) {
                // Don't log errors in listener notification to avoid recursion
                System.err.println("Error in log listener: " + e.getMessage());
            }
        }
    }

    /**
     * Output log entry to FTC telemetry
     */
    private void outputToTelemetry(LogEntry entry) {
        if (entry.level.getPriority() >= LogLevel.INFO.getPriority()) {
            String telemetryMessage = String.format("[%s] %s: %s",
                entry.level.getEmoji(),
                entry.subsystem,
                entry.message);

            telemetry.addLine(telemetryMessage);
        }
    }

    /**
     * Update Blackboard with logging information
     */
    private void updateBlackboard(LogEntry entry) {
        // Update latest log entry info
        blackboard.put("logging.latest.level", entry.level.getName());
        blackboard.put("logging.latest.subsystem", entry.subsystem);
        blackboard.put("logging.latest.message", entry.message);
        blackboard.put("logging.latest.timestamp", entry.timestamp);

        // Update error counters
        if (entry.level == LogLevel.ERROR || entry.level == LogLevel.FATAL) {
            long errorCount = blackboard.get("logging.stats.error_count", 0L);
            blackboard.put("logging.stats.error_count", errorCount + 1);
        }

        // Update subsystem activity
        blackboard.put("logging.activity." + entry.subsystem, entry.timestamp);
    }

    // === SUBSYSTEM LOGGER FACTORY ===

    /**
     * Get or create a subsystem-specific logger
     */
    public SubsystemLogger getSubsystemLogger(String subsystem) {
        return subsystemLoggers.computeIfAbsent(subsystem, SubsystemLogger::new);
    }

    /**
     * Subsystem-specific logger wrapper
     */
    public class SubsystemLogger {
        private final String subsystem;

        private SubsystemLogger(String subsystem) {
            this.subsystem = subsystem;
        }

        public void trace(String message) { Logger.this.trace(subsystem, message); }
        public void debug(String message) { Logger.this.debug(subsystem, message); }
        public void info(String message) { Logger.this.info(subsystem, message); }
        public void warn(String message) { Logger.this.warn(subsystem, message); }
        public void error(String message) { Logger.this.error(subsystem, message); }
        public void error(String message, Throwable throwable) { Logger.this.error(subsystem, message, throwable); }
        public void fatal(String message) { Logger.this.fatal(subsystem, message); }
        public void fatal(String message, Throwable throwable) { Logger.this.fatal(subsystem, message, throwable); }

        public void logf(LogLevel level, String format, Object... args) {
            Logger.this.logf(level, subsystem, format, args);
        }

        public void performance(String operation, long durationMs) {
            Logger.this.logPerformance(subsystem, operation, durationMs);
        }

        public void stateChange(String fromState, String toState) {
            Logger.this.logStateChange(subsystem, fromState, toState);
        }
    }

    // === LISTENER INTERFACE ===

    /**
     * Interface for log entry listeners
     */
    public interface LogListener {
        void onLogEntry(LogEntry entry);
    }

    /**
     * Add log listener
     */
    public void addListener(LogListener listener) {
        listeners.add(listener);
    }

    /**
     * Remove log listener
     */
    public void removeListener(LogListener listener) {
        listeners.remove(listener);
    }

    // === CONFIGURATION METHODS ===

    /**
     * Set minimum log level
     */
    public void setLogLevel(LogLevel level) {
        this.currentLogLevel = level;
        info("Logger", "Log level set to " + level.getName());
    }

    /**
     * Set maximum buffer size
     */
    public void setMaxBufferSize(int size) {
        this.maxBufferSize = Math.max(100, size);
        info("Logger", "Buffer size set to " + this.maxBufferSize);
    }

    /**
     * Enable/disable telemetry output
     */
    public void setTelemetryOutput(boolean enabled) {
        this.enableTelemetryOutput = enabled;
    }

    /**
     * Enable/disable Blackboard integration
     */
    public void setBlackboardIntegration(boolean enabled) {
        this.enableBlackboardIntegration = enabled;
    }

    /**
     * Enable/disable performance logging
     */
    public void setPerformanceLogging(boolean enabled) {
        this.enablePerformanceLogging = enabled;
        info("Logger", "Performance logging " + (enabled ? "enabled" : "disabled"));
    }

    // === QUERY AND ANALYSIS METHODS ===

    /**
     * Get recent log entries
     */
    public List<LogEntry> getRecentEntries(int count) {
        List<LogEntry> entries = new CopyOnWriteArrayList<>(logBuffer);
        int size = entries.size();
        if (count >= size) {
            return entries;
        }
        return entries.subList(size - count, size);
    }

    /**
     * Get entries by log level
     */
    public List<LogEntry> getEntriesByLevel(LogLevel level) {
        return logBuffer.stream()
            .filter(entry -> entry.level == level)
            .collect(java.util.stream.Collectors.toList());
    }

    /**
     * Get entries by subsystem
     */
    public List<LogEntry> getEntriesBySubsystem(String subsystem) {
        return logBuffer.stream()
            .filter(entry -> entry.subsystem.equals(subsystem))
            .collect(java.util.stream.Collectors.toList());
    }

    /**
     * Get logging statistics
     */
    public LoggingStats getStats() {
        return new LoggingStats(
            logCounter.get(),
            System.currentTimeMillis() - sessionStartTime,
            new ConcurrentHashMap<>(levelCounts),
            new ConcurrentHashMap<>(subsystemCounts),
            logBuffer.size(),
            maxBufferSize
        );
    }

    /**
     * Clear log buffer
     */
    public void clearBuffer() {
        logBuffer.clear();
        info("Logger", "Log buffer cleared");
    }

    /**
     * Get system health summary based on log analysis
     */
    public String getHealthSummary() {
        long totalLogs = logCounter.get();
        long errorCount = levelCounts.get(LogLevel.ERROR).get();
        long fatalCount = levelCounts.get(LogLevel.FATAL).get();

        if (fatalCount > 0) {
            return "CRITICAL - Fatal errors detected";
        } else if (errorCount > totalLogs * 0.1) {
            return "WARNING - High error rate";
        } else if (errorCount > 0) {
            return "CAUTION - Some errors present";
        } else {
            return "HEALTHY - No critical issues";
        }
    }

    // === DATA CLASSES ===

    /**
     * Log entry data container
     */
    public static class LogEntry {
        public final long id;
        public final long timestamp;
        public final LogLevel level;
        public final String subsystem;
        public final String message;
        public final Throwable throwable;
        public final String threadName;

        public LogEntry(long id, long timestamp, LogLevel level, String subsystem,
                       String message, Throwable throwable, String threadName) {
            this.id = id;
            this.timestamp = timestamp;
            this.level = level;
            this.subsystem = subsystem;
            this.message = message;
            this.throwable = throwable;
            this.threadName = threadName;
        }

        public String getFormattedTimestamp() {
            return new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date(timestamp));
        }

        @Override
        public String toString() {
            String exceptionStr = throwable != null ? " [" + throwable.getClass().getSimpleName() + "]" : "";
            return String.format("[%s] %s %s: %s%s",
                getFormattedTimestamp(),
                level.getEmoji(),
                subsystem,
                message,
                exceptionStr);
        }
    }

    /**
     * Logging statistics container
     */
    public static class LoggingStats {
        public final long totalEntries;
        public final long sessionDurationMs;
        public final Map<LogLevel, AtomicLong> levelCounts;
        public final Map<String, AtomicLong> subsystemCounts;
        public final int bufferSize;
        public final int maxBufferSize;

        public LoggingStats(long totalEntries, long sessionDurationMs,
                           Map<LogLevel, AtomicLong> levelCounts,
                           Map<String, AtomicLong> subsystemCounts,
                           int bufferSize, int maxBufferSize) {
            this.totalEntries = totalEntries;
            this.sessionDurationMs = sessionDurationMs;
            this.levelCounts = levelCounts;
            this.subsystemCounts = subsystemCounts;
            this.bufferSize = bufferSize;
            this.maxBufferSize = maxBufferSize;
        }

        public double getLogsPerSecond() {
            return sessionDurationMs > 0 ? (double) totalEntries / (sessionDurationMs / 1000.0) : 0;
        }

        @Override
        public String toString() {
            return String.format("LoggingStats{total=%d, rate=%.1f/s, buffer=%d/%d}",
                totalEntries, getLogsPerSecond(), bufferSize, maxBufferSize);
        }
    }

    // === CONVENIENCE METHODS FOR COMMON SCENARIOS ===

    /**
     * Log subsystem initialization
     */
    public void logInit(String subsystem, boolean success) {
        if (success) {
            info(subsystem, "✅ Initialized successfully");
        } else {
            error(subsystem, "❌ Initialization failed");
        }
    }

    /**
     * Log method entry (for debugging)
     */
    public void logMethodEntry(String subsystem, String methodName) {
        if (currentLogLevel == LogLevel.TRACE) {
            trace(subsystem, "→ " + methodName);
        }
    }

    /**
     * Log method exit (for debugging)
     */
    public void logMethodExit(String subsystem, String methodName) {
        if (currentLogLevel == LogLevel.TRACE) {
            trace(subsystem, "← " + methodName);
        }
    }

    /**
     * Log configuration change
     */
    public void logConfigChange(String subsystem, String parameter, Object oldValue, Object newValue) {
        info(subsystem, String.format("Config: %s changed from %s to %s", parameter, oldValue, newValue));
    }
}
