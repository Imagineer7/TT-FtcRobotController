package org.firstinspires.ftc.teamcode.util.debug;

/**
 * Log severity levels for the Dbg logging system.
 * Ordered from most verbose (TRACE) to most severe (ERROR).
 */
public enum LogLevel {
    TRACE(0, "T"),
    DEBUG(1, "D"),
    INFO(2, "I"),
    WARN(3, "W"),
    ERROR(4, "E");

    private final int priority;
    private final String shortName;

    LogLevel(int priority, String shortName) {
        this.priority = priority;
        this.shortName = shortName;
    }

    public int getPriority() {
        return priority;
    }

    public String getShortName() {
        return shortName;
    }

    /**
     * Check if this level should be logged given a minimum level.
     */
    public boolean shouldLog(LogLevel minLevel) {
        return this.priority >= minLevel.priority;
    }
}
