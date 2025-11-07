package org.firstinspires.ftc.teamcode.util.aurora.lightning;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * EventLogger - Structured event logging with timestamps and metadata
 *
 * Features:
 * - Timestamped events with categories
 * - Event filtering and search
 * - Performance profiling
 * - Error tracking with context
 * - Circular buffer to prevent memory overflow
 */
public class EventLogger {

    /**
     * Event severity levels
     */
    public enum Level {
        DEBUG,      // Detailed information for debugging
        INFO,       // General information
        WARNING,    // Something unusual but not critical
        ERROR,      // Error condition
        CRITICAL    // Critical failure
    }

    /**
     * Event data class
     */
    public static class Event {
        public final long timestampNanos;
        public final double timestampSeconds;
        public final Level level;
        public final String category;
        public final String message;
        public final String metadata;

        public Event(double timestampSeconds, Level level, String category, String message, String metadata) {
            this.timestampNanos = System.nanoTime();
            this.timestampSeconds = timestampSeconds;
            this.level = level;
            this.category = category;
            this.message = message;
            this.metadata = metadata;
        }

        @Override
        public String toString() {
            String metaStr = metadata != null && !metadata.isEmpty() ? " | " + metadata : "";
            return String.format("[%.3fs] %s/%s: %s%s",
                timestampSeconds, level, category, message, metaStr);
        }
    }

    /**
     * Profiling event for measuring operation duration
     */
    public static class ProfileEvent {
        public final String operationName;
        public final long startTimeNanos;
        public final long endTimeNanos;
        public final double durationMs;

        public ProfileEvent(String operationName, long startTimeNanos, long endTimeNanos) {
            this.operationName = operationName;
            this.startTimeNanos = startTimeNanos;
            this.endTimeNanos = endTimeNanos;
            this.durationMs = (endTimeNanos - startTimeNanos) / 1_000_000.0;
        }

        @Override
        public String toString() {
            return String.format("%s: %.2f ms", operationName, durationMs);
        }
    }

    // Event storage
    private CircularDataBuffer<Event> events;
    private ElapsedTime runtime;

    // Profiling storage
    private List<ProfileEvent> profiles = new ArrayList<>();
    private java.util.Map<String, Long> activeProfiles = new java.util.HashMap<>();

    // Statistics
    private int totalEvents = 0;
    private int errorCount = 0;
    private int warningCount = 0;

    /**
     * Constructor
     * @param capacity Maximum number of events to store
     */
    public EventLogger(int capacity) {
        this.events = new CircularDataBuffer<>(capacity);
        this.runtime = new ElapsedTime();
        runtime.reset();
    }

    /**
     * Constructor with default capacity (500 events)
     */
    public EventLogger() {
        this(500);
    }

    /**
     * Log a debug event
     */
    public void debug(String category, String message) {
        log(Level.DEBUG, category, message, null);
    }

    /**
     * Log an info event
     */
    public void info(String category, String message) {
        log(Level.INFO, category, message, null);
    }

    /**
     * Log a warning event
     */
    public void warning(String category, String message) {
        log(Level.WARNING, category, message, null);
        warningCount++;
    }

    /**
     * Log an error event
     */
    public void error(String category, String message) {
        log(Level.ERROR, category, message, null);
        errorCount++;
    }

    /**
     * Log a critical event
     */
    public void critical(String category, String message) {
        log(Level.CRITICAL, category, message, null);
        errorCount++;
    }

    /**
     * Log an event with metadata
     */
    public void log(Level level, String category, String message, String metadata) {
        Event event = new Event(runtime.seconds(), level, category, message, metadata);
        events.add(event);
        totalEvents++;
    }

    /**
     * Log an event with formatted metadata
     */
    public void logf(Level level, String category, String message, String metadataFormat, Object... args) {
        String metadata = String.format(metadataFormat, args);
        log(level, category, message, metadata);
    }

    /**
     * Start profiling an operation
     */
    public void startProfile(String operationName) {
        activeProfiles.put(operationName, System.nanoTime());
    }

    /**
     * End profiling an operation and record duration
     */
    public void endProfile(String operationName) {
        Long startTime = activeProfiles.remove(operationName);
        if (startTime != null) {
            long endTime = System.nanoTime();
            ProfileEvent profile = new ProfileEvent(operationName, startTime, endTime);
            profiles.add(profile);

            // Also log as debug event
            debug("PROFILE", String.format("%s completed in %.2f ms", operationName, profile.durationMs));
        }
    }

    /**
     * Get all events
     */
    public List<Event> getAllEvents() {
        return events.toList();
    }

    /**
     * Get events by level
     */
    public List<Event> getEventsByLevel(Level level) {
        List<Event> result = new ArrayList<>();
        for (Event event : events.toList()) {
            if (event.level == level) {
                result.add(event);
            }
        }
        return result;
    }

    /**
     * Get events by category
     */
    public List<Event> getEventsByCategory(String category) {
        List<Event> result = new ArrayList<>();
        for (Event event : events.toList()) {
            if (event.category.equals(category)) {
                result.add(event);
            }
        }
        return result;
    }

    /**
     * Get recent events (last N)
     */
    public List<Event> getRecentEvents(int count) {
        List<Event> all = events.toList();
        int start = Math.max(0, all.size() - count);
        return all.subList(start, all.size());
    }

    /**
     * Get all errors
     */
    public List<Event> getErrors() {
        List<Event> result = new ArrayList<>();
        for (Event event : events.toList()) {
            if (event.level == Level.ERROR || event.level == Level.CRITICAL) {
                result.add(event);
            }
        }
        return result;
    }

    /**
     * Get all warnings
     */
    public List<Event> getWarnings() {
        List<Event> result = new ArrayList<>();
        for (Event event : events.toList()) {
            if (event.level == Level.WARNING) {
                result.add(event);
            }
        }
        return result;
    }

    /**
     * Get all profile events
     */
    public List<ProfileEvent> getProfiles() {
        return new ArrayList<>(profiles);
    }

    /**
     * Get average duration for a profiled operation
     */
    public double getAverageProfileDuration(String operationName) {
        double sum = 0;
        int count = 0;

        for (ProfileEvent profile : profiles) {
            if (profile.operationName.equals(operationName)) {
                sum += profile.durationMs;
                count++;
            }
        }

        return count > 0 ? sum / count : 0;
    }

    /**
     * Get total event count
     */
    public int getTotalEventCount() {
        return totalEvents;
    }

    /**
     * Get error count
     */
    public int getErrorCount() {
        return errorCount;
    }

    /**
     * Get warning count
     */
    public int getWarningCount() {
        return warningCount;
    }

    /**
     * Clear all events and profiles
     */
    public void clear() {
        events.clear();
        profiles.clear();
        totalEvents = 0;
        errorCount = 0;
        warningCount = 0;
    }

    /**
     * Export events to CSV format
     */
    public String exportToCSV() {
        StringBuilder csv = new StringBuilder();
        csv.append("Timestamp,Level,Category,Message,Metadata\n");

        for (Event event : events.toList()) {
            csv.append(String.format("%.3f,%s,%s,\"%s\",\"%s\"\n",
                event.timestampSeconds,
                event.level,
                event.category,
                event.message.replace("\"", "\"\""),  // Escape quotes
                event.metadata != null ? event.metadata.replace("\"", "\"\"") : ""
            ));
        }

        return csv.toString();
    }

    /**
     * Export events to readable text format
     */
    public String exportToText() {
        StringBuilder text = new StringBuilder();
        text.append("===== EVENT LOG =====\n");
        text.append(String.format("Total Events: %d (Errors: %d, Warnings: %d)\n\n",
            totalEvents, errorCount, warningCount));

        for (Event event : events.toList()) {
            text.append(event.toString()).append("\n");
        }

        return text.toString();
    }

    /**
     * Get summary statistics
     */
    public String getSummary() {
        return String.format(
            "Events: %d total (%d stored), Errors: %d, Warnings: %d, Profiles: %d operations",
            totalEvents, events.size(), errorCount, warningCount, profiles.size()
        );
    }
}

