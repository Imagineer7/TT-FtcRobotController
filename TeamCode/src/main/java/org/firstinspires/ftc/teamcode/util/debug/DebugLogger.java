package org.firstinspires.ftc.teamcode.util.debug;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.List;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Comprehensive debug logging system for tracing state machines and boolean conditions
 * 
 * Features:
 * - Timestamped log entries
 * - Multiple log levels (DEBUG, INFO, WARNING, ERROR)
 * - Boolean condition trees for hierarchical analysis
 * - Filtering by level and category
 * - Multiple display formats
 * - Telemetry integration
 */
public class DebugLogger {
    
    public enum LogLevel {
        DEBUG(0, "🔍"),
        INFO(1, "ℹ️"),
        WARNING(2, "⚠️"),
        ERROR(3, "❌");
        
        private final int priority;
        private final String icon;
        
        LogLevel(int priority, String icon) {
            this.priority = priority;
            this.icon = icon;
        }
        
        public int getPriority() { return priority; }
        public String getIcon() { return icon; }
    }
    
    public enum DisplayMode {
        FULL,           // Show all logs
        SUMMARY,        // Show only warnings and errors
        BOOLEAN_TREE,   // Show only boolean condition checks
        RECENT          // Show only last N entries
    }
    
    private static class LogEntry {
        final long timestamp;
        final LogLevel level;
        final String category;
        final String message;
        final String details;
        
        LogEntry(LogLevel level, String category, String message, String details) {
            this.timestamp = System.currentTimeMillis();
            this.level = level;
            this.category = category;
            this.message = message;
            this.details = details;
        }
        
        String getFormattedTime(long baseTime) {
            long elapsed = timestamp - baseTime;
            return String.format("%.3fs", elapsed / 1000.0);
        }
    }
    
    private final List<LogEntry> logs = new ArrayList<>();
    private final Map<String, BooleanCheck> booleanChecks = new LinkedHashMap<>();
    private final Map<String, BooleanTree> booleanTrees = new LinkedHashMap<>();
    private final long startTime;
    private LogLevel minLevel = LogLevel.DEBUG;
    private DisplayMode displayMode = DisplayMode.FULL;
    private int maxRecentEntries = 15;
    private String categoryFilter = null;
    
    public DebugLogger() {
        this.startTime = System.currentTimeMillis();
    }
    
    // === Logging Methods ===
    
    public void debug(String category, String message) {
        log(LogLevel.DEBUG, category, message, null);
    }
    
    public void debug(String category, String message, String details) {
        log(LogLevel.DEBUG, category, message, details);
    }
    
    public void info(String category, String message) {
        log(LogLevel.INFO, category, message, null);
    }
    
    public void info(String category, String message, String details) {
        log(LogLevel.INFO, category, message, details);
    }
    
    public void warning(String category, String message) {
        log(LogLevel.WARNING, category, message, null);
    }
    
    public void warning(String category, String message, String details) {
        log(LogLevel.WARNING, category, message, details);
    }
    
    public void error(String category, String message) {
        log(LogLevel.ERROR, category, message, null);
    }
    
    public void error(String category, String message, String details) {
        log(LogLevel.ERROR, category, message, details);
    }
    
    private void log(LogLevel level, String category, String message, String details) {
        LogEntry entry = new LogEntry(level, category, message, details);
        logs.add(entry);
        
        // Also print to System.out for console debugging
        System.out.println(String.format("[%s] %s [%s] %s%s",
            entry.getFormattedTime(startTime),
            level.getIcon(),
            category,
            message,
            details != null ? " | " + details : ""));
    }
    
    // === Boolean Condition Tracking ===
    
    /**
     * Register a boolean check for tracking
     */
    public void registerCheck(String checkId, String description) {
        booleanChecks.put(checkId, new BooleanCheck(checkId, description));
    }
    
    /**
     * Update a boolean check value and log it
     */
    public void updateCheck(String checkId, boolean value) {
        BooleanCheck check = booleanChecks.get(checkId);
        if (check != null) {
            check.update(value);
            debug("BOOL_CHECK", checkId + " = " + value);
        }
    }
    
    /**
     * Update a boolean check with reason
     */
    public void updateCheck(String checkId, boolean value, String reason) {
        BooleanCheck check = booleanChecks.get(checkId);
        if (check != null) {
            check.update(value, reason);
            debug("BOOL_CHECK", checkId + " = " + value, reason);
        }
    }
    
    /**
     * Log a compound boolean condition breakdown
     */
    public void logBooleanTree(String treeName, Map<String, Boolean> conditions, boolean finalResult) {
        info("BOOL_TREE", treeName + " = " + finalResult);
        
        // Store the tree for expanded display
        BooleanTree tree = new BooleanTree(treeName, conditions, finalResult);
        booleanTrees.put(treeName, tree);
        
        for (Map.Entry<String, Boolean> entry : conditions.entrySet()) {
            String symbol = entry.getValue() ? "✓" : "✗";
            debug("BOOL_TREE", "  " + symbol + " " + entry.getKey() + " = " + entry.getValue());
        }
    }
    
    // === Configuration ===
    
    public void setMinLevel(LogLevel level) {
        this.minLevel = level;
    }
    
    public void setDisplayMode(DisplayMode mode) {
        this.displayMode = mode;
    }
    
    public DisplayMode getDisplayMode() {
        return this.displayMode;
    }
    
    public void setMaxRecentEntries(int max) {
        this.maxRecentEntries = max;
    }
    
    public void setCategoryFilter(String category) {
        this.categoryFilter = category;
    }
    
    public void clearFilter() {
        this.categoryFilter = null;
    }
    
    // === Display Methods ===
    
    /**
     * Output logs to telemetry
     */
    public void displayOnTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== DEBUG LOG ===");
        telemetry.addData("Mode", displayMode.toString());
        telemetry.addData("Entries", logs.size());
        telemetry.addLine("");
        
        List<LogEntry> filteredLogs = getFilteredLogs();
        
        switch (displayMode) {
            case FULL:
                displayFullLogs(telemetry, filteredLogs);
                break;
            case SUMMARY:
                displaySummary(telemetry, filteredLogs);
                break;
            case BOOLEAN_TREE:
                displayBooleanTree(telemetry);
                break;
            case RECENT:
                displayRecent(telemetry, filteredLogs);
                break;
        }
    }
    
    private List<LogEntry> getFilteredLogs() {
        List<LogEntry> filtered = new ArrayList<>();
        for (LogEntry entry : logs) {
            if (entry.level.getPriority() >= minLevel.getPriority()) {
                if (categoryFilter == null || entry.category.equals(categoryFilter)) {
                    filtered.add(entry);
                }
            }
        }
        return filtered;
    }
    
    private void displayFullLogs(Telemetry telemetry, List<LogEntry> logs) {
        int start = Math.max(0, logs.size() - maxRecentEntries);
        for (int i = start; i < logs.size(); i++) {
            LogEntry entry = logs.get(i);
            telemetry.addLine(formatLogEntry(entry));
        }
    }
    
    private void displaySummary(Telemetry telemetry, List<LogEntry> logs) {
        int warnings = 0;
        int errors = 0;
        
        for (LogEntry entry : logs) {
            if (entry.level == LogLevel.WARNING) warnings++;
            if (entry.level == LogLevel.ERROR) errors++;
        }
        
        telemetry.addData("Warnings", warnings);
        telemetry.addData("Errors", errors);
        telemetry.addLine("");
        
        // Show recent warnings and errors
        for (int i = logs.size() - 1; i >= 0 && (warnings > 0 || errors > 0); i--) {
            LogEntry entry = logs.get(i);
            if (entry.level == LogLevel.WARNING || entry.level == LogLevel.ERROR) {
                telemetry.addLine(formatLogEntry(entry));
            }
        }
    }
    
    private void displayBooleanTree(Telemetry telemetry) {
        telemetry.addLine("=== BOOLEAN CHECKS ===");
        
        // First show individual boolean checks
        for (BooleanCheck check : booleanChecks.values()) {
            String status = check.getCurrentValue() ? "✓ TRUE" : "✗ FALSE";
            String icon = check.getCurrentValue() ? "✓" : "✗";
            telemetry.addData(icon + " " + check.getDescription(), 
                check.getCurrentValue() ? "TRUE" : "FALSE");
            
            // Expand false checks with reason
            if (!check.getCurrentValue() && check.getReason() != null) {
                telemetry.addData("  ↳ Reason", check.getReason());
            }
        }
        
        // Show boolean trees with expanded false branches
        if (!booleanTrees.isEmpty()) {
            telemetry.addLine("");
            telemetry.addLine("=== CONDITION TREES ===");
            
            for (BooleanTree tree : booleanTrees.values()) {
                String treeIcon = tree.getFinalResult() ? "✓" : "✗";
                telemetry.addData(treeIcon + " " + tree.getName(), 
                    tree.getFinalResult() ? "TRUE" : "FALSE");
                
                // Always expand false trees, optionally expand true trees
                if (!tree.getFinalResult() || displayMode == DisplayMode.FULL) {
                    for (Map.Entry<String, Boolean> condition : tree.getConditions().entrySet()) {
                        String condIcon = condition.getValue() ? "  ✓" : "  ✗";
                        telemetry.addData(condIcon + " " + condition.getKey(), 
                            condition.getValue() ? "true" : "false");
                        
                        // For false conditions, show expanded details from BooleanCheck
                        if (!condition.getValue()) {
                            BooleanCheck check = booleanChecks.get(condition.getKey());
                            if (check != null && check.getReason() != null) {
                                telemetry.addData("    ↳", check.getReason());
                            }
                        }
                    }
                }
            }
        }
    }
    
    private void displayRecent(Telemetry telemetry, List<LogEntry> logs) {
        int start = Math.max(0, logs.size() - maxRecentEntries);
        for (int i = start; i < logs.size(); i++) {
            telemetry.addLine(formatLogEntry(logs.get(i)));
        }
    }
    
    private String formatLogEntry(LogEntry entry) {
        String base = String.format("[%s] %s %s",
            entry.getFormattedTime(startTime),
            entry.level.getIcon(),
            entry.message);
        if (entry.details != null) {
            base += "\n    " + entry.details;
        }
        return base;
    }
    
    // === Status Methods ===
    
    public int getLogCount() {
        return logs.size();
    }
    
    public int getErrorCount() {
        int count = 0;
        for (LogEntry entry : logs) {
            if (entry.level == LogLevel.ERROR) count++;
        }
        return count;
    }
    
    public int getWarningCount() {
        int count = 0;
        for (LogEntry entry : logs) {
            if (entry.level == LogLevel.WARNING) count++;
        }
        return count;
    }
    
    public void clear() {
        logs.clear();
        booleanChecks.clear();
        booleanTrees.clear();
    }
    
    // === Boolean Tree Class ===
    
    private static class BooleanTree {
        private final String name;
        private final Map<String, Boolean> conditions;
        private final boolean finalResult;
        private final long timestamp;
        
        BooleanTree(String name, Map<String, Boolean> conditions, boolean finalResult) {
            this.name = name;
            this.conditions = new LinkedHashMap<>(conditions);
            this.finalResult = finalResult;
            this.timestamp = System.currentTimeMillis();
        }
        
        String getName() { return name; }
        Map<String, Boolean> getConditions() { return conditions; }
        boolean getFinalResult() { return finalResult; }
        long getTimestamp() { return timestamp; }
    }
    
    // === Boolean Check Class ===
    
    private static class BooleanCheck {
        private final String id;
        private final String description;
        private boolean currentValue;
        private String reason;
        private long lastUpdateTime;
        
        BooleanCheck(String id, String description) {
            this.id = id;
            this.description = description;
            this.currentValue = false;
            this.lastUpdateTime = System.currentTimeMillis();
        }
        
        void update(boolean value) {
            this.currentValue = value;
            this.lastUpdateTime = System.currentTimeMillis();
        }
        
        void update(boolean value, String reason) {
            this.currentValue = value;
            this.reason = reason;
            this.lastUpdateTime = System.currentTimeMillis();
        }
        
        String getId() { return id; }
        String getDescription() { return description; }
        boolean getCurrentValue() { return currentValue; }
        String getReason() { return reason; }
        long getLastUpdateTime() { return lastUpdateTime; }
    }
}
