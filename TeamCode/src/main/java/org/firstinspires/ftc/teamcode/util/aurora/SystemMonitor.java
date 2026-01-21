package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Simple static live variable monitoring system for debugging
 * Replaces complex DebugLogger with focused live state display
 * Includes System.out logging for remote monitoring
 */
@Deprecated
public class SystemMonitor {
    
    private static final Map<String, Object> liveVars = new LinkedHashMap<>();
    private static boolean enabled = true;
    private static boolean consoleLoggingEnabled = true;
    private static long lastConsoleLogTime = 0;
    private static final long CONSOLE_LOG_INTERVAL_MS = 250; // Log to console every 250ms for better analysis
    
    /**
     * Update a live variable value
     */
    public static void set(String key, Object value) {
        if (enabled) {
            liveVars.put(key, value);
        }
    }
    
    /**
     * Update multiple live variables at once
     */
    public static void setAll(Map<String, Object> vars) {
        if (enabled) {
            liveVars.putAll(vars);
            
            // Log to console at regular intervals for remote monitoring
            if (consoleLoggingEnabled) {
                long now = System.currentTimeMillis();
                if (now - lastConsoleLogTime >= CONSOLE_LOG_INTERVAL_MS) {
                    logToConsole();
                    lastConsoleLogTime = now;
                }
            }
        }
    }
    
    /**
     * Clear a specific variable
     */
    public static void clear(String key) {
        liveVars.remove(key);
    }
    
    /**
     * Clear all variables
     */
    public static void clearAll() {
        liveVars.clear();
    }
    
    /**
     * Enable or disable monitoring
     */
    public static void setEnabled(boolean enabled) {
        SystemMonitor.enabled = enabled;
    }
    
    /**
     * Enable or disable console logging
     */
    public static void setConsoleLoggingEnabled(boolean enabled) {
        consoleLoggingEnabled = enabled;
    }
    
    /**
     * Set console logging interval in milliseconds
     */
    public static void setConsoleLogInterval(long intervalMs) {
        // No need to store, use constant for now
    }
    
    /**
     * Log current state to System.out for remote monitoring
     */
    private static void logToConsole() {
        System.out.println("═══ SystemMonitor Update ═══");
        for (Map.Entry<String, Object> entry : liveVars.entrySet()) {
            System.out.println(String.format("  %s: %s", entry.getKey(), entry.getValue()));
        }
        System.out.println("═══════════════════════════");
    }
    
    /**
     * Force immediate console log (bypass interval check)
     */
    public static void logNow(String message) {
        if (consoleLoggingEnabled) {
            System.out.println("[SystemMonitor] " + message);
        }
    }
    
    /**
     * Display all live variables on telemetry
     */
    public static void displayOnTelemetry(Telemetry telemetry) {
        if (!enabled || telemetry == null) {
            return;
        }
        
        telemetry.addLine("═══ System Monitor ═══");
        for (Map.Entry<String, Object> entry : liveVars.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
        telemetry.addLine("═══════════════════════");
    }
    
    /**
     * Get current variable count
     */
    public static int getVariableCount() {
        return liveVars.size();
    }
    
    /**
     * Get a snapshot of all variables
     */
    public static Map<String, Object> getSnapshot() {
        return new LinkedHashMap<>(liveVars);
    }
}
