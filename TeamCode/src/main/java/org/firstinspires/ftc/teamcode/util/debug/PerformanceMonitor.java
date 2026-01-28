package org.firstinspires.ftc.teamcode.util.debug;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

/**
 * PerformanceMonitor - Lightweight loop timing and subsystem profiling.
 * 
 * Features:
 * - Minimal overhead (nanoTime deltas only)
 * - Rolling averages (EWMA - Exponentially Weighted Moving Average)
 * - Max tracking (reset periodically)
 * - Per-subsystem timing
 * - Aggregated stats (log every N loops)
 * - Feature flag to disable entirely
 * 
 * Usage:
 * <pre>
 * // Setup
 * PerformanceMonitor perf = new PerformanceMonitor(telemetry);
 * perf.enable(true);
 * 
 * // In loop
 * perf.startLoop();
 * 
 * perf.startSection("perception");
 * updatePerception();
 * perf.endSection("perception");
 * 
 * perf.startSection("operations");
 * runner.update();
 * perf.endSection("operations");
 * 
 * perf.endLoop();
 * 
 * // Display stats (rate-limited automatically)
 * perf.addTelemetry();
 * </pre>
 * 
 * @author FTC 26581 Tundra Tech
 * @version 1.0
 */
public class PerformanceMonitor {
    
    private final Telemetry telemetry;
    private boolean enabled;
    private boolean telemetryEnabled;
    
    // Loop tracking
    private long loopStartNs;
    private long lastLoopEndNs;
    private int loopCount;
    
    // Section tracking (current measurement)
    private final Map<String, Long> sectionStartTimes = new HashMap<>();
    
    // Rolling statistics
    private final Map<String, Stats> sectionStats = new HashMap<>();
    private Stats loopStats;
    private Stats loopPeriodStats;  // Time between loop starts
    
    // Configuration
    private static final int STATS_REPORT_INTERVAL = 50;  // Report every N loops
    private static final double EWMA_ALPHA = 0.1;  // Weight for exponential moving average (0.1 = slow decay)
    private static final int MAX_RESET_INTERVAL = 100;  // Reset max every N loops
    
    /**
     * Rolling statistics tracker using EWMA
     */
    private static class Stats {
        double ewmaMs;      // Exponentially weighted moving average
        double maxMs;       // Maximum observed (reset periodically)
        long sampleCount;   // Total samples
        int maxResetCounter;  // Count loops since max reset
        
        Stats() {
            this.ewmaMs = 0.0;
            this.maxMs = 0.0;
            this.sampleCount = 0;
            this.maxResetCounter = 0;
        }
        
        void addSample(double ms) {
            sampleCount++;
            
            // Update EWMA: newEWMA = alpha * sample + (1 - alpha) * oldEWMA
            if (ewmaMs == 0.0) {
                ewmaMs = ms;  // First sample
            } else {
                ewmaMs = EWMA_ALPHA * ms + (1.0 - EWMA_ALPHA) * ewmaMs;
            }
            
            // Update max
            if (ms > maxMs) {
                maxMs = ms;
            }
            
            // Periodically reset max to catch recent spikes
            maxResetCounter++;
            if (maxResetCounter >= MAX_RESET_INTERVAL) {
                maxMs = ms;  // Reset to current sample
                maxResetCounter = 0;
            }
        }
        
        @Override
        public String toString() {
            return String.format("avg=%.1fms max=%.1fms", ewmaMs, maxMs);
        }
    }
    
    /**
     * Create a PerformanceMonitor instance.
     * 
     * @param telemetry FTC telemetry for output
     */
    public PerformanceMonitor(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.enabled = false;
        this.telemetryEnabled = true;
        this.loopStats = new Stats();
        this.loopPeriodStats = new Stats();
        this.loopCount = 0;
    }
    
    /**
     * Enable or disable performance monitoring.
     * When disabled, all methods become no-ops (minimal overhead).
     * 
     * @param enabled true to enable monitoring
     */
    public void enable(boolean enabled) {
        this.enabled = enabled;
        if (enabled) {
            reset();
        }
    }
    
    /**
     * Enable or disable telemetry output.
     * 
     * @param enabled true to show telemetry
     */
    public void setTelemetryEnabled(boolean enabled) {
        this.telemetryEnabled = enabled;
    }
    
    /**
     * Check if monitoring is enabled.
     */
    public boolean isEnabled() {
        return enabled;
    }
    
    /**
     * Reset all statistics.
     */
    public void reset() {
        loopStats = new Stats();
        loopPeriodStats = new Stats();
        sectionStats.clear();
        loopCount = 0;
        lastLoopEndNs = 0;
    }
    
    /**
     * Mark the start of a loop iteration.
     * Measures time since last loop (period/jitter).
     */
    public void startLoop() {
        if (!enabled) return;
        
        long now = System.nanoTime();
        loopStartNs = now;
        
        // Measure period (time between loop starts)
        if (lastLoopEndNs > 0) {
            double periodMs = (now - lastLoopEndNs) / 1_000_000.0;
            loopPeriodStats.addSample(periodMs);
        }
    }
    
    /**
     * Mark the end of a loop iteration.
     * Measures total loop time.
     */
    public void endLoop() {
        if (!enabled) return;
        
        long now = System.nanoTime();
        lastLoopEndNs = now;
        
        if (loopStartNs > 0) {
            double loopMs = (now - loopStartNs) / 1_000_000.0;
            loopStats.addSample(loopMs);
        }
        
        loopCount++;
    }
    
    /**
     * Start timing a named section.
     * 
     * @param sectionName Name of the section (e.g., "perception", "operations")
     */
    public void startSection(String sectionName) {
        if (!enabled) return;
        sectionStartTimes.put(sectionName, System.nanoTime());
    }
    
    /**
     * End timing a named section and record the measurement.
     * 
     * @param sectionName Name of the section (must match startSection call)
     */
    public void endSection(String sectionName) {
        if (!enabled) return;
        
        Long startNs = sectionStartTimes.remove(sectionName);
        if (startNs == null) {
            return;  // Section not started
        }
        
        long endNs = System.nanoTime();
        double durationMs = (endNs - startNs) / 1_000_000.0;
        
        // Get or create stats for this section
        Stats stats = sectionStats.computeIfAbsent(sectionName, k -> new Stats());
        stats.addSample(durationMs);
    }
    
    /**
     * Get loop average time in milliseconds.
     */
    public double getLoopAvgMs() {
        return enabled ? loopStats.ewmaMs : 0.0;
    }
    
    /**
     * Get loop max time in milliseconds.
     */
    public double getLoopMaxMs() {
        return enabled ? loopStats.maxMs : 0.0;
    }
    
    /**
     * Get loop period average in milliseconds.
     */
    public double getLoopPeriodAvgMs() {
        return enabled ? loopPeriodStats.ewmaMs : 0.0;
    }
    
    /**
     * Get section average time in milliseconds.
     */
    public double getSectionAvgMs(String sectionName) {
        if (!enabled) return 0.0;
        Stats stats = sectionStats.get(sectionName);
        return (stats != null) ? stats.ewmaMs : 0.0;
    }
    
    /**
     * Add performance telemetry to display.
     * Automatically rate-limited to every N loops.
     */
    public void addTelemetry() {
        if (!enabled || !telemetryEnabled) return;
        
        // Rate limit: only update telemetry every N loops
        if (loopCount % STATS_REPORT_INTERVAL != 0) {
            return;
        }
        
        telemetry.addLine("=== PERFORMANCE ===");
        telemetry.addData("Loops", loopCount);
        telemetry.addData("Loop Period", loopPeriodStats.toString());
        telemetry.addData("Loop Time", loopStats.toString());
        
        // Add section timings
        if (!sectionStats.isEmpty()) {
            telemetry.addLine("--- Sections ---");
            for (Map.Entry<String, Stats> entry : sectionStats.entrySet()) {
                telemetry.addData(entry.getKey(), entry.getValue().toString());
            }
        }
    }
    
    /**
     * Add compact single-line telemetry (for minimal display).
     */
    public void addCompactTelemetry() {
        if (!enabled || !telemetryEnabled) return;
        
        // Rate limit
        if (loopCount % STATS_REPORT_INTERVAL != 0) {
            return;
        }
        
        String summary = String.format("Loop: %s | Period: %s", 
            loopStats.toString(), 
            loopPeriodStats.toString());
        telemetry.addData("Perf", summary);
    }
    
    /**
     * Log performance summary to Dbg.
     */
    public void logSummary() {
        if (!enabled) return;
        
        // Rate limit
        if (loopCount % STATS_REPORT_INTERVAL != 0) {
            return;
        }
        
        Dbg.i(LogGroup.SYSTEM, "PERF: Loop %s | Period %s", loopStats, loopPeriodStats);
        
        for (Map.Entry<String, Stats> entry : sectionStats.entrySet()) {
            Dbg.i(LogGroup.SYSTEM, "  %s: %s", entry.getKey(), entry.getValue());
        }
    }
}
