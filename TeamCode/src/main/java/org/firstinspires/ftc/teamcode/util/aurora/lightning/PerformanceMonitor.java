package org.firstinspires.ftc.teamcode.util.aurora.lightning;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * PerformanceMonitor - Real-time system performance tracking and anomaly detection
 *
 * Monitors:
 * - Loop times and update rates
 * - Position tracking performance
 * - Sensor health
 * - Battery voltage trends
 * - Memory usage
 *
 * Provides alerts for:
 * - Slow loops (potential hang)
 * - Sensor failures
 * - Low battery
 * - Performance degradation
 */
public class PerformanceMonitor {

    // Telemetry for warnings
    private Telemetry telemetry;

    // Timing tracking
    private ElapsedTime loopTimer = new ElapsedTime();
    private CircularDataBuffer<Double> loopTimes = new CircularDataBuffer<>(100);
    private long loopCount = 0;

    // Performance thresholds
    public double slowLoopThresholdMs = 50.0;  // Alert if loop takes > 50ms
    public double targetLoopTimeMs = 20.0;     // Target 50Hz update rate

    // Position tracking performance
    private CircularDataBuffer<Double> positionErrors = new CircularDataBuffer<>(100);
    private CircularDataBuffer<Double> velocityMagnitudes = new CircularDataBuffer<>(100);

    // Battery tracking
    private CircularDataBuffer<Double> batteryVoltages = new CircularDataBuffer<>(50);
    public double lowBatteryThreshold = 12.0;  // Volts

    // Sensor update tracking
    private long lastOdometryUpdate = 0;
    private long lastVisionUpdate = 0;
    private long odometryUpdateCount = 0;
    private long visionUpdateCount = 0;

    // Anomaly detection
    private List<String> currentWarnings = new ArrayList<>();
    private CircularDataBuffer<String> recentIssues = new CircularDataBuffer<>(20);

    // Statistics
    private double totalRuntime = 0;
    private long totalLoops = 0;

    /**
     * Constructor
     */
    public PerformanceMonitor(Telemetry telemetry) {
        this.telemetry = telemetry;
        loopTimer.reset();
    }

    /**
     * Call this at the START of each control loop
     */
    public void startLoop() {
        loopTimer.reset();
    }

    /**
     * Call this at the END of each control loop
     */
    public void endLoop() {
        double loopTime = loopTimer.milliseconds();
        loopTimes.add(loopTime);
        loopCount++;
        totalLoops++;
        totalRuntime += loopTime / 1000.0;

        // Check for slow loop
        if (loopTime > slowLoopThresholdMs) {
            String warning = String.format("Slow loop: %.1f ms", loopTime);
            addWarning(warning);
        }
    }

    /**
     * Record position error for tracking
     */
    public void recordPositionError(double error) {
        positionErrors.add(error);
    }

    /**
     * Record velocity magnitude
     */
    public void recordVelocity(double vx, double vy) {
        double magnitude = Math.hypot(vx, vy);
        velocityMagnitudes.add(magnitude);
    }

    /**
     * Record battery voltage
     */
    public void recordBatteryVoltage(double voltage) {
        batteryVoltages.add(voltage);

        // Check for low battery
        if (voltage < lowBatteryThreshold) {
            addWarning(String.format("Low battery: %.1f V", voltage));
        }
    }

    /**
     * Mark that odometry was updated
     */
    public void markOdometryUpdate() {
        lastOdometryUpdate = System.nanoTime();
        odometryUpdateCount++;
    }

    /**
     * Mark that vision was updated
     */
    public void markVisionUpdate() {
        lastVisionUpdate = System.nanoTime();
        visionUpdateCount++;
    }

    /**
     * Check if loop time is currently slow
     */
    public boolean isLoopTimeSlow() {
        if (loopTimes.size() == 0) return false;
        return loopTimes.getLast() > slowLoopThresholdMs;
    }

    /**
     * Get average loop time (ms)
     */
    public double getAverageLoopTime() {
        if (loopTimes.size() == 0) return 0;
        return loopTimes.getMean();
    }

    /**
     * Get current loop rate (Hz)
     */
    public double getLoopRate() {
        double avgTime = getAverageLoopTime();
        if (avgTime == 0) return 0;
        return 1000.0 / avgTime;
    }

    /**
     * Get maximum loop time in recent history (ms)
     */
    public double getMaxLoopTime() {
        if (loopTimes.size() == 0) return 0;
        return loopTimes.getMax();
    }

    /**
     * Get average position error
     */
    public double getAveragePositionError() {
        if (positionErrors.size() == 0) return 0;
        return positionErrors.getMean();
    }

    /**
     * Get maximum position error in recent history
     */
    public double getMaxPositionError() {
        if (positionErrors.size() == 0) return 0;
        return positionErrors.getMax();
    }

    /**
     * Get current battery voltage (latest reading)
     */
    public double getCurrentBatteryVoltage() {
        if (batteryVoltages.size() == 0) return 0;
        return batteryVoltages.getLast();
    }

    /**
     * Get battery voltage trend (negative = draining)
     */
    public double getBatteryVoltageTrend() {
        if (batteryVoltages.size() < 10) return 0;

        // Simple linear trend: compare first 10 to last 10
        double firstAvg = 0;
        double lastAvg = 0;
        int midpoint = batteryVoltages.size() / 2;

        for (int i = 0; i < 10; i++) {
            firstAvg += batteryVoltages.get(i);
            lastAvg += batteryVoltages.get(batteryVoltages.size() - 10 + i);
        }

        firstAvg /= 10;
        lastAvg /= 10;

        return lastAvg - firstAvg;
    }

    /**
     * Get odometry update rate (Hz)
     */
    public double getOdometryUpdateRate() {
        if (odometryUpdateCount == 0 || totalRuntime == 0) return 0;
        return odometryUpdateCount / totalRuntime;
    }

    /**
     * Get vision update rate (Hz)
     */
    public double getVisionUpdateRate() {
        if (visionUpdateCount == 0 || totalRuntime == 0) return 0;
        return visionUpdateCount / totalRuntime;
    }

    /**
     * Check if odometry has stopped updating
     */
    public boolean isOdometryStale() {
        if (lastOdometryUpdate == 0) return false;
        long age = System.nanoTime() - lastOdometryUpdate;
        return age > 100_000_000;  // 100ms without update
    }

    /**
     * Check if vision has stopped updating
     */
    public boolean isVisionStale() {
        if (lastVisionUpdate == 0) return false;
        long age = System.nanoTime() - lastVisionUpdate;
        return age > 500_000_000;  // 500ms without update
    }

    /**
     * Add a warning to the current list
     */
    private void addWarning(String warning) {
        if (!currentWarnings.contains(warning)) {
            currentWarnings.add(warning);
            recentIssues.add(warning);
        }
    }

    /**
     * Clear current warnings
     */
    public void clearWarnings() {
        currentWarnings.clear();
    }

    /**
     * Get current active warnings
     */
    public List<String> getWarnings() {
        return new ArrayList<>(currentWarnings);
    }

    /**
     * Check if there are any active warnings
     */
    public boolean hasWarnings() {
        return !currentWarnings.isEmpty();
    }

    /**
     * Get performance summary
     */
    public PerformanceSummary getSummary() {
        PerformanceSummary summary = new PerformanceSummary();
        summary.avgLoopTime = getAverageLoopTime();
        summary.maxLoopTime = getMaxLoopTime();
        summary.loopRate = getLoopRate();
        summary.avgPositionError = getAveragePositionError();
        summary.maxPositionError = getMaxPositionError();
        summary.batteryVoltage = getCurrentBatteryVoltage();
        summary.batteryTrend = getBatteryVoltageTrend();
        summary.odometryRate = getOdometryUpdateRate();
        summary.visionRate = getVisionUpdateRate();
        summary.totalLoops = totalLoops;
        summary.totalRuntime = totalRuntime;
        return summary;
    }

    /**
     * Add performance telemetry
     */
    public void addTelemetry() {
        telemetry.addData("=== PERFORMANCE ===", "");
        telemetry.addData("Loop Time", "%.1f ms (%.0f Hz)",
            getAverageLoopTime(), getLoopRate());
        telemetry.addData("Max Loop Time", "%.1f ms", getMaxLoopTime());

        if (positionErrors.size() > 0) {
            telemetry.addData("Pos Error", "Avg:%.1f Max:%.1f in",
                getAveragePositionError(), getMaxPositionError());
        }

        if (batteryVoltages.size() > 0) {
            double trend = getBatteryVoltageTrend();
            String trendStr = trend < -0.1 ? "↓" : (trend > 0.1 ? "↑" : "−");
            telemetry.addData("Battery", "%.1f V %s",
                getCurrentBatteryVoltage(), trendStr);
        }

        if (odometryUpdateCount > 0) {
            telemetry.addData("Odometry Rate", "%.0f Hz %s",
                getOdometryUpdateRate(),
                isOdometryStale() ? "⚠️ STALE" : "✓");
        }

        if (visionUpdateCount > 0) {
            telemetry.addData("Vision Rate", "%.0f Hz %s",
                getVisionUpdateRate(),
                isVisionStale() ? "⚠️ STALE" : "✓");
        }

        // Show warnings
        if (hasWarnings()) {
            telemetry.addData("⚠️ WARNINGS", "");
            for (String warning : currentWarnings) {
                telemetry.addData(" - ", warning);
            }
        }
    }

    /**
     * Performance summary data class
     */
    public static class PerformanceSummary {
        public double avgLoopTime;
        public double maxLoopTime;
        public double loopRate;
        public double avgPositionError;
        public double maxPositionError;
        public double batteryVoltage;
        public double batteryTrend;
        public double odometryRate;
        public double visionRate;
        public long totalLoops;
        public double totalRuntime;

        @Override
        public String toString() {
            return String.format(
                "Performance: %.1fms/loop (%.0fHz), PosErr:%.1fin, Battery:%.1fV, Runtime:%.1fs",
                avgLoopTime, loopRate, avgPositionError, batteryVoltage, totalRuntime
            );
        }
    }
}

