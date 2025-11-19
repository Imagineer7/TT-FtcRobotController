/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Performance monitoring and analytics system for robot subsystems
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.LinkedList;
import java.util.Queue;

/**
 * PerformanceMonitor - Tracks robot performance metrics and provides analytics
 *
 * Features:
 * - Shot accuracy tracking
 * - Battery consumption monitoring
 * - Motor performance analysis
 * - Loop time optimization
 * - Temperature monitoring
 */
public class PerformanceMonitor {

    // Performance tracking
    private int totalShots = 0;
    private int successfulShots = 0;
    private double totalShootingTime = 0;
    private double averageSpinupTime = 0;

    // Loop timing analysis
    private Queue<Double> loopTimes = new LinkedList<>();
    private static final int LOOP_HISTORY_SIZE = 50;
    private ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTime = 0;

    // Battery monitoring
    private Queue<Double> voltageHistory = new LinkedList<>();
    private static final int VOLTAGE_HISTORY_SIZE = 100;
    private double minVoltage = Double.MAX_VALUE;
    private double maxVoltage = 0;

    // Temperature tracking (if available)
    private double maxMotorTemp = 0;
    private boolean overheated = false;

    // Performance statistics
    private double peakRPM = 0;
    private double averageRPM = 0;
    private int rpmSamples = 0;

    /**
     * Record a shot attempt
     */
    public void recordShotAttempt(boolean successful, double spinupTime) {
        totalShots++;
        if (successful) {
            successfulShots++;
        }

        // Update average spinup time
        totalShootingTime += spinupTime;
        averageSpinupTime = totalShootingTime / totalShots;
    }

    /**
     * Update loop timing statistics
     */
    public void updateLoopTiming() {
        double currentTime = loopTimer.milliseconds();
        if (lastLoopTime > 0) {
            double loopTime = currentTime - lastLoopTime;
            loopTimes.offer(loopTime);

            if (loopTimes.size() > LOOP_HISTORY_SIZE) {
                loopTimes.poll();
            }
        }
        lastLoopTime = currentTime;
    }

    /**
     * Record battery voltage
     */
    public void recordVoltage(double voltage) {
        voltageHistory.offer(voltage);
        if (voltageHistory.size() > VOLTAGE_HISTORY_SIZE) {
            voltageHistory.poll();
        }

        minVoltage = Math.min(minVoltage, voltage);
        maxVoltage = Math.max(maxVoltage, voltage);
    }

    /**
     * Record RPM measurement
     */
    public void recordRPM(double rpm) {
        peakRPM = Math.max(peakRPM, rpm);

        // Calculate running average
        averageRPM = (averageRPM * rpmSamples + rpm) / (rpmSamples + 1);
        rpmSamples++;
    }

    /**
     * Get shot accuracy percentage
     */
    public double getAccuracy() {
        return totalShots > 0 ? (double) successfulShots / totalShots * 100 : 0;
    }

    /**
     * Get average loop time in milliseconds
     */
    public double getAverageLoopTime() {
        return loopTimes.stream().mapToDouble(Double::doubleValue).average().orElse(0);
    }

    /**
     * Get maximum loop time in milliseconds
     */
    public double getMaxLoopTime() {
        return loopTimes.stream().mapToDouble(Double::doubleValue).max().orElse(0);
    }

    /**
     * Get total number of shots fired
     */
    public int getTotalShots() {
        return totalShots;
    }

    /**
     * Get current voltage drop percentage
     */
    public double getVoltageDropPercentage() {
        if (maxVoltage == 0) return 0;
        return (maxVoltage - getCurrentVoltage()) / maxVoltage * 100;
    }

    /**
     * Get current voltage (most recent reading)
     */
    private double getCurrentVoltage() {
        return voltageHistory.isEmpty() ? 0 : ((LinkedList<Double>) voltageHistory).peekLast();
    }

    /**
     * Check if performance is degraded
     */
    public boolean isPerformanceDegraded() {
        double avgLoopTime = getAverageLoopTime();
        double voltageDropPct = getVoltageDropPercentage();

        return avgLoopTime > 30 || voltageDropPct > 15 || overheated;
    }

    /**
     * Get performance recommendations
     */
    public String getRecommendations() {
        StringBuilder recommendations = new StringBuilder();

        if (getAverageLoopTime() > 25) {
            recommendations.append("• Reduce telemetry output frequency\n");
        }

        if (getVoltageDropPercentage() > 10) {
            recommendations.append("• Consider battery replacement\n");
        }

        if (getAccuracy() < 80) {
            recommendations.append("• Recalibrate shooter parameters\n");
        }

        if (averageSpinupTime > 2.5) {
            recommendations.append("• Check for mechanical friction\n");
        }

        return recommendations.toString();
    }

    /**
     * Get performance data for centralized telemetry display
     * This replaces the old updateTelemetry method to support centralized telemetry
     */
    public PerformanceData getTelemetryData() {
        return new PerformanceData(
            getAccuracy(),
            successfulShots,
            totalShots,
            getAverageLoopTime(),
            getMaxLoopTime(),
            getCurrentVoltage(),
            getVoltageDropPercentage(),
            peakRPM,
            averageRPM,
            averageSpinupTime,
            isPerformanceDegraded(),
            getRecommendations()
        );
    }

    /**
     * Data container for performance telemetry information
     */
    public static class PerformanceData {
        public final double accuracy;
        public final int successfulShots;
        public final int totalShots;
        public final double averageLoopTime;
        public final double maxLoopTime;
        public final double currentVoltage;
        public final double voltageDropPercentage;
        public final double peakRPM;
        public final double averageRPM;
        public final double averageSpinupTime;
        public final boolean performanceDegraded;
        public final String recommendations;

        public PerformanceData(double accuracy, int successfulShots, int totalShots,
                              double averageLoopTime, double maxLoopTime, double currentVoltage,
                              double voltageDropPercentage, double peakRPM, double averageRPM,
                              double averageSpinupTime, boolean performanceDegraded, String recommendations) {
            this.accuracy = accuracy;
            this.successfulShots = successfulShots;
            this.totalShots = totalShots;
            this.averageLoopTime = averageLoopTime;
            this.maxLoopTime = maxLoopTime;
            this.currentVoltage = currentVoltage;
            this.voltageDropPercentage = voltageDropPercentage;
            this.peakRPM = peakRPM;
            this.averageRPM = averageRPM;
            this.averageSpinupTime = averageSpinupTime;
            this.performanceDegraded = performanceDegraded;
            this.recommendations = recommendations;
        }
    }

    /**
     * Get performance grade as a string
     */
    public String getPerformanceGrade() {
        double avgLoopTime = getAverageLoopTime();
        if (avgLoopTime <= 50) return "🟢 EXCELLENT";
        if (avgLoopTime <= 80) return "🟡 GOOD";
        if (avgLoopTime <= 100) return "🟠 SLOW";
        return "🔴 POOR";
    }

    /**
     * Get average spinup time in seconds
     */
    public double getAverageSpinupTime() {
        return averageSpinupTime;
    }

    /**
     * Get success rate percentage (0-100)
     */
    public double getSuccessRate() {
        return getAccuracy(); // Accuracy is already success rate in percentage
    }

    /**
     * Reset all statistics
     */
    public void reset() {
        totalShots = 0;
        successfulShots = 0;
        totalShootingTime = 0;
        averageSpinupTime = 0;
        loopTimes.clear();
        voltageHistory.clear();
        minVoltage = Double.MAX_VALUE;
        maxVoltage = 0;
        peakRPM = 0;
        averageRPM = 0;
        rpmSamples = 0;
    }
}
