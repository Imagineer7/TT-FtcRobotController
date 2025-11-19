/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Machine Learning System for RPM Control Optimization
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import android.os.Environment;
import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * RpmLearningSystem - Advanced ML for adaptive RPM control
 *
 * Features:
 * - Learns optimal PID gains for different RPM ranges
 * - Tracks performance patterns (overshoot, settling time, steady-state error)
 * - Adapts to battery voltage changes
 * - Saves learned parameters to persistent storage
 * - Gradient descent optimization for gain tuning
 */
public class RpmLearningSystem {

    // Storage configuration
    private static final String CONFIG_DIR = Environment.getExternalStorageDirectory().getPath() + "/FIRST/";
    private static final String CONFIG_FILE = "rpm_learning_data.txt";

    // Learning state
    private boolean learningEnabled = false; // Default OFF - only enable in training mode
    private int totalSamples = 0;
    private double learningRate = 0.02; // 2% adjustment per iteration

    // PID gain tracking (steady-state)
    private double learnedKp = 0.00020;
    private double learnedKi = 0.00005;
    private double learnedKd = 0.00005;

    // PID gain tracking (recovery mode)
    private double learnedKpRecovery = 0.00042;
    private double learnedKiRecovery = 0.00004;
    private double learnedKdRecovery = 0.00015;

    // Performance metrics
    private List<PerformanceSample> recentSamples = new ArrayList<>();
    private static final int MAX_SAMPLES = 50;

    // RPM pattern learning
    private double avgOvershoot = 0.0;
    private double avgSettlingTime = 0.0;
    private double avgSteadyStateError = 0.0;
    private double avgRecoveryTime = 0.0;

    // Voltage compensation learning
    private double voltageCompensationFactor = 1.0;
    private double lastVoltage = 12.0;

    // Pattern detection
    private boolean detectsOscillation = false;
    private int oscillationCount = 0;

    /**
     * Performance sample for learning
     */
    public static class PerformanceSample {
        public double targetRpm;
        public double actualRpm;
        public double error;
        public double overshoot;
        public double settlingTime;
        public double voltage;
        public long timestamp;

        public PerformanceSample(double targetRpm, double actualRpm, double error,
                                double overshoot, double settlingTime, double voltage) {
            this.targetRpm = targetRpm;
            this.actualRpm = actualRpm;
            this.error = error;
            this.overshoot = overshoot;
            this.settlingTime = settlingTime;
            this.voltage = voltage;
            this.timestamp = System.currentTimeMillis();
        }
    }

    /**
     * Initialize learning system and load saved data
     */
    public RpmLearningSystem() {
        loadLearningData();
    }

    /**
     * Record RPM performance sample for learning
     */
    public void recordPerformance(double targetRpm, double actualRpm, double overshoot,
                                  double settlingTime, double voltage) {
        if (!learningEnabled) return;

        double error = Math.abs(targetRpm - actualRpm);
        PerformanceSample sample = new PerformanceSample(
            targetRpm, actualRpm, error, overshoot, settlingTime, voltage
        );

        recentSamples.add(sample);
        if (recentSamples.size() > MAX_SAMPLES) {
            recentSamples.remove(0);
        }

        totalSamples++;

        // Update running averages
        updatePerformanceMetrics();

        // Trigger learning after enough samples
        if (totalSamples % 10 == 0 && totalSamples >= 20) {
            adaptPidGains();
        }

        // Detect oscillation patterns
        detectOscillationPattern();
    }

    /**
     * Update performance metrics from recent samples
     */
    private void updatePerformanceMetrics() {
        if (recentSamples.isEmpty()) return;

        double sumOvershoot = 0;
        double sumSettling = 0;
        double sumError = 0;
        int count = Math.min(recentSamples.size(), 10); // Last 10 samples

        for (int i = recentSamples.size() - count; i < recentSamples.size(); i++) {
            PerformanceSample s = recentSamples.get(i);
            sumOvershoot += s.overshoot;
            sumSettling += s.settlingTime;
            sumError += s.error;
        }

        avgOvershoot = sumOvershoot / count;
        avgSettlingTime = sumSettling / count;
        avgSteadyStateError = sumError / count;
    }

    /**
     * Adapt PID gains based on performance using gradient descent
     */
    private void adaptPidGains() {
        if (recentSamples.size() < 10) return;

        // Gradient descent adjustments based on patterns

        // High steady-state error -> increase Kp
        if (avgSteadyStateError > 30) {
            learnedKp += learnedKp * learningRate;
            learnedKpRecovery += learnedKpRecovery * learningRate;
        }
        // Low error but slow settling -> increase Ki slightly
        else if (avgSettlingTime > 0.3 && avgSteadyStateError < 20) {
            learnedKi += learnedKi * learningRate * 0.5;
            learnedKiRecovery += learnedKiRecovery * learningRate * 0.5;
        }

        // High overshoot -> increase Kd (damping)
        if (avgOvershoot > 50) {
            learnedKd += learnedKd * learningRate;
            learnedKdRecovery += learnedKdRecovery * learningRate;
        }
        // Oscillation detected -> reduce all gains slightly
        else if (detectsOscillation) {
            learnedKp *= (1.0 - learningRate);
            learnedKi *= (1.0 - learningRate * 0.5);
            learnedKd *= (1.0 + learningRate); // Increase damping
        }

        // Clamp gains to safe ranges
        learnedKp = Math.max(0.00010, Math.min(0.00050, learnedKp));
        learnedKi = Math.max(0.00001, Math.min(0.00020, learnedKi));
        learnedKd = Math.max(0.00001, Math.min(0.00030, learnedKd));

        learnedKpRecovery = Math.max(0.00020, Math.min(0.00080, learnedKpRecovery));
        learnedKiRecovery = Math.max(0.00001, Math.min(0.00020, learnedKiRecovery));
        learnedKdRecovery = Math.max(0.00005, Math.min(0.00040, learnedKdRecovery));
    }

    /**
     * Detect oscillation patterns in RPM control
     */
    private void detectOscillationPattern() {
        if (recentSamples.size() < 6) return;

        // Check for alternating positive/negative errors
        int alternations = 0;
        for (int i = recentSamples.size() - 5; i < recentSamples.size() - 1; i++) {
            double error1 = recentSamples.get(i).error;
            double error2 = recentSamples.get(i + 1).error;

            if (Math.signum(error1) != Math.signum(error2) &&
                Math.abs(error1) > 20 && Math.abs(error2) > 20) {
                alternations++;
            }
        }

        if (alternations >= 3) {
            oscillationCount++;
            if (oscillationCount >= 2) {
                detectsOscillation = true;
            }
        } else {
            oscillationCount = Math.max(0, oscillationCount - 1);
            if (oscillationCount == 0) {
                detectsOscillation = false;
            }
        }
    }

    /**
     * Learn voltage compensation factor
     */
    public void updateVoltageCompensation(double voltage, double targetRpm, double actualRpm) {
        if (voltage < 10.0 || voltage > 14.0) return;

        // Learn how voltage affects RPM achievement
        double rpmRatio = actualRpm / targetRpm;
        double voltageRatio = voltage / 12.0;

        // Update compensation factor with exponential moving average
        double alpha = 0.1; // Smoothing factor
        voltageCompensationFactor = alpha * (rpmRatio / voltageRatio) +
                                   (1 - alpha) * voltageCompensationFactor;

        voltageCompensationFactor = Math.max(0.8, Math.min(1.2, voltageCompensationFactor));
        lastVoltage = voltage;
    }

    /**
     * Get learned PID gains for steady-state control
     */
    public double[] getLearnedGainsSteadyState() {
        return new double[] { learnedKp, learnedKi, learnedKd };
    }

    /**
     * Get learned PID gains for recovery control
     */
    public double[] getLearnedGainsRecovery() {
        return new double[] { learnedKpRecovery, learnedKiRecovery, learnedKdRecovery };
    }

    /**
     * Get voltage compensation factor
     */
    public double getVoltageCompensationFactor() {
        return voltageCompensationFactor;
    }

    /**
     * Get learning telemetry string
     */
    public String getLearningTelemetry() {
        return String.format(Locale.US, "ML Samples: %d | Kp: %.5f | Ki: %.5f | Kd: %.5f | Osc: %s",
            totalSamples, learnedKp, learnedKi, learnedKd, detectsOscillation ? "YES" : "NO");
    }

    /**
     * Get performance metrics string
     */
    public String getPerformanceMetrics() {
        return String.format(Locale.US, "Err: %.1f | Overshoot: %.1f | Settle: %.3fs | VComp: %.3f",
            avgSteadyStateError, avgOvershoot, avgSettlingTime, voltageCompensationFactor);
    }

    /**
     * Save learning data to file
     */
    public boolean saveLearningData() {
        try {
            File dir = new File(CONFIG_DIR);
            if (!dir.exists()) dir.mkdirs();

            File file = new File(CONFIG_DIR + CONFIG_FILE);
            FileWriter writer = new FileWriter(file);

            writer.write("# RPM Learning Data - Auto-generated\n");
            writer.write("# Last updated: " + System.currentTimeMillis() + "\n");
            writer.write("total_samples=" + totalSamples + "\n");
            writer.write("kp=" + learnedKp + "\n");
            writer.write("ki=" + learnedKi + "\n");
            writer.write("kd=" + learnedKd + "\n");
            writer.write("kp_recovery=" + learnedKpRecovery + "\n");
            writer.write("ki_recovery=" + learnedKiRecovery + "\n");
            writer.write("kd_recovery=" + learnedKdRecovery + "\n");
            writer.write("voltage_compensation=" + voltageCompensationFactor + "\n");
            writer.write("avg_overshoot=" + avgOvershoot + "\n");
            writer.write("avg_settling_time=" + avgSettlingTime + "\n");
            writer.write("avg_steady_state_error=" + avgSteadyStateError + "\n");

            writer.close();
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * Load learning data from file
     */
    public boolean loadLearningData() {
        try {
            File file = new File(CONFIG_DIR + CONFIG_FILE);
            if (!file.exists()) return false;

            BufferedReader reader = new BufferedReader(new FileReader(file));
            String line;

            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.startsWith("#") || line.isEmpty()) continue;

                String[] parts = line.split("=");
                if (parts.length != 2) continue;

                String key = parts[0].trim();
                String value = parts[1].trim();

                try {
                    switch (key) {
                        case "total_samples":
                            totalSamples = Integer.parseInt(value);
                            break;
                        case "kp":
                            learnedKp = Double.parseDouble(value);
                            break;
                        case "ki":
                            learnedKi = Double.parseDouble(value);
                            break;
                        case "kd":
                            learnedKd = Double.parseDouble(value);
                            break;
                        case "kp_recovery":
                            learnedKpRecovery = Double.parseDouble(value);
                            break;
                        case "ki_recovery":
                            learnedKiRecovery = Double.parseDouble(value);
                            break;
                        case "kd_recovery":
                            learnedKdRecovery = Double.parseDouble(value);
                            break;
                        case "voltage_compensation":
                            voltageCompensationFactor = Double.parseDouble(value);
                            break;
                        case "avg_overshoot":
                            avgOvershoot = Double.parseDouble(value);
                            break;
                        case "avg_settling_time":
                            avgSettlingTime = Double.parseDouble(value);
                            break;
                        case "avg_steady_state_error":
                            avgSteadyStateError = Double.parseDouble(value);
                            break;
                    }
                } catch (NumberFormatException e) {
                    // Skip invalid values
                }
            }

            reader.close();
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * Reset learning data to defaults
     */
    public boolean resetLearning() {
        totalSamples = 0;
        learnedKp = 0.00020;
        learnedKi = 0.00005;
        learnedKd = 0.00005;
        learnedKpRecovery = 0.00042;
        learnedKiRecovery = 0.00004;
        learnedKdRecovery = 0.00015;
        voltageCompensationFactor = 1.0;
        avgOvershoot = 0.0;
        avgSettlingTime = 0.0;
        avgSteadyStateError = 0.0;
        recentSamples.clear();
        oscillationCount = 0;
        detectsOscillation = false;

        try {
            File file = new File(CONFIG_DIR + CONFIG_FILE);
            if (file.exists()) file.delete();
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    // Getters and setters
    public void setLearningEnabled(boolean enabled) { this.learningEnabled = enabled; }
    public boolean isLearningEnabled() { return learningEnabled; }
    public int getTotalSamples() { return totalSamples; }
    public void setLearningRate(double rate) {
        this.learningRate = Math.max(0.01, Math.min(0.1, rate));
    }
    public double getLearningRate() { return learningRate; }
    public boolean hasLearningData() { return totalSamples > 0; }
}

