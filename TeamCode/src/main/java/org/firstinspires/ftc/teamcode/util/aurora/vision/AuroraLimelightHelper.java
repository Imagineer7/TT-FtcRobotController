/* Copyright (c) 2025 FTC Team #26581 Tundra Tech. All rights reserved.
 *
 * Aurora Limelight Vision Helper - Singleton vision system for localization support
 */

package org.firstinspires.ftc.teamcode.util.aurora.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/**
 * Aurora Limelight Vision Helper
 *
 * A singleton helper class for interfacing with the Limelight 3A camera.
 * Designed for seamless integration with the Aurora localization system.
 *
 * Key Features:
 * - Singleton pattern ensures only one instance controls the Limelight
 * - Configurable pipeline selection
 * - Automatic position caching with freshness tracking
 * - Thread-safe position retrieval
 * - Comprehensive null checking throughout
 * - Telemetry warnings for duplicate initialization attempts
 * - Position data age validation (configurable staleness threshold)
 *
 * Usage:
 * <pre>
 * // Get instance and initialize (first caller wins)
 * AuroraLimelightHelper limelight = AuroraLimelightHelper.getInstance();
 * limelight.initialize(hardwareMap, telemetry, Pipeline.APRILTAG_3D);
 *
 * // In your loop
 * limelight.update();
 *
 * // Get fresh position data
 * if (limelight.hasValidPosition()) {
 *     Pose3D pose = limelight.getLastBotPose();
 *     // Use for localization correction
 * }
 *
 * // When done
 * limelight.shutdown();
 * </pre>
 *
 * @author FTC #26581 Tundra Tech
 * @version 1.0
 */
public class AuroraLimelightHelper {

    // ============================================
    // SINGLETON IMPLEMENTATION
    // ============================================

    private static volatile AuroraLimelightHelper instance = null;
    private static final Object lock = new Object();

    // Tracking who initialized the helper
    private String initializerClassName = null;
    private int configuredPipeline = -1;

    /**
     * Get the singleton instance of AuroraLimelightHelper.
     * Thread-safe double-checked locking pattern.
     *
     * @return The singleton instance
     */
    public static AuroraLimelightHelper getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new AuroraLimelightHelper();
                }
            }
        }
        return instance;
    }

    /**
     * Reset the singleton instance.
     * Call this at the end of an OpMode to allow fresh initialization.
     * Also called automatically by shutdown().
     */
    public static void resetInstance() {
        synchronized (lock) {
            if (instance != null) {
                instance.shutdown();
            }
            instance = null;
        }
    }

    // Private constructor for singleton
    private AuroraLimelightHelper() {
        // Private to prevent direct instantiation
    }

    // ============================================
    // PIPELINE CONFIGURATION
    // ============================================

    /**
     * Predefined pipeline configurations for common use cases.
     * Pipelines must be configured in the Limelight web interface first.
     */
    public enum Pipeline {
        /** Color/reflective tape tracking */
        COLOR_TRACKING(0),

        /** Neural network object detection */
        NEURAL_DETECTOR(1),

        /** AprilTag 2D detection */
        APRILTAG_2D(2),

        /** AprilTag 3D localization (MegaTag) */
        APRILTAG_3D(3),

        /** Python-based custom pipeline */
        PYTHON_CUSTOM(4),

        /** Barcode/QR detection */
        BARCODE(5),

        /** Classifier neural network */
        CLASSIFIER(6),

        /** Custom pipeline slot 7 */
        CUSTOM_7(7),

        /** Custom pipeline slot 8 */
        CUSTOM_8(8),

        /** Custom pipeline slot 9 */
        CUSTOM_9(9);

        private final int index;

        Pipeline(int index) {
            this.index = index;
        }

        public int getIndex() {
            return index;
        }
    }

    // ============================================
    // CONFIGURATION OPTIONS
    // ============================================

    /**
     * Configuration builder for AuroraLimelightHelper.
     * Allows flexible setup with sensible defaults.
     */
    public static class Config {
        private String limelightName = "limelight";
        private Pipeline pipeline = Pipeline.APRILTAG_3D;
        private long maxDataAgeMs = 1000; // 1 second default
        private boolean enableTelemetry = true;
        private double minConfidenceThreshold = 0.0;
        private boolean autoStart = true;

        public Config setLimelightName(String name) {
            this.limelightName = name;
            return this;
        }

        public Config setPipeline(Pipeline pipeline) {
            this.pipeline = pipeline;
            return this;
        }

        /**
         * Set custom pipeline by index (0-9)
         */
        public Config setPipelineIndex(int index) {
            for (Pipeline p : Pipeline.values()) {
                if (p.getIndex() == index) {
                    this.pipeline = p;
                    return this;
                }
            }
            // If no matching enum, store index directly
            this.pipeline = null;
            return this;
        }

        public Config setMaxDataAgeMs(long maxAgeMs) {
            this.maxDataAgeMs = maxAgeMs;
            return this;
        }

        public Config setEnableTelemetry(boolean enable) {
            this.enableTelemetry = enable;
            return this;
        }

        public Config setMinConfidenceThreshold(double threshold) {
            this.minConfidenceThreshold = threshold;
            return this;
        }

        public Config setAutoStart(boolean autoStart) {
            this.autoStart = autoStart;
            return this;
        }

        // Package-private getters
        String getLimelightName() { return limelightName; }
        Pipeline getPipeline() { return pipeline; }
        long getMaxDataAgeMs() { return maxDataAgeMs; }
        boolean isEnableTelemetry() { return enableTelemetry; }
        double getMinConfidenceThreshold() { return minConfidenceThreshold; }
        boolean isAutoStart() { return autoStart; }
    }

    // ============================================
    // HARDWARE & STATE
    // ============================================

    private Limelight3A limelight;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Config config;

    // State tracking
    private boolean initialized = false;
    private boolean running = false;
    private final ElapsedTime lastValidDataTime = new ElapsedTime();
    private final ElapsedTime updateTimer = new ElapsedTime();

    // Cached data
    private LLResult lastResult = null;
    private Pose3D lastBotPose = null;
    private LLStatus lastStatus = null;
    private long lastResultTimestamp = 0;

    // Statistics
    private int totalUpdates = 0;
    private int validResults = 0;
    private int staleDataRequests = 0;

    // ============================================
    // INITIALIZATION
    // ============================================

    /**
     * Initialize the Limelight helper with default configuration.
     * Uses pipeline index 0 and default settings.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param telemetry Telemetry for logging
     * @return true if initialization successful, false otherwise
     */
    public boolean initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        return initialize(hardwareMap, telemetry, new Config());
    }

    /**
     * Initialize the Limelight helper with a specific pipeline.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param telemetry Telemetry for logging
     * @param pipeline The pipeline to use
     * @return true if initialization successful, false otherwise
     */
    public boolean initialize(HardwareMap hardwareMap, Telemetry telemetry, Pipeline pipeline) {
        return initialize(hardwareMap, telemetry, new Config().setPipeline(pipeline));
    }

    /**
     * Initialize the Limelight helper with full configuration.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param telemetry Telemetry for logging
     * @param config Configuration options
     * @return true if initialization successful, false otherwise
     */
    public boolean initialize(HardwareMap hardwareMap, Telemetry telemetry, Config config) {
        // Get caller information for tracking
        String callerClass = getCallerClassName();

        // Check if already initialized by someone else
        if (initialized) {
            int requestedPipeline = config.getPipeline() != null ?
                    config.getPipeline().getIndex() : -1;

            if (configuredPipeline != requestedPipeline) {
                // Different pipeline requested - warn!
                logWarning("⚠️ LIMELIGHT CONFLICT DETECTED! ⚠️");
                logWarning("Already initialized by: " + initializerClassName);
                logWarning("Current pipeline: " + configuredPipeline);
                logWarning("Requested pipeline by " + callerClass + ": " + requestedPipeline);
                logWarning("Ignoring new initialization - using existing configuration");
                return false;
            } else {
                // Same pipeline - just warn about duplicate init
                logWarning("Limelight already initialized by: " + initializerClassName);
                logWarning("Duplicate init request from: " + callerClass + " (same pipeline, continuing)");
                return true;
            }
        }

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.config = config;
        this.initializerClassName = callerClass;

        try {
            // Get Limelight from hardware map
            limelight = hardwareMap.get(Limelight3A.class, config.getLimelightName());

            if (limelight == null) {
                logError("Limelight not found in hardware map with name: " + config.getLimelightName());
                return false;
            }

            // Configure pipeline
            int pipelineIndex = config.getPipeline() != null ?
                    config.getPipeline().getIndex() : 0;
            limelight.pipelineSwitch(pipelineIndex);
            configuredPipeline = pipelineIndex;

            // Auto-start if configured
            if (config.isAutoStart()) {
                limelight.start();
                running = true;
            }

            initialized = true;
            lastValidDataTime.reset();
            updateTimer.reset();

            logInfo("Limelight initialized successfully");
            logInfo("  Initialized by: " + callerClass);
            logInfo("  Pipeline: " + pipelineIndex);
            logInfo("  Max data age: " + config.getMaxDataAgeMs() + "ms");

            return true;

        } catch (Exception e) {
            logError("Failed to initialize Limelight: " + e.getMessage());
            initialized = false;
            return false;
        }
    }

    // ============================================
    // UPDATE & DATA RETRIEVAL
    // ============================================

    /**
     * Update cached data from the Limelight.
     * Call this regularly in your OpMode loop.
     *
     * @return true if valid data was received, false otherwise
     */
    public boolean update() {
        if (!checkInitialized()) {
            return false;
        }

        if (!running) {
            return false;
        }

        totalUpdates++;

        try {
            // Get latest result
            LLResult result = limelight.getLatestResult();

            if (result == null) {
                return false;
            }

            lastResult = result;
            lastResultTimestamp = System.currentTimeMillis();

            // Check if result is valid
            if (result.isValid()) {
                validResults++;
                lastBotPose = result.getBotpose();
                lastValidDataTime.reset();
                return true;
            }

            return false;

        } catch (Exception e) {
            logError("Error updating Limelight data: " + e.getMessage());
            return false;
        }
    }

    /**
     * Update and also refresh status information.
     * Slightly more expensive than update() alone.
     *
     * @return true if valid data was received
     */
    public boolean updateWithStatus() {
        boolean validData = update();

        if (initialized && limelight != null) {
            try {
                lastStatus = limelight.getStatus();
            } catch (Exception e) {
                // Status fetch failed, continue anyway
            }
        }

        return validData;
    }

    // ============================================
    // POSITION DATA ACCESS
    // ============================================

    /**
     * Check if there is valid, fresh position data available.
     * Uses the configured max data age to determine freshness.
     *
     * @return true if position data is available and fresh
     */
    public boolean hasValidPosition() {
        return hasValidPosition(config != null ? config.getMaxDataAgeMs() : 1000);
    }

    /**
     * Check if there is valid position data within the specified age.
     *
     * @param maxAgeMs Maximum acceptable age in milliseconds
     * @return true if position data is available and within the age limit
     */
    public boolean hasValidPosition(long maxAgeMs) {
        if (lastBotPose == null) {
            return false;
        }

        long dataAge = getLastPositionAgeMs();
        if (dataAge > maxAgeMs) {
            staleDataRequests++;
            return false;
        }

        return true;
    }

    /**
     * Get the last bot pose from the Limelight.
     * Check hasValidPosition() first to ensure data is fresh.
     *
     * @return The last known bot pose, or null if no valid data
     */
    public Pose3D getLastBotPose() {
        return lastBotPose;
    }

    /**
     * Get the last bot pose if it's fresh enough for localization.
     * Convenience method that combines freshness check and retrieval.
     *
     * @return The bot pose if fresh, or null if stale or unavailable
     */
    public Pose3D getFreshBotPose() {
        if (hasValidPosition()) {
            return lastBotPose;
        }
        return null;
    }

    /**
     * Get the last bot pose if it's within the specified age.
     *
     * @param maxAgeMs Maximum acceptable age in milliseconds
     * @return The bot pose if within age limit, or null otherwise
     */
    public Pose3D getFreshBotPose(long maxAgeMs) {
        if (hasValidPosition(maxAgeMs)) {
            return lastBotPose;
        }
        return null;
    }

    /**
     * Get the position from the last bot pose.
     *
     * @return Position object, or null if no valid pose
     */
    public Position getLastPosition() {
        if (lastBotPose == null) {
            return null;
        }
        return lastBotPose.getPosition();
    }

    /**
     * Get the orientation from the last bot pose.
     *
     * @return YawPitchRollAngles object, or null if no valid pose
     */
    public YawPitchRollAngles getLastOrientation() {
        if (lastBotPose == null) {
            return null;
        }
        return lastBotPose.getOrientation();
    }

    /**
     * Get the age of the last valid position data.
     *
     * @return Age in milliseconds since last valid data
     */
    public long getLastPositionAgeMs() {
        return (long) lastValidDataTime.milliseconds();
    }

    /**
     * Get X position in meters.
     *
     * @return X position, or 0 if no valid data
     */
    public double getX() {
        if (lastBotPose == null || lastBotPose.getPosition() == null) {
            return 0;
        }
        return lastBotPose.getPosition().x;
    }

    /**
     * Get Y position in meters.
     *
     * @return Y position, or 0 if no valid data
     */
    public double getY() {
        if (lastBotPose == null || lastBotPose.getPosition() == null) {
            return 0;
        }
        return lastBotPose.getPosition().y;
    }

    /**
     * Get Z position (height) in meters.
     *
     * @return Z position, or 0 if no valid data
     */
    public double getZ() {
        if (lastBotPose == null || lastBotPose.getPosition() == null) {
            return 0;
        }
        return lastBotPose.getPosition().z;
    }

    /**
     * Get heading (yaw) in degrees.
     *
     * @return Heading in degrees, or 0 if no valid data
     */
    public double getHeadingDegrees() {
        if (lastBotPose == null || lastBotPose.getOrientation() == null) {
            return 0;
        }
        return lastBotPose.getOrientation().getYaw();
    }

    // ============================================
    // RAW RESULT ACCESS
    // ============================================

    /**
     * Get the last raw LLResult from the Limelight.
     * For advanced use cases that need full result data.
     *
     * @return The last LLResult, or null if no data
     */
    public LLResult getLastResult() {
        return lastResult;
    }

    /**
     * Get TX (horizontal offset to target) from last result.
     *
     * @return TX value in degrees, or 0 if no valid result
     */
    public double getTx() {
        if (lastResult == null) {
            return 0;
        }
        return lastResult.getTx();
    }

    /**
     * Get TY (vertical offset to target) from last result.
     *
     * @return TY value in degrees, or 0 if no valid result
     */
    public double getTy() {
        if (lastResult == null) {
            return 0;
        }
        return lastResult.getTy();
    }

    /**
     * Get the total latency of the last result.
     *
     * @return Total latency in milliseconds
     */
    public double getTotalLatency() {
        if (lastResult == null) {
            return 0;
        }
        return lastResult.getCaptureLatency() + lastResult.getTargetingLatency();
    }

    /**
     * Get fiducial (AprilTag) results from last data.
     *
     * @return List of fiducial results, or empty list if none
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (lastResult == null) {
            return java.util.Collections.emptyList();
        }
        List<LLResultTypes.FiducialResult> results = lastResult.getFiducialResults();
        return results != null ? results : java.util.Collections.emptyList();
    }

    /**
     * Get detector results from last data.
     *
     * @return List of detector results, or empty list if none
     */
    public List<LLResultTypes.DetectorResult> getDetectorResults() {
        if (lastResult == null) {
            return java.util.Collections.emptyList();
        }
        List<LLResultTypes.DetectorResult> results = lastResult.getDetectorResults();
        return results != null ? results : java.util.Collections.emptyList();
    }

    /**
     * Get color results from last data.
     *
     * @return List of color results, or empty list if none
     */
    public List<LLResultTypes.ColorResult> getColorResults() {
        if (lastResult == null) {
            return java.util.Collections.emptyList();
        }
        List<LLResultTypes.ColorResult> results = lastResult.getColorResults();
        return results != null ? results : java.util.Collections.emptyList();
    }

    /**
     * Get Python output array from last result.
     *
     * @return Python output array, or empty array if none
     */
    public double[] getPythonOutput() {
        if (lastResult == null) {
            return new double[0];
        }
        double[] output = lastResult.getPythonOutput();
        return output != null ? output : new double[0];
    }

    // ============================================
    // STATUS & CONTROL
    // ============================================

    /**
     * Get the last Limelight status.
     * Call updateWithStatus() to refresh.
     *
     * @return The last status, or null if not available
     */
    public LLStatus getStatus() {
        return lastStatus;
    }

    /**
     * Start the Limelight polling.
     */
    public void start() {
        if (!checkInitialized()) {
            return;
        }

        if (limelight != null) {
            limelight.start();
            running = true;
            logInfo("Limelight started");
        }
    }

    /**
     * Stop the Limelight polling.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
            running = false;
            logInfo("Limelight stopped");
        }
    }

    /**
     * Switch to a different pipeline.
     *
     * @param pipeline The pipeline to switch to
     */
    public void switchPipeline(Pipeline pipeline) {
        switchPipeline(pipeline.getIndex());
    }

    /**
     * Switch to a different pipeline by index.
     *
     * @param pipelineIndex The pipeline index (0-9)
     */
    public void switchPipeline(int pipelineIndex) {
        if (!checkInitialized()) {
            return;
        }

        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
            configuredPipeline = pipelineIndex;

            // Clear cached data when switching pipelines
            lastBotPose = null;
            lastResult = null;

            logInfo("Switched to pipeline: " + pipelineIndex);
        }
    }

    /**
     * Check if the helper is currently running.
     *
     * @return true if polling is active
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Check if the helper has been initialized.
     *
     * @return true if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Get the currently configured pipeline index.
     *
     * @return Pipeline index, or -1 if not initialized
     */
    public int getCurrentPipeline() {
        return configuredPipeline;
    }

    // ============================================
    // TELEMETRY & DIAGNOSTICS
    // ============================================

    /**
     * Add diagnostic telemetry data.
     * Call after update() for current data.
     */
    public void addTelemetry() {
        if (telemetry == null || !config.isEnableTelemetry()) {
            return;
        }

        telemetry.addLine("=== Aurora Limelight ===");
        telemetry.addData("Initialized", initialized);
        telemetry.addData("Running", running);
        telemetry.addData("Pipeline", configuredPipeline);

        if (lastStatus != null) {
            telemetry.addData("LL Status", "Temp: %.1f°C, CPU: %.1f%%, FPS: %d",
                    lastStatus.getTemp(), lastStatus.getCpu(), (int) lastStatus.getFps());
        }

        telemetry.addData("Has Valid Pose", hasValidPosition());
        telemetry.addData("Data Age", "%.0f ms", (double) getLastPositionAgeMs());

        if (lastBotPose != null) {
            Position pos = lastBotPose.getPosition();
            if (pos != null) {
                telemetry.addData("Position", "X: %.2f, Y: %.2f, Z: %.2f",
                        pos.x, pos.y, pos.z);
            }
            telemetry.addData("Heading", "%.1f°", getHeadingDegrees());
        }

        telemetry.addData("Stats", "Updates: %d, Valid: %d, Stale: %d",
                totalUpdates, validResults, staleDataRequests);
    }

    /**
     * Add compact telemetry (single line).
     */
    public void addCompactTelemetry() {
        if (telemetry == null || !config.isEnableTelemetry()) {
            return;
        }

        if (hasValidPosition()) {
            telemetry.addData("LL", "X:%.1f Y:%.1f H:%.0f° [%dms]",
                    getX(), getY(), getHeadingDegrees(), getLastPositionAgeMs());
        } else {
            telemetry.addData("LL", "No valid data (%dms old)", getLastPositionAgeMs());
        }
    }

    /**
     * Get statistics about Limelight performance.
     *
     * @return Statistics string
     */
    public String getStatistics() {
        double validRate = totalUpdates > 0 ?
                (100.0 * validResults / totalUpdates) : 0;
        return String.format("Updates: %d, Valid: %d (%.1f%%), Stale Requests: %d",
                totalUpdates, validResults, validRate, staleDataRequests);
    }

    // ============================================
    // SHUTDOWN & CLEANUP
    // ============================================

    /**
     * Shutdown the Limelight helper and release resources.
     * Also resets the singleton instance.
     */
    public void shutdown() {
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception e) {
                // Ignore shutdown errors
            }
        }

        limelight = null;
        lastResult = null;
        lastBotPose = null;
        lastStatus = null;
        initialized = false;
        running = false;
        initializerClassName = null;
        configuredPipeline = -1;

        // Reset statistics
        totalUpdates = 0;
        validResults = 0;
        staleDataRequests = 0;

        logInfo("Limelight helper shutdown complete");
    }

    // ============================================
    // INTERNAL HELPERS
    // ============================================

    private boolean checkInitialized() {
        if (!initialized) {
            logWarning("Limelight helper not initialized! Call initialize() first.");
            return false;
        }
        return true;
    }

    private String getCallerClassName() {
        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
        // Walk up the stack to find the first class that isn't this one
        for (int i = 3; i < stackTrace.length; i++) {
            String className = stackTrace[i].getClassName();
            if (!className.equals(this.getClass().getName())) {
                // Extract just the simple class name
                int lastDot = className.lastIndexOf('.');
                if (lastDot >= 0) {
                    className = className.substring(lastDot + 1);
                }
                return className;
            }
        }
        return "Unknown";
    }

    private void logInfo(String message) {
        if (telemetry != null && config != null && config.isEnableTelemetry()) {
            telemetry.addData("LL-Info", message);
        }
    }

    private void logWarning(String message) {
        if (telemetry != null) {
            telemetry.addData("LL-WARN", "⚠️ " + message);
        }
        // Also log to Android logcat for debugging
        android.util.Log.w("AuroraLimelight", message);
    }

    private void logError(String message) {
        if (telemetry != null) {
            telemetry.addData("LL-ERROR", "❌ " + message);
        }
        // Also log to Android logcat for debugging
        android.util.Log.e("AuroraLimelight", message);
    }
}

