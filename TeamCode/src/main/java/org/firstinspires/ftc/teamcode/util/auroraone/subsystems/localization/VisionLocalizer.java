package org.firstinspires.ftc.teamcode.util.auroraone.subsystems.localization;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.util.auroraone.config.RobotMap;
import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * AURORA ONE - Vision Localizer
 * Advanced vision-based localization system with AprilTag tracking and field positioning
 *
 * This class is responsible for managing the robot's vision-based localization system.
 * It provides methods to track the robot's position and orientation on the field using vision systems.
 * It will output the robot's location data for use by other subsystems.
 *
 * Features:
 * - AprilTag-based position tracking using Tunables configuration
 * - Real-time position updates with confidence scoring
 * - Field-aware zone detection and validation
 * - Camera optimization using Tunables parameters
 * - Integration with Aurora One State Machine system
 * - Data output to LocalizationUnifier for sensor fusion
 * - Robust error handling and recovery
 */
public class VisionLocalizer {

    // Hardware reference
    private RobotMap robotMap;
    private Telemetry telemetry;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Camera configuration using Tunables
    // Position: X=19.5" (right), Y=26.8" (forward from robot center)
    private static final Position CAMERA_POSITION = new Position(DistanceUnit.INCH,
            19.5, 26.8, 0, 0); // This could be moved to Tunables in the future

    // Camera Orientation: Yaw=180° (rear-facing), Pitch=-90° (horizontal), Roll=180° (upside-down)
    // NOTE: The roll=180° parameter corrects AprilTag detection calculations for upside-down mounting.
    // For the camera STREAM display to appear right-side up on the Driver Station:
    //   1. On the Robot Controller, go to: Configure Robot → Webcams → "Webcam 1"
    //   2. Find the "Rotation" or "Orientation" setting
    //   3. Set it to "180 degrees" or "Upside Down"
    // If the hardware config doesn't have this option, the stream will appear upside-down but
    // AprilTag detections will still work correctly due to the roll parameter below.
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 180, 0); // Rear-facing, horizontal, upside-down mount

    // State management
    private boolean isInitialized = false;
    private boolean isStreaming = false;

    // Position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private boolean hasValidPosition = false;
    private double positionConfidence = 0.0;

    // Timing and performance
    private ElapsedTime lastUpdateTimer = new ElapsedTime();
    private ElapsedTime performanceTimer = new ElapsedTime();
    private int totalDetections = 0;
    private int validDetections = 0;

    // LocalizationUnifier integration
    private LocalizationCallback localizationCallback;

    /**
     * Interface for sending data to LocalizationUnifier
     */
    public interface LocalizationCallback {
        void onVisionPositionUpdate(double x, double y, double heading, double confidence, long timestamp);
        void onVisionPositionLost();
    }

    /**
     * Constructor - Initialize with RobotMap
     */
    public VisionLocalizer(RobotMap robotMap, Telemetry telemetry) {
        this.robotMap = robotMap;
        this.telemetry = telemetry;
        initialize();
    }

    /**
     * Initialize the vision localization system using RobotMap
     */
    private void initialize() {
        if (!robotMap.isVisionSystemReady()) {
            isInitialized = false;
            telemetry.addLine("⚠️ VisionLocalizer: Vision system not ready in RobotMap");
            return;
        }

        try {
            initializeVisionSystem();
            isInitialized = true;
            lastUpdateTimer.reset();
            performanceTimer.reset();
            telemetry.addLine("✅ VisionLocalizer: Initialized successfully");
        } catch (Exception e) {
            isInitialized = false;
            telemetry.addLine("❌ VisionLocalizer: Initialization failed - " + e.getMessage());
        }
    }

    /**
     * Initialize the vision system with Tunables configuration
     */
    private void initializeVisionSystem() {
        // Create AprilTag processor with Tunables configuration
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(Tunables.isDebugEnabled("vision"))
                .setDrawTagOutline(Tunables.isDebugEnabled("vision"))
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                // Use Tunables for AprilTag size
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        // Create vision portal with Tunables configuration
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCameraResolution(new android.util.Size(Tunables.VISION_CAMERA_WIDTH, Tunables.VISION_CAMERA_HEIGHT))
                .addProcessor(aprilTagProcessor);

        // Try primary webcam first, then secondary
        if (robotMap.primaryWebcam != null) {
            builder.setCamera(robotMap.primaryWebcam);
            telemetry.addLine("🎥 VisionLocalizer: Using primary webcam");
        } else if (robotMap.secondaryWebcam != null) {
            builder.setCamera(robotMap.secondaryWebcam);
            telemetry.addLine("🎥 VisionLocalizer: Using secondary webcam");
        } else {
            throw new RuntimeException("No webcam available in RobotMap");
        }

        visionPortal = builder.build();

        // Apply camera optimization with Tunables
        applyCameraOptimization();
    }

    /**
     * Apply optimized camera settings using Tunables
     */
    private void applyCameraOptimization() {
        try {
            // Wait for camera to be ready with timeout
            ElapsedTime waitTimer = new ElapsedTime();
            double maxWaitSeconds = 5.0; // 5 second timeout

            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                if (waitTimer.seconds() > maxWaitSeconds) {
                    telemetry.addLine("⚠️ VisionLocalizer: Camera streaming timeout - continuing anyway");
                    return; // Exit without applying settings
                }
                Thread.sleep(20);
            }

            isStreaming = true;

            // Apply Tunables camera settings
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            if (exposureControl != null) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure((long) Tunables.VISION_EXPOSURE_MS, TimeUnit.MILLISECONDS);

                if (Tunables.isDebugEnabled("vision")) {
                    telemetry.addData("Vision Camera", "Exposure set to %.1fms", Tunables.VISION_EXPOSURE_MS);
                }
            }

            if (gainControl != null) {
                gainControl.setGain(Tunables.VISION_GAIN);

                if (Tunables.isDebugEnabled("vision")) {
                    telemetry.addData("Vision Camera", "Gain set to %d", Tunables.VISION_GAIN);
                }
            }

            telemetry.addLine("📹 VisionLocalizer: Camera optimization applied");

        } catch (Exception e) {
            telemetry.addLine("⚠️ VisionLocalizer: Camera optimization failed - " + e.getMessage());
        }
    }

    /**
     * Set callback for LocalizationUnifier integration
     */
    public void setLocalizationCallback(LocalizationCallback callback) {
        this.localizationCallback = callback;
    }

    /**
     * Main update method - call this every loop cycle
     */
    public void update() {
        if (!isInitialized()) return;

        boolean positionUpdated = updatePosition();

        // Send data to LocalizationUnifier if available
        if (localizationCallback != null) {
            if (positionUpdated && hasValidPosition) {
                localizationCallback.onVisionPositionUpdate(
                    robotX, robotY, robotHeading,
                    positionConfidence, System.currentTimeMillis()
                );
            } else if (!hasValidPosition && lastUpdateTimer.seconds() > Tunables.COMMUNICATION_TIMEOUT) {
                localizationCallback.onVisionPositionLost();
            }
        }

        // Debug output if enabled
        if (Tunables.isDebugEnabled("vision")) {
            updateDebugTelemetry();
        }
    }

    /**
     * Update position from AprilTag detections
     * Returns true if position was updated
     */
    private boolean updatePosition() {
        if (aprilTagProcessor == null) return false;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        totalDetections += detections.size();

        if (!detections.isEmpty()) {
            // Process the best detection
            AprilTagDetection bestDetection = selectBestDetection(detections);

            if (bestDetection != null && isDetectionValid(bestDetection)) {
                updateFromDetection(bestDetection);
                validDetections++;
                lastUpdateTimer.reset();
                return true;
            }
        }

        // Check if position is getting stale using Tunables timeout
        if (hasValidPosition && lastUpdateTimer.seconds() > Tunables.COMMUNICATION_TIMEOUT) {
            hasValidPosition = false;
            positionConfidence = 0.0;

            if (Tunables.isDebugEnabled("vision")) {
                telemetry.addLine("⚠️ Vision position lost - timeout exceeded");
            }
        }

        return false;
    }

    /**
     * Select the best detection from multiple AprilTags
     */
    private AprilTagDetection selectBestDetection(List<AprilTagDetection> detections) {
        AprilTagDetection bestDetection = null;
        double bestScore = 0.0;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.robotPose != null) {
                // Skip tags that are too far away using Tunables
                double distance = Math.sqrt(
                    Math.pow(detection.robotPose.getPosition().x, 2) +
                    Math.pow(detection.robotPose.getPosition().y, 2) +
                    Math.pow(detection.robotPose.getPosition().z, 2)
                );

                if (distance > Tunables.VISION_MAX_DETECTION_DISTANCE) {
                    continue;
                }

                // Calculate detection quality score
                double score = calculateDetectionScore(detection);

                if (score > bestScore && score >= Tunables.VISION_DETECTION_CONFIDENCE_THRESHOLD) {
                    bestScore = score;
                    bestDetection = detection;
                }
            }
        }

        return bestDetection;
    }

    /**
     * Calculate quality score for a detection using Tunables thresholds
     */
    private double calculateDetectionScore(AprilTagDetection detection) {
        double score = 0.0;

        // Factor in detection decision margin (higher is better)
        if (detection.decisionMargin > 0) {
            score += Math.min(detection.decisionMargin / 100.0, 1.0) * 0.4;
        }

        // Factor in distance (closer is generally better within limits)
        if (detection.robotPose != null) {
            double distance = Math.sqrt(
                Math.pow(detection.robotPose.getPosition().x, 2) +
                Math.pow(detection.robotPose.getPosition().y, 2) +
                Math.pow(detection.robotPose.getPosition().z, 2)
            );

            // Optimal range based on Tunables max distance
            double optimalMin = Tunables.VISION_MAX_DETECTION_DISTANCE * 0.3;
            double optimalMax = Tunables.VISION_MAX_DETECTION_DISTANCE * 0.8;

            if (distance >= optimalMin && distance <= optimalMax) {
                score += 0.4;
            } else if (distance < optimalMin) {
                score += (distance / optimalMin) * 0.4;
            } else {
                score += Math.max(0, (Tunables.VISION_MAX_DETECTION_DISTANCE - distance) /
                         (Tunables.VISION_MAX_DETECTION_DISTANCE - optimalMax)) * 0.4;
            }
        }

        // Base score for any valid tag
        score += 0.2;

        return score;
    }

    /**
     * Validate a detection for position updates
     */
    private boolean isDetectionValid(AprilTagDetection detection) {
        if (detection.robotPose == null || detection.metadata == null) {
            return false;
        }

        double newX = detection.robotPose.getPosition().x;
        double newY = detection.robotPose.getPosition().y;

        // Check for reasonable position bounds (within field dimensions from Tunables)
        double fieldBound = Math.max(Tunables.FIELD_WIDTH, Tunables.FIELD_LENGTH) / 2.0;
        if (Math.abs(newX) > fieldBound || Math.abs(newY) > fieldBound) {
            return false;
        }

        // Check for reasonable position jumps
        if (hasValidPosition) {
            double positionJump = Math.sqrt(
                Math.pow(newX - robotX, 2) +
                Math.pow(newY - robotY, 2)
            );

            // Use a reasonable max jump based on max velocity and update period
            double maxJump = Tunables.AUTO_MAX_VELOCITY * Tunables.SENSOR_UPDATE_PERIOD * 2.0;
            if (positionJump > maxJump) {
                if (Tunables.isDebugEnabled("vision")) {
                    telemetry.addData("Vision Jump Rejected", "%.1f inches (max %.1f)", positionJump, maxJump);
                }
                return false;
            }
        }

        return true;
    }

    /**
     * Update robot position from a valid detection
     */
    private void updateFromDetection(AprilTagDetection detection) {
        double newX = detection.robotPose.getPosition().x;
        double newY = detection.robotPose.getPosition().y;
        double newHeading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Calculate position confidence based on detection quality
        positionConfidence = calculateDetectionScore(detection);

        // Apply smoothing if we have a previous position and confidence is not very high
        if (hasValidPosition && positionConfidence < 0.9) {
            // Use a smoothing factor based on confidence
            double smoothingFactor = positionConfidence * 0.5; // 0-50% new data blend
            robotX = robotX * (1 - smoothingFactor) + newX * smoothingFactor;
            robotY = robotY * (1 - smoothingFactor) + newY * smoothingFactor;
            robotHeading = normalizeAngle(robotHeading * (1 - smoothingFactor) + newHeading * smoothingFactor);
        } else {
            // High confidence or first detection - use directly
            robotX = newX;
            robotY = newY;
            robotHeading = newHeading;
        }

        hasValidPosition = true;

        if (Tunables.isDebugEnabled("vision")) {
            telemetry.addData("Vision Position", "X: %.1f, Y: %.1f, H: %.1f°", robotX, robotY, robotHeading);
            telemetry.addData("Vision Confidence", "%.2f", positionConfidence);
        }
    }

    /**
     * Get vision localization telemetry data
     */
    public VisionLocalizationData getTelemetryData() {
        return new VisionLocalizationData(
            robotX, robotY, robotHeading,
            hasValidPosition, getPositionConfidence(),
            totalDetections, validDetections,
            lastUpdateTimer.milliseconds(),
            performanceTimer.seconds(),
            isInitialized, isStreaming,
            Tunables.isDebugEnabled("vision")
        );
    }

    /**
     * Data container for vision localization telemetry
     */
    public static class VisionLocalizationData {
        public final double x, y, heading;
        public final boolean hasPosition;
        public final double confidence;
        public final int totalDetections, validDetections;
        public final double lastUpdateMs;
        public final double uptimeSeconds;
        public final boolean initialized, streaming;
        public final boolean debugEnabled;

        public VisionLocalizationData(double x, double y, double heading, boolean hasPosition,
                                     double confidence, int totalDetections, int validDetections,
                                     double lastUpdateMs, double uptimeSeconds,
                                     boolean initialized, boolean streaming, boolean debugEnabled) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.hasPosition = hasPosition;
            this.confidence = confidence;
            this.totalDetections = totalDetections;
            this.validDetections = validDetections;
            this.lastUpdateMs = lastUpdateMs;
            this.uptimeSeconds = uptimeSeconds;
            this.initialized = initialized;
            this.streaming = streaming;
            this.debugEnabled = debugEnabled;
        }

        public double getSuccessRate() {
            return totalDetections > 0 ? (double) validDetections / totalDetections : 0.0;
        }
    }

    /**
     * Update debug telemetry
     */
    private void updateDebugTelemetry() {
        VisionLocalizationData data = getTelemetryData();

        telemetry.addData("Vision Status", "%s | %s",
            data.initialized ? "INIT" : "FAIL",
            data.streaming ? "STREAM" : "STOPPED");

        if (data.hasPosition) {
            telemetry.addData("Vision Pos", "X: %.1f, Y: %.1f, H: %.1f°", data.x, data.y, data.heading);
            telemetry.addData("Vision Conf", "%.2f (%.1fms ago)", data.confidence, data.lastUpdateMs);
        } else {
            telemetry.addLine("Vision Pos: NO POSITION");
        }

        telemetry.addData("Vision Stats", "Det: %d/%d (%.1f%% success)",
            data.validDetections, data.totalDetections, data.getSuccessRate() * 100);
    }

    // === GETTERS AND SETTERS ===

    public boolean isInitialized() {
        return isInitialized && robotMap.isVisionSystemReady();
    }

    public boolean isStreaming() {
        return isStreaming;
    }

    public double[] getCurrentPosition() {
        return new double[]{robotX, robotY, robotHeading};
    }

    public boolean hasValidPosition() {
        return hasValidPosition && lastUpdateTimer.seconds() < Tunables.COMMUNICATION_TIMEOUT;
    }

    public double getPositionConfidence() {
        if (!hasValidPosition()) return 0.0;

        // Decay confidence over time
        double timeFactor = Math.max(0, 1.0 - (lastUpdateTimer.seconds() / Tunables.COMMUNICATION_TIMEOUT));
        return positionConfidence * timeFactor;
    }

    public double getRobotX() { return robotX; }
    public double getRobotY() { return robotY; }
    public double getRobotHeading() { return robotHeading; }

    /**
     * Control camera streaming
     */
    public void pauseStreaming() {
        if (visionPortal != null && isStreaming) {
            visionPortal.stopStreaming();
            isStreaming = false;
        }
    }

    public void resumeStreaming() {
        if (visionPortal != null && !isStreaming) {
            visionPortal.resumeStreaming();
            isStreaming = true;
        }
    }

    /**
     * Reset position tracking
     */
    public void resetPosition() {
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = 0.0;
        hasValidPosition = false;
        positionConfidence = 0.0;
        lastUpdateTimer.reset();

        if (Tunables.isDebugEnabled("vision")) {
            telemetry.addLine("🔄 Vision position reset");
        }
    }

    /**
     * Cleanup resources
     */
    public void shutdown() {
        if (visionPortal != null) {
            visionPortal.close();
            isStreaming = false;
        }
        isInitialized = false;
    }

    /**
     * Get status summary for debugging
     */
    public String getStatusSummary() {
        if (!isInitialized()) {
            return "VisionLocalizer: Not initialized - Vision system not ready";
        }

        return String.format("VisionLocalizer: %s, %.1f%% success, Conf: %.2f, Debug: %s",
            hasValidPosition() ? "TRACKING" : "SEARCHING",
            getTelemetryData().getSuccessRate() * 100,
            getPositionConfidence(),
            Tunables.isDebugEnabled("vision") ? "ON" : "OFF");
    }

    /**
     * Get current Tunables configuration summary for diagnostics
     */
    public String getTunablesInfo() {
        return String.format("Tunables: %dx%d@%dfps, Exp:%.1fms, Gain:%d, MaxDist:%d\"",
            Tunables.VISION_CAMERA_WIDTH,
            Tunables.VISION_CAMERA_HEIGHT,
            Tunables.VISION_CAMERA_FPS,
            Tunables.VISION_EXPOSURE_MS,
            Tunables.VISION_GAIN,
            Tunables.VISION_MAX_DETECTION_DISTANCE);
    }

    /**
     * Normalize angle to [-180, 180] degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
