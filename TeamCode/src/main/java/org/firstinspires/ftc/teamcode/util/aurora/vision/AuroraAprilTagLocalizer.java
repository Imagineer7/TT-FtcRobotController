package org.firstinspires.ftc.teamcode.util.aurora.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Aurora AprilTag Localization System
 *
 * A dedicated high-precision AprilTag localization tool designed specifically for the Aurora system.
 * Features calibrated Logitech C290 support, real-time position tracking, and seamless Aurora integration.
 *
 * Key Features:
 * - Calibrated C290 webcam with C920 lens intrinsics
 * - Real-time position updates for Aurora positioning system
 * - Field-aware zone detection and validation
 * - High-frequency position updates (20Hz)
 * - Automatic camera optimization for AprilTag detection
 * - Position confidence scoring and outlier rejection
 *
 * @author FTC #26581 Tundra Tech
 * @version 2.0
 */
public class AuroraAprilTagLocalizer {

    // Hardware components
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private FieldMap fieldMap;

    // Camera configuration (calibrated for C290)
    private static final Position CAMERA_POSITION = new Position(DistanceUnit.INCH,
            19.5, 26.8, 0, 0); // Calibrated camera offset
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 0, 0); // Rear-facing camera

    // Camera settings optimized for C290
    private static final int EXPOSURE_MS = 10;
    private static final int GAIN_VALUE = 100;

    // Position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private boolean hasValidPosition = false;
    private double positionConfidence = 0.0;

    // Timing and performance
    private final ElapsedTime lastUpdateTimer = new ElapsedTime();
    private final ElapsedTime performanceTimer = new ElapsedTime();
    private int totalDetections = 0;
    private int validDetections = 0;

    // Configuration
    private static final double MIN_CONFIDENCE_THRESHOLD = 0.7;
    private static final double MAX_POSITION_JUMP = 48.0; // inches
    private static final long MAX_UPDATE_AGE_MS = 500;

    // Aurora integration
    private AuroraPositionCallback positionCallback;

    /**
     * Interface for Aurora system to receive position updates
     */
    public interface AuroraPositionCallback {
        void onPositionUpdate(double x, double y, double heading, double confidence, long timestamp);
        void onPositionLost();
    }

    /**
     * Initialize the Aurora AprilTag Localizer
     *
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param alliance Alliance color for field map
     * @param webcamName Name of the webcam in hardware config
     */
    public AuroraAprilTagLocalizer(HardwareMap hardwareMap, Telemetry telemetry,
                                  FieldMap.Alliance alliance, String webcamName) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.fieldMap = new FieldMap(alliance);

        initializeVisionSystem(webcamName);
        lastUpdateTimer.reset();
        performanceTimer.reset();
    }

    /**
     * Set callback for Aurora system position updates
     */
    public void setPositionCallback(AuroraPositionCallback callback) {
        this.positionCallback = callback;
    }

    /**
     * Initialize the vision system with calibrated C290 settings
     */
    private void initializeVisionSystem(String webcamName) {
        try {
            // Create AprilTag processor with calibrated lens intrinsics
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawTagOutline(true)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                    // C920 640x480 calibration values for C290
                    .setLensIntrinsics(622.001, 622.001, 319.803, 241.251)
                    .build();

            // Create vision portal with optimized settings
            VisionPortal.Builder builder = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                    .setCameraResolution(new android.util.Size(640, 480))
                    .addProcessor(aprilTagProcessor);

            visionPortal = builder.build();

            // Apply camera optimization
            applyCameraOptimization();

            telemetry.addData("Aurora Localizer", "Initialized with calibrated C290");

        } catch (Exception e) {
            telemetry.addData("Aurora Localizer Error", e.getMessage());
            visionPortal = null;
            aprilTagProcessor = null;
        }
    }

    /**
     * Apply optimized camera settings for C290 AprilTag detection
     */
    private void applyCameraOptimization() {
        try {
            // Wait for camera to be ready
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                Thread.sleep(20);
            }

            // Apply optimized exposure and gain settings
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            if (exposureControl != null) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            }

            if (gainControl != null) {
                gainControl.setGain(GAIN_VALUE);
            }

            telemetry.addData("Camera Optimization", "Applied - Exposure: %dms, Gain: %d", EXPOSURE_MS, GAIN_VALUE);

        } catch (Exception e) {
            telemetry.addData("Camera Optimization Error", e.getMessage());
        }
    }

    /**
     * Update position from AprilTag detections - call this in your main loop
     * Returns true if position was updated
     */
    public boolean updatePosition() {
        if (aprilTagProcessor == null) return false;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        totalDetections += detections.size();

        if (!detections.isEmpty()) {
            // Process the best detection
            AprilTagDetection bestDetection = selectBestDetection(detections);

            if (bestDetection != null && isDetectionValid(bestDetection)) {
                // Update position from detection
                updateFromDetection(bestDetection);
                validDetections++;
                lastUpdateTimer.reset();

                // Notify Aurora system
                if (positionCallback != null && hasValidPosition) {
                    positionCallback.onPositionUpdate(robotX, robotY, robotHeading,
                                                    positionConfidence, System.currentTimeMillis());
                }

                return true;
            }
        }

        // Check if position is getting stale
        if (hasValidPosition && lastUpdateTimer.milliseconds() > MAX_UPDATE_AGE_MS) {
            hasValidPosition = false;
            positionConfidence = 0.0;

            if (positionCallback != null) {
                positionCallback.onPositionLost();
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
                // Skip Obelisk tags
                if (detection.metadata.name.contains("Obelisk")) continue;

                // Calculate detection quality score
                double score = calculateDetectionScore(detection);

                if (score > bestScore) {
                    bestScore = score;
                    bestDetection = detection;
                }
            }
        }

        return bestDetection;
    }

    /**
     * Calculate quality score for a detection
     */
    private double calculateDetectionScore(AprilTagDetection detection) {
        double score = 0.0;

        // Factor in detection decision margin (higher is better)
        if (detection.decisionMargin > 0) {
            score += Math.min(detection.decisionMargin / 100.0, 1.0) * 0.4;
        }

        // Factor in distance (closer is generally better, but not too close)
        if (detection.robotPose != null) {
            double distance = Math.sqrt(
                Math.pow(detection.robotPose.getPosition().x, 2) +
                Math.pow(detection.robotPose.getPosition().y, 2) +
                Math.pow(detection.robotPose.getPosition().z, 2)
            );

            // Optimal range: 24-72 inches
            if (distance >= 24 && distance <= 72) {
                score += 0.4;
            } else if (distance < 24) {
                score += (distance / 24.0) * 0.4;
            } else {
                score += Math.max(0, (120 - distance) / 48.0) * 0.4;
            }
        }

        // Factor in tag ID (some tags might be more reliable)
        score += 0.2; // Base score for any valid tag

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

        // Check for reasonable position bounds (within field)
        if (Math.abs(newX) > 84 || Math.abs(newY) > 84) { // Field is roughly 144x144 inches
            return false;
        }

        // Check for reasonable position jumps
        if (hasValidPosition) {
            double positionJump = Math.sqrt(
                Math.pow(newX - robotX, 2) +
                Math.pow(newY - robotY, 2)
            );

            if (positionJump > MAX_POSITION_JUMP) {
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

        // Apply smoothing if we have a previous position
        if (hasValidPosition && positionConfidence < 0.9) {
            double smoothingFactor = 0.3; // Blend 30% new, 70% old
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
    }

    /**
     * Get current robot position for Aurora system
     */
    public double[] getCurrentPosition() {
        return new double[]{robotX, robotY, robotHeading};
    }

    /**
     * Get current robot position in path follower coordinates
     * Converts from AprilTag coordinates (center origin, ±72) to
     * path follower coordinates (bottom-left origin, 0-144)
     *
     * @return [x, y, heading] in path follower coordinate system
     */
    public double[] getCurrentPositionPathFollower() {
        double[] pathCoords = convertAprilTagToPathFollower(robotX, robotY);
        return new double[]{pathCoords[0], pathCoords[1], robotHeading};
    }

    /**
     * Convert AprilTag coordinates to path follower coordinates
     *
     * AprilTag system: Center of field is (0, 0), corners are (±72, ±72)
     * Path follower system: Bottom-left corner is (0, 0), top-right is (144, 144)
     *
     * @param aprilTagX X coordinate in AprilTag system
     * @param aprilTagY Y coordinate in AprilTag system
     * @return [x, y] in path follower coordinate system
     */
    public static double[] convertAprilTagToPathFollower(double aprilTagX, double aprilTagY) {
        // Add 72 to shift from center-origin to bottom-left origin
        double pathX = aprilTagX + 72.0;
        double pathY = aprilTagY + 72.0;
        return new double[]{pathX, pathY};
    }

    /**
     * Convert path follower coordinates to AprilTag coordinates
     *
     * Path follower system: Bottom-left corner is (0, 0), top-right is (144, 144)
     * AprilTag system: Center of field is (0, 0), corners are (±72, ±72)
     *
     * @param pathX X coordinate in path follower system
     * @param pathY Y coordinate in path follower system
     * @return [x, y] in AprilTag coordinate system
     */
    public static double[] convertPathFollowerToAprilTag(double pathX, double pathY) {
        // Subtract 72 to shift from bottom-left origin to center-origin
        double aprilTagX = pathX - 72.0;
        double aprilTagY = pathY - 72.0;
        return new double[]{aprilTagX, aprilTagY};
    }

    /**
     * Check if we have a valid position
     */
    public boolean hasValidPosition() {
        return hasValidPosition && lastUpdateTimer.milliseconds() < MAX_UPDATE_AGE_MS;
    }

    /**
     * Get position confidence (0.0 to 1.0)
     */
    public double getPositionConfidence() {
        if (!hasValidPosition()) return 0.0;

        // Decay confidence over time
        double timeFactor = Math.max(0, 1.0 - (lastUpdateTimer.milliseconds() / (double)MAX_UPDATE_AGE_MS));
        return positionConfidence * timeFactor;
    }

    /**
     * Get detailed status for Aurora integration
     */
    public AuroraLocalizationStatus getStatus() {
        return new AuroraLocalizationStatus(
            robotX, robotY, robotHeading,
            hasValidPosition(), getPositionConfidence(),
            totalDetections, validDetections,
            lastUpdateTimer.milliseconds(),
            performanceTimer.seconds()
        );
    }

    /**
     * Get field zone information
     */
    public String getCurrentZone() {
        if (!hasValidPosition()) return "UNKNOWN";
        return fieldMap.getZoneInfo(robotX, robotY);
    }

    /**
     * Cleanup resources
     */
    public void shutdown() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Control camera streaming
     */
    public void pauseStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    /**
     * Normalize angle to [-180, 180] degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Status class for Aurora integration
     */
    public static class AuroraLocalizationStatus {
        public final double x, y, heading;
        public final boolean hasPosition;
        public final double confidence;
        public final int totalDetections, validDetections;
        public final double lastUpdateMs;
        public final double uptimeSeconds;

        public AuroraLocalizationStatus(double x, double y, double heading, boolean hasPosition,
                                      double confidence, int totalDetections, int validDetections,
                                      double lastUpdateMs, double uptimeSeconds) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.hasPosition = hasPosition;
            this.confidence = confidence;
            this.totalDetections = totalDetections;
            this.validDetections = validDetections;
            this.lastUpdateMs = lastUpdateMs;
            this.uptimeSeconds = uptimeSeconds;
        }

        public double getSuccessRate() {
            return totalDetections > 0 ? (double)validDetections / totalDetections : 0.0;
        }
    }
}
