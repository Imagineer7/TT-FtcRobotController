package org.firstinspires.ftc.teamcode.util.aurora.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.auto.VisionManager;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

import java.util.HashMap;
import java.util.Map;

/**
 * Aurora Vision Positioning System - Real-time position tracking using AprilTags
 *
 * Integrates with the rebuilt VisionManager to provide:
 * - Continuous position updates while moving (non-blocking)
 * - Fusion of vision and encoder data
 * - Field-aware position validation
 * - Real-time navigation corrections
 */
public class AuroraVisionPositioning {
    private VisionManager visionManager;
    private Telemetry telemetry;
    private FieldMap fieldMap;

    // Position tracking
    private double[] currentPosition; // [x, y, heading]
    private double[] lastVisionPosition;
    private long lastVisionUpdateTime;
    private boolean hasValidPosition;

    // Known AprilTag positions for position calculation
    private Map<Integer, double[]> knownTagPositions;

    // Configuration
    private static final double POSITION_VALIDATION_TOLERANCE = 24.0; // inches
    private static final double MAX_POSITION_JUMP = 36.0; // inches - max allowed position change
    private static final long MIN_VISION_UPDATE_INTERVAL = 100; // milliseconds

    /**
     * Initialize vision positioning system
     */
    public AuroraVisionPositioning(HardwareMap hardwareMap, Telemetry telemetry,
                                  FieldMap.Alliance alliance, String cameraName) {
        this.telemetry = telemetry;
        this.fieldMap = new FieldMap(alliance);

        // Initialize vision manager
        visionManager = new VisionManager(hardwareMap, telemetry, true, cameraName);

        // Initialize position tracking
        currentPosition = new double[]{0.0, 0.0, 0.0};
        lastVisionPosition = new double[]{0.0, 0.0, 0.0};
        lastVisionUpdateTime = 0;
        hasValidPosition = false;

        // Load known AprilTag positions from field map
        loadKnownTagPositions(alliance);

        // Set camera pose for rear-facing camera (matching AuroraVisionAuto)
        visionManager.setCameraPose(-8.5, 0.0, 10.0, 180.0, -15.0, 0.0);

        telemetry.addData("VisionPositioning", "Initialized with alliance: " + alliance);
    }

    /**
     * Load known AprilTag positions from field map
     */
    private void loadKnownTagPositions(FieldMap.Alliance alliance) {
        knownTagPositions = new HashMap<>();

        // Load goal AprilTag positions
        FieldMap.FieldPosition redGoal = fieldMap.getLocation("RED_GOAL");
        FieldMap.FieldPosition blueGoal = fieldMap.getLocation("BLUE_GOAL");

        if (redGoal != null) {
            knownTagPositions.put(24, new double[]{redGoal.x, redGoal.y}); // Red goal tag
        }

        if (blueGoal != null) {
            knownTagPositions.put(20, new double[]{blueGoal.x, blueGoal.y}); // Blue goal tag
        }

        // Add other known tags as needed for your field setup
        // Example: Wall tags, specimen tags, etc.

        telemetry.addData("Tag Positions", "Loaded %d AprilTag positions", knownTagPositions.size());
    }

    /**
     * Start vision system
     */
    public void startVision() {
        if (visionManager != null) {
            visionManager.startStreaming();
            telemetry.addData("VisionPositioning", "Vision streaming started");
        }
    }

    /**
     * Stop vision system
     */
    public void stopVision() {
        if (visionManager != null) {
            visionManager.stopStreaming();
            telemetry.addData("VisionPositioning", "Vision streaming stopped");
        }
    }

    /**
     * Update position using vision data - NON-BLOCKING, call frequently
     */
    public void updatePosition() {
        if (visionManager == null || !visionManager.isVisionReady()) return;

        // Update vision detections
        visionManager.updateVision();

        // Check if enough time has passed for a position update
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastVisionUpdateTime < MIN_VISION_UPDATE_INTERVAL) {
            return; // Too soon for another update
        }

        // Try to get robot position from detected AprilTags
        double[] visionPosition = visionManager.getRobotPositionFromTags(knownTagPositions);

        if (visionPosition != null) {
            // Validate the new position
            if (isPositionValid(visionPosition)) {
                // Update current position with vision data
                updateCurrentPosition(visionPosition);
                lastVisionUpdateTime = currentTime;
                hasValidPosition = true;

                // Store for validation of future readings
                lastVisionPosition = visionPosition.clone();
            }
        }
    }

    /**
     * Validate that a vision position reading is reasonable
     */
    private boolean isPositionValid(double[] position) {
        if (position == null || position.length < 3) return false;

        double x = position[0];
        double y = position[1];
        double heading = position[2];

        // Check if position is within field boundaries
        if (Math.abs(x) > 72 || Math.abs(y) > 72) {
            return false;
        }

        // Check if heading is valid
        if (heading < -180 || heading > 360) {
            return false;
        }

        // If we have a previous position, check for reasonable movement
        if (hasValidPosition) {
            double deltaX = x - lastVisionPosition[0];
            double deltaY = y - lastVisionPosition[1];
            double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            if (distance > MAX_POSITION_JUMP) {
                telemetry.addData("VisionPositioning", "Position jump too large: %.1f inches", distance);
                return false; // Position changed too much - likely error
            }
        }

        return true;
    }

    /**
     * Update current position with new vision data
     */
    private void updateCurrentPosition(double[] visionPosition) {
        // For now, directly use vision position
        // In future versions, could implement fusion with encoder data
        currentPosition[0] = visionPosition[0]; // x
        currentPosition[1] = visionPosition[1]; // y
        currentPosition[2] = visionPosition[2]; // heading
    }

    /**
     * Get current robot position
     */
    public double[] getCurrentPosition() {
        return hasValidPosition ? currentPosition.clone() : null;
    }

    /**
     * Get current robot X position
     */
    public double getCurrentX() {
        return hasValidPosition ? currentPosition[0] : 0.0;
    }

    /**
     * Get current robot Y position
     */
    public double getCurrentY() {
        return hasValidPosition ? currentPosition[1] : 0.0;
    }

    /**
     * Get current robot heading
     */
    public double getCurrentHeading() {
        return hasValidPosition ? currentPosition[2] : 0.0;
    }

    /**
     * Check if we have a valid position
     */
    public boolean hasValidPosition() {
        return hasValidPosition;
    }

    /**
     * Calculate distance to a target position
     */
    public double getDistanceToTarget(double targetX, double targetY) {
        if (!hasValidPosition) return -1.0;

        double deltaX = targetX - currentPosition[0];
        double deltaY = targetY - currentPosition[1];
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Calculate angle to a target position
     */
    public double getAngleToTarget(double targetX, double targetY) {
        if (!hasValidPosition) return 0.0;

        double deltaX = targetX - currentPosition[0];
        double deltaY = targetY - currentPosition[1];
        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }

    /**
     * Calculate turn needed to face a target
     */
    public double getTurnToTarget(double targetX, double targetY) {
        if (!hasValidPosition) return 0.0;

        double angleToTarget = getAngleToTarget(targetX, targetY);
        double turnNeeded = angleToTarget - currentPosition[2];

        // Normalize to -180 to +180
        while (turnNeeded > 180) turnNeeded -= 360;
        while (turnNeeded < -180) turnNeeded += 360;

        return turnNeeded;
    }

    /**
     * Get number of detected AprilTags
     */
    public int getDetectedTagCount() {
        return visionManager != null ? visionManager.getDetectedTagCount() : 0;
    }

    /**
     * Check if vision system is ready
     */
    public boolean isVisionReady() {
        return visionManager != null && visionManager.isVisionReady();
    }

    /**
     * Add telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== VISION POSITIONING ===", "");
        telemetry.addData("Vision Ready", isVisionReady() ? "YES" : "NO");
        telemetry.addData("Valid Position", hasValidPosition ? "YES" : "NO");
        telemetry.addData("Tags Detected", getDetectedTagCount());

        if (hasValidPosition) {
            telemetry.addData("Position", "(%.1f, %.1f) @ %.0f°",
                              currentPosition[0], currentPosition[1], currentPosition[2]);
        } else {
            telemetry.addData("Position", "UNKNOWN - Searching for tags...");
        }

        // Add vision manager telemetry
        if (visionManager != null) {
            visionManager.addTelemetry();
        }
    }

    /**
     * Set manual camera exposure and gain
     */
    public boolean setManualExposure(int exposureMS, int gain) {
        return visionManager != null && visionManager.setManualExposure(exposureMS, gain);
    }

    /**
     * Cleanup and close vision system
     */
    public void close() {
        if (visionManager != null) {
            visionManager.close();
        }
    }
}
