package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

import java.util.Map;
import java.util.HashMap;

// Used to manage robot positioning systems like odometry and IMU for accurate movement in autonomous mode
//Collects data from EncoderManager and VisionManager to provide real-time position updates
public class AuroraPositioningManager {
    private EncoderManager encoderManager;
    private VisionManager visionManager;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Position data
    private double currentX, currentY, currentHeading;
    private double targetX, targetY, targetHeading;
    private boolean isPositionValid;

    // Vision correction settings
    private boolean useVisionCorrection;
    private double visionCorrectionThreshold = 6.0; // inches
    private long lastVisionUpdate = 0;
    private static final long VISION_UPDATE_INTERVAL = 500; // ms

    // Field map for known positions
    private FieldMap fieldMap;
    private FieldMap.Alliance alliance;

    /**
     * Initialize positioning manager
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param alliance Robot alliance (RED or BLUE)
     */
    public AuroraPositioningManager(HardwareMap hardwareMap, Telemetry telemetry, FieldMap.Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;

        // Initialize subsystems
        encoderManager = new EncoderManager(hardwareMap, telemetry);
        fieldMap = new FieldMap(alliance);

        // Initialize position
        currentX = 0;
        currentY = 0;
        currentHeading = 0;
        isPositionValid = false;
        useVisionCorrection = false;

        telemetry.addData("AuroraPositioning", "Initialized for " + alliance + " alliance");
    }

    /**
     * Initialize with vision system
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param alliance Robot alliance
     * @param useWebcam True for webcam, false for phone camera
     * @param cameraName Name of camera
     */
    public AuroraPositioningManager(HardwareMap hardwareMap, Telemetry telemetry,
                                   FieldMap.Alliance alliance, boolean useWebcam, String cameraName) {
        this(hardwareMap, telemetry, alliance);

        // Initialize vision
        visionManager = new VisionManager(hardwareMap, telemetry, useWebcam, cameraName);
        useVisionCorrection = true;

        telemetry.addData("AuroraPositioning", "Vision system enabled");
    }

    /**
     * Set starting position on the field
     * @param startX Starting X coordinate in inches
     * @param startY Starting Y coordinate in inches
     * @param startHeading Starting heading in degrees
     */
    public void setStartingPosition(double startX, double startY, double startHeading) {
        this.currentX = startX;
        this.currentY = startY;
        this.currentHeading = startHeading;

        // Initialize encoder system with starting position
        encoderManager.initialize(startX, startY, startHeading);

        // Set camera pose if vision is enabled
        if (visionManager != null) {
            visionManager.setCameraPose(0, 0, 8, 0, 0, 0); // Default camera position
            visionManager.startStreaming();
        }

        isPositionValid = true;
        telemetry.addData("AuroraPositioning", "Starting position set: (%.1f, %.1f) @ %.1f°",
                         startX, startY, startHeading);
    }

    /**
     * Update position data from all sources
     */
    public void updatePosition() {
        if (!isPositionValid) return;

        // Update encoder data
        encoderManager.update();

        // Get primary position from encoders
        currentX = encoderManager.getX();
        currentY = encoderManager.getY();
        currentHeading = encoderManager.getHeading();

        // Apply vision correction if enabled and enough time has passed
        if (useVisionCorrection && visionManager != null &&
            System.currentTimeMillis() - lastVisionUpdate > VISION_UPDATE_INTERVAL) {

            applyVisionCorrection();
            lastVisionUpdate = System.currentTimeMillis();
        }

        // Update vision system
        if (visionManager != null) {
            visionManager.updateVision();
        }
    }

    /**
     * Apply vision-based position correction using AprilTags
     */
    private void applyVisionCorrection() {
        if (visionManager == null || !visionManager.isVisionReady()) return;

        // Create a map of known AprilTag positions based on DECODE field layout
        Map<Integer, double[]> tagPositions = createAprilTagPositionMap();

        // Calculate robot position from vision
        double[] visionPosition = visionManager.getRobotPositionFromTags(tagPositions);

        if (visionPosition != null) {
            double visionX = visionPosition[0];
            double visionY = visionPosition[1];
            double visionHeading = visionPosition[2];

            // Calculate position error
            double errorX = Math.abs(currentX - visionX);
            double errorY = Math.abs(currentY - visionY);
            double totalError = Math.sqrt(errorX * errorX + errorY * errorY);

            // Apply correction if error exceeds threshold
            if (totalError > visionCorrectionThreshold) {
                // Weighted correction (favor encoder data but correct large errors)
                double correctionWeight = 0.3; // 30% vision, 70% encoder

                currentX = currentX * (1 - correctionWeight) + visionX * correctionWeight;
                currentY = currentY * (1 - correctionWeight) + visionY * correctionWeight;
                currentHeading = normalizeAngle(currentHeading * (1 - correctionWeight) + visionHeading * correctionWeight);

                // Update encoder manager with corrected position
                encoderManager.initialize(currentX, currentY, currentHeading);

                telemetry.addData("VisionCorrection", "Applied correction: %.1f in error", totalError);
            }
        }
    }

    /**
     * Create AprilTag position map based on DECODE field layout
     * @return Map of tag ID to [x, y] position
     */
    private Map<Integer, double[]> createAprilTagPositionMap() {
        Map<Integer, double[]> tagPositions = new HashMap<>();

        // DECODE season AprilTag positions (based on field specifications)
        // Goal AprilTags on back wall
        tagPositions.put(20, new double[]{-58.3727, -55.6425}); // Blue Goal
        tagPositions.put(24, new double[]{-58.3727, 55.6425});  // Red Goal

        // You can add more AprilTag positions here based on the actual field layout
        // For example, if there are tags on the loading zones or other field elements

        return tagPositions;
    }

    /**
     * Set target position for movement
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     * @param targetHeading Target heading in degrees
     */
    public void setTarget(double targetX, double targetY, double targetHeading) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = normalizeAngle(targetHeading);
    }

    /**
     * Get current X position
     * @return X position in inches
     */
    public double getCurrentX() {
        return currentX;
    }

    /**
     * Get current Y position
     * @return Y position in inches
     */
    public double getCurrentY() {
        return currentY;
    }

    /**
     * Get current heading
     * @return Heading in degrees
     */
    public double getCurrentHeading() {
        return currentHeading;
    }

    /**
     * Get distance to target position
     * @return Distance in inches
     */
    public double getDistanceToTarget() {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Get angle to target position
     * @return Angle in degrees
     */
    public double getAngleToTarget() {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }

    /**
     * Get heading error to target
     * @return Heading error in degrees (-180 to 180)
     */
    public double getHeadingError() {
        return normalizeAngle(targetHeading - currentHeading);
    }

    /**
     * Check if robot is at target position
     * @param positionTolerance Position tolerance in inches
     * @param headingTolerance Heading tolerance in degrees
     * @return True if within tolerance
     */
    public boolean isAtTarget(double positionTolerance, double headingTolerance) {
        double distanceError = getDistanceToTarget();
        double headingError = Math.abs(getHeadingError());

        return distanceError <= positionTolerance && headingError <= headingTolerance;
    }

    /**
     * Get current velocity
     * @return Array with [vx, vy, omega] in inches/sec and degrees/sec
     */
    public double[] getCurrentVelocity() {
        return new double[]{
            encoderManager.getVelocityX(),
            encoderManager.getVelocityY(),
            encoderManager.getAngularVelocity()
        };
    }

    /**
     * Get field position from named location
     * @param locationName Name of field location
     * @return Position array [x, y] or null if not found
     */
    public double[] getFieldPosition(String locationName) {
        FieldMap.FieldPosition position = fieldMap.getLocation(locationName);
        if (position != null) {
            return new double[]{position.x, position.y};
        }
        return null;
    }

    /**
     * Check if position is valid and tracking
     * @return True if position data is reliable
     */
    public boolean isPositionValid() {
        return isPositionValid && encoderManager.isReady();
    }

    /**
     * Reset position system
     */
    public void resetPosition() {
        encoderManager.resetEncoders();
        currentX = 0;
        currentY = 0;
        currentHeading = 0;
        isPositionValid = false;
    }

    /**
     * Enable or disable vision correction
     * @param enabled True to enable vision correction
     */
    public void setVisionCorrectionEnabled(boolean enabled) {
        this.useVisionCorrection = enabled && visionManager != null;
    }

    /**
     * Set vision correction threshold
     * @param threshold Error threshold in inches before applying correction
     */
    public void setVisionCorrectionThreshold(double threshold) {
        this.visionCorrectionThreshold = threshold;
    }

    /**
     * Normalize angle to -180 to 180 degrees
     * @param angle Angle in degrees
     * @return Normalized angle
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /**
     * Add positioning telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== POSITION DATA ===", "");
        telemetry.addData("Current Position", "(%.2f, %.2f) @ %.1f°",
                         currentX, currentY, currentHeading);
        telemetry.addData("Target Position", "(%.2f, %.2f) @ %.1f°",
                         targetX, targetY, targetHeading);
        telemetry.addData("Distance to Target", "%.2f in", getDistanceToTarget());
        telemetry.addData("Heading Error", "%.1f°", getHeadingError());
        telemetry.addData("Position Valid", isPositionValid ? "YES" : "NO");
        telemetry.addData("Vision Correction", useVisionCorrection ? "ENABLED" : "DISABLED");

        // Add subsystem telemetry
        encoderManager.addTelemetry();
        if (visionManager != null) {
            visionManager.addTelemetry();
        }
    }
}
