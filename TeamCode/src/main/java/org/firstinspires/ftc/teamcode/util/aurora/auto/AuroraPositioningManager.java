package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;

// Used to manage robot positioning systems like odometry and IMU for accurate movement in autonomous mode
// Integrates the new SimpleAuroraVisionManager with encoder data for real-time position updates and vision corrections
public class AuroraPositioningManager {
    private EncoderManager encoderManager;
    private SimpleAuroraVisionManager simpleVision; // REPLACED: Using simple vision instead of complex VisionManager
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
    private static final long VISION_UPDATE_INTERVAL = 200; // ms - faster updates for better integration

    // Field map for known positions
    private FieldMap fieldMap;
    private FieldMap.Alliance alliance;

    // Known AprilTag positions for vision positioning
    private Map<Integer, double[]> knownTagPositions;

    /**
     * Initialize positioning manager with encoder-only tracking
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

        // Load known tag positions
        loadKnownTagPositions();

        telemetry.addData("AuroraPositioning", "Initialized for " + alliance + " alliance");
    }

    /**
     * Initialize with vision system enabled
     */
    public AuroraPositioningManager(HardwareMap hardwareMap, Telemetry telemetry,
                                   FieldMap.Alliance alliance, boolean useWebcam, String cameraName) {
        this(hardwareMap, telemetry, alliance);

        // Initialize the new simple vision system
        simpleVision = new SimpleAuroraVisionManager(hardwareMap, telemetry, cameraName);
        simpleVision.startStreaming(); // Start streaming immediately
        useVisionCorrection = true;

        telemetry.addData("AuroraPositioning", "Simple vision system enabled with " + cameraName);
    }

    /**
     * Initialize with dual camera vision system
     */
    public AuroraPositioningManager(HardwareMap hardwareMap, Telemetry telemetry,
                                   FieldMap.Alliance alliance, String webcam1Name, String webcam2Name) {
        this(hardwareMap, telemetry, alliance);

        // Initialize the new simple vision system (use primary camera)
        simpleVision = new SimpleAuroraVisionManager(hardwareMap, telemetry, webcam1Name);
        simpleVision.startStreaming(); // Start streaming immediately
        useVisionCorrection = true;

        telemetry.addData("AuroraPositioning", "Simple vision system enabled with " + webcam1Name);
    }

    /**
     * Load known AprilTag positions from field map
     */
    private void loadKnownTagPositions() {
        knownTagPositions = new HashMap<>();

        // Load goal AprilTag positions from field map
        FieldMap.FieldPosition redGoal = fieldMap.getLocation("RED_GOAL");
        FieldMap.FieldPosition blueGoal = fieldMap.getLocation("BLUE_GOAL");

        if (redGoal != null) {
            knownTagPositions.put(24, new double[]{redGoal.x, redGoal.y}); // Red goal tag
        }

        if (blueGoal != null) {
            knownTagPositions.put(20, new double[]{blueGoal.x, blueGoal.y}); // Blue goal tag
        }

        // Add additional known tags based on your field setup
        telemetry.addData("Tag Positions", "Loaded %d AprilTag positions", knownTagPositions.size());
    }

    /**
     * Set starting position on the field
     */
    public void setStartingPosition(double startX, double startY, double startHeading) {
        this.currentX = startX;
        this.currentY = startY;
        this.currentHeading = startHeading;

        // Initialize encoder system with starting position
        encoderManager.initialize(startX, startY, startHeading);

        // Vision system is already initialized and streaming - no need to reconfigure
        // SimpleAuroraVisionManager sets camera pose during initialization like AprilTagMultiTool
        if (simpleVision != null) {
            // Camera is already configured and streaming - just ensure it's still active
            simpleVision.startStreaming();
        }

        isPositionValid = true;
        telemetry.addData("AuroraPositioning", "Starting position set: (%.1f, %.1f) @ %.1f°",
                         startX, startY, startHeading);
    }

    /**
     * Update position data from all sources - SIMPLIFIED
     * This now works exactly like SimpleVisionAuto - no complex vision management
     */
    public void updatePosition() {
        // SimpleVisionAuto doesn't call any "updateVision" method - it just gets detections directly
        // We'll follow the same pattern for Aurora

        // FIXED: Don't auto-initialize position during search phase
        // Let the FixedAuroraVisionAuto explicitly control when positioning starts
        // This prevents premature completion when tags are detected during search

        // Only try to initialize from vision if explicitly requested
        // if (!isPositionValid && simpleVision != null) {
        //     tryInitializeFromVision();
        // }

        // If we still don't have a valid position, that's OK - vision is still working
        if (!isPositionValid) {
            // Vision system is still active and can detect tags even without valid position
            telemetry.addData("UPDATE POS", "No valid position - skipping encoder updates");
            return;
        }

        // Store the position before encoder update for debugging
        double beforeX = currentX;
        double beforeY = currentY;
        double beforeHeading = currentHeading;

        telemetry.addData("UPDATE POS", "Before encoder update: (%.1f, %.1f) @ %.1f°",
                         beforeX, beforeY, beforeHeading);

        // Update encoder data (primary position source) only if we have valid position
        encoderManager.update();

        // Get primary position from encoders
        double encoderX = encoderManager.getX();
        double encoderY = encoderManager.getY();
        double encoderHeading = encoderManager.getHeading();

        telemetry.addData("UPDATE POS", "Encoder returned: (%.1f, %.1f) @ %.1f°",
                         encoderX, encoderY, encoderHeading);

        // PROBLEM: Don't immediately overwrite vision position with encoder data
        // Instead, check if encoder position is valid before using it
        if (encoderX == 0.0 && encoderY == 0.0 && encoderHeading == 0.0) {
            // Encoders are returning zero - keep vision position
            telemetry.addData("UPDATE POS", "WARNING: Encoders returning zero - keeping vision position");
        } else {
            // Encoders seem valid - use encoder data
            currentX = encoderX;
            currentY = encoderY;
            currentHeading = encoderHeading;
            telemetry.addData("UPDATE POS", "Using encoder position");
        }

        telemetry.addData("UPDATE POS", "Final position: (%.1f, %.1f) @ %.1f°",
                         currentX, currentY, currentHeading);

        // Apply vision correction if enabled and enough time has passed
        if (useVisionCorrection && simpleVision != null &&
            System.currentTimeMillis() - lastVisionUpdate > VISION_UPDATE_INTERVAL) {

            applyVisionCorrection();
            lastVisionUpdate = System.currentTimeMillis();
        }
    }

    /**
     * Try to initialize position from vision data - SIMPLIFIED
     */
    private void tryInitializeFromVision() {
        if (simpleVision == null) return;

        // Get detections directly like SimpleVisionAuto does
        List<AprilTagDetection> detections = simpleVision.getDetections();

        if (!detections.isEmpty()) {
            // Use the first detection to calculate robot position
            AprilTagDetection detection = detections.get(0);
            if (detection.ftcPose != null) {
                double[] visionPosition = calculateRobotPositionFromTag(detection);
                if (visionPosition != null) {
                    setStartingPosition(visionPosition[0], visionPosition[1], visionPosition[2]);
                    telemetry.addData("AuroraPositioning", "Position initialized from vision: (%.1f, %.1f) @ %.1f°",
                                     visionPosition[0], visionPosition[1], visionPosition[2]);
                }
            }
        }
    }

    /**
     * Calculate robot position from AprilTag detection - EXACTLY like SimpleVisionAuto
     */
    private double[] calculateRobotPositionFromTag(AprilTagDetection detection) {
        if (detection.ftcPose == null) return null;

        double range = detection.ftcPose.range;
        double bearing = detection.ftcPose.bearing;
        double yaw = detection.ftcPose.yaw;

        // Get known tag position
        double[] knownTagPos = knownTagPositions.get(detection.id);
        if (knownTagPos == null) return null;

        double tagX = knownTagPos[0];
        double tagY = knownTagPos[1];

        // Calculate robot position using EXACT same method as SimpleVisionAuto (rear-facing camera)
        double robotAngleFromTag = bearing + 180.0;
        double robotX = tagX + range * Math.cos(Math.toRadians(robotAngleFromTag));
        double robotY = tagY + range * Math.sin(Math.toRadians(robotAngleFromTag));
        double robotHeading = normalizeAngle(yaw + 180.0);

        return new double[]{robotX, robotY, robotHeading};
    }

    /**
     * Normalize angle to -180 to +180 degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Apply vision correction to encoder position - SIMPLIFIED
     */
    private void applyVisionCorrection() {
        if (simpleVision == null || !simpleVision.isReady()) return;

        // Get detections directly like SimpleVisionAuto does
        List<AprilTagDetection> detections = simpleVision.getDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            double[] visionPosition = calculateRobotPositionFromTag(detection);

            if (visionPosition != null) {
                double visionX = visionPosition[0];
                double visionY = visionPosition[1];
                double visionHeading = visionPosition[2];

                // Calculate difference between vision and encoder positions
                double deltaX = visionX - currentX;
                double deltaY = visionY - currentY;
                double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

                // Only apply correction if the difference is significant but not too large
                if (distance > visionCorrectionThreshold && distance < 24.0) {
                    // Apply weighted correction (favor encoders but incorporate vision)
                    double correctionWeight = 0.3; // 30% vision, 70% encoders

                    currentX += deltaX * correctionWeight;
                    currentY += deltaY * correctionWeight;

                    // Correct heading if difference is significant
                    double headingDiff = visionHeading - currentHeading;
                    while (headingDiff > 180) headingDiff -= 360;
                    while (headingDiff < -180) headingDiff += 360;

                    if (Math.abs(headingDiff) > 10.0) {
                        currentHeading += headingDiff * correctionWeight;
                    }

                    // Update encoder manager with corrected position
                    encoderManager.correctPosition(currentX, currentY, currentHeading);

                    telemetry.addData("Vision Correction", "Applied: Δ%.1f inches", distance);
                }
            }
        }
    }

    /**
     * Set target position for movement
     */
    public void setTarget(double targetX, double targetY, double targetHeading) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = targetHeading;
    }

    /**
     * Check if robot is at target position
     */
    public boolean isAtTarget(double positionTolerance, double headingTolerance) {
        double distanceToTarget = getDistanceToTarget();
        double headingError = Math.abs(getHeadingError());

        return distanceToTarget <= positionTolerance && headingError <= headingTolerance;
    }

    /**
     * Get distance to target position
     */
    public double getDistanceToTarget() {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Get angle to target position
     */
    public double getAngleToTarget() {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }

    /**
     * Get heading error to target
     */
    public double getHeadingError() {
        double error = targetHeading - currentHeading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    // Position getters
    public double getCurrentX() { return currentX; }
    public double getCurrentY() { return currentY; }
    public double getCurrentHeading() { return currentHeading; }
    public boolean isPositionValid() { return isPositionValid; }

    /**
     * Get current position as array [x, y, heading]
     */
    public double[] getCurrentPosition() {
        return new double[]{currentX, currentY, currentHeading};
    }

    /**
     * Check if vision system is available and ready
     */
    public boolean isVisionAvailable() {
        return simpleVision != null && simpleVision.isReady();
    }

    /**
     * Get number of detected AprilTags
     */
    public int getDetectedTagCount() {
        if (simpleVision != null && simpleVision.isReady()) {
            return simpleVision.getDetections().size();
        }
        return 0;
    }

    /**
     * Check for raw AprilTag detections (bypassing positioning logic)
     * This method is used for direct detection checking like in SimpleVisionAuto
     */
    public boolean hasRawAprilTagDetections() {
        if (simpleVision == null || !simpleVision.isReady()) {
            return false;
        }

        // Get detections directly like SimpleVisionAuto does
        List<AprilTagDetection> detections = simpleVision.getDetections();
        return !detections.isEmpty();
    }

    /**
     * Get raw AprilTag count (bypassing positioning logic)
     * This method provides direct access to tag detection count
     */
    public int getRawAprilTagCount() {
        if (simpleVision == null || !simpleVision.isReady()) {
            return 0;
        }

        // Get detections directly like SimpleVisionAuto does
        List<AprilTagDetection> detections = simpleVision.getDetections();
        return detections.size();
    }

    /**
     * Set manual camera exposure for better vision
     */
    public boolean setManualExposure(int exposureMS, int gain) {
        // This would need to be implemented in SimpleAuroraVisionManager if needed
        return false;
    }

    /**
     * Add positioning telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== POSITION DATA ===", "");
        telemetry.addData("Current Position", "(%.1f, %.1f) @ %.1f°", currentX, currentY, currentHeading);
        telemetry.addData("Position Valid", isPositionValid ? "YES" : "NO");
        telemetry.addData("Vision Correction", useVisionCorrection ? "ENABLED" : "DISABLED");

        if (isPositionValid) {
            telemetry.addData("Distance to Target", "%.1f inches", getDistanceToTarget());
            telemetry.addData("Heading Error", "%.1f degrees", getHeadingError());
        }

        // Add vision data if available
        if (simpleVision != null) {
            telemetry.addData("Vision Ready", simpleVision.isReady() ? "YES" : "NO");
            telemetry.addData("Tags Detected", getDetectedTagCount());

            // Show detected tag details
            List<AprilTagDetection> detections = simpleVision.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose != null) {
                    telemetry.addData("Tag " + detection.id,
                        "Dist:%.1f Bear:%.1f° Yaw:%.1f°",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.yaw);
                }
            }
        }

        // Add encoder data
        if (encoderManager != null) {
            encoderManager.addTelemetry();
        }
    }

    /**
     * Close positioning systems
     */
    public void close() {
        if (simpleVision != null) {
            simpleVision.close();
        }
    }

    /**
     * Get direct AprilTag detections - bypassing all Aurora vision management layers
     * This works exactly like SimpleVisionAuto's aprilTagTool.getDetections()
     */
    public List<AprilTagDetection> getDirectAprilTagDetections() {
        if (simpleVision == null || !simpleVision.isReady()) {
            return new ArrayList<>();
        }

        // Get detections directly - EXACT same call as SimpleVisionAuto
        return simpleVision.getDetections();
    }

    /**
     * Initialize position from vision data when explicitly requested
     * This should be called by FixedAuroraVisionAuto when ready to start navigation
     */
    public boolean initializePositionFromVision() {
        if (simpleVision == null) return false;

        // Get detections directly like SimpleVisionAuto does
        List<AprilTagDetection> detections = simpleVision.getDetections();

        if (!detections.isEmpty()) {
            // Use the first detection to calculate robot position
            AprilTagDetection detection = detections.get(0);
            if (detection.ftcPose != null) {
                double[] visionPosition = calculateRobotPositionFromTag(detection);
                if (visionPosition != null) {
                    // Store the vision-calculated position
                    double visionX = visionPosition[0];
                    double visionY = visionPosition[1];
                    double visionHeading = visionPosition[2];

                    telemetry.addData("VISION CALC", "Position from tag: (%.1f, %.1f) @ %.1f°",
                                     visionX, visionY, visionHeading);
                    telemetry.addData("VISION CALC", "Tag ID %d, Range %.1f, Bearing %.1f",
                                     detection.id, detection.ftcPose.range, detection.ftcPose.bearing);
                    telemetry.update();

                    // Initialize the positioning system with the calculated position
                    setStartingPosition(visionX, visionY, visionHeading);

                    // Verify the position was set correctly
                    telemetry.addData("VERIFICATION", "Set position: (%.1f, %.1f) @ %.1f°",
                                     currentX, currentY, currentHeading);
                    telemetry.addData("VERIFICATION", "Position valid: %s", isPositionValid ? "YES" : "NO");

                    return true;
                }
            }
        }
        return false;
    }
}
