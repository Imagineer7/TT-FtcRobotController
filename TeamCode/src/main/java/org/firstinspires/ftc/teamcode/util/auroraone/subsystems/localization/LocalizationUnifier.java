package org.firstinspires.ftc.teamcode.util.auroraone.subsystems.localization;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.auroraone.config.RobotMap;
import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;
import org.firstinspires.ftc.teamcode.util.auroraone.core.Blackboard;
import org.firstinspires.ftc.teamcode.util.tool.DeadWheelOdometry;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

/**
 * AURORA ONE - Localization Unifier
 * Advanced sensor fusion system combining vision and odometry for robust robot localization
 *
 * This class is responsible for managing and fusing data from multiple localization sources
 * to provide the most accurate and reliable position information for the state machine.
 * It handles sensor fusion, validation, and field zone detection.
 *
 * Features:
 * - Multiple fusion modes: Sensor Fusion, Odometry+Vision Init, Odometry Only
 * - Weighted sensor fusion using Tunables configuration
 * - Robust error detection and recovery
 * - Field zone detection and validation
 * - Position constraints and sanity checking
 * - Integration with Aurora One State Machine system
 */
public class LocalizationUnifier implements VisionLocalizer.LocalizationCallback {

    // Hardware and subsystem references
    private RobotMap robotMap;
    private Telemetry telemetry;
    private Blackboard blackboard;
    private VisionLocalizer visionLocalizer;
    private DeadWheelOdometry odometry;
    private FieldMap fieldMap;

    // Localization modes
    public enum LocalizationMode {
        SENSOR_FUSION(Tunables.LOCALIZATION_MODE_SENSOR_FUSION, "Sensor Fusion", "Weighted fusion of vision and odometry"),
        ODOMETRY_VISION_INIT(Tunables.LOCALIZATION_MODE_ODOMETRY_VISION_INIT, "Odometry+Vision Init", "Vision initialization, odometry primary with vision backup"),
        ODOMETRY_ONLY(Tunables.LOCALIZATION_MODE_ODOMETRY_ONLY, "Odometry Only", "Odometry only with custom start position");

        private final int modeId;
        private final String name;
        private final String description;

        LocalizationMode(int modeId, String name, String description) {
            this.modeId = modeId;
            this.name = name;
            this.description = description;
        }

        public int getModeId() { return modeId; }
        public String getName() { return name; }
        public String getDescription() { return description; }

        public static LocalizationMode fromId(int id) {
            for (LocalizationMode mode : values()) {
                if (mode.modeId == id) return mode;
            }
            return SENSOR_FUSION; // Default fallback
        }
    }

    // State management
    private LocalizationMode currentMode;
    private boolean isInitialized = false;
    private boolean visionInitialized = false;
    private boolean odometryInitialized = false;

    // Unified position state
    private double unifiedX = 0.0;
    private double unifiedY = 0.0;
    private double unifiedHeading = 0.0;
    private double positionConfidence = 0.0;
    private long lastUpdateTimestamp = 0;

    // Sensor data caching
    private double[] lastVisionPosition = {0, 0, 0}; // x, y, heading
    private double lastVisionConfidence = 0.0;
    private long lastVisionTimestamp = 0;

    private double[] lastOdometryPosition = {0, 0, 0}; // x, y, heading
    private double[] lastOdometryVelocity = {0, 0, 0}; // vx, vy, omega
    private long lastOdometryTimestamp = 0;

    // Timing and performance
    private ElapsedTime fusionTimer = new ElapsedTime();
    private ElapsedTime telemetryTimer = new ElapsedTime();
    private ElapsedTime performanceTimer = new ElapsedTime();

    // Validation and safety
    private boolean robotTipDetected = false;
    private int visionDetectionCount = 0;
    private double[] previousPosition = {0, 0, 0};
    private long previousPositionTimestamp = 0;

    // Statistics and monitoring
    private int totalFusionUpdates = 0;
    private int visionBasedUpdates = 0;
    private int odometryOnlyUpdates = 0;
    private double averageFusionTime = 0.0;

    /**
     * Constructor - Initialize with RobotMap and alliance
     */
    public LocalizationUnifier(RobotMap robotMap, Telemetry telemetry, FieldMap.Alliance alliance) {
        this.robotMap = robotMap;
        this.telemetry = telemetry;
        this.blackboard = Blackboard.getInstance();

        // Initialize field map with alliance
        this.fieldMap = new FieldMap(alliance);

        // Set default mode from Tunables
        this.currentMode = LocalizationMode.fromId(Tunables.LOCALIZATION_DEFAULT_MODE);

        initialize();
    }

    /**
     * Initialize the localization unifier system
     */
    private void initialize() {
        try {
            // Initialize VisionLocalizer if available
            if (robotMap.isVisionSystemReady()) {
                visionLocalizer = new VisionLocalizer(robotMap, telemetry);
                visionLocalizer.setLocalizationCallback(this);
                visionInitialized = visionLocalizer.isInitialized();

                if (visionInitialized) {
                    telemetry.addLine("✅ LocalizationUnifier: Vision system initialized");
                } else {
                    telemetry.addLine("⚠️ LocalizationUnifier: Vision system failed to initialize");
                }
            } else {
                telemetry.addLine("⚠️ LocalizationUnifier: Vision system not available");
            }

            // Initialize DeadWheelOdometry
            try {
                odometry = new DeadWheelOdometry(robotMap.getHardwareMap(), telemetry);
                odometryInitialized = odometry.isReady();

                if (odometryInitialized) {
                    telemetry.addLine("✅ LocalizationUnifier: Odometry system initialized");
                } else {
                    telemetry.addLine("⚠️ LocalizationUnifier: Odometry system not ready");
                }
            } catch (Exception e) {
                telemetry.addLine("❌ LocalizationUnifier: Odometry initialization failed - " + e.getMessage());
                odometryInitialized = false;
            }

            // Validate mode compatibility
            validateModeCompatibility();

            // Set initial position based on mode
            setInitialPosition();

            isInitialized = (visionInitialized || odometryInitialized);
            fusionTimer.reset();
            telemetryTimer.reset();
            performanceTimer.reset();

            telemetry.addLine("🤖 LocalizationUnifier: Initialized in " + currentMode.getName() + " mode");

        } catch (Exception e) {
            isInitialized = false;
            telemetry.addLine("❌ LocalizationUnifier: Initialization failed - " + e.getMessage());
        }
    }

    /**
     * Validate that current mode is compatible with available sensors
     */
    private void validateModeCompatibility() {
        switch (currentMode) {
            case SENSOR_FUSION:
                if (!visionInitialized && !odometryInitialized) {
                    telemetry.addLine("⚠️ Sensor Fusion mode requires at least one sensor - falling back to available sensor");
                    if (odometryInitialized) {
                        currentMode = LocalizationMode.ODOMETRY_ONLY;
                    }
                }
                break;

            case ODOMETRY_VISION_INIT:
                if (!odometryInitialized) {
                    telemetry.addLine("⚠️ Odometry+Vision Init mode requires odometry - falling back to vision only");
                    if (visionInitialized) {
                        currentMode = LocalizationMode.SENSOR_FUSION;
                    }
                }
                break;

            case ODOMETRY_ONLY:
                if (!odometryInitialized) {
                    telemetry.addLine("❌ Odometry Only mode requires odometry system");
                    if (visionInitialized) {
                        currentMode = LocalizationMode.SENSOR_FUSION;
                    }
                }
                break;
        }

        telemetry.addData("Localization Mode", currentMode.getName());
    }

    /**
     * Set initial position based on current mode
     */
    private void setInitialPosition() {
        switch (currentMode) {
            case SENSOR_FUSION:
                // Start with default position, will be updated by sensors
                setPosition(Tunables.LOCALIZATION_DEFAULT_START_X,
                          Tunables.LOCALIZATION_DEFAULT_START_Y,
                          Tunables.LOCALIZATION_DEFAULT_START_HEADING);
                break;

            case ODOMETRY_VISION_INIT:
                // Wait for vision to provide initial position, then use odometry
                if (visionInitialized) {
                    // Vision will provide initial position via callback
                    telemetry.addLine("🎥 Waiting for vision to provide initial position...");
                } else {
                    // Fallback to default
                    setPosition(Tunables.LOCALIZATION_DEFAULT_START_X,
                              Tunables.LOCALIZATION_DEFAULT_START_Y,
                              Tunables.LOCALIZATION_DEFAULT_START_HEADING);
                }
                break;

            case ODOMETRY_ONLY:
                // Use default start position for odometry
                setPosition(Tunables.LOCALIZATION_DEFAULT_START_X,
                          Tunables.LOCALIZATION_DEFAULT_START_Y,
                          Tunables.LOCALIZATION_DEFAULT_START_HEADING);
                break;
        }
    }

    /**
     * Main update method - call this every loop cycle
     */
    public void update() {
        if (!isInitialized) return;

        long currentTime = System.currentTimeMillis();

        // Update individual sensors
        updateSensors();

        // Perform sensor fusion based on current mode
        if (fusionTimer.seconds() >= Tunables.LOCALIZATION_FUSION_UPDATE_RATE) {
            performSensorFusion();
            fusionTimer.reset();
        }

        // Validate position and apply constraints
        validateAndConstrainPosition();

        // Update statistics
        updateStatistics();

        // Update telemetry
        if (telemetryTimer.seconds() >= Tunables.LOCALIZATION_TELEMETRY_UPDATE_RATE) {
            updateTelemetry();
            telemetryTimer.reset();
        }

        // Update Blackboard with localization data
        updateBlackboard();

        lastUpdateTimestamp = currentTime;
    }

    /**
     * Update Blackboard with current localization system state
     */
    private void updateBlackboard() {
        // Robot position and orientation
        blackboard.put("robot.position.x", unifiedX);
        blackboard.put("robot.position.y", unifiedY);
        blackboard.put("robot.position.heading", unifiedHeading);
        blackboard.put("robot.position.confidence", positionConfidence);
        blackboard.put("robot.position.last_update", lastUpdateTimestamp);

        // Robot velocity (from odometry if available)
        if (hasValidOdometryData() && lastOdometryVelocity != null) {
            blackboard.put("robot.velocity.x", lastOdometryVelocity[0]);
            blackboard.put("robot.velocity.y", lastOdometryVelocity[1]);
            blackboard.put("robot.velocity.angular", lastOdometryVelocity[2]);
        }

        // Localization system status
        blackboard.put("localization.mode", currentMode.getName());
        blackboard.put("localization.initialized", isInitialized);
        blackboard.put("localization.vision_available", visionInitialized);
        blackboard.put("localization.odometry_available", odometryInitialized);
        blackboard.put("localization.robot_tipped", robotTipDetected);

        // Field zone information
        blackboard.put("robot.field.zone", getCurrentZone());

        // Vision system data
        blackboard.put("vision.target.detected", hasValidVisionData());
        if (hasValidVisionData()) {
            blackboard.put("vision.target.x", lastVisionPosition[0]);
            blackboard.put("vision.target.y", lastVisionPosition[1]);
            blackboard.put("vision.target.confidence", lastVisionConfidence);
        }

        // Localization statistics
        blackboard.put("localization.stats.total_updates", totalFusionUpdates);
        blackboard.put("localization.stats.vision_updates", visionBasedUpdates);
        blackboard.put("localization.stats.odometry_updates", odometryOnlyUpdates);
        blackboard.put("localization.stats.average_fusion_time", averageFusionTime);

        // Check for position reset commands from Blackboard
        if (blackboard.containsKey("localization.command.reset_position")) {
            double resetX = blackboard.get("localization.command.reset_x", 0.0);
            double resetY = blackboard.get("localization.command.reset_y", 0.0);
            double resetHeading = blackboard.get("localization.command.reset_heading", 0.0);

            setPosition(resetX, resetY, resetHeading);

            // Clear the command
            blackboard.remove("localization.command.reset_position");
            blackboard.remove("localization.command.reset_x");
            blackboard.remove("localization.command.reset_y");
            blackboard.remove("localization.command.reset_heading");
        }
    }

    /**
     * Update individual sensor systems
     */
    private void updateSensors() {
        // Update vision localizer
        if (visionInitialized && visionLocalizer != null) {
            visionLocalizer.update();
        }

        // Update odometry
        if (odometryInitialized && odometry != null) {
            odometry.updatePosition();

            // Cache odometry data
            lastOdometryPosition[0] = odometry.getX();
            lastOdometryPosition[1] = odometry.getY();
            lastOdometryPosition[2] = odometry.getHeadingDegrees();
            lastOdometryVelocity = odometry.getVelocity();
            lastOdometryTimestamp = System.currentTimeMillis();

            // Check for robot tip detection
            checkForRobotTip();
        }
    }

    /**
     * Perform sensor fusion based on current mode
     */
    private void performSensorFusion() {
        long fusionStartTime = System.nanoTime();

        switch (currentMode) {
            case SENSOR_FUSION:
                performWeightedSensorFusion();
                break;

            case ODOMETRY_VISION_INIT:
                performOdometryWithVisionBackup();
                break;

            case ODOMETRY_ONLY:
                performOdometryOnlyUpdate();
                break;
        }

        // Update fusion performance metrics
        double fusionTime = (System.nanoTime() - fusionStartTime) / 1e6; // Convert to milliseconds
        averageFusionTime = (averageFusionTime * totalFusionUpdates + fusionTime) / (totalFusionUpdates + 1);
        totalFusionUpdates++;

        // Update unified position timestamp
        lastUpdateTimestamp = System.currentTimeMillis();
    }

    /**
     * Perform weighted sensor fusion (Mode 1)
     */
    private void performWeightedSensorFusion() {
        boolean hasValidVision = hasValidVisionData();
        boolean hasValidOdometry = hasValidOdometryData();

        if (!hasValidVision && !hasValidOdometry) {
            // No valid data available
            positionConfidence = 0.0;
            return;
        }

        if (hasValidVision && hasValidOdometry) {
            // Fuse both sensors using weights from Tunables
            double[] weights = Tunables.getNormalizedFusionWeights();

            // Adjust weights based on vision confidence
            double visionWeight = weights[0] * lastVisionConfidence;
            double odometryWeight = weights[1];

            // Normalize adjusted weights
            double totalWeight = visionWeight + odometryWeight;
            if (totalWeight > 0) {
                visionWeight /= totalWeight;
                odometryWeight /= totalWeight;

                // Fuse positions
                unifiedX = visionWeight * lastVisionPosition[0] + odometryWeight * lastOdometryPosition[0];
                unifiedY = visionWeight * lastVisionPosition[1] + odometryWeight * lastOdometryPosition[1];
                unifiedHeading = normalizeAngle(visionWeight * lastVisionPosition[2] + odometryWeight * lastOdometryPosition[2]);

                // Calculate unified confidence
                positionConfidence = Math.min(1.0, lastVisionConfidence * visionWeight + 0.9 * odometryWeight);

                visionBasedUpdates++;

                if (Tunables.LOCALIZATION_DETAILED_LOGGING) {
                    telemetry.addData("Fusion Weights", "V:%.2f O:%.2f", visionWeight, odometryWeight);
                }
            }
        } else if (hasValidVision) {
            // Use vision only
            unifiedX = lastVisionPosition[0];
            unifiedY = lastVisionPosition[1];
            unifiedHeading = lastVisionPosition[2];
            positionConfidence = lastVisionConfidence;

            visionBasedUpdates++;
        } else {
            // Use odometry only
            unifiedX = lastOdometryPosition[0];
            unifiedY = lastOdometryPosition[1];
            unifiedHeading = lastOdometryPosition[2];
            positionConfidence = 0.8; // High confidence in odometry

            odometryOnlyUpdates++;
        }
    }

    /**
     * Perform odometry with vision backup (Mode 2)
     */
    private void performOdometryWithVisionBackup() {
        if (hasValidOdometryData() && !robotTipDetected) {
            // Use odometry as primary
            unifiedX = lastOdometryPosition[0];
            unifiedY = lastOdometryPosition[1];
            unifiedHeading = lastOdometryPosition[2];
            positionConfidence = 0.9; // High confidence in odometry

            odometryOnlyUpdates++;

            // Sync odometry with vision occasionally for drift correction
            if (hasValidVisionData() && lastVisionConfidence > Tunables.LOCALIZATION_CONFIDENCE_THRESHOLD) {
                double positionDrift = Math.sqrt(
                    Math.pow(lastVisionPosition[0] - lastOdometryPosition[0], 2) +
                    Math.pow(lastVisionPosition[1] - lastOdometryPosition[1], 2)
                );

                // If drift is significant, correct odometry
                if (positionDrift > 12.0) { // 12 inch drift threshold
                    if (odometry != null) {
                        odometry.setPosition(lastVisionPosition[0], lastVisionPosition[1], lastVisionPosition[2]);
                        telemetry.addLine("🔄 Odometry corrected by vision (drift: " + String.format("%.1f", positionDrift) + "\")");
                    }
                }
            }
        } else if (hasValidVisionData()) {
            // Fallback to vision if odometry is unavailable or robot is tipped
            unifiedX = lastVisionPosition[0];
            unifiedY = lastVisionPosition[1];
            unifiedHeading = lastVisionPosition[2];
            positionConfidence = lastVisionConfidence;

            visionBasedUpdates++;

            if (robotTipDetected) {
                telemetry.addLine("⚠️ Robot tip detected - using vision backup");
            }
        } else {
            // No reliable data available
            positionConfidence = 0.0;
        }
    }

    /**
     * Perform odometry only update (Mode 3)
     */
    private void performOdometryOnlyUpdate() {
        if (hasValidOdometryData()) {
            unifiedX = lastOdometryPosition[0];
            unifiedY = lastOdometryPosition[1];
            unifiedHeading = lastOdometryPosition[2];
            positionConfidence = robotTipDetected ? 0.3 : 0.9; // Lower confidence if tipped

            odometryOnlyUpdates++;
        } else {
            positionConfidence = 0.0;
        }
    }

    /**
     * Check for robot tip detection using velocity analysis
     */
    private void checkForRobotTip() {
        if (lastOdometryVelocity == null) return;

        // Check for unreasonable velocities that might indicate a tip
        double totalVelocity = Math.sqrt(
            lastOdometryVelocity[0] * lastOdometryVelocity[0] +
            lastOdometryVelocity[1] * lastOdometryVelocity[1]
        );

        double angularVelocity = Math.abs(lastOdometryVelocity[2]);

        // Detect tip based on velocity thresholds
        boolean velocityTip = totalVelocity > Tunables.LOCALIZATION_VELOCITY_SANITY_CHECK;
        boolean angularTip = angularVelocity > Tunables.LOCALIZATION_TIP_DETECTION_THRESHOLD;

        robotTipDetected = velocityTip || angularTip;

        if (robotTipDetected && Tunables.LOCALIZATION_DETAILED_LOGGING) {
            telemetry.addData("Tip Detection", "V:%.1f A:%.1f", totalVelocity, angularVelocity);
        }
    }

    /**
     * Validate and constrain position to reasonable bounds
     */
    private void validateAndConstrainPosition() {
        // Check for unreasonable position jumps
        if (previousPositionTimestamp > 0) {
            double deltaTime = (lastUpdateTimestamp - previousPositionTimestamp) / 1000.0; // Convert to seconds
            if (deltaTime > 0) {
                double positionJump = Math.sqrt(
                    Math.pow(unifiedX - previousPosition[0], 2) +
                    Math.pow(unifiedY - previousPosition[1], 2)
                );

                double impliedVelocity = positionJump / deltaTime;

                // Check if position jump is reasonable
                if (positionJump > Tunables.LOCALIZATION_MAX_POSITION_JUMP) {
                    // Reject unreasonable jump
                    unifiedX = previousPosition[0];
                    unifiedY = previousPosition[1];
                    unifiedHeading = previousPosition[2];
                    positionConfidence *= 0.5; // Reduce confidence

                    if (Tunables.LOCALIZATION_DETAILED_LOGGING) {
                        telemetry.addData("Position Jump Rejected", "%.1f inches", positionJump);
                    }
                }
            }
        }

        // Apply field constraints if enabled
        if (Tunables.LOCALIZATION_USE_FIELD_CONSTRAINTS) {
            double fieldMargin = 6.0; // 6 inch margin from field edge
            unifiedX = Math.max(-fieldMargin, Math.min(Tunables.FIELD_WIDTH + fieldMargin, unifiedX));
            unifiedY = Math.max(-fieldMargin, Math.min(Tunables.FIELD_LENGTH + fieldMargin, unifiedY));
        }

        // Normalize heading
        unifiedHeading = normalizeAngle(unifiedHeading);

        // Store previous position for next validation
        previousPosition[0] = unifiedX;
        previousPosition[1] = unifiedY;
        previousPosition[2] = unifiedHeading;
        previousPositionTimestamp = lastUpdateTimestamp;
    }

    /**
     * Check if vision data is valid and recent
     */
    private boolean hasValidVisionData() {
        if (!visionInitialized || visionLocalizer == null) return false;

        long age = System.currentTimeMillis() - lastVisionTimestamp;
        boolean isRecent = age < (Tunables.LOCALIZATION_VISION_TIMEOUT * 1000);
        boolean hasMinDetections = visionDetectionCount >= Tunables.LOCALIZATION_MIN_VISION_DETECTIONS;
        boolean hasMinConfidence = lastVisionConfidence >= Tunables.LOCALIZATION_MIN_VISION_CONFIDENCE;

        return isRecent && hasMinDetections && hasMinConfidence;
    }

    /**
     * Check if odometry data is valid and recent
     */
    private boolean hasValidOdometryData() {
        if (!odometryInitialized || odometry == null) return false;

        long age = System.currentTimeMillis() - lastOdometryTimestamp;
        boolean isRecent = age < (Tunables.LOCALIZATION_STALENESS_TIMEOUT * 1000);

        return isRecent && odometry.isReady();
    }

    /**
     * Update performance statistics
     */
    private void updateStatistics() {
        // Performance statistics are updated in performSensorFusion()
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        if (!Tunables.isDebugEnabled("sensors") && !Tunables.LOCALIZATION_DETAILED_LOGGING) {
            return; // Skip telemetry if debug not enabled
        }

        telemetry.addLine("=== LOCALIZATION UNIFIER ===");
        telemetry.addData("Mode", currentMode.getName());
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", unifiedX, unifiedY, unifiedHeading);
        telemetry.addData("Confidence", "%.2f", positionConfidence);
        telemetry.addData("Zone", getCurrentZone());

        // Sensor status
        telemetry.addData("Vision", "%s (%.2f conf)",
            visionInitialized ? (hasValidVisionData() ? "GOOD" : "STALE") : "UNAVAIL",
            lastVisionConfidence);
        telemetry.addData("Odometry", "%s%s",
            odometryInitialized ? (hasValidOdometryData() ? "GOOD" : "STALE") : "UNAVAIL",
            robotTipDetected ? " [TIPPED]" : "");

        // Statistics
        telemetry.addData("Updates", "Total:%d Vision:%d Odom:%d",
            totalFusionUpdates, visionBasedUpdates, odometryOnlyUpdates);
        telemetry.addData("Fusion Time", "%.1fms avg", averageFusionTime);
    }

    // === VisionLocalizer.LocalizationCallback Implementation ===

    @Override
    public void onVisionPositionUpdate(double x, double y, double heading, double confidence, long timestamp) {
        lastVisionPosition[0] = x;
        lastVisionPosition[1] = y;
        lastVisionPosition[2] = heading;
        lastVisionConfidence = confidence;
        lastVisionTimestamp = timestamp;
        visionDetectionCount++;

        // For ODOMETRY_VISION_INIT mode, use first good vision reading to initialize odometry
        if (currentMode == LocalizationMode.ODOMETRY_VISION_INIT && odometryInitialized &&
            visionDetectionCount == Tunables.LOCALIZATION_MIN_VISION_DETECTIONS &&
            confidence > Tunables.LOCALIZATION_CONFIDENCE_THRESHOLD) {

            odometry.setPosition(x, y, heading);
            telemetry.addLine("🎯 Odometry initialized with vision position");
        }
    }

    @Override
    public void onVisionPositionLost() {
        // Vision data is now stale - handled in hasValidVisionData()
        if (Tunables.LOCALIZATION_DETAILED_LOGGING) {
            telemetry.addLine("⚠️ Vision position lost");
        }
    }

    // === Public Interface Methods ===

    /**
     * Get current unified position
     */
    public double[] getCurrentPosition() {
        return new double[]{unifiedX, unifiedY, unifiedHeading};
    }

    public double getX() { return unifiedX; }
    public double getY() { return unifiedY; }
    public double getHeading() { return unifiedHeading; }
    public double getPositionConfidence() { return positionConfidence; }

    /**
     * Set robot position (affects both vision and odometry systems)
     */
    public void setPosition(double x, double y, double heading) {
        unifiedX = x;
        unifiedY = y;
        unifiedHeading = normalizeAngle(heading);

        // Update odometry if available
        if (odometryInitialized && odometry != null) {
            odometry.setPosition(x, y, heading);
        }

        // Reset vision localizer if available
        if (visionInitialized && visionLocalizer != null) {
            visionLocalizer.resetPosition();
        }

        positionConfidence = 0.8; // Moderate confidence for manually set position
        lastUpdateTimestamp = System.currentTimeMillis();

        telemetry.addData("Position Set", "X:%.1f Y:%.1f H:%.1f°", x, y, heading);
    }

    /**
     * Get current field zone
     */
    public String getCurrentZone() {
        if (fieldMap == null) return "UNKNOWN";

        // Use FieldMap's zone detection logic
        return fieldMap.getZoneInfo(unifiedX, unifiedY);
    }

    /**
     * Get closest goal position
     */
    public FieldMap.FieldPosition getClosestGoal() {
        if (fieldMap == null) return null;
        return fieldMap.getClosestGoal(unifiedX, unifiedY);
    }

    /**
     * Get localization system telemetry data
     */
    public LocalizationData getTelemetryData() {
        return new LocalizationData(
            unifiedX, unifiedY, unifiedHeading, positionConfidence,
            currentMode, lastUpdateTimestamp,
            visionInitialized, odometryInitialized, robotTipDetected,
            totalFusionUpdates, visionBasedUpdates, odometryOnlyUpdates,
            averageFusionTime, performanceTimer.seconds(),
            hasValidVisionData(), hasValidOdometryData()
        );
    }

    /**
     * Data container for localization telemetry
     */
    public static class LocalizationData {
        public final double x, y, heading, confidence;
        public final LocalizationMode mode;
        public final long lastUpdateTimestamp;
        public final boolean visionAvailable, odometryAvailable, robotTipped;
        public final int totalUpdates, visionUpdates, odometryUpdates;
        public final double averageFusionTime, uptimeSeconds;
        public final boolean visionValid, odometryValid;

        public LocalizationData(double x, double y, double heading, double confidence,
                               LocalizationMode mode, long lastUpdateTimestamp,
                               boolean visionAvailable, boolean odometryAvailable, boolean robotTipped,
                               int totalUpdates, int visionUpdates, int odometryUpdates,
                               double averageFusionTime, double uptimeSeconds,
                               boolean visionValid, boolean odometryValid) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.confidence = confidence;
            this.mode = mode;
            this.lastUpdateTimestamp = lastUpdateTimestamp;
            this.visionAvailable = visionAvailable;
            this.odometryAvailable = odometryAvailable;
            this.robotTipped = robotTipped;
            this.totalUpdates = totalUpdates;
            this.visionUpdates = visionUpdates;
            this.odometryUpdates = odometryUpdates;
            this.averageFusionTime = averageFusionTime;
            this.uptimeSeconds = uptimeSeconds;
            this.visionValid = visionValid;
            this.odometryValid = odometryValid;
        }

        public double getVisionUsagePercentage() {
            return totalUpdates > 0 ? (double) visionUpdates / totalUpdates * 100.0 : 0.0;
        }

        public double getOdometryUsagePercentage() {
            return totalUpdates > 0 ? (double) odometryUpdates / totalUpdates * 100.0 : 0.0;
        }
    }

    // === Control Methods ===

    /**
     * Change localization mode
     */
    public void setLocalizationMode(LocalizationMode mode) {
        this.currentMode = mode;
        validateModeCompatibility();
        setInitialPosition();

        telemetry.addData("Mode Changed", mode.getName());
    }

    /**
     * Reset all localization systems
     */
    public void reset() {
        if (odometryInitialized && odometry != null) {
            odometry.reset();
        }

        if (visionInitialized && visionLocalizer != null) {
            visionLocalizer.resetPosition();
        }

        unifiedX = 0.0;
        unifiedY = 0.0;
        unifiedHeading = 0.0;
        positionConfidence = 0.0;

        // Reset statistics
        totalFusionUpdates = 0;
        visionBasedUpdates = 0;
        odometryOnlyUpdates = 0;
        visionDetectionCount = 0;

        telemetry.addLine("🔄 Localization systems reset");
    }

    /**
     * Get status summary for debugging
     */
    public String getStatusSummary() {
        if (!isInitialized) {
            return "LocalizationUnifier: Not initialized";
        }

        return String.format("LocalizationUnifier: %s mode, %.1f%% conf, Zone: %s",
            currentMode.getName(),
            positionConfidence * 100,
            getCurrentZone());
    }

    // === Utility Methods ===

    public boolean isInitialized() { return isInitialized; }
    public LocalizationMode getCurrentMode() { return currentMode; }
    public boolean isVisionAvailable() { return visionInitialized; }
    public boolean isOdometryAvailable() { return odometryInitialized; }
    public boolean isRobotTipped() { return robotTipDetected; }

    /**
     * Normalize angle to [-180, 180] degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
