package org.firstinspires.ftc.teamcode.util.aurora.localization;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.debug.DebugLogger;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

/**
 * Fusion Localization System - Main API
 * 
 * Provides dual-pose estimation for FTC robot:
 * - Relative pose: Smooth odometry-based (no jumps, for drivetrain)
 * - Absolute pose: Vision-corrected (minimal drift, for turret/field tasks)
 * 
 * Features:
 * - Extended Kalman Filter (EKF) fusion
 * - Latency-compensated vision corrections
 * - Statistical measurement gating (Mahalanobis)
 * - Gradual corrections (no pose snapping)
 * - Support for 6 predefined start poses
 * 
 * Usage:
 * ```java
 * // Initialize
 * FusionLocalizer localizer = new FusionLocalizer(
 *     hardware,
 *     PredefinedPoses.StartPosition.RED_AUDIENCE_LEFT
 * );
 * 
 * // In loop
 * localizer.update();
 * RobotPose2D relativePose = localizer.getRelativePose();  // For drivetrain
 * RobotPose2D absolutePose = localizer.getAbsolutePose();  // For turret
 * ```
 */
public class FusionLocalizer {
    
    private static final String LOG_TAG = "FusionLocalizer";

    // Configuration
    private final LocalizationConfig config;
    
    // Hardware
    private final AuroraHardwareConfig hardware;
    private final GoBildaPinpointDriver odometry;
    private final LimelightVisionHelper limelight;
    
    // Core components
    private final StateEstimator estimator;
    private final VisionCorrector corrector;
    private final MeasurementValidator validator;
    private final PoseHistory history;
    
    // Debug logger (optional)
    private DebugLogger debugLogger;

    // State
    private RobotPose2D relativePose;  // Odometry-only (smooth, drifts)
    private RobotPose2D absolutePose;  // Vision-corrected (stable, minimal drift)
    
    private boolean initialized = false;
    private long lastUpdateTime = 0;
    
    // Statistics
    private int visionAcceptCount = 0;
    private int visionRejectCount = 0;
    
    /**
     * Create fusion localizer at origin
     */
    public FusionLocalizer(AuroraHardwareConfig hardware) {
        this(hardware, new RobotPose2D(), new LocalizationConfig());
    }
    
    /**
     * Create fusion localizer with predefined start position
     */
    public FusionLocalizer(
        AuroraHardwareConfig hardware,
        PredefinedPoses.StartPosition startPosition
    ) {
        this(hardware, startPosition, new LocalizationConfig());
    }
    
    /**
     * Create fusion localizer with predefined start position and custom config
     */
    public FusionLocalizer(
        AuroraHardwareConfig hardware,
        PredefinedPoses.StartPosition startPosition,
        LocalizationConfig config
    ) {
        this(hardware, PredefinedPoses.getPose(startPosition, config), config);
    }
    
    /**
     * Create fusion localizer with custom start pose
     */
    public FusionLocalizer(
        AuroraHardwareConfig hardware,
        RobotPose2D startPose
    ) {
        this(hardware, startPose, new LocalizationConfig());
    }
    
    /**
     * Create fusion localizer with custom start pose and config (full control)
     */
    public FusionLocalizer(
        AuroraHardwareConfig hardware,
        RobotPose2D startPose,
        LocalizationConfig config
    ) {
        this(hardware, startPose, config, null);
    }

    /**
     * Create fusion localizer with custom start pose, config, and debug logger
     */
    public FusionLocalizer(
        AuroraHardwareConfig hardware,
        RobotPose2D startPose,
        LocalizationConfig config,
        DebugLogger debugLogger
    ) {
        this.hardware = hardware;
        this.config = config;
        this.debugLogger = debugLogger;

        logDebug("Initializing FusionLocalizer",
            String.format("startPose=(%.1f, %.1f)mm @ %.1f°",
                startPose.x, startPose.y, Math.toDegrees(startPose.heading)));

        // Get hardware components
        this.odometry = hardware.getOdometry();
        // Create Limelight directly from hardwareMap (not part of AuroraHardwareConfig)
        this.limelight = new LimelightVisionHelper(hardware.getHardwareMap());
        
        if (odometry == null) {
            logWarning("Odometry is NULL - localization will not work!");
        }

        if (!limelight.isInitialized()) {
            logWarning("Limelight not initialized: " + limelight.getInitializationError());
        }

        // Create components
        this.history = new PoseHistory(config.historyBufferDepth, config.historyBufferMaxSize);
        this.estimator = new StateEstimator(config, odometry);
        this.corrector = new VisionCorrector(config, limelight, history);
        this.validator = new MeasurementValidator(config);
        
        // Initialize state
        this.relativePose = startPose.copy();
        this.absolutePose = startPose.copy();
        
        // Initialize estimator
        estimator.initialize(startPose);
        
        // Add initial pose to history
        history.add(startPose);
        
        this.initialized = true;
        this.lastUpdateTime = System.currentTimeMillis();

        logInfo("FusionLocalizer initialized successfully");
    }
    
    /**
     * Update localization (call every loop)
     * 
     * Performs:
     * 1. Predict step (odometry + IMU)
     * 2. Validate vision measurement (if available)
     * 3. Correct step (if measurement valid)
     * 4. Blend poses (relative vs absolute)
     */
    public void update() {
        if (!initialized) {
            return;
        }
        
        // === PREDICT STEP (always) ===
        relativePose = estimator.predict(relativePose);
        
        // Update absolute pose from relative pose
        // The relative pose is in the Pinpoint's coordinate frame (where the robot started)
        // The absolute pose should be in the field's coordinate frame
        //
        // The heading offset tells us the rotation between these frames:
        // - Relative heading 0° = Absolute heading (headingOffset)°
        // - So we need to rotate the position by headingOffset to get field coordinates

        if (limelight != null && limelight.isHeadingCalibrated()) {
            double offsetRad = Math.toRadians(limelight.getHeadingOffset());

            // Rotate position from relative frame to absolute field frame
            double cos_offset = Math.cos(offsetRad);
            double sin_offset = Math.sin(offsetRad);
            absolutePose.x = cos_offset * relativePose.x - sin_offset * relativePose.y;
            absolutePose.y = sin_offset * relativePose.x + cos_offset * relativePose.y;

            // Convert heading to absolute
            absolutePose.heading = RobotPose2D.angleWrap(relativePose.heading + offsetRad);
        } else {
            // Not calibrated yet - just use relative pose directly
            absolutePose.x = relativePose.x;
            absolutePose.y = relativePose.y;
            absolutePose.heading = relativePose.heading;
        }

        absolutePose.vx = relativePose.vx;
        absolutePose.vy = relativePose.vy;
        absolutePose.vheading = relativePose.vheading;
        absolutePose.timestamp = relativePose.timestamp;
        
        // Add to history (for latency compensation)
        history.add(relativePose.copy());
        
        // === CORRECT STEP (if vision valid) ===
        if (shouldAttemptVisionCorrection()) {
            // CRITICAL FOR MEGATAG2: Update robot orientation BEFORE getting vision pose
            // MegaTag2 uses this as a reference for calculating robot position
            // MUST be in DEGREES, not radians!
            double headingDegrees = Math.toDegrees(relativePose.heading);

            // Check if heading will be calibrated this update
            boolean wasCalibrated = limelight.isHeadingCalibrated();

            limelight.updateRobotOrientation(headingDegrees);

            // Log if calibration just happened
            if (!wasCalibrated && limelight.isHeadingCalibrated()) {
                Log.i(LOG_TAG, String.format("HEADING_CALIBRATED: offset=%.1f° (MT1=%.1f°, Odo=%.1f°)",
                    limelight.getHeadingOffset(), limelight.getLastMT1HeadingDeg(), headingDegrees));
                logInfo("HEADING_CALIBRATED", String.format("offset=%.1f° from MT1", limelight.getHeadingOffset()));
            }

            // Direct logcat output for debugging
            Log.d(LOG_TAG, String.format("MT2_HEADING_INPUT: %.1f° (sent to Limelight, offset=%.0f°, calibrated=%s)",
                limelight.getLastRobotHeadingDeg(), limelight.getHeadingOffset(),
                limelight.isHeadingCalibrated() ? "YES" : "NO"));
            logDebug("MT2_HEADING_INPUT", String.format("%.1f° (sent to Limelight)", limelight.getLastRobotHeadingDeg()));

            // Get robot velocity for gating
            double velocity = estimator.getVelocityMagnitude();
            double angularVelocity = estimator.getAngularVelocity();
            
            // Create measurement from vision
            RobotPose2D measurement = createVisionMeasurement();
            
            if (measurement != null) {
                // Log the measurement we're about to validate
                String measStr = String.format("raw=(%.1f, %.1f)mm @ %.1f°",
                    measurement.x, measurement.y, Math.toDegrees(measurement.heading));
                Log.d(LOG_TAG, "VISION_MEASUREMENT: " + measStr);
                logDebug("VISION_MEASUREMENT", measStr);

                // Log current predicted pose for comparison
                String predStr = String.format("(%.1f, %.1f)mm @ %.1f°",
                    absolutePose.x, absolutePose.y, Math.toDegrees(absolutePose.heading));
                Log.d(LOG_TAG, "PREDICTED_POSE: " + predStr);
                logDebug("PREDICTED_POSE", predStr);

                // Calculate innovation (difference)
                double innovX = measurement.x - absolutePose.x;
                double innovY = measurement.y - absolutePose.y;
                double innovH = Math.toDegrees(RobotPose2D.angleWrap(measurement.heading - absolutePose.heading));
                String innovStr = String.format("Δ=(%.1f, %.1f)mm, Δheading=%.1f°", innovX, innovY, innovH);
                Log.d(LOG_TAG, "INNOVATION: " + innovStr);
                logDebug("INNOVATION", innovStr);

                // Validate measurement
                boolean valid = validator.validate(
                    limelight,
                    absolutePose,
                    measurement,
                    measurement.timestamp,
                    velocity,
                    angularVelocity
                );
                
                if (valid) {
                    String acceptStr = String.format("Innovation=(%.1f, %.1f)mm", innovX, innovY);
                    Log.i(LOG_TAG, "VISION_ACCEPTED: " + acceptStr);
                    logInfo("VISION_ACCEPTED", acceptStr);

                    // Apply correction
                    absolutePose = corrector.correct(absolutePose);
                    visionAcceptCount++;
                    
                    // Optionally blend relative pose toward absolute (slow drift correction)
                    if (config.blendRelativePose) {
                        double alpha = config.relativePoseBlendAlpha;
                        relativePose.x = (1 - alpha) * relativePose.x + alpha * absolutePose.x;
                        relativePose.y = (1 - alpha) * relativePose.y + alpha * absolutePose.y;
                        double dh = RobotPose2D.angleWrap(absolutePose.heading - relativePose.heading);
                        relativePose.heading = RobotPose2D.angleWrap(relativePose.heading + alpha * dh);
                    }
                } else {
                    // Rejected
                    String reason = validator.getLastRejectionReason();
                    Log.w(LOG_TAG, "VISION_REJECTED: " + reason);
                    logWarning("VISION_REJECTED: " + reason);
                    visionRejectCount++;
                }
            }
        }
        
        lastUpdateTime = System.currentTimeMillis();
    }
    
    /**
     * Get relative pose (odometry-based, smooth, no jumps)
     * Use for: drivetrain control, path following
     */
    public RobotPose2D getRelativePose() {
        return relativePose.copy();
    }
    
    /**
     * Get absolute pose (vision-corrected, stable, minimal drift)
     * Use for: turret aiming, field-relative tasks
     */
    public RobotPose2D getAbsolutePose() {
        return absolutePose.copy();
    }
    
    /**
     * Get current heading (from absolute pose)
     */
    public double getHeading(AngleUnit unit) {
        return absolutePose.getHeading(unit);
    }
    
    /**
     * Get X position (from absolute pose)
     */
    public double getX(DistanceUnit unit) {
        return absolutePose.getX(unit);
    }
    
    /**
     * Get Y position (from absolute pose)
     */
    public double getY(DistanceUnit unit) {
        return absolutePose.getY(unit);
    }
    
    /**
     * Get velocity X component
     */
    public double getVelocityX(DistanceUnit unit) {
        return unit.fromMm(absolutePose.vx);
    }
    
    /**
     * Get velocity Y component
     */
    public double getVelocityY(DistanceUnit unit) {
        return unit.fromMm(absolutePose.vy);
    }
    
    /**
     * Get velocity magnitude
     */
    public double getVelocityMagnitude(DistanceUnit unit) {
        return absolutePose.getVelocityMagnitude(unit);
    }
    
    /**
     * Get heading velocity
     */
    public double getHeadingVelocity(AngleUnit unit) {
        return unit.fromRadians(absolutePose.vheading);
    }
    
    /**
     * Check if vision correction is active
     */
    public boolean isVisionActive() {
        long timeSinceCorrection = corrector.getTimeSinceLastCorrection();
        return timeSinceCorrection >= 0 && timeSinceCorrection < 2000;
    }
    
    /**
     * Get time since last vision correction (ms)
     */
    public long getTimeSinceLastVisionUpdate() {
        return corrector.getTimeSinceLastCorrection();
    }
    
    /**
     * Get pose uncertainty (absolute pose)
     */
    public double getPoseUncertainty() {
        return absolutePose.getUncertainty();
    }
    
    /**
     * Get position uncertainty (absolute pose)
     */
    public double getPositionUncertainty() {
        return absolutePose.getPositionUncertainty();
    }
    
    /**
     * Get heading uncertainty (absolute pose)
     */
    public double getHeadingUncertainty(AngleUnit unit) {
        return unit.fromRadians(absolutePose.getHeadingUncertainty());
    }
    
    /**
     * Check if system is initialized
     */
    public boolean isInitialized() {
        return initialized;
    }
    
    /**
     * Set heading offset for MegaTag2 (in degrees).
     * Use this to compensate for coordinate system differences between odometry and Limelight.
     *
     * @param offsetDegrees Offset in degrees (0, 90, 180, 270 are common values)
     */
    public void setHeadingOffset(double offsetDegrees) {
        if (limelight != null) {
            limelight.setHeadingOffset(offsetDegrees);
            logInfo("Heading offset set", String.format("%.0f°", offsetDegrees));
        }
    }

    /**
     * Get current heading offset for MegaTag2
     * @return Offset in degrees
     */
    public double getHeadingOffset() {
        return limelight != null ? limelight.getHeadingOffset() : 0.0;
    }

    /**
     * Check if heading has been calibrated from MT1
     */
    public boolean isHeadingCalibrated() {
        return limelight != null && limelight.isHeadingCalibrated();
    }

    /**
     * Reset heading calibration (will re-calibrate from MT1 on next update)
     */
    public void resetHeadingCalibration() {
        if (limelight != null) {
            limelight.resetHeadingCalibration();
            logInfo("Heading calibration reset - will re-calibrate from MT1");
        }
    }

    /**
     * Get vision acceptance rate
     */
    public double getVisionAcceptanceRate() {
        int total = visionAcceptCount + visionRejectCount;
        if (total == 0) return 0.0;
        return (double)visionAcceptCount / total;
    }
    
    /**
     * Get total vision measurements accepted
     */
    public int getVisionAcceptCount() {
        return visionAcceptCount;
    }
    
    /**
     * Get total vision measurements rejected
     */
    public int getVisionRejectCount() {
        return visionRejectCount;
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // RAW VISION ACCESSORS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get the raw MegaTag2 pose directly from the Limelight.
     * This is the unfiltered, unaveraged vision reading.
     * Use this when you need the direct vision pose for other purposes (e.g., turret aiming).
     *
     * The pose is in absolute field coordinates (mm) with the field center at (0,0).
     * Includes the 180° rotation correction for the DECODE field.
     *
     * @return RobotPose2D with the current MegaTag2 position, or null if no valid reading
     */
    public RobotPose2D getRawVisionPose() {
        if (limelight == null || !limelight.hasTarget()) {
            return null;
        }

        org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose3D =
            limelight.getRobotPose();

        if (pose3D == null) {
            return null;
        }

        // Get raw values in meters
        double rawX = pose3D.getPosition().x;
        double rawY = pose3D.getPosition().y;
        double rawYaw = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);

        // Apply 180° rotation and convert to mm (same as createVisionMeasurement)
        double xMM = -rawX * 1000.0;
        double yMM = -rawY * 1000.0;

        RobotPose2D visionPose = new RobotPose2D(xMM, yMM, rawYaw);
        visionPose.timestamp = System.currentTimeMillis();

        return visionPose;
    }

    /**
     * Get the raw MegaTag1 pose directly from the Limelight.
     * MegaTag1 doesn't require heading input and works with single tags.
     * Less stable but doesn't depend on IMU heading accuracy.
     *
     * @return RobotPose2D with the current MegaTag1 position, or null if no valid reading
     */
    public RobotPose2D getRawMT1Pose() {
        if (limelight == null || !limelight.hasTarget()) {
            return null;
        }

        org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose3D =
            limelight.getRobotPoseMT1();

        if (pose3D == null) {
            return null;
        }

        // Get raw values in meters
        double rawX = pose3D.getPosition().x;
        double rawY = pose3D.getPosition().y;
        double rawYaw = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);

        // Apply 180° rotation and convert to mm
        double xMM = -rawX * 1000.0;
        double yMM = -rawY * 1000.0;

        RobotPose2D visionPose = new RobotPose2D(xMM, yMM, rawYaw);
        visionPose.timestamp = System.currentTimeMillis();

        return visionPose;
    }

    /**
     * Check if Limelight currently sees an AprilTag
     */
    public boolean hasVisionTarget() {
        return limelight != null && limelight.hasTarget();
    }

    /**
     * Get the LimelightVisionHelper for direct access to vision methods.
     * Use with caution - prefer the higher-level methods when possible.
     *
     * @return The LimelightVisionHelper, or null if not initialized
     */
    public LimelightVisionHelper getLimelight() {
        return limelight;
    }

    /**
     * Reset to a new pose (manual reset)
     * This also recalibrates the heading offset since relative and absolute are now aligned.
     */
    public void reset(RobotPose2D newPose) {
        relativePose = newPose.copy();
        absolutePose = newPose.copy();
        estimator.reset(newPose);
        corrector.reset();
        validator.reset();
        history.clear();
        history.add(newPose);
        visionAcceptCount = 0;
        visionRejectCount = 0;

        // CRITICAL: When we reset to a known absolute pose, the heading offset must be recalibrated
        // Since relativePose.heading now equals absolutePose.heading, offset should be 0
        // But we want: absoluteHeading = odometryHeading + offset
        // After reset, odometry will report the new heading directly, so offset = 0
        if (limelight != null) {
            // Reset calibration - the next MT1 reading will recalibrate
            limelight.resetHeadingCalibration();
            Log.i(LOG_TAG, String.format("RESET: Pose set to (%.1f, %.1f)mm @ %.1f° - heading calibration reset",
                newPose.x, newPose.y, Math.toDegrees(newPose.heading)));
        }
    }

    /**
     * Reset to an absolute pose and set the heading offset explicitly.
     * Use this when you want to force a specific absolute heading without recalibration.
     *
     * @param newAbsolutePose The absolute pose to reset to
     * @param relativeHeadingDeg The current odometry heading in degrees (what odometry reports NOW)
     */
    public void resetWithHeadingOffset(RobotPose2D newAbsolutePose, double relativeHeadingDeg) {
        // Set the absolute pose
        absolutePose = newAbsolutePose.copy();

        // Set relative pose with the current odometry heading but vision's position
        relativePose = newAbsolutePose.copy();
        relativePose.heading = Math.toRadians(relativeHeadingDeg);

        estimator.reset(relativePose);
        corrector.reset();
        validator.reset();
        history.clear();
        history.add(relativePose);
        visionAcceptCount = 0;
        visionRejectCount = 0;

        // Calculate and set the heading offset
        // absoluteHeading = relativeHeading + offset
        // offset = absoluteHeading - relativeHeading
        double absoluteHeadingDeg = Math.toDegrees(newAbsolutePose.heading);
        double offset = absoluteHeadingDeg - relativeHeadingDeg;

        // Normalize to -180 to 180
        while (offset > 180) offset -= 360;
        while (offset < -180) offset += 360;

        if (limelight != null) {
            limelight.setHeadingOffset(offset);
            Log.i(LOG_TAG, String.format(
                "RESET_WITH_OFFSET: Absolute=(%.1f, %.1f)mm @ %.1f°, RelHeading=%.1f°, Offset=%.1f°",
                newAbsolutePose.x, newAbsolutePose.y, absoluteHeadingDeg, relativeHeadingDeg, offset));
        }
    }
    
    /**
     * Reset to a predefined position
     */
    public void reset(PredefinedPoses.StartPosition position) {
        reset(PredefinedPoses.getPose(position, config));
    }
    
    /**
     * Add telemetry (comprehensive status)
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("═══ Fusion Localization ═══");
        
        // Relative pose
        telemetry.addLine(String.format("Relative: (%.1f, %.1f) mm, %.1f°",
            relativePose.x, relativePose.y, Math.toDegrees(relativePose.heading)));
        
        // Absolute pose
        telemetry.addLine(String.format("Absolute: (%.1f, %.1f) mm, %.1f°",
            absolutePose.x, absolutePose.y, Math.toDegrees(absolutePose.heading)));
        
        // Uncertainty
        telemetry.addData("Uncertainty", "%.1f mm", absolutePose.getPositionUncertainty());
        
        // Vision status
        if (isVisionActive()) {
            long age = getTimeSinceLastVisionUpdate();
            telemetry.addData("Vision", "✅ Active (%d ms ago)", age);
        } else {
            telemetry.addData("Vision", "❌ Inactive");
        }
        
        // Statistics
        telemetry.addData("Accepted/Rejected", "%d / %d (%.0f%%)",
            visionAcceptCount, visionRejectCount, getVisionAcceptanceRate() * 100);
        
        // Last rejection reason (if any)
        if (visionRejectCount > 0) {
            String reason = validator.getLastRejectionReason();
            if (!reason.isEmpty()) {
                telemetry.addData("Last Reject", reason);
            }
        }
        
        // Hardware status
        telemetry.addLine();
        if (estimator.isOdometryHealthy()) {
            telemetry.addData("Odometry", "✅ %.1f Hz", estimator.getOdometryFrequency());
        } else {
            telemetry.addData("Odometry", "⚠️ Unhealthy");
        }
        
        if (limelight != null && limelight.isInitialized()) {
            boolean hasTarget = limelight.hasTarget();
            telemetry.addData("Limelight", hasTarget ? "✅ Target" : "❌ No target");

            // Heading calibration status
            if (limelight.isHeadingCalibrated()) {
                telemetry.addData("MT2 Heading", "✅ Calibrated (offset=%.0f°)", limelight.getHeadingOffset());
            } else {
                telemetry.addData("MT2 Heading", "⏳ Calibrating from MT1...");
            }
        } else {
            telemetry.addData("Limelight", "❌ Not initialized");
        }
    }
    
    /**
     * Add detailed debug telemetry
     */
    public void addDebugTelemetry(Telemetry telemetry) {
        addTelemetry(telemetry);
        
        telemetry.addLine();
        telemetry.addLine("═══ Debug Info ═══");
        
        // Innovation
        telemetry.addData("Innovation", "(%.1f, %.1f) mm",
            corrector.getLastInnovationX(), corrector.getLastInnovationY());
        telemetry.addData("Mahalanobis", "%.2f", corrector.getLastMahalanobisDistance());
        
        // Covariance
        telemetry.addData("Covariance Trace", "%.1f", absolutePose.covariance.trace());
        
        // History
        telemetry.addData("History Size", "%d poses", history.size());
        telemetry.addData("History Span", "%d ms", history.getAgeSpan());
        
        // Velocity
        telemetry.addData("Velocity", "%.1f mm/s", estimator.getVelocityMagnitude());
        telemetry.addData("Angular Vel", "%.2f rad/s", estimator.getAngularVelocity());
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // PRIVATE HELPER METHODS
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Check if we should attempt vision correction this update
     */
    private boolean shouldAttemptVisionCorrection() {
        if (limelight == null || !limelight.isInitialized()) {
            return false;
        }
        
        if (!limelight.hasTarget()) {
            return false;
        }
        
        return true;
    }
    
    /**
     * Create measurement from vision system
     */
    private RobotPose2D createVisionMeasurement() {
        if (limelight == null) {
            return null;
        }
        
        // Get MegaTag2 pose (what we're using)
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose3D =
            limelight.getRobotPose();
        
        // Also get MegaTag1 for comparison/debugging
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D mt1Pose =
            limelight.getRobotPoseMT1();

        if (mt1Pose != null) {
            double mt1X = mt1Pose.getPosition().x;
            double mt1Y = mt1Pose.getPosition().y;
            double mt1Yaw = mt1Pose.getOrientation().getYaw(AngleUnit.DEGREES);
            Log.d(LOG_TAG, String.format("MT1_RAW: (%.3f, %.3f)m @ %.1f°", mt1X, mt1Y, mt1Yaw));
        }

        if (pose3D == null) {
            Log.d(LOG_TAG, "MT2_RAW: null (no pose available)");
            return null;
        }
        
        // Log raw vision data before transformation
        double rawX = pose3D.getPosition().x;
        double rawY = pose3D.getPosition().y;
        double rawYaw = pose3D.getOrientation().getYaw(AngleUnit.DEGREES);

        String rawStr = String.format("(%.3f, %.3f)m @ %.1f°", rawX, rawY, rawYaw);
        Log.d(LOG_TAG, "MT2_RAW: " + rawStr);

        // STALE DATA DETECTION: Check if MT2's returned heading matches what we sent
        // If MT2 returns a very different heading, it means it used old/stale orientation data
        double sentHeading = limelight.getLastRobotHeadingDeg();
        double headingDiff = Math.abs(rawYaw - sentHeading);
        // Normalize to 0-180 range
        if (headingDiff > 180) headingDiff = 360 - headingDiff;

        // If heading difference is > 30°, the data is stale (MT2 didn't use our update yet)
        // This happens because Limelight may return cached data from before our orientation update
        if (headingDiff > 30.0) {
            Log.w(LOG_TAG, String.format("MT2_STALE: Heading mismatch! Sent=%.1f°, Got=%.1f°, Δ=%.1f° - DISCARDING",
                sentHeading, rawYaw, headingDiff));

            // Fall back to MT1 if available and the heading difference is large
            if (mt1Pose != null) {
                double mt1X = mt1Pose.getPosition().x;
                double mt1Y = mt1Pose.getPosition().y;
                double mt1Yaw = mt1Pose.getOrientation().getYaw(AngleUnit.RADIANS);

                // Apply 180° rotation to MT1 as well, and convert meters to mm
                double mt1XMM = -mt1X * 1000.0;
                double mt1YMM = -mt1Y * 1000.0;
                RobotPose2D mt1Measurement = new RobotPose2D(mt1XMM, mt1YMM, mt1Yaw);
                mt1Measurement.timestamp = System.currentTimeMillis();

                Log.i(LOG_TAG, String.format("MT2_FALLBACK_TO_MT1: Using MT1 instead: (%.3f, %.3f)m @ %.1f°",
                    -mt1X, -mt1Y, Math.toDegrees(mt1Yaw)));

                logDebug("LL_RAW_POSE", String.format("(%.3f, %.3f)m @ %.1f° (MT1 fallback)", -mt1X, -mt1Y, Math.toDegrees(mt1Yaw)));
                return mt1Measurement;
            }

            return null;  // Discard stale MT2 data if no MT1 available
        }

        logDebug("LL_RAW_POSE", rawStr + " (before 180° rotation)");

        // Apply 180-degree rotation to correct field orientation
        double rotatedX = -rawX;
        double rotatedY = -rawY;
        
        logDebug("LL_TRANSFORMED",
            String.format("(%.3f, %.3f)m @ %.1f° (after 180° rotation)", rotatedX, rotatedY, rawYaw));

        // Log current odometry heading that should be fed to MegaTag2
        double odoHeadingDeg = Math.toDegrees(relativePose.heading);
        logDebug("ODO_HEADING_FOR_MT2",
            String.format("%.1f° (should be fed to Limelight for MegaTag2)", odoHeadingDeg));

        // Convert from meters to millimeters (RobotPose2D uses mm)
        double xMM = rotatedX * 1000.0;
        double yMM = rotatedY * 1000.0;

        RobotPose2D measurement = new RobotPose2D(
            xMM,
            yMM,
            pose3D.getOrientation().getYaw(AngleUnit.RADIANS)
        );
        measurement.timestamp = System.currentTimeMillis();
        
        return measurement;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // DEBUG LOGGING METHODS
    // ═══════════════════════════════════════════════════════════════════════

    private void logDebug(String message) {
        if (debugLogger != null) {
            debugLogger.debug(LOG_TAG, message);
        }
    }

    private void logDebug(String message, String details) {
        if (debugLogger != null) {
            debugLogger.debug(LOG_TAG, message, details);
        }
    }

    private void logInfo(String message) {
        if (debugLogger != null) {
            debugLogger.info(LOG_TAG, message);
        }
    }

    private void logInfo(String message, String details) {
        if (debugLogger != null) {
            debugLogger.info(LOG_TAG, message, details);
        }
    }

    private void logWarning(String message) {
        if (debugLogger != null) {
            debugLogger.warning(LOG_TAG, message);
        }
    }

    private void logError(String message) {
        if (debugLogger != null) {
            debugLogger.error(LOG_TAG, message);
        }
    }
}
