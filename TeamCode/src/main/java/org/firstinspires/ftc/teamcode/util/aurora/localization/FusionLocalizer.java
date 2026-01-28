package org.firstinspires.ftc.teamcode.util.aurora.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.LimelightVisionHelper;
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
        this.hardware = hardware;
        this.config = config;
        
        // Get hardware components
        this.odometry = hardware.getOdometry();
        // Create Limelight directly from hardwareMap (not part of AuroraHardwareConfig)
        this.limelight = new LimelightVisionHelper(hardware.getHardwareMap());
        
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
        
        // Update absolute pose with same odometry delta (before vision correction)
        // This ensures absolute pose doesn't jump when vision is unavailable
        absolutePose.x = relativePose.x;
        absolutePose.y = relativePose.y;
        absolutePose.heading = relativePose.heading;
        absolutePose.vx = relativePose.vx;
        absolutePose.vy = relativePose.vy;
        absolutePose.vheading = relativePose.vheading;
        absolutePose.timestamp = relativePose.timestamp;
        
        // Add to history (for latency compensation)
        history.add(relativePose.copy());
        
        // === CORRECT STEP (if vision valid) ===
        if (shouldAttemptVisionCorrection()) {
            // Get robot velocity for gating
            double velocity = estimator.getVelocityMagnitude();
            double angularVelocity = estimator.getAngularVelocity();
            
            // Create measurement from vision
            RobotPose2D measurement = createVisionMeasurement();
            
            if (measurement != null) {
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
    
    /**
     * Reset to a new pose (manual reset)
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
        
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose3D = 
            limelight.getRobotPose();
        
        if (pose3D == null) {
            return null;
        }
        
        // Apply 180-degree rotation to correct field orientation
        double rawX = pose3D.getPosition().x;
        double rawY = pose3D.getPosition().y;
        double rotatedX = -rawX;
        double rotatedY = -rawY;
        
        RobotPose2D measurement = new RobotPose2D(
            rotatedX,
            rotatedY,
            pose3D.getOrientation().getYaw(AngleUnit.RADIANS)
        );
        measurement.timestamp = System.currentTimeMillis();
        
        return measurement;
    }
}
