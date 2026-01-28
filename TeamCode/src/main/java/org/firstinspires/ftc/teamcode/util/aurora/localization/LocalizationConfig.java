package org.firstinspires.ftc.teamcode.util.aurora.localization;

/**
 * Configuration parameters for the fusion localization system.
 * 
 * Tuning Guide:
 * - Process noise (Q): Controls how much uncertainty grows during prediction
 *   - Higher = trusts odometry less, vision more
 *   - Lower = trusts odometry more, vision less
 * 
 * - Measurement noise (R): Controls trust in vision measurements
 *   - Higher = trusts vision less (slower corrections)
 *   - Lower = trusts vision more (faster corrections)
 * 
 * - Gating threshold: Controls measurement rejection
 *   - Higher = accepts more measurements (less conservative)
 *   - Lower = rejects more measurements (more conservative)
 */
public class LocalizationConfig {
    
    // ═══════════════════════════════════════════════════════════════════════
    // Process Noise Covariance (Q) - Odometry uncertainty growth
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Process noise for X position (mm per update)
     * Default: 5.0 mm - typical odometry drift per update cycle
     */
    public double processNoiseX = 5.0;
    
    /**
     * Process noise for Y position (mm per update)
     * Default: 5.0 mm - typical odometry drift per update cycle
     */
    public double processNoiseY = 5.0;
    
    /**
     * Process noise for heading (radians per update)
     * Default: 0.02 rad (~1.1°) - typical IMU drift per update cycle
     */
    public double processNoiseHeading = 0.02;
    
    // ═══════════════════════════════════════════════════════════════════════
    // Measurement Noise Covariance (R) - Vision measurement uncertainty
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Measurement noise for X position from vision (mm)
     * Default: 50.0 mm - typical Limelight accuracy
     */
    public double measurementNoiseX = 50.0;
    
    /**
     * Measurement noise for Y position from vision (mm)
     * Default: 50.0 mm - typical Limelight accuracy
     */
    public double measurementNoiseY = 50.0;
    
    /**
     * Measurement noise for heading from vision (radians)
     * Default: 0.1 rad (~5.7°) - typical Limelight heading accuracy
     */
    public double measurementNoiseHeading = 0.1;
    
    // ═══════════════════════════════════════════════════════════════════════
    // Measurement Gating Parameters
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Mahalanobis distance threshold for measurement acceptance
     * Default: 3.0 (99.7% confidence interval for Chi-squared distribution)
     * 
     * Statistical interpretation:
     * - 1.0 = 68.3% confidence (1 sigma)
     * - 2.0 = 95.4% confidence (2 sigma)
     * - 3.0 = 99.7% confidence (3 sigma) [RECOMMENDED]
     * - 4.0 = 99.99% confidence (4 sigma)
     */
    public double mahalanobisThreshold = 3.0;
    
    /**
     * Relaxed Mahalanobis threshold for first vision update
     * Default: 5.0 - more permissive for initial correction
     */
    public double mahalanobisThresholdInitial = 5.0;
    
    /**
     * Maximum innovation magnitude (safety check, mm)
     * Reject measurements with larger corrections regardless of statistics
     * Default: 1000.0 mm (1 meter)
     */
    public double maxInnovationMagnitude = 1000.0;
    
    // ═══════════════════════════════════════════════════════════════════════
    // Vision Update Constraints
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Maximum robot velocity for vision updates (mm/s)
     * Reject vision if moving faster than this
     * Default: 500.0 mm/s (~20 in/s)
     */
    public double maxVelocityForVision = 500.0;
    
    /**
     * Maximum angular velocity for vision updates (rad/s)
     * Reject vision if rotating faster than this
     * Default: 0.52 rad/s (~30 deg/s)
     */
    public double maxAngularVelocityForVision = 0.52;
    
    /**
     * Maximum distance to AprilTag for reliable measurements (mm)
     * Default: 4000.0 mm (4 meters, ~13 feet)
     */
    public double maxVisionDistance = 4000.0;
    
    /**
     * Maximum vision latency to accept (ms)
     * Reject measurements older than this
     * Default: 200 ms
     */
    public long maxVisionLatency = 200;
    
    /**
     * Minimum time between vision updates (ms)
     * Rate limit vision corrections
     * Default: 100 ms (10 Hz max)
     */
    public long minVisionUpdateInterval = 100;
    
    // ═══════════════════════════════════════════════════════════════════════
    // Fusion Parameters
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Alpha blending factor for gradual corrections (0-1)
     * 0.0 = no vision correction (odometry only)
     * 1.0 = instant correction (can cause jumps)
     * 0.1 = 10% correction per update (smooth, recommended)
     * 
     * Default: 0.1 - corrections spread over ~10 updates (~200ms)
     */
    public double correctionAlpha = 0.1;
    
    /**
     * Alpha blending factor for initial correction (first vision update)
     * Default: 0.5 - faster initial convergence
     */
    public double correctionAlphaInitial = 0.5;
    
    /**
     * Whether to blend relative pose with vision corrections
     * true = relative pose slowly drifts toward absolute (smoother)
     * false = relative pose independent (pure odometry)
     * 
     * Default: true
     */
    public boolean blendRelativePose = true;
    
    /**
     * Blending rate for relative pose (if enabled)
     * Slower than absolute pose to maintain smoothness
     * Default: 0.02 - very slow drift correction
     */
    public double relativePoseBlendAlpha = 0.02;
    
    // ═══════════════════════════════════════════════════════════════════════
    // Initial Uncertainty
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Initial X position uncertainty (mm)
     * Reflects human placement accuracy
     * Default: 50.0 mm (~2 inches)
     */
    public double initialUncertaintyX = 50.0;
    
    /**
     * Initial Y position uncertainty (mm)
     * Default: 50.0 mm (~2 inches)
     */
    public double initialUncertaintyY = 50.0;
    
    /**
     * Initial heading uncertainty (radians)
     * Default: 0.17 rad (~10 degrees) - human alignment error
     */
    public double initialUncertaintyHeading = 0.17;
    
    // ═══════════════════════════════════════════════════════════════════════
    // History Buffer Configuration
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Depth of pose history buffer (milliseconds)
     * Must be larger than maximum expected vision latency
     * Default: 500 ms
     */
    public long historyBufferDepth = 500;
    
    /**
     * Maximum number of poses to store in history
     * Prevents unbounded memory growth
     * Default: 50 poses
     */
    public int historyBufferMaxSize = 50;
    
    // ═══════════════════════════════════════════════════════════════════════
    // Safety Limits
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Maximum covariance trace before warning/capping (mm^2)
     * If uncertainty exceeds this, cap growth and warn
     * Default: 100000.0 (sqrt = 316 mm)
     */
    public double maxCovarianceTrace = 100000.0;
    
    /**
     * Vision timeout - increase uncertainty if no vision (ms)
     * Default: 30000 ms (30 seconds)
     */
    public long visionTimeout = 30000;
    
    /**
     * Odometry timeout - disable system if no odometry (ms)
     * Default: 1000 ms (1 second)
     */
    public long odometryTimeout = 1000;
    
    // ═══════════════════════════════════════════════════════════════════════
    // Constructor
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Create config with default values
     */
    public LocalizationConfig() {
        // All defaults set via field initializers above
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // Convenience Methods
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Get process noise covariance matrix Q
     */
    public Matrix3x3 getProcessNoiseMatrix() {
        return Matrix3x3.diagonal(
            processNoiseX * processNoiseX,
            processNoiseY * processNoiseY,
            processNoiseHeading * processNoiseHeading
        );
    }
    
    /**
     * Get measurement noise covariance matrix R
     */
    public Matrix3x3 getMeasurementNoiseMatrix() {
        return Matrix3x3.diagonal(
            measurementNoiseX * measurementNoiseX,
            measurementNoiseY * measurementNoiseY,
            measurementNoiseHeading * measurementNoiseHeading
        );
    }
    
    /**
     * Get initial covariance matrix P0
     */
    public Matrix3x3 getInitialCovarianceMatrix() {
        return Matrix3x3.diagonal(
            initialUncertaintyX * initialUncertaintyX,
            initialUncertaintyY * initialUncertaintyY,
            initialUncertaintyHeading * initialUncertaintyHeading
        );
    }
    
    /**
     * Create a conservative config (trusts vision less, odometry more)
     */
    public static LocalizationConfig conservative() {
        LocalizationConfig config = new LocalizationConfig();
        config.processNoiseX = 2.0;  // Lower = trust odometry more
        config.processNoiseY = 2.0;
        config.measurementNoiseX = 100.0;  // Higher = trust vision less
        config.measurementNoiseY = 100.0;
        config.correctionAlpha = 0.05;  // Slower corrections
        config.mahalanobisThreshold = 2.0;  // More restrictive gating
        return config;
    }
    
    /**
     * Create an aggressive config (trusts vision more, odometry less)
     */
    public static LocalizationConfig aggressive() {
        LocalizationConfig config = new LocalizationConfig();
        config.processNoiseX = 10.0;  // Higher = trust odometry less
        config.processNoiseY = 10.0;
        config.measurementNoiseX = 25.0;  // Lower = trust vision more
        config.measurementNoiseY = 25.0;
        config.correctionAlpha = 0.2;  // Faster corrections
        config.mahalanobisThreshold = 4.0;  // More permissive gating
        return config;
    }
}
