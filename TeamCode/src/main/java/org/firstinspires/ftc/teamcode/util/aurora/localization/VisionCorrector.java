package org.firstinspires.ftc.teamcode.util.aurora.localization;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Extended Kalman Filter (EKF) update/correction step.
 * 
 * Implements vision-based pose correction with:
 * - Latency compensation (historical state correction)
 * - Kalman gain computation
 * - Gradual correction (alpha blending)
 * - Innovation tracking
 * 
 * The corrector applies vision measurements to historical poses,
 * then propagates the correction forward to the current time.
 */
public class VisionCorrector {
    
    private final LocalizationConfig config;
    private final LimelightVisionHelper limelight;
    private final PoseHistory history;
    
    private boolean firstCorrection = true;
    private long lastCorrectionTime = 0;
    
    // Innovation tracking (for telemetry)
    private double lastInnovationX = 0;
    private double lastInnovationY = 0;
    private double lastInnovationHeading = 0;
    private double lastMahalanobisDistance = 0;
    
    /**
     * Create vision corrector
     * 
     * @param config Configuration parameters
     * @param limelight Vision system (from AuroraHardwareConfig)
     * @param history Pose history buffer
     */
    public VisionCorrector(
        LocalizationConfig config,
        LimelightVisionHelper limelight,
        PoseHistory history
    ) {
        this.config = config;
        this.limelight = limelight;
        this.history = history;
    }
    
    /**
     * Attempt to correct pose using vision measurement
     * 
     * Process:
     * 1. Get vision measurement
     * 2. Retrieve historical pose at vision timestamp
     * 3. Compute innovation (measurement - prediction)
     * 4. Compute Kalman gain
     * 5. Apply correction to historical pose
     * 6. Propagate correction to current pose
     * 7. Update covariance
     * 
     * @param currentPose Current pose estimate
     * @return Corrected pose, or currentPose if no correction applied
     */
    public RobotPose2D correct(RobotPose2D currentPose) {
        if (limelight == null || !limelight.hasTarget()) {
            return currentPose.copy();
        }
        
        // Get vision pose (3D from Limelight)
        Pose3D visionPose3D = limelight.getRobotPose();
        if (visionPose3D == null) {
            return currentPose.copy();
        }
        
        // Convert to 2D measurement
        // CRITICAL: Limelight returns position in METERS, convert to MILLIMETERS
        double rawX = visionPose3D.getPosition().x * 1000.0;  // meters to mm
        double rawY = visionPose3D.getPosition().y * 1000.0;  // meters to mm
        double rawYaw = visionPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

        // COORDINATE SYSTEM ISSUE:
        // MegaTag2 uses robot's IMU heading to determine pose. If the IMU coordinate system
        // doesn't match Limelight's AprilTag field coordinate system, MegaTag2 will compute
        // the wrong position (rotated ~90° from actual).
        //
        // The fix is NOT to rotate the output, but to ensure:
        // 1. IMU is configured with correct orientation (Logo/USB facing directions)
        // 2. Limelight's AprilTag field layout matches FTC coordinate system
        // 3. Both systems agree on what heading=0° means
        //
        // For now, using raw values to diagnose the actual transformation needed
        double transformedX = rawX;  // No transformation yet
        double transformedY = rawY;
        double transformedYaw = rawYaw;

        RobotPose2D measurement = new RobotPose2D(
            transformedX,
            transformedY,
            transformedYaw
        );
        measurement.timestamp = System.currentTimeMillis();  // Vision is "now"
        
        // Estimate vision capture time (assume 100ms latency)
        long estimatedCaptureTime = measurement.timestamp - 100;
        
        // Retrieve historical pose at vision capture time
        RobotPose2D historicalPose = history.get(estimatedCaptureTime);
        if (historicalPose == null) {
            // No historical data - use current pose as fallback
            historicalPose = currentPose.copy();
        }
        
        // Compute innovation (measurement - prediction)
        double innovationX = measurement.x - historicalPose.x;
        double innovationY = measurement.y - historicalPose.y;
        double innovationHeading = RobotPose2D.angleWrap(
            measurement.heading - historicalPose.heading
        );
        
        // Track innovation (for telemetry)
        lastInnovationX = innovationX;
        lastInnovationY = innovationY;
        lastInnovationHeading = innovationHeading;
        
        // Compute innovation covariance: S = H*P*H^T + R
        // For direct measurement (H = I), S = P + R
        Matrix3x3 S = historicalPose.covariance.add(config.getMeasurementNoiseMatrix());
        
        // Compute Mahalanobis distance (for tracking)
        try {
            Matrix3x3 Sinv = S.inverse();
            double[] Sinv_innovation = Sinv.multiply(innovationX, innovationY, innovationHeading);
            lastMahalanobisDistance = Math.sqrt(
                innovationX * Sinv_innovation[0] +
                innovationY * Sinv_innovation[1] +
                innovationHeading * Sinv_innovation[2]
            );
        } catch (ArithmeticException e) {
            lastMahalanobisDistance = 999.9;
        }
        
        // Compute Kalman gain: K = P * H^T * S^-1
        // For H = I, K = P * S^-1
        Matrix3x3 K;
        try {
            Matrix3x3 Sinv = S.inverse();
            K = historicalPose.covariance.multiply(Sinv);
        } catch (ArithmeticException e) {
            // Singular covariance - skip correction
            return currentPose.copy();
        }
        
        // Apply correction with Kalman gain: x_corrected = x_pred + K * innovation
        double[] correction = K.multiply(innovationX, innovationY, innovationHeading);
        
        // Get alpha factor (gradual correction)
        double alpha = firstCorrection ? 
            config.correctionAlphaInitial : 
            config.correctionAlpha;
        
        // Apply alpha blending to correction
        double correctionX = alpha * correction[0];
        double correctionY = alpha * correction[1];
        double correctionHeading = alpha * correction[2];
        
        // Create corrected pose
        RobotPose2D correctedPose = currentPose.copy();
        correctedPose.x = currentPose.x + correctionX;
        correctedPose.y = currentPose.y + correctionY;
        correctedPose.heading = RobotPose2D.angleWrap(
            currentPose.heading + correctionHeading
        );
        
        // Update covariance: P_new = (I - K*H) * P_old
        // For H = I, P_new = (I - K) * P_old
        Matrix3x3 I = Matrix3x3.identity();
        Matrix3x3 I_minus_K = I.subtract(K);
        correctedPose.covariance = I_minus_K.multiply(currentPose.covariance);
        correctedPose.covariance.enforceSymmetry();
        
        // Ensure covariance remains positive definite
        if (!correctedPose.covariance.isPositiveDefinite()) {
            // Fallback: just reduce uncertainty slightly
            correctedPose.covariance = currentPose.covariance.multiply(0.9);
        }
        
        // Update tracking
        lastCorrectionTime = System.currentTimeMillis();
        firstCorrection = false;
        
        return correctedPose;
    }
    
    /**
     * Get time since last successful correction
     */
    public long getTimeSinceLastCorrection() {
        if (lastCorrectionTime == 0) {
            return -1;
        }
        return System.currentTimeMillis() - lastCorrectionTime;
    }
    
    /**
     * Get last innovation (X component)
     */
    public double getLastInnovationX() {
        return lastInnovationX;
    }
    
    /**
     * Get last innovation (Y component)
     */
    public double getLastInnovationY() {
        return lastInnovationY;
    }
    
    /**
     * Get last innovation (heading component)
     */
    public double getLastInnovationHeading() {
        return lastInnovationHeading;
    }
    
    /**
     * Get last Mahalanobis distance
     */
    public double getLastMahalanobisDistance() {
        return lastMahalanobisDistance;
    }
    
    /**
     * Get innovation magnitude (Euclidean distance)
     */
    public double getInnovationMagnitude() {
        return Math.sqrt(
            lastInnovationX * lastInnovationX +
            lastInnovationY * lastInnovationY
        );
    }
    
    /**
     * Check if first correction has been applied
     */
    public boolean isFirstCorrectionDone() {
        return !firstCorrection;
    }
    
    /**
     * Reset corrector state
     */
    public void reset() {
        firstCorrection = true;
        lastCorrectionTime = 0;
        lastInnovationX = 0;
        lastInnovationY = 0;
        lastInnovationHeading = 0;
        lastMahalanobisDistance = 0;
    }
}
