package org.firstinspires.ftc.teamcode.util.aurora.localization;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Validates vision measurements before fusion.
 * 
 * Implements multi-level gating:
 * 1. Hardware checks (sensor operational)
 * 2. Quality checks (data freshness, robot motion)
 * 3. Geometric checks (distance, number of tags)
 * 4. Statistical checks (Mahalanobis distance)
 * 5. Safety checks (innovation magnitude)
 */
public class MeasurementValidator {
    
    private final LocalizationConfig config;
    private long lastValidMeasurementTime = 0;
    private int consecutiveRejections = 0;
    private int consecutiveAcceptances = 0;
    private boolean firstMeasurement = true;
    
    // Rejection reason tracking (for telemetry)
    private String lastRejectionReason = "";
    
    /**
     * Create measurement validator
     */
    public MeasurementValidator(LocalizationConfig config) {
        this.config = config;
    }
    
    /**
     * Validate a vision measurement
     * 
     * @param limelight Vision system
     * @param predicted Predicted pose from odometry
     * @param measurement Vision measurement to validate
     * @param timestamp Measurement timestamp
     * @param robotVelocity Robot velocity magnitude (mm/s)
     * @param robotAngularVelocity Robot angular velocity (rad/s)
     * @return true if measurement should be accepted
     */
    public boolean validate(
        LimelightVisionHelper limelight,
        RobotPose2D predicted,
        RobotPose2D measurement,
        long timestamp,
        double robotVelocity,
        double robotAngularVelocity
    ) {
        // Level 1: Hardware checks
        if (!validateHardware(limelight)) {
            return false;
        }
        
        // Level 2: Quality checks
        if (!validateQuality(limelight, robotVelocity, robotAngularVelocity)) {
            return false;
        }
        
        // Level 3: Geometric checks
        if (!validateGeometry(limelight)) {
            return false;
        }
        
        // Level 4: Temporal checks
        if (!validateTiming(timestamp)) {
            return false;
        }
        
        // Level 5: Statistical checks
        if (!validateStatistics(predicted, measurement)) {
            return false;
        }
        
        // Level 6: Safety checks
        if (!validateSafety(predicted, measurement)) {
            return false;
        }
        
        // ACCEPTED
        onAcceptance();
        return true;
    }
    
    /**
     * Level 1: Hardware validation
     */
    private boolean validateHardware(LimelightVisionHelper limelight) {
        if (limelight == null) {
            reject("Limelight not initialized");
            return false;
        }
        
        if (!limelight.isInitialized()) {
            reject("Limelight not initialized");
            return false;
        }
        
        if (!limelight.hasTarget()) {
            reject("No AprilTag visible");
            return false;
        }
        
        return true;
    }
    
    /**
     * Level 2: Quality validation
     */
    private boolean validateQuality(
        LimelightVisionHelper limelight,
        double velocity,
        double angularVelocity
    ) {
        // Check data freshness
        if (!limelight.isDataFresh()) {
            reject("Vision data stale");
            return false;
        }
        
        if (!limelight.isDataQualityGood()) {
            reject("Vision data quality poor");
            return false;
        }
        
        // Check robot motion
        if (velocity > config.maxVelocityForVision) {
            reject(String.format("Velocity too high: %.1f mm/s", velocity));
            return false;
        }
        
        if (Math.abs(angularVelocity) > config.maxAngularVelocityForVision) {
            reject(String.format("Angular velocity too high: %.2f rad/s", angularVelocity));
            return false;
        }
        
        return true;
    }
    
    /**
     * Level 3: Geometric validation
     */
    private boolean validateGeometry(LimelightVisionHelper limelight) {
        Pose3D pose3D = limelight.getRobotPose();
        if (pose3D == null) {
            reject("Null vision pose");
            return false;
        }
        
        // Check distance to target (using Z distance as approximation)
        double distance = Math.abs(pose3D.getPosition().z);
        if (distance > config.maxVisionDistance) {
            reject(String.format("Target too far: %.1f mm", distance));
            return false;
        }
        
        return true;
    }
    
    /**
     * Level 4: Temporal validation
     */
    private boolean validateTiming(long timestamp) {
        long currentTime = System.currentTimeMillis();
        long age = currentTime - timestamp;
        
        // Check latency
        if (age > config.maxVisionLatency) {
            reject(String.format("Measurement too old: %d ms", age));
            return false;
        }
        
        // Check if measurement is in future (clock sync issue)
        if (age < -10) {  // Allow 10ms tolerance
            reject("Measurement in future (clock sync issue)");
            return false;
        }
        
        // Check rate limiting
        long timeSinceLastValid = currentTime - lastValidMeasurementTime;
        if (timeSinceLastValid < config.minVisionUpdateInterval) {
            reject("Rate limit (too soon after last update)");
            return false;
        }
        
        return true;
    }
    
    /**
     * Level 5: Statistical validation (Mahalanobis distance gating)
     */
    private boolean validateStatistics(RobotPose2D predicted, RobotPose2D measurement) {
        // BYPASS for first measurement - allow initial position correction from vision
        // This is needed because the robot may start at an unknown position
        // and the first vision reading should "teleport" it to the correct location
        if (firstMeasurement) {
            return true;  // Accept first measurement unconditionally
        }

        // Compute innovation
        double dx = measurement.x - predicted.x;
        double dy = measurement.y - predicted.y;
        double dh = RobotPose2D.angleWrap(measurement.heading - predicted.heading);
        
        // Compute innovation covariance S = H*P*H^T + R
        // For direct measurement (H = I), S = P + R
        Matrix3x3 S = predicted.covariance.add(config.getMeasurementNoiseMatrix());
        
        // Compute Mahalanobis distance: d = sqrt(innovation^T * S^-1 * innovation)
        double mahalanobis;
        try {
            Matrix3x3 Sinv = S.inverse();
            double[] Sinv_innovation = Sinv.multiply(dx, dy, dh);
            mahalanobis = Math.sqrt(
                dx * Sinv_innovation[0] +
                dy * Sinv_innovation[1] +
                dh * Sinv_innovation[2]
            );
        } catch (ArithmeticException e) {
            // Singular covariance - reject
            reject("Singular covariance matrix");
            return false;
        }
        
        // Use normal threshold (firstMeasurement already handled above)
        if (mahalanobis > config.mahalanobisThreshold) {
            reject(String.format("Mahalanobis distance too large: %.2f > %.2f",
                mahalanobis, config.mahalanobisThreshold));
            return false;
        }
        
        return true;
    }
    
    /**
     * Level 6: Safety validation
     */
    private boolean validateSafety(RobotPose2D predicted, RobotPose2D measurement) {
        // BYPASS for first measurement - allow initial position correction
        if (firstMeasurement) {
            return true;
        }

        // Check innovation magnitude (Euclidean distance)
        double dx = measurement.x - predicted.x;
        double dy = measurement.y - predicted.y;
        double innovationMagnitude = Math.sqrt(dx*dx + dy*dy);
        
        if (innovationMagnitude > config.maxInnovationMagnitude) {
            reject(String.format("Innovation too large: %.1f mm", innovationMagnitude));
            return false;
        }
        
        return true;
    }
    
    /**
     * Record rejection
     */
    private void reject(String reason) {
        lastRejectionReason = reason;
        consecutiveRejections++;
        consecutiveAcceptances = 0;
    }
    
    /**
     * Record acceptance
     */
    private void onAcceptance() {
        lastValidMeasurementTime = System.currentTimeMillis();
        consecutiveRejections = 0;
        consecutiveAcceptances++;
        firstMeasurement = false;
        lastRejectionReason = "";
    }
    
    /**
     * Get last rejection reason
     */
    public String getLastRejectionReason() {
        return lastRejectionReason;
    }
    
    /**
     * Get consecutive rejections count
     */
    public int getConsecutiveRejections() {
        return consecutiveRejections;
    }
    
    /**
     * Get consecutive acceptances count
     */
    public int getConsecutiveAcceptances() {
        return consecutiveAcceptances;
    }
    
    /**
     * Get time since last valid measurement
     */
    public long getTimeSinceLastValid() {
        if (lastValidMeasurementTime == 0) {
            return -1;
        }
        return System.currentTimeMillis() - lastValidMeasurementTime;
    }
    
    /**
     * Reset validator state (useful when starting new localization session)
     */
    public void reset() {
        lastValidMeasurementTime = 0;
        consecutiveRejections = 0;
        consecutiveAcceptances = 0;
        firstMeasurement = true;
        lastRejectionReason = "";
    }
}
