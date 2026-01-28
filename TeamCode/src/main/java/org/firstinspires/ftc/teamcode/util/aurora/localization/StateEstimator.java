package org.firstinspires.ftc.teamcode.util.aurora.localization;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

/**
 * Extended Kalman Filter (EKF) prediction step.
 * 
 * Implements the predict phase of state estimation:
 * 1. Read odometry delta and IMU heading
 * 2. Integrate motion model
 * 3. Propagate covariance (uncertainty grows)
 * 4. Update velocity estimates
 * 
 * Motion Model:
 * - Uses differential drive kinematics
 * - IMU provides absolute heading (no drift)
 * - Odometry provides relative position (accumulates drift)
 */
public class StateEstimator {
    
    private final LocalizationConfig config;
    private final GoBildaPinpointDriver odometry;
    
    private Pose2D lastOdometryPose;
    private long lastUpdateTime;
    private boolean initialized = false;
    
    /**
     * Create state estimator
     * 
     * @param config Configuration parameters
     * @param odometry Odometry computer (from AuroraHardwareConfig)
     */
    public StateEstimator(LocalizationConfig config, GoBildaPinpointDriver odometry) {
        this.config = config;
        this.odometry = odometry;
        this.lastUpdateTime = System.currentTimeMillis();
    }
    
    /**
     * Initialize estimator with starting pose
     * Must be called before first predict()
     */
    public void initialize(RobotPose2D startPose) {
        if (odometry != null) {
            // Set odometry to starting pose
            odometry.setPosition(startPose.toPose2D());
            lastOdometryPose = startPose.toPose2D();
        }
        lastUpdateTime = System.currentTimeMillis();
        initialized = true;
    }
    
    /**
     * Predict new pose from odometry
     * 
     * Process:
     * 1. Read current odometry pose
     * 2. Compute delta since last update
     * 3. Apply motion model
     * 4. Propagate covariance
     * 5. Update velocities
     * 
     * @param currentPose Current pose estimate
     * @return Predicted pose after motion
     */
    public RobotPose2D predict(RobotPose2D currentPose) {
        if (!initialized || odometry == null) {
            return currentPose.copy();
        }
        
        // Update odometry
        odometry.update();
        
        // Get current odometry reading
        Pose2D currentOdometryPose = odometry.getPosition();
        
        // Compute delta from last update
        double dx_odo = currentOdometryPose.getX(DistanceUnit.MM) - 
                        lastOdometryPose.getX(DistanceUnit.MM);
        double dy_odo = currentOdometryPose.getY(DistanceUnit.MM) - 
                        lastOdometryPose.getY(DistanceUnit.MM);
        
        // Get current heading from IMU (via odometry computer)
        double currentHeading = currentOdometryPose.getHeading(AngleUnit.RADIANS);
        
        // Apply motion model
        // Transform odometry delta to field frame using current heading
        // Note: This is a simplification; full EKF would use midpoint integration
        double cos_h = Math.cos(currentPose.heading);
        double sin_h = Math.sin(currentPose.heading);
        
        double dx_field = cos_h * dx_odo - sin_h * dy_odo;
        double dy_field = sin_h * dx_odo + cos_h * dy_odo;
        
        // Predict new state
        RobotPose2D predicted = new RobotPose2D();
        predicted.x = currentPose.x + dx_field;
        predicted.y = currentPose.y + dy_field;
        predicted.heading = currentHeading;  // Direct from IMU
        predicted.normalizeHeading();
        
        // Compute time delta
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastUpdateTime) / 1000.0;  // seconds
        if (dt < 0.001) dt = 0.001;  // Minimum 1ms
        
        // Estimate velocities from deltas
        predicted.vx = dx_field / dt;
        predicted.vy = dy_field / dt;
        predicted.vheading = RobotPose2D.angleWrap(currentHeading - currentPose.heading) / dt;
        
        // Propagate covariance: P_new = F * P_old * F^T + Q
        // For simple motion model with IMU heading, F ≈ I (identity)
        // So: P_new = P_old + Q
        Matrix3x3 Q = config.getProcessNoiseMatrix();
        
        // Scale process noise by time delta (more time = more drift)
        // But cap scaling to avoid explosion during slow updates
        double timeScale = Math.min(dt * 10.0, 5.0);  // Cap at 5x
        Q = Q.multiply(timeScale);
        
        predicted.covariance = currentPose.covariance.add(Q);
        predicted.covariance.enforceSymmetry();
        
        // Safety: Cap covariance growth
        if (predicted.covariance.trace() > config.maxCovarianceTrace) {
            double scale = Math.sqrt(config.maxCovarianceTrace / predicted.covariance.trace());
            predicted.covariance = predicted.covariance.multiply(scale);
        }
        
        // Update timestamp
        predicted.timestamp = currentTime;
        
        // Save for next iteration
        lastOdometryPose = currentOdometryPose;
        lastUpdateTime = currentTime;
        
        return predicted;
    }
    
    /**
     * Get current odometry velocity
     * @return Velocity magnitude in mm/s
     */
    public double getVelocityMagnitude() {
        if (odometry == null) return 0.0;
        double vx = odometry.getVelX(DistanceUnit.MM);
        double vy = odometry.getVelY(DistanceUnit.MM);
        return Math.sqrt(vx*vx + vy*vy);
    }
    
    /**
     * Get current angular velocity
     * @return Angular velocity in rad/s
     */
    public double getAngularVelocity() {
        if (odometry == null) return 0.0;
        return odometry.getHeadingVelocity();
    }
    
    /**
     * Get X velocity component
     */
    public double getVelocityX() {
        if (odometry == null) return 0.0;
        return odometry.getVelX(DistanceUnit.MM);
    }
    
    /**
     * Get Y velocity component
     */
    public double getVelocityY() {
        if (odometry == null) return 0.0;
        return odometry.getVelY(DistanceUnit.MM);
    }
    
    /**
     * Check if estimator is initialized
     */
    public boolean isInitialized() {
        return initialized;
    }
    
    /**
     * Get odometry status
     */
    public GoBildaPinpointDriver.DeviceStatus getOdometryStatus() {
        if (odometry == null) return null;
        return odometry.getDeviceStatus();
    }
    
    /**
     * Get odometry update frequency
     */
    public double getOdometryFrequency() {
        if (odometry == null) return 0.0;
        return odometry.getFrequency();
    }
    
    /**
     * Check if odometry is healthy
     */
    public boolean isOdometryHealthy() {
        if (odometry == null) return false;
        
        // Check status
        GoBildaPinpointDriver.DeviceStatus status = odometry.getDeviceStatus();
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            return false;
        }
        
        // Check update rate
        double freq = odometry.getFrequency();
        if (freq < 20.0) {  // Should be ~50Hz or higher
            return false;
        }
        
        return true;
    }
    
    /**
     * Reset estimator (for manual position reset)
     */
    public void reset(RobotPose2D newPose) {
        initialize(newPose);
    }
}
