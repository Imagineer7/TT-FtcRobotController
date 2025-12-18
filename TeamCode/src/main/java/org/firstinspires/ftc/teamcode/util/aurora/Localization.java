package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

/**
 * AURORA Localization System
 * 
 * This class provides comprehensive robot localization using sensor fusion between
 * the goBILDA Pinpoint odometry computer and Limelight vision system.
 * 
 * Primary localization source: Pinpoint odometry (continuous tracking)
 * Secondary correction source: Limelight AprilTag positioning (drift correction)
 * 
 * The Pinpoint provides fast, continuous position updates using odometry pods and IMU.
 * The Limelight provides absolute position corrections when AprilTags are visible.
 */
public class Localization {
    
    // Hardware
    private GoBildaPinpointDriver odometry;
    private LimelightVisionHelper limelight;
    
    // Configuration constants
    private static final String ODOMETRY_NAME = "odo";
    
    // Odometry pod offsets (from requirements: strafe X pod offset is 0mm, forward Y pod offset is 201.857mm)
    // Note: These differ from AuroraHardwareConfig which uses inches. Requirements specify mm.
    private static final double STRAFE_X_POD_OFFSET = 0.0; // mm - left/right offset
    private static final double FORWARD_Y_POD_OFFSET = 201.857; // mm - forward/back offset
    
    // Sensor fusion parameters
    private static final double LIMELIGHT_UPDATE_INTERVAL_MS = 500; // minimum time between vision corrections
    private long lastLimelightUpdateTime = 0;
    
    // Position tracking
    private Pose2D currentPose;
    private boolean odometryInitialized = false;
    private boolean limelightInitialized = false;
    
    /**
     * Create a new Localization system
     * @param hardwareMap The OpMode's hardwareMap
     */
    public Localization(HardwareMap hardwareMap) {
        initializeOdometry(hardwareMap);
        initializeLimelight(hardwareMap);
        currentPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    }
    
    /**
     * Create a new Localization system with custom starting position
     * @param hardwareMap The OpMode's hardwareMap
     * @param startingPose Initial robot pose
     */
    public Localization(HardwareMap hardwareMap, Pose2D startingPose) {
        initializeOdometry(hardwareMap);
        initializeLimelight(hardwareMap);
        setPosition(startingPose);
    }
    
    /**
     * Initialize the Pinpoint odometry computer
     */
    private void initializeOdometry(HardwareMap hardwareMap) {
        try {
            odometry = hardwareMap.get(GoBildaPinpointDriver.class, ODOMETRY_NAME);
            
            // Configure odometry offsets
            odometry.setOffsets(STRAFE_X_POD_OFFSET, FORWARD_Y_POD_OFFSET, DistanceUnit.MM);
            
            // Configure encoder directions
            // Forward (X) pod should increase when robot moves forward
            // Strafe (Y) pod should increase when robot moves left
            odometry.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,   // Forward pod direction
                GoBildaPinpointDriver.EncoderDirection.REVERSED   // Strafe pod direction
            );
            
            // Set encoder resolution for goBILDA 4-bar pods
            odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            
            // Reset position and calibrate IMU
            odometry.resetPosAndIMU();
            
            odometryInitialized = true;
        } catch (Exception e) {
            odometry = null;
            odometryInitialized = false;
        }
    }
    
    /**
     * Initialize the Limelight vision system
     */
    private void initializeLimelight(HardwareMap hardwareMap) {
        limelight = new LimelightVisionHelper(hardwareMap);
        limelightInitialized = limelight.isInitialized();
    }
    
    /**
     * Check if odometry is initialized and operational
     */
    public boolean isOdometryInitialized() {
        return odometryInitialized;
    }
    
    /**
     * Check if Limelight is initialized and operational
     */
    public boolean isLimelightInitialized() {
        return limelightInitialized;
    }
    
    /**
     * Update localization data
     * Call this method once per loop to update position tracking
     * 
     * This method:
     * 1. Updates odometry position
     * 2. Checks if vision correction is available and needed
     * 3. Applies vision correction if appropriate
     */
    public void update() {
        // Always update odometry
        if (odometryInitialized && odometry != null) {
            odometry.update();
            currentPose = odometry.getPosition();
        }
        
        // Periodically correct with Limelight if available
        if (shouldUpdateWithLimelight()) {
            applyLimelightCorrection();
        }
    }
    
    /**
     * Update only odometry without vision correction
     * Use this for faster updates when vision correction is not needed
     */
    public void updateOdometryOnly() {
        if (odometryInitialized && odometry != null) {
            odometry.update();
            currentPose = odometry.getPosition();
        }
    }
    
    /**
     * Force a vision correction update
     * Use this when you know the robot is in a good position to see AprilTags
     * 
     * @return true if correction was applied, false otherwise
     */
    public boolean forceVisionCorrection() {
        return applyLimelightCorrection();
    }
    
    /**
     * Get the current robot position
     * @return Current pose (X, Y, Heading)
     */
    public Pose2D getPosition() {
        return currentPose;
    }
    
    /**
     * Get the current X position
     * @param unit Distance unit to return
     * @return X position in specified unit
     */
    public double getX(DistanceUnit unit) {
        return currentPose.getX(unit);
    }
    
    /**
     * Get the current Y position
     * @param unit Distance unit to return
     * @return Y position in specified unit
     */
    public double getY(DistanceUnit unit) {
        return currentPose.getY(unit);
    }
    
    /**
     * Get the current heading
     * @param unit Angle unit to return
     * @return Heading in specified unit
     */
    public double getHeading(AngleUnit unit) {
        return currentPose.getHeading(unit);
    }
    
    /**
     * Get X velocity from odometry
     * @param unit Distance unit to return
     * @return X velocity in unit/sec
     */
    public double getVelocityX(DistanceUnit unit) {
        if (odometry == null) return 0.0;
        return odometry.getVelX(unit);
    }
    
    /**
     * Get Y velocity from odometry
     * @param unit Distance unit to return
     * @return Y velocity in unit/sec
     */
    public double getVelocityY(DistanceUnit unit) {
        if (odometry == null) return 0.0;
        return odometry.getVelY(unit);
    }
    
    /**
     * Get heading velocity from odometry
     * @param unit Angle unit to return
     * @return Heading velocity in unit/sec
     */
    public double getHeadingVelocity(AngleUnit unit) {
        if (odometry == null) return 0.0;
        return unit.fromRadians(odometry.getHeadingVelocity());
    }
    
    /**
     * Set the robot position
     * Use this to set a known starting position or to apply external corrections
     * 
     * @param pose New position to set
     */
    public void setPosition(Pose2D pose) {
        if (odometry != null) {
            odometry.setPosition(pose);
        }
        currentPose = pose;
    }
    
    /**
     * Reset position to origin (0, 0, 0) and recalibrate IMU
     * Robot MUST be stationary when calling this
     */
    public void resetPosition() {
        if (odometry != null) {
            odometry.resetPosAndIMU();
        }
        currentPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    }
    
    /**
     * Reset position to a specific pose and recalibrate IMU
     * Robot MUST be stationary when calling this
     * 
     * @param pose Position to reset to
     */
    public void resetPosition(Pose2D pose) {
        resetPosition();
        setPosition(pose);
    }
    
    /**
     * Calculate distance to a waypoint
     * @param targetX Target X coordinate in mm
     * @param targetY Target Y coordinate in mm
     * @return Distance to target in mm
     */
    public double getDistanceToPoint(double targetX, double targetY) {
        double currentX = currentPose.getX(DistanceUnit.MM);
        double currentY = currentPose.getY(DistanceUnit.MM);
        
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        
        return Math.sqrt(dx*dx + dy*dy);
    }
    
    /**
     * Calculate angle to a waypoint
     * @param targetX Target X coordinate in mm
     * @param targetY Target Y coordinate in mm
     * @return Angle to target in radians
     */
    public double getAngleToPoint(double targetX, double targetY) {
        double currentX = currentPose.getX(DistanceUnit.MM);
        double currentY = currentPose.getY(DistanceUnit.MM);
        
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        
        return Math.atan2(dy, dx);
    }
    
    /**
     * Get the Limelight helper for direct access
     * @return LimelightVisionHelper instance, or null if not initialized
     */
    public LimelightVisionHelper getLimelight() {
        return limelight;
    }
    
    /**
     * Get the odometry computer for direct access
     * @return GoBildaPinpointDriver instance, or null if not initialized
     */
    public GoBildaPinpointDriver getOdometry() {
        return odometry;
    }
    
    /**
     * Get odometry device status
     * @return Device status, or null if odometry not initialized
     */
    public GoBildaPinpointDriver.DeviceStatus getOdometryStatus() {
        if (odometry == null) return null;
        return odometry.getDeviceStatus();
    }
    
    /**
     * Get odometry loop frequency
     * @return Frequency in Hz, or 0 if not initialized
     */
    public double getOdometryFrequency() {
        if (odometry == null) return 0.0;
        return odometry.getFrequency();
    }
    
    /**
     * Check if Limelight data is fresh
     * @return true if vision data is recent and reliable
     */
    public boolean isVisionDataFresh() {
        if (limelight == null) return false;
        return limelight.isDataFresh();
    }
    
    /**
     * Get the age of Limelight data in milliseconds
     * @return age in ms, or -1 if no data available
     */
    public long getVisionDataAge() {
        if (limelight == null) return -1;
        return limelight.getDataAge();
    }
    
    /**
     * Check if Limelight data quality is good
     * @return true if data is fresh and reliable
     */
    public boolean isVisionDataQualityGood() {
        if (limelight == null) return false;
        return limelight.isDataQualityGood();
    }
    
    /**
     * Stop all sensors (call when OpMode ends)
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // PRIVATE HELPER METHODS
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Check if we should update position with Limelight
     */
    private boolean shouldUpdateWithLimelight() {
        if (!limelightInitialized || limelight == null) return false;
        if (!limelight.hasTarget()) return false;
        
        // Check data freshness and quality
        if (!limelight.isDataFresh() || !limelight.isDataQualityGood()) return false;
        
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastLimelightUpdateTime < LIMELIGHT_UPDATE_INTERVAL_MS) {
            return false;
        }
        
        return true;
    }
    
    /**
     * Apply position correction from Limelight
     * @return true if correction was applied, false otherwise
     */
    private boolean applyLimelightCorrection() {
        if (limelight == null || !limelight.hasTarget()) return false;
        
        // Use single reading instead of filtered to avoid blocking
        // The filtering with getFilteredRobotPose() takes ~100ms which is too slow for control loops
        // For most use cases, a single reading with freshness checks is sufficient
        Pose3D visionPose3D = limelight.getRobotPose();
        if (visionPose3D == null) return false;
        
        // Convert 3D pose to 2D pose for odometry
        // Limelight uses field coordinates, which matches our needs
        Pose2D visionPose2D = new Pose2D(
            DistanceUnit.MM,
            visionPose3D.getPosition().x,
            visionPose3D.getPosition().y,
            AngleUnit.RADIANS,
            visionPose3D.getOrientation().getYaw()
        );
        
        // Apply correction to odometry
        setPosition(visionPose2D);
        
        // Update timestamp
        lastLimelightUpdateTime = System.currentTimeMillis();
        
        return true;
    }
}
