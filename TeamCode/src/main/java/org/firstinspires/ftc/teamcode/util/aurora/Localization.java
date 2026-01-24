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
 *
 * Update Modes:
 * - AUTOMATIC: Limelight updates applied automatically when conditions are met
 * - MANUAL: Limelight updates only when explicitly requested via updateWithLimelight()
 * - DISABLED: No Limelight updates, odometry only
 */
public class Localization {
    
    /**
     * Update mode for Limelight sensor fusion
     */
    public enum LimelightUpdateMode {
        AUTOMATIC,  // Auto-update when conditions met (default)
        MANUAL,     // Only update on manual call
        DISABLED    // No Limelight updates
    }

    // Hardware
    private GoBildaPinpointDriver odometry;
    private LimelightVisionHelper limelight;
    
    // Update mode
    private LimelightUpdateMode updateMode = LimelightUpdateMode.AUTOMATIC;

    // Configuration constants
    private static final String ODOMETRY_NAME = "odo";

    // Odometry pod offsets relative to robot center
    // Based on GoBILDA Pinpoint coordinate system:
    // - X pod (measures FORWARD motion): Left of center = positive, Right = negative
    // - Y pod (measures STRAFE motion): Forward of center = positive, Backward = negative
    //
    // Physical configuration:
    // - X pod (forward-measuring): 0mm sideways offset (centered left-right)
    // - Y pod (strafe-measuring): 201.857mm forward of center
    private static final double X_POD_OFFSET = 201.857; // mm - X pod sideways offset (centered)
    private static final double Y_POD_OFFSET = 0.0; // mm - Y pod forward offset

    // Sensor fusion parameters
    private static final double LIMELIGHT_UPDATE_INTERVAL_MS = 500; // minimum time between vision corrections
    private static final double LIMELIGHT_MAX_DISTANCE_MM = 1219; // 4 feet in mm - only trust close targets
    private static final double HEADING_STABILITY_THRESHOLD_DEG = 4.0; // degrees - heading must be within this range
    private static final int HEADING_STABILITY_COUNT = 10; // number of consistent readings needed
    private static final double POSE_JUMP_THRESHOLD_MM = 300; // mm - max position jump allowed
    private static final double MAX_VELOCITY_FOR_UPDATE_MM_PER_SEC = 10; // mm/s - max velocity to allow Limelight updates

    private long lastLimelightUpdateTime = 0;
    
    // Limelight heading stability tracking
    private double lastLimelightHeading = 0.0;
    private int stableHeadingCount = 0;
    private boolean limelightHeadingStable = false;
    private double stableLimelightHeading = 0.0;
    private boolean odometryUpdatedByLimelight = false;  // Track if pose was updated this loop

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
            // setOffsets(xPodOffset, yPodOffset) where:
            // - xPodOffset = sideways offset of forward-measuring pod
            // - yPodOffset = forward offset of strafe-measuring pod
            odometry.setOffsets(X_POD_OFFSET, Y_POD_OFFSET, DistanceUnit.MM);

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
     * Get Limelight initialization error message
     * @return Error message if Limelight failed to initialize, null otherwise
     */
    public String getLimelightInitializationError() {
        if (limelight != null) {
            return limelight.getInitializationError();
        }
        return "Limelight object is null";
    }

    /**
     * Update localization data
     * Call this method once per loop to update position tracking
     * 
     * This method:
     * 1. Updates odometry position
     * 2. Always tracks Limelight heading stability (for telemetry and manual updates)
     * 3. If in AUTOMATIC mode: Applies Limelight updates when conditions met
     * 4. If in MANUAL mode: Only updates odometry (call updateWithLimelight() to use vision)
     * 5. If in DISABLED mode: Only updates odometry
     */
    public void update() {
        // Reset update flag
        odometryUpdatedByLimelight = false;

        // Always update odometry first
        if (odometryInitialized && odometry != null) {
            odometry.update();
            currentPose = odometry.getPosition();
        }
        
        // Always check and track Limelight heading stability (needed for manual updates and telemetry)
        // This runs regardless of mode so stability info is always available
        updateLimelightHeadingStability();

        // Only do automatic Limelight updates if in AUTOMATIC mode
        if (updateMode != LimelightUpdateMode.AUTOMATIC) {
            return;  // Skip automatic Limelight updates
        }


        // If Limelight has stable heading within range AND robot is moving slowly, use it AND update odometry
        if (limelightHeadingStable && shouldUseLimelightHeading() && isVelocityLowEnoughForUpdate()) {
            // Get the full Limelight pose
            Pose3D visionPose3D = limelight.getRobotPose();
            if (visionPose3D != null) {
                // Apply 180-degree rotation to correct field orientation
                // Rotation: new_x = -old_x, new_y = -old_y
                double rawX = visionPose3D.getPosition().x;
                double rawY = visionPose3D.getPosition().y;
                double rotatedX = -rawX;
                double rotatedY = -rawY;

                // Create 2D pose from Limelight data with rotated coordinates
                Pose2D limelightPose = new Pose2D(
                    DistanceUnit.MM,
                    rotatedX,
                    rotatedY,
                    AngleUnit.RADIANS,
                    stableLimelightHeading  // Use the stable heading we've been tracking
                );

                // Check if position change is reasonable (not a huge jump)
                double currentX = currentPose.getX(DistanceUnit.MM);
                double currentY = currentPose.getY(DistanceUnit.MM);
                double dx = limelightPose.getX(DistanceUnit.MM) - currentX;
                double dy = limelightPose.getY(DistanceUnit.MM) - currentY;
                double positionJump = Math.sqrt(dx*dx + dy*dy);

                if (positionJump <= POSE_JUMP_THRESHOLD_MM) {
                    // Position change is reasonable - update odometry to match Limelight
                    // This ensures smooth handoff when we lose sight of AprilTag
                    setPosition(limelightPose);
                    currentPose = limelightPose;
                    odometryUpdatedByLimelight = true;  // Mark that we updated odometry
                } else {
                    // Position jump too large - only use heading, keep position from odometry
                    currentPose = new Pose2D(
                        DistanceUnit.MM,
                        currentX,
                        currentY,
                        AngleUnit.RADIANS,
                        stableLimelightHeading
                    );
                }
            } else {
                // Fallback: just override heading if pose is null
                currentPose = new Pose2D(
                    DistanceUnit.MM,
                    currentPose.getX(DistanceUnit.MM),
                    currentPose.getY(DistanceUnit.MM),
                    AngleUnit.RADIANS,
                    stableLimelightHeading
                );
            }
        } else if (shouldUpdateWithLimelight()) {
            // Fallback: Apply full pose correction if conditions are met
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
     * Set the Limelight update mode
     * @param mode Update mode (AUTOMATIC, MANUAL, or DISABLED)
     */
    public void setLimelightUpdateMode(LimelightUpdateMode mode) {
        this.updateMode = mode;
    }

    /**
     * Get the current Limelight update mode
     * @return Current update mode
     */
    public LimelightUpdateMode getLimelightUpdateMode() {
        return updateMode;
    }

    /**
     * Manually trigger a Limelight update
     * This method respects all safety checks (velocity, distance, stability, etc.)
     * Use this when in MANUAL mode to explicitly request a vision correction
     *
     * @return true if update was applied, false if conditions not met
     */
    public boolean updateWithLimelight() {
        if (!limelightInitialized || limelight == null) {
            return false;
        }

        // Reset flag
        odometryUpdatedByLimelight = false;

        // Check and track heading stability
        updateLimelightHeadingStability();

        // Try to apply update with all safety checks
        if (limelightHeadingStable && shouldUseLimelightHeading() && isVelocityLowEnoughForUpdate()) {
            // Get the full Limelight pose
            Pose3D visionPose3D = limelight.getRobotPose();
            if (visionPose3D != null) {
                // Apply 180-degree rotation to correct field orientation
                // Rotation: new_x = -old_x, new_y = -old_y
                double rawX = visionPose3D.getPosition().x;
                double rawY = visionPose3D.getPosition().y;
                double rotatedX = -rawX;
                double rotatedY = -rawY;

                // Create 2D pose from Limelight data with rotated coordinates
                Pose2D limelightPose = new Pose2D(
                    DistanceUnit.MM,
                    rotatedX,
                    rotatedY,
                    AngleUnit.RADIANS,
                    stableLimelightHeading
                );

                // Check if position change is reasonable
                double currentX = currentPose.getX(DistanceUnit.MM);
                double currentY = currentPose.getY(DistanceUnit.MM);
                double dx = limelightPose.getX(DistanceUnit.MM) - currentX;
                double dy = limelightPose.getY(DistanceUnit.MM) - currentY;
                double positionJump = Math.sqrt(dx*dx + dy*dy);

                if (positionJump <= POSE_JUMP_THRESHOLD_MM) {
                    // Update odometry to match Limelight
                    setPosition(limelightPose);
                    currentPose = limelightPose;
                    odometryUpdatedByLimelight = true;
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Force a vision correction update (bypasses mode check)
     * Use this when you know the robot is in a good position to see AprilTags
     * Still respects safety checks (velocity, distance, stability)
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
     * Check if Limelight heading is stable
     * @return true if heading has been consistent over multiple readings
     */
    public boolean isLimelightHeadingStable() {
        return limelightHeadingStable;
    }

    /**
     * Get the stable Limelight heading (if stable)
     * @param unit Angle unit to return
     * @return Stable heading, or 0 if not stable
     */
    public double getStableLimelightHeading(AngleUnit unit) {
        if (!limelightHeadingStable) return 0.0;
        return unit.fromRadians(stableLimelightHeading);
    }

    /**
     * Get the number of consecutive stable heading readings
     * @return Count of stable readings (0-N)
     */
    public int getStableHeadingCount() {
        return stableHeadingCount;
    }

    /**
     * Check if odometry was updated with Limelight pose this loop
     * @return true if odometry pose was corrected with Limelight data
     */
    public boolean wasOdometryUpdatedByLimelight() {
        return odometryUpdatedByLimelight;
    }

    /**
     * Get current robot velocity magnitude
     * @param unit Distance unit to return
     * @return Velocity magnitude in unit/sec
     */
    public double getVelocityMagnitude(DistanceUnit unit) {
        if (odometry == null) return 0.0;
        double vx = odometry.getVelX(unit);
        double vy = odometry.getVelY(unit);
        return Math.sqrt(vx*vx + vy*vy);
    }

    /**
     * Check if robot velocity is low enough for Limelight updates
     * @return true if velocity is within safe threshold
     */
    public boolean isVelocityLowForUpdate() {
        return isVelocityLowEnoughForUpdate();
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
        
        // Calculate distance to target (approximate using Z distance)
        double distanceToTarget = Math.abs(visionPose3D.getPosition().z);

        // Only apply full pose correction if within maximum distance and position won't jump too much
        if (distanceToTarget > LIMELIGHT_MAX_DISTANCE_MM) {
            return false;
        }

        // Check if position jump would be too large (indicates bad reading)
        double currentX = currentPose.getX(DistanceUnit.MM);
        double currentY = currentPose.getY(DistanceUnit.MM);

        // Apply 180-degree rotation to correct field orientation
        // Rotation: new_x = -old_x, new_y = -old_y
        double rawX = visionPose3D.getPosition().x;
        double rawY = visionPose3D.getPosition().y;
        double rotatedX = -rawX;
        double rotatedY = -rawY;

        double dx = rotatedX - currentX;
        double dy = rotatedY - currentY;
        double positionJump = Math.sqrt(dx*dx + dy*dy);

        if (positionJump > POSE_JUMP_THRESHOLD_MM) {
            // Position jump too large - likely bad reading, skip
            return false;
        }

        // Convert 3D pose to 2D pose for odometry with rotated coordinates
        // Limelight uses field coordinates, which matches our needs
        Pose2D visionPose2D = new Pose2D(
            DistanceUnit.MM,
            rotatedX,
            rotatedY,
            AngleUnit.RADIANS,
            visionPose3D.getOrientation().getYaw()
        );
        
        // Apply correction to odometry
        setPosition(visionPose2D);
        
        // Update timestamp
        lastLimelightUpdateTime = System.currentTimeMillis();
        
        return true;
    }

    /**
     * Update Limelight heading stability tracking
     * Checks if heading is consistent over multiple readings
     */
    private void updateLimelightHeadingStability() {
        if (!limelightInitialized || limelight == null || !limelight.hasTarget()) {
            // No target - reset stability tracking
            stableHeadingCount = 0;
            limelightHeadingStable = false;
            return;
        }

        Pose3D visionPose = limelight.getRobotPose();
        if (visionPose == null) {
            stableHeadingCount = 0;
            limelightHeadingStable = false;
            return;
        }

        double currentHeading = visionPose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Check if heading is stable (within threshold of last reading)
        double headingDiff = Math.abs(currentHeading - lastLimelightHeading);

        // Handle wraparound (e.g., 359° to 1° is only 2° difference)
        if (headingDiff > 180) {
            headingDiff = 360 - headingDiff;
        }

        if (headingDiff <= HEADING_STABILITY_THRESHOLD_DEG) {
            // Heading is stable - increment counter
            stableHeadingCount++;

            if (stableHeadingCount >= HEADING_STABILITY_COUNT) {
                // We have enough stable readings
                limelightHeadingStable = true;
                stableLimelightHeading = AngleUnit.RADIANS.fromDegrees(currentHeading);
            }
        } else {
            // Heading changed too much - reset
            stableHeadingCount = 0;
            limelightHeadingStable = false;
        }

        lastLimelightHeading = currentHeading;
    }

    /**
     * Check if we should use Limelight heading directly
     * @return true if Limelight heading is stable and target is within range
     */
    private boolean shouldUseLimelightHeading() {
        if (!limelightInitialized || limelight == null || !limelight.hasTarget()) {
            return false;
        }

        // Check data quality
        if (!limelight.isDataFresh() || !limelight.isDataQualityGood()) {
            return false;
        }

        Pose3D visionPose = limelight.getRobotPose();
        if (visionPose == null) {
            return false;
        }

        // Check distance to target (using Z distance as approximation)
        double distanceToTarget = Math.abs(visionPose.getPosition().z);

        // Only use Limelight heading if target is within maximum distance
        return distanceToTarget <= LIMELIGHT_MAX_DISTANCE_MM;
    }

    /**
     * Check if robot velocity is low enough for accurate Limelight updates
     * @return true if robot is moving slowly enough
     */
    private boolean isVelocityLowEnoughForUpdate() {
        if (odometry == null) return false;

        // Get velocity components
        double vx = odometry.getVelX(DistanceUnit.MM);
        double vy = odometry.getVelY(DistanceUnit.MM);

        // Calculate total velocity magnitude
        double velocityMagnitude = Math.sqrt(vx*vx + vy*vy);

        // Only allow updates if moving slowly
        return velocityMagnitude <= MAX_VELOCITY_FOR_UPDATE_MM_PER_SEC;
    }
}
