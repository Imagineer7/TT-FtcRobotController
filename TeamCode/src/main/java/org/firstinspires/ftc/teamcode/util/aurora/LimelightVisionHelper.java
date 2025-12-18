package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

/**
 * AURORA Limelight Vision Helper
 * 
 * This class provides an interface for the Limelight 3A vision system.
 * It handles target detection, pose estimation, and AprilTag reading with
 * advanced filtering to reduce noise and outliers.
 * 
 * Pipeline Configuration:
 * - Pipeline 3: AprilTags positioning
 * - Pipeline 2: Obelisk AprilTags for motif pattern (ID 21="GPP", 22="PGP", 23="PPG")
 */
public class LimelightVisionHelper {
    
    // Hardware
    private Limelight3A limelight;
    
    // Configuration constants
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int POSITIONING_PIPELINE = 3;
    private static final int OBELISK_PIPELINE = 2;
    
    // Filtering constants for pose readings
    private static final int NUM_READINGS = 10;
    private static final double POSITION_THRESHOLD = 100.0; // mm - readings within this distance are grouped
    private static final double MIN_GROUP_SIZE = 3; // minimum readings in a group to consider it valid
    
    // Data freshness tracking
    private static final long STALE_DATA_THRESHOLD_MS = 500; // data older than this is considered stale
    private long lastValidDataTimestamp = 0;
    private long lastUpdateAttemptTimestamp = 0;
    private int consecutiveFailedReads = 0;
    
    // Camera configuration (adjust based on your robot)
    private double cameraHeightMM = 200.0; // height of camera above ground
    private double cameraMountAngleDeg = 0.0; // angle of camera mount
    
    /**
     * Create a new LimelightVisionHelper
     * @param hardwareMap The OpMode's hardwareMap
     */
    public LimelightVisionHelper(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.pipelineSwitch(POSITIONING_PIPELINE);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }
    
    /**
     * Check if Limelight is initialized and operational
     */
    public boolean isInitialized() {
        return limelight != null;
    }
    
    /**
     * Check if any valid targets are visible
     */
    public boolean hasTarget() {
        if (limelight == null) return false;
        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();
        
        // Update freshness tracking
        lastUpdateAttemptTimestamp = System.currentTimeMillis();
        if (hasTarget) {
            lastValidDataTimestamp = System.currentTimeMillis();
            consecutiveFailedReads = 0;
        } else {
            consecutiveFailedReads++;
        }
        
        return hasTarget;
    }
    
    /**
     * Get the horizontal offset to the target in degrees
     * @return horizontal offset (tx), or 0 if no target
     */
    public double getTargetX() {
        if (!hasTarget()) return 0.0;
        return limelight.getLatestResult().getTx();
    }
    
    /**
     * Get the vertical offset to the target in degrees
     * @return vertical offset (ty), or 0 if no target
     */
    public double getTargetY() {
        if (!hasTarget()) return 0.0;
        return limelight.getLatestResult().getTy();
    }
    
    /**
     * Get the target area as percentage of image
     * @return target area (0-100), or 0 if no target
     */
    public double getTargetArea() {
        if (!hasTarget()) return 0.0;
        return limelight.getLatestResult().getTa();
    }
    
    /**
     * Calculate distance to target based on known target height
     * Uses the formula: distance = (targetHeight - cameraHeight) / tan(mountAngle + ty)
     * 
     * @param targetHeightMM height of the target above ground in mm
     * @return distance to target in mm, or -1 if no target
     */
    public double getDistanceToTarget(double targetHeightMM) {
        if (!hasTarget()) return -1.0;
        
        double ty = getTargetY();
        double angleToTarget = cameraMountAngleDeg + ty;
        double heightDifference = targetHeightMM - cameraHeightMM;
        
        // distance = height / tan(angle)
        double distance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
        return Math.abs(distance);
    }
    
    /**
     * Set camera height above ground
     * @param heightMM camera height in mm
     */
    public void setCameraHeight(double heightMM) {
        this.cameraHeightMM = heightMM;
    }
    
    /**
     * Set camera mount angle
     * @param angleDeg angle in degrees (positive is tilted up)
     */
    public void setCameraMountAngle(double angleDeg) {
        this.cameraMountAngleDeg = angleDeg;
    }
    
    /**
     * Switch to positioning pipeline (Pipeline 3)
     */
    public void setPositioningPipeline() {
        if (limelight != null) {
            limelight.pipelineSwitch(POSITIONING_PIPELINE);
        }
    }
    
    /**
     * Switch to obelisk reading pipeline (Pipeline 2)
     */
    public void setObeliskPipeline() {
        if (limelight != null) {
            limelight.pipelineSwitch(OBELISK_PIPELINE);
        }
    }
    
    /**
     * Set LED mode
     * @param mode 0=pipeline default, 1=force off, 2=force blink, 3=force on
     */
    public void setLEDMode(int mode) {
        // LED control is typically done through the Limelight web interface
        // The SDK doesn't expose direct LED control, so this is a placeholder
        // In practice, LED mode is configured per-pipeline
    }
    
    /**
     * Get robot pose from Limelight with filtering
     * Takes multiple readings, groups similar readings, and averages the largest group
     * 
     * @return filtered robot pose, or null if no valid pose available
     */
    public Pose3D getFilteredRobotPose() {
        if (limelight == null) return null;
        
        List<Pose3D> readings = new ArrayList<>();
        
        // Collect multiple readings
        for (int i = 0; i < NUM_READINGS; i++) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D pose = result.getBotpose();
                if (pose != null) {
                    readings.add(pose);
                }
            }
            // Small delay between readings
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        
        if (readings.isEmpty()) return null;
        if (readings.size() == 1) return readings.get(0);
        
        // Group readings by proximity
        List<List<Pose3D>> groups = groupReadings(readings);
        
        // Find the largest group
        List<Pose3D> largestGroup = groups.get(0);
        for (List<Pose3D> group : groups) {
            if (group.size() > largestGroup.size()) {
                largestGroup = group;
            }
        }
        
        // Only use the group if it has enough readings
        if (largestGroup.size() < MIN_GROUP_SIZE) {
            return null;
        }
        
        // Average the largest group
        return averagePoses(largestGroup);
    }
    
    /**
     * Get robot pose from a single reading (no filtering)
     * Use getFilteredRobotPose() for more reliable results
     * 
     * @return robot pose, or null if no valid pose
     */
    public Pose3D getRobotPose() {
        if (!hasTarget()) return null;
        return limelight.getLatestResult().getBotpose();
    }
    
    /**
     * Get fiducial (AprilTag) results
     * @return list of detected AprilTags
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (!hasTarget()) return new ArrayList<>();
        return limelight.getLatestResult().getFiducialResults();
    }
    
    /**
     * Get obelisk motif pattern from AprilTag ID
     * @param tagId AprilTag ID (21, 22, or 23)
     * @return motif pattern string ("GPP", "PGP", "PPG"), or null if invalid
     */
    public String getObeliskPattern(int tagId) {
        switch (tagId) {
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            default: return null;
        }
    }
    
    /**
     * Read obelisk pattern from visible AprilTags
     * Switches to obelisk pipeline, reads tags, and returns to positioning pipeline
     * 
     * @return detected motif pattern, or null if none found
     */
    public String readObeliskPattern() {
        if (limelight == null) return null;
        
        // Switch to obelisk pipeline
        setObeliskPipeline();
        
        // Wait for pipeline switch
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        // Read fiducials
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        String pattern = null;
        
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            String p = getObeliskPattern(id);
            if (p != null) {
                pattern = p;
                break;
            }
        }
        
        // Switch back to positioning pipeline
        setPositioningPipeline();
        
        return pattern;
    }
    
    /**
     * Get Limelight status information
     */
    public LLStatus getStatus() {
        if (limelight == null) return null;
        return limelight.getStatus();
    }
    
    /**
     * Get the latest result from Limelight
     */
    public LLResult getLatestResult() {
        if (limelight == null) return null;
        return limelight.getLatestResult();
    }
    
    /**
     * Check if Limelight data is fresh (recent)
     * @return true if data was updated recently, false if stale
     */
    public boolean isDataFresh() {
        if (lastValidDataTimestamp == 0) return false;
        long age = System.currentTimeMillis() - lastValidDataTimestamp;
        return age < STALE_DATA_THRESHOLD_MS;
    }
    
    /**
     * Get the age of the last valid data in milliseconds
     * @return age in ms, or -1 if no valid data has been received
     */
    public long getDataAge() {
        if (lastValidDataTimestamp == 0) return -1;
        return System.currentTimeMillis() - lastValidDataTimestamp;
    }
    
    /**
     * Get the timestamp of the last valid data
     * @return timestamp in ms, or 0 if no valid data
     */
    public long getLastValidDataTimestamp() {
        return lastValidDataTimestamp;
    }
    
    /**
     * Get the number of consecutive failed reads
     * @return number of failed reads since last valid data
     */
    public int getConsecutiveFailedReads() {
        return consecutiveFailedReads;
    }
    
    /**
     * Check if data quality is good
     * Considers both freshness and read success rate
     * @return true if data is fresh and reliable
     */
    public boolean isDataQualityGood() {
        return isDataFresh() && consecutiveFailedReads < 5;
    }
    
    /**
     * Reset freshness tracking (useful after long periods without updates)
     */
    public void resetFreshnessTracking() {
        lastValidDataTimestamp = 0;
        lastUpdateAttemptTimestamp = 0;
        consecutiveFailedReads = 0;
    }
    
    /**
     * Stop the Limelight (call when OpMode ends)
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
     * Group pose readings by proximity
     */
    private List<List<Pose3D>> groupReadings(List<Pose3D> readings) {
        List<List<Pose3D>> groups = new ArrayList<>();
        
        for (Pose3D reading : readings) {
            boolean addedToGroup = false;
            
            // Try to add to existing group
            for (List<Pose3D> group : groups) {
                if (isInGroup(reading, group)) {
                    group.add(reading);
                    addedToGroup = true;
                    break;
                }
            }
            
            // Create new group if not added
            if (!addedToGroup) {
                List<Pose3D> newGroup = new ArrayList<>();
                newGroup.add(reading);
                groups.add(newGroup);
            }
        }
        
        return groups;
    }
    
    /**
     * Check if a reading belongs to a group
     */
    private boolean isInGroup(Pose3D reading, List<Pose3D> group) {
        if (group.isEmpty()) return false;
        
        // Check distance to any member of the group
        for (Pose3D member : group) {
            double distance = calculateDistance(reading, member);
            if (distance <= POSITION_THRESHOLD) {
                return true;
            }
        }
        
        return false;
    }
    
    /**
     * Calculate Euclidean distance between two poses
     */
    private double calculateDistance(Pose3D pose1, Pose3D pose2) {
        double dx = pose1.getPosition().x - pose2.getPosition().x;
        double dy = pose1.getPosition().y - pose2.getPosition().y;
        double dz = pose1.getPosition().z - pose2.getPosition().z;
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    /**
     * Average a list of poses
     */
    private Pose3D averagePoses(List<Pose3D> poses) {
        if (poses.isEmpty()) return null;
        
        double sumX = 0, sumY = 0, sumZ = 0;
        double sumYaw = 0, sumPitch = 0, sumRoll = 0;
        
        for (Pose3D pose : poses) {
            sumX += pose.getPosition().x;
            sumY += pose.getPosition().y;
            sumZ += pose.getPosition().z;
            sumYaw += pose.getOrientation().getYaw();
            sumPitch += pose.getOrientation().getPitch();
            sumRoll += pose.getOrientation().getRoll();
        }
        
        int count = poses.size();
        return new Pose3D(
            sumX / count, sumY / count, sumZ / count,
            sumYaw / count, sumPitch / count, sumRoll / count
        );
    }
}
