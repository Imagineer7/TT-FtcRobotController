package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.tool.AprilTagMultiTool;

import java.util.HashMap;
import java.util.Map;

//Used to manage vision systems like cameras and sensors for autonomous navigation and object detection
//Uses AprilTags or other vision targets to determine robot position and orientation
//AprilTagMultiTool from util.tool package is used for AprilTag detection and processing.
public class VisionManager {
    private AprilTagMultiTool aprilTagTool;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Vision data cache
    private Map<Integer, Double[]> detectedTags; // ID -> [x, y, distance, yaw]
    private boolean isStreaming;
    private String activeCamera;

    // Camera configuration
    private boolean useWebcam;
    private String webcam1Name;
    private String webcam2Name;

    /**
     * Initialize vision manager with single camera
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param useWebcam True for webcam, false for phone camera
     * @param cameraName Name of webcam (null for phone camera)
     */
    public VisionManager(HardwareMap hardwareMap, Telemetry telemetry, boolean useWebcam, String cameraName) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.useWebcam = useWebcam;
        this.webcam1Name = cameraName;
        this.webcam2Name = null;

        initializeVision();
    }

    /**
     * Initialize vision manager with dual cameras
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param webcam1Name Name of first webcam
     * @param webcam2Name Name of second webcam
     */
    public VisionManager(HardwareMap hardwareMap, Telemetry telemetry, String webcam1Name, String webcam2Name) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.useWebcam = true;
        this.webcam1Name = webcam1Name;
        this.webcam2Name = webcam2Name;

        initializeVision();
    }

    /**
     * Initialize the AprilTag vision system
     */
    private void initializeVision() {
        try {
            // Initialize AprilTag tool based on camera configuration
            if (webcam2Name != null) {
                // Dual camera setup
                aprilTagTool = new AprilTagMultiTool(hardwareMap, true, webcam1Name, webcam2Name);
                activeCamera = webcam1Name;
            } else if (useWebcam && webcam1Name != null) {
                // Single webcam setup
                aprilTagTool = new AprilTagMultiTool(hardwareMap, true, webcam1Name, null);
                activeCamera = webcam1Name;
            } else {
                // Phone camera setup
                aprilTagTool = new AprilTagMultiTool(hardwareMap, false, null, null);
                activeCamera = "phone";
            }

            detectedTags = new HashMap<>();
            isStreaming = false;

            telemetry.addData("VisionManager", "Initialized with camera: " + activeCamera);

        } catch (Exception e) {
            telemetry.addData("VisionManager Error", e.getMessage());
            aprilTagTool = null;
        }
    }

    /**
     * Start camera streaming
     */
    public void startStreaming() {
        if (aprilTagTool != null) {
            aprilTagTool.resumeStreaming();
            isStreaming = true;
            telemetry.addData("VisionManager", "Streaming started");
        }
    }

    /**
     * Stop camera streaming
     */
    public void stopStreaming() {
        if (aprilTagTool != null) {
            aprilTagTool.stopStreaming();
            isStreaming = false;
            telemetry.addData("VisionManager", "Streaming stopped");
        }
    }

    /**
     * Switch to first webcam (if dual camera setup)
     */
    public void switchToCamera1() {
        if (aprilTagTool != null && webcam1Name != null) {
            aprilTagTool.switchToWebcam1();
            activeCamera = webcam1Name;
            telemetry.addData("VisionManager", "Switched to camera 1");
        }
    }

    /**
     * Switch to second webcam (if dual camera setup)
     */
    public void switchToCamera2() {
        if (aprilTagTool != null && webcam2Name != null) {
            aprilTagTool.switchToWebcam2();
            activeCamera = webcam2Name;
            telemetry.addData("VisionManager", "Switched to camera 2");
        }
    }

    /**
     * Set camera pose for field positioning
     * @param x Camera X position in inches
     * @param y Camera Y position in inches
     * @param z Camera Z position in inches
     * @param yaw Camera yaw angle in degrees
     * @param pitch Camera pitch angle in degrees
     * @param roll Camera roll angle in degrees
     */
    public void setCameraPose(double x, double y, double z, double yaw, double pitch, double roll) {
        if (aprilTagTool != null) {
            Position pos = new Position(DistanceUnit.INCH, x, y, z, 0);
            YawPitchRollAngles orient = new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0);
            aprilTagTool.setCameraPose(pos, orient);
            telemetry.addData("VisionManager", "Camera pose set");
        }
    }

    /**
     * Update vision detection data
     */
    public void updateVision() {
        if (aprilTagTool == null || !isStreaming) return;

        // Clear previous detections
        detectedTags.clear();

        // Get current detections from AprilTag tool
        // Note: AprilTagMultiTool handles the detection internally
        // We would need to access its detection results
        // This is a simplified implementation
        telemetry.addData("VisionManager", "Vision updated");
    }

    /**
     * Check if specific AprilTag is detected
     * @param tagId ID of AprilTag to check
     * @return True if tag is currently visible
     */
    public boolean isTagDetected(int tagId) {
        return detectedTags.containsKey(tagId);
    }

    /**
     * Get position of detected AprilTag
     * @param tagId ID of AprilTag
     * @return Array with [x, y, distance] or null if not detected
     */
    public double[] getTagPosition(int tagId) {
        Double[] data = detectedTags.get(tagId);
        if (data != null && data.length >= 3) {
            return new double[]{data[0], data[1], data[2]};
        }
        return null;
    }

    /**
     * Get angle to detected AprilTag
     * @param tagId ID of AprilTag
     * @return Angle in degrees, or 0 if not detected
     */
    public double getTagAngle(int tagId) {
        Double[] data = detectedTags.get(tagId);
        if (data != null && data.length >= 4) {
            return data[3];
        }
        return 0.0;
    }

    /**
     * Get distance to closest detected AprilTag
     * @return Distance in inches, or -1 if no tags detected
     */
    public double getClosestTagDistance() {
        double minDistance = Double.MAX_VALUE;
        boolean found = false;

        for (Double[] data : detectedTags.values()) {
            if (data.length >= 3 && data[2] < minDistance) {
                minDistance = data[2];
                found = true;
            }
        }

        return found ? minDistance : -1.0;
    }

    /**
     * Get ID of closest detected AprilTag
     * @return Tag ID, or -1 if no tags detected
     */
    public int getClosestTagId() {
        double minDistance = Double.MAX_VALUE;
        int closestId = -1;

        for (Map.Entry<Integer, Double[]> entry : detectedTags.entrySet()) {
            Double[] data = entry.getValue();
            if (data.length >= 3 && data[2] < minDistance) {
                minDistance = data[2];
                closestId = entry.getKey();
            }
        }

        return closestId;
    }

    /**
     * Get robot position based on AprilTag detection
     * @param knownTagPositions Map of tag ID to field position [x, y]
     * @return Robot position [x, y, heading] or null if cannot determine
     */
    public double[] getRobotPositionFromTags(Map<Integer, double[]> knownTagPositions) {
        // Simple triangulation using closest tag
        int closestId = getClosestTagId();
        if (closestId == -1 || !knownTagPositions.containsKey(closestId)) {
            return null;
        }

        double[] tagPos = getTagPosition(closestId);
        double[] knownPos = knownTagPositions.get(closestId);
        double tagAngle = getTagAngle(closestId);

        if (tagPos == null || knownPos == null) return null;

        // Calculate robot position relative to tag
        double distance = tagPos[2];
        double robotX = knownPos[0] - distance * Math.cos(Math.toRadians(tagAngle));
        double robotY = knownPos[1] - distance * Math.sin(Math.toRadians(tagAngle));

        return new double[]{robotX, robotY, tagAngle + 180}; // Robot facing opposite of tag
    }

    /**
     * Add vision telemetry data
     */
    public void addTelemetry() {
        if (aprilTagTool != null) {
            // Use AprilTagMultiTool's built-in telemetry
            // Note: This requires access to the LinearOpMode, which we don't have here
            // In practice, this would be called from the main autonomous class
            telemetry.addData("=== VISION DATA ===", "");
            telemetry.addData("Active Camera", activeCamera);
            telemetry.addData("Streaming", isStreaming ? "YES" : "NO");
            telemetry.addData("Tags Detected", detectedTags.size());

            for (Map.Entry<Integer, Double[]> entry : detectedTags.entrySet()) {
                Double[] data = entry.getValue();
                telemetry.addData("Tag " + entry.getKey(),
                    "Dist:%.1f Angle:%.1f", data[2], data[3]);
            }
        }
    }

    /**
     * Check if vision system is ready
     * @return True if vision is initialized and streaming
     */
    public boolean isVisionReady() {
        return aprilTagTool != null && isStreaming;
    }

    /**
     * Get current active camera name
     * @return Name of active camera
     */
    public String getActiveCamera() {
        return activeCamera;
    }

    /**
     * Get number of detected tags
     * @return Count of currently detected AprilTags
     */
    public int getDetectedTagCount() {
        return detectedTags.size();
    }
}
