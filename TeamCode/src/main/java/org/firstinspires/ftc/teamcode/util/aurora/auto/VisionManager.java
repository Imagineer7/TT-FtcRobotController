package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * Aurora Vision Manager - Streamlined AprilTag detection and position tracking
 *
 * Rebuilt to match AuroraVisionAuto.java's proven approach:
 * - Direct AprilTag detection without complex wrappers
 * - Non-blocking vision updates
 * - Reliable robot position calculation from tags
 * - Real-time movement integration
 * - Minimal overhead for fast processing
 */
public class VisionManager {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Camera configuration
    private WebcamName primaryWebcam;
    private WebcamName secondaryWebcam;
    private boolean useWebcam;
    private String activeCameraName;

    // Vision data cache - matches AuroraVisionAuto format
    private Map<Integer, double[]> detectedTags; // ID -> [range, bearing, yaw, elevation]
    private boolean isStreaming;
    private boolean visionReady;

    // Camera pose for field positioning
    private Position cameraPosition;
    private YawPitchRollAngles cameraOrientation;

    /**
     * Initialize with single webcam (primary use case)
     */
    public VisionManager(HardwareMap hardwareMap, Telemetry telemetry, boolean useWebcam, String cameraName) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.useWebcam = useWebcam;
        this.activeCameraName = cameraName != null ? cameraName : "phone";

        // Initialize detection cache
        detectedTags = new HashMap<>();
        isStreaming = false;
        visionReady = false;

        // Set default camera pose (rear-facing like AuroraVisionAuto)
        setDefaultCameraPose();

        // Initialize vision system
        initializeVisionSystem(cameraName, null);
    }

    /**
     * Initialize with dual webcams
     */
    public VisionManager(HardwareMap hardwareMap, Telemetry telemetry, String webcam1Name, String webcam2Name) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.useWebcam = true;
        this.activeCameraName = webcam1Name;

        // Initialize detection cache
        detectedTags = new HashMap<>();
        isStreaming = false;
        visionReady = false;

        // Set default camera pose
        setDefaultCameraPose();

        // Initialize vision system with dual cameras
        initializeVisionSystem(webcam1Name, webcam2Name);
    }

    /**
     * Set default camera pose to match AuroraVisionAuto configuration
     */
    private void setDefaultCameraPose() {
        // Camera mounted on back of robot facing backwards (like AuroraVisionAuto)
        double cameraX = -8.5;  // inches behind robot center
        double cameraY = 0.0;   // centered
        double cameraZ = 10.0;  // inches up
        double cameraYaw = 180.0;   // facing backwards
        double cameraPitch = -15.0; // angled down
        double cameraRoll = 0.0;

        cameraPosition = new Position(DistanceUnit.INCH, cameraX, cameraY, cameraZ, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, cameraYaw, cameraPitch, cameraRoll, 0);
    }

    /**
     * Initialize the AprilTag vision system
     */
    private void initializeVisionSystem(String webcam1Name, String webcam2Name) {
        try {
            // Create AprilTag processor with optimized settings for real-time detection
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .build();

            // Create vision portal builder
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Configure camera source
            if (useWebcam && webcam1Name != null) {
                try {
                    primaryWebcam = hardwareMap.get(WebcamName.class, webcam1Name);
                    builder.setCamera(primaryWebcam);

                    if (webcam2Name != null) {
                        secondaryWebcam = hardwareMap.get(WebcamName.class, webcam2Name);
                    }

                    telemetry.addData("VisionManager", "Using webcam: " + webcam1Name);
                } catch (Exception e) {
                    // Fallback to phone camera
                    builder.setCamera(BuiltinCameraDirection.BACK);
                    useWebcam = false;
                    activeCameraName = "phone";
                    telemetry.addData("VisionManager", "Webcam failed, using phone camera");
                }
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
                activeCameraName = "phone";
                telemetry.addData("VisionManager", "Using phone camera");
            }

            // Add processor and build
            builder.addProcessor(aprilTagProcessor);
            builder.enableLiveView(true);
            visionPortal = builder.build();

            visionReady = true;
            telemetry.addData("VisionManager", "Initialized successfully");

        } catch (Exception e) {
            telemetry.addData("VisionManager Error", "Initialization failed: " + e.getMessage());
            visionReady = false;
        }
    }

    /**
     * Set camera pose for field positioning
     */
    public void setCameraPose(double x, double y, double z, double yaw, double pitch, double roll) {
        cameraPosition = new Position(DistanceUnit.INCH, x, y, z, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0);

        // Note: Camera pose cannot be changed after processor is built
        // This is stored for future rebuilds if needed
        telemetry.addData("VisionManager", "Camera pose updated (%.1f, %.1f, %.1f)", x, y, z);
    }

    /**
     * Start camera streaming
     */
    public void startStreaming() {
        if (visionPortal != null && visionReady) {
            visionPortal.resumeStreaming();
            isStreaming = true;
            telemetry.addData("VisionManager", "Streaming started on " + activeCameraName);
        }
    }

    /**
     * Stop camera streaming
     */
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
            isStreaming = false;
            telemetry.addData("VisionManager", "Streaming stopped");
        }
    }

    /**
     * Switch to primary webcam (if available)
     */
    public void switchToWebcam1() {
        if (primaryWebcam != null && visionPortal != null) {
            visionPortal.setActiveCamera(primaryWebcam);
            activeCameraName = primaryWebcam.toString();
            telemetry.addData("VisionManager", "Switched to primary webcam");
        }
    }

    /**
     * Switch to secondary webcam (if available)
     */
    public void switchToWebcam2() {
        if (secondaryWebcam != null && visionPortal != null) {
            visionPortal.setActiveCamera(secondaryWebcam);
            activeCameraName = secondaryWebcam.toString();
            telemetry.addData("VisionManager", "Switched to secondary webcam");
        }
    }

    /**
     * Update vision detection data - NON-BLOCKING like AuroraVisionAuto
     * Call this frequently in your main loop
     */
    public void updateVision() {
        if (!isVisionReady()) return;

        // Clear previous detections
        detectedTags.clear();

        try {
            // Get current detections - this is fast and non-blocking
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            // Process each detection
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose != null) {
                    // Store in same format as AuroraVisionAuto: [range, bearing, yaw, elevation]
                    double[] tagData = new double[4];
                    tagData[0] = detection.ftcPose.range;     // Distance to tag
                    tagData[1] = detection.ftcPose.bearing;   // Bearing angle
                    tagData[2] = detection.ftcPose.yaw;       // Yaw angle
                    tagData[3] = detection.ftcPose.elevation; // Elevation angle

                    detectedTags.put(detection.id, tagData);
                }
            }

        } catch (Exception e) {
            telemetry.addData("VisionManager Error", "Update failed: " + e.getMessage());
        }
    }

    /**
     * Check if specific AprilTag is detected
     */
    public boolean isTagDetected(int tagId) {
        return detectedTags.containsKey(tagId);
    }

    /**
     * Get robot position from detected AprilTags - EXACT same algorithm as AuroraVisionAuto
     */
    public double[] getRobotPositionFromTags(Map<Integer, double[]> knownTagPositions) {
        // Find the closest detected tag
        int closestId = getClosestTagId();
        if (closestId == -1 || !knownTagPositions.containsKey(closestId)) {
            return null;
        }

        // Get detection data: [range, bearing, yaw, elevation]
        double[] detectionData = detectedTags.get(closestId);
        double[] knownTagPos = knownTagPositions.get(closestId);

        if (detectionData == null || knownTagPos == null) return null;

        // Extract values using AuroraVisionAuto's exact approach
        double range = detectionData[0];    // Distance to tag
        double bearing = detectionData[1];  // Bearing angle
        double yaw = detectionData[2];      // Yaw angle

        // Get known tag position
        double tagX = knownTagPos[0];
        double tagY = knownTagPos[1];

        // Calculate robot position using AuroraVisionAuto's exact method (rear-facing camera)
        double robotAngleFromTag = bearing + 180.0;
        double robotX = tagX + range * Math.cos(Math.toRadians(robotAngleFromTag));
        double robotY = tagY + range * Math.sin(Math.toRadians(robotAngleFromTag));
        double robotHeading = normalizeAngle(yaw + 180.0);

        return new double[]{robotX, robotY, robotHeading};
    }

    /**
     * Get ID of closest detected AprilTag
     */
    public int getClosestTagId() {
        double minDistance = Double.MAX_VALUE;
        int closestId = -1;

        for (Map.Entry<Integer, double[]> entry : detectedTags.entrySet()) {
            double[] data = entry.getValue();
            if (data.length >= 1 && data[0] < minDistance) {
                minDistance = data[0];
                closestId = entry.getKey();
            }
        }

        return closestId;
    }

    /**
     * Get distance to closest detected AprilTag
     */
    public double getClosestTagDistance() {
        int closestId = getClosestTagId();
        if (closestId != -1) {
            double[] data = detectedTags.get(closestId);
            return data != null ? data[0] : -1.0;
        }
        return -1.0;
    }

    /**
     * Get angle to detected AprilTag
     */
    public double getTagAngle(int tagId) {
        double[] data = detectedTags.get(tagId);
        if (data != null && data.length >= 3) {
            return data[2]; // Yaw angle
        }
        return 0.0;
    }

    /**
     * Get tag position data
     */
    public double[] getTagPosition(int tagId) {
        double[] data = detectedTags.get(tagId);
        if (data != null && data.length >= 4) {
            return new double[]{data[0], data[1], data[2]}; // range, bearing, yaw
        }
        return null;
    }

    /**
     * Get number of detected tags
     */
    public int getDetectedTagCount() {
        return detectedTags.size();
    }

    /**
     * Check if vision system is ready and streaming
     */
    public boolean isVisionReady() {
        return visionReady && visionPortal != null && isStreaming;
    }

    /**
     * Get active camera name
     */
    public String getActiveCamera() {
        return activeCameraName;
    }

    /**
     * Set manual camera exposure and gain for better detection
     */
    public boolean setManualExposure(int exposureMS, int gain) {
        if (!isVisionReady()) return false;

        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

            return true;
        } catch (Exception e) {
            telemetry.addData("VisionManager", "Exposure control failed: " + e.getMessage());
            return false;
        }
    }

    /**
     * Add telemetry data - matches AuroraVisionAuto format
     */
    public void addTelemetry() {
        telemetry.addData("=== VISION DATA ===", "");
        telemetry.addData("Active Camera", activeCameraName);
        telemetry.addData("Streaming", isStreaming ? "YES" : "NO");
        telemetry.addData("Vision Ready", isVisionReady() ? "YES" : "NO");
        telemetry.addData("Tags Detected", detectedTags.size());

        if (detectedTags.size() == 0) {
            telemetry.addData("Status", "SEARCHING FOR APRILTAGS...");
            telemetry.addData("Tip", "Ensure tags are visible and well-lit");
        } else {
            for (Map.Entry<Integer, double[]> entry : detectedTags.entrySet()) {
                double[] data = entry.getValue();
                telemetry.addData("Tag " + entry.getKey(),
                    "Dist:%.1f Bear:%.1f° Yaw:%.1f°", data[0], data[1], data[2]);
            }
        }
    }

    /**
     * Add detailed debugging telemetry
     */
    public void addDetailedTelemetry() {
        addTelemetry();
        telemetry.addData("=== DETAILED DEBUG ===", "");
        telemetry.addData("Camera Position", "X:%.1f Y:%.1f Z:%.1f",
                          cameraPosition.x, cameraPosition.y, cameraPosition.z);
        telemetry.addData("Camera Orientation", "Yaw:%.1f Pitch:%.1f Roll:%.1f",
                          cameraOrientation.getYaw(AngleUnit.DEGREES),
                          cameraOrientation.getPitch(AngleUnit.DEGREES),
                          cameraOrientation.getRoll(AngleUnit.DEGREES));
    }

    /**
     * Normalize angle to -180 to +180 degrees - same as AuroraVisionAuto
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Close vision portal and cleanup resources
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionReady = false;
            isStreaming = false;
            telemetry.addData("VisionManager", "Vision portal closed");
        }
    }

    /**
     * Get direct AprilTag detections - works exactly like SimpleVisionAuto
     * This bypasses all caching and management layers for immediate detection access
     */
    public List<AprilTagDetection> getDirectDetections() {
        if (!isVisionReady() || aprilTagProcessor == null) {
            return new ArrayList<>();
        }

        try {
            // Get detections directly from the AprilTag processor
            // This is the EXACT same call that SimpleVisionAuto uses: aprilTagTool.getDetections()
            return aprilTagProcessor.getDetections();
        } catch (Exception e) {
            telemetry.addData("Direct Detection Error", e.getMessage());
            return new ArrayList<>();
        }
    }
}
