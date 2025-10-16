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

import java.util.ArrayList;
import java.util.List;

/**
 * Simple Aurora Vision Manager - Designed to work exactly like SimpleVisionAuto
 *
 * This is a streamlined vision system that works identically to AprilTagMultiTool
 * without any complex caching, clearing, or management layers that interfere with detection.
 *
 * Key principles:
 * - Direct access to AprilTag processor
 * - No caching or clearing of detection data
 * - Single initialization, continuous detection
 * - Identical behavior to SimpleVisionAuto's AprilTagMultiTool
 */
public class SimpleAuroraVisionManager {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private boolean isInitialized;
    private String cameraName;

    /**
     * Initialize with webcam (preferred) or fallback to phone camera
     */
    public SimpleAuroraVisionManager(HardwareMap hardwareMap, Telemetry telemetry, String webcamName) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.cameraName = webcamName;
        this.isInitialized = false;

        initializeVisionSystem();
    }

    /**
     * Initialize the vision system using the exact same approach as AprilTagMultiTool
     */
    private void initializeVisionSystem() {
        try {
            telemetry.addData("SimpleAuroraVision", "Initializing...");
            telemetry.update();

            // Create AprilTag processor with identical settings to AprilTagMultiTool
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
                    // Use same camera pose as SimpleVisionAuto (rear-facing camera)
                    .setCameraPose(
                        new Position(DistanceUnit.INCH, -8.5, 0.0, 10.0, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 180.0, -15.0, 0.0, 0)
                    )
                    .build();

            // Create vision portal builder - identical to AprilTagMultiTool
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set camera source with same fallback logic as AprilTagMultiTool
            if (cameraName != null) {
                try {
                    WebcamName webcam = hardwareMap.get(WebcamName.class, cameraName);
                    builder.setCamera(webcam);
                    telemetry.addData("SimpleAuroraVision", "Using webcam: " + cameraName);
                } catch (Exception e) {
                    // Fallback to phone camera if webcam fails
                    builder.setCamera(BuiltinCameraDirection.BACK);
                    telemetry.addData("SimpleAuroraVision", "Webcam failed, using phone camera");
                }
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
                telemetry.addData("SimpleAuroraVision", "Using phone camera");
            }

            // Add processor and enable live view - identical to AprilTagMultiTool
            builder.addProcessor(aprilTagProcessor);
            builder.enableLiveView(true);

            // Build the vision portal
            visionPortal = builder.build();

            isInitialized = true;
            telemetry.addData("SimpleAuroraVision", "Initialized successfully");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("SimpleAuroraVision ERROR", "Failed to initialize: " + e.getMessage());
            telemetry.update();
            isInitialized = false;
        }
    }

    /**
     * Start streaming - identical to AprilTagMultiTool.resumeStreaming()
     */
    public void startStreaming() {
        if (visionPortal != null && isInitialized) {
            visionPortal.resumeStreaming();
            telemetry.addData("SimpleAuroraVision", "Streaming started");
        }
    }

    /**
     * Stop streaming
     */
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
            telemetry.addData("SimpleAuroraVision", "Streaming stopped");
        }
    }

    /**
     * Get AprilTag detections - IDENTICAL to AprilTagMultiTool.getDetections()
     * This is the exact same method that SimpleVisionAuto uses successfully
     */
    public List<AprilTagDetection> getDetections() {
        if (!isInitialized || aprilTagProcessor == null) {
            return new ArrayList<>();
        }

        try {
            // This is the EXACT same call that SimpleVisionAuto makes
            return aprilTagProcessor.getDetections();
        } catch (Exception e) {
            telemetry.addData("SimpleAuroraVision", "Detection error: " + e.getMessage());
            return new ArrayList<>();
        }
    }

    /**
     * Check if vision system is ready
     */
    public boolean isReady() {
        return isInitialized && visionPortal != null && aprilTagProcessor != null;
    }

    /**
     * Get camera state for debugging
     */
    public String getCameraState() {
        if (visionPortal != null) {
            return visionPortal.getCameraState().toString();
        }
        return "UNKNOWN";
    }

    /**
     * Add telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== SIMPLE AURORA VISION ===", "");
        telemetry.addData("Initialized", isInitialized ? "YES" : "NO");
        telemetry.addData("Camera", cameraName != null ? cameraName : "phone");
        telemetry.addData("Camera State", getCameraState());

        if (isReady()) {
            List<AprilTagDetection> detections = getDetections();
            telemetry.addData("Tags Detected", detections.size());

            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose != null) {
                    telemetry.addData("Tag " + detection.id,
                        "Dist:%.1f Bear:%.1f° Yaw:%.1f°",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.yaw);
                }
            }
        } else {
            telemetry.addData("Status", "Vision system not ready");
        }
    }

    /**
     * Close vision portal
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            telemetry.addData("SimpleAuroraVision", "Vision portal closed");
        }
    }
}
