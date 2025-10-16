package org.firstinspires.ftc.teamcode.util.aurora.vision;

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

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Aurora AprilTag Detector - Simplified, high-performance AprilTag detection
 *
 * This replaces the complex AprilTagMultiTool with a streamlined implementation
 * that matches AuroraVisionAuto.java's proven approach:
 * - Direct VisionPortal usage without excessive abstraction
 * - Fast, non-blocking detection updates
 * - Reliable camera switching and exposure control
 * - Minimal overhead for real-time autonomous use
 */
public class AuroraAprilTagDetector {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private WebcamName primaryWebcam;
    private WebcamName secondaryWebcam;
    private boolean useWebcam;
    private String activeCameraName;
    private boolean isReady;

    /**
     * Initialize with single webcam
     */
    public AuroraAprilTagDetector(HardwareMap hardwareMap, boolean useWebcam, String cameraName) {
        this.useWebcam = useWebcam;
        this.activeCameraName = cameraName != null ? cameraName : "phone";
        this.isReady = false;

        initializeDetector(hardwareMap, cameraName, null);
    }

    /**
     * Initialize with dual webcams
     */
    public AuroraAprilTagDetector(HardwareMap hardwareMap, String webcam1Name, String webcam2Name) {
        this.useWebcam = true;
        this.activeCameraName = webcam1Name;
        this.isReady = false;

        initializeDetector(hardwareMap, webcam1Name, webcam2Name);
    }

    /**
     * Initialize the AprilTag detection system
     */
    private void initializeDetector(HardwareMap hardwareMap, String webcam1Name, String webcam2Name) {
        try {
            // Create AprilTag processor with optimized settings
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
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
                } catch (Exception e) {
                    // Fallback to phone camera
                    builder.setCamera(BuiltinCameraDirection.BACK);
                    useWebcam = false;
                    activeCameraName = "phone";
                }
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
                activeCameraName = "phone";
            }

            // Add processor and build
            builder.addProcessor(aprilTagProcessor);
            builder.enableLiveView(true);
            visionPortal = builder.build();

            isReady = true;

        } catch (Exception e) {
            isReady = false;
            throw new RuntimeException("Failed to initialize AprilTag detector: " + e.getMessage());
        }
    }

    /**
     * Set camera pose for field positioning
     */
    public void setCameraPose(Position position, YawPitchRollAngles orientation) {
        // Note: Camera pose must be set during processor creation
        // This method is provided for API compatibility but has limitations
        // in the current FTC SDK
    }

    /**
     * Start camera streaming
     */
    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    /**
     * Stop camera streaming
     */
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    /**
     * Switch to primary webcam
     */
    public void switchToWebcam1() {
        if (primaryWebcam != null && visionPortal != null) {
            visionPortal.setActiveCamera(primaryWebcam);
            activeCameraName = primaryWebcam.toString();
        }
    }

    /**
     * Switch to secondary webcam
     */
    public void switchToWebcam2() {
        if (secondaryWebcam != null && visionPortal != null) {
            visionPortal.setActiveCamera(secondaryWebcam);
            activeCameraName = secondaryWebcam.toString();
        }
    }

    /**
     * Get current AprilTag detections - FAST and NON-BLOCKING
     */
    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor != null) {
            return aprilTagProcessor.getDetections();
        }
        return java.util.Collections.emptyList();
    }

    /**
     * Set manual camera exposure and gain
     */
    public boolean setManualExposure(int exposureMS, int gain) {
        if (!isReady || visionPortal == null) return false;

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
            return false;
        }
    }

    /**
     * Get camera control limits for exposure tuning
     */
    public int getMinExposure() {
        if (visionPortal == null) return 1;
        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            return (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        } catch (Exception e) {
            return 1;
        }
    }

    public int getMaxExposure() {
        if (visionPortal == null) return 1000;
        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            return (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        } catch (Exception e) {
            return 1000;
        }
    }

    public int getMinGain() {
        if (visionPortal == null) return 0;
        try {
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            return gainControl.getMinGain();
        } catch (Exception e) {
            return 0;
        }
    }

    public int getMaxGain() {
        if (visionPortal == null) return 255;
        try {
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            return gainControl.getMaxGain();
        } catch (Exception e) {
            return 255;
        }
    }

    /**
     * Check if detector is ready
     */
    public boolean isReady() {
        return isReady && visionPortal != null;
    }

    /**
     * Get active camera name
     */
    public String getActiveCameraName() {
        return activeCameraName;
    }

    /**
     * Get camera state
     */
    public VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) {
            return visionPortal.getCameraState();
        }
        return VisionPortal.CameraState.CAMERA_DEVICE_CLOSED;
    }

    /**
     * Close vision portal and cleanup
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            isReady = false;
        }
    }
}
