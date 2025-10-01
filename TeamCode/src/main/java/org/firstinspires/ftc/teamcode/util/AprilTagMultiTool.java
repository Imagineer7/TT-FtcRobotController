package org.firstinspires.ftc.teamcode.util;

/*
AprilTagMultiTool Usage Instructions
====================================

This utility class provides a simple, unified interface for AprilTag detection, camera control, and telemetry in FTC OpModes.

Basic Usage:
------------
1. Create an instance in your OpMode:
    AprilTagMultiTool aprilTagUtil = new AprilTagMultiTool(hardwareMap, true, "Webcam 1", null);
    // For two webcams: new AprilTagMultiTool(hardwareMap, true, "Webcam 1", "Webcam 2");
    // For phone camera: new AprilTagMultiTool(hardwareMap, false, null, null);

2. Start streaming:
    aprilTagUtil.resumeStreaming();

3. In your main loop, add AprilTag telemetry:
    aprilTagUtil.addTelemetry(this); // 'this' is your LinearOpMode
    telemetry.update();

4. To switch cameras (if using two webcams):
    aprilTagUtil.switchToWebcam1();
    aprilTagUtil.switchToWebcam2();

5. To set camera pose (position and orientation):
    Position pos = new Position(DistanceUnit.INCH, x, y, z, 0);
    YawPitchRollAngles orient = new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0);
    aprilTagUtil.setCameraPose(pos, orient);

6. To manually set exposure and gain:
    int minExp = aprilTagUtil.getMinExposure();
    int maxExp = aprilTagUtil.getMaxExposure();
    int minGain = aprilTagUtil.getMinGain();
    int maxGain = aprilTagUtil.getMaxGain();
    aprilTagUtil.setManualExposure(desiredExposure, desiredGain);

7. To get AprilTag detections:
    List<AprilTagDetection> detections = aprilTagUtil.getDetections();

8. When finished, close the portal:
    aprilTagUtil.close();

Notes:
------
- You can use AprilTag detections for navigation, localization, or autonomous actions.
- Use telemetry to display tag info, or process detections for robot control.
- For advanced camera switching, use ClassFactory for switchable cameras.
- Always call close() at the end of your OpMode to release camera resources.

See the FTC SDK samples for more advanced AprilTag usage and camera configuration.
*/

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.concurrent.TimeUnit;

public class AprilTagMultiTool {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private WebcamName webcam1, webcam2;
    private boolean useWebcam;
    private HardwareMap hardwareMap;
    private Position cameraPosition = null;
    private YawPitchRollAngles cameraOrientation = null;

    public AprilTagMultiTool(HardwareMap hardwareMap, boolean useWebcam, String webcamName1, String webcamName2) {
        this.hardwareMap = hardwareMap;
        this.useWebcam = useWebcam;
        // Default camera pose: no translation, horizontal camera
        cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (useWebcam) {
            webcam1 = hardwareMap.get(WebcamName.class, webcamName1);
            builder.setCamera(webcam1);
            if (webcamName2 != null) {
                webcam2 = hardwareMap.get(WebcamName.class, webcamName2);
                // For switchable camera, use ClassFactory if needed
            }
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
    builder.addProcessor(aprilTag);
    // Keep preview running (auto-stop only)
    builder.setAutoStopLiveView(false);
    visionPortal = builder.build();
    }

    // Allow user to set camera pose
    public void setCameraPose(Position position, YawPitchRollAngles orientation) {
        this.cameraPosition = position;
        this.cameraOrientation = orientation;
        // Rebuild AprilTagProcessor with new pose
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        // Replace processor in vision portal
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    // Exposure/gain control
    public boolean setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return false;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return false;
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        return true;
    }

    public int getMinExposure() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        return (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
    }
    public int getMaxExposure() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        return (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
    }
    public int getMinGain() {
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        return gainControl.getMinGain();
    }
    public int getMaxGain() {
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        return gainControl.getMaxGain();
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void close() {
        visionPortal.close();
    }

    public void switchToWebcam1() {
        if (webcam1 != null) visionPortal.setActiveCamera(webcam1);
    }

    public void switchToWebcam2() {
        if (webcam2 != null) visionPortal.setActiveCamera(webcam2);
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public void addTelemetry(LinearOpMode opMode) {
        List<AprilTagDetection> detections = getDetections();
        opMode.telemetry.addData("# AprilTags Detected", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        opMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        opMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        opMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}