package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.auto.AuroraLocalizationManager;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

/**
 * Aurora AprilTag Localization Demo
 *
 * Demonstrates how to integrate the new AuroraAprilTagLocalizer with your Aurora system.
 * Shows real-time position tracking, confidence monitoring, and Aurora integration.
 */
@Autonomous(name="Aurora Localization Demo", group="Aurora")
public class AuroraLocalizationDemo extends LinearOpMode implements AuroraLocalizationManager.AuroraPositionUpdateListener {

    // Aurora components
    private AuroraLocalizationManager localizationManager;

    // Position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private AuroraLocalizationManager.PositionSource currentSource = AuroraLocalizationManager.PositionSource.UNKNOWN;

    // Performance monitoring
    private final ElapsedTime runtime = new ElapsedTime();
    private int positionUpdates = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Aurora Localization System...");
        telemetry.update();

        // Initialize Aurora localization with calibrated C290
        localizationManager = new AuroraLocalizationManager(
            hardwareMap,
            telemetry,
            FieldMap.Alliance.RED,  // Change to BLUE as needed
            "Webcam 1"              // Your webcam name in hardware config
        );

        // Set this OpMode as the Aurora listener
        localizationManager.setAuroraListener(this);

        telemetry.addData("Status", "Aurora Localization Ready");
        telemetry.addData("Camera", "Calibrated C290 with C920 intrinsics");
        telemetry.addData("Position Source", "Initializing...");
        telemetry.addData(">", "Touch START to begin");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();

            telemetry.addData("Status", "Aurora Localization Active");
            telemetry.update();

            // Main Aurora localization loop
            while (opModeIsActive()) {
                // Update localization system
                localizationManager.updateLocalization();

                // Display comprehensive status
                displayAuroraStatus();

                // Run at 20Hz for optimal performance
                sleep(50);
            }

            // Cleanup
            localizationManager.shutdown();
        }
    }

    /**
     * Aurora position update callback - this is where your Aurora system gets position data
     */
    @Override
    public void onPositionUpdated(double x, double y, double heading, AuroraLocalizationManager.PositionSource source) {
        // Update internal position tracking
        this.robotX = x;
        this.robotY = y;
        this.robotHeading = heading;
        this.currentSource = source;
        this.positionUpdates++;

        // Here you would feed this position data into your Aurora positioning system
        // Example: auroraPositioningManager.updatePosition(x, y, heading, source);
    }

    /**
     * Position source change callback
     */
    @Override
    public void onPositionSourceChanged(AuroraLocalizationManager.PositionSource oldSource,
                                      AuroraLocalizationManager.PositionSource newSource) {
        telemetry.addData("Position Source Changed", "%s → %s", oldSource, newSource);

        // Here you would notify your Aurora system about the source change
        // This helps Aurora make decisions about navigation confidence
    }

    /**
     * Display comprehensive Aurora localization status
     */
    private void displayAuroraStatus() {
        AuroraLocalizationManager.LocalizationStatus status = localizationManager.getStatus();

        telemetry.addData("=== AURORA LOCALIZATION ===", "");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Position Updates", positionUpdates);
        telemetry.addLine();

        // Current position
        telemetry.addData("=== ROBOT POSITION ===", "");
        telemetry.addData("X Position", "%.1f inches", robotX);
        telemetry.addData("Y Position", "%.1f inches", robotY);
        telemetry.addData("Heading", "%.1f degrees", robotHeading);
        telemetry.addData("Position Source", currentSource.toString());
        telemetry.addLine();

        // Vision system status
        telemetry.addData("=== VISION STATUS ===", "");
        telemetry.addData("Vision Active", status.hasVision ? "YES" : "NO");
        telemetry.addData("Vision Confidence", "%.2f", status.visionConfidence);

        if (status.visionStatus != null) {
            telemetry.addData("Vision Success Rate", "%.1f%%",
                status.visionStatus.getSuccessRate() * 100);
            telemetry.addData("Total Detections", status.visionStatus.totalDetections);
            telemetry.addData("Valid Detections", status.visionStatus.validDetections);
            telemetry.addData("Last Update", "%.0f ms ago", status.visionStatus.lastUpdateMs);
        }
        telemetry.addLine();

        // Field information
        telemetry.addData("=== FIELD INFO ===", "");
        String currentZone = localizationManager.getCurrentZone();
        telemetry.addData("Current Zone", currentZone);

        // Distance from field center
        double distanceFromCenter = Math.sqrt(robotX * robotX + robotY * robotY);
        telemetry.addData("Distance from Center", "%.1f inches", distanceFromCenter);
        telemetry.addLine();

        // Aurora integration status
        telemetry.addData("=== AURORA INTEGRATION ===", "");
        telemetry.addData("Position Quality", getPositionQuality());
        telemetry.addData("Navigation Ready", isNavigationReady() ? "YES" : "NO");
        telemetry.addData("Recommended Actions", getRecommendedActions());
        telemetry.addLine();

        // Controls
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("D-pad Up", "Resume vision streaming");
        telemetry.addData("D-pad Down", "Pause vision streaming");
        telemetry.addData("X Button", "Reset position to (0,0)");

        // Handle controls
        handleControls();

        telemetry.update();
    }

    /**
     * Get position quality assessment
     */
    private String getPositionQuality() {
        switch (currentSource) {
            case VISION_PRIMARY:
                return "EXCELLENT (High-confidence vision)";
            case VISION_ENCODER:
                return "GOOD (Vision + encoder fusion)";
            case ENCODER_ONLY:
                return "FAIR (Encoders only)";
            default:
                return "POOR (No reliable position)";
        }
    }

    /**
     * Check if Aurora navigation is ready
     */
    private boolean isNavigationReady() {
        return currentSource != AuroraLocalizationManager.PositionSource.UNKNOWN &&
               positionUpdates > 5; // Need a few updates to be confident
    }

    /**
     * Get recommended actions for Aurora system
     */
    private String getRecommendedActions() {
        switch (currentSource) {
            case VISION_PRIMARY:
                return "All systems go - full autonomous capability";
            case VISION_ENCODER:
                return "Good for navigation - monitor vision recovery";
            case ENCODER_ONLY:
                return "Limited precision - seek AprilTag visibility";
            default:
                return "Initialize position or find AprilTags";
        }
    }

    /**
     * Handle gamepad controls
     */
    private void handleControls() {
        if (gamepad1.dpad_up) {
            localizationManager.resumeVision();
            telemetry.addData("Vision", "RESUMED");
        } else if (gamepad1.dpad_down) {
            localizationManager.pauseVision();
            telemetry.addData("Vision", "PAUSED");
        }

        if (gamepad1.x) {
            localizationManager.setPosition(0, 0, 0);
            telemetry.addData("Position", "RESET to (0,0)");
        }
    }
}
