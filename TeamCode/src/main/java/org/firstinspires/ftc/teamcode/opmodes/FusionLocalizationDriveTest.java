package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.localization.FusionLocalizer;
import org.firstinspires.ftc.teamcode.util.aurora.localization.LocalizationConfig;
import org.firstinspires.ftc.teamcode.util.aurora.localization.PredefinedPoses;
import org.firstinspires.ftc.teamcode.util.aurora.localization.RobotPose2D;

/**
 * Comprehensive Drive Test for Fusion Localization System
 *
 * This OpMode tests the fusion localization system during actual robot movement.
 * Drive the robot around the field and observe:
 * - Relative pose (smooth odometry) - GREEN on field
 * - Absolute pose (vision-corrected) - BLUE on field
 * - Vision correction activity
 * - Pose divergence over time
 * - Real-time field visualization via FTC Dashboard Panels
 *
 * The test demonstrates:
 * 1. Smooth drivetrain control using relative pose (no jumps)
 * 2. Vision corrections applied to absolute pose
 * 3. Gradual drift correction without snapping
 * 4. EKF fusion behavior with MegaTag2
 * 5. Visual feedback on FTC Dashboard field display
 *
 * Field Visualization:
 * ═══════════════════════════════════════════════════════════════════
 * The robot is drawn on the FTC Dashboard Panels field with:
 * - GREEN robot: RELATIVE pose (input) - Smooth odometry-based
 * - BLUE robot: ABSOLUTE pose (output) - Vision-corrected
 * - ORANGE robot: RAW LIMELIGHT - What vision sees right now (unfiltered)
 * - Light GREEN trail: Relative pose history
 * - Light BLUE trail: Absolute pose history
 * - ORANGE line: Connects relative and absolute poses (shows divergence)
 *
 * Open FTC Dashboard (http://192.168.43.1:8080/dash) to see live field view!
 * ═══════════════════════════════════════════════════════════════════
 *
 * Controls:
 * ═══════════════════════════════════════════════════════════════════
 * INIT PHASE:
 * - D-Pad Up/Down: Select start position
 * - START: Begin test
 *
 * RUNNING PHASE:
 * Gamepad 1:
 * - Left Stick: Forward/backward, strafe left/right
 * - Right Stick X: Rotate
 * - Left Bumper: Precision mode (25% speed)
 * - Right Bumper: Toggle field-centric mode
 * - A: Reset to origin (0, 0, 0°)
 * - B: Reset to selected start position
 * - X: Cycle telemetry display (Standard → Debug → Statistics → Comparison)
 * - Y: Toggle pose trace recording
 * - D-Pad Left: 🔧 FORCE MANUAL VISION UPDATE (use Limelight pose immediately)
 * - Back: Emergency stop
 * ═══════════════════════════════════════════════════════════════════
 *
 * Test Procedure:
 * 1. Place robot at known start position
 * 2. Initialize and select correct start position
 * 3. Drive around field, passing by AprilTags
 * 4. Observe vision corrections happening
 * 5. Watch pose divergence accumulate and get corrected
 * 6. Compare relative vs absolute pose accuracy
 *
 * What to Look For:
 * - Relative pose should be smooth, no jumps
 * - Absolute pose may jump slightly during vision corrections
 * - Pose divergence should stay small if vision is working
 * - Vision acceptance rate should be > 70% when tags visible
 * - Odometry frequency should be 50-100 Hz
 * - Vision updates should happen every 100-500ms when tags visible
 *
 * Troubleshooting:
 * - Low vision acceptance rate → Check Limelight pipeline, lighting
 * - Large pose divergence → Vision not seeing tags, check field setup
 * - Jerky drivetrain → Robot-centric mode may help, or reduce speed
 * - No vision updates → Check Limelight hardware connection
 *
 * @author AURORA Team
 * @version 1.0
 * @see FusionLocalizer
 * @see FusionLocalizationTestOpMode (stationary test)
 */
@TeleOp(name = "Test: Fusion Localization (Drive)", group = "Test")
public class FusionLocalizationDriveTest extends LinearOpMode {

    // Hardware and systems
    private AuroraHardwareConfig hardware;
    private FusionLocalizer localizer;

    // Drive motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Panels field drawing
    private FieldManager panelsField;
    private static final double ROBOT_RADIUS = 9.0; // inches

    // Drawing styles for visualization
    private static final Style RELATIVE_POSE_STYLE = new Style("", "#4CAF50", 1.0); // Green - input pose
    private static final Style ABSOLUTE_POSE_STYLE = new Style("", "#3F51B5", 1.0); // Blue - output pose
    private static final Style LIMELIGHT_RAW_STYLE = new Style("", "#FF9800", 0.8); // Orange - raw Limelight
    private static final Style RELATIVE_TRACE_STYLE = new Style("", "#81C784", 0.5); // Light green - relative history
    private static final Style ABSOLUTE_TRACE_STYLE = new Style("", "#7986CB", 0.5); // Light blue - absolute history

    // Configuration
    private int selectedStartIndex = 0;
    private boolean fieldCentricMode = false;

    // Telemetry display mode
    private enum DisplayMode {
        STANDARD,    // Normal view: poses, vision status
        DEBUG,       // Debug view: EKF internals, covariance
        STATISTICS,  // Statistics view: acceptance rate, counts
        COMPARISON   // Side-by-side pose comparison
    }
    private DisplayMode displayMode = DisplayMode.STANDARD;

    // Pose trace recording (for visualization)
    private boolean recordTrace = false;
    private java.util.List<RobotPose2D> relativePoseTrace = new java.util.ArrayList<>();
    private java.util.List<RobotPose2D> absolutePoseTrace = new java.util.ArrayList<>();
    private static final int MAX_TRACE_SIZE = 100;

    // Button edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // Drive control
    private boolean precisionMode = false;

    // Manual vision update
    private boolean forceNextVisionUpdate = false;
    private long lastManualUpdateTime = 0;

    // Performance tracking
    private ElapsedTime loopTimer = new ElapsedTime();
    private double maxLoopTime = 0;
    private int loopCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // ═══════════════════════════════════════════════════════════════════════
        // INITIALIZATION PHASE
        // ═══════════════════════════════════════════════════════════════════════

        telemetry.addLine("🤖 Fusion Localization Drive Test");
        telemetry.addLine("═══════════════════════════════════════════════");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();

        // Get drive motors
        leftFront = hardware.getFrontLeftMotor();
        rightFront = hardware.getFrontRightMotor();
        leftBack = hardware.getBackLeftMotor();
        rightBack = hardware.getBackRightMotor();

        // Verify drive system
        if (!hardware.isDriveSystemInitialized()) {
            telemetry.addLine();
            telemetry.addLine("❌ ERROR: Drive system not initialized!");
            telemetry.addLine("Check hardware configuration.");
            telemetry.update();
            while (opModeIsActive()) {
                idle();
            }
            return;
        }

        telemetry.addLine("✅ Hardware initialized");
        telemetry.update();
        sleep(500);

        // Initialize Panels field for visualization
        try {
            panelsField = PanelsField.INSTANCE.getField();
            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            telemetry.addLine("✅ Panels field initialized");
        } catch (Exception e) {
            telemetry.addLine("⚠️ Panels field not available (visualization disabled)");
            panelsField = null;
        }
        telemetry.update();
        sleep(500);

        // ═══════════════════════════════════════════════════════════════════════
        // START POSITION SELECTION
        // ═══════════════════════════════════════════════════════════════════════

        telemetry.clear();
        telemetry.addLine("═══ START POSITION SELECTION ═══");
        telemetry.addLine();
        telemetry.addLine("Use D-Pad Up/Down to select your starting position");
        telemetry.addLine("Position must match robot's actual location!");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            // Handle D-Pad input
            if (gamepad1.dpad_up && !lastDpadUp) {
                selectedStartIndex = (selectedStartIndex - 1 + 7) % 7;  // 7 positions now
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                selectedStartIndex = (selectedStartIndex + 1) % 7;  // 7 positions now
            }

            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // Show selected position (or field center option)
            if (selectedStartIndex == 6) {
                telemetry.addData("► Selected", "FIELD CENTER (0, 0)");
                telemetry.addData("  Position", "(0.0, 0.0) in");
                telemetry.addData("  Heading", "0.0°");
                telemetry.addData("  Note", "Custom position for testing");
            } else {
                PredefinedPoses.StartPosition selected =
                    PredefinedPoses.selectPosition(selectedStartIndex);
                RobotPose2D startPose = PredefinedPoses.getPose(selected);

                telemetry.addData("► Selected", PredefinedPoses.getName(selected));
                telemetry.addData("  Position", "(%.1f, %.1f) in",
                    startPose.getX(DistanceUnit.INCH),
                    startPose.getY(DistanceUnit.INCH));
                telemetry.addData("  Heading", "%.1f°",
                    startPose.getHeading(AngleUnit.DEGREES));
            }
            telemetry.update();

            idle();
        }

        // ═══════════════════════════════════════════════════════════════════════
        // CREATE LOCALIZER
        // ═══════════════════════════════════════════════════════════════════════

        telemetry.clear();
        telemetry.addLine("🤖 Creating Fusion Localizer...");

        // Determine start position
        RobotPose2D startPose;
        if (selectedStartIndex == 6) {
            // Field center option
            startPose = new RobotPose2D(0, 0, 0);
            telemetry.addData("Start Position", "FIELD CENTER");
        } else {
            PredefinedPoses.StartPosition startPosition =
                PredefinedPoses.selectPosition(selectedStartIndex);
            startPose = PredefinedPoses.getPose(startPosition);
            telemetry.addData("Start Position", PredefinedPoses.getName(startPosition));
        }

        telemetry.addData("MegaTag Mode", "MegaTag2 (Multi-tag)");
        telemetry.update();

        // Create config with RELAXED thresholds for testing
        LocalizationConfig config = new LocalizationConfig();

        // RELAXED THRESHOLDS to help with initial convergence
        config.mahalanobisThreshold = 10.0;  // Very relaxed (was 3.0)
        config.mahalanobisThresholdInitial = 15.0;  // Very relaxed for first update (was 5.0)
        config.maxInnovationMagnitude = 3000.0;  // Allow 3 meter jumps (was 1000.0 = 1m)

        // RELAXED measurement noise (trust vision more)
        config.measurementNoiseX = 100.0;  // Was 50.0
        config.measurementNoiseY = 100.0;  // Was 50.0
        config.measurementNoiseHeading = 0.2;  // Was 0.1

        telemetry.addLine();
        telemetry.addLine("⚙️ Using RELAXED thresholds:");
        telemetry.addData("Mahalanobis", "%.1f (normal) / %.1f (initial)",
            config.mahalanobisThreshold, config.mahalanobisThresholdInitial);
        telemetry.addData("Max Innovation", "%.0f mm", config.maxInnovationMagnitude);
        telemetry.update();
        sleep(1000);

        localizer = new FusionLocalizer(hardware, startPose, config);

        // Verify initialization
        if (!localizer.isInitialized()) {
            telemetry.addLine();
            telemetry.addLine("❌ ERROR: Localizer not initialized!");
            telemetry.update();
            while (opModeIsActive()) {
                idle();
            }
            return;
        }

        telemetry.addLine("✅ Fusion Localizer Ready");
        telemetry.addLine();
        telemetry.addLine("═══ CONTROLS ═══");
        telemetry.addLine("Left Stick: Drive");
        telemetry.addLine("Right Stick X: Rotate");
        telemetry.addLine("Left Bumper: Precision mode");
        telemetry.addLine("Right Bumper: Toggle field-centric");
        telemetry.addLine("X: Cycle display");
        telemetry.addLine("Y: Toggle trace recording");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        // ═══════════════════════════════════════════════════════════════════════
        // MAIN LOOP
        // ═══════════════════════════════════════════════════════════════════════

        while (opModeIsActive()) {
            double loopStart = loopTimer.milliseconds();

            // Manual vision update: Force accept next Limelight reading
            if (forceNextVisionUpdate) {
                performManualVisionUpdate();
                forceNextVisionUpdate = false;
            }

            // CRITICAL: Update localizer (predict + optional correct)
            localizer.update();

            // Get current poses
            RobotPose2D relativePose = localizer.getRelativePose();
            RobotPose2D absolutePose = localizer.getAbsolutePose();

            // Record trace if enabled
            if (recordTrace) {
                recordPoseTrace(relativePose, absolutePose);
            }

            // Handle button inputs
            handleButtons(relativePose);

            // Drive robot using RELATIVE pose (smooth, no jumps)
            driveRobot(relativePose);

            // Draw robot poses on field (if Panels available)
            drawFieldVisualization(relativePose, absolutePose);

            // Display telemetry
            displayTelemetry(relativePose, absolutePose);

            // Track loop performance
            double loopTime = loopTimer.milliseconds() - loopStart;
            maxLoopTime = Math.max(maxLoopTime, loopTime);
            loopCount++;
        }

        // ═══════════════════════════════════════════════════════════════════════
        // CLEANUP
        // ═══════════════════════════════════════════════════════════════════════

        // Stop motors
        stopDrive();

        // Final statistics
        telemetry.clear();
        telemetry.addLine("═══ TEST COMPLETE ═══");
        telemetry.addLine();
        telemetry.addData("Total Loops", loopCount);
        telemetry.addData("Max Loop Time", "%.1f ms", maxLoopTime);
        telemetry.addData("Vision Accepted", localizer.getVisionAcceptCount());
        telemetry.addData("Vision Rejected", localizer.getVisionRejectCount());
        telemetry.addData("Acceptance Rate", "%.1f%%",
            localizer.getVisionAcceptanceRate() * 100);
        if (recordTrace) {
            telemetry.addData("Trace Points", relativePoseTrace.size());
        }
        telemetry.update();
    }

    /**
     * Handle button inputs
     */
    private void handleButtons(RobotPose2D currentPose) {
        // A: Reset to origin
        if (gamepad1.a && !lastA) {
            localizer.reset(new RobotPose2D(0, 0, 0));
            relativePoseTrace.clear();
            absolutePoseTrace.clear();
        }

        // B: Reset to start position
        if (gamepad1.b && !lastB) {
            // Reset to the originally selected start position
            if (selectedStartIndex == 6) {
                localizer.reset(new RobotPose2D(0, 0, 0));
            } else {
                PredefinedPoses.StartPosition position =
                    PredefinedPoses.selectPosition(selectedStartIndex);
                localizer.reset(position);
            }
            relativePoseTrace.clear();
            absolutePoseTrace.clear();
        }

        // X: Cycle display mode
        if (gamepad1.x && !lastX) {
            DisplayMode[] modes = DisplayMode.values();
            int currentIndex = displayMode.ordinal();
            displayMode = modes[(currentIndex + 1) % modes.length];
        }

        // Y: Toggle trace recording
        if (gamepad1.y && !lastY) {
            recordTrace = !recordTrace;
            if (recordTrace) {
                relativePoseTrace.clear();
                absolutePoseTrace.clear();
            }
        }

        // D-Pad Left: Force next vision update (manual override)
        if (gamepad1.dpad_left && !lastDpadLeft) {
            forceNextVisionUpdate = true;
            lastManualUpdateTime = System.currentTimeMillis();
        }

        // Left Bumper: Toggle precision mode
        if (gamepad1.left_bumper && !lastLeftBumper) {
            precisionMode = !precisionMode;
        }

        // Right Bumper: Toggle field-centric mode
        if (gamepad1.right_bumper && !lastRightBumper) {
            fieldCentricMode = !fieldCentricMode;
        }

        // Back: Emergency stop
        if (gamepad1.back) {
            requestOpModeStop();
        }

        // Update last states
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadLeft = gamepad1.dpad_left;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
    }

    /**
     * Drive robot using relative pose (smooth, no jumps)
     */
    private void driveRobot(RobotPose2D relativePose) {
        // Get gamepad inputs
        double axial = -gamepad1.left_stick_y;  // Forward/backward (negated for correct direction)
        double lateral = gamepad1.left_stick_x;  // Strafe left/right
        double yaw = gamepad1.right_stick_x;     // Rotate

        // Apply precision mode scaling
        double speedScale = precisionMode ? 0.25 : 1.0;
        axial *= speedScale;
        lateral *= speedScale;
        yaw *= speedScale * 0.8;  // Slightly slower rotation

        // Apply field-centric transformation if enabled
        if (fieldCentricMode) {
            double heading = relativePose.heading;
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);

            double fieldAxial = axial * cos - lateral * sin;
            double fieldLateral = axial * sin + lateral * cos;

            axial = fieldAxial;
            lateral = fieldLateral;
        }

        // Calculate motor powers (mecanum drive kinematics)
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize powers
        double maxPower = Math.max(
            Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
            Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        );
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    /**
     * Stop all drive motors
     */
    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * Record pose trace for visualization
     */
    private void recordPoseTrace(RobotPose2D relativePose, RobotPose2D absolutePose) {
        relativePoseTrace.add(relativePose.copy());
        absolutePoseTrace.add(absolutePose.copy());

        // Limit trace size
        while (relativePoseTrace.size() > MAX_TRACE_SIZE) {
            relativePoseTrace.remove(0);
            absolutePoseTrace.remove(0);
        }
    }

    /**
     * Display telemetry based on current mode
     */
    private void displayTelemetry(RobotPose2D relativePose, RobotPose2D absolutePose) {
        telemetry.clear();

        switch (displayMode) {
            case STANDARD:
                displayStandardTelemetry(relativePose, absolutePose);
                break;
            case DEBUG:
                localizer.addDebugTelemetry(telemetry);
                break;
            case STATISTICS:
                displayStatistics(relativePose, absolutePose);
                break;
            case COMPARISON:
                displayComparison(relativePose, absolutePose);
                break;
        }

        // Always show controls at bottom
        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════════════════════");
        telemetry.addData("Drive Mode", fieldCentricMode ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Speed", precisionMode ? "Precision (25%)" : "Normal (100%)");
        telemetry.addData("Display", displayMode.toString());
        if (recordTrace) {
            telemetry.addData("Trace", "Recording (%d points)", relativePoseTrace.size());
        }
        telemetry.addLine();
        telemetry.addData("🔧 D-Pad Left", "Force vision update");

        telemetry.update();
    }

    /**
     * Display standard telemetry
     */
    private void displayStandardTelemetry(RobotPose2D relativePose, RobotPose2D absolutePose) {
        telemetry.addLine("═══ FUSION LOCALIZATION (DRIVE TEST) ═══");
        telemetry.addLine();

        // Relative pose (used for driving)
        telemetry.addLine("--- RELATIVE POSE (Drivetrain) ---");
        telemetry.addData("Position", "(%.1f, %.1f) in",
            relativePose.getX(DistanceUnit.INCH),
            relativePose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", "%.1f°",
            relativePose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", "%.1f in/s",
            localizer.getVelocityMagnitude(DistanceUnit.INCH));
        telemetry.addLine();

        // Absolute pose (vision-corrected)
        telemetry.addLine("--- ABSOLUTE POSE (Vision-Corrected) ---");
        telemetry.addData("Position", "(%.1f, %.1f) in",
            absolutePose.getX(DistanceUnit.INCH),
            absolutePose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", "%.1f°",
            absolutePose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Uncertainty", "%.1f in",
            absolutePose.getPositionUncertainty() / 25.4);
        telemetry.addLine();

        // Vision status
        telemetry.addLine("--- VISION STATUS ---");
        if (localizer.isVisionActive()) {
            long age = localizer.getTimeSinceLastVisionUpdate();
            telemetry.addData("Status", "✅ ACTIVE (%d ms ago)", age);
        } else {
            telemetry.addData("Status", "❌ NO TAGS VISIBLE");
        }
        telemetry.addData("Acceptance", "%.0f%% (%d/%d)",
            localizer.getVisionAcceptanceRate() * 100,
            localizer.getVisionAcceptCount(),
            localizer.getVisionAcceptCount() + localizer.getVisionRejectCount());

        // Raw Limelight reading (ORANGE on field) - DIAGNOSTIC MODE
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D limelightRaw = getLimelightPose();
        if (limelightRaw != null) {
            // Raw values from Limelight
            double rawXmeters = limelightRaw.getPosition().x;
            double rawYmeters = limelightRaw.getPosition().y;
            double rawXinches = rawXmeters * 39.37;  // meters to inches
            double rawYinches = rawYmeters * 39.37;
            double rawYawDeg = limelightRaw.getOrientation().getYaw(
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

            // Current odometry heading for comparison
            double odoHeadingDeg = relativePose.getHeading(AngleUnit.DEGREES);
            double headingDiff = rawYawDeg - odoHeadingDeg;
            while (headingDiff > 180) headingDiff -= 360;
            while (headingDiff < -180) headingDiff += 360;

            telemetry.addLine("🟠 Raw Limelight (DIAGNOSTIC - No Transform):");
            telemetry.addData("  LL Position", "(%.1f, %.1f) @ %.1f°",
                rawXinches, rawYinches, rawYawDeg);
            telemetry.addData("  Odo Heading", "%.1f°", odoHeadingDeg);
            telemetry.addData("  Heading Δ", "%.1f° (LL - Odo)", headingDiff);
            telemetry.addLine();
            telemetry.addLine("DIAGNOSIS: Compare ORANGE robot to GREEN/BLUE");
            telemetry.addLine("  - If ~90° rotated: IMU/AprilTag coord mismatch");
            telemetry.addLine("  - Check which axis/direction is wrong");
        } else {
            telemetry.addData("🟠 Raw Limelight", "No data");
        }

        // Show coordinates for debugging (both use center as origin)
        telemetry.addLine();
        telemetry.addLine("--- DEBUG INFO ---");
        telemetry.addData("Coordinates", "(%.1f, %.1f) in @ %.1f°",
            relativePose.getX(DistanceUnit.INCH),
            relativePose.getY(DistanceUnit.INCH),
            relativePose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Panels Drawing", "Check Dashboard!");
    }

    /**
     * Display statistics
     */
    private void displayStatistics(RobotPose2D relativePose, RobotPose2D absolutePose) {
        telemetry.addLine("═══ STATISTICS ═══");
        telemetry.addLine();

        // Performance
        telemetry.addLine("--- PERFORMANCE ---");
        telemetry.addData("Loop Count", loopCount);
        telemetry.addData("Max Loop Time", "%.1f ms", maxLoopTime);
        if (loopCount > 0) {
            double avgHz = 1000.0 / maxLoopTime;
            telemetry.addData("Est. Loop Rate", "%.1f Hz", avgHz);
        }
        telemetry.addLine();

        // Vision statistics
        telemetry.addLine("--- VISION MEASUREMENTS ---");
        telemetry.addData("Total Accepted", localizer.getVisionAcceptCount());
        telemetry.addData("Total Rejected", localizer.getVisionRejectCount());
        telemetry.addData("Acceptance Rate", "%.1f%%",
            localizer.getVisionAcceptanceRate() * 100);
        long timeSinceVision = localizer.getTimeSinceLastVisionUpdate();
        if (timeSinceVision >= 0) {
            telemetry.addData("Last Update", "%d ms ago", timeSinceVision);
        } else {
            telemetry.addData("Last Update", "Never");
        }
        telemetry.addLine();

        // Pose divergence
        double dx = absolutePose.x - relativePose.x;
        double dy = absolutePose.y - relativePose.y;
        double distance = Math.sqrt(dx*dx + dy*dy);
        double dh = Math.toDegrees(RobotPose2D.angleWrap(absolutePose.heading - relativePose.heading));

        telemetry.addLine("--- POSE DIVERGENCE ---");
        telemetry.addData("Position Diff", "%.1f in", distance / 25.4);
        telemetry.addData("Heading Diff", "%.1f°", dh);
        telemetry.addData("Rel Uncertainty", "%.1f in", relativePose.getPositionUncertainty() / 25.4);
        telemetry.addData("Abs Uncertainty", "%.1f in", absolutePose.getPositionUncertainty() / 25.4);
    }

    /**
     * Display side-by-side comparison
     */
    private void displayComparison(RobotPose2D relativePose, RobotPose2D absolutePose) {
        telemetry.addLine("═══ POSE COMPARISON ═══");
        telemetry.addLine();

        // X position
        telemetry.addData("X Position", "");
        telemetry.addData("  Relative", "%.1f in", relativePose.getX(DistanceUnit.INCH));
        telemetry.addData("  Absolute", "%.1f in", absolutePose.getX(DistanceUnit.INCH));
        telemetry.addData("  Difference", "%.1f in",
            (absolutePose.x - relativePose.x) / 25.4);
        telemetry.addLine();

        // Y position
        telemetry.addData("Y Position", "");
        telemetry.addData("  Relative", "%.1f in", relativePose.getY(DistanceUnit.INCH));
        telemetry.addData("  Absolute", "%.1f in", absolutePose.getY(DistanceUnit.INCH));
        telemetry.addData("  Difference", "%.1f in",
            (absolutePose.y - relativePose.y) / 25.4);
        telemetry.addLine();

        // Heading
        telemetry.addData("Heading", "");
        telemetry.addData("  Relative", "%.1f°", relativePose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("  Absolute", "%.1f°", absolutePose.getHeading(AngleUnit.DEGREES));
        double dh = Math.toDegrees(RobotPose2D.angleWrap(absolutePose.heading - relativePose.heading));
        telemetry.addData("  Difference", "%.1f°", dh);
        telemetry.addLine();

        // Distance from origin
        double relDist = Math.sqrt(relativePose.x*relativePose.x + relativePose.y*relativePose.y);
        double absDist = Math.sqrt(absolutePose.x*absolutePose.x + absolutePose.y*absolutePose.y);
        telemetry.addData("Distance from Origin", "");
        telemetry.addData("  Relative", "%.1f in", relDist / 25.4);
        telemetry.addData("  Absolute", "%.1f in", absDist / 25.4);

        // Vision health
        telemetry.addLine();
        telemetry.addData("Vision", localizer.isVisionActive() ? "✅ Active" : "❌ Inactive");
        telemetry.addData("Acceptance", "%.0f%%", localizer.getVisionAcceptanceRate() * 100);

        // Raw Limelight reading (what vision sees right now)
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D limelightRaw = getLimelightPose();
        if (limelightRaw != null) {
            double rawX = -limelightRaw.getPosition().x;
            double rawY = -limelightRaw.getPosition().y;
            telemetry.addLine();
            telemetry.addData("🟠 Raw Limelight", "(%.1f, %.1f) in @ %.1f°",
                rawX, rawY,
                limelightRaw.getOrientation().getYaw(
                    org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
        }

        // Manual update indicator
        if (System.currentTimeMillis() - lastManualUpdateTime < 2000) {
            telemetry.addLine();
            telemetry.addLine("🔧 MANUAL UPDATE APPLIED!");
        }
    }

    /**
     * Perform manual vision update - force accept Limelight reading
     * This bypasses all validation and directly resets the absolute pose to Limelight position
     */
    private void performManualVisionUpdate() {
        // Get Limelight pose directly from hardware
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D limelightPose =
            getLimelightPose();

        if (limelightPose == null) {
            telemetry.addLine("⚠️ No Limelight data available!");
            telemetry.update();
            sleep(500);
            return;
        }

        // Apply 180-degree rotation correction (same as FusionLocalizer does)
        double rawX = limelightPose.getPosition().x;
        double rawY = limelightPose.getPosition().y;
        double rotatedX = -rawX;
        double rotatedY = -rawY;
        double heading = limelightPose.getOrientation().getYaw(
            org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

        // Create new pose from Limelight
        RobotPose2D newPose = new RobotPose2D(rotatedX, rotatedY, heading);

        // Reset localizer to this pose (forces both relative and absolute)
        localizer.reset(newPose);

        // Clear traces
        relativePoseTrace.clear();
        absolutePoseTrace.clear();

        // Show confirmation
        telemetry.clear();
        telemetry.addLine("✅ MANUAL VISION UPDATE APPLIED");
        telemetry.addLine();
        telemetry.addData("New Position", "(%.1f, %.1f) mm",
            newPose.x, newPose.y);
        telemetry.addData("New Heading", "%.1f°",
            Math.toDegrees(newPose.heading));
        telemetry.addLine();
        telemetry.addLine("Robot pose reset to Limelight position");
        telemetry.update();
        sleep(500);

        lastManualUpdateTime = System.currentTimeMillis();
    }

    /**
     * Get raw Limelight pose from hardware
     */
    private org.firstinspires.ftc.robotcore.external.navigation.Pose3D getLimelightPose() {
        try {
            // Get Limelight from hardware map
            com.qualcomm.hardware.limelightvision.Limelight3A limelight =
                hardware.getHardwareMap().get(
                    com.qualcomm.hardware.limelightvision.Limelight3A.class,
                    "limelight"
                );

            // Get latest result
            com.qualcomm.hardware.limelightvision.LLResult result =
                limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Use getBotpose_MT2() for MegaTag2 pose (if available), otherwise getBotpose()
                // MegaTag2 uses multiple AprilTags for better accuracy
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose = null;
                try {
                    // Try MegaTag2 method first
                    pose = result.getBotpose_MT2();
                } catch (Exception e) {
                    // Fallback to regular botpose
                    pose = result.getBotpose();
                }

                // Check if pose has valid position data (not all zeros)
                if (pose != null &&
                    (Math.abs(pose.getPosition().x) > 0.01 ||
                     Math.abs(pose.getPosition().y) > 0.01)) {
                    return pose;
                }
            }

            return null;
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Draw field visualization with both pose estimates
     * Shows:
     * - RELATIVE pose (GREEN) - Input pose (odometry-based, smooth)
     * - ABSOLUTE pose (BLUE) - Output pose (vision-corrected)
     * - Pose traces (lighter colors) - History trail
     */
    private void drawFieldVisualization(RobotPose2D relativePose, RobotPose2D absolutePose) {
        if (panelsField == null) return;

        try {
            // Convert poses from mm to inches for field drawing
            // Both Panels and FusionLocalizer use center as origin (0,0)
            double relX = relativePose.getX(DistanceUnit.INCH);
            double relY = relativePose.getY(DistanceUnit.INCH);
            double relHeading = relativePose.heading;

            double absX = absolutePose.getX(DistanceUnit.INCH);
            double absY = absolutePose.getY(DistanceUnit.INCH);
            double absHeading = absolutePose.heading;

            // Check for NaN or invalid values
            if (Double.isNaN(relX) || Double.isNaN(relY) || Double.isNaN(relHeading) ||
                Double.isNaN(absX) || Double.isNaN(absY) || Double.isNaN(absHeading)) {
                return; // Skip drawing if any values are invalid
            }

            // Draw pose traces (history) first (so they're behind the current poses)
            if (recordTrace && relativePoseTrace.size() > 1) {
                drawPoseTrace(relativePoseTrace, RELATIVE_TRACE_STYLE, DistanceUnit.INCH);
                drawPoseTrace(absolutePoseTrace, ABSOLUTE_TRACE_STYLE, DistanceUnit.INCH);
            }

            // Get raw Limelight reading (if available)
            org.firstinspires.ftc.robotcore.external.navigation.Pose3D limelightRaw = getLimelightPose();

            // Draw raw Limelight pose (ORANGE) - UNMODIFIED to diagnose coordinate system
            if (limelightRaw != null) {
                // Convert meters to inches (no rotation applied)
                double rawX = limelightRaw.getPosition().x * 1000.0 / 25.4;  // meters to inches
                double rawY = limelightRaw.getPosition().y * 1000.0 / 25.4;
                double rawYaw = limelightRaw.getOrientation().getYaw(
                    org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

                drawRobotPose(rawX, rawY, rawYaw, LIMELIGHT_RAW_STYLE, "LIME");
            }

            // Draw RELATIVE pose (GREEN) - Input pose
            drawRobotPose(relX, relY, relHeading, RELATIVE_POSE_STYLE, "REL");

            // Draw ABSOLUTE pose (BLUE) - Output pose
            drawRobotPose(absX, absY, absHeading, ABSOLUTE_POSE_STYLE, "ABS");

            // Draw connection line between poses (shows divergence)
            if (Math.hypot(absX - relX, absY - relY) > 0.5) { // Only if divergence > 0.5 inches
                panelsField.setStyle(new Style("", "#FF9800", 0.5)); // Orange, semi-transparent
                panelsField.moveCursor(relX, relY);
                panelsField.line(absX, absY);
            }

            // Update field display
            panelsField.update();
        } catch (Exception e) {
            // Silently fail to avoid disrupting main loop
        }
    }

    /**
     * Draw a single robot pose on the field
     * @param x X position in inches
     * @param y Y position in inches
     * @param heading Heading in radians
     * @param style Drawing style
     * @param label Label to show above robot
     */
    private void drawRobotPose(double x, double y, double heading, Style style, String label) {
        if (panelsField == null || Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) {
            return;
        }

        // Draw robot body as circle
        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.circle(ROBOT_RADIUS);

        // Draw heading indicator as line
        double headingLength = ROBOT_RADIUS * 1.2;
        double x1 = x + Math.cos(heading) * ROBOT_RADIUS / 2;
        double y1 = y + Math.sin(heading) * ROBOT_RADIUS / 2;
        double x2 = x + Math.cos(heading) * headingLength;
        double y2 = y + Math.sin(heading) * headingLength;

        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * Draw pose trace (history trail)
     * @param trace List of poses to draw
     * @param style Drawing style
     * @param unit Distance unit for conversion
     */
    private void drawPoseTrace(java.util.List<RobotPose2D> trace, Style style, DistanceUnit unit) {
        if (panelsField == null || trace == null || trace.size() < 2) {
            return;
        }

        panelsField.setStyle(style);

        // Draw small circles at each trace point
        for (RobotPose2D pose : trace) {
            double x = pose.getX(unit);
            double y = pose.getY(unit);

            if (!Double.isNaN(x) && !Double.isNaN(y)) {
                panelsField.moveCursor(x, y);
                panelsField.circle(2.0); // Small circle
            }
        }

        // Draw lines connecting trace points
        for (int i = 0; i < trace.size() - 1; i++) {
            RobotPose2D p1 = trace.get(i);
            RobotPose2D p2 = trace.get(i + 1);

            double x1 = p1.getX(unit);
            double y1 = p1.getY(unit);
            double x2 = p2.getX(unit);
            double y2 = p2.getY(unit);

            if (!Double.isNaN(x1) && !Double.isNaN(y1) && !Double.isNaN(x2) && !Double.isNaN(y2)) {
                panelsField.moveCursor(x1, y1);
                panelsField.line(x2, y2);
            }
        }
    }
}
