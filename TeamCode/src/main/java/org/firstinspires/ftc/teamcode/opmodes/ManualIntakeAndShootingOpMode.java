package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.AutoGyroTurret;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IntelMechanumDrive;
import org.firstinspires.ftc.teamcode.util.aurora.LimelightVisionHelper;
import org.firstinspires.ftc.teamcode.util.aurora.Localization;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.v3.IntakePerception;

/**
 * ManualIntakeAndShootingOpMode - Full manual control for intake and shooting
 *
 * This OpMode provides complete manual control over the robot's drive, intake,
 * and shooting systems using the BasicIndexingHelper and BasicFiringHelper classes.
 *
 * GAMEPAD 1 - DRIVE & TURRET:
 *   - Left Stick Y:  Forward/backward movement
 *   - Left Stick X:  Strafe left/right
 *   - Right Stick X: Rotation
 *   - D-Pad:         Fine movement at 25% power (hold for precise positioning)
 *   - Right Bumper:  Run front intake forward
 *   - Left Bumper:   Run back intake forward
 *   - Guide Button:  Toggle ejection on/off
 *
 *   TURRET CONTROLS (Auto-Gyro):
 *   - A Button:      Set turret target to current robot heading (point forward)
 *   - B Button:      Enable/disable auto-gyro mode (toggle)
 *   - X Button:      Manual Limelight update (sync pose when near AprilTag)
 *   - Y Button:      Toggle between AprilTag targets (Blue tag 20 @ 54° / Red tag 24 @ -54°)
 *   - D-Pad Left:    Reset turret to forward (0° robot-relative)
 *
 * GAMEPAD 2 - FIRING AND INDEXING:
 *   Hold to fire:
 *   - A: Short range (1900 RPM)
 *   - B: Mid-range (2200 RPM)
 *   - Y: Long range (2800 RPM)
 *
 *   - X: Stop all firing/shooting systems
 *   - Back Button: Toggle Driver 2 control (allows gamepad2 joysticks to drive)
 *
 *   D-Pad manual controls:
 *   - Up:    Manual transfer front to center
 *   - Down:  Manual transfer back to center
 *   - Left:  Uptake forward (feed to shooter)
 *   - Right: Uptake reverse (retract from shooter)
 *
 *   Intake controls:
 *   - Right Bumper:  Run front intake forward
 *   - Left Bumper:   Run back intake forward
 *   - Right Trigger: Run front intake backward (eject)
 *   - Left Trigger:  Run back intake backward (eject)
 *
 *   Drive controls (when Driver 2 enabled):
 *   - Left Stick:    Move (only if gamepad1 sticks idle)
 *   - Right Stick X: Rotate (only if gamepad1 sticks idle)
 */
@TeleOp(name="Manual Intake & Shooting", group="Competition")
public class ManualIntakeAndShootingOpMode extends LinearOpMode {

    // AprilTag target headings (field-relative - used as fallback when no pose update)
    private static final double BLUE_TAG_20_HEADING = 54.0;   // Blue alliance tag
    private static final double RED_TAG_24_HEADING = -54.0;   // Red alliance tag

    // AprilTag field coordinates (from official field specs)
    // Tag 20 (Blue): -1.482m, -1.413m, 0.749m, Yaw 54°
    // Tag 24 (Red): -1.482m, 1.413m, 0.749m, Yaw -54°
    // Converted to inches (1m = 39.3701 inches)
    private static final double BLUE_TAG_X = -1.482 * 39.3701;  // -58.35 inches
    private static final double BLUE_TAG_Y = -1.413 * 39.3701;  // -55.62 inches
    private static final double RED_TAG_X = -1.482 * 39.3701;   // -58.35 inches
    private static final double RED_TAG_Y = 1.413 * 39.3701;    // 55.62 inches (ORIGINAL VALUE RESTORED)

    // Pose update tracking
    private boolean poseUpdatedWithLimelight = false;  // Track if pose has been updated via Limelight

    // Hardware and helpers
    private AuroraHardwareConfig hardware;
    private BasicIndexingHelper indexingHelper;
    private Shooter shooter;
    private BasicFiringHelper firingHelper;
    private IntelMechanumDrive drive;
    private AutoGyroTurret autoGyroTurret;
    private Localization localization;

    // Intake perception for artifact detection
    private IntakePerception frontIntakePerception;
    private IntakePerception backIntakePerception;

    // AprilTag target toggle state
    private boolean targetingBlueTag = true;  // Start with Blue tag 20

    // Button state tracking for edge detection
    private boolean lastButtonA = false;
    private boolean lastButtonB = false;
    private boolean lastButtonY = false;

    // Button edge detection for ejection toggle
    private boolean lastGuideButton = false;

    // Driver two control
    private boolean driverTwoEnabled = false;
    private boolean lastBackButton = false;

    // Turret button edge detection (gamepad1)
    private boolean lastAButton = false;
    private boolean lastBButton = false;
    private boolean lastXButton = false;
    private boolean lastYButton = false;

    @Override
    public void runOpMode() {
        // ═══════════════════════════════════════════════════════════════════════
        // INITIALIZATION
        // ═══════════════════════════════════════════════════════════════════════

        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("  Manual Intake & Shooting OpMode");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        // Create shooter
        ShooterConfig shooterConfig = new ShooterConfig();
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        shooter.enable();

        // Create indexing helper
        indexingHelper = new BasicIndexingHelper(hardware, telemetry);

        // Create firing helper
        firingHelper = new BasicFiringHelper(shooter, indexingHelper, hardware, telemetry);

        // Create drive system
        drive = new IntelMechanumDrive(hardware, gamepad1);

        // Initialize localization for robot heading
        telemetry.addLine("Initializing localization...");
        telemetry.update();

        localization = new Localization(hardwareMap);

        // Check and display initialization status
        telemetry.addLine();
        telemetry.addLine("═══ LOCALIZATION STATUS ═══");

        if (!localization.isOdometryInitialized()) {
            telemetry.addData("⚠ Odometry", "Not available - turret heading may be inaccurate");
        } else {
            telemetry.addData("✅ Odometry", "Initialized successfully");
        }

        if (!localization.isLimelightInitialized()) {
            telemetry.addData("❌ Limelight", "NOT INITIALIZED");
            String error = localization.getLimelightInitializationError();
            if (error != null) {
                telemetry.addData("  Error", error);
            }
            telemetry.addData("  Expected", "Device 'limelight' (Limelight3A)");
            telemetry.addData("  Action", "Check Driver Station hardware config");
        } else {
            telemetry.addData("✅ Limelight", "Initialized successfully");
        }

        telemetry.addLine("═══════════════════════════════════");
        telemetry.update();

        // Pause for 2 seconds so driver can see the status
        //sleep(2000);

        localization.resetPosition();

        // Set Limelight to MANUAL mode - updates only when explicitly requested
        localization.setLimelightUpdateMode(Localization.LimelightUpdateMode.MANUAL);

        // Initialize auto-gyro turret
        autoGyroTurret = new AutoGyroTurret(hardwareMap, telemetry);

        // Enable auto-gyro by default and reset to forward (with 180° offset for correct orientation)
        double initialHeading = 0.0;
        if (localization.isOdometryInitialized()) {
            initialHeading = localization.getHeading(AngleUnit.DEGREES);
        }
        autoGyroTurret.resetToForward(initialHeading);
        autoGyroTurret.enable();

        // Initialize IntakePerception for both intakes
        IndexingConfig indexingConfig = new IndexingConfig();

        // Front intake perception
        frontIntakePerception = new IntakePerception(
            IntakePerception.IntakeSide.FRONT,
            hardware.getFrontDistanceSensor(),        // Laser sensor
            hardware.getFrontLeftDistanceSensor(),    // REV 2m ToF sensor
            hardware.getFrontRightColorSensor(),      // Outward color sensor
            hardware.getFrontLeftColorSensor(),       // Mouth color sensor
            indexingConfig
        );

        // Back intake perception
        backIntakePerception = new IntakePerception(
            IntakePerception.IntakeSide.BACK,
            hardware.getBackDistanceSensor(),         // Laser sensor
            hardware.getBackRightDistanceSensor(),    // REV 2m ToF sensor
            hardware.getBackRightColorSensor(),       // Outward color sensor (note: reused)
            hardware.getLeftRightColorSensor(),       // Mouth color sensor
            indexingConfig
        );

        telemetry.addLine("✅ Initialization complete!");
        telemetry.addData("Turret", "Auto-gyro ENABLED by default");
        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("GAMEPAD 1 - DRIVE & TURRET");
        telemetry.addLine("  Left Stick: Move");
        telemetry.addLine("  Right Stick X: Rotate");
        telemetry.addLine("  D-Pad: Fine Movement (50% power)");
        telemetry.addLine("  Right/Left Bumper: Intakes");
        telemetry.addLine("  Guide Button: Toggle Ejection");
        telemetry.addLine();
        telemetry.addLine("  TURRET (Auto-Gyro):");
        telemetry.addLine("  A: Set to robot heading");
        telemetry.addLine("  B: Enable/Disable auto-gyro");
        telemetry.addLine("  X: Manual Limelight update");
        telemetry.addLine("  Y: Toggle AprilTag target");
        telemetry.addLine("  D-Pad Left: Reset turret forward");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD 2 - FIRING & INDEXING");
        telemetry.addLine("  A (Hold): Short Range Fire");
        telemetry.addLine("  B (Hold): Mid Range Fire");
        telemetry.addLine("  Y (Hold): Long Range Fire");
        telemetry.addLine("  X: Stop All");
        telemetry.addLine("  Back: Toggle Driver 2 Control");
        telemetry.addLine();
        telemetry.addLine("  Joysticks: Drive (when enabled)");
        telemetry.addLine();
        telemetry.addLine("  DPad Up: Transfer Front → Center");
        telemetry.addLine("  DPad Down: Transfer Back → Center");
        telemetry.addLine("  DPad Left: Uptake Forward");
        telemetry.addLine("  DPad Right: Uptake Reverse");
        telemetry.addLine();
        telemetry.addLine("  Right Bumper: Front Intake Fwd");
        telemetry.addLine("  Left Bumper: Back Intake Fwd");
        telemetry.addLine("  Right Trigger: Front Intake Rev");
        telemetry.addLine("  Left Trigger: Back Intake Rev");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        // ═══════════════════════════════════════════════════════════════════════
        // MAIN CONTROL LOOP
        // ═══════════════════════════════════════════════════════════════════════

        while (opModeIsActive()) {
            // CRITICAL: Update all helpers every loop
            // NOTE: shooter.update() is called inside firingHelper.update() - don't call twice!
            indexingHelper.update();
            firingHelper.update();

            // Update intake perception to refresh sensor data
            frontIntakePerception.update();
            backIntakePerception.update();

            // Update localization for robot heading (MANUAL mode - Limelight only on X button)
            if (localization != null) {
                localization.update();  // Updates odometry, Limelight only when manually triggered
            }

            // Get current robot heading for turret control
            double robotHeading = 0.0;
            if (localization != null && localization.isOdometryInitialized()) {
                robotHeading = localization.getHeading(AngleUnit.DEGREES);
            }

            // ═══════════════════════════════════════════════════════════════
            // GAMEPAD 1 - TURRET CONTROLS (AUTO-GYRO)
            // ═══════════════════════════════════════════════════════════════

            // A Button - Set current robot heading as turret target (point forward)
            if (gamepad1.a && !lastAButton) {
                // Set turret to point in the same direction the robot is currently facing
                // This makes the field-relative target = current robot heading
                autoGyroTurret.setFieldRelativeHeading(robotHeading+180, robotHeading+180);
                autoGyroTurret.enable();
                telemetry.addLine(String.format("🎯 Turret: Set to robot heading (%.1f°)", robotHeading));
            }
            lastAButton = gamepad1.a;

            // B Button - Toggle auto-gyro mode on/off
            if (gamepad1.b && !lastBButton) {
                boolean newState = autoGyroTurret.toggle();
                if (newState) {
                    telemetry.addLine("✅ Turret: Auto-gyro ENABLED");
                } else {
                    telemetry.addLine("❌ Turret: Auto-gyro DISABLED (servo at 0°)");
                }
            }
            lastBButton = gamepad1.b;

            // X Button - Manual Limelight update (sync pose when near AprilTag)
            if (gamepad1.x && !lastXButton) {
                boolean success = localization.updateWithLimelight();
                if (success) {
                    poseUpdatedWithLimelight = true;  // Enable calculated turret angles
                    telemetry.addLine("✅ Limelight: Pose updated successfully");
                } else {
                    telemetry.addLine("⚠️ Limelight: Update failed (check conditions)");
                }
            }
            lastXButton = gamepad1.x;

            // Y Button - Toggle between AprilTag targets (Blue tag 20 @ 54° and Red tag 24 @ -54°)
            if (gamepad1.y && !lastYButton) {
                // Toggle target
                targetingBlueTag = !targetingBlueTag;

                double targetHeading;
                String tagName = targetingBlueTag ? "Blue Tag 20" : "Red Tag 24";

                // If pose has been updated with Limelight at least once, calculate angle from robot to tag
                if (poseUpdatedWithLimelight && localization.isOdometryInitialized()) {
                    // Get current robot position (in rotated Limelight coordinate system)
                    double robotXRotated = localization.getX(DistanceUnit.INCH);
                    double robotYRotated = localization.getY(DistanceUnit.INCH);

                    // Apply inverse 180° rotation to convert robot position back to field coordinate system
                    // Since Limelight data was rotated 180°, we need to rotate it back to match AprilTag coords
                    // Inverse of 180° rotation is another 180° rotation: x' = -x, y' = -y
                    double robotX = -robotXRotated;
                    double robotY = -robotYRotated;

                    // Get target tag coordinates (in original field coordinate system)
                    double tagX = targetingBlueTag ? BLUE_TAG_X : RED_TAG_X;
                    double tagY = targetingBlueTag ? BLUE_TAG_Y : RED_TAG_Y;

                    // Calculate angle from robot to target tag
                    double deltaX = tagX - robotX;
                    double deltaY = tagY - robotY;

                    // Calculate angle in degrees (atan2 returns radians)
                    // atan2(y, x) gives angle from positive X-axis
                    targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));

                    // Add 180° to flip turret around (front of turret instead of back)
                    targetHeading += 180.0;

                    // Calculate distance for telemetry
                    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

                    telemetry.addLine(String.format("🎯 Turret: Target %s (calculated)", tagName));
                    telemetry.addLine(String.format("   Angle: %.1f° | Distance: %.1f in", targetHeading, distance));
                    telemetry.addLine(String.format("   Robot: (%.1f, %.1f) → Tag: (%.1f, %.1f)",
                                                    robotX, robotY, tagX, tagY));
                } else {
                    // Fallback to fixed heading if pose hasn't been updated yet
                    targetHeading = targetingBlueTag ? BLUE_TAG_20_HEADING : RED_TAG_24_HEADING;
                    telemetry.addLine(String.format("🎯 Turret: Target %s (%.0f° fixed)", tagName, targetHeading));
                    if (!poseUpdatedWithLimelight) {
                        telemetry.addLine("   ⚠️ Using fixed angle - press X near tag to enable calculated angles");
                    }
                }

                // Set turret to calculated or fixed target heading
                autoGyroTurret.setFieldRelativeHeading(targetHeading, robotHeading);
                autoGyroTurret.enable();
            }
            lastYButton = gamepad1.y;

            // Continuous turret angle update: Recalculate angle to target as robot moves
            // Only when pose has been updated and turret is enabled
            if (poseUpdatedWithLimelight && autoGyroTurret.isEnabled() && localization.isOdometryInitialized()) {
                // Get current robot position (in rotated Limelight coordinate system)
                double robotXRotated = localization.getX(DistanceUnit.INCH);
                double robotYRotated = localization.getY(DistanceUnit.INCH);

                // Apply inverse 180° rotation to convert back to field coordinate system
                double robotX = -robotXRotated;
                double robotY = -robotYRotated;

                // Get target tag coordinates
                double tagX = targetingBlueTag ? BLUE_TAG_X : RED_TAG_X;
                double tagY = targetingBlueTag ? BLUE_TAG_Y : RED_TAG_Y;

                // Calculate angle from robot to target tag
                double deltaX = tagX - robotX;
                double deltaY = tagY - robotY;

                // Calculate field-relative angle
                double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));

                // Add 180° to flip turret around (front instead of back)
                targetHeading += 180.0;

                // Update turret to point at target
                autoGyroTurret.setFieldRelativeHeading(targetHeading, robotHeading);
            }

            // Update turret to maintain field-relative heading (if enabled)
            // If disabled, this will set servo to position 0 (0° / forward)
            autoGyroTurret.update(robotHeading);

            // ═══════════════════════════════════════════════════════════════
            // GAMEPAD 1 - DRIVE CONTROLS
            // ═══════════════════════════════════════════════════════════

            // Robot-centric drive (relative to robot's orientation)
            double forward = -gamepad1.right_stick_y;  // Note: Y-axis is inverted
            double strafe = gamepad1.right_stick_x;
            double rotate = gamepad1.left_stick_x;

            // Check if gamepad1 joysticks are idle (within deadzone)
            final double JOYSTICK_DEADZONE = 0.1;
            boolean gamepad1Idle = (Math.abs(gamepad1.right_stick_y) < JOYSTICK_DEADZONE &&
                                   Math.abs(gamepad1.right_stick_x) < JOYSTICK_DEADZONE &&
                                   Math.abs(gamepad1.left_stick_x) < JOYSTICK_DEADZONE);

            // Driver 2 controls (gamepad2 joysticks) - only if enabled and gamepad1 is idle
            if (driverTwoEnabled && gamepad1Idle) {
                forward = -gamepad2.right_stick_y;  // Note: Y-axis is inverted
                strafe = gamepad2.right_stick_x;
                rotate = gamepad2.left_stick_x;
            }

            // D-Pad controls
            final double FINE_MOVE_POWER = 0.5;  // 50% power for precise control

            // D-Pad Left - Reset turret to forward
            if (gamepad1.dpad_left) {
                autoGyroTurret.resetToForward(robotHeading);
                autoGyroTurret.enable();
                telemetry.addLine("⬆️ Turret: Reset to forward");
            }

            // D-Pad Up/Down/Right - Fine movement controls (low power for precise positioning)
            if (gamepad1.dpad_up) {
                forward = FINE_MOVE_POWER;  // Move forward slowly
            }
            if (gamepad1.dpad_down) {
                forward = -FINE_MOVE_POWER;  // Move backward slowly
            }
            if (gamepad1.dpad_right) {
                strafe = FINE_MOVE_POWER;  // Strafe right slowly
            }

            drive.setMechanumPowers(forward, strafe, rotate);

            // Gamepad 1 Bumpers - Intake Control
            if (!indexingHelper.isTransferActive() && !firingHelper.isEjecting()) {
                if (gamepad1.right_bumper) {
                    indexingHelper.runFrontIntake(true, 1.0);
                } else if (!gamepad2.right_bumper && !(gamepad2.right_trigger > 0.1)) {
                    // Only stop if gamepad2 isn't controlling it
                    indexingHelper.runFrontIntake(false, 0);
                }

                if (gamepad1.left_bumper) {
                    indexingHelper.runBackIntake(true, 1.0);
                } else if (!gamepad2.left_bumper && !(gamepad2.left_trigger > 0.1)) {
                    // Only stop if gamepad2 isn't controlling it
                    indexingHelper.runBackIntake(false, 0);
                }
            }

            // Gamepad 1 Guide Button - Toggle Ejection (Edge Detection)
            boolean currentGuideButton = gamepad1.guide;
            if (currentGuideButton && !lastGuideButton) {
                if (firingHelper.isEjecting()) {
                    firingHelper.stopEjection();
                    telemetry.addLine("⏹️ Ejection stopped");
                } else {
                    firingHelper.startEjection();
                    telemetry.addLine("▶️ Ejection started");
                }
            }
            lastGuideButton = currentGuideButton;

            // ═══════════════════════════════════════════════════════════════
            // GAMEPAD 2 - DRIVER TWO CONTROL TOGGLE
            // ═══════════════════════════════════════════════════════════════

            // Gamepad 2 Back Button - Toggle Driver Two Control (Edge Detection)
            boolean currentBackButton = gamepad2.back;
            if (currentBackButton && !lastBackButton) {
                driverTwoEnabled = !driverTwoEnabled;
                if (driverTwoEnabled) {
                    telemetry.addLine("✅ Driver 2 control ENABLED");
                } else {
                    telemetry.addLine("❌ Driver 2 control DISABLED");
                }
            }
            lastBackButton = currentBackButton;

            // ═══════════════════════════════════════════════════════════════
            // GAMEPAD 2 - FIRING CONTROLS (EDGE DETECTION)
            // ═══════════════════════════════════════════════════════════
            // Press A/B/Y to start firing or fire next shot
            // Shooter stays spinning after button release (keep-alive mode)
            // Press X to stop shooter completely

            // Check if turret is busy (moving/settling) - don't fire if busy
            boolean turretBusy = autoGyroTurret.isBusy();

            // Short Range (A button) - press to fire
            boolean currentButtonA = gamepad2.a;
            if (currentButtonA && !lastButtonA) {  // Button just pressed
                if (turretBusy) {
                    telemetry.addLine("⏳ Turret busy - waiting to fire...");
                } else if (!firingHelper.isFiring()) {
                    firingHelper.startFiringShortRange();
                    telemetry.addLine("🔥 Started Short Range firing");
                } else if (firingHelper.isReadyForNextShot()) {
                    firingHelper.startFiringShortRange();
                    telemetry.addLine("🔥 Firing next shot (Short Range)");
                }
            }
            lastButtonA = currentButtonA;

            // Mid-Range (B button) - press to fire
            boolean currentButtonB = gamepad2.b;
            if (currentButtonB && !lastButtonB) {  // Button just pressed
                if (turretBusy) {
                    telemetry.addLine("⏳ Turret busy - waiting to fire...");
                } else if (!firingHelper.isFiring()) {
                    firingHelper.startFiringMidRange();
                    telemetry.addLine("🔥 Started Mid Range firing");
                } else if (firingHelper.isReadyForNextShot()) {
                    firingHelper.startFiringMidRange();
                    telemetry.addLine("🔥 Firing next shot (Mid Range)");
                }
            }
            lastButtonB = currentButtonB;

            // Long Range (Y button) - press to fire
            boolean currentButtonY = gamepad2.y;
            if (currentButtonY && !lastButtonY) {  // Button just pressed
                if (turretBusy) {
                    telemetry.addLine("⏳ Turret busy - waiting to fire...");
                } else if (!firingHelper.isFiring()) {
                    firingHelper.startFiringLongRange();
                    telemetry.addLine("🔥 Started Long Range firing");
                } else if (firingHelper.isReadyForNextShot()) {
                    firingHelper.startFiringLongRange();
                    telemetry.addLine("🔥 Firing next shot (Long Range)");
                }
            }
            lastButtonY = currentButtonY;

            // X button - Stop all shooter/firing systems
            if (gamepad2.x) {
                shooter.stopMotors();
                firingHelper.cancelFiring();
                indexingHelper.stopUptake();
                telemetry.addLine("⏹️ Stopped all firing systems");
            }

            // ═══════════════════════════════════════════════════════════════
            // GAMEPAD 2 - D-PAD MANUAL CONTROLS
            // ═══════════════════════════════════════════════════════════

            // Manual transfer controls - allow during firing, but not during ejection
            if (!firingHelper.isEjecting()) {
                // Declare both transfer states at the top so they're available for cross-checking
                boolean frontTransferActive = gamepad2.dpad_up;
                boolean backTransferActive = gamepad2.dpad_down;

                // DPad Up - Manual transfer front to center (hold button)
                indexingHelper.transferFrontIntakeToCenterManual(frontTransferActive);

                // When front transfer is active, check if back intake has artifact
                if (frontTransferActive) {
                    // Check back intake for artifact (minimum MEDIUM confidence)
                    IntakePerception.PresenceConfidence backConfidence = backIntakePerception.getPresenceConfidence();
                    boolean backHasArtifact = (backConfidence == IntakePerception.PresenceConfidence.MEDIUM ||
                                              backConfidence == IntakePerception.PresenceConfidence.HIGH);

                    if (!backHasArtifact) {
                        // No artifact in back intake - run back transfer backward to help clear path
                        indexingHelper.setBackTransferPower(1.0);
                        telemetry.addData("🔄 Back Transfer", "Running backward (no artifact detected)");
                    } else {
                        // Artifact detected in back intake - stop back transfer
                        indexingHelper.setBackTransferPower(0);
                        telemetry.addData("⚠️ Back Intake", "Artifact detected - transfer stopped");
                    }
                } else if (!backTransferActive) {
                    // Front transfer not active AND back transfer not active - ensure back transfer is stopped
                    indexingHelper.setBackTransferPower(0);
                }

                // DPad Down - Manual transfer back to center (hold button)
                indexingHelper.transferBackIntakeToCenterManual(backTransferActive);

                // When back transfer is active, check if front intake has artifact
                if (backTransferActive) {
                    // Check front intake for artifact (minimum MEDIUM confidence)
                    IntakePerception.PresenceConfidence frontConfidence = frontIntakePerception.getPresenceConfidence();
                    boolean frontHasArtifact = (frontConfidence == IntakePerception.PresenceConfidence.MEDIUM ||
                                               frontConfidence == IntakePerception.PresenceConfidence.HIGH);

                    if (!frontHasArtifact) {
                        // No artifact in front intake - run front transfer backward to help clear path
                        indexingHelper.setFrontTransferPower(1.0);
                        telemetry.addData("🔄 Front Transfer", "Running backward (no artifact detected)");
                    } else {
                        // Artifact detected in front intake - stop front transfer
                        indexingHelper.setFrontTransferPower(0);
                        telemetry.addData("⚠️ Front Intake", "Artifact detected - transfer stopped");
                    }
                } else if (!frontTransferActive) {
                    // Back transfer not active AND front transfer not active - ensure front transfer is stopped
                    indexingHelper.setFrontTransferPower(0);
                }
            }

            // Uptake manual controls - only when not busy with timed movements or firing
            if (!indexingHelper.isUptakeBusy() && !indexingHelper.isTransferActive() &&
                !firingHelper.isFiring() && !firingHelper.isEjecting()) {

                if (gamepad2.dpad_left) {
                    // DPad Left - Uptake forward (feed to shooter)
                    indexingHelper.setUptakePower(1.0);
                } else if (gamepad2.dpad_right) {
                    // DPad Right - Uptake reverse (retract from shooter)
                    indexingHelper.setUptakePower(-1.0);
                } else {
                    // Stop uptake when no button pressed
                    indexingHelper.setUptakePower(0);
                }
            }

            // ═══════════════════════════════════════════════════════════════
            // GAMEPAD 2 - INTAKE CONTROLS (BUMPERS & TRIGGERS)
            // ═══════════════════════════════════════════════════════════════

            // Simple intake control - only if no transfer or ejection active
            if (!indexingHelper.isTransferActive() && !firingHelper.isEjecting()) {
                // Right Bumper - Run front intake forward
                // Right Trigger - Run front intake backward (eject)
                if (gamepad2.right_bumper) {
                    indexingHelper.runFrontIntake(true, 1.0);
                } else if (gamepad2.right_trigger > 0.1) {
                    indexingHelper.runFrontIntake(true, -1.0);  // Negative power for reverse
                } else {
                    indexingHelper.runFrontIntake(false, 0);
                }

                // Left Bumper - Run back intake forward
                // Left Trigger - Run back intake backward (eject)
                if (gamepad2.left_bumper) {
                    indexingHelper.runBackIntake(true, 1.0);
                } else if (gamepad2.left_trigger > 0.1) {
                    indexingHelper.runBackIntake(true, -1.0);  // Negative power for reverse
                } else {
                    indexingHelper.runBackIntake(false, 0);
                }
            }

            // ═══════════════════════════════════════════════════════════════
            // TELEMETRY - STATUS DISPLAY
            // ═══════════════════════════════════════════════════════════

            telemetry.addData("═══ DRIVE ═══", "");
            telemetry.addData("Forward", String.format("%.2f", forward));
            telemetry.addData("Strafe", String.format("%.2f", strafe));
            telemetry.addData("Rotate", String.format("%.2f", rotate));
            telemetry.addData("GP1 Idle", gamepad1Idle ? "YES" : "NO");
            telemetry.addData("Driver 2 Enabled", driverTwoEnabled ? "YES ✅" : "NO");
            if (driverTwoEnabled && gamepad1Idle) {
                telemetry.addData("Active Driver", "DRIVER 2 (GP2)");
            } else {
                telemetry.addData("Active Driver", "DRIVER 1 (GP1)");
            }
            telemetry.addData("", "");

            telemetry.addData("═══ FIRING STATUS ═══", "");
            telemetry.addData("Active", firingHelper.isFiring() ? "YES ✅" : "NO");
            if (firingHelper.isFiring()) {
                telemetry.addData("State", firingHelper.getFiringState());
                telemetry.addData("Preset", firingHelper.getPresetName());
                telemetry.addData("Target RPM", String.format("%.0f", firingHelper.getTargetRPM()));
                telemetry.addData("Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
                telemetry.addData("Ready for Next", firingHelper.isReadyForNextShot() ? "YES ✅" : "NO");
            }
            telemetry.addData("", "");

            telemetry.addData("═══ SHOOTER ═══", "");
            telemetry.addData("Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
            telemetry.addData("Target RPM", String.format("%.0f", shooter.getTargetRPM()));
            telemetry.addData("Ready", shooter.isReadyToFire() ? "YES ✅" : "NO");
            telemetry.addData("State", shooter.getState());
            telemetry.addData("", "");

            telemetry.addData("═══ TURRET (AUTO-GYRO) ═══", "");
            telemetry.addData("Mode", autoGyroTurret.isEnabled() ? "ENABLED ✅" : "DISABLED (Servo @ 0°)");
            telemetry.addData("Busy", autoGyroTurret.isBusy() ? "YES ⏳" : "NO");
            if (autoGyroTurret.isEnabled()) {
                telemetry.addData("Field Target", String.format("%.1f°", autoGyroTurret.getFieldRelativeHeading()));
                telemetry.addData("Field Current", String.format("%.1f°", autoGyroTurret.getCurrentFieldHeading(robotHeading)));
                telemetry.addData("At Target", autoGyroTurret.isAtTarget(robotHeading) ? "YES ✅" : "NO");
            }
            telemetry.addData("Robot Heading", String.format("%.1f°", robotHeading));
            telemetry.addData("", "");

            telemetry.addData("═══ INDEXING ═══", "");
            telemetry.addData("Transfer Active", indexingHelper.isTransferActive() ? "YES" : "NO");
            if (indexingHelper.isTransferActive()) {
                telemetry.addData("Transfer State", indexingHelper.getTransferState());
                telemetry.addData("Transfer Type", indexingHelper.getTransferType());
            }
            telemetry.addData("Any Busy", indexingHelper.isAnyBusy() ? "YES" : "NO");
            telemetry.addData("Ejection", firingHelper.isEjecting() ? "ACTIVE ⚠️" : "IDLE");
            telemetry.addData("", "");

            telemetry.addData("═══ PERCEPTION ═══", "");
            // Front intake perception
            IntakePerception.PresenceConfidence frontConf = frontIntakePerception.getPresenceConfidence();
            String frontStatus = getConfidenceIcon(frontConf) + " " + frontConf.toString();
            telemetry.addData("Front Intake", frontStatus);

            // Back intake perception
            IntakePerception.PresenceConfidence backConf = backIntakePerception.getPresenceConfidence();
            String backStatus = getConfidenceIcon(backConf) + " " + backConf.toString();
            telemetry.addData("Back Intake", backStatus);
            telemetry.addData("", "");

            telemetry.addData("═══ LIMELIGHT DEBUG ═══", "");
            if (localization != null && localization.isLimelightInitialized()) {
                LimelightVisionHelper limelight = localization.getLimelight();
                boolean hasTarget = limelight.hasTarget();

                telemetry.addData("Status", "✅ ONLINE");
                telemetry.addData("Has Target", hasTarget ? "✅ YES" : "❌ NO");

                if (hasTarget) {
                    // AprilTag detection info
                    telemetry.addData("Target X", String.format("%.2f°", limelight.getTargetX()));
                    telemetry.addData("Target Y", String.format("%.2f°", limelight.getTargetY()));
                    telemetry.addData("Target Area", String.format("%.2f%%", limelight.getTargetArea()));

                    // Data quality indicators
                    boolean dataFresh = localization.isVisionDataFresh();
                    boolean dataQualityGood = localization.isVisionDataQualityGood();
                    long dataAge = localization.getVisionDataAge();

                    telemetry.addData("Data Fresh", dataFresh ? "✅ YES" : "⚠️ STALE");
                    telemetry.addData("Data Quality", dataQualityGood ? "✅ GOOD" : "⚠️ POOR");
                    telemetry.addData("Data Age", String.format("%d ms", dataAge));

                    // Heading stability tracking
                    boolean headingStable = localization.isLimelightHeadingStable();
                    int stableCount = localization.getStableHeadingCount();
                    boolean odometryUpdated = localization.wasOdometryUpdatedByLimelight();

                    // Velocity check for accurate updates
                    double velocityMM = localization.getVelocityMagnitude(DistanceUnit.MM);
                    boolean velocityLow = localization.isVelocityLowForUpdate();

                    telemetry.addData("Heading Stable", headingStable ? "✅ YES" : "❌ NO");
                    telemetry.addData("Stable Count", stableCount + "/10");

                    // Velocity status
                    telemetry.addData("Robot Velocity", String.format("%.1f mm/s", velocityMM));
                    telemetry.addData("Velocity OK", velocityLow ? "✅ YES (< 100mm/s)" : "⚠️ TOO FAST");

                    if (headingStable) {
                        double stableHeading = localization.getStableLimelightHeading(AngleUnit.DEGREES);
                        telemetry.addData("Stable Heading", String.format("%.1f°", stableHeading));

                        if (velocityLow) {
                            telemetry.addData("Using LL Heading", "✅ YES (stable + slow)");
                            telemetry.addData("Odometry Updated", odometryUpdated ? "✅ YES (synced)" : "⚠️ Jump too large");
                        } else {
                            telemetry.addData("Using LL Heading", "⚠️ SKIPPED (moving too fast)");
                            telemetry.addData("Odometry Updated", "❌ NO (velocity too high)");
                        }
                    } else {
                        telemetry.addData("Using LL Heading", "❌ NO (not stable)");
                        telemetry.addData("Odometry Updated", "❌ NO");
                    }

                    // Robot pose from Limelight
                    org.firstinspires.ftc.robotcore.external.navigation.Pose3D visionPose = limelight.getRobotPose();
                    if (visionPose != null) {
                        telemetry.addData("Vision X", String.format("%.1f mm", visionPose.getPosition().x));
                        telemetry.addData("Vision Y", String.format("%.1f mm", visionPose.getPosition().y));
                        telemetry.addData("Vision Yaw", String.format("%.1f°", visionPose.getOrientation().getYaw()));

                        // Distance to target (approximate using Z)
                        double distMM = Math.abs(visionPose.getPosition().z);
                        double distFeet = distMM / 304.8;
                        telemetry.addData("Target Dist", String.format("%.1f ft (%.0f mm)", distFeet, distMM));
                    } else {
                        telemetry.addData("Vision Pose", "❌ NULL");
                    }
                } else {
                    telemetry.addData("Info", "No AprilTags visible");
                }
            } else if (localization != null) {
                telemetry.addData("Status", "❌ NOT INITIALIZED");
                String error = localization.getLimelightInitializationError();
                if (error != null) {
                    telemetry.addData("Error", error);
                }
                telemetry.addData("Expected", "Device 'limelight' (Limelight3A)");
                telemetry.addData("Fix", "Configure in Driver Station");
            } else {
                telemetry.addData("Status", "❌ LOCALIZATION NULL");
            }
            telemetry.addData("", "");

            telemetry.addData("═══ ACTIVE CONTROLS ═══", "");
            // Gamepad 1 - Turret
            if (gamepad1.a) telemetry.addLine("🎯 GP1 A - Set Robot Heading");
            if (gamepad1.b) telemetry.addLine("🔄 GP1 B - Toggle Auto-Gyro");
            if (gamepad1.x) telemetry.addLine("📡 GP1 X - Manual Limelight Update");
            if (gamepad1.y) telemetry.addLine("🧭 GP1 Y - Toggle AprilTag Target");
            // Gamepad 1 - Drive
            if (gamepad1.guide) telemetry.addLine("🔘 GP1 Guide - Toggle Ejection");
            if (gamepad1.dpad_up) telemetry.addLine("⬆️ GP1 DPad Up - Fine Fwd");
            if (gamepad1.dpad_down) telemetry.addLine("⬇️ GP1 DPad Down - Fine Back");
            if (gamepad1.dpad_left) telemetry.addLine("⬅️ GP1 DPad Left - Reset Turret");
            if (gamepad1.dpad_right) telemetry.addLine("➡️ GP1 DPad Right - Fine Strafe R");
            if (gamepad1.right_bumper) telemetry.addLine("🔼 GP1 RB - Front Intake");
            if (gamepad1.left_bumper) telemetry.addLine("🔽 GP1 LB - Back Intake");
            // Gamepad 2
            if (gamepad2.back) telemetry.addLine("🔄 GP2 Back - Toggle Driver 2");
            if (gamepad2.a) telemetry.addLine("🔴 A - Short Range");
            if (gamepad2.b) telemetry.addLine("🔵 B - Mid Range");
            if (gamepad2.y) telemetry.addLine("🟡 Y - Long Range");
            if (gamepad2.x) telemetry.addLine("⏹️ X - Stop");
            if (gamepad2.dpad_up) telemetry.addLine("⬆️ Front → Center");
            if (gamepad2.dpad_down) telemetry.addLine("⬇️ Back → Center");
            if (gamepad2.dpad_left) telemetry.addLine("⬅️ Uptake Forward");
            if (gamepad2.dpad_right) telemetry.addLine("➡️ Uptake Reverse");
            if (gamepad2.right_bumper) telemetry.addLine("🔼 Front Intake Forward");
            if (gamepad2.left_bumper) telemetry.addLine("🔽 Back Intake Forward");
            if (gamepad2.right_trigger > 0.1) telemetry.addLine("🔽 Front Intake Reverse");
            if (gamepad2.left_trigger > 0.1) telemetry.addLine("🔽 Back Intake Reverse");

            telemetry.update();
        }

        // ═══════════════════════════════════════════════════════════════════════
        // CLEANUP - Stop everything on exit
        // ═══════════════════════════════════════════════════════════════════════

        firingHelper.stopAll();
        shooter.stopMotors();
        indexingHelper.stopAll();
        drive.stop();

        telemetry.addLine("🛑 OpMode stopped - all systems disabled");
        telemetry.update();
    }

    /**
     * Get icon for presence confidence level
     */
    private String getConfidenceIcon(IntakePerception.PresenceConfidence confidence) {
        switch (confidence) {
            case NONE:
                return "⚪";  // Empty circle - no artifact
            case LOW:
                return "🟡";  // Yellow - low confidence
            case MEDIUM:
                return "🟠";  // Orange - medium confidence
            case HIGH:
                return "🟢";  // Green - high confidence
            default:
                return "❓";  // Unknown
        }
    }
}
