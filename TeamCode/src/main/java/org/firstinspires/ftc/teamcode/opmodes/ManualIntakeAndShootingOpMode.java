package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.AutoGyroTurret;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IntelMechanumDrive;
import org.firstinspires.ftc.teamcode.util.aurora.localization.LimelightVisionHelper;
import org.firstinspires.ftc.teamcode.util.aurora.localization.Localization;
import org.firstinspires.ftc.teamcode.util.aurora.PerformanceMonitor;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.v3.IntakePerception;

// Pedro Pathing imports for field drawing
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//NOT LEGACY OPMODE
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
 *   TURRET CONTROLS (Manual Auto-Gyro Only):
 *   - A Button:      Set turret target to current robot heading (point forward)
 *   - B Button:      Enable/disable auto-gyro mode (toggle)
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
@Disabled
public class ManualIntakeAndShootingOpMode extends LinearOpMode {

    // Hardware and helpers
    private AuroraHardwareConfig hardware;
    private BasicIndexingHelper indexingHelper;
    private Shooter shooter;
    private BasicFiringHelper firingHelper;
    private IntelMechanumDrive drive;
    private AutoGyroTurret autoGyroTurret;
    private Localization localization;
    private PerformanceMonitor performanceMonitor;

    // Intake perception for artifact detection
    private IntakePerception frontIntakePerception;
    private IntakePerception backIntakePerception;

    // Pedro Pathing follower for field drawing
    private Follower follower;
    private FieldManager panelsField;
    private TelemetryManager telemetryM;
    private static final double ROBOT_RADIUS = 9.0; // inches
    private static final Style robotLook = new Style("", "#3F51B5", 0.75);

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

    // Telemetry optimization
    private static final boolean ENABLE_DEBUG_TELEMETRY = false;  // Set to false for competition
    private static final int TELEMETRY_UPDATE_INTERVAL_MS = 100;  // Update telemetry every 100ms (10Hz)
    private long lastTelemetryUpdateTime = 0;
    private int loopsSinceLastTelemetry = 0;

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
        @SuppressWarnings("deprecation")
        IntakePerception tempFront = new IntakePerception(
            IntakePerception.IntakeSide.FRONT,
            hardware.getFrontDistanceSensor(),        // goBILDA laser sensor (confirmation)
            hardware.getFrontIntakeColorLeft(),       // Left color sensor
            hardware.getFrontIntakeColorRight(),      // Right color sensor
            indexingConfig
        );
        frontIntakePerception = tempFront;

        // Back intake perception
        @SuppressWarnings("deprecation")
        IntakePerception tempBack = new IntakePerception(
            IntakePerception.IntakeSide.BACK,
            hardware.getBackDistanceSensor(),         // goBILDA laser sensor (confirmation)
            hardware.getBackIntakeColorLeft(),        // Left color sensor
            hardware.getBackIntakeColorRight(),       // Right color sensor
            indexingConfig
        );
        backIntakePerception = tempBack;

        // Initialize Performance Monitor
        performanceMonitor = new PerformanceMonitor(telemetry);
        performanceMonitor.setEnabled(true);

        // Initialize Pedro Pathing follower for field drawing
        telemetry.addLine("Initializing field drawing...");
        telemetry.update();

        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(72, 72)); // Default starting position
            panelsField = PanelsField.INSTANCE.getField();
            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            telemetry.addLine("✅ Field drawing initialized");
        } catch (Exception e) {
            telemetry.addLine("⚠️ Field drawing initialization failed: " + e.getMessage());
            follower = null; // Set to null so we can check later
        }
        telemetry.update();

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
        telemetry.addLine("  TURRET (Auto-Gyro - Manual Only):");
        telemetry.addLine("  A: Set to robot heading");
        telemetry.addLine("  B: Enable/Disable auto-gyro");
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
            // Start performance monitoring for this loop
            performanceMonitor.startLoop();

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

            // Update follower pose with localization data for field drawing
            if (follower != null && localization != null && localization.isOdometryInitialized()) {
                double x = localization.getX(DistanceUnit.INCH);
                double y = localization.getY(DistanceUnit.INCH);
                double heading = localization.getHeading(AngleUnit.RADIANS);
                follower.setPose(new Pose(x, y, heading));
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

            // X Button - DISABLED (was Manual Limelight update)
            // Y Button - DISABLED (was Toggle AprilTag targets)
            // NOTE: AprilTag auto-targeting has been disabled for manual control only
            // Turret now only maintains the heading set by A button or manual controls

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

            // Uptake manual controls - allow manual control even during transfer pre-positioning
            // Only block during active firing or ejection
            if (!firingHelper.isFiring() && !firingHelper.isEjecting()) {

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
            // TELEMETRY - OPTIMIZED FOR PERFORMANCE
            // ═══════════════════════════════════════════════════════════
            // Only update telemetry every 100ms (10Hz) to avoid lag

            loopsSinceLastTelemetry++;
            long currentTime = System.currentTimeMillis();
            boolean shouldUpdateTelemetry = (currentTime - lastTelemetryUpdateTime) >= TELEMETRY_UPDATE_INTERVAL_MS;

            if (shouldUpdateTelemetry) {
                lastTelemetryUpdateTime = currentTime;

                // CRITICAL INFO ONLY - Keep it minimal for performance
                telemetry.addData("Driver", driverTwoEnabled && gamepad1Idle ? "GP2" : "GP1");
                telemetry.addData("Shooter", String.format("%.0f RPM", shooter.getCurrentRPM()));

                if (firingHelper.isFiring()) {
                    telemetry.addData("Firing", firingHelper.getPresetName());
                }

                if (autoGyroTurret.isEnabled()) {
                    telemetry.addData("Turret", String.format("%.0f°", autoGyroTurret.getFieldRelativeHeading()));
                }

                // End performance monitoring and check for warnings
                double voltage = hardware.getVoltageSensor() != null ?
                    hardware.getVoltageSensor().getVoltage() : 12.5;
                performanceMonitor.endLoop(voltage, gamepad1, gamepad2);

                // Show performance metrics
                performanceMonitor.displayTelemetry(false);  // false = compact mode

                telemetry.addData("Loop Hz", String.format("%.1f", loopsSinceLastTelemetry * 10.0));
                loopsSinceLastTelemetry = 0;

                // OPTIONAL: Full debug telemetry (only if enabled)
                if (ENABLE_DEBUG_TELEMETRY) {
                    addDebugTelemetry(robotHeading);
                }

                telemetry.update();
            }

            // Draw robot position on field (if follower initialized) - but only every other telemetry update
            if (shouldUpdateTelemetry && follower != null && (currentTime % 200) < 100) {
                drawRobotOnField(follower.getPose());
            }
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

    /**
     * Add detailed debug telemetry (only when ENABLE_DEBUG_TELEMETRY = true)
     * This method is EXPENSIVE and should only be used for debugging, not competition
     */
    private void addDebugTelemetry(double robotHeading) {
        telemetry.addLine("═══ DEBUG MODE ═══");

        // Shooter details
        telemetry.addData("Shooter State", shooter.getState());
        telemetry.addData("Shooter Ready", shooter.isReadyToFire() ? "YES" : "NO");

        // Turret details
        if (autoGyroTurret.isEnabled()) {
            telemetry.addData("Turret Current", String.format("%.1f°", autoGyroTurret.getCurrentFieldHeading(robotHeading)));
            telemetry.addData("Turret At Target", autoGyroTurret.isAtTarget(robotHeading) ? "YES" : "NO");
        }

        // Indexing details
        if (indexingHelper.isTransferActive()) {
            telemetry.addData("Transfer", indexingHelper.getTransferType() + " - " + indexingHelper.getTransferState());
        }

        // Perception
        telemetry.addData("Front Intake", getConfidenceIcon(frontIntakePerception.getPresenceConfidence()));
        telemetry.addData("Back Intake", getConfidenceIcon(backIntakePerception.getPresenceConfidence()));

        // Limelight summary (compact)
        if (localization != null && localization.isLimelightInitialized()) {
            LimelightVisionHelper limelight = localization.getLimelight();
            if (limelight.hasTarget()) {
                telemetry.addData("Limelight", String.format("Target @ %.1f°", limelight.getTargetX()));
            }
        }
    }

    /**
     * Draw the robot on the FTC Control Panels field display
     * @param pose Current robot pose (x, y, heading)
     */
    private void drawRobotOnField(Pose pose) {
        if (panelsField == null || pose == null) return;

        try {
            // Check for NaN values
            if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
                return;
            }

            // Set drawing style
            panelsField.setStyle(robotLook);

            // Draw robot body as circle
            panelsField.moveCursor(pose.getX(), pose.getY());
            panelsField.circle(ROBOT_RADIUS);

            // Draw heading indicator as a line using trigonometry
            double heading = pose.getHeading();
            double x1 = pose.getX() + Math.cos(heading) * ROBOT_RADIUS / 2;
            double y1 = pose.getY() + Math.sin(heading) * ROBOT_RADIUS / 2;
            double x2 = pose.getX() + Math.cos(heading) * ROBOT_RADIUS;
            double y2 = pose.getY() + Math.sin(heading) * ROBOT_RADIUS;

            panelsField.setStyle(robotLook);
            panelsField.moveCursor(x1, y1);
            panelsField.line(x2, y2);

            // Send packet to FTC Control Panels
            panelsField.update();
        } catch (Exception e) {
            // Silently fail to avoid disrupting telemetry
        }
    }
}
