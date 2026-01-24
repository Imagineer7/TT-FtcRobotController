package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.AutoGyroTurret;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IntelMechanumDrive;
import org.firstinspires.ftc.teamcode.util.aurora.Localization;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;

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
 *   - X Button:      Reset turret to forward (0° robot-relative)
 *   - Y Button:      Set turret to default field heading (0° = North)
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

    // Hardware and helpers
    private AuroraHardwareConfig hardware;
    private BasicIndexingHelper indexingHelper;
    private Shooter shooter;
    private BasicFiringHelper firingHelper;
    private IntelMechanumDrive drive;
    private AutoGyroTurret autoGyroTurret;
    private Localization localization;

    // Button state tracking for firing
    private boolean firingButtonPressed = false;

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
        localization = new Localization(hardwareMap);
        if (!localization.isOdometryInitialized()) {
            telemetry.addData("⚠ WARN", "Odometry not available - turret heading may be inaccurate");
        }
        localization.resetPosition();

        // Initialize auto-gyro turret
        autoGyroTurret = new AutoGyroTurret(hardwareMap, telemetry);

        // Enable auto-gyro by default and set to forward (robot heading)
        double initialHeading = 0.0;
        if (localization.isOdometryInitialized()) {
            initialHeading = localization.getHeading(AngleUnit.DEGREES);
        }
        autoGyroTurret.setFieldRelativeHeading(initialHeading, initialHeading);
        autoGyroTurret.enable();

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
        telemetry.addLine("  X: Reset to forward");
        telemetry.addLine("  Y: Set to North (0°)");
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

            // Update localization for robot heading
            if (localization != null) {
                localization.update();
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
                autoGyroTurret.setFieldRelativeHeading(robotHeading, robotHeading);
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

            // X Button - Reset turret to forward (robot-relative 0°)
            if (gamepad1.x && !lastXButton) {
                autoGyroTurret.resetToForward(robotHeading);
                autoGyroTurret.enable();
                telemetry.addLine("⬆️ Turret: Reset to forward");
            }
            lastXButton = gamepad1.x;

            // Y Button - Set turret to default field heading (North = 0°)
            if (gamepad1.y && !lastYButton) {
                autoGyroTurret.setFieldRelativeHeading(0.0, robotHeading);
                autoGyroTurret.enable();
                telemetry.addLine("🧭 Turret: Set to North (0°)");
            }
            lastYButton = gamepad1.y;

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

            // D-Pad fine movement controls (low power for precise positioning)
            final double FINE_MOVE_POWER = 0.5;  // 50% power for precise control

            if (gamepad1.dpad_up) {
                forward = FINE_MOVE_POWER;  // Move forward slowly
            }
            if (gamepad1.dpad_down) {
                forward = -FINE_MOVE_POWER;  // Move backward slowly
            }
            if (gamepad1.dpad_right) {
                strafe = FINE_MOVE_POWER;  // Strafe right slowly
            }
            if (gamepad1.dpad_left) {
                strafe = -FINE_MOVE_POWER;  // Strafe left slowly
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
            // GAMEPAD 2 - FIRING CONTROLS (HOLD TO FIRE)
            // ═══════════════════════════════════════════════════════════════

            // Track if any firing button is pressed
            firingButtonPressed = gamepad2.a || gamepad2.b || gamepad2.y;
            firingHelper.setButtonHeld(firingButtonPressed);

            // Check if turret is busy (moving/settling) - don't fire if busy
            boolean turretBusy = autoGyroTurret.isBusy();

            // Short Range (A button) - hold to fire
            if (gamepad2.a) {
                if (turretBusy) {
                    telemetry.addLine("⏳ Turret busy - waiting to fire...");
                } else if (!firingHelper.isFiring()) {
                    firingHelper.startFiringShortRange();
                    telemetry.addLine("🔥 Starting Short Range firing");
                } else if (firingHelper.isReadyForNextShot()) {
                    firingHelper.startFiringShortRange();
                    telemetry.addLine("🔥 Firing next shot (Short Range)");
                }
            }

            // Mid-Range (B button) - hold to fire
            if (gamepad2.b) {
                if (turretBusy) {
                    telemetry.addLine("⏳ Turret busy - waiting to fire...");
                } else if (!firingHelper.isFiring()) {
                    firingHelper.startFiringMidRange();
                    telemetry.addLine("🔥 Starting Mid Range firing");
                } else if (firingHelper.isReadyForNextShot()) {
                    firingHelper.startFiringMidRange();
                    telemetry.addLine("🔥 Firing next shot (Mid Range)");
                }
            }

            // Long Range (Y button) - hold to fire
            if (gamepad2.y) {
                if (turretBusy) {
                    telemetry.addLine("⏳ Turret busy - waiting to fire...");
                } else if (!firingHelper.isFiring()) {
                    firingHelper.startFiringLongRange();
                    telemetry.addLine("🔥 Starting Long Range firing");
                } else if (firingHelper.isReadyForNextShot()) {
                    firingHelper.startFiringLongRange();
                    telemetry.addLine("🔥 Firing next shot (Long Range)");
                }
            }

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
                // DPad Up - Manual transfer front to center (hold button)
                indexingHelper.transferFrontIntakeToCenterManual(gamepad2.dpad_up);

                // DPad Down - Manual transfer back to center (hold button)
                indexingHelper.transferBackIntakeToCenterManual(gamepad2.dpad_down);
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
            telemetry.addData("Button Held", firingButtonPressed ? "YES" : "NO");
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

            telemetry.addData("═══ ACTIVE CONTROLS ═══", "");
            // Gamepad 1 - Turret
            if (gamepad1.a) telemetry.addLine("🎯 GP1 A - Set Robot Heading");
            if (gamepad1.b) telemetry.addLine("🔄 GP1 B - Toggle Auto-Gyro");
            if (gamepad1.x) telemetry.addLine("⬆️ GP1 X - Reset Forward");
            if (gamepad1.y) telemetry.addLine("🧭 GP1 Y - Set North");
            // Gamepad 1 - Drive
            if (gamepad1.guide) telemetry.addLine("🔘 GP1 Guide - Toggle Ejection");
            if (gamepad1.dpad_up) telemetry.addLine("⬆️ GP1 DPad Up - Fine Fwd");
            if (gamepad1.dpad_down) telemetry.addLine("⬇️ GP1 DPad Down - Fine Back");
            if (gamepad1.dpad_left) telemetry.addLine("⬅️ GP1 DPad Left - Fine Strafe L");
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
}
