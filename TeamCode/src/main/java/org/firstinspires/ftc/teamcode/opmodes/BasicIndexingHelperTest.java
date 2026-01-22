package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;

/**
 * BasicIndexingHelperTest - Test OpMode for BasicIndexingHelper
 *
 * This OpMode demonstrates how to use the BasicIndexingHelper class for manual
 * control of indexing system motors and servos, including complex transfer sequences.
 *
 * Controls:
 *   GAMEPAD 1:
 *   - Left Bumper:  Run front intake (continuous)
 *   - Right Bumper: Run back intake (continuous)
 *   - A: Transfer front intake to center (MANUAL - hold button)
 *   - B: Transfer back intake to center (MANUAL - hold button)
 *   - X: Transfer front intake to center (TIMED - 2.5s auto)
 *   - Y: Transfer back intake to center (TIMED - 2.5s auto)
 *   - DPad Up: Pre-position artifacts (400ms)
 *   - DPad Down: Un-pre-position artifacts (500ms)
 *
 *   GAMEPAD 2:
 *   - DPad Up:   Uptake servos forward (manual)
 *   - DPad Down: Uptake servos reverse (manual)
 *   - Left Bumper:  Injector servos forward
 *   - Left Trigger:  Injector servos reverse
 *   - Right Bumper: Individual injector right forward
 *   - Right Trigger: Individual injector left reverse
 *   - A: Run uptake timed (500ms)
 *   - B: Run both injectors timed (500ms)
 *   - START: Cancel any active transfer sequence
 */
@TeleOp(name="Basic Indexing & Firing Test", group="Testing")
public class BasicIndexingHelperTest extends LinearOpMode {

    // Hardware and helpers
    private AuroraHardwareConfig hardware;
    private BasicIndexingHelper indexingHelper;
    private Shooter shooter;
    private BasicFiringHelper firingHelper;

    // Button edge detection
    private boolean lastX1 = false;
    private boolean lastY1 = false;
    private boolean lastA2 = false;
    private boolean lastB2 = false;
    private boolean lastY2 = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

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

        telemetry.addLine("✅ Initialization complete!");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // CRITICAL: Update all helpers every loop
            // NOTE: shooter.update() is called inside firingHelper.update() - don't call twice!
            indexingHelper.update();
            firingHelper.update();

            // ═══════════════════════════════════════════════════════════
            // GAMEPAD 1 - Intake and Transfer Controls
            // ═══════════════════════════════════════════════════════════

            // Manual transfer sequences (hold button) - HIGHEST PRIORITY
            // Don't run during ejection
            if (!firingHelper.isEjecting()) {
                indexingHelper.transferFrontIntakeToCenterManual(gamepad1.a);
                indexingHelper.transferBackIntakeToCenterManual(gamepad1.b);
            }

            // Timed transfer sequences (edge detection) - HIGH PRIORITY
            boolean currentX1 = gamepad1.x;
            if (currentX1 && !lastX1) {
                indexingHelper.transferFrontIntakeToCenterTimed();
                telemetry.addLine("▶️ Front transfer timed (2.5s)");
            }
            lastX1 = currentX1;

            boolean currentY1 = gamepad1.y;
            if (currentY1 && !lastY1) {
                indexingHelper.transferBackIntakeToCenterTimed();
                telemetry.addLine("▶️ Back transfer timed (2.5s)");
            }
            lastY1 = currentY1;

            // Simple intake control (no transfer or ejection) - ONLY if no transfer/ejection active
            if (!indexingHelper.isTransferActive() && !firingHelper.isEjecting()) {
                if (gamepad1.left_bumper) {
                    indexingHelper.runFrontIntake(true, 1.0);
                } else {
                    indexingHelper.runFrontIntake(false, 0);
                }

                if (gamepad1.right_bumper) {
                    indexingHelper.runBackIntake(true, 1.0);
                } else {
                    indexingHelper.runBackIntake(false, 0);
                }
            }

            // Pre-positioning controls - don't run during ejection
            if (!firingHelper.isEjecting()) {
                if (gamepad1.dpad_up) {
                    indexingHelper.prePositionArtifacts();
                }
                if (gamepad1.dpad_down) {
                    indexingHelper.unPrePositionArtifacts();
                }
            }

            // Ejection controls
            if (gamepad1.dpad_left) {
                firingHelper.startEjection();
            }
            if (gamepad1.dpad_right) {
                firingHelper.stopEjection();
            }

            // ═══════════════════════════════════════════════════════════
            // GAMEPAD 2 - Manual Uptake and Injector Controls
            // ═══════════════════════════════════════════════════════════

            // Uptake manual control - ONLY if no timed movement, transfer, firing, or ejection active
            if (!indexingHelper.isUptakeBusy() && !indexingHelper.isTransferActive() && !firingHelper.isFiring() && !firingHelper.isEjecting()) {
                if (gamepad2.dpad_up) {
                    indexingHelper.setUptakePower(1.0);
                } else if (gamepad2.dpad_down) {
                    indexingHelper.setUptakePower(-1.0);
                } else {
                    indexingHelper.setUptakePower(0);
                }
            }

            // Injector manual control - ONLY if no timed movement, transfer, firing, or ejection active
            if (!indexingHelper.isInjectorBusy() && !indexingHelper.isTransferActive() && !firingHelper.isFiring() && !firingHelper.isEjecting()) {
                if (gamepad2.left_bumper) {
                    indexingHelper.setInjectorPower(1.0);  // Both servos forward
                } else if (gamepad2.left_trigger > 0.5) {
                    indexingHelper.setInjectorPower(-1.0);  // Both servos reverse
                } else if (gamepad2.right_bumper) {
                    indexingHelper.setInjectorRightPower(1.0);  // Individual control
                } else if (gamepad2.right_trigger > 0.5) {
                    indexingHelper.setInjectorLeftPower(-1.0);  // Individual control
                } else {
                    indexingHelper.setInjectorPower(0);  // Stop both
                }
            }

            // Firing with presets - hold button to fire and keep shooter spinning
            // Update button state for automatic stop when released
            boolean firingButtonPressed = gamepad2.a || gamepad2.b || gamepad2.y;
            firingHelper.setButtonHeld(firingButtonPressed);

            // Short Range (A button) - hold to fire
            if (gamepad2.a) {
                if (!firingHelper.isFiring()) {
                    telemetry.addLine(">>> Starting Short Range firing");
                    firingHelper.startFiringShortRange();
                } else if (firingHelper.isReadyForNextShot()) {
                    telemetry.addLine(">>> Firing next shot (Short Range)");
                    firingHelper.startFiringShortRange();
                }
            }

            // Mid-Range (B button) - hold to fire
            if (gamepad2.b) {
                if (!firingHelper.isFiring()) {
                    telemetry.addLine(">>> Starting Mid Range firing");
                    firingHelper.startFiringMidRange();
                } else if (firingHelper.isReadyForNextShot()) {
                    telemetry.addLine(">>> Firing next shot (Mid Range)");
                    firingHelper.startFiringMidRange();
                }
            }

            // Long Range (Y button) - hold to fire
            if (gamepad2.y) {
                if (!firingHelper.isFiring()) {
                    telemetry.addLine(">>> Starting Long Range firing");
                    firingHelper.startFiringLongRange();
                } else if (firingHelper.isReadyForNextShot()) {
                    telemetry.addLine(">>> Firing next shot (Long Range)");
                    firingHelper.startFiringLongRange();
                }
            }
            // Note: Shooter will stop automatically when button released (handled in update())

            // Manual shooter stop
            if (gamepad2.x) {
                shooter.stopMotors();
                firingHelper.cancelFiring();
            }

            // Cancel firing with START
            if (gamepad2.start) {
                firingHelper.cancelFiring();
            }

            // Emergency stop all with BACK
            if (gamepad2.back) {
                firingHelper.stopAll();
                indexingHelper.stopAll();
                shooter.stopMotors();
                telemetry.addLine("🛑 EMERGENCY STOP");
            }

            // ═══════════════════════════════════════════════════════════
            // TELEMETRY
            // ═══════════════════════════════════════════════════════════

            telemetry.addData("═══ GAMEPAD 1 - INTAKES & TRANSFER ═══", "");
            telemetry.addData("L/R Bumper", "Front/Back Intake Only");
            telemetry.addData("A (Hold)", "Front → Center (Manual)");
            telemetry.addData("B (Hold)", "Back → Center (Manual)");
            telemetry.addData("X (Press)", "Front → Center (Auto 2.5s)");
            telemetry.addData("Y (Press)", "Back → Center (Auto 2.5s)");
            telemetry.addData("DPad Up", "Pre-position Artifacts");
            telemetry.addData("DPad Down", "Un-pre-position Artifacts");
            telemetry.addData("DPad Left", "▶️ Start Ejection");
            telemetry.addData("DPad Right", "⏹️ Stop Ejection");
            telemetry.addData("", "");

            telemetry.addData("═══ GAMEPAD 2 - FIRING ═══", "");
            telemetry.addData("A (Hold)", "🔥 Fire Long Range (3200 RPM)");
            telemetry.addData("B (Hold)", "🔥 Fire Mid Range (2800 RPM)");
            telemetry.addData("Y (Hold)", "🔥 Fire Short Range (2400 RPM)");
            telemetry.addData("X", "⏹️ Stop Shooter");
            telemetry.addData("START", "❌ Cancel Firing");
            telemetry.addData("BACK", "🛑 EMERGENCY STOP");
            telemetry.addData("", "");
            telemetry.addData("DPad Up/Down", "Uptake Manual (when free)");
            telemetry.addData("Bumpers/Triggers", "Injector Manual (when free)");
            telemetry.addData("", "");

            // Transfer status
            telemetry.addData("═══ TRANSFER STATUS ═══", "");
            telemetry.addData("Active", indexingHelper.isTransferActive() ? "YES" : "NO");
            telemetry.addData("State", indexingHelper.getTransferState());
            telemetry.addData("Type", indexingHelper.getTransferType());
            telemetry.addData("", "");

            // DEBUG: Button press detection
            telemetry.addData("═══ DEBUG - BUTTON PRESS ═══", "");
            telemetry.addData("GP2.A (Short)", gamepad2.a ? "PRESSED" : "not pressed");
            telemetry.addData("GP2.B (Mid)", gamepad2.b ? "PRESSED" : "not pressed");
            telemetry.addData("GP2.Y (Long)", gamepad2.y ? "PRESSED" : "not pressed");
            telemetry.addData("", "");

            // Firing status
            telemetry.addData("═══ FIRING STATUS ═══", "");
            telemetry.addData("Active", firingHelper.isFiring() ? "YES ✅" : "NO");
            telemetry.addData("Helper Enabled", firingHelper.isEnabled() ? "YES" : "NO");
            if (firingHelper.isFiring()) {
                telemetry.addData("State", firingHelper.getFiringState());
                telemetry.addData("Preset", firingHelper.getPresetName());
                telemetry.addData("Target RPM", String.format("%.0f", firingHelper.getTargetRPM()));
                telemetry.addData("Ready for Next", firingHelper.isReadyForNextShot() ? "YES ✅" : "NO");
            }
            telemetry.addData("Ejection", firingHelper.isEjecting() ? "ACTIVE ⚠️" : "IDLE");
            telemetry.addData("Button Held", firingButtonPressed ? "YES" : "NO");
            telemetry.addData("", "");

            // Shooter status
            telemetry.addData("═══ SHOOTER ═══", "");
            telemetry.addData("Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
            telemetry.addData("Target RPM", String.format("%.0f", shooter.getTargetRPM()));
            telemetry.addData("Ready", shooter.isReadyToFire() ? "YES ✅" : "NO");
            telemetry.addData("State", shooter.getState());
            telemetry.addData("Enabled", shooter.isEnabled() ? "YES" : "NO");
            telemetry.addData("", "");

            // Hardware status
            indexingHelper.addTelemetry();

            telemetry.update();
        }

        // Stop everything on exit
        firingHelper.stopAll();
        shooter.stopMotors();
        indexingHelper.stopAll();
    }
}
