package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.*;

/**
 * BasicFiringHelperExample - Demonstrates the correct usage of BasicFiringHelper
 *
 * This example shows:
 * 1. How to start firing with keep-alive mode
 * 2. How to fire multiple shots while shooter stays spinning
 * 3. How to stop the shooter
 *
 * Controls:
 * - Gamepad1 X (HOLD): Hold to spin up and keep shooter running, release to stop
 * - Gamepad1 A: Fire a shot (when ready)
 * - Gamepad1 B: Stop firing immediately (emergency stop)
 * - Gamepad1 Y: Start ejection
 * - Gamepad1 Right Bumper: Stop ejection
 */
@TeleOp(name="Basic Firing Helper Example", group="Testing")
@Disabled
public class BasicFiringHelperExample extends LinearOpMode {

    // Hardware and subsystems
    private AuroraHardwareConfig hardware;
    private Shooter shooter;
    private BasicIndexingHelper indexingHelper;
    private BasicFiringHelper firingHelper;

    // Button state tracking (for edge detection)
    private boolean lastX = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        // ═══════════════════════════════════════════════════════════════════════
        // INITIALIZATION
        // ═══════════════════════════════════════════════════════════════════════

        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        // Initialize shooter
        ShooterConfig shooterConfig = new ShooterConfig();
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        shooter.enable();

        // Initialize indexing helper
        indexingHelper = new BasicIndexingHelper(hardware, telemetry);

        // Initialize firing helper
        firingHelper = new BasicFiringHelper(shooter, indexingHelper, hardware, telemetry);
        firingHelper.setEnabled(true);

        telemetry.addData("Status", "✅ Ready to start");
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("  X (HOLD)", "Hold to spin up, release to stop");
        telemetry.addData("  A", "Fire shot (when ready)");
        telemetry.addData("  B", "Emergency stop");
        telemetry.addData("  Y", "Start ejection");
        telemetry.addData("  RB", "Stop ejection");
        telemetry.update();

        waitForStart();

        // ═══════════════════════════════════════════════════════════════════════
        // MAIN LOOP
        // ═══════════════════════════════════════════════════════════════════════

        while (opModeIsActive()) {
            // CRITICAL: Update all subsystems every loop
            // NOTE: shooter.update() is called inside firingHelper.update() - don't call twice!
            indexingHelper.update();
            firingHelper.update();

            // ───────────────────────────────────────────────────────────────────
            // BUTTON CONTROLS (with edge detection)
            // ───────────────────────────────────────────────────────────────────

            // X: Hold-to-fire mode (hold to keep spinning, release to stop)
            boolean currentX = gamepad1.x;

            // Pressed: Start keep-alive mode
            if (currentX && !lastX) {
                // Start firing with keep-alive mode enabled
                // This will spin up, fire once, then wait in READY_TO_FIRE state
                boolean success = firingHelper.startFiring(
                    ShooterConfig.ShooterPreset.MID_RANGE.getTargetRPM(),
                    ShooterConfig.ShooterPreset.MID_RANGE.getName(),
                    true  // Keep-alive mode
                );

                if (success) {
                    telemetry.addData("Action", "✅ Starting keep-alive mode");
                } else {
                    telemetry.addData("Action", "❌ Failed to start (already active?)");
                }
            }

            // Released: Stop firing
            if (!currentX && lastX) {
                if (firingHelper.isFiring()) {
                    firingHelper.cancelFiring();
                    telemetry.addData("Action", "✅ Stopped (X released)");
                }
            }

            lastX = currentX;

            // A: Fire a shot (only works when in READY_TO_FIRE state)
            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                if (firingHelper.isReadyForNextShot()) {
                    // Shooter is spun up and ready - fire a shot
                    boolean success = firingHelper.fireShot();

                    if (success) {
                        telemetry.addData("Action", "✅ Firing shot");
                    } else {
                        telemetry.addData("Action", "❌ Failed to fire");
                    }
                } else {
                    telemetry.addData("Action", "⚠️ Not ready yet - wait for spinup");
                }
            }
            lastA = currentA;

            // B: Stop firing (cancel keep-alive mode)
            boolean currentB = gamepad1.b;
            if (currentB && !lastB) {
                firingHelper.cancelFiring();
                telemetry.addData("Action", "✅ Stopped firing");
            }
            lastB = currentB;

            // Y: Start ejection
            boolean currentY = gamepad1.y;
            if (currentY && !lastY) {
                boolean success = firingHelper.startEjection();
                if (success) {
                    telemetry.addData("Action", "✅ Starting ejection");
                } else {
                    telemetry.addData("Action", "❌ Failed to start ejection");
                }
            }
            lastY = currentY;

            // Right Bumper: Stop ejection
            boolean currentRB = gamepad1.right_bumper;
            if (currentRB && !lastRightBumper) {
                firingHelper.stopEjection();
                telemetry.addData("Action", "✅ Stopped ejection");
            }
            lastRightBumper = currentRB;

            // ───────────────────────────────────────────────────────────────────
            // TELEMETRY
            // ───────────────────────────────────────────────────────────────────

            telemetry.addData("═══════════════════════════", "");
            telemetry.addData("Status", "Running");
            telemetry.addData("", "");

            // Firing status
            telemetry.addData("Firing Active", firingHelper.isFiring() ? "YES" : "NO");
            if (firingHelper.isFiring()) {
                telemetry.addData("  State", firingHelper.getFiringState());
                telemetry.addData("  Preset", firingHelper.getPresetName());
                telemetry.addData("  Target RPM", String.format("%.0f", firingHelper.getTargetRPM()));
            }

            // Ready status
            if (firingHelper.isReadyForNextShot()) {
                telemetry.addData("🎯 READY", "Press A to fire!");
            }

            // Shooter status
            telemetry.addData("", "");
            telemetry.addData("Shooter RPM", String.format("%.0f", shooter.getCurrentRPM()));
            telemetry.addData("Shooter State", shooter.getState());

            // Ejection status
            telemetry.addData("", "");
            telemetry.addData("Ejection Active", firingHelper.isEjecting() ? "YES" : "NO");

            // Controls reminder
            telemetry.addData("", "");
            telemetry.addData("Controls", "");
            telemetry.addData("  X (HOLD)", "Hold to spin, release to stop");
            telemetry.addData("  A", "Fire shot");
            telemetry.addData("  B", "Emergency stop");
            telemetry.addData("  Y", "Start ejection");
            telemetry.addData("  RB", "Stop ejection");

            telemetry.update();
        }

        // ═══════════════════════════════════════════════════════════════════════
        // CLEANUP
        // ═══════════════════════════════════════════════════════════════════════

        firingHelper.stopAll();
        shooter.disable();
    }
}
