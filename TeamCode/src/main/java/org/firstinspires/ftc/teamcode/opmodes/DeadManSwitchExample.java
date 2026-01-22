package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.*;

/**
 * DeadManSwitchExample - Demonstrates dead-man switch control for firing
 *
 * This example shows how to implement a dead-man switch where:
 * - Holding right trigger starts firing and keeps it active
 * - Releasing right trigger immediately stops firing
 * - While holding, shots fire automatically when ready
 *
 * This is the recommended pattern for safe, responsive control.
 *
 * Controls:
 * - Right Trigger: Hold to fire (dead-man switch)
 * - Left Trigger: Hold to fire at low RPM
 * - A Button: Toggle rate limiting on/off
 * - B Button: Emergency stop (cancel all)
 * - Y Button: Start ejection
 * - X Button: Stop ejection
 */
@TeleOp(name="Dead-Man Switch Example", group="Testing")
public class DeadManSwitchExample extends LinearOpMode {

    // Hardware and subsystems
    private AuroraHardwareConfig hardware;
    private Shooter shooter;
    private BasicIndexingHelper indexingHelper;
    private BasicFiringHelper firingHelper;

    // Dead-man switch state tracking
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;

    // Rate limiting
    private boolean rateLimitEnabled = true;
    private long lastShotTime = 0;
    private static final long SHOT_DELAY_MS = 500; // 500ms between shots

    // Shot counting
    private int totalShotsFired = 0;
    private int shotsThisHold = 0;

    // Button tracking for other controls
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastX = false;

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
        telemetry.addData("Dead-Man Switch Mode", "ACTIVE");
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("  RT", "Hold to fire (high RPM)");
        telemetry.addData("  LT", "Hold to fire (low RPM)");
        telemetry.addData("  A", "Toggle rate limiting");
        telemetry.addData("  B", "Emergency stop");
        telemetry.addData("  Y", "Start ejection");
        telemetry.addData("  X", "Stop ejection");
        telemetry.update();

        waitForStart();

        // ═══════════════════════════════════════════════════════════════════════
        // MAIN LOOP
        // ═══════════════════════════════════════════════════════════════════════

        while (opModeIsActive()) {
            // CRITICAL: Update all subsystems every loop
            shooter.update();
            indexingHelper.update();
            firingHelper.update();

            // ───────────────────────────────────────────────────────────────────
            // DEAD-MAN SWITCH CONTROL
            // ───────────────────────────────────────────────────────────────────

            handleDeadManSwitch();

            // ───────────────────────────────────────────────────────────────────
            // OTHER CONTROLS
            // ───────────────────────────────────────────────────────────────────

            // A: Toggle rate limiting
            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                rateLimitEnabled = !rateLimitEnabled;
                telemetry.addData("Action", "Rate limiting: " + (rateLimitEnabled ? "ON" : "OFF"));
            }
            lastA = currentA;

            // B: Emergency stop
            boolean currentB = gamepad1.b;
            if (currentB && !lastB) {
                firingHelper.cancelFiring();
                totalShotsFired = 0;
                shotsThisHold = 0;
                telemetry.addData("Action", "🚨 EMERGENCY STOP");
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

            // X: Stop ejection
            boolean currentX = gamepad1.x;
            if (currentX && !lastX) {
                firingHelper.stopEjection();
                telemetry.addData("Action", "✅ Stopped ejection");
            }
            lastX = currentX;

            // ───────────────────────────────────────────────────────────────────
            // TELEMETRY
            // ───────────────────────────────────────────────────────────────────

            displayTelemetry();
        }

        // ═══════════════════════════════════════════════════════════════════════
        // CLEANUP
        // ═══════════════════════════════════════════════════════════════════════

        firingHelper.stopAll();
        shooter.disable();
    }

    /**
     * Handle dead-man switch logic for both triggers
     */
    private void handleDeadManSwitch() {
        // Right trigger = high RPM
        boolean currentRightTrigger = gamepad1.right_trigger > 0.5;

        // Left trigger = low RPM
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.5;

        // ─── Right Trigger (High RPM) ───
        if (currentRightTrigger && !lastRightTrigger) {
            // Just pressed - start firing at high RPM
            boolean success = firingHelper.startFiring(3200, "HIGH-DEAD-MAN", true);
            if (success) {
                shotsThisHold = 0;
                telemetry.addData("Dead-Man", "✅ HIGH RPM Started");
            }
        } else if (!currentRightTrigger && lastRightTrigger) {
            // Just released - STOP (dead-man switch activated)
            if (firingHelper.isFiring()) {
                firingHelper.cancelFiring();
                telemetry.addData("Dead-Man", "❌ Stopped (released)");
                telemetry.addData("  Shots This Hold", shotsThisHold);
            }
        }

        // ─── Left Trigger (Low RPM) ───
        if (currentLeftTrigger && !lastLeftTrigger) {
            // Just pressed - start firing at low RPM
            boolean success = firingHelper.startFiring(2000, "LOW-DEAD-MAN", true);
            if (success) {
                shotsThisHold = 0;
                telemetry.addData("Dead-Man", "✅ LOW RPM Started");
            }
        } else if (!currentLeftTrigger && lastLeftTrigger) {
            // Just released - STOP (dead-man switch activated)
            if (firingHelper.isFiring()) {
                firingHelper.cancelFiring();
                telemetry.addData("Dead-Man", "❌ Stopped (released)");
                telemetry.addData("  Shots This Hold", shotsThisHold);
            }
        }

        // ─── Auto-Fire While Held ───
        if ((currentRightTrigger || currentLeftTrigger) && firingHelper.isReadyForNextShot()) {
            boolean shouldFire = true;

            // Apply rate limiting if enabled
            if (rateLimitEnabled) {
                long now = System.currentTimeMillis();
                if (now - lastShotTime < SHOT_DELAY_MS) {
                    shouldFire = false;
                } else {
                    lastShotTime = now;
                }
            }

            if (shouldFire) {
                boolean success = firingHelper.fireShot();
                if (success) {
                    totalShotsFired++;
                    shotsThisHold++;
                }
            }
        }

        // Update state
        lastRightTrigger = currentRightTrigger;
        lastLeftTrigger = currentLeftTrigger;
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry() {
        telemetry.addData("═══════════════════════════", "");
        telemetry.addData("Dead-Man Switch Mode", "ACTIVE");
        telemetry.addData("", "");

        // Dead-man switch status
        boolean rightHeld = gamepad1.right_trigger > 0.5;
        boolean leftHeld = gamepad1.left_trigger > 0.5;

        if (rightHeld) {
            telemetry.addData("🔥 RIGHT TRIGGER", "HELD (HIGH RPM)");
        } else if (leftHeld) {
            telemetry.addData("🔥 LEFT TRIGGER", "HELD (LOW RPM)");
        } else {
            telemetry.addData("🔒 TRIGGERS", "Released (safe)");
        }

        // Firing status
        telemetry.addData("", "");
        telemetry.addData("Firing Active", firingHelper.isFiring() ? "YES" : "NO");
        if (firingHelper.isFiring()) {
            telemetry.addData("  State", firingHelper.getFiringState());
            telemetry.addData("  Preset", firingHelper.getPresetName());
            telemetry.addData("  Target RPM", String.format("%.0f", firingHelper.getTargetRPM()));
        }

        // Ready status
        if (firingHelper.isReadyForNextShot()) {
            telemetry.addData("🎯 STATUS", "READY TO FIRE");
        }

        // Shot counter
        telemetry.addData("", "");
        telemetry.addData("Shots This Hold", shotsThisHold);
        telemetry.addData("Total Shots Fired", totalShotsFired);

        // Rate limiting
        telemetry.addData("", "");
        telemetry.addData("Rate Limiting", rateLimitEnabled ? "ON (" + SHOT_DELAY_MS + "ms)" : "OFF");
        if (rateLimitEnabled) {
            long now = System.currentTimeMillis();
            long timeSinceLastShot = now - lastShotTime;
            if (timeSinceLastShot < SHOT_DELAY_MS) {
                long timeRemaining = SHOT_DELAY_MS - timeSinceLastShot;
                telemetry.addData("  Next Shot In", timeRemaining + "ms");
            }
        }

        // Shooter status
        telemetry.addData("", "");
        telemetry.addData("Shooter RPM", String.format("%.0f", shooter.getCurrentRPM()));
        telemetry.addData("Shooter State", shooter.getState());

        // Ejection status
        if (firingHelper.isEjecting()) {
            telemetry.addData("", "");
            telemetry.addData("Ejection", "ACTIVE");
        }

        // Controls reminder
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("  RT", rightHeld ? "HELD ⚡" : "Hold to fire (high)");
        telemetry.addData("  LT", leftHeld ? "HELD ⚡" : "Hold to fire (low)");
        telemetry.addData("  A", "Toggle rate limit");
        telemetry.addData("  B", "Emergency stop");

        telemetry.update();
    }
}
