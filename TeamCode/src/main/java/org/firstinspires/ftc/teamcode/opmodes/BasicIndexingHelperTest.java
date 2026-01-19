package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;

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
@TeleOp(name="Basic Indexing Helper Test", group="Testing")
public class BasicIndexingHelperTest extends LinearOpMode {

    // Hardware and helper
    private AuroraHardwareConfig hardware;
    private BasicIndexingHelper indexingHelper;

    // Button edge detection
    private boolean lastA1 = false;
    private boolean lastB1 = false;
    private boolean lastX1 = false;
    private boolean lastY1 = false;
    private boolean lastA2 = false;
    private boolean lastB2 = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        // Create helper
        indexingHelper = new BasicIndexingHelper(hardware, telemetry);

        telemetry.addLine("✅ Initialization complete!");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // CRITICAL: Update helper every loop
            indexingHelper.update();

            // ═══════════════════════════════════════════════════════════
            // GAMEPAD 1 - Intake and Transfer Controls
            // ═══════════════════════════════════════════════════════════

            // Manual transfer sequences (hold button) - HIGHEST PRIORITY
            indexingHelper.transferFrontIntakeToCenterManual(gamepad1.a);
            indexingHelper.transferBackIntakeToCenterManual(gamepad1.b);

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

            // Simple intake control (no transfer) - ONLY if no transfer active
            if (!indexingHelper.isTransferActive()) {
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

            // Pre-positioning controls
            if (gamepad1.dpad_up) {
                indexingHelper.prePositionArtifacts();
            }
            if (gamepad1.dpad_down) {
                indexingHelper.unPrePositionArtifacts();
            }

            // ═══════════════════════════════════════════════════════════
            // GAMEPAD 2 - Uptake and Injector Controls
            // ═══════════════════════════════════════════════════════════

            // Uptake manual control - ONLY if no timed movement or transfer active
            if (!indexingHelper.isUptakeBusy() && !indexingHelper.isTransferActive()) {
                if (gamepad2.dpad_up) {
                    indexingHelper.setUptakePower(1.0);
                } else if (gamepad2.dpad_down) {
                    indexingHelper.setUptakePower(-1.0);
                } else {
                    indexingHelper.setUptakePower(0);
                }
            }

            // Injector manual control - ONLY if no timed movement or transfer active
            if (!indexingHelper.isInjectorBusy() && !indexingHelper.isTransferActive()) {
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

            // Timed movements (edge detection)
            boolean currentA2 = gamepad2.a;
            if (currentA2 && !lastA2) {
                indexingHelper.setUptakeTimed(1.0, 500);
                telemetry.addLine("▶️ Uptake timed (500ms)");
            }
            lastA2 = currentA2;

            boolean currentB2 = gamepad2.b;
            if (currentB2 && !lastB2) {
                indexingHelper.setInjectorTimed(1.0, 500);
                telemetry.addLine("▶️ Injector timed (500ms)");
            }
            lastB2 = currentB2;

            // Cancel transfer (START button)
            if (gamepad2.start) {
                indexingHelper.cancelTransfer();
                telemetry.addLine("❌ Transfer cancelled");
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
            telemetry.addData("", "");

            telemetry.addData("═══ GAMEPAD 2 - UPTAKE/INJECTOR ═══", "");
            telemetry.addData("DPad Up/Down", "Uptake ↑/↓");
            telemetry.addData("L Bumper", "Both Injectors →");
            telemetry.addData("L Trigger", "Both Injectors ←");
            telemetry.addData("R Bumper", "Injector Right → (Individual)");
            telemetry.addData("R Trigger", "Injector Left ← (Individual)");
            telemetry.addData("A", "Uptake Timed (500ms)");
            telemetry.addData("B", "Both Injectors Timed (500ms)");
            telemetry.addData("START", "❌ Cancel Transfer");
            telemetry.addData("", "");

            // Transfer status
            telemetry.addData("═══ TRANSFER STATUS ═══", "");
            telemetry.addData("Active", indexingHelper.isTransferActive() ? "YES" : "NO");
            telemetry.addData("State", indexingHelper.getTransferState());
            telemetry.addData("Type", indexingHelper.getTransferType());
            telemetry.addData("", "");

            // Status telemetry
            indexingHelper.addTelemetry();

            telemetry.update();
        }

        // Stop everything on exit
        indexingHelper.stopAll();
    }
}
