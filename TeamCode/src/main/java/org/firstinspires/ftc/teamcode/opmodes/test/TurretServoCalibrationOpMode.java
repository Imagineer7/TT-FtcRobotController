package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret Servo Calibration OpMode
 *
 * Use this OpMode to calibrate the servo travel tuner for precise 360° turret rotation.
 *
 * INSTRUCTIONS:
 * 1. Start with servo travel tuner at default settings
 * 2. Run this OpMode and test positions 0.0 to 1.0
 * 3. Use servo travel tuner to adjust until:
 *    - Position 0.0 = turret at -180° (or left limit)
 *    - Position 0.5 = turret at 0° (center/forward)
 *    - Position 1.0 = turret at +180° (or right limit)
 *    - Total rotation = exactly 360°
 * 4. Once calibrated, update SERVO_MAX_POSITION_360_TUNED constant in Turret.java
 *
 * CONTROLS:
 * Gamepad 1:
 * - DPAD UP:    Increase position by 0.1
 * - DPAD DOWN:  Decrease position by 0.1
 * - DPAD RIGHT: Increase position by 0.01
 * - DPAD LEFT:  Decrease position by 0.01
 * - A:          Set to 0.0 (minimum)
 * - B:          Set to 0.5 (center)
 * - Y:          Set to 1.0 (maximum)
 * - X:          Reset to last saved position
 * - LEFT BUMPER: Save current position
 * - RIGHT BUMPER: Test sequence (0.0 → 0.25 → 0.5 → 0.75 → 1.0)
 * - START:      Emergency stop (disable servo)
 */
@TeleOp(name="Turret Servo Calibration", group="Testing")
@Disabled
public class TurretServoCalibrationOpMode extends LinearOpMode {

    // Hardware
    private Servo turretServo;

    // State
    private double currentPosition = 0.5;
    private double savedPosition = 0.5;
    private boolean isEnabled = true;
    private boolean isTestSequenceRunning = false;

    // Button edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastStart = false;

    // Constants
    private static final String TURRET_SERVO_NAME = "TurretLeft";
    private static final double POSITION_STEP_LARGE = 0.1;
    private static final double POSITION_STEP_SMALL = 0.01;

    @Override
    public void runOpMode() {
        // Initialize hardware
        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        try {
            turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
            telemetry.addData("Servo", "✓ Found: " + TURRET_SERVO_NAME);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Servo not found: " + TURRET_SERVO_NAME);
            telemetry.addData("Exception", e.getMessage());
            telemetry.update();
            return;
        }

        // Set to center position
        turretServo.setPosition(currentPosition);

        telemetry.addData("Status", "✓ Ready");
        telemetry.addData("", "");
        telemetry.addData("Instructions", "Use DPAD and buttons to control");
        telemetry.addData("", "See OpMode header for full controls");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Handle input
            handleInput();

            // Update servo
            if (isEnabled && !isTestSequenceRunning) {
                turretServo.setPosition(currentPosition);
            }

            // Display telemetry
            displayTelemetry();

            // Small delay for stability
            sleep(50);
        }

        // Cleanup
        if (turretServo != null) {
            turretServo.setPosition(0.5); // Return to center
        }
    }

    /**
     * Handle gamepad input
     */
    private void handleInput() {
        // DPAD UP - Increase by 0.1
        if (gamepad1.dpad_up && !lastDpadUp) {
            currentPosition = clamp(currentPosition + POSITION_STEP_LARGE);
        }
        lastDpadUp = gamepad1.dpad_up;

        // DPAD DOWN - Decrease by 0.1
        if (gamepad1.dpad_down && !lastDpadDown) {
            currentPosition = clamp(currentPosition - POSITION_STEP_LARGE);
        }
        lastDpadDown = gamepad1.dpad_down;

        // DPAD RIGHT - Increase by 0.01
        if (gamepad1.dpad_right && !lastDpadRight) {
            currentPosition = clamp(currentPosition + POSITION_STEP_SMALL);
        }
        lastDpadRight = gamepad1.dpad_right;

        // DPAD LEFT - Decrease by 0.01
        if (gamepad1.dpad_left && !lastDpadLeft) {
            currentPosition = clamp(currentPosition - POSITION_STEP_SMALL);
        }
        lastDpadLeft = gamepad1.dpad_left;

        // A - Set to minimum (0.0)
        if (gamepad1.a && !lastA) {
            currentPosition = 0.0;
        }
        lastA = gamepad1.a;

        // B - Set to center (0.5)
        if (gamepad1.b && !lastB) {
            currentPosition = 0.5;
        }
        lastB = gamepad1.b;

        // Y - Set to maximum (1.0)
        if (gamepad1.y && !lastY) {
            currentPosition = 1.0;
        }
        lastY = gamepad1.y;

        // X - Restore saved position
        if (gamepad1.x && !lastX) {
            currentPosition = savedPosition;
        }
        lastX = gamepad1.x;

        // LEFT BUMPER - Save current position
        if (gamepad1.left_bumper && !lastLeftBumper) {
            savedPosition = currentPosition;
        }
        lastLeftBumper = gamepad1.left_bumper;

        // RIGHT BUMPER - Test sequence
        if (gamepad1.right_bumper && !lastRightBumper) {
            runTestSequence();
        }
        lastRightBumper = gamepad1.right_bumper;

        // START - Emergency stop
        if (gamepad1.start && !lastStart) {
            isEnabled = !isEnabled;
        }
        lastStart = gamepad1.start;
    }

    /**
     * Run automatic test sequence
     */
    private void runTestSequence() {
        isTestSequenceRunning = true;

        double[] testPositions = {0.0, 0.25, 0.5, 0.75, 1.0, 0.5};
        int[] delayMs = {2000, 1000, 1000, 1000, 1000, 1000};

        for (int i = 0; i < testPositions.length && opModeIsActive(); i++) {
            currentPosition = testPositions[i];
            turretServo.setPosition(currentPosition);

            long startTime = System.currentTimeMillis();
            while (opModeIsActive() &&
                   (System.currentTimeMillis() - startTime) < delayMs[i]) {
                displayTelemetry();
                telemetry.addData("", "");
                telemetry.addData("TEST SEQUENCE", "Step " + (i+1) + "/" + testPositions.length);
                telemetry.update();
                sleep(50);
            }
        }

        isTestSequenceRunning = false;
    }

    /**
     * Display telemetry information
     */
    private void displayTelemetry() {
        telemetry.clear();

        // Header
        telemetry.addData("═══ TURRET SERVO CALIBRATION ═══", "");
        telemetry.addData("", "");

        // Status
        telemetry.addData("Servo Status", isEnabled ? "✓ ENABLED" : "✗ DISABLED (Press START)");
        telemetry.addData("", "");

        // Current position
        telemetry.addData("Current Position", String.format("%.3f", currentPosition));
        telemetry.addData("Saved Position", String.format("%.3f", savedPosition));
        telemetry.addData("", "");

        // Calculated values (assuming 360° tuned range)
        double servoDegrees360 = currentPosition * 360.0;
        double turretAngle360 = (servoDegrees360 - 180.0) / 2.92; // Center at 180°

        telemetry.addData("If 360° Tuned", "");
        telemetry.addData("  Servo Degrees", String.format("%.1f°", servoDegrees360));
        telemetry.addData("  Turret Angle", String.format("%.1f°", turretAngle360));
        telemetry.addData("", "");

        // Calculated values (assuming 1800° range - 5 turn)
        double servoDegrees1800 = currentPosition * 1800.0;
        double turretAngle1800 = (servoDegrees1800 - 900.0) / 2.92; // Center at 900°

        telemetry.addData("If 1800° Range", "");
        telemetry.addData("  Servo Degrees", String.format("%.1f°", servoDegrees1800));
        telemetry.addData("  Turret Angle", String.format("%.1f°", turretAngle1800));
        telemetry.addData("", "");

        // Key positions reference
        telemetry.addData("═══ KEY POSITIONS ═══", "");
        telemetry.addData("0.00", "Minimum (should be -180° or left limit)");
        telemetry.addData("0.50", "Center (should be 0° / forward)");
        telemetry.addData("1.00", "Maximum (should be +180° or right limit)");
        telemetry.addData("", "");

        // Controls reminder
        telemetry.addData("═══ CONTROLS ═══", "");
        telemetry.addData("DPAD ↑/↓", "±0.1");
        telemetry.addData("DPAD ←/→", "±0.01");
        telemetry.addData("A/B/Y", "0.0 / 0.5 / 1.0");
        telemetry.addData("X", "Restore saved");
        telemetry.addData("L-BUMPER", "Save position");
        telemetry.addData("R-BUMPER", "Test sequence");
        telemetry.addData("START", "Emergency stop");

        telemetry.update();
    }

    /**
     * Clamp position to valid servo range
     */
    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
