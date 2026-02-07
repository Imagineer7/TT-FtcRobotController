package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret Servo Calibration Test
 *
 * Purpose: Determine exact servo positions for turret forward and backward orientations
 * to calculate precise gear ratio and rotation range.
 *
 * Test Procedure:
 * 1. Start at servo position 0.5 (known forward position)
 * 2. Use DPAD UP/DOWN to increment/decrement servo position
 * 3. Rotate turret clockwise until it points backward (180° from forward)
 * 4. Record servo position and calculate rotation
 * 5. Reset to 0.5
 * 6. Rotate turret counter-clockwise until it points backward
 * 7. Record servo position and calculate rotation
 * 8. Calculate gear ratio from collected data
 *
 * Controls:
 * - DPAD UP: Increase servo position (small increment)
 * - DPAD DOWN: Decrease servo position (small increment)
 * - DPAD RIGHT: Increase servo position (large increment)
 * - DPAD LEFT: Decrease servo position (large increment)
 * - A: Reset to 0.5 (forward position)
 * - B: Record current position as backward (clockwise)
 * - X: Record current position as backward (counter-clockwise)
 * - Y: Calculate and display gear ratio
 * - START: Toggle increment size
 *
 * Data to Record:
 * - Forward position: 0.5 (known)
 * - Backward CW position: (measure by rotating clockwise)
 * - Backward CCW position: (measure by rotating counter-clockwise)
 * - Full rotation servo degrees = (forward - backward) * 2
 * - Gear ratio = full rotation servo / 360° turret
 */
@TeleOp(name="🔧 Turret Servo Calibration", group="Calibration")
public class TurretServoCalibrationTest extends LinearOpMode {

    // Hardware
    private Servo turretServo;

    // Servo control
    private double currentPosition = 0.5;  // Start at approximate forward position
    private static final double SMALL_INCREMENT = 0.001;  // 0.1% = ~0.81° servo
    private static final double LARGE_INCREMENT = 0.01;   // 1% = ~8.1° servo
    private double currentIncrement = SMALL_INCREMENT;

    // Calibration constants
    private static final double SERVO_PHYSICAL_RANGE = 810.0;  // Degrees

    // Recorded positions
    private double forwardPosition = -1;  // To be measured (not assumed!)
    private double backwardClockwisePosition = -1;  // Not yet recorded
    private double backwardCounterClockwisePosition = -1;  // Not yet recorded

    // Button state tracking (for edge detection)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastStart = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        telemetry.setAutoClear(false);
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("    TURRET SERVO CALIBRATION TEST");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("Initializing turret servo...");
        telemetry.update();

        try {
            turretServo = hardwareMap.get(Servo.class, "TurretLeft");
            telemetry.addLine("✓ Turret servo found");
        } catch (Exception e) {
            telemetry.addLine("✗ ERROR: Could not find servo 'TurretLeft'");
            telemetry.addLine("  Check hardware configuration!");
            telemetry.update();
            while (opModeIsActive()) {
                sleep(100);
            }
            return;
        }

        // Set initial position
        turretServo.setPosition(currentPosition);

        telemetry.addLine("✓ Servo initialized at position 0.5 (approx)");
        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("CALIBRATION PROCEDURE:");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("1. Use DPAD to find where turret points");
        telemetry.addLine("   FORWARD (straight ahead)");
        telemetry.addLine("2. Press Y to record forward position");
        telemetry.addLine("3. Press DPAD UP/DOWN to rotate CW");
        telemetry.addLine("   until turret points BACKWARD (180°)");
        telemetry.addLine("4. Press B to record CW backward position");
        telemetry.addLine("5. Press A to return to approx forward");
        telemetry.addLine("6. Press DPAD to rotate CCW to backward");
        telemetry.addLine("7. Press X to record CCW backward position");
        telemetry.addLine("8. View calculated gear ratio & constants");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            handleControls();
            updateTelemetry();
            sleep(50);  // Small delay for button debouncing
        }
    }

    /**
     * Handle gamepad controls
     */
    private void handleControls() {
        // DPAD UP - Increment position (small)
        if (gamepad1.dpad_up && !lastDpadUp) {
            currentPosition = Math.min(1.0, currentPosition + currentIncrement);
            turretServo.setPosition(currentPosition);
        }
        lastDpadUp = gamepad1.dpad_up;

        // DPAD DOWN - Decrement position (small)
        if (gamepad1.dpad_down && !lastDpadDown) {
            currentPosition = Math.max(0.0, currentPosition - currentIncrement);
            turretServo.setPosition(currentPosition);
        }
        lastDpadDown = gamepad1.dpad_down;

        // DPAD RIGHT - Increment position (large)
        if (gamepad1.dpad_right && !lastDpadRight) {
            currentPosition = Math.min(1.0, currentPosition + LARGE_INCREMENT);
            turretServo.setPosition(currentPosition);
        }
        lastDpadRight = gamepad1.dpad_right;

        // DPAD LEFT - Decrement position (large)
        if (gamepad1.dpad_left && !lastDpadLeft) {
            currentPosition = Math.max(0.0, currentPosition - LARGE_INCREMENT);
            turretServo.setPosition(currentPosition);
        }
        lastDpadLeft = gamepad1.dpad_left;

        // A - Reset to approximate forward position (0.5)
        if (gamepad1.a && !lastA) {
            currentPosition = 0.5;
            turretServo.setPosition(currentPosition);
        }
        lastA = gamepad1.a;

        // B - Record backward clockwise position
        if (gamepad1.b && !lastB) {
            backwardClockwisePosition = currentPosition;
        }
        lastB = gamepad1.b;

        // X - Record backward counter-clockwise position
        if (gamepad1.x && !lastX) {
            backwardCounterClockwisePosition = currentPosition;
        }
        lastX = gamepad1.x;

        // Y - Record forward position
        if (gamepad1.y && !lastY) {
            forwardPosition = currentPosition;
        }
        lastY = gamepad1.y;

        // START - Toggle increment size
        if (gamepad1.start && !lastStart) {
            if (currentIncrement == SMALL_INCREMENT) {
                currentIncrement = LARGE_INCREMENT;
            } else {
                currentIncrement = SMALL_INCREMENT;
            }
        }
        lastStart = gamepad1.start;
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.clear();

        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("    TURRET SERVO CALIBRATION TEST");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine();

        // Current servo state
        telemetry.addLine("━━━ CURRENT SERVO STATE ━━━");
        telemetry.addData("Servo Position", "%.4f", currentPosition);
        telemetry.addData("Servo Degrees", "%.2f°", currentPosition * SERVO_PHYSICAL_RANGE);
        telemetry.addData("Increment Size", currentIncrement == SMALL_INCREMENT ? "SMALL (0.001)" : "LARGE (0.01)");
        telemetry.addLine();

        // Relative to forward (if recorded)
        if (forwardPosition >= 0) {
            double degreesFromForward = (currentPosition - forwardPosition) * SERVO_PHYSICAL_RANGE;
            telemetry.addData("From Forward", "%.2f° servo", degreesFromForward);
        }
        telemetry.addLine();

        // Recorded positions
        telemetry.addLine("━━━ RECORDED POSITIONS ━━━");

        if (forwardPosition >= 0) {
            telemetry.addData("Forward", "%.4f = %.2f°", forwardPosition, forwardPosition * SERVO_PHYSICAL_RANGE);
        } else {
            telemetry.addData("Forward", "NOT RECORDED (press Y)");
        }

        if (backwardClockwisePosition >= 0) {
            telemetry.addData("Backward CW", "%.4f = %.2f°",
                backwardClockwisePosition,
                backwardClockwisePosition * SERVO_PHYSICAL_RANGE);
            if (forwardPosition >= 0) {
                double cwRotation = Math.abs(backwardClockwisePosition - forwardPosition) * SERVO_PHYSICAL_RANGE;
                telemetry.addData("  CW Rotation", "%.2f° servo", cwRotation);
            }
        } else {
            telemetry.addData("Backward CW", "NOT RECORDED (press B)");
        }

        if (backwardCounterClockwisePosition >= 0) {
            telemetry.addData("Backward CCW", "%.4f = %.2f°",
                backwardCounterClockwisePosition,
                backwardCounterClockwisePosition * SERVO_PHYSICAL_RANGE);
            if (forwardPosition >= 0) {
                double ccwRotation = Math.abs(forwardPosition - backwardCounterClockwisePosition) * SERVO_PHYSICAL_RANGE;
                telemetry.addData("  CCW Rotation", "%.2f° servo", ccwRotation);
            }
        } else {
            telemetry.addData("Backward CCW", "NOT RECORDED (press X)");
        }
        telemetry.addLine();

        // Calculations
        if (forwardPosition >= 0 && backwardClockwisePosition >= 0 && backwardCounterClockwisePosition >= 0) {
            telemetry.addLine("━━━ CALCULATED RESULTS ━━━");

            // Calculate rotations using measured forward position
            double forwardServo = forwardPosition * SERVO_PHYSICAL_RANGE;
            double backwardCWServo = backwardClockwisePosition * SERVO_PHYSICAL_RANGE;
            double backwardCCWServo = backwardCounterClockwisePosition * SERVO_PHYSICAL_RANGE;

            // CW rotation: forward → backward (should be positive if CW > forward)
            // But based on your data: CW (0.18) < forward (0.5), so turret rotated "backwards" in servo space
            // This means we need to calculate the actual rotation correctly
            double cwRotation = Math.abs(backwardCWServo - forwardServo);
            double ccwRotation = Math.abs(backwardCCWServo - forwardServo);

            double avgHalfRotation = (cwRotation + ccwRotation) / 2.0;
            double fullRotationServo = avgHalfRotation * 2.0;

            // Calculate gear ratio (assuming 180° turret rotation from forward to back)
            double turretRotation = 180.0;  // Half rotation
            double gearRatio = avgHalfRotation / turretRotation;

            // Calculate total turret range
            double totalTurretRange = SERVO_PHYSICAL_RANGE / gearRatio;

            // Calculate forward offset in turret degrees
            double forwardOffsetTurret = forwardServo / gearRatio;

            telemetry.addData("Forward Servo", "%.2f°", forwardServo);
            telemetry.addData("Backward CW Servo", "%.2f°", backwardCWServo);
            telemetry.addData("Backward CCW Servo", "%.2f°", backwardCCWServo);
            telemetry.addLine();
            telemetry.addData("CW Half Rotation", "%.2f° servo", cwRotation);
            telemetry.addData("CCW Half Rotation", "%.2f° servo", ccwRotation);
            telemetry.addData("Avg Half Rotation", "%.2f° servo", avgHalfRotation);
            telemetry.addData("Full Rotation", "%.2f° servo for 360° turret", fullRotationServo);
            telemetry.addLine();
            telemetry.addData("Gear Ratio", "%.4f (servo rotates %.2fx faster)", gearRatio, gearRatio);
            telemetry.addData("Total Turret Range", "%.2f° turret rotation", totalTurretRange);
            telemetry.addData("Forward Offset", "%.2f° turret", forwardOffsetTurret);
            telemetry.addLine();

            // Generate code snippet
            telemetry.addLine("━━━ CODE TO USE ━━━");
            telemetry.addLine("Update AutoGyroTurret.java:");
            telemetry.addData("GEAR_RATIO", "%.4f", gearRatio);
            telemetry.addData("MAX_TURRET_ROTATION", "%.2f", totalTurretRange);
            telemetry.addData("FORWARD_OFFSET", "%.2f", forwardOffsetTurret);
            telemetry.addData("FORWARD_SERVO_POS", "%.4f", forwardPosition);
            telemetry.addLine();
        } else {
            telemetry.addLine("━━━ AWAITING DATA ━━━");
            if (forwardPosition < 0) {
                telemetry.addLine("⚠ Need forward position (press Y)");
            }
            if (backwardClockwisePosition < 0) {
                telemetry.addLine("⚠ Need CW backward position (press B)");
            }
            if (backwardCounterClockwisePosition < 0) {
                telemetry.addLine("⚠ Need CCW backward position (press X)");
            }
            telemetry.addLine();
        }

        // Controls reminder
        telemetry.addLine("━━━ CONTROLS ━━━");
        telemetry.addLine("DPAD ↑/↓    : Adjust servo (small)");
        telemetry.addLine("DPAD ←/→    : Adjust servo (large)");
        telemetry.addLine("A           : Reset to 0.5");
        telemetry.addLine("Y           : Record FORWARD position");
        telemetry.addLine("B           : Record backward CW");
        telemetry.addLine("X           : Record backward CCW");
        telemetry.addLine("START       : Toggle increment size");

        telemetry.update();
    }
}

