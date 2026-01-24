package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AutoGyroTurret;
import org.firstinspires.ftc.teamcode.util.aurora.Localization;

/**
 * AutoGyroTurretTest - Test OpMode for field-relative turret control
 *
 * This OpMode demonstrates the AutoGyroTurret class which maintains
 * a field-relative heading for the turret using direct servo control.
 *
 * Hardware Requirements:
 * - Turret servo ("TurretLeft") configured with 360° travel tuner
 * - Servo position 0.0 = 0° turret, 1.0 = 360° turret
 * - Odometry (GoBILDA Pinpoint) for robot heading
 *
 * Controls:
 * ═══════════════════════════════════════════════════════════════════════
 * Gamepad 1:
 *   DPAD UP    - Set turret to point North (0°)
 *   DPAD RIGHT - Set turret to point East (90°)
 *   DPAD DOWN  - Set turret to point South (180°)
 *   DPAD LEFT  - Set turret to point West (270°)
 *
 *   A - Lock current turret direction as field-relative target
 *   B - Toggle auto-gyro mode ON/OFF
 *   X - Reset turret to point forward (robot-relative)
 *
 *   LEFT_STICK X  - Manual turret control (when auto-gyro disabled)
 *   RIGHT_STICK X - Manual turret control (when auto-gyro disabled)
 *
 * How It Works:
 * ═══════════════════════════════════════════════════════════════════════
 * 1. Set a field-relative heading (e.g., DPAD UP for North)
 * 2. Drive the robot around - turret automatically rotates to maintain
 *    that field heading
 * 3. The turret uses the shortest path and handles wraparound intelligently
 *
 * Example Scenario:
 * ═══════════════════════════════════════════════════════════════════════
 * 1. Robot faces North (0°), press DPAD RIGHT to aim turret East
 * 2. Turret rotates 90° (servo position 0.25)
 * 3. Rotate robot 180° (now facing South)
 * 4. Turret automatically rotates to 270° (servo position 0.75) to still point East
 * 5. Result: Turret maintains East heading regardless of robot orientation
 */
@TeleOp(name="AutoGyro Turret Test", group="Testing")
public class AutoGyroTurretTest extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════════════════════════

    private Servo turretServo;
    private AutoGyroTurret autoGyro;
    private Localization localization;

    // Current turret angle for manual control (0-360°)
    private double currentTurretAngle = 0.0;

    // ═══════════════════════════════════════════════════════════════════════
    // STATE TRACKING
    // ═══════════════════════════════════════════════════════════════════════

    // Button edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    // ═══════════════════════════════════════════════════════════════════════
    // MAIN OPMODE
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        // Initialize hardware
        telemetry.addLine("Initializing AutoGyro Turret Test...");
        telemetry.update();

        initializeHardware();

        if (turretServo == null) {
            telemetry.addData("ERROR", "Turret servo failed to initialize!");
            telemetry.addLine("Check hardware configuration for 'TurretLeft'");
            telemetry.update();
            return;
        }

        telemetry.addLine("✓ Hardware initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("DPAD: Set field headings (N/E/S/W)");
        telemetry.addLine("A: Lock current heading");
        telemetry.addLine("B: Toggle auto-gyro");
        telemetry.addLine("X: Reset to forward");
        telemetry.addLine("LEFT STICK X: Manual control (when disabled)");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        // Get initial robot heading
        double robotHeading = getRobotHeading();

        // Enable auto-gyro and set to forward
        autoGyro.enable();
        autoGyro.resetToForward(robotHeading);

        // Main control loop
        while (opModeIsActive()) {
            // Get current robot heading
            robotHeading = getRobotHeading();

            // Handle button controls
            handleControls(robotHeading);

            // Update auto-gyro (maintains field-relative heading)
            autoGyro.update(robotHeading);


            // Display telemetry
            displayTelemetry(robotHeading);
            telemetry.update();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ═══════════════════════════════════════════════════════════════════════

    private void initializeHardware() {
        // Initialize turret servo directly
        try {
            turretServo = hardwareMap.get(Servo.class, "TurretLeft");
            // Read initial position
            currentTurretAngle = turretServo.getPosition() * 360.0;
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to get turret servo: " + e.getMessage());
            turretServo = null;
        }

        // Initialize auto-gyro helper (now uses direct servo control)
        autoGyro = new AutoGyroTurret(hardwareMap, telemetry);

        // Initialize localization for robot heading
        localization = new Localization(hardwareMap);

        if (!localization.isOdometryInitialized()) {
            telemetry.addData("WARN", "Odometry not available - heading may be inaccurate");
        }

        localization.resetPosition();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CONTROL HANDLING
    // ═══════════════════════════════════════════════════════════════════════

    private void handleControls(double robotHeading) {
        // Field-relative heading presets (DPAD)
        if (gamepad1.dpad_up && !lastDpadUp) {
            // Point North (0°)
            autoGyro.setFieldRelativeHeading(0.0, robotHeading);
            autoGyro.enable();
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            // Point East (90°)
            autoGyro.setFieldRelativeHeading(90.0, robotHeading);
            autoGyro.enable();
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            // Point South (180°)
            autoGyro.setFieldRelativeHeading(180.0, robotHeading);
            autoGyro.enable();
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            // Point West (270°)
            autoGyro.setFieldRelativeHeading(270.0, robotHeading);
            autoGyro.enable();
        }

        // Lock current heading (A button)
        if (gamepad1.a && !lastA) {
            autoGyro.lockCurrentHeading(robotHeading);
            autoGyro.enable();
        }

        // Toggle auto-gyro mode (B button)
        if (gamepad1.b && !lastB) {
            boolean newState = autoGyro.toggle();
            telemetry.speak(newState ? "Auto gyro enabled" : "Auto gyro disabled");
        }

        // Reset to forward (X button)
        if (gamepad1.x && !lastX) {
            autoGyro.resetToForward(robotHeading);
            autoGyro.enable();
        }

        // Center turret (Y button) - direct servo control
        if (gamepad1.y && !lastY) {
            if (turretServo != null) {
                turretServo.setPosition(0.5); // 180° = backward/center
                currentTurretAngle = 180.0;
                telemetry.speak("Center position");
            }
            autoGyro.disable();
        }

        // HARDWARE TEST BUTTONS (gamepad2)
        // Test servo hardware directly - bypasses all logic
        if (gamepad2.a) {
            // Test position 0.0
            if (turretServo != null) {
                turretServo.setPosition(0.0);
                telemetry.speak("Servo zero");
            }
            autoGyro.disable();
        }
        if (gamepad2.b) {
            // Test position 0.25
            if (turretServo != null) {
                turretServo.setPosition(0.25);
                telemetry.speak("Servo quarter");
            }
            autoGyro.disable();
        }
        if (gamepad2.x) {
            // Test position 0.5
            if (turretServo != null) {
                turretServo.setPosition(0.5);
                telemetry.speak("Servo half");
            }
            autoGyro.disable();
        }
        if (gamepad2.y) {
            // Test position 1.0
            if (turretServo != null) {
                turretServo.setPosition(1.0);
                telemetry.speak("Servo max");
            }
            autoGyro.disable();
        }

        // Manual turret control (when auto-gyro disabled)
        if (!autoGyro.isEnabled() && turretServo != null) {
            double manualInput = -gamepad1.left_stick_x - gamepad1.right_stick_x;
            if (Math.abs(manualInput) > 0.1) {
                // Adjust turret angle based on input
                currentTurretAngle += manualInput * 3.0; // 3 degrees per frame

                // Normalize to 0-360 range
                while (currentTurretAngle < 0) currentTurretAngle += 360;
                while (currentTurretAngle >= 360) currentTurretAngle -= 360;

                // Set servo position directly
                double servoPosition = currentTurretAngle / 360.0;
                turretServo.setPosition(servoPosition);
            }
        }

        // Update button states for edge detection
        lastDpadUp = gamepad1.dpad_up;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SENSOR READING
    // ═══════════════════════════════════════════════════════════════════════

    private double getRobotHeading() {
        if (localization != null && localization.isOdometryInitialized()) {
            // Update localization (odometry + periodic Limelight corrections)
            localization.update();
            return localization.getHeading(AngleUnit.DEGREES);
        }
        return 0.0; // Fallback if localization unavailable
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    private void displayTelemetry(double robotHeading) {
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("      AUTO-GYRO TURRET TEST");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine();

        // Robot Pose (Position + Heading)
        if (localization != null && localization.isOdometryInitialized()) {
            telemetry.addLine("ROBOT POSE:");
            telemetry.addData("  X Position", "%.2f in", localization.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("  Y Position", "%.2f in", localization.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("  Heading", "%.1f°", robotHeading);

            // Velocity info
            double velX = localization.getVelocityX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            double velY = localization.getVelocityY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            double velHeading = localization.getHeadingVelocity(AngleUnit.DEGREES);
            telemetry.addData("  Velocity X", "%.1f in/s", velX);
            telemetry.addData("  Velocity Y", "%.1f in/s", velY);
            telemetry.addData("  Rotation", "%.1f °/s", velHeading);
            telemetry.addLine();
        } else {
            telemetry.addData("Robot Heading", "%.1f°", robotHeading);
            telemetry.addData("⚠ Odometry", "NOT AVAILABLE");
            telemetry.addLine();
        }

        // Auto-gyro status
        telemetry.addLine("AUTO-GYRO STATUS:");
        telemetry.addData("  Mode", autoGyro.isEnabled() ? "ENABLED" : "DISABLED");
        if (autoGyro.isEnabled()) {
            double fieldTarget = autoGyro.getFieldRelativeHeading();
            double fieldCurrent = autoGyro.getCurrentFieldHeading(robotHeading);
            double fieldError = autoGyro.getHeadingError(robotHeading);
            boolean atTarget = autoGyro.isAtTarget(robotHeading);

            telemetry.addData("  Field Target", "%.1f°", fieldTarget);
            telemetry.addData("  Field Current", "%.1f°", fieldCurrent);
            telemetry.addData("  Field Error", "%.1f°", fieldError);
            telemetry.addData("  At Target", atTarget ? "YES" : "NO");
        }
        telemetry.addLine();

        // Turret servo status with DEBUG INFO
        if (turretServo != null) {
            double servoPos = turretServo.getPosition();
            double turretAngle = servoPos * 360.0;

            telemetry.addLine("TURRET SERVO DEBUG:");
            telemetry.addData("  Raw Servo Pos", "%.4f (%.1f°)", servoPos, turretAngle);
            telemetry.addData("  Manual Tracked", "%.1f°", currentTurretAngle);

            // Calculate what SHOULD be happening if auto-gyro is enabled
            if (autoGyro.isEnabled()) {
                double fieldTarget = autoGyro.getFieldRelativeHeading();
                double requiredTurret = fieldTarget - robotHeading;

                // Normalize to 0-360 range
                while (requiredTurret < 0) requiredTurret += 360;
                while (requiredTurret >= 360) requiredTurret -= 360;

                double expectedServoPos = requiredTurret / 360.0;
                double delta = expectedServoPos - servoPos;
                double deltaDegrees = delta * 360.0;

                telemetry.addLine();
                telemetry.addData("  Field Target", "%.1f°", fieldTarget);
                telemetry.addData("  Robot Heading", "%.1f°", robotHeading);
                telemetry.addData("  Required Turret", "%.1f°", requiredTurret);
                telemetry.addData("  Expected Servo", "%.3f", expectedServoPos);
                telemetry.addLine();
                telemetry.addData("  🎯 DELTA", "%.3f (%.1f°)", delta, deltaDegrees);

                if (Math.abs(delta) > 0.05) {
                    telemetry.addData("  ⚠️ STATUS", "SERVO NOT RESPONDING!");
                } else {
                    telemetry.addData("  ✓ STATUS", "Servo at target");
                }
            }
        } else {
            telemetry.addData("Turret Servo", "NOT INITIALIZED");
        }
        telemetry.addLine();

        // Control hints
        telemetry.addLine("───────────────────────────────────────");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("GP1 DPAD: Set field heading (N/E/S/W)");
        telemetry.addLine("GP1 A: Lock | B: Toggle | X: Forward | Y: Center");
        telemetry.addLine();
        telemetry.addLine("HARDWARE TEST (GP2):");
        telemetry.addLine("A:0.0  B:0.25  X:0.5  Y:1.0");

        if (!autoGyro.isEnabled()) {
            telemetry.addLine();
            telemetry.addData("Manual Control", "GP1 LEFT/RIGHT STICK X");
        }

        telemetry.addLine("═══════════════════════════════════════");
    }
}
