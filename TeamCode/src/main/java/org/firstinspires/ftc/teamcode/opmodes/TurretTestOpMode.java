package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Turret;

/**
 * Turret Test OpMode - Demonstrates turret control and integration
 *
 * This OpMode provides a simple interface to test and calibrate the turret system.
 *
 * Controls:
 * - DPAD UP: Face forward (0°)
 * - DPAD DOWN: Face backward (180°)
 * - DPAD LEFT: Face left (90°)
 * - DPAD RIGHT: Face right (-90°)
 * - RIGHT STICK X: Manual rotation control
 * - A: Set to 45° (northeast)
 * - B: Set to -45° (southeast)
 * - X: Reset angle tracking
 * - Y: Toggle turret enable/disable
 * - LEFT BUMPER: Start calibration test (measure servo speed)
 * - RIGHT BUMPER: Reset calibration data
 * - BACK: Toggle calibration mode display
 *
 * The OpMode displays:
 * - Current turret angle
 * - Target turret angle
 * - At target status
 * - Turret mode (300°, 5-turn, or continuous)
 * - Enable status
 * - CALIBRATION DATA (when testing)
 */
@TeleOp(name = "🎯 Turret Test", group = "Testing")
public class TurretTestOpMode extends LinearOpMode {

    private AuroraHardwareConfig hardware;
    private Turret turret;

    // ═══════════════════════════════════════════════════════════════════════
    // MANUAL CALIBRATION SYSTEM
    // ═══════════════════════════════════════════════════════════════════════
    // This system allows manual tuning of rotation time by visual verification.
    // Since servos don't provide actual position feedback, humans must verify
    // if the servo rotated the correct amount, then adjust time accordingly.

    private enum CalibrationState {
        IDLE,
        RUNNING,
        STOPPED
    }

    private CalibrationState calibrationState = CalibrationState.IDLE;
    private boolean showCalibrationMode = false;

    // Tuning parameters
    // Based on actual testing: 360° in 0.34s = ~1059 °/s (consistent!)
    // Hardware limitation: Commands shorter than 0.3s don't work accurately
    // Use 360° test for best results (requires 0.34s)
    private double testRotationTime = 0.34;  // Seconds - user adjusts this
    private double testTargetDegrees = 360.0;  // Degrees - 360° for best accuracy
    private final double TIME_ADJUSTMENT_VERY_FINE = 0.001;  // Seconds per button press (0.001s = 1ms, very precise)
    private final double TIME_ADJUSTMENT_FINE = 0.01;  // Seconds per button press (10ms steps)
    private final double TIME_ADJUSTMENT_COARSE = 0.05;  // Seconds per button press (50ms steps)
    private final double ANGLE_ADJUSTMENT = 45.0;  // Degrees per button press
    private static final double MIN_TIME = 0.3;  // Minimum test time (hardware limitation)

    // Test execution
    private long testStartTime = 0;
    private boolean testRunning = false;

    // Hardware access for continuous rotation calibration
    private com.qualcomm.robotcore.hardware.CRServo hardwareServo = null;

    // Gear ratio from Turret class
    private static final double GEAR_RATIO = 108.0 / 37.0; // 2.92

    // History tracking for multiple tests
    private java.util.ArrayList<Double> testHistory = new java.util.ArrayList<>();
    private java.util.ArrayList<Double> angleHistory = new java.util.ArrayList<>();

    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetry.addLine("🎯 Initializing Turret Test...");
        telemetry.update();

        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        // Initialize turret
        turret = new Turret(hardwareMap, telemetry);
        boolean turretReady = turret.initialize();

        // Display initialization status
        telemetry.addLine();
        if (turretReady) {
            telemetry.addLine("✅ Turret initialized successfully!");
            telemetry.addData("Mode", turret.getMode());
        } else {
            telemetry.addLine("❌ Turret initialization failed!");
            telemetry.addLine("Check hardware configuration");
        }

        telemetry.addLine();
        telemetry.addLine("Press START to begin test");
        telemetry.update();

        waitForStart();

        // Enable turret
        turret.enable();

        // Get direct hardware servo access for manual calibration
        // We'll use continuous rotation mode for calibration
        try {
            hardwareServo = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "turretServo");
            telemetry.addLine("✅ Hardware servo access obtained (CR mode)");
        } catch (Exception e) {
            telemetry.addLine("⚠️ Could not access hardware servo");
            telemetry.addLine("Calibration may not work correctly");
        }
        telemetry.update();
        sleep(1000);

        while (opModeIsActive()) {
            // Update turret state
            turret.update();

            // Update calibration state machine
            updateCalibration();

            // ═══════════════════════════════════════════════════════════════
            // CONTROL INPUTS
            // ═══════════════════════════════════════════════════════════════

            // Calibration controls (highest priority when in calibration mode)
            if (showCalibrationMode) {
                handleCalibrationControls();
            }

            if (gamepad1.back) {
                showCalibrationMode = !showCalibrationMode;
                if (!showCalibrationMode) {
                    stopCalibrationTest(); // Stop any running test
                }
                sleep(200); // Debounce
            }

            // Normal controls (only when not calibrating)
            if (calibrationState == CalibrationState.IDLE) {
                // Direct position control with right trigger (0.0-1.0 servo position)
                // This tests setServoPositionDirect() method
                if (gamepad1.right_trigger >= 0.05) {
                    turret.setServoPositionDirect(gamepad1.right_trigger);
                }
                // Angle control with left trigger (tests setAngle() method)
                // For 5-turn mode: 0.5 trigger = 0°, 0.0 = -308.5°, 1.0 = +308.5°
                else if (gamepad1.left_trigger >= 0.05) {
                    // Map trigger to full turret angle range (centered at 0.5)
                    // 5-turn: ±308.5° range
                    // 2.25-turn: ±138.5° range
                    // 300°: ±51.5° range
                    double maxAngle = turret.getMode() == Turret.TurretMode.POSITION_MODE_5TURN ? 308.5 :
                                     turret.getMode() == Turret.TurretMode.POSITION_MODE_2_25TURN ? 138.5 : 51.5;
                    double targetAngle = (gamepad1.left_trigger - 0.5) * 2.0 * maxAngle;
                    turret.setAngle(targetAngle);
                }
                // Only use other controls if triggers are not being used
                else if (gamepad1.dpad_up) {
                    turret.setToForward();
                    telemetry.addLine("→ Facing FORWARD");
                } else if (gamepad1.dpad_down) {
                    turret.setToBackward();
                    telemetry.addLine("→ Facing BACKWARD");
                } else if (gamepad1.dpad_left) {
                    turret.setToLeft();
                    telemetry.addLine("→ Facing LEFT");
                } else if (gamepad1.dpad_right) {
                    turret.setToRight();
                    telemetry.addLine("→ Facing RIGHT");
                }
                // Custom angles with buttons
                else if (gamepad1.a) {
                    turret.setAngle(45.0);
                    telemetry.addLine("→ Set to 45°");
                } else if (gamepad1.b) {
                    turret.setAngle(-45.0);
                    telemetry.addLine("→ Set to -45°");
                }
                // Manual rotation with right stick
                else {
                    double rotation = -gamepad1.right_stick_x;
                    if (Math.abs(rotation) > 0.1) {
                        turret.rotate(rotation);
                    } else {
                        // Stop turret if no input
                        turret.stop();
                    }
                }

                // Utility controls
                if (gamepad1.x) {
                    turret.resetAngleTracking();
                    telemetry.addLine("→ Angle tracking reset");
                }

                if (gamepad1.y) {
                    if (turret.isEnabled()) {
                        turret.disable();
                        telemetry.addLine("→ Turret DISABLED");
                    } else {
                        turret.enable();
                        telemetry.addLine("→ Turret ENABLED");
                    }
                    sleep(200); // Debounce
                }
            }

            // ═══════════════════════════════════════════════════════════════
            // TELEMETRY DISPLAY
            // ═══════════════════════════════════════════════════════════════

            if (showCalibrationMode) {
                displayCalibrationTelemetry();
            } else {
                displayNormalTelemetry();
            }


            telemetry.update();
        }

        // Cleanup
        turret.stop();
        turret.disable();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MANUAL CALIBRATION METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Handle calibration mode controls
     */
    private void handleCalibrationControls() {
        if (hardwareServo == null) {
            return;
        }

        // Time adjustment - very fine (triggers + d-pad) for precise tuning
        if ((gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5) && !testRunning) {
            if (gamepad1.dpad_up) {
                testRotationTime += TIME_ADJUSTMENT_VERY_FINE;
                sleep(100);
            } else if (gamepad1.dpad_down) {
                testRotationTime = Math.max(MIN_TIME, testRotationTime - TIME_ADJUSTMENT_VERY_FINE);
                sleep(100);
            }
        }
        // Time adjustment - fine (D-pad up/down alone)
        else if (gamepad1.dpad_up && !testRunning) {
            testRotationTime += TIME_ADJUSTMENT_FINE;
            sleep(100);
        } else if (gamepad1.dpad_down && !testRunning) {
            testRotationTime = Math.max(MIN_TIME, testRotationTime - TIME_ADJUSTMENT_FINE);
            sleep(100);
        }

        // Time adjustment - coarse (D-pad left/right)
        if (gamepad1.dpad_left && !testRunning) {
            testRotationTime = Math.max(MIN_TIME, testRotationTime - TIME_ADJUSTMENT_COARSE);
            sleep(100);
        } else if (gamepad1.dpad_right && !testRunning) {
            testRotationTime += TIME_ADJUSTMENT_COARSE;
            sleep(100);
        }

        // Target angle adjustment (A/B buttons)
        if (gamepad1.a && !testRunning) {
            testTargetDegrees += ANGLE_ADJUSTMENT;
            sleep(200);
        } else if (gamepad1.b && !testRunning) {
            testTargetDegrees = Math.max(ANGLE_ADJUSTMENT, testTargetDegrees - ANGLE_ADJUSTMENT);
            sleep(200);
        }

        // Start/stop test (Left bumper)
        if (gamepad1.left_bumper) {
            if (!testRunning) {
                startCalibrationTest();
            } else {
                stopCalibrationTest();
            }
            sleep(300);
        }

        // Save current settings (Right bumper)
        if (gamepad1.right_bumper && !testRunning) {
            saveTestResult();
            sleep(300);
        }

        // Reset history (Y button)
        if (gamepad1.y && !testRunning) {
            testHistory.clear();
            angleHistory.clear();
            sleep(200);
        }
    }

    /**
     * Start a calibration test run
     */
    private void startCalibrationTest() {
        if (hardwareServo == null) {
            return;
        }

        testStartTime = System.currentTimeMillis();
        testRunning = true;
        calibrationState = CalibrationState.RUNNING;

        // Run servo at full speed
        hardwareServo.setPower(1.0);
    }

    /**
     * Stop the calibration test
     */
    private void stopCalibrationTest() {
        if (hardwareServo == null) {
            return;
        }

        hardwareServo.setPower(0.0);
        testRunning = false;
        calibrationState = CalibrationState.STOPPED;
    }

    /**
     * Save the current test result
     */
    private void saveTestResult() {
        testHistory.add(testRotationTime);
        angleHistory.add(testTargetDegrees);
    }

    /**
     * Update calibration state machine
     */
    private void updateCalibration() {
        if (!testRunning || hardwareServo == null) {
            return;
        }

        // Check if test duration elapsed
        long elapsed = System.currentTimeMillis() - testStartTime;
        double elapsedSeconds = elapsed / 1000.0;

        if (elapsedSeconds >= testRotationTime) {
            stopCalibrationTest();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY DISPLAY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Display normal operation telemetry
     */
    private void displayNormalTelemetry() {
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("🎯 TURRET STATUS");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addData("Mode", turret.getMode());
        telemetry.addData("Enabled", turret.isEnabled() ? "✅ YES" : "❌ NO");
        telemetry.addLine();

        telemetry.addLine("───────────────────────────────────");
        telemetry.addLine("📐 POSITION");
        telemetry.addLine("───────────────────────────────────");
        telemetry.addData("Current Angle", "%.1f°", turret.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("Error", "%.1f°", turret.getAngleError());
        telemetry.addData("Servo Position", "%.3f", turret.getServoPosition());
        telemetry.addData("At Target", turret.isAtTarget() ? "✅ YES" : "⚠️ NO");
        telemetry.addLine();

        telemetry.addLine("───────────────────────────────────");
        telemetry.addLine("📊 TRIGGER INPUT");
        telemetry.addLine("───────────────────────────────────");

        // Calculate angle range based on mode
        double maxAngle = turret.getMode() == Turret.TurretMode.POSITION_MODE_5TURN ? 308.5 :
                         turret.getMode() == Turret.TurretMode.POSITION_MODE_2_25TURN ? 138.5 : 51.5;

        // Right trigger - direct position control
        telemetry.addData("Right Trigger", "%.3f (pos)", gamepad1.right_trigger);
        telemetry.addLine("  Direct servo position 0.0-1.0");

        // Left trigger - angle control
        double leftTriggerAngle = (gamepad1.left_trigger - 0.5) * 2.0 * maxAngle;
        telemetry.addData("Left Trigger", "%.3f (%.1f°)", gamepad1.left_trigger, leftTriggerAngle);
        telemetry.addLine(String.format("  Angle: 0.0=%.1f°, 0.5=0°, 1.0=+%.1f°", -maxAngle, maxAngle));
        telemetry.addLine();

        telemetry.addLine("───────────────────────────────────");
        telemetry.addLine("🎮 CONTROLS");
        telemetry.addLine("───────────────────────────────────");
        telemetry.addLine("RIGHT TRIGGER: Direct position (0.0-1.0)");
        telemetry.addLine("  Tests setServoPositionDirect()");
        telemetry.addLine("  Raw servo position control");
        telemetry.addLine();
        telemetry.addLine("LEFT TRIGGER: Angle control");
        telemetry.addLine("  Tests setAngle() method");
        telemetry.addLine("  0.0 = max negative, 0.5 = 0°, 1.0 = max positive");
        telemetry.addLine(String.format("  Range: ±%.1f° (centered)", maxAngle));
        telemetry.addLine();
        telemetry.addLine("D-Pad: Preset positions");
        telemetry.addLine("Right Stick X: Manual rotation");
        telemetry.addLine("A/B: 45°/-45° angles");
        telemetry.addLine("X: Reset tracking");
        telemetry.addLine("Y: Toggle enable");
        telemetry.addLine();
        telemetry.addLine("BACK: Show calibration mode");
    }

    /**
     * Display manual calibration telemetry
     */
    private void displayCalibrationTelemetry() {
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("🎯 MANUAL SERVO CALIBRATION");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("Visually verify rotation and adjust");
        telemetry.addLine("time until accurate. Servo runs at");
        telemetry.addLine("FULL POWER for the set duration.");
        telemetry.addLine();

        // Current test parameters
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("📐 CURRENT TEST SETTINGS");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addData("Target Rotation", "%.0f°", testTargetDegrees);
        telemetry.addData("Duration", "%.3f sec", testRotationTime);

        // Warning for short duration tests
        if (testRotationTime < 0.3) {
            telemetry.addLine("⚠️ WARNING: Duration < 0.3s");
            telemetry.addLine("   Hardware can't accurately time");
            telemetry.addLine("   short commands! Use ≥0.3s");
        }

        // Calculate and display expected speed
        double expectedTurretSpeed = testTargetDegrees / testRotationTime;
        double expectedServoSpeed = expectedTurretSpeed * GEAR_RATIO;
        telemetry.addData("Expected Speed", "%.1f °/s (turret)", expectedTurretSpeed);
        telemetry.addData("", "%.1f °/s (servo)", expectedServoSpeed);
        telemetry.addLine();

        // Live test status
        telemetry.addLine("───────────────────────────────────");
        telemetry.addLine("🔄 TEST STATUS");
        telemetry.addLine("───────────────────────────────────");

        if (testRunning) {
            long elapsed = System.currentTimeMillis() - testStartTime;
            double elapsedSec = elapsed / 1000.0;
            double remaining = testRotationTime - elapsedSec;

            telemetry.addData("Status", "🟢 RUNNING");
            telemetry.addData("Elapsed", "%.2f sec", elapsedSec);
            telemetry.addData("Remaining", "%.2f sec", Math.max(0, remaining));

            // Progress bar
            int progress = (int)((elapsedSec / testRotationTime) * 20);
            StringBuilder bar = new StringBuilder("[");
            for (int i = 0; i < 20; i++) {
                bar.append(i < progress ? "█" : "░");
            }
            bar.append("]");
            telemetry.addData("Progress", bar.toString());
        } else {
            telemetry.addData("Status", calibrationState == CalibrationState.STOPPED ? "⏹️ STOPPED" : "⏸️ READY");
            telemetry.addLine("Press LEFT BUMPER to start test");
        }
        telemetry.addLine();

        // Test history
        if (testHistory.size() > 0) {
            telemetry.addLine("═══════════════════════════════════");
            telemetry.addLine("📋 TEST HISTORY");
            telemetry.addLine("═══════════════════════════════════");

            int startIdx = Math.max(0, testHistory.size() - 5); // Show last 5
            for (int i = startIdx; i < testHistory.size(); i++) {
                double time = testHistory.get(i);
                double angle = angleHistory.get(i);
                double speed = angle / time;
                telemetry.addLine(String.format("%d: %.2fs for %.0f° (%.1f°/s)",
                    i + 1, time, angle, speed));
            }
            telemetry.addLine();

            // Average of all tests
            double avgTime = 0;
            double avgAngle = 0;
            for (int i = 0; i < testHistory.size(); i++) {
                avgTime += testHistory.get(i);
                avgAngle += angleHistory.get(i);
            }
            avgTime /= testHistory.size();
            avgAngle /= testHistory.size();
            double avgSpeed = avgAngle / avgTime;

            telemetry.addLine("───────────────────────────────────");
            telemetry.addLine("📊 AVERAGE OF ALL TESTS");
            telemetry.addLine("───────────────────────────────────");
            telemetry.addData("Tests", testHistory.size());
            telemetry.addData("Avg Speed", "%.2f °/s (turret)", avgSpeed);
            telemetry.addLine();

            // Code to copy
            telemetry.addLine("═══════════════════════════════════");
            telemetry.addLine("💻 COPY THIS TO Turret.java");
            telemetry.addLine("═══════════════════════════════════");
            telemetry.addLine("Line ~168 (continuousRotationSpeed):");
            telemetry.addLine(String.format("private double continuousRotationSpeed = %.2f;", avgSpeed));
            telemetry.addLine();
        }

        // Controls help
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("🎮 CONTROLS");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("D-PAD ↑↓: Adjust time ±0.01s (10ms)");
        telemetry.addLine("D-PAD ←→: Adjust time ±0.05s (50ms)");
        telemetry.addLine("TRIGGER + D-PAD ↑↓: ±0.001s (1ms!)");
        telemetry.addLine("A/B: Change target angle ±45°");
        telemetry.addLine("LEFT BUMPER: Start/Stop test");
        telemetry.addLine("RIGHT BUMPER: Save current settings");
        telemetry.addLine("Y: Clear test history");
        telemetry.addLine("BACK: Exit calibration");
        telemetry.addLine();

        // Instructions
        telemetry.addLine("───────────────────────────────────");
        telemetry.addLine("📝 INSTRUCTIONS");
        telemetry.addLine("───────────────────────────────────");
        telemetry.addLine("⚠️ Use 360° test (default)!");
        telemetry.addLine("   Hardware can't time <0.3s accurately");
        telemetry.addLine();
        telemetry.addLine("1. Keep 360° target (default)");
        telemetry.addLine("2. Default 0.34s should be close!");
        telemetry.addLine("3. Press LEFT BUMPER to run test");
        telemetry.addLine("4. WATCH full 360° rotation");
        telemetry.addLine("5. Did it complete full circle?");
        telemetry.addLine("   - Too far? Decrease time");
        telemetry.addLine("   - Not far enough? Increase time");
        telemetry.addLine("6. Press RIGHT BUMPER to save");
        telemetry.addLine("7. Repeat 3-5 times, use average!");
        telemetry.addLine();
        telemetry.addLine("Measured: 0.34s = ~1059°/s ✓");
    }
}

