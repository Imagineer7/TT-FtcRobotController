/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Example OpMode demonstrating RobotLiftController usage
 */

package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.RobotLiftController;

/**
 * Example OpMode showing how to use the RobotLiftController
 *
 * Controls:
 * - Left Stick Y: Manual lift control
 * - D-pad Up: Move to high position
 * - D-pad Right: Move to mid position
 * - D-pad Left: Move to low position
 * - D-pad Down: Move to ground position
 * - X: Show basic telemetry
 * - Y: Show detailed telemetry
 * - A: Show calibration telemetry / Confirm calibration test
 * - B: Reset performance stats
 * - START: Run automatic no-load speed calibration test
 *
 * CALIBRATION TEST:
 * 1. Remove ALL load from lift (detach game piece mechanism)
 * 2. Press START button to begin test
 * 3. Lift will automatically run at 100% power for 1000 ticks
 * 4. Test measures time and calculates EXPECTED_TICKS_PER_SECOND_NO_LOAD
 * 5. Copy the displayed value to RobotLiftController.java
 */
@TeleOp(name="Lift Controller Example", group="Examples")
public class LiftControllerExample extends LinearOpMode {

    private RobotLiftController liftController;
    private int telemetryMode = 0;  // 0=basic, 1=detailed, 2=calibration

    // No-load speed calibration test
    private enum CalibrationState {
        IDLE,
        WAITING_TO_START,
        RUNNING_TEST,
        TEST_COMPLETE,
        TEST_FAILED
    }

    private CalibrationState calibrationState = CalibrationState.IDLE;
    private double calibrationStartTime = 0;
    private int calibrationStartPosition = 0;
    private double calculatedNoLoadSpeed = 0;
    private static final int CALIBRATION_TEST_DISTANCE = 1000;  // ticks
    private static final double CALIBRATION_TEST_POWER = 1.0;  // 100% power

    @Override
    public void runOpMode() {
        // Initialize lift controller with gamepad vibration support
        telemetry.addLine("Initializing Lift Controller Example...");
        telemetry.update();

        liftController = new RobotLiftController(hardwareMap, telemetry, gamepad1, null);
        if (!liftController.initialize()) {
            telemetry.addLine("❌ Failed to initialize lift controller!");
            telemetry.addLine("Check motor names: robotLiftLeft & robotLiftRight");
            telemetry.update();
            return;
        }

        telemetry.addLine("✅ Ready to start!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("• Left Stick Y: Manual control");
        telemetry.addLine("• D-pad: Preset positions");
        telemetry.addLine("• X/Y/A: Change telemetry mode");
        telemetry.addLine("• B: Reset stats");
        telemetry.addLine("• START: Run no-load speed calibration");
        telemetry.addLine();
        telemetry.addLine("⚠️ CALIBRATION TEST:");
        telemetry.addLine("Remove ALL load from lift before");
        telemetry.addLine("running calibration test!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Start calibration test if START button pressed
            if (gamepad1.start && calibrationState == CalibrationState.IDLE) {
                startCalibrationTest();
            }

            // Handle calibration test state machine
            if (calibrationState != CalibrationState.IDLE) {
                updateCalibrationTest();
            }

            // Get manual control input (invert if needed for your setup)
            double liftInput = -gamepad1.left_stick_y;

            // Only allow manual control when not running calibration
            if (calibrationState == CalibrationState.IDLE) {
                // Handle preset position buttons
                if (gamepad1.dpad_up) {
                    liftController.moveToHigh();
                } else if (gamepad1.dpad_right) {
                    liftController.moveToMid();
                } else if (gamepad1.dpad_left) {
                    liftController.moveToLow();
                } else if (gamepad1.dpad_down) {
                    liftController.moveToGround();
                }

                // Cancel auto-position if driver moves stick
                if (Math.abs(liftInput) > 0.1 && liftController.isPositionControlActive()) {
                    liftController.cancelPositionControl();
                }

                // Update lift controller with manual input
                liftController.update(liftInput);
            } else {
                // During calibration, don't allow manual control
                liftInput = 0;
            }

            // Handle telemetry mode changes
            if (gamepad1.x) {
                telemetryMode = 0;  // Basic
            } else if (gamepad1.y) {
                telemetryMode = 1;  // Detailed
            } else if (gamepad1.a) {
                telemetryMode = 2;  // Calibration
            }

            // Reset performance stats
            if (gamepad1.b) {
                liftController.resetPerformanceStats();
            }

            // Display telemetry based on mode
            telemetry.clear();

            // Show calibration test status if running
            if (calibrationState != CalibrationState.IDLE) {
                displayCalibrationStatus();
            } else {
                switch (telemetryMode) {
                    case 0:
                        liftController.addTelemetry();
                        telemetry.addLine();
                        telemetry.addLine("Press Y for detailed telemetry");
                        telemetry.addLine("Press A for calibration telemetry");
                        break;

                    case 1:
                        liftController.addDetailedTelemetry();
                        telemetry.addLine();
                        telemetry.addLine("Press X for basic telemetry");
                        telemetry.addLine("Press A for calibration telemetry");
                        break;

                    case 2:
                        liftController.addCalibrationTelemetry();
                        telemetry.addLine();
                        telemetry.addLine("Press X for basic telemetry");
                        telemetry.addLine("Press Y for detailed telemetry");
                        break;
                }

                telemetry.addLine();
                telemetry.addData("Status", liftController.getStatusString());

                // Show PID status
                if (liftController.isGravityPidActive()) {
                    telemetry.addLine();
                    telemetry.addLine("🤖 Auto Gravity PID: ACTIVE");
                    telemetry.addData("Learned Power", "%.4f", liftController.getGravityCompensationPower());
                }

                // Show option to start calibration
                telemetry.addLine();
                telemetry.addLine("📊 Press START to run no-load test");
            }

            // Update vibration feedback for physical warnings
            liftController.updateVibrationFeedback();

            telemetry.update();
        }

        // Cleanup
        liftController.stop();
        telemetry.clear();
        telemetry.addLine("Lift Controller Example stopped");
        telemetry.update();
    }

    /**
     * Start the no-load speed calibration test
     */
    private void startCalibrationTest() {
        // Check if lift is at a position where it can safely run the test
        int currentPos = liftController.getCurrentPosition();
        int maxPos = 3000;  // Adjust based on your lift's max position

        if (currentPos + CALIBRATION_TEST_DISTANCE > maxPos) {
            telemetry.addLine("⚠️ Lift too high to run test!");
            telemetry.addLine("Lower lift and try again");
            telemetry.update();
            sleep(2000);
            return;
        }

        // Start the test
        calibrationState = CalibrationState.WAITING_TO_START;
        calibrationStartPosition = currentPos;
        calibrationStartTime = getRuntime();

        telemetry.addLine("🚀 Starting calibration test...");
        telemetry.addLine("DO NOT TOUCH CONTROLS!");
        telemetry.update();
        sleep(1000);  // Give user a moment to see message

        calibrationState = CalibrationState.RUNNING_TEST;
    }

    /**
     * Update the calibration test state machine
     */
    private void updateCalibrationTest() {
        int currentPos = liftController.getCurrentPosition();
        double currentTime = getRuntime();

        switch (calibrationState) {
            case WAITING_TO_START:
                // Already handled in startCalibrationTest
                break;

            case RUNNING_TEST:
                // Apply full power upward
                liftController.cancelPositionControl();  // Cancel any auto-positioning
                liftController.disableGravityCompensation();  // Disable PID compensation

                // Check if we've traveled the required distance
                int distanceTraveled = currentPos - calibrationStartPosition;

                if (distanceTraveled >= CALIBRATION_TEST_DISTANCE) {
                    // Test complete!
                    double elapsedTime = currentTime - calibrationStartTime;
                    calculatedNoLoadSpeed = CALIBRATION_TEST_DISTANCE / elapsedTime;
                    calibrationState = CalibrationState.TEST_COMPLETE;

                    // Stop the lift
                    liftController.update(0);

                } else if (distanceTraveled < 0) {
                    // Lift went down instead of up - test failed
                    calibrationState = CalibrationState.TEST_FAILED;
                    liftController.update(0);

                } else if (currentTime - calibrationStartTime > 5.0) {
                    // Test taking too long (more than 5 seconds) - probably stalled
                    calibrationState = CalibrationState.TEST_FAILED;
                    liftController.update(0);

                } else {
                    // Continue running test at full power
                    liftController.update(CALIBRATION_TEST_POWER);
                }
                break;

            case TEST_COMPLETE:
            case TEST_FAILED:
                // Display results, wait for user to acknowledge
                if (gamepad1.a) {
                    calibrationState = CalibrationState.IDLE;
                    liftController.resetGravityCompensation();  // Re-enable PID
                }
                break;

            case IDLE:
                // Nothing to do
                break;
        }
    }

    /**
     * Display calibration test status and results
     */
    private void displayCalibrationStatus() {
        telemetry.addLine("=== NO-LOAD SPEED CALIBRATION ===");
        telemetry.addLine();

        switch (calibrationState) {
            case WAITING_TO_START:
                telemetry.addLine("⏳ Preparing test...");
                break;

            case RUNNING_TEST:
                int distanceTraveled = liftController.getCurrentPosition() - calibrationStartPosition;
                double progress = (distanceTraveled * 100.0) / CALIBRATION_TEST_DISTANCE;
                telemetry.addLine("🏃 RUNNING TEST...");
                telemetry.addLine();
                telemetry.addData("Distance", "%d / %d ticks", distanceTraveled, CALIBRATION_TEST_DISTANCE);
                telemetry.addData("Progress", "%.1f%%", progress);
                telemetry.addData("Time", "%.2f seconds", getRuntime() - calibrationStartTime);
                telemetry.addLine();
                telemetry.addLine("⚠️ DO NOT TOUCH CONTROLS!");
                break;

            case TEST_COMPLETE:
                telemetry.addLine("✅ TEST COMPLETE!");
                telemetry.addLine();
                telemetry.addData("Time", "%.3f seconds",
                    (CALIBRATION_TEST_DISTANCE / calculatedNoLoadSpeed));
                telemetry.addData("Distance", "%d ticks", CALIBRATION_TEST_DISTANCE);
                telemetry.addLine();
                telemetry.addLine("📊 CALCULATED NO-LOAD SPEED:");
                telemetry.addData("Ticks/Second", "%.2f", calculatedNoLoadSpeed);
                telemetry.addLine();
                telemetry.addLine("════════════════════════════════");
                telemetry.addLine("UPDATE THIS CONSTANT:");
                telemetry.addLine();
                telemetry.addLine("private static final double");
                telemetry.addLine("  EXPECTED_TICKS_PER_SECOND_NO_LOAD");
                telemetry.addData("    = ", "%.1f;", calculatedNoLoadSpeed);
                telemetry.addLine();
                telemetry.addLine("════════════════════════════════");
                telemetry.addLine();
                telemetry.addLine("Copy this value to RobotLiftController.java");
                telemetry.addLine("(around line 154)");
                telemetry.addLine();
                telemetry.addLine("Press A to return to normal operation");
                break;

            case TEST_FAILED:
                telemetry.addLine("❌ TEST FAILED!");
                telemetry.addLine();
                telemetry.addLine("Possible causes:");
                telemetry.addLine("• Lift has load attached");
                telemetry.addLine("• Motor power too low");
                telemetry.addLine("• Mechanical obstruction");
                telemetry.addLine("• Lift started too high");
                telemetry.addLine();
                telemetry.addLine("Please:");
                telemetry.addLine("1. Remove ALL load from lift");
                telemetry.addLine("2. Lower lift to bottom");
                telemetry.addLine("3. Check for obstructions");
                telemetry.addLine("4. Try again");
                telemetry.addLine();
                telemetry.addLine("Press A to return to normal operation");
                break;

            case IDLE:
                // Should not reach here
                break;
        }
    }
}

