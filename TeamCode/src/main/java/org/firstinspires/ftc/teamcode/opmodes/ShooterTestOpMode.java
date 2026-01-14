package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.aurora.DecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Turret;

/**
 * Shooter Test OpMode - Tests DecodeHelper shooter system with turret control
 *
 * This OpMode provides comprehensive testing for the dual-motor friction flywheel shooter
 * and integrated turret aiming system.
 *
 * SHOOTER CONTROLS:
 *
 * PRESET SHOOTING:
 * - Y Button: Long Range Shooting (4400 RPM)
 * - B Button: Mid-Range Shooting (3500 RPM)
 * - A Button: Short Range Shooting (2800 RPM)
 * - Left Trigger: Warmup Mode (65% of target RPM)
 *
 * D-PAD CONTROLS (Context-Sensitive):
 *
 * Default Mode (no modifier held):
 * - DPAD UP: Run uptake servos forward (feed into shooter)
 * - DPAD DOWN: Run uptake servos reverse (eject from shooter)
 * - DPAD LEFT: Rotate turret left (tap: +5°, hold: continuous at 200°/s)
 * - DPAD RIGHT: Rotate turret right (tap: -5°, hold: continuous at 200°/s)
 *
 * RPM Adjustment Mode (hold Left Bumper + DPAD):
 * - LB + DPAD UP: Increase target RPM by 100
 * - LB + DPAD DOWN: Decrease target RPM by 100
 * - LB + DPAD LEFT: Decrease target RPM by 500
 * - LB + DPAD RIGHT: Increase target RPM by 500
 *
 * OTHER CONTROLS:
 * - Right Bumper: Spin up to current target / Enable warmup
 * - Right Trigger: Manual fire (when ready)
 * - X Button: Stop shooter (IDLE)
 * - Back Button: Clear error state
 * - Start Button: Reset turret to center (0°)
 * - LT + RB: Emergency stop turret
 *
 * TELEMETRY:
 * - Current state and RPM data
 * - Target RPM and percentage achieved
 * - Motor sync status
 * - Ready/Firing indicators
 * - Time until next shot allowed
 * - Turret angle and status
 * - Uptake servo status
 * - Servo position (for debugging)
 */
@TeleOp(name = "🎯 Shooter Test", group = "Testing")
public class ShooterTestOpMode extends LinearOpMode {

    private DecodeHelper shooter;
    private Turret turret;
    private DcMotor leftShooterMotor;
    private DcMotor rightShooterMotor;
    private CRServo uptakeServoL;
    private CRServo uptakeServoR;

    // Manual target tracking
    private double manualTargetRPM = 3500;

    // Control mode
    private enum ControlMode {
        PRESET,  // Using preset buttons
        MANUAL   // Using manual RPM adjustment
    }
    private ControlMode controlMode = ControlMode.PRESET;

    // Button state tracking
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevX = false;
    private boolean prevRightBumper = false;
    private boolean prevRightTrigger = false;
    private boolean prevBack = false;

    // Turret adjustment step size (degrees per button press)
    private static final double TURRET_ANGLE_STEP = 5.0;  // Reduced from 10° to 5° for finer control

    // Turret continuous rotation timing
    private static final long TURRET_REPEAT_DELAY_MS = 25;    // Delay between increments when holding (200°/sec)
    private long lastTurretChangeTime = 0;
    private boolean turretLeftWasPressed = false;
    private boolean turretRightWasPressed = false;

    // Uptake servo power
    private static final double UPTAKE_POWER = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addLine("🎯 Shooter Test OpMode");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        boolean initSuccess = false;
        try {
            // Initialize motors (Shooter Front and Shooter Back as configured in hardware)
            leftShooterMotor = hardwareMap.get(DcMotor.class, "Shooter Front");
            rightShooterMotor = hardwareMap.get(DcMotor.class, "Shooter Back");

            // Initialize DecodeHelper
            shooter = new DecodeHelper(leftShooterMotor, rightShooterMotor, telemetry);

            // Initialize Turret
            turret = new Turret(hardwareMap, telemetry);
            boolean turretReady = turret.initialize();

            // Initialize Uptake Servos
            boolean uptakeReady = false;
            try {
                uptakeServoL = hardwareMap.get(CRServo.class, "Uptake Transfer Left");
                uptakeServoR = hardwareMap.get(CRServo.class, "Uptake Transfer Right");
                uptakeServoL.setPower(0);
                uptakeServoR.setPower(0);
                uptakeReady = true;
            } catch (Exception e) {
                telemetry.addLine("⚠️ Uptake servos not found: " + e.getMessage());
            }

            telemetry.addLine("✅ Hardware initialized successfully!");
            telemetry.addLine();

            if (turretReady) {
                telemetry.addLine("✅ Turret initialized: " + turret.getMode());
            } else {
                telemetry.addLine("⚠️ Turret initialization failed");
                telemetry.addLine("   Shooter will work without turret");
            }

            if (uptakeReady) {
                telemetry.addLine("✅ Uptake servos initialized");
            } else {
                telemetry.addLine("⚠️ Uptake servos not available");
            }

            telemetry.addLine();
            telemetry.addLine("Ready to start!");
            telemetry.addLine("Press PLAY to begin testing");
            initSuccess = true;

        } catch (Exception e) {
            telemetry.addLine("❌ ERROR: Failed to initialize hardware");
            telemetry.addLine(e.getMessage());
            telemetry.addLine();
            telemetry.addLine("Please check:");
            telemetry.addLine("1. Motors 'Shooter Front' and 'Shooter Back' exist in hardware config");
            telemetry.addLine("2. Robot is properly connected");
            telemetry.addLine();
            telemetry.addLine("Press STOP to exit");
        }

        telemetry.update();
        waitForStart();

        // Only run main loop if initialization succeeded
        if (initSuccess) {
            // Enable turret if initialized
            if (turret != null && turret.isInitialized()) {
                turret.enable();
            }

            // Main control loop
            while (opModeIsActive()) {
                // Handle control inputs
                handlePresetControls();
                handleDpadControls();  // Unified handler for all d-pad controls
                handleTestingControls();

                // Update shooter system
                shooter.update();

                // Update turret system
                if (turret != null && turret.isInitialized()) {
                    turret.update();
                }

                // Display telemetry
                displayTelemetry();
                telemetry.update();

                // Small delay to prevent excessive updates
                sleep(20);
            }

            // Stop shooter, turret, and uptake servos on exit
            shooter.disableShooter();
            if (turret != null && turret.isInitialized()) {
                turret.disable();
            }
            if (uptakeServoL != null) uptakeServoL.setPower(0);
            if (uptakeServoR != null) uptakeServoR.setPower(0);
            telemetry.addLine("🛑 OpMode stopped - all systems disabled");
            telemetry.update();
        } else {
            telemetry.addLine("❌ OpMode cannot run due to initialization failure");
            telemetry.update();
        }
    }

    /**
     * Handle preset shooting controls
     */
    private void handlePresetControls() {
        boolean anyPresetActive = false;

        // Y Button - Long Range
        if (gamepad1.y) {
            shooter.handleShootButton(true, ShooterConfig.ShooterPreset.LONG_RANGE);
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
        }
        // B Button - Mid Range
        else if (gamepad1.b) {
            shooter.handleShootButton(true, ShooterConfig.ShooterPreset.MID_RANGE);
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
        }
        // A Button - Short Range
        else if (gamepad1.a) {
            shooter.handleShootButton(true, ShooterConfig.ShooterPreset.SHORT_RANGE);
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
        }

        // If no preset button held, release
        if (!anyPresetActive && controlMode == ControlMode.PRESET) {
            shooter.handleShootButton(false, null);
        }

        // Left Trigger - Warmup Mode
        boolean warmupActive = gamepad1.left_trigger > 0.3;
        // Only pass preset when warmup is actually active, otherwise pass null to avoid auto-spinup
        ShooterConfig.ShooterPreset warmupPreset = warmupActive ?
                (shooter.getActivePreset() != null ? shooter.getActivePreset() : ShooterConfig.ShooterPreset.LONG_RANGE) :
                null;
        shooter.handleWarmupButton(warmupActive, warmupPreset);
    }

    /**
     * Handle all D-PAD controls (context-sensitive based on modifier button)
     * - If Left Bumper held: RPM adjustment mode
     * - If no modifier: Turret and uptake servo control
     */
    private void handleDpadControls() {
        boolean rpmAdjustMode = gamepad1.left_bumper;

        if (rpmAdjustMode) {
            // RPM ADJUSTMENT MODE (Left Bumper + DPAD)

            // DPAD UP - Increase by 100
            if (gamepad1.dpad_up && !prevDpadUp) {
                manualTargetRPM += 100;
                manualTargetRPM = Math.min(manualTargetRPM, 6000);
                controlMode = ControlMode.MANUAL;
            }

            // DPAD DOWN - Decrease by 100
            if (gamepad1.dpad_down && !prevDpadDown) {
                manualTargetRPM -= 100;
                manualTargetRPM = Math.max(manualTargetRPM, 0);
                controlMode = ControlMode.MANUAL;
            }

            // DPAD RIGHT - Increase by 500
            if (gamepad1.dpad_right && !prevDpadRight) {
                manualTargetRPM += 500;
                manualTargetRPM = Math.min(manualTargetRPM, 6000);
                controlMode = ControlMode.MANUAL;
            }

            // DPAD LEFT - Decrease by 500
            if (gamepad1.dpad_left && !prevDpadLeft) {
                manualTargetRPM -= 500;
                manualTargetRPM = Math.max(manualTargetRPM, 0);
                controlMode = ControlMode.MANUAL;
            }

            // Stop uptake servos when in RPM mode
            if (uptakeServoL != null) uptakeServoL.setPower(0);
            if (uptakeServoR != null) uptakeServoR.setPower(0);

        } else {
            // DEFAULT MODE - Turret and Uptake Control

            // DPAD UP - Run uptake servos forward
            if (gamepad1.dpad_up) {
                if (uptakeServoL != null) uptakeServoL.setPower(UPTAKE_POWER);
                if (uptakeServoR != null) uptakeServoR.setPower(UPTAKE_POWER);  // Same direction (one servo is physically reversed)
            }
            // DPAD DOWN - Run uptake servos reverse
            else if (gamepad1.dpad_down) {
                if (uptakeServoL != null) uptakeServoL.setPower(-UPTAKE_POWER);
                if (uptakeServoR != null) uptakeServoR.setPower(-UPTAKE_POWER);  // Same direction (one servo is physically reversed)
            }
            // No DPAD UP/DOWN - Stop uptake servos
            else {
                if (uptakeServoL != null) uptakeServoL.setPower(0);
                if (uptakeServoR != null) uptakeServoR.setPower(0);
            }

            // DPAD LEFT - Rotate turret left (increase angle)
            // Continuous rotation while held, with delay between increments
            if (gamepad1.dpad_left && turret != null && turret.isInitialized()) {
                long currentTime = System.currentTimeMillis();
                long timeSinceLastChange = currentTime - lastTurretChangeTime;

                // First press - trigger immediately
                if (!turretLeftWasPressed) {
                    double currentAngle = turret.getCurrentAngle();
                    double newAngle = currentAngle + TURRET_ANGLE_STEP;
                    // Clamp to reasonable range (-180 to 180)
                    newAngle = Math.max(-180, Math.min(180, newAngle));
                    turret.setAngle(newAngle);
                    lastTurretChangeTime = currentTime;
                    turretLeftWasPressed = true;
                }
                // Held down - check if enough time has passed
                else if (timeSinceLastChange >= TURRET_REPEAT_DELAY_MS) {
                    double currentAngle = turret.getCurrentAngle();
                    double newAngle = currentAngle + TURRET_ANGLE_STEP;
                    // Clamp to reasonable range (-180 to 180)
                    newAngle = Math.max(-180, Math.min(180, newAngle));
                    turret.setAngle(newAngle);
                    lastTurretChangeTime = currentTime;
                }
            } else {
                turretLeftWasPressed = false;
            }

            // DPAD RIGHT - Rotate turret right (decrease angle)
            // Continuous rotation while held, with delay between increments
            if (gamepad1.dpad_right && turret != null && turret.isInitialized()) {
                long currentTime = System.currentTimeMillis();
                long timeSinceLastChange = currentTime - lastTurretChangeTime;

                // First press - trigger immediately
                if (!turretRightWasPressed) {
                    double currentAngle = turret.getCurrentAngle();
                    double newAngle = currentAngle - TURRET_ANGLE_STEP;
                    // Clamp to reasonable range (-180 to 180)
                    newAngle = Math.max(-180, Math.min(180, newAngle));
                    turret.setAngle(newAngle);
                    lastTurretChangeTime = currentTime;
                    turretRightWasPressed = true;
                }
                // Held down - check if enough time has passed
                else if (timeSinceLastChange >= TURRET_REPEAT_DELAY_MS) {
                    double currentAngle = turret.getCurrentAngle();
                    double newAngle = currentAngle - TURRET_ANGLE_STEP;
                    // Clamp to reasonable range (-180 to 180)
                    newAngle = Math.max(-180, Math.min(180, newAngle));
                    turret.setAngle(newAngle);
                    lastTurretChangeTime = currentTime;
                }
            } else {
                turretRightWasPressed = false;
            }
        }

        // Update button state tracking
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;

        // X Button - Stop shooter
        if (gamepad1.x && !prevX) {
            shooter.disableShooter();
            controlMode = ControlMode.MANUAL;
        }
        prevX = gamepad1.x;
    }

    /**
     * Handle testing controls (spin up, warmup, fire, etc.)
     */
    private void handleTestingControls() {
        // Left Trigger + Right Bumper - Emergency stop turret
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_bumper && turret != null && turret.isInitialized()) {
            turret.stop();
        }

        // Right Bumper - Spin up to target (when not holding for RPM mode)
        // Only trigger on release to avoid conflict with modifier function
        boolean rightBumperPressed = gamepad1.right_bumper;
        if (!rightBumperPressed && prevRightBumper) {
            // Button released - check if it was a tap (not held with DPAD)
            if (controlMode == ControlMode.MANUAL) {
                shooter.setTargetRPM(manualTargetRPM);
                shooter.spinUp();
            }
        }
        prevRightBumper = rightBumperPressed;

        // Right Trigger - Manual fire
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        if (rightTriggerPressed && !prevRightTrigger) {
            if (shooter.isReady()) {
                shooter.fire();
            }
        }
        prevRightTrigger = rightTriggerPressed;

        // Back Button - Clear error
        if (gamepad1.back && !prevBack) {
            shooter.clearError();
        }
        prevBack = gamepad1.back;

        // Start Button - Reset turret to center
        if (gamepad1.start && turret != null && turret.isInitialized()) {
            turret.setToCenter();
        }
    }


    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry() {
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("🎯 SHOOTER TEST OPMODE");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine();

        // Control mode
        telemetry.addLine("CONTROL MODE: " + controlMode);
        if (controlMode == ControlMode.MANUAL) {
            telemetry.addLine("Manual Target: " + String.format("%.0f RPM", manualTargetRPM));
        }
        telemetry.addLine();

        // Shooter status
        telemetry.addLine("SHOOTER STATUS:");
        telemetry.addLine(shooter.getStatusString());
        telemetry.addLine();

        // RPM Details
        telemetry.addLine("RPM DETAILS:");
        telemetry.addData("  Left Motor", "%.0f RPM", shooter.getLeftRPM());
        telemetry.addData("  Right Motor", "%.0f RPM", shooter.getRightRPM());
        telemetry.addData("  Average", "%.0f RPM", shooter.getAverageRPM());
        telemetry.addData("  Sync Error (instant)", "%.0f RPM", shooter.getRPMSyncError());
        telemetry.addData("  Sync Error (avg)", "%.0f RPM", shooter.getAverageSyncError());
        telemetry.addData("  Progress", "%.1f%%", shooter.getRPMPercentage());
        telemetry.addLine();

        // State information
        telemetry.addLine("STATE INFORMATION:");
        telemetry.addData("  State", shooter.getState().toString());
        telemetry.addData("  Ready", shooter.isReady() ? "✅ YES" : "❌ NO");
        telemetry.addData("  At Target", shooter.isAtTargetRPM() ? "✅ YES" : "❌ NO");

        if (shooter.getActivePreset() != null) {
            telemetry.addData("  Active Preset", shooter.getActivePreset().getName());
        }

        if (shooter.getState() == DecodeHelper.ShooterState.SPINNING_UP) {
            telemetry.addData("  Spin-up Time", "%.2f s", shooter.getSpinUpTime() / 1000.0);
        }

        long nextShotTime = shooter.getTimeUntilNextShot();
        if (nextShotTime > 0) {
            telemetry.addData("  Next Shot In", "%.2f s", nextShotTime / 1000.0);
        }
        telemetry.addLine();

        // Turret status (if initialized)
        if (turret != null && turret.isInitialized()) {
            telemetry.addLine("TURRET STATUS:");
            telemetry.addData("  Status", turret.isEnabled() ? "✅ ENABLED" : "❌ DISABLED");
            telemetry.addData("  Mode", turret.getMode().toString());
            telemetry.addData("  Current Angle", "%.1f°", turret.getCurrentAngle());
            telemetry.addData("  Target Angle", "%.1f°", turret.getTargetAngle());
            telemetry.addData("  At Target", turret.isAtTarget() ? "✅ YES" : "❌ NO");
            telemetry.addData("  Angle Error", "%.1f°", turret.getAngleError());
            // Add servo position for debugging (only works in position mode)
            if (turret.getServoPosition() >= 0) {
                telemetry.addData("  Servo Position", "%.3f", turret.getServoPosition());
            }
            telemetry.addData("  Dpad Right", gamepad1.dpad_right ? "PRESSED" : "released");
            telemetry.addData("  Dpad Left", gamepad1.dpad_left ? "PRESSED" : "released");
            telemetry.addLine();
        }

        // Uptake servo status
        if (uptakeServoL != null && uptakeServoR != null) {
            telemetry.addLine("UPTAKE SERVOS:");
            double leftPower = uptakeServoL.getPower();
            double rightPower = uptakeServoR.getPower();
            String status = (leftPower != 0 || rightPower != 0) ? "🔄 RUNNING" : "⏸️ STOPPED";
            telemetry.addData("  Status", status);
            telemetry.addData("  Left Power", "%.2f", leftPower);
            telemetry.addData("  Right Power", "%.2f", rightPower);
            telemetry.addLine();
        }

        // Controls reference
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  Y/B/A: Long/Mid/Short Range");
        telemetry.addLine("  LT: Warmup | RT: Manual Fire");
        telemetry.addLine();
        telemetry.addLine("DEFAULT MODE (no LB):");
        telemetry.addLine("  DPAD UP/DOWN: Uptake Forward/Rev");
        telemetry.addLine("  DPAD L/R: Turret ±5° (hold=cont.)");
        telemetry.addLine();
        telemetry.addLine("RPM MODE (hold LB + DPAD):");
        telemetry.addLine("  LB+UP/DOWN: RPM ±100");
        telemetry.addLine("  LB+L/R: RPM ±500");
        telemetry.addLine();
        telemetry.addLine("  RB: Spin Up | X: Stop | Back: Clear");
        telemetry.addLine("  Start: Center Turret | LT+RB: E-Stop");
    }
}

