package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;

/**
 * BasicIndexingHelper - Manual control for indexing system motors and servos
 *
 * This helper class provides:
 * - Individual motor/servo power control
 * - Non-blocking timed movements for each actuator
 * - Priority-based operation management (latest call takes priority)
 * - Status checks for timed movements
 * - Global timeout protection
 *
 * Hardware Components:
 * - Front Intake: DC motor roller, transfer CR servo, bottom CR servo
 * - Back Intake: DC motor roller, transfer CR servo, bottom CR servo
 * - Injector System: Left and right CR servos
 * - Uptake System: Left and right CR servos
 *
 * Usage Pattern:
 *   BasicIndexingHelper helper = new BasicIndexingHelper(hardware, telemetry);
 *
 *   // Set power (immediate)
 *   helper.setFrontRollerPower(1.0);
 *
 *   // Timed movement (non-blocking)
 *   helper.setFrontRollerTimed(1.0, 1000); // 1.0 power for 1000ms
 *
 *   // Check status
 *   if (!helper.isFrontRollerBusy()) {
 *       // Movement complete
 *   }
 *
 *   // Complex operations
 *   helper.runFrontIntake(true, 1.0); // Run entire front intake
 *
 *   // MUST call update() in loop
 *   helper.update();
 */
public class BasicIndexingHelper {

    // Hardware references
    private final AuroraHardwareConfig hardware;
    private final Telemetry telemetry;

    // Individual hardware components
    private final DcMotor frontRollerMotor;
    private final DcMotor backRollerMotor;
    private final CRServo frontBottomIntakeServo;
    private final CRServo backBottomIntakeServo;
    private final CRServo frontTransferServo;
    private final CRServo backTransferServo;
    private final CRServo uptakeServoL;
    private final CRServo uptakeServoR;
    private final CRServo injectorServoLeft;
    private final CRServo injectorServoRight;

    // Global timeout (milliseconds)
    private static final long GLOBAL_TIMEOUT = 10000; // 10 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // TIMED MOVEMENT TRACKING
    // ═══════════════════════════════════════════════════════════════════════

    // Front Roller Motor
    private long frontRollerEndTime = 0;
    private double frontRollerTimedPower = 0;
    private boolean frontRollerTimedActive = false;

    // Back Roller Motor
    private long backRollerEndTime = 0;
    private double backRollerTimedPower = 0;
    private boolean backRollerTimedActive = false;

    // Front Bottom Intake Servo
    private long frontBottomIntakeEndTime = 0;
    private double frontBottomIntakeTimedPower = 0;
    private boolean frontBottomIntakeTimedActive = false;

    // Back Bottom Intake Servo
    private long backBottomIntakeEndTime = 0;
    private double backBottomIntakeTimedPower = 0;
    private boolean backBottomIntakeTimedActive = false;

    // Front Transfer Servo
    private long frontTransferEndTime = 0;
    private double frontTransferTimedPower = 0;
    private boolean frontTransferTimedActive = false;

    // Back Transfer Servo
    private long backTransferEndTime = 0;
    private double backTransferTimedPower = 0;
    private boolean backTransferTimedActive = false;

    // Uptake Left Servo
    private long uptakeLEndTime = 0;
    private double uptakeLTimedPower = 0;
    private boolean uptakeLTimedActive = false;

    // Uptake Right Servo
    private long uptakeREndTime = 0;
    private double uptakeRTimedPower = 0;
    private boolean uptakeRTimedActive = false;

    // Injector Left Servo
    private long injectorLeftEndTime = 0;
    private double injectorLeftTimedPower = 0;
    private boolean injectorLeftTimedActive = false;

    // Injector Right Servo
    private long injectorRightEndTime = 0;
    private double injectorRightTimedPower = 0;
    private boolean injectorRightTimedActive = false;

    // Global status
    private boolean enabled = true;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new BasicIndexingHelper
     * @param hardware AuroraHardwareConfig with initialized hardware
     * @param telemetry Telemetry for status updates
     */
    public BasicIndexingHelper(AuroraHardwareConfig hardware, Telemetry telemetry) {
        this.hardware = hardware;
        this.telemetry = telemetry;

        // Get hardware references
        this.frontRollerMotor = hardware.getFrontRollerMotor();
        this.backRollerMotor = hardware.getBackRollerMotor();
        this.frontBottomIntakeServo = hardware.getFrontBottomIntakeServo();
        this.backBottomIntakeServo = hardware.getBackBottomIntakeServo();
        this.frontTransferServo = hardware.getFrontTransferServo();
        this.backTransferServo = hardware.getBackTransferServo();
        this.uptakeServoL = hardware.getUptakeServoL();
        this.uptakeServoR = hardware.getUptakeServoR();
        this.injectorServoLeft = hardware.getInjectorServoLeft();
        this.injectorServoRight = hardware.getInjectorServoRight();

        // Validate hardware
        if (!hardware.isIndexingSystemInitialized()) {
            telemetry.addData("⚠️ WARNING", "Indexing system not initialized");
            enabled = false;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UPDATE METHOD - MUST BE CALLED EVERY LOOP
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update all timed movements - MUST be called every loop
     * Handles automatic stopping of timed movements when complete
     * Also updates complex transfer sequences
     */
    public void update() {
        if (!enabled) return;

        long currentTime = System.currentTimeMillis();

        // Update transfer sequences
        updateTransferSequences();

        // Update front roller motor
        if (frontRollerTimedActive) {
            if (currentTime >= frontRollerEndTime) {
                frontRollerMotor.setPower(0);
                frontRollerTimedActive = false;
            } else {
                frontRollerMotor.setPower(frontRollerTimedPower);
            }
        }

        // Update back roller motor
        if (backRollerTimedActive) {
            if (currentTime >= backRollerEndTime) {
                backRollerMotor.setPower(0);
                backRollerTimedActive = false;
            } else {
                backRollerMotor.setPower(backRollerTimedPower);
            }
        }

        // Update front bottom intake servo
        if (frontBottomIntakeTimedActive) {
            if (currentTime >= frontBottomIntakeEndTime) {
                frontBottomIntakeServo.setPower(0);
                frontBottomIntakeTimedActive = false;
            } else {
                frontBottomIntakeServo.setPower(frontBottomIntakeTimedPower);
            }
        }

        // Update back bottom intake servo
        if (backBottomIntakeTimedActive) {
            if (currentTime >= backBottomIntakeEndTime) {
                backBottomIntakeServo.setPower(0);
                backBottomIntakeTimedActive = false;
            } else {
                backBottomIntakeServo.setPower(backBottomIntakeTimedPower);
            }
        }

        // Update front transfer servo
        if (frontTransferTimedActive) {
            if (currentTime >= frontTransferEndTime) {
                frontTransferServo.setPower(0);
                frontTransferTimedActive = false;
            } else {
                frontTransferServo.setPower(frontTransferTimedPower);
            }
        }

        // Update back transfer servo
        if (backTransferTimedActive) {
            if (currentTime >= backTransferEndTime) {
                backTransferServo.setPower(0);
                backTransferTimedActive = false;
            } else {
                backTransferServo.setPower(backTransferTimedPower);
            }
        }

        // Update uptake left servo
        if (uptakeLTimedActive) {
            if (currentTime >= uptakeLEndTime) {
                uptakeServoL.setPower(0);
                uptakeLTimedActive = false;
            } else {
                uptakeServoL.setPower(uptakeLTimedPower);
            }
        }

        // Update uptake right servo
        if (uptakeRTimedActive) {
            if (currentTime >= uptakeREndTime) {
                uptakeServoR.setPower(0);
                uptakeRTimedActive = false;
            } else {
                uptakeServoR.setPower(uptakeRTimedPower);
            }
        }

        // Update injector left servo
        if (injectorLeftTimedActive) {
            if (currentTime >= injectorLeftEndTime) {
                injectorServoLeft.setPower(0);
                injectorLeftTimedActive = false;
            } else {
                injectorServoLeft.setPower(injectorLeftTimedPower);
            }
        }

        // Update injector right servo
        if (injectorRightTimedActive) {
            if (currentTime >= injectorRightEndTime) {
                injectorServoRight.setPower(0);
                injectorRightTimedActive = false;
            } else {
                injectorServoRight.setPower(injectorRightTimedPower);
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FRONT ROLLER MOTOR CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set front roller motor power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setFrontRollerPower(double power) {
        if (!enabled) return;
        frontRollerTimedActive = false; // Cancel timed movement
        frontRollerMotor.setPower(power);
    }

    /**
     * Run front roller motor for specified duration (non-blocking)
     * Cancels any previous timed movement on this motor
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setFrontRollerTimed(double power, long durationMs) {
        if (!enabled) return;

        // Validate timeout
        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Front roller duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        frontRollerTimedPower = power;
        frontRollerEndTime = System.currentTimeMillis() + durationMs;
        frontRollerTimedActive = true;
    }

    /**
     * Check if front roller has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isFrontRollerBusy() {
        return frontRollerTimedActive;
    }

    /**
     * Stop front roller motor immediately
     */
    public void stopFrontRoller() {
        frontRollerTimedActive = false;
        frontRollerMotor.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // BACK ROLLER MOTOR CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set back roller motor power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setBackRollerPower(double power) {
        if (!enabled) return;
        backRollerTimedActive = false; // Cancel timed movement
        backRollerMotor.setPower(power);
    }

    /**
     * Run back roller motor for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setBackRollerTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Back roller duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        backRollerTimedPower = power;
        backRollerEndTime = System.currentTimeMillis() + durationMs;
        backRollerTimedActive = true;
    }

    /**
     * Check if back roller has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isBackRollerBusy() {
        return backRollerTimedActive;
    }

    /**
     * Stop back roller motor immediately
     */
    public void stopBackRoller() {
        backRollerTimedActive = false;
        backRollerMotor.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FRONT BOTTOM INTAKE SERVO CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set front bottom intake servo power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setFrontBottomIntakePower(double power) {
        if (!enabled) return;
        frontBottomIntakeTimedActive = false;
        frontBottomIntakeServo.setPower(power);
    }

    /**
     * Run front bottom intake servo for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setFrontBottomIntakeTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Front bottom intake duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        frontBottomIntakeTimedPower = power;
        frontBottomIntakeEndTime = System.currentTimeMillis() + durationMs;
        frontBottomIntakeTimedActive = true;
    }

    /**
     * Check if front bottom intake servo has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isFrontBottomIntakeBusy() {
        return frontBottomIntakeTimedActive;
    }

    /**
     * Stop front bottom intake servo immediately
     */
    public void stopFrontBottomIntake() {
        frontBottomIntakeTimedActive = false;
        frontBottomIntakeServo.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // BACK BOTTOM INTAKE SERVO CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set back bottom intake servo power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setBackBottomIntakePower(double power) {
        if (!enabled) return;
        backBottomIntakeTimedActive = false;
        backBottomIntakeServo.setPower(power);
    }

    /**
     * Run back bottom intake servo for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setBackBottomIntakeTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Back bottom intake duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        backBottomIntakeTimedPower = power;
        backBottomIntakeEndTime = System.currentTimeMillis() + durationMs;
        backBottomIntakeTimedActive = true;
    }

    /**
     * Check if back bottom intake servo has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isBackBottomIntakeBusy() {
        return backBottomIntakeTimedActive;
    }

    /**
     * Stop back bottom intake servo immediately
     */
    public void stopBackBottomIntake() {
        backBottomIntakeTimedActive = false;
        backBottomIntakeServo.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FRONT TRANSFER SERVO CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set front transfer servo power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setFrontTransferPower(double power) {
        if (!enabled) return;
        frontTransferTimedActive = false;
        frontTransferServo.setPower(power);
    }

    /**
     * Run front transfer servo for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setFrontTransferTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Front transfer duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        frontTransferTimedPower = power;
        frontTransferEndTime = System.currentTimeMillis() + durationMs;
        frontTransferTimedActive = true;
    }

    /**
     * Check if front transfer servo has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isFrontTransferBusy() {
        return frontTransferTimedActive;
    }

    /**
     * Stop front transfer servo immediately
     */
    public void stopFrontTransfer() {
        frontTransferTimedActive = false;
        frontTransferServo.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // BACK TRANSFER SERVO CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set back transfer servo power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setBackTransferPower(double power) {
        if (!enabled) return;
        backTransferTimedActive = false;
        backTransferServo.setPower(power);
    }

    /**
     * Run back transfer servo for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setBackTransferTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Back transfer duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        backTransferTimedPower = power;
        backTransferEndTime = System.currentTimeMillis() + durationMs;
        backTransferTimedActive = true;
    }

    /**
     * Check if back transfer servo has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isBackTransferBusy() {
        return backTransferTimedActive;
    }

    /**
     * Stop back transfer servo immediately
     */
    public void stopBackTransfer() {
        backTransferTimedActive = false;
        backTransferServo.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UPTAKE SERVO CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set uptake left servo power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setUptakeLPower(double power) {
        if (!enabled) return;
        uptakeLTimedActive = false;
        uptakeServoL.setPower(power);
    }

    /**
     * Run uptake left servo for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setUptakeLTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Uptake L duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        uptakeLTimedPower = power;
        uptakeLEndTime = System.currentTimeMillis() + durationMs;
        uptakeLTimedActive = true;
    }

    /**
     * Check if uptake left servo has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isUptakeLBusy() {
        return uptakeLTimedActive;
    }

    /**
     * Stop uptake left servo immediately
     */
    public void stopUptakeL() {
        uptakeLTimedActive = false;
        uptakeServoL.setPower(0);
    }

    /**
     * Set uptake right servo power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setUptakeRPower(double power) {
        if (!enabled) return;
        uptakeRTimedActive = false;
        uptakeServoR.setPower(power);
    }

    /**
     * Run uptake right servo for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setUptakeRTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Uptake R duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        uptakeRTimedPower = power;
        uptakeREndTime = System.currentTimeMillis() + durationMs;
        uptakeRTimedActive = true;
    }

    /**
     * Check if uptake right servo has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isUptakeRBusy() {
        return uptakeRTimedActive;
    }

    /**
     * Stop uptake right servo immediately
     */
    public void stopUptakeR() {
        uptakeRTimedActive = false;
        uptakeServoR.setPower(0);
    }

    /**
     * Set both uptake servos power (immediate, cancels any timed movements)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setUptakePower(double power) {
        setUptakeLPower(power);
        setUptakeRPower(power);
    }

    /**
     * Run both uptake servos for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setUptakeTimed(double power, long durationMs) {
        setUptakeLTimed(power, durationMs);
        setUptakeRTimed(power, durationMs);
    }

    /**
     * Check if either uptake servo has an active timed movement
     * @return true if any uptake servo is busy
     */
    public boolean isUptakeBusy() {
        return uptakeLTimedActive || uptakeRTimedActive;
    }

    /**
     * Stop both uptake servos immediately
     */
    public void stopUptake() {
        stopUptakeL();
        stopUptakeR();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // INJECTOR SERVO CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set injector left servo power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setInjectorLeftPower(double power) {
        if (!enabled) return;
        injectorLeftTimedActive = false;
        injectorServoLeft.setPower(power);
    }

    /**
     * Run injector left servo for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setInjectorLeftTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Injector L duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        injectorLeftTimedPower = power;
        injectorLeftEndTime = System.currentTimeMillis() + durationMs;
        injectorLeftTimedActive = true;
    }

    /**
     * Check if injector left servo has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isInjectorLeftBusy() {
        return injectorLeftTimedActive;
    }

    /**
     * Stop injector left servo immediately
     */
    public void stopInjectorLeft() {
        injectorLeftTimedActive = false;
        injectorServoLeft.setPower(0);
    }

    /**
     * Set injector right servo power (immediate, cancels any timed movement)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setInjectorRightPower(double power) {
        if (!enabled) return;
        injectorRightTimedActive = false;
        injectorServoRight.setPower(power);
    }

    /**
     * Run injector right servo for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setInjectorRightTimed(double power, long durationMs) {
        if (!enabled) return;

        if (durationMs > GLOBAL_TIMEOUT) {
            telemetry.addData("⚠️ WARNING", "Injector R duration exceeds global timeout");
            durationMs = GLOBAL_TIMEOUT;
        }

        injectorRightTimedPower = power;
        injectorRightEndTime = System.currentTimeMillis() + durationMs;
        injectorRightTimedActive = true;
    }

    /**
     * Check if injector right servo has an active timed movement
     * @return true if timed movement is in progress
     */
    public boolean isInjectorRightBusy() {
        return injectorRightTimedActive;
    }

    /**
     * Stop injector right servo immediately
     */
    public void stopInjectorRight() {
        injectorRightTimedActive = false;
        injectorServoRight.setPower(0);
    }

    /**
     * Set both injector servos power (immediate, cancels any timed movements)
     * positive power for left, negative for right as they are mounted oppositely.
     * Positive method input power means both servos pull in artifacts from the front intake.
     * Negative method input power means both servos pull in artifacts from the back intake.
     * Note: If there is an artifact already in the center slot(where the injectors pull artifacts to), any artifacts being pulled in will push other artifacts out.
     * @param power Power level (-1.0 to 1.0)
     */
    public void setInjectorPower(double power) {
        setInjectorLeftPower(power);
        setInjectorRightPower(-power);
    }

    /**
     * Run both injector servos for specified duration (non-blocking)
     * @param power Power level (-1.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void setInjectorTimed(double power, long durationMs) {
        setInjectorLeftTimed(power, durationMs);
        setInjectorRightTimed(-power, durationMs);
    }

    /**
     * Check if either injector servo has an active timed movement
     * @return true if any injector servo is busy
     */
    public boolean isInjectorBusy() {
        return injectorLeftTimedActive || injectorRightTimedActive;
    }

    /**
     * Stop both injector servos immediately
     */
    public void stopInjector() {
        stopInjectorLeft();
        stopInjectorRight();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // COMPLEX SUBSYSTEM OPERATIONS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Run entire front intake subsystem for manual artifact collection
     * Includes: roller motor, transfer servo, and bottom intake servo
     *
     * @param enable If false, stops all components and returns immediately
     * @param power Power level for all components (0.0 to 1.0 for intake, negative for eject)
     */
    public void runFrontIntake(boolean enable, double power) {
        if (!enable) {
            setFrontRollerPower(0);
            setFrontTransferPower(0);
            setFrontBottomIntakePower(0);
            return;
        }

        // Run all front intake components at specified power
        setFrontRollerPower(power);
        setFrontTransferPower(-power);
        setFrontBottomIntakePower(-power); // Bottom intake runs opposite direction
    }

    /**
     * Run entire front intake subsystem for specified duration (non-blocking)
     *
     * @param enable If false, stops all components and returns immediately
     * @param power Power level for all components (0.0 to 1.0 for intake, negative for eject)
     * @param durationMs Duration in milliseconds
     */
    public void runFrontIntakeTimed(boolean enable, double power, long durationMs) {
        if (!enable) {
            setFrontRollerPower(0);
            setFrontTransferPower(0);
            setFrontBottomIntakePower(0);
            return;
        }

        setFrontRollerTimed(power, durationMs);
        setFrontTransferTimed(-power, durationMs);
        setFrontBottomIntakeTimed(-power, durationMs); // Bottom intake runs opposite direction
    }

    /**
     * Check if any front intake component is busy with a timed movement
     * @return true if any component is busy
     */
    public boolean isFrontIntakeBusy() {
        return frontRollerTimedActive || frontTransferTimedActive || frontBottomIntakeTimedActive;
    }

    /**
     * Stop entire front intake subsystem immediately
     */
    public void stopFrontIntake() {
        stopFrontRoller();
        stopFrontTransfer();
        stopFrontBottomIntake();
    }

    /**
     * Run entire back intake subsystem for manual artifact collection
     * Includes: roller motor, transfer servo, and bottom intake servo
     *
     * @param enable If false, stops all components and returns immediately
     * @param power Power level for all components (0.0 to 1.0 for intake, negative for eject)
     */
    public void runBackIntake(boolean enable, double power) {
        if (!enable) {
            setBackRollerPower(0);
            setBackTransferPower(0);
            setBackBottomIntakePower(0);
            return;
        }

        // Run all back intake components at specified power
        setBackRollerPower(power);
        setBackTransferPower(-power);
        setBackBottomIntakePower(-power); // Bottom intake runs opposite direction
    }

    /**
     * Run entire back intake subsystem for specified duration (non-blocking)
     *
     * @param enable If false, stops all components and returns immediately
     * @param power Power level for all components (0.0 to 1.0 for intake, negative for eject)
     * @param durationMs Duration in milliseconds
     */
    public void runBackIntakeTimed(boolean enable, double power, long durationMs) {
        if (!enable) {
            setBackRollerPower(0);
            setBackTransferPower(0);
            setBackBottomIntakePower(0);
            return;
        }

        setBackRollerTimed(power, durationMs);
        setBackTransferTimed(-power, durationMs);
        setBackBottomIntakeTimed(-power, durationMs); // Bottom intake runs opposite direction
    }

    /**
     * Check if any back intake component is busy with a timed movement
     * @return true if any component is busy
     */
    public boolean isBackIntakeBusy() {
        return backRollerTimedActive || backTransferTimedActive || backBottomIntakeTimedActive;
    }

    /**
     * Stop entire back intake subsystem immediately
     */
    public void stopBackIntake() {
        stopBackRoller();
        stopBackTransfer();
        stopBackBottomIntake();
    }

    /**
     * Run both intake subsystems simultaneously
     *
     * @param enable If false, stops all components and returns immediately
     * @param power Power level for all components (0.0 to 1.0 for intake, negative for eject)
     */
    public void runBothIntakes(boolean enable, double power) {
        runFrontIntake(enable, power);
        runBackIntake(enable, power);
    }

    /**
     * Run both intake subsystems for specified duration (non-blocking)
     *
     * @param enable If false, stops all components and returns immediately
     * @param power Power level for all components (0.0 to 1.0 for intake, negative for eject)
     * @param durationMs Duration in milliseconds
     */
    public void runBothIntakesTimed(boolean enable, double power, long durationMs) {
        runFrontIntakeTimed(enable, power, durationMs);
        runBackIntakeTimed(enable, power, durationMs);
    }

    /**
     * Check if any intake component is busy with a timed movement
     * @return true if any component is busy
     */
    public boolean isAnyIntakeBusy() {
        return isFrontIntakeBusy() || isBackIntakeBusy();
    }

    /**
     * Stop all intake subsystems immediately
     */
    public void stopAllIntakes() {
        stopFrontIntake();
        stopBackIntake();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GLOBAL CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Stop all motors and servos immediately
     * Cancels all timed movements
     */
    public void stopAll() {
        stopFrontIntake();
        stopBackIntake();
        stopUptake();
        stopInjector();
    }

    /**
     * Check if any actuator has an active timed movement
     * @return true if any component is busy
     */
    public boolean isAnyBusy() {
        return frontRollerTimedActive || backRollerTimedActive ||
               frontBottomIntakeTimedActive || backBottomIntakeTimedActive ||
               frontTransferTimedActive || backTransferTimedActive ||
               uptakeLTimedActive || uptakeRTimedActive ||
               injectorLeftTimedActive || injectorRightTimedActive;
    }

    /**
     * Wait for all timed movements to complete (blocking)
     * Use sparingly - prefer checking isBusy() in your loop
     *
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return true if all movements completed, false if timeout
     */
    public boolean waitForAllComplete(long timeoutMs) {
        long startTime = System.currentTimeMillis();
        while (isAnyBusy() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            update();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                return false;
            }
        }
        return !isAnyBusy();
    }

    /**
     * Enable/disable the helper
     * When disabled, all methods return immediately without action
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            stopAll();
        }
    }

    /**
     * Check if helper is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Get comprehensive status telemetry
     */
    public void addTelemetry() {
        telemetry.addData("═══ Basic Indexing Helper ═══", "");
        telemetry.addData("Enabled", enabled);
        telemetry.addData("Any Busy", isAnyBusy());

        telemetry.addData("Front Roller", frontRollerTimedActive ? "TIMED" : "IDLE");
        telemetry.addData("Back Roller", backRollerTimedActive ? "TIMED" : "IDLE");
        telemetry.addData("Front Transfer", frontTransferTimedActive ? "TIMED" : "IDLE");
        telemetry.addData("Back Transfer", backTransferTimedActive ? "TIMED" : "IDLE");
        telemetry.addData("Uptake L", uptakeLTimedActive ? "TIMED" : "IDLE");
        telemetry.addData("Uptake R", uptakeRTimedActive ? "TIMED" : "IDLE");
        telemetry.addData("Injector L", injectorLeftTimedActive ? "TIMED" : "IDLE");
        telemetry.addData("Injector R", injectorRightTimedActive ? "TIMED" : "IDLE");
        telemetry.addData("Transfer State", transferSequenceState);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // COMPLEX TRANSFER SEQUENCES
    // ═══════════════════════════════════════════════════════════════════════

    // Transfer sequence tracking
    private enum TransferSequenceState {
        IDLE,
        UN_PREPOSITIONING,
        TRANSFERRING,
        PREPOSITIONING,
        COMPLETE
    }

    private TransferSequenceState transferSequenceState = TransferSequenceState.IDLE;
    private long transferSequenceStartTime = 0;
    private boolean transferSequenceActive = false;
    private String currentTransferType = "NONE";

    // Pre-positioning constants
    private static final long PREPOSITION_DURATION_MS = 600;
    private static final long UN_PREPOSITION_DURATION_MS = 800;
    private static final double PREPOSITION_POWER = 1.0;
    private static final double UN_PREPOSITION_POWER = -1.0;

    // Transfer constants
    private static final long DEFAULT_TRANSFER_DURATION_MS = 2500;  // 2.5 seconds

    /**
     * Pre-position artifacts in uptake servos
     * Runs uptake servos forward for 400ms to position artifacts for shooting
     */
    public void prePositionArtifacts() {
        if (!enabled) return;
        setUptakeTimed(PREPOSITION_POWER, PREPOSITION_DURATION_MS);
        telemetry.addData("Uptake", "Pre-positioning artifacts");
    }

    /**
     * Un-pre-position artifacts in uptake servos
     * Runs uptake servos in reverse for 500ms to clear the uptake area
     */
    public void unPrePositionArtifacts() {
        if (!enabled) return;
        setUptakeTimed(UN_PREPOSITION_POWER, UN_PREPOSITION_DURATION_MS);
        telemetry.addData("Uptake", "Un-pre-positioning artifacts");
    }

    /**
     * Check if uptake pre-positioning/un-pre-positioning is in progress
     * @return true if uptake servos are busy
     */
    public boolean isPrePositioning() {
        return isUptakeBusy();
    }

    /**
     * Transfer artifact from front intake to center (timed sequence)
     * Automatically handles un-pre-positioning before and pre-positioning after
     *
     * Sequence:
     * 1. Un-pre-position uptake (500ms)
     * 2. Run front intake + injectors (durationMs)
     * 3. Pre-position uptake (400ms)
     *
     * @param durationMs Duration to run intake and injectors (default: 2500ms)
     */
    public void transferFrontIntakeToCenterTimed(long durationMs) {
        if (!enabled) return;

        if (transferSequenceActive) {
            telemetry.addData("⚠️ WARNING", "Transfer already in progress");
            return;
        }
        Dbg.d(LogGroup.TRANSFER, "Starting transferFrontIntakeToCenterTimed");

        transferSequenceActive = true;
        transferSequenceState = TransferSequenceState.UN_PREPOSITIONING;
        transferSequenceStartTime = System.currentTimeMillis();
        currentTransferType = "FRONT_TIMED:" + durationMs;

        // Start un-pre-positioning
        unPrePositionArtifacts();

        telemetry.addData("Transfer", "Front intake → Center (timed)");
        telemetry.addData("Duration", durationMs + "ms");
        Dbg.d(LogGroup.TRANSFER, "TransferSequenceState.UN_PREPOSITIONING");
    }

    /**
     * Transfer artifact from front intake to center (timed, default duration)
     * Uses default 2.5 second transfer time
     */
    public void transferFrontIntakeToCenterTimed() {
        transferFrontIntakeToCenterTimed(DEFAULT_TRANSFER_DURATION_MS);
    }

    /**
     * Transfer artifact from back intake to center (timed sequence)
     * Automatically handles un-pre-positioning before and pre-positioning after
     *
     * Sequence:
     * 1. Un-pre-position uptake (500ms)
     * 2. Run back intake + injectors (durationMs)
     * 3. Pre-position uptake (400ms)
     *
     * @param durationMs Duration to run intake and injectors (default: 2500ms)
     */
    public void transferBackIntakeToCenterTimed(long durationMs) {
        if (!enabled) return;

        if (transferSequenceActive) {
            telemetry.addData("⚠️ WARNING", "Transfer already in progress");
            return;
        }
        Dbg.d(LogGroup.TRANSFER, "Starting transferBackIntakeToCenterTimed");

        transferSequenceActive = true;
        transferSequenceState = TransferSequenceState.UN_PREPOSITIONING;
        transferSequenceStartTime = System.currentTimeMillis();
        currentTransferType = "BACK_TIMED:" + durationMs;

        // Start un-pre-positioning
        unPrePositionArtifacts();

        telemetry.addData("Transfer", "Back intake → Center (timed)");
        telemetry.addData("Duration", durationMs + "ms");
        Dbg.d(LogGroup.TRANSFER, "Started transferBackIntakeToCenterTimed");
    }

    /**
     * Transfer artifact from back intake to center (timed, default duration)
     * Uses default 2.5 second transfer time
     */
    public void transferBackIntakeToCenterTimed() {
        transferBackIntakeToCenterTimed(DEFAULT_TRANSFER_DURATION_MS);
    }

    /**
     * Start manual transfer from front intake to center
     * Automatically un-pre-positions before starting
     * Call with button state every loop
     *
     * @param buttonPressed true while button is held down
     */
    public void transferFrontIntakeToCenterManual(boolean buttonPressed) {
        if (!enabled) return;

        if (buttonPressed) {
            if (!transferSequenceActive) {
                // Starting new manual transfer
                transferSequenceActive = true;
                transferSequenceState = TransferSequenceState.UN_PREPOSITIONING;
                transferSequenceStartTime = System.currentTimeMillis();
                currentTransferType = "FRONT_MANUAL";

                // Un-pre-position first
                unPrePositionArtifacts();
                telemetry.addData("Transfer", "Front manual - Un-prepositioning");
            } else if (currentTransferType.equals("FRONT_MANUAL")) {
                // Only proceed if this is OUR transfer
                if (transferSequenceState == TransferSequenceState.UN_PREPOSITIONING) {
                    // Check if un-pre-positioning is done
                    if (!isUptakeBusy()) {
                        transferSequenceState = TransferSequenceState.TRANSFERRING;
                        telemetry.addData("Transfer", "Front manual - Transferring");
                        Dbg.d(LogGroup.TRANSFER, "TransferSequenceState.TRANSFERRING");
                    }
                } else if (transferSequenceState == TransferSequenceState.TRANSFERRING) {
                    // Run intake and injectors while button is held
                    runFrontIntake(true, 1.0);
                    setInjectorPower(1.0);
                }
            }
            // If another transfer is active, do nothing
        } else {
            // Button released - only process if this is our transfer
            if (transferSequenceActive && currentTransferType.equals("FRONT_MANUAL")) {
                if (transferSequenceState == TransferSequenceState.TRANSFERRING) {
                    // Stop intake and injectors
                    runFrontIntake(false, 0);
                    setInjectorPower(0);

                    // Start pre-positioning
                    transferSequenceState = TransferSequenceState.PREPOSITIONING;
                    prePositionArtifacts();
                    telemetry.addData("Transfer", "Front manual - Pre-positioning");
                    Dbg.d(LogGroup.TRANSFER, "TransferSequenceState.PREPOSITIONING");
                } else if (transferSequenceState == TransferSequenceState.PREPOSITIONING) {
                    // Wait for pre-positioning to complete (uptake timed movement)
                    // The uptake servos are running for PREPOSITION_DURATION_MS (400ms)
                    // We wait for them to finish, then transfer is complete
                    if (!isUptakeBusy()) {
                        // Prepositioning complete - artifact is positioned and ready to fire
                        transferSequenceState = TransferSequenceState.COMPLETE;
                        transferSequenceActive = false;
                        currentTransferType = "NONE";
                        Dbg.d(LogGroup.TRANSFER, "PREPOSITIONING complete - transfer finished");
                    } else {
                        Dbg.d(LogGroup.TRANSFER, "PREPOSITIONING (waiting for uptake timer)");
                    }
                } else if (transferSequenceState == TransferSequenceState.UN_PREPOSITIONING) {
                    // Button released before transfer started - abort
                    transferSequenceActive = false;
                    transferSequenceState = TransferSequenceState.IDLE;
                    currentTransferType = "NONE";
                    setUptakePower(0);
                }
            }
        }
    }

    /**
     * Start manual transfer from back intake to center
     * Automatically un-pre-positions before starting
     * Call with button state every loop
     *
     * @param buttonPressed true while button is held down
     */
    public void transferBackIntakeToCenterManual(boolean buttonPressed) {
        if (!enabled) return;

        if (buttonPressed) {
            if (!transferSequenceActive) {
                // Starting new manual transfer
                transferSequenceActive = true;
                transferSequenceState = TransferSequenceState.UN_PREPOSITIONING;
                transferSequenceStartTime = System.currentTimeMillis();
                currentTransferType = "BACK_MANUAL";

                // Un-pre-position first
                unPrePositionArtifacts();
                telemetry.addData("Transfer", "Back manual - Un-prepositioning");
                Dbg.d(LogGroup.TRANSFER, "TransferSequenceState.UN_PREPOSITIONING");
            } else if (currentTransferType.equals("BACK_MANUAL")) {
                // Only proceed if this is OUR transfer
                if (transferSequenceState == TransferSequenceState.UN_PREPOSITIONING) {
                    // Check if un-pre-positioning is done
                    if (!isUptakeBusy()) {
                        transferSequenceState = TransferSequenceState.TRANSFERRING;
                        telemetry.addData("Transfer", "Back manual - Transferring");
                        Dbg.d(LogGroup.TRANSFER, "TransferSequenceState.TRANSFERRING");
                    }
                } else if (transferSequenceState == TransferSequenceState.TRANSFERRING) {
                    // Run intake and injectors while button is held
                    runBackIntake(true, 1.0);
                    setInjectorPower(-1.0);
                }
            }
            // If another transfer is active, do nothing
        } else {
            // Button released - only process if this is our transfer
            if (transferSequenceActive && currentTransferType.equals("BACK_MANUAL")) {
                if (transferSequenceState == TransferSequenceState.TRANSFERRING) {
                    // Stop intake and injectors
                    runBackIntake(false, 0);
                    setInjectorPower(0);

                    // Start pre-positioning
                    transferSequenceState = TransferSequenceState.PREPOSITIONING;
                    prePositionArtifacts();
                    telemetry.addData("Transfer", "Back manual - Pre-positioning");
                    Dbg.d(LogGroup.TRANSFER, "TransferSequenceState.PREPOSITIONING");
                } else if (transferSequenceState == TransferSequenceState.PREPOSITIONING) {
                    // Check if pre-positioning is done
                    if (!isUptakeBusy()) {
                        transferSequenceState = TransferSequenceState.COMPLETE;
                        transferSequenceActive = false;
                        currentTransferType = "NONE";
                    }
                } else if (transferSequenceState == TransferSequenceState.UN_PREPOSITIONING) {
                    // Button released before transfer started - abort
                    transferSequenceActive = false;
                    transferSequenceState = TransferSequenceState.IDLE;
                    currentTransferType = "NONE";
                    setUptakePower(0);
                }
            }
        }
    }

    /**
     * Update timed transfer sequences
     * This is called automatically by update() - do not call directly
     */
    private void updateTransferSequences() {
        if (!transferSequenceActive) return;
        if (!currentTransferType.contains("TIMED")) return;

        // Parse transfer duration from currentTransferType
        String[] parts = currentTransferType.split(":");
        if (parts.length != 2) return;

        long transferDuration;
        try {
            transferDuration = Long.parseLong(parts[1]);
        } catch (NumberFormatException e) {
            transferDuration = DEFAULT_TRANSFER_DURATION_MS;
        }

        boolean isFrontTransfer = parts[0].equals("FRONT_TIMED");

        switch (transferSequenceState) {
            case UN_PREPOSITIONING:
                // Wait for un-pre-positioning to complete
                if (!isUptakeBusy()) {
                    transferSequenceState = TransferSequenceState.TRANSFERRING;
                    transferSequenceStartTime = System.currentTimeMillis();

                    // Start intake and injectors
                    if (isFrontTransfer) {
                        runFrontIntakeTimed(true, 1.0, transferDuration);
                        setInjectorTimed(1.0, transferDuration);
                    } else {
                        runBackIntakeTimed(true, 1.0, transferDuration);
                        setInjectorTimed(-1.0, transferDuration);
                    }

                }
                break;

            case TRANSFERRING:
                // Wait for transfer to complete
                Dbg.d(LogGroup.TRANSFER, "TRANSFERRING - Elapsed Time: %dms", (System.currentTimeMillis() - transferSequenceStartTime));
                if (!isAnyIntakeBusy() && !isInjectorBusy()) {
                    transferSequenceState = TransferSequenceState.PREPOSITIONING;

                    // Start pre-positioning
                    prePositionArtifacts();
                }
                break;

            case PREPOSITIONING:
                // Wait for pre-positioning to complete (uptake timed movement)
                // The uptake servos are running for PREPOSITION_DURATION_MS (400ms)
                // We wait for them to finish, then transfer is complete
                if (!isUptakeBusy()) {
                    // Prepositioning complete - artifact is positioned and ready to fire
                    transferSequenceState = TransferSequenceState.COMPLETE;
                    transferSequenceActive = false;
                    currentTransferType = "NONE";
                    telemetry.addData("Transfer", "✅ Complete");
                    Dbg.d(LogGroup.TRANSFER, "PREPOSITIONING complete - transfer finished");
                } else {
                    Dbg.d(LogGroup.TRANSFER, "PREPOSITIONING (waiting for uptake timer)");
                }
                break;

            case COMPLETE:
            case IDLE:
                transferSequenceActive = false;
                break;
        }
    }

    /**
     * Check if a transfer sequence is currently active
     * @return true if any transfer sequence is in progress
     */
    public boolean isTransferActive() {
        return transferSequenceActive;
    }

    /**
     * Get current transfer sequence state
     * @return current state as string
     */
    public String getTransferState() {
        return transferSequenceState.toString();
    }

    /**
     * Get current transfer type
     * @return transfer type description
     */
    public String getTransferType() {
        return currentTransferType;
    }

    /**
     * Cancel any active transfer sequence
     * Stops all motors and servos
     */
    public void cancelTransfer() {
        transferSequenceActive = false;
        transferSequenceState = TransferSequenceState.IDLE;
        currentTransferType = "NONE";
        stopAll();
        telemetry.addData("Transfer", "❌ Cancelled");
    }
}
