package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * DecodeHelper - DECODE Season Game-Specific Shooter Subsystem
 *
 * This class manages the dual-motor friction flywheel shooter system with:
 * - Continuous RPM tracking and synchronization
 * - PID-based velocity control for both motors
 * - Warm-up mode for reduced spin-up latency
 * - Controlled firing sequences with safety checks
 * - Clean API for external control (no gamepad binding)
 *
 * Hardware: 2x DC Motors with encoders driving friction flywheels
 * Max RPM: 6000 | Gear Ratio: 1:1
 *
 * INTAKE: WIP - Will be added when hardware is finalized
 * INDEXING: WIP - Semi-indexer functionality to be implemented
 */
public class DecodeHelper {

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE COMPONENTS
    // ═══════════════════════════════════════════════════════════════════════

    private final DcMotor leftShooterMotor;
    private final DcMotor rightShooterMotor;
    private final Telemetry telemetry;

    // ═══════════════════════════════════════════════════════════════════════
    // STATE MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════

    private ShooterState currentState = ShooterState.IDLE;
    private double targetRPM = 0.0;
    private ShooterConfig.ShooterPreset activePreset = null;

    // RPM tracking
    private double leftRPM = 0.0;
    private double rightRPM = 0.0;
    private double averageRPM = 0.0;
    private double rpmSyncError = 0.0;

    // Encoder tracking
    private int lastLeftPosition = 0;
    private int lastRightPosition = 0;
    private long lastUpdateTime = 0;

    // PID controllers for each motor
    private final PIDController leftPID;
    private final PIDController rightPID;

    // Timing
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime stabilizationTimer = new ElapsedTime();
    private final ElapsedTime firingTimer = new ElapsedTime();
    private long lastShotTime = 0;
    private long stabilizationStartTime = 0;  // Manual tracking for stabilization
    private static final double FIRING_SEQUENCE_TIME = 0.2;  // 200ms for feed sequence

    // Status flags
    private boolean atTargetRPM = false;
    private boolean rpmStabilized = false;
    private boolean transitioningFromWarmup = false;  // Prevents premature shooting after warmup

    // Sync error tracking for improved detection
    private final ElapsedTime syncErrorTimer = new ElapsedTime();
    private double syncErrorAccumulator = 0;
    private int syncErrorSampleCount = 0;
    private boolean syncErrorDetected = false;
    private static final int SYNC_ERROR_SAMPLES = 10;  // Average over 10 samples
    private static final double SYNC_ERROR_PERSIST_TIME = 0.5;  // Must persist for 500ms

    // Button state tracking (for edge detection)
    private boolean prevShootButton = false;
    private boolean prevWarmupButton = false;

    /**
     * Shooter operational states
     */
    public enum ShooterState {
        IDLE,           // Motors off
        WARMUP,         // Spinning at reduced speed (65% target)
        SPINNING_UP,    // Accelerating to target RPM
        READY,          // At target RPM and stabilized
        FIRING,         // Currently firing
        RECOVERY,       // Recovering RPM after shot
        ERROR           // Safety error detected
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new DecodeHelper shooter subsystem
     *
     * @param leftMotor Left shooter motor
     * @param rightMotor Right shooter motor
     * @param telemetry Telemetry for status reporting
     */
    public DecodeHelper(DcMotor leftMotor, DcMotor rightMotor, Telemetry telemetry) {
        this.leftShooterMotor = leftMotor;
        this.rightShooterMotor = rightMotor;
        this.telemetry = telemetry;

        // Configure motors
        configureMotors();

        // Initialize PID controllers
        leftPID = new PIDController(
            ShooterConfig.PID_KP,
            ShooterConfig.PID_KI,
            ShooterConfig.PID_KD,
            ShooterConfig.PID_KF
        );

        rightPID = new PIDController(
            ShooterConfig.PID_KP,
            ShooterConfig.PID_KI,
            ShooterConfig.PID_KD,
            ShooterConfig.PID_KF
        );

        // Initialize encoder positions
        lastLeftPosition = leftShooterMotor.getCurrentPosition();
        lastRightPosition = rightShooterMotor.getCurrentPosition();
        lastUpdateTime = System.currentTimeMillis();
    }

    /**
     * Simplified constructor using AuroraHardwareConfig
     */
    public DecodeHelper(AuroraHardwareConfig hardware, Telemetry telemetry) {
        this(hardware.getLeftShooterMotor(), hardware.getRightShooterMotor(), telemetry);
    }

    /**
     * Configure motor settings
     */
    private void configureMotors() {
        // STOP MOTORS FIRST - prevent auto-start
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);

        // Set run mode - RUN_USING_ENCODER is required for velocity control
        // DO NOT change mode if already set by AuroraHardwareConfig to avoid mode-switch glitches
        if (leftShooterMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (rightShooterMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set zero power behavior (coast for flywheels)
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set directions
        leftShooterMotor.setDirection(
            ShooterConfig.REVERSE_LEFT_MOTOR ?
            DcMotorSimple.Direction.REVERSE :
            DcMotorSimple.Direction.FORWARD
        );

        rightShooterMotor.setDirection(
            ShooterConfig.REVERSE_RIGHT_MOTOR ?
            DcMotorSimple.Direction.REVERSE :
            DcMotorSimple.Direction.FORWARD
        );

        // STOP MOTORS AGAIN - ensure they stay off
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MAIN UPDATE LOOP
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update shooter subsystem - Call this in your OpMode loop
     * This method handles RPM measurement, PID control, and state management
     */
    public void update() {
        // Update RPM measurements
        updateRPMMeasurements();

        // Apply PID control based on current state
        switch (currentState) {
            case IDLE:
                // Motors off
                setMotorPowers(0, 0);
                break;

            case WARMUP:
                // Maintain warm-up RPM
                double warmupTarget = ShooterConfig.getWarmupRPM(targetRPM);
                applyPIDControl(warmupTarget);
                checkIfAtTarget(warmupTarget, ShooterConfig.RPM_TOLERANCE * 1.5);
                break;

            case SPINNING_UP:
                // Accelerate to target
                applyPIDControl(targetRPM);
                checkIfAtTarget(targetRPM, getRPMTolerance());

                // If transitioning from warmup, check if we've reached full RPM
                if (transitioningFromWarmup && atTargetRPM) {
                    transitioningFromWarmup = false;  // Transition complete
                }

                // Check for spinup timeout
                if (spinUpTimer.milliseconds() > getMaxSpinupTime()) {
                    telemetry.addLine("⚠️ WARNING: Spin-up timeout - still spinning");
                    // Don't set ERROR state, just warn and continue
                    // currentState = ShooterState.ERROR;
                }

                // Transition to READY when at target AND stabilized
                if (atTargetRPM && rpmStabilized) {
                    currentState = ShooterState.READY;
                }
                break;

            case READY:
                // Maintain target RPM
                applyPIDControl(targetRPM);
                checkIfAtTarget(targetRPM, getRPMTolerance());

                // Check if we fell out of tolerance
                if (!atTargetRPM) {
                    currentState = ShooterState.SPINNING_UP;
                    stabilizationTimer.reset();
                }
                break;

            case FIRING:
                // Maintain RPM during firing
                applyPIDControl(targetRPM);

                // Check if firing sequence is complete
                if (firingTimer.seconds() >= FIRING_SEQUENCE_TIME) {
                    currentState = ShooterState.RECOVERY;
                    stabilizationTimer.reset();
                }
                break;

            case RECOVERY:
                // Recover RPM after shot
                applyPIDControl(targetRPM);
                checkIfAtTarget(targetRPM, getRPMTolerance());

                if (atTargetRPM) {
                    currentState = ShooterState.READY;
                }
                break;

            case ERROR:
                // WARNING ONLY - Don't stop motors, just maintain current state
                // The error is now just a warning, shooter continues
                // Safety stop disabled for testing
                // setMotorPowers(0, 0);

                // Continue PID control even in error state
                applyPIDControl(targetRPM);
                break;
        }

        // Check for safety violations - COMPLETELY DISABLED FOR TESTING
        // Nothing should stop the shooter automatically now
        // if (ShooterConfig.ENABLE_SAFETY_CHECKS) {
        //     checkSafetyConditions();
        // }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // RPM MEASUREMENT
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update RPM measurements from encoder data
     */
    private void updateRPMMeasurements() {
        long currentTime = System.currentTimeMillis();
        long deltaTime = currentTime - lastUpdateTime;

        // Only update at specified rate
        if (deltaTime < ShooterConfig.UPDATE_RATE_MS) {
            return;
        }

        // Get current encoder positions
        int currentLeftPosition = leftShooterMotor.getCurrentPosition();
        int currentRightPosition = rightShooterMotor.getCurrentPosition();

        // Calculate position deltas
        int leftDelta = currentLeftPosition - lastLeftPosition;
        int rightDelta = currentRightPosition - lastRightPosition;

        // Calculate RPM: (ticks / time) * (60000 ms/min) / (ticks/rev)
        double deltaSeconds = deltaTime / 1000.0;
        leftRPM = (leftDelta / ShooterConfig.TICKS_PER_REVOLUTION) * (60.0 / deltaSeconds);
        rightRPM = (rightDelta / ShooterConfig.TICKS_PER_REVOLUTION) * (60.0 / deltaSeconds);

        // Calculate average and sync error
        averageRPM = (leftRPM + rightRPM) / 2.0;
        rpmSyncError = Math.abs(leftRPM - rightRPM);

        // Update tracking variables
        lastLeftPosition = currentLeftPosition;
        lastRightPosition = currentRightPosition;
        lastUpdateTime = currentTime;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PID CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Apply PID control to both motors with synchronization
     */
    private void applyPIDControl(double targetRPM) {
        // Calculate base PID outputs
        double leftOutput = leftPID.calculate(leftRPM, targetRPM);
        double rightOutput = rightPID.calculate(rightRPM, targetRPM);

        // Apply synchronization correction
        double syncCorrection = (rightRPM - leftRPM) * ShooterConfig.SYNC_CORRECTION_FACTOR * 0.001;
        leftOutput += syncCorrection;
        rightOutput -= syncCorrection;

        // Clamp outputs
        leftOutput = clampPower(leftOutput);
        rightOutput = clampPower(rightOutput);

        // Apply to motors
        setMotorPowers(leftOutput, rightOutput);
    }

    /**
     * Check if shooter is at target RPM
     */
    private void checkIfAtTarget(double target, double tolerance) {
        boolean wasAtTarget = atTargetRPM;

        // Check if both motors are within tolerance
        boolean leftAtTarget = Math.abs(leftRPM - target) < tolerance;
        boolean rightAtTarget = Math.abs(rightRPM - target) < tolerance;
        boolean syncOk = rpmSyncError < ShooterConfig.MAX_RPM_SYNC_ERROR;

        atTargetRPM = leftAtTarget && rightAtTarget && syncOk;

        // Simple stabilization logic using system time
        long currentTime = System.currentTimeMillis();

        if (!atTargetRPM) {
            // Not at target - reset stabilization
            rpmStabilized = false;
            stabilizationStartTime = 0;
        } else if (!wasAtTarget) {
            // Just reached target - start tracking stabilization time
            stabilizationStartTime = currentTime;
            rpmStabilized = false;
        } else if (stabilizationStartTime > 0) {
            // Continuously at target - check elapsed time
            long elapsedTime = currentTime - stabilizationStartTime;
            if (elapsedTime >= ShooterConfig.RPM_STABILIZATION_TIME_MS) {
                rpmStabilized = true;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API - CONTROL METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set target RPM directly
     * Note: This does NOT automatically start the motors. Call spinUp() or enableWarmup() to start.
     */
    public void setTargetRPM(double rpm) {
        rpm = ShooterConfig.clampRPM(rpm);

        if (rpm < ShooterConfig.MIN_LAUNCH_RPM && rpm > 0) {
            telemetry.addLine("⚠️ RPM below minimum launch speed");
        }

        this.targetRPM = rpm;
        this.activePreset = null;

        // Apply gain scheduling - update PID gains based on target RPM
        updatePIDGains(rpm);

        // Only auto-disable if setting RPM to 0
        if (rpm == 0) {
            disableShooter();
        }
        // Note: We do NOT auto-spinUp here anymore to prevent unwanted motor starts
        // User must explicitly call spinUp() or enableWarmup() to start motors
    }

    /**
     * Update PID controller gains based on target RPM (gain scheduling)
     */
    private void updatePIDGains(double targetRPM) {
        double kp = ShooterConfig.getKP(targetRPM);
        double ki = ShooterConfig.getKI(targetRPM);
        double kd = ShooterConfig.getKD(targetRPM);
        double kf = ShooterConfig.getKF(targetRPM);

        leftPID.updateGains(kp, ki, kd, kf);
        rightPID.updateGains(kp, ki, kd, kf);
    }

    /**
     * Set target using a preset configuration
     */
    public void setPreset(ShooterConfig.ShooterPreset preset) {
        this.activePreset = preset;
        setTargetRPM(preset.getTargetRPM());
    }

    /**
     * Enable warm-up mode (65% of target RPM)
     */
    public void enableWarmup() {
        if (targetRPM == 0) {
            telemetry.addLine("⚠️ Cannot warm up: no target RPM set");
            return;
        }

        currentState = ShooterState.WARMUP;
        leftPID.reset();
        rightPID.reset();
    }

    /**
     * Spin up to target RPM
     */
    public void spinUp() {
        if (targetRPM == 0) {
            telemetry.addLine("⚠️ Cannot spin up: no target RPM set");
            return;
        }

        // Check if we're transitioning from warmup
        if (currentState == ShooterState.WARMUP) {
            transitioningFromWarmup = true;
        }

        currentState = ShooterState.SPINNING_UP;
        spinUpTimer.reset();
        stabilizationTimer.reset();
        leftPID.reset();
        rightPID.reset();
        atTargetRPM = false;
        rpmStabilized = false;
    }

    /**
     * Disable shooter (stop motors)
     */
    public void disableShooter() {
        currentState = ShooterState.IDLE;
        setMotorPowers(0, 0);
        leftPID.reset();
        rightPID.reset();
        atTargetRPM = false;
        rpmStabilized = false;
    }

    /**
     * Fire sequence - launches artifacts
     * Returns true if fire command was accepted, false if conditions not met
     */
    public boolean fire() {
        // CRITICAL: Don't fire if transitioning from warmup (not at full RPM yet)
        if (transitioningFromWarmup) {
            telemetry.addLine("❌ Cannot fire: Transitioning from warmup");
            return false;
        }

        // Check if ready to fire
        if (currentState != ShooterState.READY) {
            telemetry.addLine("❌ Cannot fire: Not ready (State: " + currentState + ")");
            return false;
        }

        // Check minimum shot interval
        long timeSinceLastShot = System.currentTimeMillis() - lastShotTime;
        long minInterval = activePreset != null ? activePreset.getShotIntervalMs() : ShooterConfig.MIN_SHOT_INTERVAL_MS;

        if (timeSinceLastShot < minInterval) {
            telemetry.addLine("❌ Cannot fire: Too soon (" + (minInterval - timeSinceLastShot) + "ms)");
            return false;
        }

        // Execute fire sequence
        currentState = ShooterState.FIRING;
        lastShotTime = System.currentTimeMillis();
        shotTimer.reset();
        firingTimer.reset();  // Start firing sequence timer

        telemetry.addLine("🎯 FIRING!");

        // Note: Actual feeding mechanism (servos/indexer) would be triggered here
        // This will be implemented when intake/indexer hardware is finalized

        // State will transition to RECOVERY in update() after FIRING_SEQUENCE_TIME

        return true;
    }


    /**
     * Clear error state and return to idle
     */
    public void clearError() {
        if (currentState == ShooterState.ERROR) {
            currentState = ShooterState.IDLE;
            leftPID.reset();
            rightPID.reset();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API - STATUS GETTERS
    // ═══════════════════════════════════════════════════════════════════════

    public double getLeftRPM() { return leftRPM; }
    public double getRightRPM() { return rightRPM; }
    public double getAverageRPM() { return averageRPM; }
    public double getRPMSyncError() { return rpmSyncError; }
    public double getAverageSyncError() {
        return syncErrorSampleCount > 0 ? syncErrorAccumulator / syncErrorSampleCount : 0;
    }
    public double getTargetRPM() { return targetRPM; }

    public boolean isAtTargetRPM() { return atTargetRPM; }
    public boolean isStabilized() { return rpmStabilized; }
    public boolean targetReached() { return atTargetRPM; }
    public boolean isReady() { return currentState == ShooterState.READY; }
    public boolean isSpinningUp() { return currentState == ShooterState.SPINNING_UP; }
    public boolean isWarmedUp() { return currentState == ShooterState.WARMUP; }
    public boolean isFiring() { return currentState == ShooterState.FIRING; }
    public boolean isIdle() { return currentState == ShooterState.IDLE; }
    public boolean isError() { return currentState == ShooterState.ERROR; }

    public ShooterState getState() { return currentState; }
    public ShooterConfig.ShooterPreset getActivePreset() { return activePreset; }

    /**
     * Get time remaining before next shot is allowed (milliseconds)
     */
    public long getTimeUntilNextShot() {
        long minInterval = activePreset != null ? activePreset.getShotIntervalMs() : ShooterConfig.MIN_SHOT_INTERVAL_MS;
        long timeSinceLastShot = System.currentTimeMillis() - lastShotTime;
        return Math.max(0, minInterval - timeSinceLastShot);
    }

    /**
     * Get time spent spinning up (milliseconds)
     */
    public long getSpinUpTime() {
        return (long) spinUpTimer.milliseconds();
    }

    /**
     * Get percentage of target RPM achieved
     */
    public double getRPMPercentage() {
        if (targetRPM == 0) return 0;
        return (averageRPM / targetRPM) * 100.0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // BUTTON HANDLER METHODS (FOR EASY OPMODE INTEGRATION)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Handle shoot button with automatic edge detection
     * Call this in your OpMode loop for easy shooting control
     *
     * @param buttonPressed Current state of shoot button
     * @param preset Shooter preset to use
     * @return true if a shot was fired this frame
     */
    public boolean handleShootButton(boolean buttonPressed, ShooterConfig.ShooterPreset preset) {
        boolean shotFired = false;

        // Apply preset
        if (preset != null) {
            setPreset(preset);
        }

        // Detect button press edge
        if (buttonPressed && !prevShootButton) {
            // Button just pressed - start spinning or exit warmup
            if (currentState == ShooterState.WARMUP) {
                // Exit warmup, transition to full power
                spinUp();
            } else if (currentState == ShooterState.IDLE) {
                // Start from idle
                spinUp();
            }
        } else if (!buttonPressed && prevShootButton) {
            // Button just released - stop shooter
            disableShooter();
        }

        // While button held and ready - fire
        if (buttonPressed && isReady()) {
            shotFired = fire();
        }

        prevShootButton = buttonPressed;
        return shotFired;
    }

    /**
     * Handle warmup button with automatic edge detection
     * Warmup spins shooter at 65% of target RPM to save power while staying ready
     *
     * @param buttonPressed Current state of warmup button
     * @param preset Shooter preset to use
     */
    public void handleWarmupButton(boolean buttonPressed, ShooterConfig.ShooterPreset preset) {
        // Apply preset
        if (preset != null) {
            setPreset(preset);
        }

        // Don't allow warmup control if actively shooting
        if (currentState == ShooterState.FIRING || currentState == ShooterState.RECOVERY) {
            return;
        }

        // Detect button press edge
        if (buttonPressed && !prevWarmupButton) {
            // Button just pressed - start warmup
            if (currentState == ShooterState.IDLE) {
                enableWarmup();
            }
        } else if (!buttonPressed && prevWarmupButton) {
            // Button just released - stop warmup (if not shooting)
            if (currentState == ShooterState.WARMUP) {
                disableShooter();
            }
        }

        prevWarmupButton = buttonPressed;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SAFETY AND UTILITIES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check safety conditions and trigger error state if violated
     * Uses time-based filtering to avoid false positives from brief spikes
     */
    private void checkSafetyConditions() {
        // Update moving average of sync error
        syncErrorAccumulator += rpmSyncError;
        syncErrorSampleCount++;

        double averageSyncError = syncErrorAccumulator / syncErrorSampleCount;

        // Reset accumulator every N samples to prevent overflow and stay current
        if (syncErrorSampleCount >= SYNC_ERROR_SAMPLES) {
            syncErrorAccumulator = averageSyncError;
            syncErrorSampleCount = 1;
        }

        // Check if average sync error exceeds threshold
        if (averageSyncError > ShooterConfig.EMERGENCY_SYNC_ERROR) {
            // Start timer if this is first detection
            if (!syncErrorDetected) {
                syncErrorDetected = true;
                syncErrorTimer.reset();
            }

            // Only trigger WARNING (not error) if it persists for the required time
            if (syncErrorTimer.seconds() >= SYNC_ERROR_PERSIST_TIME) {
                // WARNING ONLY - Don't stop the shooter, just notify
                telemetry.addLine("⚠️ WARNING: High RPM sync error detected!");
                telemetry.addLine(String.format("Average sync error: %.0f RPM (threshold: %.0f RPM)",
                    averageSyncError, ShooterConfig.EMERGENCY_SYNC_ERROR));
                telemetry.addLine(String.format("Persisted for: %.2f seconds", syncErrorTimer.seconds()));
                telemetry.addLine("Shooter continuing - monitor sync error!");

                // DO NOT set error state or stop motors
                // currentState = ShooterState.ERROR;
                // setMotorPowers(0, 0);

                // Reset timer to avoid spamming warnings every frame
                syncErrorTimer.reset();
            }
        } else {
            // Sync error dropped below threshold - reset detection
            if (syncErrorDetected) {
                syncErrorDetected = false;
            }
        }

        // Additional state-aware safety: Ignore sync errors during initial spinup
        // Motors naturally have different acceleration rates
        if (currentState == ShooterState.SPINNING_UP && spinUpTimer.milliseconds() < 500) {
            // Reset sync error tracking during first 500ms of spinup
            syncErrorDetected = false;
            syncErrorAccumulator = 0;
            syncErrorSampleCount = 0;
        }

        // Check for motor stall (RPM too low for applied power)
        // This could be implemented with more sophisticated detection
    }

    /**
     * Clamp motor power to safe range
     */
    private double clampPower(double power) {
        return Math.max(ShooterConfig.MIN_POWER, Math.min(ShooterConfig.MAX_POWER, power));
    }

    /**
     * Set motor powers directly
     */
    private void setMotorPowers(double leftPower, double rightPower) {
        leftShooterMotor.setPower(leftPower);
        rightShooterMotor.setPower(rightPower);
    }

    /**
     * Get RPM tolerance based on active preset or default
     */
    private double getRPMTolerance() {
        // Use default tolerance (presets don't have individual tolerances in new system)
        return ShooterConfig.RPM_TOLERANCE;
    }

    /**
     * Get max spinup time based on active preset or default
     */
    private long getMaxSpinupTime() {
        // Use spinup time from preset if available, otherwise default
        return activePreset != null ? activePreset.getSpinupTimeMs() : ShooterConfig.MAX_SPINUP_TIME_MS;
    }

    /**
     * Get detailed stabilization debug information
     */
    public String getStabilizationDebugInfo() {
        long currentTime = System.currentTimeMillis();
        if (stabilizationStartTime == 0) {
            return "not tracking (not at target)";
        } else {
            long elapsedTime = currentTime - stabilizationStartTime;
            long requiredTime = ShooterConfig.RPM_STABILIZATION_TIME_MS;
            return String.format("tracking: %dms/%dms", elapsedTime, requiredTime);
        }
    }

    /**
     * Get detailed status string for telemetry
     */
    public String getStatusString() {
        if (currentState == ShooterState.ERROR) {
            return String.format(
                "🚨 ERROR STATE 🚨\n" +
                "Target: %.0f RPM | Avg: %.0f RPM\n" +
                "L: %.0f | R: %.0f | Sync Error: %.0f RPM\n" +
                "❌ EXCESSIVE SYNC ERROR (threshold: %.0f RPM)\n" +
                "Press BACK to clear error",
                targetRPM,
                averageRPM,
                leftRPM,
                rightRPM,
                rpmSyncError,
                ShooterConfig.EMERGENCY_SYNC_ERROR
            );
        }

        return String.format(
            "State: %s | Target: %.0f RPM | Avg: %.0f RPM (%.1f%%)\n" +
            "L: %.0f | R: %.0f | Sync Error: %.0f | %s",
            currentState,
            targetRPM,
            averageRPM,
            getRPMPercentage(),
            leftRPM,
            rightRPM,
            rpmSyncError,
            atTargetRPM ? "✅ AT TARGET" : "⏳ SPINNING"
        );
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GAMEPADCONFIG INTEGRATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Handle shooting controls from GamepadConfig
     * Call this in your OpMode loop for automatic shooting control
     *
     * @param gamepadConfig GamepadConfig instance
     * @return true if a shot was fired this frame
     */
    public boolean handleGamepadShootControls(GamepadConfig gamepadConfig) {
        boolean shotFired = false;

        // Handle long range shooting (Y button)
        if (gamepadConfig.isLongRangeShootHeld()) {
            shotFired = handleShootButton(true, ShooterConfig.ShooterPreset.LONG_RANGE);
        }
        // Handle short range shooting (A button)
        else if (gamepadConfig.isShortRangeShootHeld()) {
            shotFired = handleShootButton(true, ShooterConfig.ShooterPreset.SHORT_RANGE);
        }
        // Handle auto/mid range shooting (B button)
        else if (gamepadConfig.isAutoRangeShootHeld()) {
            shotFired = handleShootButton(true, ShooterConfig.ShooterPreset.MID_RANGE);
        }
        // No button held - stop shooter
        else {
            handleShootButton(false, null);
        }

        // Handle warmup mode (left trigger)
        boolean warmupActive = gamepadConfig.isWarmupActive();
        ShooterConfig.ShooterPreset warmupPreset = activePreset != null ?
                activePreset : ShooterConfig.ShooterPreset.LONG_RANGE;
        handleWarmupButton(warmupActive, warmupPreset);

        return shotFired;
    }

    /**
     * Get manual shooter power from GamepadConfig (right trigger)
     * Use this for manual shooter control mode
     *
     * @param gamepadConfig GamepadConfig instance
     * @return Manual power (0.0 to 1.0) from right trigger
     */
    public double getManualPowerFromGamepad(GamepadConfig gamepadConfig) {
        return gamepadConfig.getManualShooterPower();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // INNER CLASS: PID CONTROLLER
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Simple PID controller with feedforward
     * Supports gain scheduling via updateGains()
     */
    private static class PIDController {
        private double kP, kI, kD, kF;  // Made non-final for gain scheduling
        private double integral = 0;
        private double lastError = 0;
        private long lastTime = 0;

        public PIDController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            reset();
        }

        /**
         * Update PID gains (for gain scheduling)
         */
        public void updateGains(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        public double calculate(double current, double target) {
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;

            if (lastTime == 0) {
                lastTime = currentTime;
                return kF * target; // Return feedforward only on first call
            }

            double error = target - current;

            // Proportional
            double p = kP * error;

            // Integral (with anti-windup)
            integral += error * deltaTime;
            integral = Math.max(-ShooterConfig.MAX_INTEGRAL, Math.min(ShooterConfig.MAX_INTEGRAL, integral));
            double i = kI * integral;

            // Derivative
            double derivative = (error - lastError) / deltaTime;
            double d = kD * derivative;

            // Feedforward
            double f = kF * target;

            lastError = error;
            lastTime = currentTime;

            return p + i + d + f;
        }

        public void reset() {
            integral = 0;
            lastError = 0;
            lastTime = 0;
        }
    }
}
