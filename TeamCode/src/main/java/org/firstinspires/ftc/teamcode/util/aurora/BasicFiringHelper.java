package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;

/**
 * BasicFiringHelper - Non-blocking firing and ejection control
 *
 * This helper class provides:
 * - Non-blocking firing sequences (spin up → feed → fire)
 * - RPM preset management (high, medium, low range)
 * - Keep-alive mode (shooter stays spinning for rapid follow-up shots)
 * - Ejection sequences (clear artifacts from robot)
 * - State machine architecture for reliable operation
 * - Cancel operations mid-sequence
 *
 * Firing Sequence:
 * 1. Spin up shooter to target RPM
 * 2. Wait for shooter ready (at target + stable)
 * 3. Feed artifact with uptake servos (300ms)
 * 4. Complete (or transition to READY_TO_FIRE if keep-alive mode)
 *
 * Keep-Alive Mode:
 * When enabled, shooter stays spinning after first shot.
 * Use fireShot() or startFiring() to trigger subsequent shots.
 * Call cancelFiring() to stop the shooter.
 *
 * Ejection Sequence:
 * - Run both intakes backward (eject from intakes)
 * - Run shooter at low speed (safe ejection from center)
 * - Run uptake forward (push artifact out through shooter)
 *
 * Usage Pattern (Single Shot):
 *   BasicFiringHelper firingHelper = new BasicFiringHelper(shooter, indexingHelper, hardware, telemetry);
 *
 *   // Start single shot (shooter stops after)
 *   firingHelper.startFiring(ShooterConfig.RPM_HIGH_BASKET, "HIGH", false);
 *
 *   // In loop
 *   firingHelper.update();
 *
 *   // Check if complete
 *   if (!firingHelper.isFiring()) {
 *       // Ready for next shot
 *   }
 *
 * Usage Pattern (Keep-Alive / Multiple Shots):
 *   // Start with keep-alive enabled
 *   firingHelper.startFiring(ShooterConfig.RPM_HIGH_BASKET, "HIGH", true);
 *
 *   // In loop
 *   firingHelper.update();
 *
 *   // Wait for ready
 *   if (firingHelper.isReadyForNextShot()) {
 *       // Fire subsequent shots
 *       if (gamepad1.a) {
 *           firingHelper.fireShot();  // Fire another shot
 *       }
 *   }
 *
 *   // Stop when done
 *   if (gamepad1.b) {
 *       firingHelper.cancelFiring();
 *   }
 */
public class BasicFiringHelper {

    // Dependencies
    private final Shooter shooter;
    private final BasicIndexingHelper indexingHelper;
    private final Telemetry telemetry;
    private final AuroraHardwareConfig hardware;

    // Global timeout (milliseconds)
    private static final long GLOBAL_TIMEOUT = 15000; // 15 seconds max for any firing operation

    // Firing constants
    private static final long FEED_DURATION_MS = 300; // Uptake feed time
    private static final double FEED_POWER = 1.0;     // Full power for feeding
    private static final long SHOOTER_READY_TIMEOUT = 10000; // 10 seconds to reach target RPM
    private static final long MIN_SPINUP_TIME = 2000; // Minimum 2 seconds before checking ready
    private static final boolean USE_RELAXED_READY_CHECK = true; // If true, fire when at target RPM even if not stabilized

    // Ejection constants
    private static final double EJECTION_SHOOTER_RPM = 1200.0; // Low speed for safe ejection
    private static final double EJECTION_INTAKE_POWER = -1.0;  // Reverse intakes (rollers backward)
    private static final double EJECTION_TRANSFER_POWER = 1.0; // Forward transfer (push out)
    private static final double EJECTION_UPTAKE_POWER = 1.0;   // Forward to push out


    // State tracking
    private enum FiringState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        COMPLETE,
        READY_TO_FIRE  // Shooter spun up, waiting for next fire command
    }

    private FiringState firingState = FiringState.IDLE;
    private boolean firingActive = false;
    private long firingStartTime = 0;
    private double targetRPM = 0;
    private String firingPresetName = "NONE";

    // Keep-alive mode - shooter stays spinning between shots
    private boolean keepAliveMode = false;
    private boolean buttonHeld = false;

    // Shot tracking - for reliable "shot fired" detection
    private int shotsFiredCount = 0;  // Total shots fired (increments when shot completes)
    private int lastReportedShotCount = 0;  // For detecting new shots

    // Ejection tracking
    private boolean ejectionActive = false;
    private double ejectionIntakePower = EJECTION_INTAKE_POWER;

    // Enabled state
    private boolean enabled = true;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new BasicFiringHelper
     * @param shooter Shooter instance for RPM control
     * @param indexingHelper BasicIndexingHelper for uptake control
     * @param hardware AuroraHardwareConfig for direct hardware access
     * @param telemetry Telemetry for status updates
     */
    public BasicFiringHelper(Shooter shooter, BasicIndexingHelper indexingHelper, AuroraHardwareConfig hardware, Telemetry telemetry) {
        this.shooter = shooter;
        this.indexingHelper = indexingHelper;
        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UPDATE METHOD - MUST BE CALLED EVERY LOOP
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update firing sequences - MUST be called every loop
     * Handles state transitions and timeout protection
     *
     * ⚠️ CRITICAL: This method calls shooter.update() internally!
     * DO NOT call shooter.update() separately in your OpMode loop!
     * Calling shooter.update() twice per loop will cause the shooter to pulse on/off.
     *
     * CORRECT OpMode pattern:
     *   while (opModeIsActive()) {
     *       indexingHelper.update();
     *       firingHelper.update();  // ← shooter.update() called here internally
     *       // ... rest of code
     *   }
     *
     * WRONG OpMode pattern (DO NOT DO THIS):
     *   while (opModeIsActive()) {
     *       shooter.update();       // ❌ WRONG - duplicate call!
     *       firingHelper.update();  // ❌ Also calls shooter.update() internally
     *   }
     */
    public void update() {
        if (!enabled) return;

        // DEBUG: Track shooter state before operations
        String stateBefore = shooter.getState().toString();

        // IMPORTANT: Update firing state machine FIRST to set desired state via spinUp()
        // This ensures the state is set before DecodeHelper.update() processes it
        updateFiringSequence();

        // Update ejection - continuously maintain motor/servo power
        updateEjection();

        // CRITICAL: Update shooter state machine (DecodeHelper) AFTER setting desired state
        // This processes RPM measurements, PID control, and state transitions
        shooter.update();

        String stateAfter = shooter.getState().toString();

        // DEBUG: Log if state changed unexpectedly
        if (!stateBefore.equals(stateAfter)) {
            telemetry.addData("DEBUG State Change", stateBefore + " → " + stateAfter);
        }


        // Check for global timeout
        if (firingActive) {
            long elapsed = System.currentTimeMillis() - firingStartTime;
            if (elapsed > GLOBAL_TIMEOUT) {
                telemetry.addData("⚠️ FIRING TIMEOUT", "Exceeded " + GLOBAL_TIMEOUT + "ms");
                cancelFiring();
            }
        }
    }

    /**
     * Update ejection - no action needed
     * Motor/servo power set once in startEjection() and left to run
     * Shooter maintains RPM automatically via shooter.update()
     */
    private void updateEjection() {
        if (!ejectionActive) return;

        // Nothing to do - shooter.update() maintains RPM automatically
        // Motors and servos were set to run continuously in startEjection()
    }

    /**
     * Update firing state machine
     * Handles automatic progression through firing sequence
     */
    private void updateFiringSequence() {
        if (!firingActive) return;

        switch (firingState) {
            case SPINNING_UP:
                // DO NOT call shooter.spinUp() here repeatedly!
                // It was already called once in startFiring() to initiate the spinup.
                // DecodeHelper.update() handles all RPM control and state transitions automatically.
                // Calling spinUp() repeatedly is unnecessary and was based on a misunderstanding.

                // The shooter will automatically transition from SPINNING_UP -> READY when:
                // 1. RPM reaches target (within tolerance)
                // 2. RPM is stabilized (consistent over time)

                // Wait for shooter to reach target RPM and stabilize
                long elapsed = System.currentTimeMillis() - firingStartTime;
                double currentRPM = shooter.getCurrentRPM();
                boolean isReady = shooter.isReadyToFire();

                // Relaxed ready check: fire if at target RPM even if not stabilized (after minimum time)
                boolean canFire = isReady;
                if (USE_RELAXED_READY_CHECK && elapsed >= MIN_SPINUP_TIME) {
                    // Check if at target RPM (even if not stabilized)
                    double rpmDelta = Math.abs(targetRPM - currentRPM);
                    boolean atTargetRPM = rpmDelta < 200; // Within 200 RPM is good enough
                    if (atTargetRPM) {
                        canFire = true;
                        telemetry.addData("ℹ️ Info", "Using relaxed ready check (at target, may not be fully stabilized)");
                    }
                }

                if (canFire) {
                    // Shooter ready - start feeding
                    firingState = FiringState.FEEDING;
                    indexingHelper.setUptakeTimed(FEED_POWER, FEED_DURATION_MS);
                    telemetry.addData("Firing", "✅ Feeding artifact (300ms)");
                    telemetry.addData("  Ready Status", isReady ? "FULLY READY" : "AT TARGET RPM");
                } else {
                    // Still spinning up - show detailed debug info
                    telemetry.addData("🔄 Spinning Up", "Waiting for ready...");
                    telemetry.addData("  Target RPM", String.format("%.0f", targetRPM));
                    telemetry.addData("  Current RPM", String.format("%.0f", currentRPM));
                    telemetry.addData("  Delta", String.format("%.0f", Math.abs(targetRPM - currentRPM)));
                    telemetry.addData("  Shooter State", shooter.getState());
                    telemetry.addData("  Shooter Ready", isReady ? "YES" : "NO");
                    telemetry.addData("  Elapsed", elapsed + "ms");

                    // Show what we're waiting for
                    if (elapsed < MIN_SPINUP_TIME) {
                        telemetry.addData("  Waiting", "Min spinup time (" + (MIN_SPINUP_TIME - elapsed) + "ms left)");
                    } else if (Math.abs(targetRPM - currentRPM) >= 200) {
                        telemetry.addData("  Waiting", "Closer to target RPM (need < 200 delta)");
                    } else {
                        telemetry.addData("  Waiting", "Stabilization (isStabilized check)");
                    }

                    // Check for timeout
                    if (elapsed > SHOOTER_READY_TIMEOUT) {
                        telemetry.addData("⚠️ FIRING ERROR", "Shooter failed to reach target RPM after " + elapsed + "ms");
                        telemetry.addData("  Final RPM", String.format("%.0f", currentRPM));
                        telemetry.addData("  Target was", String.format("%.0f", targetRPM));
                        cancelFiring();
                    }
                }
                break;

            case FEEDING:
                // CRITICAL: DO NOT call shooter.spinUp() here!
                // DecodeHelper.update() automatically maintains RPM in all states
                // Calling spinUp() during feeding causes state to reset back to SPINNING_UP
                // which destroys PID stability and causes RPM oscillations
                //
                // The shooter's update() method applies PID control in ALL states including FIRING/RECOVERY
                // We just need to wait for uptake to finish

                // Wait for uptake to finish feeding
                if (!indexingHelper.isUptakeBusy()) {
                    // Feeding complete - artifact ejected (POINT OF NO RETURN)
                    // Shot counter increments here because:
                    // - Artifact has been pushed through uptake into shooter
                    // - Past point of no return - artifact leaves center slot permanently
                    // - Conservative assumption: if feeding started, treat as fired
                    // - Even if jams in shooter, counter increments (safe/consistent model)
                    shotsFiredCount++;  // Increment shot counter
                    
                    if (keepAliveMode) {
                        // Keep-alive mode: transition to READY_TO_FIRE instead of stopping
                        // Shooter will stay spinning, waiting for external call to fire again
                        firingState = FiringState.READY_TO_FIRE;
                        telemetry.addData("Firing", "✅ Shot complete - Ready for next");
                        Dbg.d(LogGroup.FIRING, "Shot fired - transitioning to READY_TO_FIRE");
                    } else {
                        // Normal mode: complete and stop
                        firingState = FiringState.COMPLETE;
                        firingActive = false;
                        shooter.stopMotors();
                        telemetry.addData("Firing", "✅ Complete");
                        Dbg.i(LogGroup.FIRING, "Shot fired - firing sequence complete");
                    }
                }
                break;

            case READY_TO_FIRE:
                // CRITICAL: Shooter is spinning and waiting for next fire command
                // DO NOT call shooter.spinUp() here - it forces state back to SPINNING_UP!
                // The DecodeHelper.update() automatically maintains RPM in READY state
                // Calling spinUp() causes oscillation by forcing unnecessary state transitions

                // Display status
                telemetry.addData("Firing", "✅ Ready for next shot");
                telemetry.addData("  Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
                telemetry.addData("  Target RPM", String.format("%.0f", targetRPM));
                telemetry.addData("  Delta", String.format("%.0f", Math.abs(targetRPM - shooter.getCurrentRPM())));
                telemetry.addData("  Shooter State", shooter.getState());

                // Check if button released - stop shooter when button released
                if (!buttonHeld) {
                    telemetry.addData("Firing", "⏹️ Button released - stopping shooter");
                    cancelFiring();
                }
                break;

            case COMPLETE:
            case IDLE:
                firingActive = false;
                keepAliveMode = false;
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIRING CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Start firing sequence with specified RPM
     * Non-blocking - will complete automatically
     *
     * Sequence:
     * 1. Spin up shooter to target RPM
     * 2. Wait for shooter ready
     * 3. Feed artifact (300ms)
     *
     * @param rpm Target RPM for shooter
     * @return true if started successfully, false if already firing
     */
    public boolean startFiring(double rpm) {
        return startFiring(rpm, "CUSTOM");
    }

    /**
     * Start firing sequence with specified RPM and preset name
     *
     * @param rpm Target RPM for shooter
     * @param presetName Name of preset for telemetry
     * @return true if started successfully, false if already firing
     */
    public boolean startFiring(double rpm, String presetName) {
        return startFiring(rpm, presetName, false);
    }

    /**
     * Start firing with a ShooterConfig preset
     * Recommended method - uses preset configuration from ShooterConfig
     *
     * @param preset ShooterConfig.ShooterPreset to use
     * @param keepAlive If true, shooter stays spinning after first shot
     * @return true if started successfully
     */
    public boolean startFiringWithPreset(ShooterConfig.ShooterPreset preset, boolean keepAlive) {
        return startFiring(preset.getTargetRPM(), preset.getName(), keepAlive);
    }

    /**
     * Start firing with a ShooterConfig preset (keep-alive mode enabled)
     *
     * @param preset ShooterConfig.ShooterPreset to use
     * @return true if started successfully
     */
    public boolean startFiringWithPreset(ShooterConfig.ShooterPreset preset) {
        return startFiringWithPreset(preset, true);
    }

    /**
     * Start firing sequence with specified RPM, preset name, and keep-alive mode
     *
     * @param rpm Target RPM for shooter
     * @param presetName Name of preset for telemetry
     * @param keepAlive If true, shooter stays spinning after first shot for rapid follow-up shots
     * @return true if started successfully, false if already firing
     */
    public boolean startFiring(double rpm, String presetName, boolean keepAlive) {
        if (!enabled) {
            telemetry.addData("⚠️ WARNING", "Firing helper disabled");
            return false;
        }

        // Safety: Stop ejection if somehow still active
        if (ejectionActive) {
            telemetry.addData("⚠️ WARNING", "Ejection was active - stopping it");
            stopEjection();
        }

        // If already in READY_TO_FIRE state (keep-alive), allow another shot
        if (firingActive && firingState == FiringState.READY_TO_FIRE) {
            // Trigger another shot
            firingState = FiringState.FEEDING;
            indexingHelper.setUptakeTimed(FEED_POWER, FEED_DURATION_MS);
            telemetry.addData("Firing", "Feeding next artifact (300ms)");
            return true;
        }

        // If firing in progress (not ready), reject
        if (firingActive) {
            telemetry.addData("⚠️ WARNING", "Firing already in progress");
            return false;
        }

        // Start new firing sequence
        firingActive = true;
        firingState = FiringState.SPINNING_UP;
        firingStartTime = System.currentTimeMillis();
        targetRPM = rpm;
        firingPresetName = presetName;
        keepAliveMode = keepAlive;

        // Spin up shooter - MUST call both setTargetRPM and spinUp
        shooter.setTargetRPM(rpm);
        shooter.spinUp(); // This transitions the DecodeHelper state machine from IDLE to SPINNING_UP
        telemetry.addData("Firing", "Spinning up to " + rpm + " RPM (" + presetName + ")");
        if (keepAlive) {
            telemetry.addData("Mode", "Keep-alive (continuous)");
        }
        Dbg.d(LogGroup.FIRING, "startFiring() called - targetRPM=%.0f, preset=%s, keepAlive=%b", rpm, presetName, keepAlive);

        return true;
    }

    /**
     * Fire a single shot (only works when in READY_TO_FIRE state)
     * Use this method to trigger follow-up shots after the shooter is spun up
     *
     * This is the recommended way to fire subsequent shots in keep-alive mode:
     * 1. Call startFiring() with keepAlive=true to spin up
     * 2. Wait for isReadyForNextShot() to return true
     * 3. Call fireShot() to trigger each subsequent shot
     *
     * @return true if shot started, false if not ready
     */
    public boolean fireShot() {
        if (!enabled) {
            telemetry.addData("⚠️ WARNING", "Firing helper disabled");
            return false;
        }

        if (!firingActive || firingState != FiringState.READY_TO_FIRE) {
            telemetry.addData("⚠️ WARNING", "Not in READY_TO_FIRE state");
            return false;
        }

        // Transition to FEEDING state
        firingState = FiringState.FEEDING;
        indexingHelper.setUptakeTimed(FEED_POWER, FEED_DURATION_MS);
        telemetry.addData("Firing", "✅ Firing shot (300ms)");
        Dbg.d(LogGroup.FIRING, "fireShot() called - feeding artifact");
        return true;
    }

    /**
     * Update button state for keep-alive mode
     * Call this every loop with the current button state
     *
     * @param pressed true if firing button is currently pressed
     */
    public void setButtonHeld(boolean pressed) {
        this.buttonHeld = pressed;
    }

    /**
     * Check if ready to fire another shot (in READY_TO_FIRE state)
     * Use this to know when you can call startFiring() for a follow-up shot
     *
     * @return true if shooter is spun up and waiting for next shot
     */
    public boolean isReadyForNextShot() {
        return firingActive && firingState == FiringState.READY_TO_FIRE;
    }

    /**
     * Start firing with long range preset (2800 RPM)
     * For shots from far distance
     * Uses keep-alive mode - shooter stays spinning while button held
     * @return true if started successfully
     */
    public boolean startFiringLongRange() {
        return startFiring(ShooterConfig.ShooterPreset.LONG_RANGE.getTargetRPM(),
                          ShooterConfig.ShooterPreset.LONG_RANGE.getName(),
                          true);
    }

    /**
     * Start firing with mid-range preset (2300 RPM)
     * For shots from medium distance
     * Uses keep-alive mode - shooter stays spinning while button held
     * @return true if started successfully
     */
    public boolean startFiringMidRange() {
        return startFiring(ShooterConfig.ShooterPreset.MID_RANGE.getTargetRPM(),
                          ShooterConfig.ShooterPreset.MID_RANGE.getName(),
                          true);
    }

    /**
     * Start firing with short range preset (1100 RPM)
     * For shots from close distance
     * Uses keep-alive mode - shooter stays spinning while button held
     * @return true if started successfully
     */
    public boolean startFiringShortRange() {
        return startFiring(ShooterConfig.ShooterPreset.SHORT_RANGE.getTargetRPM(),
                          ShooterConfig.ShooterPreset.SHORT_RANGE.getName(),
                          true);
    }

    /**
     * Cancel firing sequence immediately
     * Stops shooter and uptake servos
     */
    public void cancelFiring() {
        if (!firingActive) return;

        firingActive = false;
        firingState = FiringState.IDLE;

        // Stop shooter
        telemetry.addData("DEBUG", "❌ cancelFiring() called - stopping shooter");
        shooter.stopMotors();

        // Stop uptake if feeding
        indexingHelper.stopUptake();

        telemetry.addData("Firing", "❌ Cancelled");
    }

    /**
     * Check if firing sequence is active
     * @return true if firing in progress
     */
    public boolean isFiring() {
        return firingActive;
    }

    /**
     * Get current firing state
     * @return current state as string
     */
    public String getFiringState() {
        return firingState.toString();
    }

    /**
     * Get target RPM for current firing
     * @return target RPM, or 0 if not firing
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Get current shooter RPM (actual measured speed).
     * 
     * @return current shooter RPM
     */
    public double getCurrentRPM() {
        return shooter.getCurrentRPM();
    }

    /**
     * Check if shooter is ready to fire (at target RPM and stable).
     * 
     * @return true if shooter is ready to fire
     */
    public boolean isShooterReady() {
        return shooter.isReadyToFire();
    }

    /**
     * Start spinning up shooter to target RPM.
     * This is the preferred method for OpModes to spin up the shooter.
     * 
     * Note: After spinup, use startFiring() to actually fire when ready.
     */
    public void spinUpShooter() {
        shooter.spinUp();
    }

    /**
     * Stop the shooter motors completely.
     * This is the preferred method for OpModes to stop the shooter.
     * 
     * Note: This stops the shooter immediately, regardless of firing state.
     */
    public void stopShooter() {
        shooter.stopMotors();
    }

    /**
     * Get firing preset name
     * @return preset name
     */
    public String getPresetName() {
        return firingPresetName;
    }

    /**
     * Get total number of shots fired since helper creation
     *
     * **Shot Fired Definition:**
     * Counter increments when uptake feeding completes (artifact ejected from center slot).
     * This is the "point of no return" - once feeding begins, artifact is considered fired.
     *
     * **Conservative Assumption:**
     * Even if artifact jams in shooter after feeding, counter still increments.
     * This ensures slot ledger always reflects physical state (artifact left center slot).
     *
     * **Usage (Operations):**
     * Operations should snapshot this counter at start and compare during/after execution:
     * ```java
     * int startCount = firingHelper.getShotsFiredCount();
     * // ... operation runs
     * boolean shotFired = firingHelper.getShotsFiredCount() > startCount;
     * ```
     *
     * **Why This Works:**
     * - Stateless: Each operation has independent snapshot
     * - Race-free: No shared edge detection state
     * - Reliable: Works even if telemetry/controller also checks counter
     *
     * @return total shots fired count (point-of-no-return boundary)
     */
    public int getShotsFiredCount() {
        return shotsFiredCount;
    }

    /**
     * Check if a new shot has been fired since last check
     *
     * **CONTROLLER ORCHESTRATION ONLY - DO NOT USE IN OPERATIONS**
     *
     * This provides edge-detection for "shot just fired" events for controller orchestration
     * (e.g., detect shot complete → queue next transfer operation).
     *
     * **Stateful:** This method maintains internal state (`lastReportedShotCount`).
     * Only the controller should call this method. Operations MUST use getShotsFiredCount()
     * with snapshot pattern instead.
     *
     * **Usage Pattern (Controller):**
     * ```java
     * // Controller calls ONCE per loop for orchestration
     * if (firingHelper.hasNewShotFired()) {
     *     // Shot just completed - queue transfer for next artifact
     *     queueTransferOperation();
     * }
     * ```
     *
     * **Do NOT use in FireOperation** - operations use count-based detection with snapshots.
     *
     * @return true if shot count increased since last call to this method
     */
    public boolean hasNewShotFired() {
        if (shotsFiredCount > lastReportedShotCount) {
            lastReportedShotCount = shotsFiredCount;
            return true;
        }
        return false;
    }

    /**
     * Reset the "last reported" shot count
     *
     * **CONTROLLER USE ONLY**
     *
     * Use this to restart edge detection from current count.
     * Controller should call this at the start of a new burst sequence.
     *
     * @see #hasNewShotFired() for usage pattern
     */
    public void resetShotDetection() {
        lastReportedShotCount = shotsFiredCount;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // EJECTION CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Start ejection sequence
     * Clears artifacts from robot by:
     * - Running intakes backward (eject from intakes)
     * - Running shooter at low speed (safe ejection from center)
     * - Running uptake forward (push out through shooter)
     *
     * Call stopEjection() to stop
     *
     * @param intakePower Intake reverse power (default -1.0 if 0)
     * @param shooterRPM Shooter ejection RPM (default 1500 if 0)
     * @return true if started successfully
     */
    public boolean startEjection(double intakePower, double shooterRPM) {
        if (!enabled) {
            telemetry.addData("⚠️ WARNING", "Firing helper disabled");
            return false;
        }

        if (firingActive) {
            telemetry.addData("⚠️ WARNING", "Cannot eject during firing");
            return false;
        }

        if (ejectionActive) {
            telemetry.addData("⚠️ WARNING", "Ejection already active");
            return false;
        }

        // Use defaults if not specified
        if (intakePower == 0) intakePower = EJECTION_INTAKE_POWER;
        if (shooterRPM == 0) shooterRPM = EJECTION_SHOOTER_RPM;

        ejectionActive = true;
        ejectionIntakePower = intakePower; // Save for reference

        // Directly control motors via hardware - set power ONCE and let it run
        hardware.getFrontRollerMotor().setPower(intakePower);
        hardware.getBackRollerMotor().setPower(intakePower);
        hardware.getFrontTransferServo().setPower(EJECTION_TRANSFER_POWER);
        hardware.getBackTransferServo().setPower(EJECTION_TRANSFER_POWER);
        hardware.getUptakeServoL().setPower(EJECTION_UPTAKE_POWER);
        hardware.getUptakeServoR().setPower(EJECTION_UPTAKE_POWER);

        // Run shooter at low speed
        shooter.setTargetRPM(shooterRPM);
        shooter.spinUp();

        telemetry.addData("Ejection", "Active (Intakes: " + intakePower + ", Shooter: " + shooterRPM + " RPM)");

        return true;
    }

    /**
     * Start ejection with default settings
     * Intakes: -1.0 (reverse), Shooter: 1500 RPM, Uptake: 1.0 (forward)
     * @return true if started successfully
     */
    public boolean startEjection() {
        return startEjection(0, 0); // Use defaults
    }

    /**
     * Stop ejection sequence
     * Stops all motors and servos directly via hardware
     */
    public void stopEjection() {
        if (!ejectionActive) return;

        telemetry.addData("DEBUG", "❌ stopEjection() called - stopping shooter");
        ejectionActive = false;

        // Stop all motors/servos directly via hardware
        hardware.getFrontRollerMotor().setPower(0);
        hardware.getBackRollerMotor().setPower(0);
        hardware.getFrontTransferServo().setPower(0);
        hardware.getBackTransferServo().setPower(0);
        hardware.getUptakeServoL().setPower(0);
        hardware.getUptakeServoR().setPower(0);

        // Stop shooter
        shooter.stopMotors();


        telemetry.addData("Ejection", "Stopped");
    }

    /**
     * Check if ejection is active
     * @return true if ejection in progress
     */
    public boolean isEjecting() {
        return ejectionActive;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GLOBAL CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Stop all operations immediately
     * Cancels firing and ejection
     */
    public void stopAll() {
        cancelFiring();
        stopEjection();
    }

    /**
     * Check if any operation is active
     * @return true if firing or ejecting
     */
    public boolean isAnyOperationActive() {
        return firingActive || ejectionActive;
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
        telemetry.addData("═══ Basic Firing Helper ═══", "");
        telemetry.addData("Enabled", enabled);

        telemetry.addData("Firing Active", firingActive);
        if (firingActive) {
            telemetry.addData("  State", firingState);
            telemetry.addData("  Preset", firingPresetName);
            telemetry.addData("  Target RPM", String.format("%.0f", targetRPM));
            telemetry.addData("  Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
            telemetry.addData("  Shooter Ready", shooter.isReadyToFire());
            telemetry.addData("  Keep-Alive", keepAliveMode ? "YES" : "NO");
            telemetry.addData("  Button Held", buttonHeld ? "YES" : "NO");
            long elapsed = System.currentTimeMillis() - firingStartTime;
            telemetry.addData("  Elapsed", elapsed + "ms");
        }

        telemetry.addData("Ejection Active", ejectionActive);

        // Debug shooter state
        telemetry.addData("  Shooter State", shooter.getState());
        telemetry.addData("  Shooter Enabled", shooter.isEnabled());
    }
}
