package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;

/**
 * FireOperation - Fire center artifact via BasicFiringHelper
 *
 * **CRITICAL INVARIANT: One FireOperation = Exactly One Shot (or zero if cancelled before firing)**
 *
 * Supports two firing modes:
 *
 * **Single-shot mode (keepAlive=false):**
 * - Spins up shooter, fires ONE shot, stops shooter
 * - Use for standalone shots
 *
 * **Keep-alive mode (keepAlive=true):**
 * - First shot: Spins up shooter + fires ONE shot (keeps shooter running)
 * - Subsequent shots: Fires ONE shot (shooter already spinning)
 * - Controller manages: Fire → Transfer next → Fire → etc.
 * - Much faster than spinning up each time (2-3x speedup)
 * - **Controller responsibility**: Must call firingHelper.cancelFiring() when burst complete
 *
 * Operation flow:
 * 1. Check preconditions (center occupied, prepositioned, shooter ready if subsequent shot)
 * 2. Start firing: startFiring(rpm, keepAlive) OR fireShot() if shooter already spun up
 * 3. Wait for completion (ONE shot fired, ready for next OR stopped)
 * 4. **Check shouldContinue() every loop** - cancel if returns false
 * 5. If cancelled: determine if shot actually fired (check BasicFiringHelper state)
 * 6. If shot fired: clear center slot + consume shot from plan
 * 7. If cancelled before shot: ledger unchanged, no shot consumption
 *
 * Preconditions:
 * - Center slot occupied
 * - Artifact prepositioned
 * - If first shot: shooter ready OR will spin up
 * - If subsequent shot (keep-alive): shooter in READY_TO_FIRE state
 *
 * Cancellation Semantics (mechanically safe):
 * - Before shot starts: Abort cleanly, no ledger changes, no shot consumption
 * - During/after shot: Shot completes (cannot "unshoot"), ledger updated, shot consumed
 * - Detection: Use BasicFiringHelper state to determine if shot actually fired
 * - Conservative assumption: If feeding began, assume shot fired (safe side)
 *
 * Keep-Alive Exit Behavior:
 * - FireOperation does NOT stop shooter automatically in keep-alive mode
 * - Controller MUST explicitly call firingHelper.cancelFiring() when burst complete
 * - Failure to do so leaves shooter spinning indefinitely (unsafe/wasteful)
 * - Recommended: Controller tracks "burst active" flag and cancels on trigger release
 *
 * Hardware: Uses BasicFiringHelper with keep-alive support
 * Phase 5: Integrates with ShotPlanningCoordinator
 */
public class FireOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // INTERFACES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Callback to check if we should continue firing
     * Checked every loop during operation
     * Return false to cancel firing mid-operation
     */
    public interface ShouldContinueCallback {
        boolean shouldContinue();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final BasicFiringHelper firingHelper;
    private final Shooter shooter;
    private final double targetRPM;
    private final boolean keepAlive;  // Keep shooter running after shot
    private final ShotPlanningCoordinator shotPlanner;  // Optional (Phase 5)
    private final ShouldContinueCallback shouldContinueCallback;  // Optional cancel check

    private ArtifactIdentity firedArtifact;
    private boolean isSubsequentShot;  // True if shooter already spun up
    private boolean shotActuallyFired;  // True if physical shot occurred (for cancellation safety)
    private int shotCountAtStart;  // Shot count when operation started (for reliable detection)
    private boolean wasCancelledBeforeShot;  // True if cancelled before shot fired (for explicit telemetry)

    // Fire timeout
    private static final long FIRE_TIMEOUT_MS = 8000;  // 8 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTORS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new FireOperation (single-shot mode, no shot planning)
     *
     * @param ledger Slot ledger to update
     * @param firingHelper BasicFiringHelper for firing control
     * @param shooter Shooter instance for RPM check
     * @param targetRPM Target RPM for firing
     * @param telemetry Telemetry for logging
     */
    public FireOperation(SlotLedger ledger,
                        BasicFiringHelper firingHelper,
                        Shooter shooter,
                        double targetRPM,
                        Telemetry telemetry) {
        this(ledger, firingHelper, shooter, targetRPM, false, null, null, telemetry);
    }

    /**
     * Create a new FireOperation (with keep-alive mode, no shot planning)
     *
     * @param ledger Slot ledger to update
     * @param firingHelper BasicFiringHelper for firing control
     * @param shooter Shooter instance for RPM check
     * @param targetRPM Target RPM for firing
     * @param keepAlive If true, keeps shooter running after shot for faster follow-up
     * @param telemetry Telemetry for logging
     */
    public FireOperation(SlotLedger ledger,
                        BasicFiringHelper firingHelper,
                        Shooter shooter,
                        double targetRPM,
                        boolean keepAlive,
                        Telemetry telemetry) {
        this(ledger, firingHelper, shooter, targetRPM, keepAlive, null, null, telemetry);
    }

    /**
     * Create a new FireOperation (with shot planning - Phase 5)
     *
     * @param ledger Slot ledger to update
     * @param firingHelper BasicFiringHelper for firing control
     * @param shooter Shooter instance for RPM check
     * @param targetRPM Target RPM for firing
     * @param keepAlive If true, keeps shooter running after shot for faster follow-up
     * @param shotPlanner Shot planning coordinator (null if not using)
     * @param telemetry Telemetry for logging
     */
    public FireOperation(SlotLedger ledger,
                        BasicFiringHelper firingHelper,
                        Shooter shooter,
                        double targetRPM,
                        boolean keepAlive,
                        ShotPlanningCoordinator shotPlanner,
                        Telemetry telemetry) {
        this(ledger, firingHelper, shooter, targetRPM, keepAlive, shotPlanner, null, telemetry);
    }

    /**
     * Create a new FireOperation (full constructor with shouldContinue callback)
     *
     * @param ledger Slot ledger to update
     * @param firingHelper BasicFiringHelper for firing control
     * @param shooter Shooter instance for RPM check
     * @param targetRPM Target RPM for firing
     * @param keepAlive If true, keeps shooter running after shot for faster follow-up
     * @param shotPlanner Shot planning coordinator (null if not using)
     * @param shouldContinueCallback Callback to check if we should continue firing (null = always continue)
     * @param telemetry Telemetry for logging
     */
    public FireOperation(SlotLedger ledger,
                        BasicFiringHelper firingHelper,
                        Shooter shooter,
                        double targetRPM,
                        boolean keepAlive,
                        ShotPlanningCoordinator shotPlanner,
                        ShouldContinueCallback shouldContinueCallback,
                        Telemetry telemetry) {
        super(telemetry, FIRE_TIMEOUT_MS);
        this.ledger = ledger;
        this.firingHelper = firingHelper;
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        this.keepAlive = keepAlive;
        this.shotPlanner = shotPlanner;
        this.shouldContinueCallback = shouldContinueCallback;
        this.firedArtifact = null;
        this.isSubsequentShot = false;
        this.shotActuallyFired = false;
        this.shotCountAtStart = 0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OPERATION IMPLEMENTATION
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    protected boolean doStart() {
        // Record shot count at start for reliable shot-fired detection
        shotCountAtStart = firingHelper.getShotsFiredCount();
        
        // Check preconditions
        if (!ledger.isCenterOccupied()) {
            fail(RejectReason.CENTER_EMPTY);
            setStatusMessage("Center slot is empty");
            return false;
        }

        // Get artifact that will be fired
        firedArtifact = ledger.getCenter();

        // Check if this is a subsequent shot (shooter already spun up in keep-alive mode)
        isSubsequentShot = firingHelper.isReadyForNextShot();

        if (isSubsequentShot) {
            // Subsequent shot: shooter already spinning, just fire
            // No RPM check needed - shooter is already at target
            if (!firingHelper.fireShot()) {
                fail(RejectReason.HARDWARE_BUSY);
                setStatusMessage("Failed to fire shot (not in READY_TO_FIRE state)");
                return false;
            }
            setStatusMessage("Firing " + firedArtifact.getColorClass() + " (subsequent shot)");
            return true;
        }

        // First shot: need to spin up shooter
        // Check shooter readiness (only if not using keep-alive, since keep-alive will spin up)
        if (!keepAlive && !shooter.isReadyToFire()) {
            fail(RejectReason.SHOOTER_NOT_READY);
            setStatusMessage("Shooter not ready (RPM: " + 
                           String.format("%.0f", shooter.getCurrentRPM()) + 
                           " / " + String.format("%.0f", targetRPM) + ")");
            return false;
        }

        // Check if already firing
        if (firingHelper.isFiring() && !firingHelper.isReadyForNextShot()) {
            fail(RejectReason.HARDWARE_BUSY);
            setStatusMessage("Already firing");
            return false;
        }

        // Start firing sequence with keep-alive mode
        if (!firingHelper.startFiring(targetRPM, "CUSTOM", keepAlive)) {
            fail(RejectReason.HARDWARE_BUSY);
            setStatusMessage("Failed to start firing");
            return false;
        }

        setStatusMessage("Firing " + firedArtifact.getColorClass() + 
                        (keepAlive ? " (keep-alive)" : " (single-shot)"));
        return true;
    }

    @Override
    protected boolean doUpdate() {
        // Check if we should continue firing (manual override detection)
        if (shouldContinueCallback != null && !shouldContinueCallback.shouldContinue()) {
            // User wants to cancel (e.g., released fire button)
            setStatusMessage("Cancelled - shouldContinue returned false");
            logInfo("Firing cancelled by shouldContinue callback");
            
            // Determine if shot actually fired based on helper state
            // Conservative assumption: if we're past FEEDING state, assume shot fired
            // This is mechanically safe - can't "unshoot" an artifact
            shotActuallyFired = determineShotFired();
            
            if (shotActuallyFired) {
                logInfo("Shot was fired before cancellation - will consume");
            } else {
                logInfo("Shot NOT fired - cancelling before physical shot");
                wasCancelledBeforeShot = true;  // Mark for explicit telemetry
            }
            
            // Cancel the firing helper (stop shooter if not keep-alive)
            firingHelper.cancelFiring();
            
            // If shot actually fired, we complete normally (doCommit will run)
            // If shot NOT fired, we fail without consuming
            if (!shotActuallyFired) {
                fail(RejectReason.GATING_MANUAL_INPUT_DETECTED);  // Don't commit if no shot fired
            }
            
            return false;  // Done (cancelled or completed)
        }

        // Check if shot is complete
        // In keep-alive mode, we're done when we reach READY_TO_FIRE (shot fired, ready for next)
        // In single-shot mode, we're done when firing stops completely
        
        if (keepAlive || isSubsequentShot) {
            // Keep-alive mode: done when ready for next shot (shot complete, shooter still spinning)
            if (firingHelper.isReadyForNextShot()) {
                setStatusMessage("Shot complete (ready for next)");
                shotActuallyFired = true;  // Definitely fired if we reached READY_TO_FIRE
                return false;  // Done
            }
            
            // Still firing
            if (!firingHelper.isFiring()) {
                // Firing stopped unexpectedly (shouldn't happen in keep-alive mode)
                // Conservative: assume shot fired if we got this far
                shotActuallyFired = true;
                setStatusMessage("Firing stopped unexpectedly");
                return false;  // Done (error case)
            }
        } else {
            // Single-shot mode: done when firing completely stops
            if (!firingHelper.isFiring()) {
                setStatusMessage("Firing complete");
                shotActuallyFired = true;  // Finished normally, shot fired
                return false;  // Done
            }
        }

        setStatusMessage("Firing... (" + firingHelper.getFiringState() + ")");
        return true;  // Still running
    }
    
    /**
     * Determine if a shot actually fired based on helper's shot counter
     * This is the most reliable method - BasicFiringHelper increments counter
     * only when feeding completes (artifact physically ejected)
     *
     * @return true if shot physically occurred
     */
    private boolean determineShotFired() {
        // Check if shot count increased since we started
        int currentCount = firingHelper.getShotsFiredCount();
        return currentCount > shotCountAtStart;
    }

    @Override
    protected void doCommit() {
        // Clear center slot (artifact has been fired)
        // This is only called if state == COMPLETE (shot actually fired)
        ledger.setCenter(null);
        
        logInfo("Fired artifact: " + firedArtifact.getColorClass() + 
               " (conf=" + String.format("%.2f", firedArtifact.getColorConfidence()) + 
               ", shotFired=" + shotActuallyFired + ")");
        
        // Phase 5: Consume shot from plan
        if (shotPlanner != null) {
            shotPlanner.consumeShot();
            logInfo("Shot consumed from plan (ONE shot = ONE consumption)");
        }
        
        logDebug("Slot Ledger", ledger.toSnapshot());
    }

    @Override
    public String getOperationName() {
        if (keepAlive || isSubsequentShot) {
            return "Fire[CENTER,KeepAlive]";
        }
        return "Fire[CENTER]";
    }

    @Override
    protected boolean isCancellable() {
        // Firing can be cancelled (stop shooter/uptake)
        return true;
    }

    @Override
    protected void doCancel() {
        // Cancel firing sequence
        firingHelper.cancelFiring();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get next slot to transfer after firing (Phase 5)
     * Call this after commit() to determine which artifact should move to center
     *
     * @return FRONT or BACK slot to transfer, or null if no more shots
     */
    public SlotLedger.Slot getNextTransferSlot() {
        if (shotPlanner == null) {
            return null;  // No shot planning
        }

        return shotPlanner.getNextTransferSlot(ledger);
    }

    /**
     * Check if shooter is in keep-alive mode (ready for rapid follow-up shots)
     * Use this to determine if you can queue another FireOperation immediately
     *
     * @return true if shooter is spun up and waiting for next shot
     */
    public boolean isShooterReadyForNext() {
        return firingHelper.isReadyForNextShot();
    }

    /**
     * Check if keep-alive mode is enabled for this operation
     *
     * @return true if keep-alive mode enabled
     */
    public boolean isKeepAliveEnabled() {
        return keepAlive;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        // Show explicit state (helps OperationRunner distinguish outcomes)
        if (wasCancelledBeforeShot) {
            telemetry.addData("Result", "❌ CANCELLED (before shot)");
        } else if (getState() == OperationState.FAILED) {
            telemetry.addData("Result", "❌ FAILED (" + getFailureReason() + ")");
        } else if (shotActuallyFired) {
            telemetry.addData("Result", "✅ COMPLETE (shot fired)");
        }
        
        telemetry.addData("Mode", keepAlive ? "Keep-Alive" : "Single-Shot");
        telemetry.addData("Shot Type", isSubsequentShot ? "Subsequent" : "First");
        telemetry.addData("Shot Fired", shotActuallyFired ? "YES" : "NO");
        telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
        telemetry.addData("Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
        telemetry.addData("Firing State", firingHelper.getFiringState());
        telemetry.addData("Ready for Next", firingHelper.isReadyForNextShot() ? "YES" : "NO");
        
        if (firedArtifact != null) {
            telemetry.addData("Fired Artifact", firedArtifact.getColorClass());
            telemetry.addData("Confidence", String.format("%.2f", 
                             firedArtifact.getColorConfidence()));
        }
        
        if (shotPlanner != null) {
            SlotLedger.Slot nextSlot = getNextTransferSlot();
            telemetry.addData("Next Transfer", nextSlot != null ? nextSlot : "None");
        }
    }
}
