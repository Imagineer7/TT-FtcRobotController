package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;

/**
 * FireOperation - Fire center artifact via BasicFiringHelper
 *
 * This operation demonstrates shot plan consumption:
 * 1. Check preconditions (center occupied, prepositioned, shooter ready)
 * 2. Start firing sequence via BasicFiringHelper
 * 3. Wait for completion
 * 4. Clear center slot
 * 5. Consume shot from plan (if coordinator provided)
 *
 * Preconditions:
 * - Center slot occupied
 * - Artifact prepositioned
 * - Shooter at target RPM and stable
 * - Not already firing
 *
 * Hardware: Uses BasicFiringHelper.startFiring()
 * Phase 5: Integrates with ShotPlanningCoordinator
 */
public class FireOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final BasicFiringHelper firingHelper;
    private final Shooter shooter;
    private final double targetRPM;
    private final ShotPlanningCoordinator shotPlanner;  // Optional (Phase 5)

    private ArtifactIdentity firedArtifact;

    // Fire timeout
    private static final long FIRE_TIMEOUT_MS = 8000;  // 8 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTORS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new FireOperation (without shot planning)
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
        this(ledger, firingHelper, shooter, targetRPM, null, telemetry);
    }

    /**
     * Create a new FireOperation (with shot planning - Phase 5)
     *
     * @param ledger Slot ledger to update
     * @param firingHelper BasicFiringHelper for firing control
     * @param shooter Shooter instance for RPM check
     * @param targetRPM Target RPM for firing
     * @param shotPlanner Shot planning coordinator (null if not using)
     * @param telemetry Telemetry for logging
     */
    public FireOperation(SlotLedger ledger,
                        BasicFiringHelper firingHelper,
                        Shooter shooter,
                        double targetRPM,
                        ShotPlanningCoordinator shotPlanner,
                        Telemetry telemetry) {
        super(telemetry, FIRE_TIMEOUT_MS);
        this.ledger = ledger;
        this.firingHelper = firingHelper;
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        this.shotPlanner = shotPlanner;
        this.firedArtifact = null;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OPERATION IMPLEMENTATION
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    protected boolean doStart() {
        // Check preconditions
        if (!ledger.isCenterOccupied()) {
            fail(RejectReason.CENTER_EMPTY);
            setStatusMessage("Center slot is empty");
            return false;
        }

        // Check shooter readiness
        if (!shooter.isReadyToFire()) {
            fail(RejectReason.SHOOTER_NOT_READY);
            setStatusMessage("Shooter not ready (RPM: " + 
                           String.format("%.0f", shooter.getCurrentRPM()) + 
                           " / " + String.format("%.0f", targetRPM) + ")");
            return false;
        }

        // Check if already firing
        if (firingHelper.isFiring()) {
            fail(RejectReason.HARDWARE_BUSY);
            setStatusMessage("Already firing");
            return false;
        }

        // Get artifact that will be fired
        firedArtifact = ledger.getCenter();

        // Start firing sequence
        firingHelper.startFiring(targetRPM);

        setStatusMessage("Firing " + firedArtifact.getColorClass() + " artifact");
        return true;
    }

    @Override
    protected boolean doUpdate() {
        // Check if firing is complete
        boolean stillFiring = firingHelper.isFiring();

        if (!stillFiring) {
            setStatusMessage("Firing complete");
            return false;  // Done
        }

        setStatusMessage("Firing... (" + firingHelper.getFiringState() + ")");
        return true;  // Still running
    }

    @Override
    protected void doCommit() {
        // Clear center slot (artifact has been fired)
        ledger.setCenter(null);
        
        logInfo("Fired artifact: " + firedArtifact.getColorClass() + 
               " (conf=" + String.format("%.2f", firedArtifact.getColorConfidence()) + ")");
        
        // Phase 5: Consume shot from plan
        if (shotPlanner != null) {
            shotPlanner.consumeShot();
            logInfo("Shot consumed from plan");
        }
        
        logDebug("Slot Ledger", ledger.toSnapshot());
    }

    @Override
    public String getOperationName() {
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

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
        telemetry.addData("Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
        telemetry.addData("Firing State", firingHelper.getFiringState());
        
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
