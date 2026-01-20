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
 * 5. (Phase 5) Advance next artifact from shot plan
 *
 * Preconditions:
 * - Center slot occupied
 * - Artifact prepositioned
 * - Shooter at target RPM and stable
 * - Not already firing
 *
 * Hardware: Uses BasicFiringHelper.startFiring()
 */
public class FireOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final BasicFiringHelper firingHelper;
    private final Shooter shooter;
    private final double targetRPM;

    private ArtifactIdentity firedArtifact;

    // Fire timeout
    private static final long FIRE_TIMEOUT_MS = 8000;  // 8 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new FireOperation
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
        super(telemetry, FIRE_TIMEOUT_MS);
        this.ledger = ledger;
        this.firingHelper = firingHelper;
        this.shooter = shooter;
        this.targetRPM = targetRPM;
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

        setStatusMessage("Firing... (" + firingHelper.getFiringStateName() + ")");
        return true;  // Still running
    }

    @Override
    protected void doCommit() {
        // Clear center slot (artifact has been fired)
        ledger.clearCenter();
        
        logInfo("Fired artifact: " + firedArtifact.getColorClass() + 
               " (conf=" + String.format("%.2f", firedArtifact.getColorConfidence()) + ")");
        logDebug("Slot Ledger", ledger.toSnapshot());
        
        // TODO Phase 5: Advance next artifact from shot plan
        // If shot plan has remaining artifacts, queue TransferOperation
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
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
        telemetry.addData("Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
        telemetry.addData("Firing State", firingHelper.getFiringStateName());
        
        if (firedArtifact != null) {
            telemetry.addData("Fired Artifact", firedArtifact.getColorClass());
            telemetry.addData("Confidence", String.format("%.2f", 
                             firedArtifact.getColorConfidence()));
        }
    }
}
