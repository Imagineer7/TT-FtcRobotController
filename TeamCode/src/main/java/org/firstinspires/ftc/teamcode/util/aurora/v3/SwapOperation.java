package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;

/**
 * SwapOperation - Push-style swap between center and intake slot
 *
 * This operation demonstrates atomic rearrangement coordination:
 * 1. Check preconditions (both slots occupied, exactly 2 artifacts)
 * 2. Start push hardware (simultaneous transfer both directions)
 * 3. Wait for completion
 * 4. Atomically swap slot identities
 *
 * Preconditions:
 * - Center slot occupied
 * - Target intake slot occupied
 * - Exactly 2 artifacts total (ensures clean swap)
 * - System not in manual mode
 *
 * Hardware: Uses simultaneous transfer + injector movements (push-style)
 */
public class SwapOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final BasicIndexingHelper helper;
    private final IndexingConfig config;
    private final SlotLedger.Slot intakeSlot;

    private ArtifactIdentity centerArtifact;
    private ArtifactIdentity intakeArtifact;

    // Swap timeout
    private static final long SWAP_TIMEOUT_MS = 5000;  // 5 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new SwapOperation
     *
     * @param ledger Slot ledger to update
     * @param helper BasicIndexingHelper for hardware control
     * @param config IndexingConfig for parameters
     * @param intakeSlot Which intake slot to swap with (FRONT or BACK)
     * @param telemetry Telemetry for logging
     */
    public SwapOperation(SlotLedger ledger,
                        BasicIndexingHelper helper,
                        IndexingConfig config,
                        SlotLedger.Slot intakeSlot,
                        Telemetry telemetry) {
        super(telemetry, SWAP_TIMEOUT_MS);
        this.ledger = ledger;
        this.helper = helper;
        this.config = config;
        this.intakeSlot = intakeSlot;
        this.centerArtifact = null;
        this.intakeArtifact = null;
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

        if (!ledger.isOccupied(intakeSlot)) {
            fail(RejectReason.SLOT_EMPTY);
            setStatusMessage("Intake slot " + intakeSlot + " is empty");
            return false;
        }

        // Verify exactly 2 artifacts for clean swap
        if (ledger.getArtifactCount() != 2) {
            fail(RejectReason.PRECONDITION_WRONG_ARTIFACT_COUNT);
            setStatusMessage("Need exactly 2 artifacts for swap, have " + ledger.getArtifactCount());
            return false;
        }

        // Get artifacts for swap
        centerArtifact = ledger.getCenter();
        intakeArtifact = ledger.get(intakeSlot);

        // Start push-style swap hardware
        // Center → Intake: Run injectors + uptake backward (push center artifact to intake)
        // Intake → Center: Run transfer + injectors forward (pull intake artifact to center)
        
        if (intakeSlot == SlotLedger.Slot.FRONT) {
            // Push center artifact to front intake
            helper.setUptakeLTimed(-1.0, 800);  // Reverse to push out
            helper.setUptakeRTimed(-1.0, 800);
            helper.setInjectorLeftTimed(-1.0, 800);  // Reverse to push to intake
            helper.setInjectorRightTimed(-1.0, 800);
            
            // Pull front artifact to center
            helper.setFrontTransferTimed(1.0, 800);  // Forward to pull to center
        } else if (intakeSlot == SlotLedger.Slot.BACK) {
            // Push center artifact to back intake
            helper.setUptakeLTimed(-1.0, 800);
            helper.setUptakeRTimed(-1.0, 800);
            helper.setInjectorLeftTimed(-1.0, 800);
            helper.setInjectorRightTimed(-1.0, 800);
            
            // Pull back artifact to center
            helper.setBackTransferTimed(1.0, 800);
        } else {
            fail(RejectReason.INVALID_PARAMETERS);
            setStatusMessage("Invalid intake slot: " + intakeSlot);
            return false;
        }

        setStatusMessage("Swapping CENTER ↔ " + intakeSlot);
        return true;
    }

    @Override
    protected boolean doUpdate() {
        // Check if hardware is complete
        boolean stillBusy;
        
        if (intakeSlot == SlotLedger.Slot.FRONT) {
            stillBusy = helper.isUptakeLBusy() || helper.isUptakeRBusy() || 
                       helper.isInjectorLeftBusy() || helper.isInjectorRightBusy() ||
                       helper.isFrontTransferBusy();
        } else {
            stillBusy = helper.isUptakeLBusy() || helper.isUptakeRBusy() || 
                       helper.isInjectorLeftBusy() || helper.isInjectorRightBusy() ||
                       helper.isBackTransferBusy();
        }

        if (!stillBusy) {
            setStatusMessage("Swap hardware complete");
            return false;  // Done
        }

        setStatusMessage("Swapping...");
        return true;  // Still running
    }

    @Override
    protected void doCommit() {
        // Atomically swap artifacts between slots
        ledger.swap(SlotLedger.Slot.CENTER, intakeSlot);
        
        logInfo("Committed swap: CENTER ↔ " + intakeSlot);
        logDebug("Swapped", centerArtifact + " ↔ " + intakeArtifact);
        logDebug("Slot Ledger", ledger.toSnapshot());
    }

    @Override
    public String getOperationName() {
        return "Swap[CENTER↔" + intakeSlot + "]";
    }

    @Override
    protected boolean isCancellable() {
        // Swap is non-cancellable once started (mid-swap state would be inconsistent)
        return false;
    }

    @Override
    protected void doCancel() {
        // Not called since isCancellable() returns false
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        telemetry.addData("Intake Slot", intakeSlot);
        
        if (centerArtifact != null && intakeArtifact != null) {
            telemetry.addData("Center Artifact", centerArtifact.getColorClass());
            telemetry.addData("Intake Artifact", intakeArtifact.getColorClass());
        }
    }
}
