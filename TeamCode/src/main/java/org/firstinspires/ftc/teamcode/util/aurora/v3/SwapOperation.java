package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;

/**
 * SwapOperation - Push-based swap between center and intake slot
 *
 * This operation performs a PUSH-STYLE rearrangement where the artifact from the
 * intake PUSHES the center artifact out to the OPPOSITE intake, and takes its place.
 *
 * Physical constraint: You CANNOT put two artifacts in the same intake simultaneously.
 * Therefore, this is NOT a literal swap. Instead:
 * - If swapping with FRONT: intake artifact pushes center artifact to BACK
 * - If swapping with BACK: intake artifact pushes center artifact to FRONT
 *
 * Example: Swap(CENTER ↔ FRONT) with artifacts [FRONT:P, CENTER:G, BACK:empty]
 * Result: [FRONT:empty, CENTER:P, BACK:G]
 * The Purple from FRONT pushed the Green from CENTER to BACK, and Purple took center.
 *
 * Preconditions:
 * - Center slot occupied
 * - Target intake slot occupied  
 * - Opposite intake slot EMPTY (critical - must have space for pushed artifact)
 * - Exactly 2 artifacts total (ensures clean swap)
 * - System not in manual mode
 *
 * Hardware: Uses push mechanics (intake → center, center → opposite intake)
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

        // CRITICAL: Verify opposite intake is EMPTY (must have space for pushed artifact)
        SlotLedger.Slot oppositeIntake = (intakeSlot == SlotLedger.Slot.FRONT) ? 
                                          SlotLedger.Slot.BACK : SlotLedger.Slot.FRONT;
        if (ledger.isOccupied(oppositeIntake)) {
            fail(RejectReason.SLOT_OCCUPIED);
            setStatusMessage("Opposite intake " + oppositeIntake + " must be empty for push-based swap");
            return false;
        }

        // Get artifacts for swap
        centerArtifact = ledger.getCenter();
        intakeArtifact = ledger.get(intakeSlot);

        // Start PUSH-BASED swap hardware
        // Physical constraint: Cannot put two artifacts in same intake simultaneously
        // Solution: Use intake artifact to PUSH center artifact to opposite intake
        //
        // Flow:
        // 1. Intake artifact → Center (via transfer + injectors forward)
        // 2. Center artifact → Opposite intake (pushed by incoming artifact)
        
        if (intakeSlot == SlotLedger.Slot.FRONT) {
            // Front artifact pushes center artifact to back
            // Front → Center: Pull via transfer + injectors
            helper.setBackTransferTimed(-1.0, 3000);  // Reverse to receive pushed artifact
            helper.transferFrontIntakeToCenterTimed(2500);  // Pull front to center

        } else if (intakeSlot == SlotLedger.Slot.BACK) {
            // Back artifact pushes center artifact to front
            // Back → Center: Pull via transfer + injectors
            helper.setFrontTransferTimed(-1.0, 3000);  // Reverse to receive pushed artifact
            helper.transferBackIntakeToCenterTimed(2500);  // Pull back to center
        } else {
            fail(RejectReason.INVALID_PARAMETERS);
            setStatusMessage("Invalid intake slot: " + intakeSlot);
            return false;
        }

        setStatusMessage("Push-swapping: " + intakeSlot + " → CENTER, CENTER → " + oppositeIntake);
        return true;
    }

    @Override
    protected boolean doUpdate() {
        // Check if hardware is complete
        // For push-based swap, we need to check:
        // - Transfer from intake (pulling to center)
        // - Injectors (pulling to center)
        // - Uptake (pushing out to opposite)
        // - Transfer to opposite (receiving pushed artifact)
        
        boolean stillBusy;
        
        if (intakeSlot == SlotLedger.Slot.FRONT) {
            // Front → Center, Center → Back
            stillBusy = helper.isFrontTransferBusy() ||  // Pulling front to center
                       helper.isInjectorLeftBusy() || helper.isInjectorRightBusy() ||  // Pulling to center
                       helper.isUptakeLBusy() || helper.isUptakeRBusy() ||  // Pushing out to back
                       helper.isBackTransferBusy();  // Receiving at back
        } else {
            // Back → Center, Center → Front
            stillBusy = helper.isBackTransferBusy() ||  // Pulling back to center
                       helper.isInjectorLeftBusy() || helper.isInjectorRightBusy() ||  // Pulling to center
                       helper.isUptakeLBusy() || helper.isUptakeRBusy() ||  // Pushing out to front
                       helper.isFrontTransferBusy();  // Receiving at front
        }

        if (!stillBusy) {
            setStatusMessage("Push-swap hardware complete");
            return false;  // Done
        }

        setStatusMessage("Push-swapping...");
        return true;  // Still running
    }

    @Override
    protected void doCommit() {
        // Push-based swap: 3 slot updates
        // 1. intakeSlot artifact → CENTER
        // 2. CENTER artifact → opposite intake
        // 3. intakeSlot becomes empty
        
        SlotLedger.Slot oppositeIntake = (intakeSlot == SlotLedger.Slot.FRONT) ? 
                                          SlotLedger.Slot.BACK : SlotLedger.Slot.FRONT;
        
        // Execute the push-based swap atomically
        ArtifactIdentity centerToMove = ledger.getCenter();
        ArtifactIdentity intakeToMove = ledger.get(intakeSlot);
        
        // Update slots (package-private access)
        ledger.set(oppositeIntake, centerToMove);  // Center artifact goes to opposite
        ledger.setCenter(intakeToMove);            // Intake artifact goes to center
        ledger.set(intakeSlot, null);              // Source intake becomes empty
        
        logInfo("Committed push-swap: " + intakeSlot + " → CENTER, CENTER → " + oppositeIntake);
        logDebug("Pushed", intakeToMove + " to center, " + centerToMove + " to " + oppositeIntake);
        logDebug("Slot Ledger", ledger.toSnapshot());
    }

    @Override
    public String getOperationName() {
        SlotLedger.Slot oppositeIntake = (intakeSlot == SlotLedger.Slot.FRONT) ? 
                                          SlotLedger.Slot.BACK : SlotLedger.Slot.FRONT;
        return "PushSwap[" + intakeSlot + "→CENTER→" + oppositeIntake + "]";
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
