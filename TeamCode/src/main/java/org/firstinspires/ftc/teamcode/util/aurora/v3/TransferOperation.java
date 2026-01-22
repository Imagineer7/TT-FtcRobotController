package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;

/**
 * TransferOperation - Transfer artifact from intake slot to center
 *
 * This operation demonstrates Checkpoint 2 (transfer completion + settle + resample):
 * 1. Check preconditions (source occupied, center empty)
 * 2. Start transfer hardware (injector + uptake servos)
 * 3. Wait for hardware completion
 * 4. Wait TRANSFER_SETTLE_DELAY for artifact to settle in center
 * 5. Sample color at Checkpoint 2 (potentially improved reading)
 * 6. Commit artifact to center slot with updated color
 *
 * Preconditions:
 * - Source slot occupied
 * - Center slot empty
 *
 * Hardware: Uses BasicIndexingHelper.transferXIntakeToCenterTimed()
 */
public class TransferOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final IntakePerception perception;
    private final BasicIndexingHelper helper;
    private final IndexingConfig config;
    private final SlotLedger.Slot sourceSlot;

    private ArtifactIdentity transferredArtifact;
    private boolean hardwareComplete;
    private long hardwareCompleteTime;
    private boolean colorResampled;

    // Transfer settle delay (Checkpoint 2 policy)
    private static final long TRANSFER_SETTLE_DELAY_MS = 200;
    
    // Transfer timeout
    private static final long TRANSFER_TIMEOUT_MS = 4000;  // 4 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new TransferOperation
     *
     * @param ledger Slot ledger to update
     * @param perception Intake perception for color resampling (NOTE: will need center perception in full system)
     * @param helper BasicIndexingHelper for hardware control
     * @param config IndexingConfig for parameters
     * @param sourceSlot Which slot to transfer from (FRONT or BACK)
     * @param telemetry Telemetry for logging
     */
    public TransferOperation(SlotLedger ledger,
                            IntakePerception perception,
                            BasicIndexingHelper helper,
                            IndexingConfig config,
                            SlotLedger.Slot sourceSlot,
                            Telemetry telemetry) {
        super(telemetry, TRANSFER_TIMEOUT_MS);
        this.ledger = ledger;
        this.perception = perception;
        this.helper = helper;
        this.config = config;
        this.sourceSlot = sourceSlot;
        this.transferredArtifact = null;
        this.hardwareComplete = false;
        this.hardwareCompleteTime = 0;
        this.colorResampled = false;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OPERATION IMPLEMENTATION
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    protected boolean doStart() {
        // Check preconditions
        if (!ledger.isOccupied(sourceSlot)) {
            fail(RejectReason.SLOT_EMPTY);
            setStatusMessage("Source slot " + sourceSlot + " is empty");
            return false;
        }

        if (ledger.isCenterOccupied()) {
            fail(RejectReason.CENTER_OCCUPIED);
            setStatusMessage("Center slot is occupied");
            return false;
        }

        // Get artifact from source slot
        transferredArtifact = ledger.get(sourceSlot);

        // Start transfer hardware
        if (sourceSlot == SlotLedger.Slot.FRONT) {
            helper.transferFrontIntakeToCenterTimed();
        } else if (sourceSlot == SlotLedger.Slot.BACK) {
            helper.transferBackIntakeToCenterTimed();
        } else {
            fail(RejectReason.INVALID_PARAMETERS);
            setStatusMessage("Invalid source slot: " + sourceSlot);
            return false;
        }

        setStatusMessage("Transferring " + sourceSlot + " → CENTER");
        return true;
    }

    @Override
    protected boolean doUpdate() {
        // Check if hardware is complete
        if (!hardwareComplete) {
            boolean stillBusy;
            if (sourceSlot == SlotLedger.Slot.FRONT) {
                stillBusy = helper.isFrontTransferBusy() || helper.isInjectorLeftBusy() || 
                           helper.isInjectorRightBusy() || helper.isUptakeLBusy() || helper.isUptakeRBusy();
            } else {
                stillBusy = helper.isBackTransferBusy() || helper.isInjectorLeftBusy() || 
                           helper.isInjectorRightBusy() || helper.isUptakeLBusy() || helper.isUptakeRBusy();
            }

            if (!stillBusy) {
                hardwareComplete = true;
                hardwareCompleteTime = System.currentTimeMillis();
                setStatusMessage("Hardware complete, waiting for settle");
            } else {
                setStatusMessage("Transferring...");
                return true;  // Still busy
            }
        }

        // Checkpoint 2: Color resampling after settle delay
        if (hardwareComplete && !colorResampled) {
            long elapsedSinceComplete = System.currentTimeMillis() - hardwareCompleteTime;
            
            if (elapsedSinceComplete >= TRANSFER_SETTLE_DELAY_MS) {
                // Enable color sampling
                perception.enableColorSampling();
                
                // Resample color - artifact has settled in new location
                ArtifactIdentity.ColorClass newColor = perception.getBestColorClass();
                double newConfidence = perception.getBestColorConfidence();
                
                // Disable color sampling
                perception.disableColorSampling();
                
                // Update artifact with potentially improved color reading
                transferredArtifact = transferredArtifact.withUpdatedColor(
                    newColor, newConfidence, ArtifactIdentity.ClassificationSource.COLOR_SENSOR
                );
                
                colorResampled = true;
                
                // Log color update
                logInfo("Color checkpoint 2: " + newColor + " (conf=" + 
                       String.format("%.2f", newConfidence) + ")");
                setStatusMessage("Transfer complete with " + newColor);
                return false;  // Done
            } else {
                // Still waiting for settle
                long remaining = TRANSFER_SETTLE_DELAY_MS - elapsedSinceComplete;
                setStatusMessage("Settling (" + remaining + "ms)");
                return true;
            }
        }

        return false;  // Complete
    }

    @Override
    protected void doCommit() {
        // Move artifact from source to center
        ledger.clear(sourceSlot);
        ledger.setCenter(transferredArtifact);
        
        logInfo("Committed transfer: " + sourceSlot + " → CENTER");
        logDebug("Artifact", transferredArtifact.toString());
        logDebug("Slot Ledger", ledger.toSnapshot());
    }

    @Override
    public String getOperationName() {
        return "Transfer[" + sourceSlot + "→CENTER]";
    }

    @Override
    protected void doCancel() {
        // Stop transfer hardware
        if (sourceSlot == SlotLedger.Slot.FRONT) {
            helper.stopFrontTransfer();
        } else {
            helper.stopBackTransfer();
        }
        helper.stopInjectorLeft();
        helper.stopInjectorRight();
        helper.stopUptakeL();
        helper.stopUptakeR();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        telemetry.addData("Source Slot", sourceSlot);
        telemetry.addData("Hardware Complete", hardwareComplete);
        telemetry.addData("Color Resampled", colorResampled);
        
        if (transferredArtifact != null) {
            telemetry.addData("Artifact Color", transferredArtifact.getColorClass());
            telemetry.addData("Confidence", String.format("%.2f", 
                             transferredArtifact.getColorConfidence()));
        }
    }
}
