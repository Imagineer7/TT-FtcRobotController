package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;

/**
 * PrepositionOperation - Position center artifact for firing
 *
 * This operation prepares the center artifact for firing by:
 * 1. Check preconditions (center occupied, not already prepositioned)
 * 2. Run uptake servos briefly to position artifact against shooter
 * 3. Wait for completion
 * 4. Set preposition flag (tracked externally)
 *
 * Preconditions:
 * - Center slot occupied
 * - Not already prepositioned
 *
 * Hardware: Uses BasicIndexingHelper.prePositionArtifacts()
 */
public class PrepositionOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final BasicIndexingHelper helper;
    private boolean prepositioned;

    // Preposition timeout
    private static final long PREPOSITION_TIMEOUT_MS = 2000;  // 2 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new PrepositionOperation
     *
     * @param ledger Slot ledger to check
     * @param helper BasicIndexingHelper for hardware control
     * @param prepositioned Current preposition state (external tracking)
     * @param telemetry Telemetry for logging
     */
    public PrepositionOperation(SlotLedger ledger,
                               BasicIndexingHelper helper,
                               boolean prepositioned,
                               Telemetry telemetry) {
        super(telemetry, PREPOSITION_TIMEOUT_MS);
        this.ledger = ledger;
        this.helper = helper;
        this.prepositioned = prepositioned;
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

        if (prepositioned) {
            fail(RejectReason.INVALID_PARAMETERS);
            setStatusMessage("Already prepositioned");
            return false;
        }

        // Start preposition hardware
        helper.prePositionArtifacts();

        setStatusMessage("Pre-positioning artifact");
        return true;
    }

    @Override
    protected boolean doUpdate() {
        // Check if preposition hardware is complete
        boolean stillBusy = helper.isPrePositioning();

        if (!stillBusy) {
            setStatusMessage("Preposition complete");
            return false;  // Done
        }

        setStatusMessage("Pre-positioning...");
        return true;  // Still running
    }

    @Override
    protected void doCommit() {
        // Preposition flag is tracked externally (by main controller)
        // This operation just confirms hardware completion
        logInfo("Preposition complete - artifact ready to fire");
        logDebug("Center Artifact", ledger.getCenter().toString());
    }

    @Override
    public String getOperationName() {
        return "Preposition[CENTER]";
    }

    @Override
    protected void doCancel() {
        // Stop preposition hardware (reverse uptake briefly)
        helper.unPrePositionArtifacts();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        if (ledger.isCenterOccupied()) {
            telemetry.addData("Center Artifact", ledger.getCenter().getColorClass());
        }
    }
}
