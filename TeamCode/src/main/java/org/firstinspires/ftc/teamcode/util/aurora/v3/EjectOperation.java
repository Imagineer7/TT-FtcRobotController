package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;

/**
 * EjectOperation - Clear artifacts from robot (emergency operation)
 *
 * This operation clears all artifacts by:
 * 1. Check preconditions (at least one artifact present)
 * 2. Start ejection hardware (reverse intakes + low-speed shooter + forward uptake)
 * 3. Wait for completion
 * 4. Clear all slots
 *
 * Preconditions:
 * - At least one artifact present (or none to clear slots anyway)
 *
 * Hardware: Uses BasicFiringHelper.startEjection() or manual ejection sequence
 *
 * Note: This is an emergency operation that bypasses normal safety checks
 */
public class EjectOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final BasicFiringHelper firingHelper;
    private final BasicIndexingHelper indexingHelper;
    private final EjectMode mode;

    private int artifactsCleared;

    /**
     * Ejection mode
     */
    public enum EjectMode {
        /** Eject all artifacts (clear all slots) */
        ALL,
        /** Eject center artifact only */
        CENTER,
        /** Software-only clear (no hardware) */
        SOFTWARE_CLEAR
    }

    // Eject timeout
    private static final long EJECT_TIMEOUT_MS = 6000;  // 6 seconds
    
    // Ejection duration
    private static final long EJECTION_DURATION_MS = 3000;  // 3 seconds of ejection
    private long ejectionStartTime;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new EjectOperation
     *
     * @param ledger Slot ledger to update
     * @param firingHelper BasicFiringHelper for ejection control
     * @param indexingHelper BasicIndexingHelper for manual ejection
     * @param mode Ejection mode (ALL, CENTER, SOFTWARE_CLEAR)
     * @param telemetry Telemetry for logging
     */
    public EjectOperation(SlotLedger ledger,
                         BasicFiringHelper firingHelper,
                         BasicIndexingHelper indexingHelper,
                         EjectMode mode,
                         Telemetry telemetry) {
        super(telemetry, EJECT_TIMEOUT_MS);
        this.ledger = ledger;
        this.firingHelper = firingHelper;
        this.indexingHelper = indexingHelper;
        this.mode = mode;
        this.artifactsCleared = 0;
        this.ejectionStartTime = 0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OPERATION IMPLEMENTATION
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    protected boolean doStart() {
        // Count artifacts to clear
        artifactsCleared = ledger.getArtifactCount();

        if (artifactsCleared == 0 && mode != EjectMode.SOFTWARE_CLEAR) {
            logWarn("No artifacts to eject, but clearing slots anyway");
        }

        // Start ejection based on mode
        if (mode == EjectMode.SOFTWARE_CLEAR) {
            // Software-only clear - no hardware
            setStatusMessage("Software clear - no hardware movement");
            return true;
        } else if (mode == EjectMode.ALL) {
            // Eject all artifacts
            firingHelper.startEjection();
            ejectionStartTime = System.currentTimeMillis();
            setStatusMessage("Ejecting all artifacts");
        } else if (mode == EjectMode.CENTER) {
            // Eject center artifact only
            // Run uptake forward + shooter low speed
            indexingHelper.setUptakeLTimed(1.0, EJECTION_DURATION_MS);
            indexingHelper.setUptakeRTimed(1.0, EJECTION_DURATION_MS);
            ejectionStartTime = System.currentTimeMillis();
            setStatusMessage("Ejecting center artifact");
        }

        return true;
    }

    @Override
    protected boolean doUpdate() {
        if (mode == EjectMode.SOFTWARE_CLEAR) {
            // Software clear is instant
            setStatusMessage("Software clear complete");
            return false;  // Done
        }

        // Check if ejection hardware is complete
        boolean stillBusy;
        
        if (mode == EjectMode.ALL) {
            stillBusy = firingHelper.isEjecting();
            
            // Also check time-based completion
            long elapsed = System.currentTimeMillis() - ejectionStartTime;
            if (elapsed >= EJECTION_DURATION_MS) {
                stillBusy = false;
            }
        } else {
            // Center only - check uptake servos
            stillBusy = indexingHelper.isUptakeLBusy() || indexingHelper.isUptakeRBusy();
        }

        if (!stillBusy) {
            setStatusMessage("Ejection hardware complete");
            return false;  // Done
        }

        long elapsed = System.currentTimeMillis() - ejectionStartTime;
        setStatusMessage("Ejecting... (" + elapsed + "ms / " + EJECTION_DURATION_MS + "ms)");
        return true;  // Still running
    }

    @Override
    protected void doCommit() {
        // Clear slots based on mode
        if (mode == EjectMode.ALL || mode == EjectMode.SOFTWARE_CLEAR) {
            ledger.clearAll();
            logInfo("Cleared all slots (" + artifactsCleared + " artifacts)");
        } else if (mode == EjectMode.CENTER) {
            ledger.clearCenter();
            logInfo("Cleared center slot");
        }
        
        logDebug("Slot Ledger", ledger.toSnapshot());
    }

    @Override
    public String getOperationName() {
        return "Eject[" + mode + "]";
    }

    @Override
    protected boolean isCancellable() {
        // Ejection can be cancelled
        return true;
    }

    @Override
    protected void doCancel() {
        // Stop ejection hardware
        if (mode == EjectMode.ALL) {
            firingHelper.cancelEjection();
        } else if (mode == EjectMode.CENTER) {
            indexingHelper.stopUptakeL();
            indexingHelper.stopUptakeR();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        telemetry.addData("Eject Mode", mode);
        telemetry.addData("Artifacts Cleared", artifactsCleared);
        
        if (ejectionStartTime > 0) {
            long elapsed = System.currentTimeMillis() - ejectionStartTime;
            telemetry.addData("Ejection Time", elapsed + "ms / " + EJECTION_DURATION_MS + "ms");
        }
    }
}
