package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.ShotPlanner;

import java.util.List;

/**
 * ShotPlanningCoordinator - Bridges ShotPlanner with v3 operations
 *
 * This coordinator:
 * - Calls ShotPlanner.updateShotPlan() every loop with SlotLedger
 * - Determines if rearrangement is needed
 * - Determines next artifact to transfer after firing
 * - Provides shot plan consumption interface
 *
 * Phase 5 Integration:
 * - FireOperation calls consumeShot() after firing
 * - Main controller calls isRearrangementNeeded() to trigger SwapOperation
 * - Operations use getNextTransferSlot() to know which artifact to move
 *
 * Design:
 * - ShotPlanner now works directly with v3 data model (ArtifactIdentity + SlotLedger)
 * - Coordinator manages consumption state and telemetry
 * - No more Artifact → ArtifactIdentity conversion needed
 */
public class ShotPlanningCoordinator {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final ShotPlanner planner;
    private final Telemetry telemetry;


    // Shot plan consumption state
    private int shotsFired;
    private boolean planActive;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new ShotPlanningCoordinator
     *
     * @param planner Existing ShotPlanner instance
     * @param telemetry Telemetry for logging
     */
    public ShotPlanningCoordinator(ShotPlanner planner, Telemetry telemetry) {
        this.planner = planner;
        this.telemetry = telemetry;
        this.shotsFired = 0;
        this.planActive = false;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update shot plan based on current slot ledger state
     * Call this every loop (before operations)
     *
     * @param ledger Current slot ledger
     * @param manualMode True if manual control active (disables auto-rearrangement)
     */
    public void update(SlotLedger ledger, boolean manualMode) {
        // Update shot planner with v3 data directly
        planner.setManualPushMode(manualMode);
        boolean planned = planner.updateShotPlan(ledger);

        planActive = planned && ledger.getArtifactCount() > 0;

        // Log planning result
        if (planActive) {
            logDebug("Shot Plan", getShotPlanString());
            if (isRearrangementNeeded()) {
                SlotLedger.Slot slot = getRearrangementSlot();
                logDebug("Rearrangement", "Swap " + (slot != null ? slot : "Unknown") + " with CENTER");
            }
        }
    }

    /**
     * Check if rearrangement is needed
     * Use this to trigger SwapOperation
     *
     * @return true if planner recommends rearrangement
     */
    public boolean isRearrangementNeeded() {
        return planner.isRearrangementNeeded();
    }

    /**
     * Get which slot should be swapped with center
     * Only valid if isRearrangementNeeded() returns true
     *
     * @return FRONT or BACK slot that should be swapped, or null
     */
    public SlotLedger.Slot getRearrangementSlot() {
        return planner.getDesiredSwapSlot();
    }

    /**
     * Get next slot to transfer to center after firing
     * Use this after FireOperation to determine which artifact to move next
     *
     * @param ledger Current slot ledger (after firing)
     * @return FRONT or BACK slot to transfer, or null if no more shots
     */
    public SlotLedger.Slot getNextTransferSlot(SlotLedger ledger) {
        // Get current shot plan (ArtifactIdentity list)
        List<ArtifactIdentity> plan = planner.getShotPlan();

        if (plan.isEmpty() || shotsFired >= plan.size()) {
            return null;  // No more shots in plan
        }

        // Next artifact in plan
        ArtifactIdentity nextArtifact = plan.get(shotsFired);

        // Find which slot has this artifact (match by sequence ID)
        ArtifactIdentity frontIdentity = ledger.getFront();
        ArtifactIdentity backIdentity = ledger.getBack();

        if (frontIdentity != null && frontIdentity.getSequenceId() == nextArtifact.getSequenceId()) {
            return SlotLedger.Slot.FRONT;
        }

        if (backIdentity != null && backIdentity.getSequenceId() == nextArtifact.getSequenceId()) {
            return SlotLedger.Slot.BACK;
        }

        // Fallback: prefer front if available
        if (ledger.isOccupied(SlotLedger.Slot.FRONT)) {
            return SlotLedger.Slot.FRONT;
        } else if (ledger.isOccupied(SlotLedger.Slot.BACK)) {
            return SlotLedger.Slot.BACK;
        }

        return null;
    }

    /**
     * Mark a shot as fired (advance shot plan consumption)
     * Call this after FireOperation commits
     */
    public void consumeShot() {
        shotsFired++;
        logInfo("Shot consumed (" + shotsFired + " fired)");
    }

    /**
     * Reset shot plan consumption
     * Call this when starting a new batch of shots
     */
    public void resetConsumption() {
        shotsFired = 0;
        planActive = false;
        logInfo("Shot plan consumption reset");
    }

    /**
     * Set motif pattern for shot ordering
     * @param pattern One of "PPG", "PGP", or "GPP"
     */
    public void setMotifPattern(String pattern) {
        planner.setMotifPattern(pattern);
        logInfo("Motif pattern set to: " + pattern);
    }

    /**
     * Get current shot plan as string for telemetry
     */
    public String getShotPlanString() {
        List<ArtifactIdentity> plan = planner.getShotPlan();
        if (plan.isEmpty()) {
            return "Empty";
        }

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < plan.size(); i++) {
            if (i > 0) sb.append(" → ");
            ArtifactIdentity.ColorClass color = plan.get(i).getColorClass();
            sb.append(color == ArtifactIdentity.ColorClass.PURPLE ? 'P' :
                     color == ArtifactIdentity.ColorClass.GREEN ? 'G' : '?');
        }
        return sb.toString();
    }


    // ═══════════════════════════════════════════════════════════════════════
    // LOGGING HELPERS
    // ═══════════════════════════════════════════════════════════════════════

    private void logInfo(String message) {
        if (telemetry != null) {
            telemetry.addData("[ShotPlan]", message);
        }
    }

    private void logDebug(String key, Object value) {
        if (telemetry != null) {
            telemetry.addData("  " + key, value);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Add shot planning telemetry
     */
    public void addTelemetry() {
        telemetry.addData("═══ Shot Planning ═══", "");
        telemetry.addData("Plan Active", planActive);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Shot Plan", getShotPlanString());
        telemetry.addData("Motif Pattern", planner.getMotifPattern());
        
        if (isRearrangementNeeded()) {
            telemetry.addData("Rearrangement", "Needed");
            SlotLedger.Slot slot = getRearrangementSlot();
            telemetry.addData("  Swap Slot", slot != null ? slot : "Unknown");
        } else {
            telemetry.addData("Rearrangement", "Not needed");
        }
    }
}
