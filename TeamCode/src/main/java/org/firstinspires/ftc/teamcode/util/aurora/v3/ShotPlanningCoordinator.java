package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.Artifact;
import org.firstinspires.ftc.teamcode.util.aurora.ShotPlanner;

import java.util.ArrayList;
import java.util.List;

/**
 * ShotPlanningCoordinator - Bridges existing ShotPlanner with v3 operations
 *
 * This coordinator:
 * - Converts SlotLedger state → Artifact list for ShotPlanner
 * - Calls ShotPlanner.updateShotPlan() every loop
 * - Determines if rearrangement is needed
 * - Determines next artifact to transfer after firing
 * - Provides shot plan consumption interface
 *
 * Phase 5 Integration:
 * - FireOperation calls consumeShot() after firing
 * - Main controller calls requestRearrangement() when needed
 * - Operations use getNextTransferSlot() to know which artifact to move
 *
 * Design:
 * - ShotPlanner remains unchanged (existing tests still pass)
 * - Coordinator adapts between v3 data model and ShotPlanner
 * - Confidence-aware planning (prefer high-confidence artifacts)
 */
public class ShotPlanningCoordinator {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final ShotPlanner planner;
    private final Telemetry telemetry;

    // Current slot mappings for planner
    private Artifact centerArtifact;
    private Artifact frontArtifact;
    private Artifact backArtifact;

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
        this.centerArtifact = null;
        this.frontArtifact = null;
        this.backArtifact = null;
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
        // Convert slot ledger to artifact list + location mappings
        List<Artifact> artifacts = convertLedgerToArtifacts(ledger);

        // Update shot planner
        planner.setManualPushMode(manualMode);
        boolean planned = planner.updateShotPlan(
            artifacts,
            centerArtifact,
            frontArtifact,
            backArtifact
        );

        planActive = planned && !artifacts.isEmpty();

        // Log planning result
        if (planActive) {
            logDebug("Shot Plan", getShotPlanString());
            if (isRearrangementNeeded()) {
                logDebug("Rearrangement", "Desired center: " + 
                        planner.getDesiredCenterArtifact().getColorString());
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
        return planner.getDesiredCenterArtifact() != null;
    }

    /**
     * Get which slot should be swapped with center
     * Only valid if isRearrangementNeeded() returns true
     *
     * @return FRONT or BACK slot that has desired artifact, or null
     */
    public SlotLedger.Slot getRearrangementSlot() {
        Artifact desired = planner.getDesiredCenterArtifact();
        if (desired == null) {
            return null;
        }

        // Find which slot has this artifact
        if (frontArtifact != null && artifactsMatch(frontArtifact, desired)) {
            return SlotLedger.Slot.FRONT;
        } else if (backArtifact != null && artifactsMatch(backArtifact, desired)) {
            return SlotLedger.Slot.BACK;
        }

        return null;
    }

    /**
     * Get next slot to transfer to center after firing
     * Use this after FireOperation to determine which artifact to move next
     *
     * @param ledger Current slot ledger (after firing)
     * @return FRONT or BACK slot to transfer, or null if no more shots
     */
    public SlotLedger.Slot getNextTransferSlot(SlotLedger ledger) {
        // Get current shot plan
        List<Artifact> plan = planner.getShotPlan();
        
        if (plan.isEmpty() || shotsFired >= plan.size()) {
            return null;  // No more shots in plan
        }

        // Next artifact in plan
        Artifact nextArtifact = plan.get(shotsFired);

        // Find which slot has this artifact
        if (ledger.isOccupied(SlotLedger.Slot.FRONT)) {
            Artifact frontArt = convertToArtifact(ledger.getFront(), SlotLedger.Slot.FRONT);
            if (artifactsMatch(frontArt, nextArtifact)) {
                return SlotLedger.Slot.FRONT;
            }
        }

        if (ledger.isOccupied(SlotLedger.Slot.BACK)) {
            Artifact backArt = convertToArtifact(ledger.getBack(), SlotLedger.Slot.BACK);
            if (artifactsMatch(backArt, nextArtifact)) {
                return SlotLedger.Slot.BACK;
            }
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
        List<Artifact> plan = planner.getShotPlan();
        if (plan.isEmpty()) {
            return "Empty";
        }

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < plan.size(); i++) {
            if (i > 0) sb.append(" → ");
            sb.append(plan.get(i).getColorString().charAt(0));  // P or G
        }
        return sb.toString();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CONVERSION HELPERS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Convert SlotLedger to list of Artifacts for ShotPlanner
     */
    private List<Artifact> convertLedgerToArtifacts(SlotLedger ledger) {
        List<Artifact> artifacts = new ArrayList<>();
        centerArtifact = null;
        frontArtifact = null;
        backArtifact = null;

        // Convert center
        if (ledger.isCenterOccupied()) {
            centerArtifact = convertToArtifact(ledger.getCenter(), SlotLedger.Slot.CENTER);
            artifacts.add(centerArtifact);
        }

        // Convert front
        if (ledger.isOccupied(SlotLedger.Slot.FRONT)) {
            frontArtifact = convertToArtifact(ledger.getFront(), SlotLedger.Slot.FRONT);
            artifacts.add(frontArtifact);
        }

        // Convert back
        if (ledger.isOccupied(SlotLedger.Slot.BACK)) {
            backArtifact = convertToArtifact(ledger.getBack(), SlotLedger.Slot.BACK);
            artifacts.add(backArtifact);
        }

        return artifacts;
    }

    /**
     * Convert ArtifactIdentity → Artifact
     */
    private Artifact convertToArtifact(ArtifactIdentity identity, SlotLedger.Slot slot) {
        // Convert color class
        String colorString;
        switch (identity.getColorClass()) {
            case PURPLE:
                colorString = "PURPLE";
                break;
            case GREEN:
                colorString = "GREEN";
                break;
            default:
                colorString = "UNKNOWN";
                break;
        }

        // Convert location
        Artifact.Location location;
        switch (slot) {
            case CENTER:
                location = Artifact.Location.CENTER;
                break;
            case FRONT:
                location = Artifact.Location.FRONT_INTAKE;
                break;
            case BACK:
                location = Artifact.Location.BACK_INTAKE;
                break;
            default:
                location = Artifact.Location.UNKNOWN;
                break;
        }

        // Create artifact with sequence ID
        return new Artifact(identity.getSequenceId(), colorString, location, false);
    }

    /**
     * Check if two artifacts match (same sequence ID or color)
     */
    private boolean artifactsMatch(Artifact a, Artifact b) {
        if (a == null || b == null) {
            return false;
        }

        // Match by sequence ID (most reliable)
        if (a.getSequenceId() == b.getSequenceId()) {
            return true;
        }

        // Fallback: match by color if not unknown
        if (!a.getColorString().equals("UNKNOWN") && !b.getColorString().equals("UNKNOWN")) {
            return a.getColorString().equals(b.getColorString());
        }

        return false;
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
