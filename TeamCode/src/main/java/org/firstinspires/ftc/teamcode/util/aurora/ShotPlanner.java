package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.teamcode.util.aurora.v3.ArtifactIdentity;
import org.firstinspires.ftc.teamcode.util.aurora.v3.SlotLedger;

import java.util.ArrayList;
import java.util.List;

/**
 * ShotPlanner - Pure logic component for determining shot order and rearrangement strategy
 *
 * Updated for v3 system using ArtifactIdentity and SlotLedger
 *
 * RESPONSIBILITIES:
 * - Runs every loop cycle (unless explicitly skipped by rules)
 * - Decides which artifact should be in the center position
 * - Produces a shot plan (ordered list of artifacts to fire)
 * - Validates artifact location states before planning
 * - Does NOT move hardware (coordination handled by ShotPlanningCoordinator)
 *
 * PLANNER SKIP CONDITIONS:
 * - All artifacts are green
 * - Two artifacts and both are green
 * - Artifact count == 1 (can't rearrange)
 * - Artifact count == 3 (can't rearrange)
 * - manualPushMode == true
 *
 * REARRANGEMENT RULES:
 * - 1 artifact: ❌ Never rearrange
 * - 2 artifacts: ✅ Push operation allowed
 * - 3 artifacts: ❌ Never rearrange
 *
 * PATTERN MATCHING STRATEGY:
 * 1. Generate all feasible orders based on artifact count and mechanical constraints
 * 2. Score each order against the motif pattern
 * 3. Select the best scoring order
 * 4. If best order's first artifact ≠ current center, output desiredCenterSlot
 */
public class ShotPlanner {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    /** Current motif pattern (PPG, PGP, or GPP) */
    private String motifPattern = "GPP";

    /** Current shot plan (ordered list of artifacts) */
    private List<ArtifactIdentity> shotPlan = new ArrayList<>();

    /** Desired slot to swap with center for rearrangement (null if no rearrangement needed) */
    private SlotLedger.Slot desiredSwapSlot = null;

    /** Manual push mode flag (when true, planner skips rearrangement) */
    private boolean manualPushMode = false;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    public ShotPlanner() {
        this.motifPattern = "PPG";
        this.shotPlan = new ArrayList<>();
        this.desiredSwapSlot = null;
        this.manualPushMode = false;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set the motif pattern for shot ordering
     * @param pattern One of "PPG", "PGP", or "GPP"
     */
    public void setMotifPattern(String pattern) {
        if (pattern != null && (pattern.equals("PPG") || pattern.equals("PGP") || pattern.equals("GPP"))) {
            this.motifPattern = pattern;
        }
    }

    /**
     * Set manual push mode
     * @param enabled true to enable manual push mode (disables auto-rearrangement)
     */
    public void setManualPushMode(boolean enabled) {
        this.manualPushMode = enabled;
    }

    /**
     * Update the shot plan based on current artifact state
     * This is the main entry point called every loop cycle
     *
     * @param ledger The current slot ledger state
     * @return true if planning succeeded, false if skipped
     */
    public boolean updateShotPlan(SlotLedger ledger) {
        // Clear previous desired swap slot
        desiredSwapSlot = null;

        // Get artifacts from ledger
        ArtifactIdentity centerArtifact = ledger.getCenter();
        ArtifactIdentity frontArtifact = ledger.getFront();
        ArtifactIdentity backArtifact = ledger.getBack();

        // Count non-null artifacts
        int artifactCount = ledger.getArtifactCount();

        // Check skip conditions
        if (shouldSkipPlanning(ledger, artifactCount)) {
            // Use default order when planning is skipped
            shotPlan = buildDefaultOrder(ledger);
            return false;
        }

        // Generate feasible orders
        List<ShotOrder> feasibleOrders = generateFeasibleOrders(ledger);

        if (feasibleOrders.isEmpty()) {
            // No feasible orders - use default
            shotPlan = buildDefaultOrder(ledger);
            return false;
        }

        // Score each order against motif pattern
        int bestScore = -1;
        ShotOrder bestOrder = null;

        for (ShotOrder order : feasibleOrders) {
            int score = scoreOrder(order.artifacts);

            // Select best order (prefer higher score, tie-break by no rearrangement)
            if (score > bestScore || (score == bestScore && !order.requiresSwap && bestOrder != null && bestOrder.requiresSwap)) {
                bestScore = score;
                bestOrder = order;
            }
        }

        // Set the shot plan
        if (bestOrder != null) {
            shotPlan = bestOrder.artifacts;

            // Determine if rearrangement is needed
            if (artifactCount == 2 && bestOrder.requiresSwap && !manualPushMode) {
                // Request rearrangement by setting desiredSwapSlot
                desiredSwapSlot = bestOrder.swapSlot;
            }
        } else {
            shotPlan = buildDefaultOrder(ledger);
        }

        return true;
    }

    /**
     * Get the current shot plan (ordered list of artifacts to fire)
     * @return Mutable list of artifacts in firing order
     */
    public List<ArtifactIdentity> getShotPlan() {
        return new ArrayList<>(shotPlan);
    }

    /**
     * Clear the shot plan (used when resetting system)
     */
    public void clearShotPlan() {
        shotPlan.clear();
        desiredSwapSlot = null;
    }

    /**
     * Get the desired swap slot for rearrangement
     * @return Slot to swap with center (FRONT or BACK), or null if no rearrangement needed
     */
    public SlotLedger.Slot getDesiredSwapSlot() {
        return desiredSwapSlot;
    }

    /**
     * Check if rearrangement is needed
     * @return true if a swap is recommended
     */
    public boolean isRearrangementNeeded() {
        return desiredSwapSlot != null;
    }

    /**
     * Get the current motif pattern
     * @return Motif pattern string (PPG, PGP, or GPP)
     */
    public String getMotifPattern() {
        return motifPattern;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PRIVATE HELPER METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Inner class to represent a potential shot order with metadata
     */
    private static class ShotOrder {
        final List<ArtifactIdentity> artifacts;
        final boolean requiresSwap;
        final SlotLedger.Slot swapSlot;  // Which slot to swap if requiresSwap is true

        ShotOrder(List<ArtifactIdentity> artifacts, boolean requiresSwap, SlotLedger.Slot swapSlot) {
            this.artifacts = artifacts;
            this.requiresSwap = requiresSwap;
            this.swapSlot = swapSlot;
        }
    }

    /**
     * Check if planning should be skipped
     */
    private boolean shouldSkipPlanning(SlotLedger ledger, int artifactCount) {
        // Skip if manual push mode is enabled
        if (manualPushMode) {
            return true;
        }

        // Skip if only 1 artifact (can't rearrange)
        if (artifactCount == 1) {
            return true;
        }

        // Skip if 3 artifacts (can't rearrange)
        if (artifactCount == 3) {
            return true;
        }

        // Skip if all artifacts are green
        boolean allGreen = true;
        ArtifactIdentity center = ledger.getCenter();
        ArtifactIdentity front = ledger.getFront();
        ArtifactIdentity back = ledger.getBack();

        if (center != null && center.getColorClass() != ArtifactIdentity.ColorClass.GREEN) {
            allGreen = false;
        }
        if (front != null && front.getColorClass() != ArtifactIdentity.ColorClass.GREEN) {
            allGreen = false;
        }
        if (back != null && back.getColorClass() != ArtifactIdentity.ColorClass.GREEN) {
            allGreen = false;
        }

        if (allGreen && artifactCount > 0) {
            return true;
        }

        // Skip if exactly 2 artifacts and both are green
        if (artifactCount == 2) {
            int greenCount = 0;
            if (center != null && center.getColorClass() == ArtifactIdentity.ColorClass.GREEN) {
                greenCount++;
            }
            if (front != null && front.getColorClass() == ArtifactIdentity.ColorClass.GREEN) {
                greenCount++;
            }
            if (back != null && back.getColorClass() == ArtifactIdentity.ColorClass.GREEN) {
                greenCount++;
            }
            if (greenCount == 2) {
                return true;
            }
        }

        return false;
    }

    /**
     * Generate all feasible orders based on artifact count and mechanical constraints
     */
    private List<ShotOrder> generateFeasibleOrders(SlotLedger ledger) {
        List<ShotOrder> orders = new ArrayList<>();

        ArtifactIdentity centerArtifact = ledger.getCenter();
        ArtifactIdentity frontArtifact = ledger.getFront();
        ArtifactIdentity backArtifact = ledger.getBack();
        int count = ledger.getArtifactCount();

        if (count == 1) {
            // Only one order possible - no swap needed
            List<ArtifactIdentity> currentOrder = new ArrayList<>();
            if (centerArtifact != null) {
                currentOrder.add(centerArtifact);
            } else if (frontArtifact != null) {
                currentOrder.add(frontArtifact);
            } else if (backArtifact != null) {
                currentOrder.add(backArtifact);
            }
            orders.add(new ShotOrder(currentOrder, false, null));

        } else if (count == 2) {
            // Two orders possible: current order and swapped order

            // Current order: center first, then storage (no swap)
            List<ArtifactIdentity> currentOrder = new ArrayList<>();
            if (centerArtifact != null) {
                currentOrder.add(centerArtifact);
            }
            if (frontArtifact != null) {
                currentOrder.add(frontArtifact);
            } else if (backArtifact != null) {
                currentOrder.add(backArtifact);
            }
            orders.add(new ShotOrder(currentOrder, false, null));

            // Swapped order: storage first, then center (requires swap)
            List<ArtifactIdentity> swappedOrder = new ArrayList<>();
            SlotLedger.Slot swapSlot = null;

            if (frontArtifact != null) {
                swappedOrder.add(frontArtifact);
                swapSlot = SlotLedger.Slot.FRONT;
            } else if (backArtifact != null) {
                swappedOrder.add(backArtifact);
                swapSlot = SlotLedger.Slot.BACK;
            }
            if (centerArtifact != null) {
                swappedOrder.add(centerArtifact);
            }
            orders.add(new ShotOrder(swappedOrder, true, swapSlot));

        } else if (count == 3) {
            // Only current physical order is possible (no rearrangement)
            // Physical order: center, front intake, back intake
            List<ArtifactIdentity> currentOrder = new ArrayList<>();
            if (centerArtifact != null) {
                currentOrder.add(centerArtifact);
            }
            // Add artifacts in physical position order (front then back)
            if (frontArtifact != null) {
                currentOrder.add(frontArtifact);
            }
            if (backArtifact != null) {
                currentOrder.add(backArtifact);
            }
            orders.add(new ShotOrder(currentOrder, false, null));
        }

        return orders;
    }

    /**
     * Score an order against the motif pattern
     * 
     * Scoring algorithm:
     * - First artifact matching motif[0]: +3 points
     * - Second artifact matching motif[1]: +2 points
     * - Third artifact matching motif[2]: +1 point
     */
    private int scoreOrder(List<ArtifactIdentity> order) {
        int score = 0;

        // Parse motif pattern into colors
        ArtifactIdentity.ColorClass[] motifColors = parseMotifPattern(motifPattern);

        // Score each position
        for (int i = 0; i < Math.min(order.size(), motifColors.length); i++) {
            if (order.get(i).getColorClass() == motifColors[i]) {
                score += (3 - i); // 3 for first, 2 for second, 1 for third
            }
        }

        return score;
    }

    /**
     * Parse motif pattern string into array of color classes
     */
    private ArtifactIdentity.ColorClass[] parseMotifPattern(String pattern) {
        ArtifactIdentity.ColorClass[] colors = new ArtifactIdentity.ColorClass[pattern.length()];
        for (int i = 0; i < pattern.length(); i++) {
            char c = pattern.charAt(i);
            if (c == 'P') {
                colors[i] = ArtifactIdentity.ColorClass.PURPLE;
            } else if (c == 'G') {
                colors[i] = ArtifactIdentity.ColorClass.GREEN;
            } else {
                colors[i] = ArtifactIdentity.ColorClass.UNKNOWN;
            }
        }
        return colors;
    }

    /**
     * Build default order when planning is skipped
     * Default order: respects physical positions (center, front intake, back intake)
     * This ensures the physical firing order is used when no rearrangement is possible
     */
    private List<ArtifactIdentity> buildDefaultOrder(SlotLedger ledger) {
        List<ArtifactIdentity> defaultOrder = new ArrayList<>();

        // Build order: center first, then front intake, then back intake
        if (ledger.getCenter() != null) {
            defaultOrder.add(ledger.getCenter());
        }
        if (ledger.getFront() != null) {
            defaultOrder.add(ledger.getFront());
        }
        if (ledger.getBack() != null) {
            defaultOrder.add(ledger.getBack());
        }

        return defaultOrder;
    }
}
