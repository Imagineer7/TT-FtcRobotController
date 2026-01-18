package org.firstinspires.ftc.teamcode.util.aurora;

import java.util.ArrayList;
import java.util.List;

/**
 * ShotPlanner - Pure logic component for determining shot order and rearrangement strategy
 *
 * RESPONSIBILITIES:
 * - Runs every loop cycle (unless explicitly skipped by rules)
 * - Decides which artifact should be in the center position
 * - Produces a shot plan (ordered list of artifacts to fire)
 * - Validates artifact location states before planning
 * - Does NOT move hardware (hardware control is PlannerExecutor's job)
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
 * 4. If best order's first artifact ≠ current center, output desiredCenterArtifact
 */
public class ShotPlanner {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    /** Current motif pattern (PPG, PGP, or GPP) */
    private String motifPattern = "PPG";

    /** Current shot plan (ordered list of artifacts) */
    private List<Artifact> shotPlan = new ArrayList<>();

    /** Desired center artifact for rearrangement (null if no rearrangement needed) */
    private Artifact desiredCenterArtifact = null;

    /** Manual push mode flag (when true, planner skips rearrangement) */
    private boolean manualPushMode = false;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    public ShotPlanner() {
        this.motifPattern = "PPG";
        this.shotPlan = new ArrayList<>();
        this.desiredCenterArtifact = null;
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
     * @param artifacts List of all artifacts in the system
     * @param artifactInCenter Current artifact in center position (null if empty)
     * @param artifactInFrontIntake Current artifact in front intake (null if empty)
     * @param artifactInBackIntake Current artifact in back intake (null if empty)
     * @return true if planning succeeded, false if skipped
     */
    public boolean updateShotPlan(List<Artifact> artifacts,
                                   Artifact artifactInCenter,
                                   Artifact artifactInFrontIntake,
                                   Artifact artifactInBackIntake) {
        // Clear previous desired center artifact
        desiredCenterArtifact = null;

        // Validate artifact location states
        if (!validateArtifactStates(artifacts, artifactInCenter, artifactInFrontIntake, artifactInBackIntake)) {
            // Invalid state - use default order
            shotPlan = buildDefaultOrder(artifacts);
            return false;
        }

        // Count non-fired artifacts
        int artifactCount = countActiveArtifacts(artifacts);

        // Check skip conditions
        if (shouldSkipPlanning(artifacts, artifactCount)) {
            // Use default order when planning is skipped
            shotPlan = buildDefaultOrder(artifacts);
            return false;
        }

        // Generate feasible orders
        List<List<Artifact>> feasibleOrders = generateFeasibleOrders(
            artifacts, artifactInCenter, artifactInFrontIntake, artifactInBackIntake);

        if (feasibleOrders.isEmpty()) {
            // No feasible orders - use default
            shotPlan = buildDefaultOrder(artifacts);
            return false;
        }

        // Score each order against motif pattern
        int bestScore = -1;
        List<Artifact> bestOrder = null;
        boolean bestRequiresRearrangement = false;

        for (List<Artifact> order : feasibleOrders) {
            int score = scoreOrder(order);
            boolean requiresRearrangement = doesOrderRequireRearrangement(order, artifactInCenter);

            // Select best order (prefer higher score, tie-break by no rearrangement)
            if (score > bestScore || (score == bestScore && !requiresRearrangement && bestRequiresRearrangement)) {
                bestScore = score;
                bestOrder = order;
                bestRequiresRearrangement = requiresRearrangement;
            }
        }

        // Set the shot plan
        if (bestOrder != null) {
            shotPlan = bestOrder;

            // Determine if rearrangement is needed
            if (artifactCount == 2 && bestRequiresRearrangement && !manualPushMode) {
                // Request rearrangement by setting desiredCenterArtifact
                if (!bestOrder.isEmpty() && bestOrder.get(0) != artifactInCenter) {
                    desiredCenterArtifact = bestOrder.get(0);
                }
            }
        } else {
            shotPlan = buildDefaultOrder(artifacts);
        }

        return true;
    }

    /**
     * Get the current shot plan (ordered list of artifacts to fire)
     * @return Mutable list of artifacts in firing order
     */
    public List<Artifact> getShotPlan() {
        return new ArrayList<>(shotPlan);
    }

    /**
     * Get the desired center artifact for rearrangement
     * @return Desired artifact, or null if no rearrangement needed
     */
    public Artifact getDesiredCenterArtifact() {
        return desiredCenterArtifact;
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
     * Validate artifact location states
     */
    private boolean validateArtifactStates(List<Artifact> artifacts,
                                           Artifact artifactInCenter,
                                           Artifact artifactInFrontIntake,
                                           Artifact artifactInBackIntake) {
        // Ensure artifact references are consistent with artifact list
        for (Artifact a : artifacts) {
            if (a.getLocation() == Artifact.Location.CENTER_STORAGE && a != artifactInCenter) {
                return false;
            }
            if (a.getLocation() == Artifact.Location.FRONT_INTAKE && a != artifactInFrontIntake) {
                return false;
            }
            if (a.getLocation() == Artifact.Location.BACK_INTAKE && a != artifactInBackIntake) {
                return false;
            }
        }
        return true;
    }

    /**
     * Count active (non-fired) artifacts
     */
    private int countActiveArtifacts(List<Artifact> artifacts) {
        int count = 0;
        for (Artifact a : artifacts) {
            if (a.getLocation() != Artifact.Location.FIRED) {
                count++;
            }
        }
        return count;
    }

    /**
     * Check if planning should be skipped
     */
    private boolean shouldSkipPlanning(List<Artifact> artifacts, int artifactCount) {
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
        for (Artifact a : artifacts) {
            if (a.getLocation() != Artifact.Location.FIRED && a.getColor() != Artifact.Color.GREEN) {
                allGreen = false;
                break;
            }
        }
        if (allGreen) {
            return true;
        }

        // Skip if exactly 2 artifacts and both are green
        if (artifactCount == 2) {
            int greenCount = 0;
            for (Artifact a : artifacts) {
                if (a.getLocation() != Artifact.Location.FIRED && a.getColor() == Artifact.Color.GREEN) {
                    greenCount++;
                }
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
    private List<List<Artifact>> generateFeasibleOrders(List<Artifact> artifacts,
                                                         Artifact artifactInCenter,
                                                         Artifact artifactInFrontIntake,
                                                         Artifact artifactInBackIntake) {
        List<List<Artifact>> orders = new ArrayList<>();

        // Get active artifacts
        List<Artifact> activeArtifacts = new ArrayList<>();
        for (Artifact a : artifacts) {
            if (a.getLocation() != Artifact.Location.FIRED) {
                activeArtifacts.add(a);
            }
        }

        int count = activeArtifacts.size();

        if (count == 1) {
            // Only one order possible
            orders.add(new ArrayList<>(activeArtifacts));
        } else if (count == 2) {
            // Two orders possible: current order and swapped order
            // Current order: center first, then storage
            List<Artifact> currentOrder = new ArrayList<>();
            if (artifactInCenter != null) {
                currentOrder.add(artifactInCenter);
            }
            if (artifactInFrontIntake != null) {
                currentOrder.add(artifactInFrontIntake);
            } else if (artifactInBackIntake != null) {
                currentOrder.add(artifactInBackIntake);
            }
            orders.add(currentOrder);

            // Swapped order: storage first, then center
            List<Artifact> swappedOrder = new ArrayList<>();
            if (artifactInFrontIntake != null) {
                swappedOrder.add(artifactInFrontIntake);
            } else if (artifactInBackIntake != null) {
                swappedOrder.add(artifactInBackIntake);
            }
            if (artifactInCenter != null) {
                swappedOrder.add(artifactInCenter);
            }
            orders.add(swappedOrder);
        } else if (count == 3) {
            // Only current physical order is possible (no rearrangement)
            // Physical order: center, front intake, back intake
            List<Artifact> currentOrder = new ArrayList<>();
            if (artifactInCenter != null) {
                currentOrder.add(artifactInCenter);
            }
            // Add artifacts in physical position order (front then back)
            if (artifactInFrontIntake != null) {
                currentOrder.add(artifactInFrontIntake);
            }
            if (artifactInBackIntake != null) {
                currentOrder.add(artifactInBackIntake);
            }
            orders.add(currentOrder);
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
    private int scoreOrder(List<Artifact> order) {
        int score = 0;

        // Parse motif pattern into colors
        Artifact.Color[] motifColors = parseMotifPattern(motifPattern);

        // Score each position
        for (int i = 0; i < Math.min(order.size(), motifColors.length); i++) {
            if (order.get(i).getColor() == motifColors[i]) {
                score += (3 - i); // 3 for first, 2 for second, 1 for third
            }
        }

        return score;
    }

    /**
     * Parse motif pattern string into array of colors
     */
    private Artifact.Color[] parseMotifPattern(String pattern) {
        Artifact.Color[] colors = new Artifact.Color[pattern.length()];
        for (int i = 0; i < pattern.length(); i++) {
            char c = pattern.charAt(i);
            if (c == 'P') {
                colors[i] = Artifact.Color.PURPLE;
            } else if (c == 'G') {
                colors[i] = Artifact.Color.GREEN;
            } else {
                colors[i] = Artifact.Color.UNKNOWN;
            }
        }
        return colors;
    }

    /**
     * Check if an order requires rearrangement from current state
     */
    private boolean doesOrderRequireRearrangement(List<Artifact> order, Artifact artifactInCenter) {
        if (order.isEmpty()) {
            return false;
        }
        // Rearrangement is needed if first artifact in order is not currently in center
        return order.get(0) != artifactInCenter;
    }

    /**
     * Build default order when planning is skipped
     * Default order: respects physical positions (center, front intake, back intake)
     * This ensures the physical firing order is used when no rearrangement is possible
     */
    private List<Artifact> buildDefaultOrder(List<Artifact> artifacts) {
        List<Artifact> defaultOrder = new ArrayList<>();

        // Get active artifacts and group by location
        Artifact centerArtifact = null;
        Artifact frontArtifact = null;
        Artifact backArtifact = null;

        for (Artifact a : artifacts) {
            if (a.getLocation() == Artifact.Location.CENTER_STORAGE) {
                centerArtifact = a;
            } else if (a.getLocation() == Artifact.Location.FRONT_INTAKE) {
                frontArtifact = a;
            } else if (a.getLocation() == Artifact.Location.BACK_INTAKE) {
                backArtifact = a;
            }
        }

        // Build order: center first, then front intake, then back intake
        if (centerArtifact != null) {
            defaultOrder.add(centerArtifact);
        }
        if (frontArtifact != null) {
            defaultOrder.add(frontArtifact);
        }
        if (backArtifact != null) {
            defaultOrder.add(backArtifact);
        }

        return defaultOrder;
    }
}
