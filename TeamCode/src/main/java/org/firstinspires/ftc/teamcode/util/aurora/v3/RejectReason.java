package org.firstinspires.ftc.teamcode.util.aurora.v3;

/**
 * RejectReason - Enumeration of reasons why an operation request was rejected
 *
 * This provides structured, telemetry-friendly explanations for why
 * the indexing system refused to execute a requested operation.
 * 
 * Reasons are grouped by category for easier diagnosis:
 * - SYSTEM_* : System state issues (busy, full, disabled, error)
 * - SLOT_* : Slot state issues (occupied, empty)
 * - HARDWARE_* : Hardware readiness issues
 * - PRECONDITION_* : Operation precondition failures
 * - GATING_* : Safety gating failures
 * - PLANNING_* : Shot planning issues
 * - SENSOR_* : Sensor-related issues
 */
public enum RejectReason {
    // ═══════════════════════════════════════════════════════════════════════
    // SYSTEM STATE (System-level issues)
    // ═══════════════════════════════════════════════════════════════════════
    SYSTEM_FULL("System full - max 3 artifacts", RejectCategory.SYSTEM),
    SYSTEM_BUSY("Operation already in progress", RejectCategory.SYSTEM),
    SYSTEM_DISABLED("System is disabled", RejectCategory.SYSTEM),
    SYSTEM_ERROR("System in error state", RejectCategory.SYSTEM),
    
    // ═══════════════════════════════════════════════════════════════════════
    // SLOT STATE (Slot-level issues)
    // ═══════════════════════════════════════════════════════════════════════
    SLOT_OCCUPIED("Target slot is already occupied", RejectCategory.SLOT),
    SLOT_EMPTY("Source slot is empty", RejectCategory.SLOT),
    CENTER_EMPTY("Center slot is empty", RejectCategory.SLOT),
    CENTER_OCCUPIED("Center slot is occupied", RejectCategory.SLOT),
    
    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE STATE (Hardware readiness)
    // ═══════════════════════════════════════════════════════════════════════
    HARDWARE_BUSY("Hardware is busy with another operation", RejectCategory.HARDWARE),
    HARDWARE_NOT_READY("Hardware not initialized", RejectCategory.HARDWARE),
    SHOOTER_NOT_READY("Shooter not at target RPM", RejectCategory.HARDWARE),
    
    // ═══════════════════════════════════════════════════════════════════════
    // PRECONDITIONS (Operation-specific requirements)
    // ═══════════════════════════════════════════════════════════════════════
    PRECONDITION_NOT_PREPOSITIONED("Artifact not pre-positioned for firing", RejectCategory.PRECONDITION),
    PRECONDITION_WRONG_ARTIFACT_COUNT("Invalid artifact count for this operation", RejectCategory.PRECONDITION),
    PRECONDITION_NO_ARTIFACTS("No artifacts available", RejectCategory.PRECONDITION),
    PRECONDITION_MANUAL_MODE_ACTIVE("Manual mode active - auto operations disabled", RejectCategory.PRECONDITION),
    
    // ═══════════════════════════════════════════════════════════════════════
    // SAFETY GATING (Safety rules)
    // ═══════════════════════════════════════════════════════════════════════
    GATING_RULE_FAILED("Safety gating rule not satisfied", RejectCategory.GATING),
    GATING_MANUAL_INPUT_DETECTED("Manual input detected - operation cancelled", RejectCategory.GATING),
    GATING_FIRING_NOT_ACTIVE("Firing sequence not active", RejectCategory.GATING),
    
    // ═══════════════════════════════════════════════════════════════════════
    // PLANNING (Shot planning issues)
    // ═══════════════════════════════════════════════════════════════════════
    PLANNING_NO_REARRANGEMENT_NEEDED("Current order is optimal", RejectCategory.PLANNING),
    PLANNING_REARRANGEMENT_NOT_ALLOWED("Rearrangement not possible with current artifact count", RejectCategory.PLANNING),
    PLANNING_PLANNER_BUSY("Planner/Executor already busy", RejectCategory.PLANNING),
    
    // ═══════════════════════════════════════════════════════════════════════
    // SENSOR (Sensor-related issues - advisory only)
    // ═══════════════════════════════════════════════════════════════════════
    SENSOR_DISAGREEMENT("Sensor readings inconsistent (advisory)", RejectCategory.SENSOR),
    SENSOR_DETECTION_TIMEOUT("Artifact detection timed out", RejectCategory.SENSOR),
    SENSOR_CONFIDENCE_TOO_LOW("Color confidence too low", RejectCategory.SENSOR),
    
    // ═══════════════════════════════════════════════════════════════════════
    // OTHER
    // ═══════════════════════════════════════════════════════════════════════
    INVALID_PARAMETERS("Invalid operation parameters", RejectCategory.OTHER),
    NOT_IMPLEMENTED("Operation not yet implemented", RejectCategory.OTHER),
    UNKNOWN("Unknown rejection reason", RejectCategory.OTHER);

    private final String message;
    private final RejectCategory category;

    RejectReason(String message, RejectCategory category) {
        this.message = message;
        this.category = category;
    }

    public String getMessage() {
        return message;
    }
    
    public RejectCategory getCategory() {
        return category;
    }
    
    public boolean isSystemIssue() {
        return category == RejectCategory.SYSTEM;
    }
    
    public boolean isHardwareIssue() {
        return category == RejectCategory.HARDWARE;
    }
    
    public boolean isSafetyIssue() {
        return category == RejectCategory.GATING;
    }

    @Override
    public String toString() {
        return "[" + category + "] " + name() + ": " + message;
    }
    
    /**
     * Rejection category for grouping
     */
    public enum RejectCategory {
        SYSTEM,       // System state issues
        SLOT,         // Slot availability issues
        HARDWARE,     // Hardware readiness
        PRECONDITION, // Operation preconditions
        GATING,       // Safety gating
        PLANNING,     // Shot planning
        SENSOR,       // Sensor issues (advisory)
        OTHER         // Miscellaneous
    }
}
