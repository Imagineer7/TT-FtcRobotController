package org.firstinspires.ftc.teamcode.util.aurora.v3;

/**
 * RejectReason - Enumeration of reasons why an operation request was rejected
 *
 * This provides structured, telemetry-friendly explanations for why
 * the indexing system refused to execute a requested operation.
 */
public enum RejectReason {
    // System State Rejections
    SYSTEM_FULL("System full - max 3 artifacts"),
    SYSTEM_BUSY("Operation already in progress"),
    SYSTEM_DISABLED("System is disabled"),
    SYSTEM_ERROR("System in error state"),
    
    // Slot State Rejections
    SLOT_OCCUPIED("Target slot is already occupied"),
    SLOT_EMPTY("Source slot is empty"),
    CENTER_EMPTY("Center slot is empty"),
    CENTER_OCCUPIED("Center slot is occupied"),
    
    // Hardware State Rejections
    HARDWARE_BUSY("Hardware is busy with another operation"),
    HARDWARE_NOT_READY("Hardware not initialized"),
    SHOOTER_NOT_READY("Shooter not at target RPM"),
    
    // Operation Precondition Failures
    NOT_PREPOSITIONED("Artifact not pre-positioned for firing"),
    WRONG_ARTIFACT_COUNT("Invalid artifact count for this operation"),
    NO_ARTIFACTS("No artifacts available"),
    MANUAL_MODE_ACTIVE("Manual mode active - auto operations disabled"),
    
    // Safety Gating
    GATING_RULE_FAILED("Safety gating rule not satisfied"),
    MANUAL_INPUT_DETECTED("Manual input detected - operation cancelled"),
    
    // Planning Rejections
    NO_REARRANGEMENT_NEEDED("Current order is optimal"),
    REARRANGEMENT_NOT_ALLOWED("Rearrangement not possible with current artifact count"),
    PLANNER_BUSY("Planner/Executor already busy"),
    
    // Sensor/Detection Rejections
    SENSOR_DISAGREEMENT("Sensor readings inconsistent"),
    DETECTION_TIMEOUT("Artifact detection timed out"),
    CONFIDENCE_TOO_LOW("Color confidence too low"),
    
    // Generic
    INVALID_PARAMETERS("Invalid operation parameters"),
    NOT_IMPLEMENTED("Operation not yet implemented"),
    UNKNOWN("Unknown rejection reason");

    private final String message;

    RejectReason(String message) {
        this.message = message;
    }

    public String getMessage() {
        return message;
    }

    @Override
    public String toString() {
        return name() + ": " + message;
    }
}
