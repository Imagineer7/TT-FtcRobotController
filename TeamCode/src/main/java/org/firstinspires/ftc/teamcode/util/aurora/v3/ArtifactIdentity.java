package org.firstinspires.ftc.teamcode.util.aurora.v3;

/**
 * ArtifactIdentity - Represents the identity and metadata of an artifact in the v3 indexing system
 *
 * This is a data class that holds:
 * - Color classification (GREEN, PURPLE, UNKNOWN)
 * - Color confidence (0.0 to 1.0)
 * - Source of classification (COLOR_SENSOR, INFERRED, OPERATOR)
 * - Debug metadata (sequence ID, timestamp)
 *
 * Instances are immutable to ensure consistent state tracking.
 */
public class ArtifactIdentity {

    // ═══════════════════════════════════════════════════════════════════════
    // ENUMS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Color class of the artifact
     */
    public enum ColorClass {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    /**
     * Source of color classification
     */
    public enum ClassificationSource {
        /** Color determined by color sensor reading */
        COLOR_SENSOR,
        
        /** Color inferred from context (e.g., transfer location) */
        INFERRED,
        
        /** Color manually set by operator */
        OPERATOR,
        
        /** Initial state - no classification yet */
        NONE
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final ColorClass colorClass;
    private final double colorConfidence;  // 0.0 to 1.0
    private final ClassificationSource source;
    private final long timestamp;
    private final int sequenceId;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new ArtifactIdentity
     * 
     * @param colorClass The color classification
     * @param colorConfidence Confidence in classification (0.0 to 1.0)
     * @param source Source of the classification
     * @param sequenceId Unique sequence identifier for tracking
     */
    public ArtifactIdentity(ColorClass colorClass, double colorConfidence, 
                           ClassificationSource source, int sequenceId) {
        this.colorClass = colorClass;
        this.colorConfidence = Math.max(0.0, Math.min(1.0, colorConfidence)); // Clamp 0-1
        this.source = source;
        this.sequenceId = sequenceId;
        this.timestamp = System.currentTimeMillis();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS
    // ═══════════════════════════════════════════════════════════════════════

    public ColorClass getColorClass() { return colorClass; }
    public double getColorConfidence() { return colorConfidence; }
    public ClassificationSource getSource() { return source; }
    public long getTimestamp() { return timestamp; }
    public int getSequenceId() { return sequenceId; }

    // ═══════════════════════════════════════════════════════════════════════
    // FACTORY METHODS (Immutable updates)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a copy with updated color classification
     * Only updates if new confidence is significantly stronger
     * 
     * @param newColor New color classification
     * @param newConfidence New confidence level
     * @param newSource New source of classification
     * @return New ArtifactIdentity with updated color, or this instance if not significantly better
     */
    public ArtifactIdentity withUpdatedColor(ColorClass newColor, double newConfidence, 
                                             ClassificationSource newSource) {
        // Don't update if confidence isn't significantly better (unless OPERATOR override)
        if (newSource != ClassificationSource.OPERATOR) {
            // Require at least 0.15 improvement to change from known color
            if (this.colorClass != ColorClass.UNKNOWN && 
                newConfidence < this.colorConfidence + 0.15) {
                return this;  // Keep existing classification
            }
        }
        
        return new ArtifactIdentity(newColor, newConfidence, newSource, this.sequenceId);
    }

    /**
     * Create a copy with increased confidence (same color)
     * 
     * @param additionalConfidence Amount to increase confidence by
     * @return New ArtifactIdentity with increased confidence
     */
    public ArtifactIdentity withIncreasedConfidence(double additionalConfidence) {
        double newConfidence = Math.min(1.0, this.colorConfidence + additionalConfidence);
        return new ArtifactIdentity(this.colorClass, newConfidence, this.source, this.sequenceId);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATIC FACTORY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create an unknown artifact (initial state)
     */
    public static ArtifactIdentity createUnknown(int sequenceId) {
        return new ArtifactIdentity(ColorClass.UNKNOWN, 0.0, ClassificationSource.NONE, sequenceId);
    }

    /**
     * Create from color sensor reading
     */
    public static ArtifactIdentity createFromSensor(ColorClass color, double confidence, int sequenceId) {
        return new ArtifactIdentity(color, confidence, ClassificationSource.COLOR_SENSOR, sequenceId);
    }

    /**
     * Create from operator input
     */
    public static ArtifactIdentity createFromOperator(ColorClass color, int sequenceId) {
        return new ArtifactIdentity(color, 1.0, ClassificationSource.OPERATOR, sequenceId);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if this artifact has a known color (not UNKNOWN)
     */
    public boolean hasKnownColor() {
        return colorClass != ColorClass.UNKNOWN;
    }

    /**
     * Check if confidence is high enough for reliable planning
     */
    public boolean hasHighConfidence() {
        return colorConfidence >= 0.8;
    }

    /**
     * Check if confidence is medium (usable but not fully reliable)
     */
    public boolean hasMediumConfidence() {
        return colorConfidence >= 0.5 && colorConfidence < 0.8;
    }

    /**
     * Check if confidence is low (unreliable)
     */
    public boolean hasLowConfidence() {
        return colorConfidence < 0.5;
    }

    @Override
    public String toString() {
        return String.format("ArtifactIdentity{seq=%d, color=%s, conf=%.2f, source=%s}",
            sequenceId, colorClass, colorConfidence, source);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        
        ArtifactIdentity that = (ArtifactIdentity) obj;
        return sequenceId == that.sequenceId;  // Artifacts are equal if they have the same sequence ID
    }

    @Override
    public int hashCode() {
        return sequenceId;
    }
}
