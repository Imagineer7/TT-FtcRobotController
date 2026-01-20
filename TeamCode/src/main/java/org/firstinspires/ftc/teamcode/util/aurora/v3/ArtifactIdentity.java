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

    // Color update policy constants
    private static final double UNKNOWN_UPGRADE_THRESHOLD = 0.6;  // Min confidence to upgrade UNKNOWN
    private static final double SWITCH_MARGIN = 0.20;              // Additional confidence needed to switch colors
    private static final double MIN_SWITCH_CONFIDENCE = 0.75;      // Absolute minimum to switch colors
    
    /**
     * Create a copy with updated color classification
     * 
     * Policy:
     * - OPERATOR override always wins
     * - UNKNOWN → known: requires newConfidence >= UNKNOWN_UPGRADE_THRESHOLD
     * - Known → different known: requires newConfidence >= currentConfidence + SWITCH_MARGIN 
     *                            AND newConfidence >= MIN_SWITCH_CONFIDENCE
     * - Same color: updates confidence upward only
     * 
     * @param newColor New color classification
     * @param newConfidence New confidence level
     * @param newSource New source of classification
     * @return New ArtifactIdentity with updated color, or this instance if update rejected
     */
    public ArtifactIdentity withUpdatedColor(ColorClass newColor, double newConfidence, 
                                             ClassificationSource newSource) {
        // OPERATOR override always wins
        if (newSource == ClassificationSource.OPERATOR) {
            return new ArtifactIdentity(newColor, newConfidence, newSource, this.sequenceId);
        }
        
        // Case 1: Current color is UNKNOWN - allow upgrade if confidence meets threshold
        if (this.colorClass == ColorClass.UNKNOWN) {
            if (newColor != ColorClass.UNKNOWN && newConfidence >= UNKNOWN_UPGRADE_THRESHOLD) {
                return new ArtifactIdentity(newColor, newConfidence, newSource, this.sequenceId);
            }
            // Otherwise stay UNKNOWN (don't downgrade confidence)
            return this;
        }
        
        // Case 2: Same color - update confidence upward only
        if (newColor == this.colorClass) {
            if (newConfidence > this.colorConfidence) {
                return new ArtifactIdentity(newColor, newConfidence, newSource, this.sequenceId);
            }
            return this;  // Don't downgrade confidence
        }
        
        // Case 3: Switching to different color - require high bar
        if (newColor != ColorClass.UNKNOWN && 
            newConfidence >= this.colorConfidence + SWITCH_MARGIN &&
            newConfidence >= MIN_SWITCH_CONFIDENCE) {
            return new ArtifactIdentity(newColor, newConfidence, newSource, this.sequenceId);
        }
        
        // Case 4: Switching to UNKNOWN - never allow (committed color stays)
        return this;
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
