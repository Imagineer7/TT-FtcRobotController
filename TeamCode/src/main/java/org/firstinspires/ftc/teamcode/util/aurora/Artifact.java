package org.firstinspires.ftc.teamcode.util.aurora;

/**
 * Artifact - Represents a game artifact (sample/specimen) in the indexing system
 *
 * This class tracks the state of an individual artifact including:
 * - Color/type of the artifact
 * - Current physical location in the robot
 * - Order in which it was collected
 *
 * Artifacts are immutable once created to ensure consistent state tracking.
 */
public class Artifact {

    // ═══════════════════════════════════════════════════════════════════════
    // ENUMS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Color/Type of the artifact
     */
    public enum Color {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    /**
     * Physical location of the artifact in the robot
     */
    public enum Location {
        /** Artifact is in the front intake storage area */
        FRONT_INTAKE,
        
        /** Artifact is in the back intake storage area */
        BACK_INTAKE,
        
        /** Artifact is in the center storage (ready to fire or being transferred) */
        CENTER_STORAGE,
        
        /** Artifact has been fired/ejected */
        FIRED,
        
        /** Location is unknown or not yet determined */
        UNKNOWN
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final Color color;
    private final Location location;
    private final int collectionOrder;
    private final long collectionTimestamp;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new Artifact
     * @param color The color/type of the artifact
     * @param location The current physical location
     * @param collectionOrder The order in which this artifact was collected (1, 2, or 3)
     */
    public Artifact(Color color, Location location, int collectionOrder) {
        this.color = color;
        this.location = location;
        this.collectionOrder = collectionOrder;
        this.collectionTimestamp = System.currentTimeMillis();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS
    // ═══════════════════════════════════════════════════════════════════════

    public Color getColor() { return color; }
    public Location getLocation() { return location; }
    public int getCollectionOrder() { return collectionOrder; }
    public long getCollectionTimestamp() { return collectionTimestamp; }

    // ═══════════════════════════════════════════════════════════════════════
    // FACTORY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a copy of this artifact with a new location
     * @param newLocation The new location for the artifact
     * @return A new Artifact instance with the updated location
     */
    public Artifact withLocation(Location newLocation) {
        return new Artifact(this.color, newLocation, this.collectionOrder);
    }

    /**
     * Create a copy of this artifact with a new color
     * @param newColor The new color for the artifact
     * @return A new Artifact instance with the updated color
     */
    public Artifact withColor(Color newColor) {
        return new Artifact(newColor, this.location, this.collectionOrder);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public String toString() {
        return String.format("Artifact{color=%s, location=%s, order=%d}", 
            color, location, collectionOrder);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        
        Artifact artifact = (Artifact) obj;
        return collectionOrder == artifact.collectionOrder &&
               color == artifact.color &&
               location == artifact.location;
    }

    @Override
    public int hashCode() {
        int result = color != null ? color.hashCode() : 0;
        result = 31 * result + (location != null ? location.hashCode() : 0);
        result = 31 * result + collectionOrder;
        return result;
    }
}
