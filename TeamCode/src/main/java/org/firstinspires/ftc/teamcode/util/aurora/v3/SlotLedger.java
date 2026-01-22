package org.firstinspires.ftc.teamcode.util.aurora.v3;

/**
 * SlotLedger - Single source of truth for artifact slot tracking in v3 indexing system
 *
 * This class maintains the authoritative state of which artifacts are in which slots:
 * - CENTER: The center storage slot (ready to fire)
 * - FRONT: The front intake slot (storage)
 * - BACK: The back intake slot (storage)
 *
 * Key principles:
 * - Slots can be null (empty) or contain an ArtifactIdentity
 * - State changes only occur via explicit operations (never sensor-driven)
 * - Provides derived views (count, list, etc.) computed from slots
 * - Thread-safe for single-threaded use (no concurrent access)
 */
public class SlotLedger {

    // ═══════════════════════════════════════════════════════════════════════
    // ENUMS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Slot identifier
     */
    public enum Slot {
        CENTER,
        FRONT,
        BACK
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private ArtifactIdentity centerSlot;
    private ArtifactIdentity frontSlot;
    private ArtifactIdentity backSlot;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new empty SlotLedger
     */
    public SlotLedger() {
        this.centerSlot = null;
        this.frontSlot = null;
        this.backSlot = null;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SLOT ACCESSORS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get artifact in center slot
     * @return ArtifactIdentity or null if empty
     */
    public ArtifactIdentity getCenter() {
        return centerSlot;
    }

    /**
     * Get artifact in front slot
     * @return ArtifactIdentity or null if empty
     */
    public ArtifactIdentity getFront() {
        return frontSlot;
    }

    /**
     * Get artifact in back slot
     * @return ArtifactIdentity or null if empty
     */
    public ArtifactIdentity getBack() {
        return backSlot;
    }

    /**
     * Get artifact in specified slot
     * @param slot The slot to query
     * @return ArtifactIdentity or null if empty
     */
    public ArtifactIdentity get(Slot slot) {
        switch (slot) {
            case CENTER: return centerSlot;
            case FRONT: return frontSlot;
            case BACK: return backSlot;
            default: return null;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SLOT MUTATORS (Package-private - only operations can call)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set artifact in center slot
     * Package-private: only operations can modify slots
     * @param artifact ArtifactIdentity or null to clear
     */
    void setCenter(ArtifactIdentity artifact) {
        this.centerSlot = artifact;
    }

    /**
     * Set artifact in front slot
     * Package-private: only operations can modify slots
     * @param artifact ArtifactIdentity or null to clear
     */
    void setFront(ArtifactIdentity artifact) {
        this.frontSlot = artifact;
    }

    /**
     * Set artifact in back slot
     * Package-private: only operations can modify slots
     * @param artifact ArtifactIdentity or null to clear
     */
    void setBack(ArtifactIdentity artifact) {
        this.backSlot = artifact;
    }

    /**
     * Set artifact in specified slot
     * Package-private: only operations can modify slots
     * @param slot The slot to set
     * @param artifact ArtifactIdentity or null to clear
     */
    void set(Slot slot, ArtifactIdentity artifact) {
        switch (slot) {
            case CENTER: centerSlot = artifact; break;
            case FRONT: frontSlot = artifact; break;
            case BACK: backSlot = artifact; break;
        }
    }

    /**
     * Clear a specific slot
     * Package-private: only operations can modify slots
     * @param slot The slot to clear
     */
    void clear(Slot slot) {
        set(slot, null);
    }

    /**
     * Clear all slots
     * Package-private: only operations can modify slots
     */
    void clearAll() {
        centerSlot = null;
        frontSlot = null;
        backSlot = null;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // DERIVED QUERIES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if center slot is occupied
     */
    public boolean isCenterOccupied() {
        return centerSlot != null;
    }

    /**
     * Check if front slot is occupied
     */
    public boolean isFrontOccupied() {
        return frontSlot != null;
    }

    /**
     * Check if back slot is occupied
     */
    public boolean isBackOccupied() {
        return backSlot != null;
    }

    /**
     * Check if specified slot is occupied
     */
    public boolean isOccupied(Slot slot) {
        return get(slot) != null;
    }

    /**
     * Check if specified slot is empty
     */
    public boolean isEmpty(Slot slot) {
        return get(slot) == null;
    }

    /**
     * Get total count of artifacts (occupied slots)
     */
    public int getArtifactCount() {
        int count = 0;
        if (centerSlot != null) count++;
        if (frontSlot != null) count++;
        if (backSlot != null) count++;
        return count;
    }

    /**
     * Check if system is full (all 3 slots occupied)
     */
    public boolean isFull() {
        return getArtifactCount() == 3;
    }

    /**
     * Check if system is empty (all slots empty)
     */
    public boolean isEmpty() {
        return getArtifactCount() == 0;
    }

    /**
     * Find which slot contains a specific artifact (by sequence ID)
     * @param artifact The artifact to find
     * @return Slot containing the artifact, or null if not found
     */
    public Slot findSlot(ArtifactIdentity artifact) {
        if (artifact == null) return null;
        
        if (centerSlot != null && centerSlot.equals(artifact)) return Slot.CENTER;
        if (frontSlot != null && frontSlot.equals(artifact)) return Slot.FRONT;
        if (backSlot != null && backSlot.equals(artifact)) return Slot.BACK;
        
        return null;
    }

    /**
     * Get first empty slot (for collection)
     * Priority: FRONT → BACK → CENTER (center should rarely be used for collection)
     * @return First empty slot, or null if full
     */
    public Slot getFirstEmptySlot() {
        if (frontSlot == null) return Slot.FRONT;
        if (backSlot == null) return Slot.BACK;
        if (centerSlot == null) return Slot.CENTER;
        return null;
    }

    /**
     * Get the occupied intake slot (FRONT or BACK), if exactly one is occupied
     * @return FRONT or BACK if exactly one intake is occupied, null otherwise
     */
    public Slot getOccupiedIntakeSlot() {
        boolean frontOccupied = frontSlot != null;
        boolean backOccupied = backSlot != null;
        
        if (frontOccupied && !backOccupied) return Slot.FRONT;
        if (backOccupied && !frontOccupied) return Slot.BACK;
        
        return null;  // Either both occupied or both empty
    }

    /**
     * Get the empty intake slot (FRONT or BACK), if exactly one is empty
     * @return FRONT or BACK if exactly one intake is empty, null otherwise
     */
    public Slot getEmptyIntakeSlot() {
        boolean frontEmpty = frontSlot == null;
        boolean backEmpty = backSlot == null;
        
        if (frontEmpty && !backEmpty) return Slot.FRONT;
        if (backEmpty && !frontEmpty) return Slot.BACK;
        
        return null;  // Either both empty or both occupied
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SWAP OPERATIONS (Package-private atomic slot exchanges)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Swap artifacts between two slots atomically
     * Package-private: only operations can modify slots
     * @param slot1 First slot
     * @param slot2 Second slot
     */
    void swap(Slot slot1, Slot slot2) {
        ArtifactIdentity temp = get(slot1);
        set(slot1, get(slot2));
        set(slot2, temp);
    }

    /**
     * Swap center with front
     * Package-private: only operations can modify slots
     */
    void swapCenterWithFront() {
        swap(Slot.CENTER, Slot.FRONT);
    }

    /**
     * Swap center with back
     * Package-private: only operations can modify slots
     */
    void swapCenterWithBack() {
        swap(Slot.CENTER, Slot.BACK);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SNAPSHOT & DEBUG
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a snapshot of current state for logging/telemetry
     */
    public String toSnapshot() {
        StringBuilder sb = new StringBuilder();
        sb.append("SlotLedger[");
        sb.append("CENTER=").append(centerSlot != null ? centerSlot.toString() : "EMPTY");
        sb.append(", FRONT=").append(frontSlot != null ? frontSlot.toString() : "EMPTY");
        sb.append(", BACK=").append(backSlot != null ? backSlot.toString() : "EMPTY");
        sb.append("]");
        return sb.toString();
    }

    @Override
    public String toString() {
        return toSnapshot();
    }
}
