package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.List;

/**
 * IndexingSystem - Push-Based Artifact Indexing System for Aurora Robot
 *
 * This system manages the collection, storage, and firing of up to 3 artifacts using
 * a push-based mechanical indexing mechanism. The system respects physical constraints
 * where center transfer wheels do not contact artifacts directly.
 *
 * INDEXING RULES:
 * 1. First Artifact:
 *    - Goes into center storage
 *    - Cannot be moved unless pushed by the second artifact
 *
 * 2. Second Artifact:
 *    - Enters center storage
 *    - Pushes first artifact into the opposite intake for storage
 *    - Remains in center storage (becomes forced first shot)
 *
 * 3. Third Artifact:
 *    - Stored in the same intake it was collected from
 *    - Does not push any other artifact
 *
 * EARLY FIRE HANDLING:
 * - One artifact: Transfer to center and fire
 * - Two artifacts: Fire second artifact from center, first remains in storage
 *
 * SHOT PLANNING:
 * - First shot is mechanically forced (second artifact collected)
 * - Software plans shots 2 and 3 based on artifact colors and strategy
 */
public class IndexingSystem {

    // ═══════════════════════════════════════════════════════════════════════
    // SYSTEM STATE
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Current state of the indexing system
     */
    public enum SystemState {
        IDLE,                   // No operations in progress
        COLLECTING,             // Actively collecting an artifact
        TRANSFERRING,           // Transferring artifact to center
        PUSHING,                // Pushing artifact to storage
        READY_TO_FIRE,          // Artifact in center, ready to fire
        FIRING,                 // Actively firing an artifact
        ERROR                   // System error detected
    }

    /**
     * Source intake for artifact collection
     */
    public enum IntakeSource {
        FRONT,
        BACK,
        UNKNOWN
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final AuroraHardwareConfig hardware;
    private final IndexingConfig config;
    private final Telemetry telemetry;

    // System state
    private SystemState currentState;
    private long stateStartTime;

    // Artifact memory - tracks all artifacts in the system
    private final List<Artifact> artifacts;
    private int nextCollectionOrder;

    // Current operation tracking
    private Artifact artifactInCenter;
    private Artifact artifactInFrontIntake;
    private Artifact artifactInBackIntake;
    private IntakeSource lastIntakeSource;

    // Timing and operation state
    private long operationStartTime;
    private boolean operationInProgress;

    // Shot planning
    private Artifact plannedSecondShot;
    private Artifact plannedThirdShot;

    // Safety and error tracking
    private String lastError;
    private int errorCount;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new IndexingSystem
     * @param hardware The Aurora hardware configuration
     * @param config The indexing configuration parameters
     * @param telemetry The telemetry system for logging
     */
    public IndexingSystem(AuroraHardwareConfig hardware, IndexingConfig config, Telemetry telemetry) {
        this.hardware = hardware;
        this.config = config;
        this.telemetry = telemetry;

        this.currentState = SystemState.IDLE;
        this.stateStartTime = System.currentTimeMillis();

        this.artifacts = new ArrayList<>();
        this.nextCollectionOrder = 1;

        this.artifactInCenter = null;
        this.artifactInFrontIntake = null;
        this.artifactInBackIntake = null;
        this.lastIntakeSource = IntakeSource.UNKNOWN;

        this.operationStartTime = 0;
        this.operationInProgress = false;

        this.plannedSecondShot = null;
        this.plannedThirdShot = null;

        this.lastError = "";
        this.errorCount = 0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Called when an artifact is detected at an intake
     * @param artifact The detected artifact with color and source information
     * @return true if artifact collection started successfully
     */
    public boolean onArtifactDetected(Artifact artifact, IntakeSource source) {
        // Safety check: can't collect more than 3 artifacts
        if (getArtifactCount() >= IndexingConfig.MAX_ARTIFACTS) {
            setError("Cannot collect artifact: system full (3/3 artifacts)");
            return false;
        }

        // Safety check: can't start new collection while operation in progress
        if (operationInProgress) {
            if (config.isDebugTelemetry()) {
                telemetry.addLine("Cannot collect: operation in progress");
            }
            return false;
        }

        // Update artifact with correct collection order
        Artifact collectionArtifact = new Artifact(
            artifact.getColor(),
            Artifact.Location.UNKNOWN,
            nextCollectionOrder
        );

        lastIntakeSource = source;
        startArtifactCollection(collectionArtifact);
        return true;
    }

    /**
     * Called when fire signal is issued
     * @return true if firing started successfully
     */
    public boolean onFireSignal() {
        // Can only fire if we have artifacts
        if (getArtifactCount() == 0) {
            if (config.isDebugTelemetry()) {
                telemetry.addLine("Cannot fire: no artifacts");
            }
            return false;
        }

        // Can't fire during another operation
        if (operationInProgress && currentState != SystemState.READY_TO_FIRE) {
            if (config.isDebugTelemetry()) {
                telemetry.addLine("Cannot fire: operation in progress");
            }
            return false;
        }

        return handleEarlyFire();
    }

    /**
     * Periodic update method - call this regularly from OpMode loop
     * Handles state machine transitions and ongoing operations
     */
    public void update() {
        long currentTime = System.currentTimeMillis();
        long stateElapsedTime = currentTime - stateStartTime;
        long operationElapsedTime = currentTime - operationStartTime;

        // Check for operation timeout
        if (operationInProgress && operationElapsedTime > config.getOperationTimeoutMs()) {
            setError("Operation timeout in state: " + currentState);
            resetToIdle();
            return;
        }

        // State machine processing
        switch (currentState) {
            case COLLECTING:
                updateCollecting(operationElapsedTime);
                break;

            case TRANSFERRING:
                updateTransferring(operationElapsedTime);
                break;

            case PUSHING:
                updatePushing(operationElapsedTime);
                break;

            case FIRING:
                updateFiring(operationElapsedTime);
                break;

            case READY_TO_FIRE:
                // Waiting for fire signal, nothing to update
                break;

            case IDLE:
                // Nothing to do
                break;

            case ERROR:
                // Try auto-recovery if enabled
                if (config.isEnableAutoRecovery() && stateElapsedTime > 1000) {
                    resetToIdle();
                }
                break;
        }

        // Update telemetry if debug enabled
        if (config.isDebugTelemetry()) {
            updateTelemetry();
        }
    }

    /**
     * Reset the indexing system to initial state
     * Clears all artifact memory and resets counters
     */
    public void reset() {
        artifacts.clear();
        nextCollectionOrder = 1;
        artifactInCenter = null;
        artifactInFrontIntake = null;
        artifactInBackIntake = null;
        lastIntakeSource = IntakeSource.UNKNOWN;
        plannedSecondShot = null;
        plannedThirdShot = null;
        lastError = "";
        errorCount = 0;
        resetToIdle();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CORE INDEXING LOGIC - COLLECTION AND STORAGE
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Start collecting an artifact from an intake
     */
    private void startArtifactCollection(Artifact artifact) {
        changeState(SystemState.COLLECTING);
        operationInProgress = true;
        operationStartTime = System.currentTimeMillis();

        // Add artifact to tracking
        artifacts.add(artifact);

        if (config.isDebugTelemetry()) {
            telemetry.addLine(String.format("Collecting artifact #%d from %s", 
                artifact.getCollectionOrder(), lastIntakeSource));
        }
    }

    /**
     * Update collection state
     */
    private void updateCollecting(long elapsedTime) {
        // Collection completes after intake roller time
        if (elapsedTime >= config.getIntakeRollerTimeMs()) {
            completeCollection();
        }
    }

    /**
     * Complete artifact collection and determine next action
     */
    private void completeCollection() {
        Artifact artifact = artifacts.get(artifacts.size() - 1);

        switch (artifact.getCollectionOrder()) {
            case 1:
                // First artifact: goes to center storage
                startTransferToCenter(artifact);
                break;

            case 2:
                // Second artifact: goes to center, will push first to opposite intake
                startSecondArtifactIndexing(artifact);
                break;

            case 3:
                // Third artifact: stays in same intake it was collected from
                storeThirdArtifact(artifact);
                break;

            default:
                setError("Invalid collection order: " + artifact.getCollectionOrder());
                break;
        }
    }

    /**
     * Transfer artifact to center storage
     */
    private void startTransferToCenter(Artifact artifact) {
        changeState(SystemState.TRANSFERRING);
        operationStartTime = System.currentTimeMillis();

        if (config.isDebugTelemetry()) {
            telemetry.addLine("Transferring artifact to center storage");
        }
    }

    /**
     * Update transferring state
     */
    private void updateTransferring(long elapsedTime) {
        if (elapsedTime >= config.getTransferServoTimeMs() + config.getCenterAcceptTimeMs()) {
            completeTransferToCenter();
        }
    }

    /**
     * Complete transfer to center
     */
    private void completeTransferToCenter() {
        Artifact artifact = artifacts.get(artifacts.size() - 1);
        
        // Update artifact location
        Artifact updatedArtifact = artifact.withLocation(Artifact.Location.CENTER_STORAGE);
        artifacts.set(artifacts.size() - 1, updatedArtifact);
        artifactInCenter = updatedArtifact;

        nextCollectionOrder++;

        if (updatedArtifact.getCollectionOrder() == 1) {
            // First artifact in center, ready for more collection
            changeState(SystemState.IDLE);
            operationInProgress = false;
        } else {
            // Ready to fire
            changeState(SystemState.READY_TO_FIRE);
            operationInProgress = false;
        }
    }

    /**
     * Handle second artifact indexing (pushes first to opposite intake)
     */
    private void startSecondArtifactIndexing(Artifact secondArtifact) {
        if (artifactInCenter == null || artifactInCenter.getCollectionOrder() != 1) {
            setError("Cannot index second artifact: first artifact not in center");
            return;
        }

        changeState(SystemState.PUSHING);
        operationStartTime = System.currentTimeMillis();

        if (config.isDebugTelemetry()) {
            telemetry.addLine("Second artifact pushing first to opposite intake");
        }
    }

    /**
     * Update pushing state
     */
    private void updatePushing(long elapsedTime) {
        long totalPushTime = config.getPushStartDelayMs() + 
                            config.getSecondArtifactPushTimeMs() + 
                            config.getStorageIntakeAcceptTimeMs();

        if (elapsedTime >= totalPushTime) {
            completePushOperation();
        }
    }

    /**
     * Complete push operation
     */
    private void completePushOperation() {
        Artifact firstArtifact = artifactInCenter;
        Artifact secondArtifact = artifacts.get(artifacts.size() - 1);

        // Determine opposite intake from where second artifact came
        Artifact.Location oppositeIntake = (lastIntakeSource == IntakeSource.FRONT) 
            ? Artifact.Location.BACK_INTAKE 
            : Artifact.Location.FRONT_INTAKE;

        // Move first artifact to opposite intake
        Artifact movedFirst = firstArtifact.withLocation(oppositeIntake);
        for (int i = 0; i < artifacts.size(); i++) {
            if (artifacts.get(i).getCollectionOrder() == 1) {
                artifacts.set(i, movedFirst);
                break;
            }
        }

        // Update storage references
        if (oppositeIntake == Artifact.Location.FRONT_INTAKE) {
            artifactInFrontIntake = movedFirst;
        } else {
            artifactInBackIntake = movedFirst;
        }

        // Move second artifact to center
        Artifact secondInCenter = secondArtifact.withLocation(Artifact.Location.CENTER_STORAGE);
        artifacts.set(artifacts.size() - 1, secondInCenter);
        artifactInCenter = secondInCenter;

        nextCollectionOrder++;

        // Plan shots
        planShots();

        changeState(SystemState.READY_TO_FIRE);
        operationInProgress = false;

        if (config.isDebugTelemetry()) {
            telemetry.addLine("Push complete: first->storage, second->center");
        }
    }

    /**
     * Store third artifact in its collection intake
     */
    private void storeThirdArtifact(Artifact thirdArtifact) {
        Artifact.Location storageLocation = (lastIntakeSource == IntakeSource.FRONT)
            ? Artifact.Location.FRONT_INTAKE
            : Artifact.Location.BACK_INTAKE;

        // Check if intake is already occupied
        if ((storageLocation == Artifact.Location.FRONT_INTAKE && artifactInFrontIntake != null) ||
            (storageLocation == Artifact.Location.BACK_INTAKE && artifactInBackIntake != null)) {
            setError("Cannot store third artifact: intake already occupied");
            return;
        }

        Artifact stored = thirdArtifact.withLocation(storageLocation);
        artifacts.set(artifacts.size() - 1, stored);

        if (storageLocation == Artifact.Location.FRONT_INTAKE) {
            artifactInFrontIntake = stored;
        } else {
            artifactInBackIntake = stored;
        }

        nextCollectionOrder++;

        // Plan shots now that we have all three artifacts
        planShots();

        changeState(SystemState.IDLE);
        operationInProgress = false;

        if (config.isDebugTelemetry()) {
            telemetry.addLine(String.format("Third artifact stored in %s", storageLocation));
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIRING LOGIC
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Handle early fire scenarios
     */
    private boolean handleEarlyFire() {
        int count = getArtifactCount();

        if (count == 1) {
            // One artifact: should be in center already or transfer it
            if (artifactInCenter != null) {
                return startFiring();
            } else {
                // This shouldn't happen in normal operation
                setError("Early fire with 1 artifact but none in center");
                return false;
            }
        } else if (count == 2) {
            // Two artifacts: fire the one in center (second collected)
            if (artifactInCenter != null) {
                return startFiring();
            } else {
                setError("Early fire with 2 artifacts but none in center");
                return false;
            }
        } else if (count == 3) {
            // Full robot: fire the one in center
            if (artifactInCenter != null) {
                return startFiring();
            } else {
                setError("Fire signal but no artifact in center");
                return false;
            }
        }

        return false;
    }

    /**
     * Start firing the artifact in center storage
     */
    private boolean startFiring() {
        if (artifactInCenter == null) {
            return false;
        }

        changeState(SystemState.FIRING);
        operationInProgress = true;
        operationStartTime = System.currentTimeMillis();

        if (config.isDebugTelemetry()) {
            telemetry.addLine(String.format("Firing artifact: %s", artifactInCenter));
        }

        return true;
    }

    /**
     * Update firing state
     */
    private void updateFiring(long elapsedTime) {
        if (elapsedTime >= config.getFireFeedTimeMs()) {
            completeFiring();
        }
    }

    /**
     * Complete firing operation
     */
    private void completeFiring() {
        if (artifactInCenter == null) {
            setError("Completed firing but no artifact was in center");
            return;
        }

        // Mark artifact as fired
        Artifact fired = artifactInCenter.withLocation(Artifact.Location.FIRED);
        for (int i = 0; i < artifacts.size(); i++) {
            if (artifacts.get(i).getCollectionOrder() == artifactInCenter.getCollectionOrder()) {
                artifacts.set(i, fired);
                break;
            }
        }

        artifactInCenter = null;

        // After firing, check if we need to move another artifact to center
        int remainingCount = getArtifactCount();
        
        if (remainingCount > 0) {
            // Move next artifact from storage to center for next shot
            moveNextArtifactToCenter();
        } else {
            // All artifacts fired
            changeState(SystemState.IDLE);
            operationInProgress = false;
        }

        if (config.isDebugTelemetry()) {
            telemetry.addLine("Firing complete");
        }
    }

    /**
     * Move the next artifact from storage to center for firing
     */
    private void moveNextArtifactToCenter() {
        // Priority: use planned shot order
        Artifact nextArtifact = null;
        
        if (plannedSecondShot != null && 
            plannedSecondShot.getLocation() != Artifact.Location.FIRED) {
            nextArtifact = plannedSecondShot;
        } else if (plannedThirdShot != null && 
                   plannedThirdShot.getLocation() != Artifact.Location.FIRED) {
            nextArtifact = plannedThirdShot;
        } else {
            // No planned shots, take any available artifact
            for (Artifact a : artifacts) {
                if (a.getLocation() == Artifact.Location.FRONT_INTAKE || 
                    a.getLocation() == Artifact.Location.BACK_INTAKE) {
                    nextArtifact = a;
                    break;
                }
            }
        }

        if (nextArtifact != null) {
            startTransferFromStorageToCenter(nextArtifact);
        } else {
            changeState(SystemState.IDLE);
            operationInProgress = false;
        }
    }

    /**
     * Start transfer from storage intake to center
     */
    private void startTransferFromStorageToCenter(Artifact artifact) {
        changeState(SystemState.TRANSFERRING);
        operationStartTime = System.currentTimeMillis();

        // Clear storage reference
        if (artifact.getLocation() == Artifact.Location.FRONT_INTAKE) {
            artifactInFrontIntake = null;
        } else if (artifact.getLocation() == Artifact.Location.BACK_INTAKE) {
            artifactInBackIntake = null;
        }

        if (config.isDebugTelemetry()) {
            telemetry.addLine(String.format("Transferring artifact from %s to center", 
                artifact.getLocation()));
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SHOT PLANNING LOGIC
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Plan the second and third shots based on artifact colors and strategy
     * First shot is forced by the push-based system (second artifact collected)
     */
    private void planShots() {
        // Get all artifacts that haven't been fired yet
        List<Artifact> available = new ArrayList<>();
        for (Artifact a : artifacts) {
            if (a.getLocation() != Artifact.Location.FIRED) {
                available.add(a);
            }
        }

        if (available.size() < 2) {
            // Not enough artifacts to plan
            return;
        }

        // Simple strategy: plan shots based on collection order
        // Can be enhanced with color-based strategy later
        for (Artifact a : available) {
            if (a.getLocation() == Artifact.Location.CENTER_STORAGE) {
                // This is the forced first shot (already in center)
                continue;
            }
            
            if (plannedSecondShot == null) {
                plannedSecondShot = a;
            } else if (plannedThirdShot == null) {
                plannedThirdShot = a;
            }
        }

        if (config.isDebugTelemetry()) {
            telemetry.addLine("Shot plan updated");
            if (plannedSecondShot != null) {
                telemetry.addLine(String.format("  2nd shot: %s", plannedSecondShot));
            }
            if (plannedThirdShot != null) {
                telemetry.addLine(String.format("  3rd shot: %s", plannedThirdShot));
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATE MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════

    private void changeState(SystemState newState) {
        currentState = newState;
        stateStartTime = System.currentTimeMillis();
    }

    private void resetToIdle() {
        currentState = SystemState.IDLE;
        operationInProgress = false;
        stateStartTime = System.currentTimeMillis();
    }

    private void setError(String error) {
        lastError = error;
        errorCount++;
        currentState = SystemState.ERROR;
        operationInProgress = false;
        
        if (telemetry != null) {
            telemetry.addLine("⚠️ Indexing Error: " + error);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS - SYSTEM STATE
    // ═══════════════════════════════════════════════════════════════════════

    public SystemState getCurrentState() { return currentState; }
    public int getArtifactCount() {
        int count = 0;
        for (Artifact a : artifacts) {
            if (a.getLocation() != Artifact.Location.FIRED) {
                count++;
            }
        }
        return count;
    }

    public Artifact getArtifactInCenter() { return artifactInCenter; }
    public Artifact getArtifactInFrontIntake() { return artifactInFrontIntake; }
    public Artifact getArtifactInBackIntake() { return artifactInBackIntake; }
    
    public List<Artifact> getAllArtifacts() { return new ArrayList<>(artifacts); }
    
    public boolean isReadyToFire() { 
        return currentState == SystemState.READY_TO_FIRE && artifactInCenter != null; 
    }
    
    public boolean isOperationInProgress() { return operationInProgress; }
    
    public String getLastError() { return lastError; }
    public int getErrorCount() { return errorCount; }

    public Artifact getPlannedSecondShot() { return plannedSecondShot; }
    public Artifact getPlannedThirdShot() { return plannedThirdShot; }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("🤖 INDEXING SYSTEM");
        telemetry.addData("State", currentState);
        telemetry.addData("Artifacts", String.format("%d/%d", 
            getArtifactCount(), IndexingConfig.MAX_ARTIFACTS));
        telemetry.addLine("");
        
        telemetry.addData("Center", artifactInCenter != null ? 
            artifactInCenter.getColor() : "Empty");
        telemetry.addData("Front Intake", artifactInFrontIntake != null ? 
            artifactInFrontIntake.getColor() : "Empty");
        telemetry.addData("Back Intake", artifactInBackIntake != null ? 
            artifactInBackIntake.getColor() : "Empty");
        
        if (lastError != null && !lastError.isEmpty()) {
            telemetry.addLine("");
            telemetry.addData("⚠️ Last Error", lastError);
        }
    }

    /**
     * Get a summary string of the current system state
     */
    public String getStatusSummary() {
        return String.format("State: %s | Artifacts: %d/%d | Center: %s", 
            currentState,
            getArtifactCount(),
            IndexingConfig.MAX_ARTIFACTS,
            artifactInCenter != null ? artifactInCenter.getColor().toString() : "Empty"
        );
    }
}
