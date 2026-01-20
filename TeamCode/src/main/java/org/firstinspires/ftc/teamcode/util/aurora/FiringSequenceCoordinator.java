package org.firstinspires.ftc.teamcode.util.aurora;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * FiringSequenceCoordinator - Manages automated firing sequences
 *
 * This class coordinates the IndexingSystem and Shooter to execute
 * automated firing sequences. It handles:
 * - Starting and stopping firing sequences
 * - Coordinating shooter spin-up with indexing system
 * - Managing shot counts and timing
 * - Safety timeouts
 */
public class FiringSequenceCoordinator {

    private final IndexingSystemOld indexingSystem;
    private final Shooter shooter;

    // Firing sequence state
    private boolean firingSequenceActive = false;
    private long firingSequenceStartTime = 0;
    private int currentShotNumber = 1;
    
    // Configuration
    private static final long DEFAULT_FIRING_TIMEOUT_MS = 30000; // 30 seconds
    private long firingTimeoutMs = DEFAULT_FIRING_TIMEOUT_MS;

    /**
     * Create a new FiringSequenceCoordinator
     *
     * @param indexingSystem The indexing system to coordinate
     * @param shooter The shooter to coordinate
     */
    public FiringSequenceCoordinator(IndexingSystemOld indexingSystem, Shooter shooter) {
        this.indexingSystem = indexingSystem;
        this.shooter = shooter;
        
        // Initialize live variables
        initializeLiveVariables();
    }

    /**
     * Initialize live variables with default values
     */
    private void initializeLiveVariables() {
        Map<String, Object> vars = new LinkedHashMap<>();

        vars.put("firingActive", false);
        vars.put("currentShotNumber", 1);
        vars.put("firingElapsedMs", 0L);
        vars.put("firingTimeoutMs", firingTimeoutMs);

        vars.put("shooterEnabled", false);
        vars.put("shooterAtTarget", false);
        vars.put("shooterStable", false);
        vars.put("shooterCurrentRPM", "0");
        vars.put("shooterTargetRPM", "0");

        vars.put("indexingReady", false);
        vars.put("indexingOpInProgress", false);
        vars.put("indexingState", "IDLE");

        SystemMonitor.setAll(vars);
    }

    /**
     * Check if a firing sequence can be started
     *
     * @return true if all conditions are met to start firing
     */
    public boolean canStartFiring() {
        boolean hasArtifacts = indexingSystem.getArtifactCount() > 0;
        boolean notFiring = !firingSequenceActive;
        boolean shooterReady = shooter.isReadyToFire();
        boolean indexingReady = indexingSystem.isReadyToFire();
        boolean noOperation = !indexingSystem.isOperationInProgress();

        // Update all boolean checks with detailed reasons
        
        // Enhanced shooter ready check with detailed breakdown
        // CRITICAL: Use Shooter methods, NOT DecodeHelper directly, to match isReadyToFire() logic
        boolean shooterEnabled = shooter.isEnabled();
        boolean shooterAtTargetRPM = shooter.isAtTargetRPM();
        boolean shooterStabilized = shooter.isRPMStable();
        String stabilizationInfo = shooter.getDecodeHelper().getStabilizationDebugInfo();
        
        // Also query DecodeHelper directly for comparison/debugging
        boolean decodeAtTarget = shooter.getDecodeHelper().isAtTargetRPM();
        boolean decodeStabilized = shooter.getDecodeHelper().isStabilized();
        
        boolean canStart = hasArtifacts && notFiring && shooterReady && indexingReady && noOperation;
        
        // Update SystemMonitor
        SystemMonitor.set("canStartFiring", canStart);
        SystemMonitor.set("shooterReady", shooterReady);
        SystemMonitor.set("shooterAtTargetRPM", shooterAtTargetRPM);
        SystemMonitor.set("shooterStabilized", shooterStabilized);

        return canStart;
    }

    /**
     * Start an automated firing sequence
     *
     * @return true if firing sequence started successfully
     */
    public boolean startFiring() {

        if (indexingSystem.getArtifactCount() == 0) {
            return false;
        }

        // CRITICAL: Track the moment firingSequenceActive is set to true
        firingSequenceActive = true;
        
        // Notify indexing system that firing sequence is active
        indexingSystem.setFiringSequenceActive(true);
        
        firingSequenceStartTime = System.currentTimeMillis();
        currentShotNumber = 1;


        // Ensure shooter is spinning up
        if (!shooter.isAtTargetRPM()) {
            shooter.spinUp();
        }

        return true;
    }

    /**
     * Update the firing sequence (call this in your loop)
     * Handles coordination between shooter and indexing system
     */
    public void update() {
        if (!firingSequenceActive) {
            return;
        }

        // Update live variables
        updateLiveVariables();


        // Check if we still have artifacts to fire
        if (indexingSystem.getArtifactCount() == 0) {
            completeFiring();
            return;
        }

        // Wait for shooter to be ready
        if (!shooter.isAtTargetRPM()) {
            return; // Wait for shooter to spin up
        }

        // Check if indexing system is ready to fire
        if (indexingSystem.isReadyToFire() && !indexingSystem.isOperationInProgress()) {
            // Fire the current shot
            boolean fired = indexingSystem.onFireSignal();
            if (fired) {
                currentShotNumber++;
            }
        }

        // Safety timeout
        long elapsed = System.currentTimeMillis() - firingSequenceStartTime;
        if (elapsed > firingTimeoutMs) {
            SystemMonitor.logNow(String.format("Firing timeout after %.1fs", elapsed / 1000.0));
            completeFiring();
        }
    }

    /**
     * Update live variables for real-time monitoring
     */
    private void updateLiveVariables() {
        Map<String, Object> vars = new LinkedHashMap<>();

        vars.put("firingActive", firingSequenceActive);
        vars.put("currentShotNumber", currentShotNumber);

        long elapsed = firingSequenceActive ? (System.currentTimeMillis() - firingSequenceStartTime) : 0;
        vars.put("firingElapsedMs", elapsed);
        vars.put("firingTimeoutMs", firingTimeoutMs);

        vars.put("shooterEnabled", shooter.isEnabled());
        vars.put("shooterAtTarget", shooter.isAtTargetRPM());
        vars.put("shooterStable", shooter.isRPMStable());
        vars.put("shooterCurrentRPM", String.format("%.0f", shooter.getCurrentRPM()));
        vars.put("shooterTargetRPM", String.format("%.0f", shooter.getTargetRPM()));

        vars.put("indexingReady", indexingSystem.isReadyToFire());
        vars.put("indexingOpInProgress", indexingSystem.isOperationInProgress());
        vars.put("indexingState", indexingSystem.getCurrentState().toString());

        SystemMonitor.setAll(vars);
    }

    /**
     * Set the firing timeout
     *
     * @param timeoutMs Timeout in milliseconds
     */
    public void setFiringTimeout(long timeoutMs) {
        this.firingTimeoutMs = timeoutMs;
    }

    /**
     * Get the current firing timeout
     *
     * @return Timeout in milliseconds
     */
    public long getFiringTimeout() {
        return firingTimeoutMs;
    }

    /**
     * Complete the firing sequence
     */
    public void completeFiring() {
        firingSequenceActive = false;
        
        // Notify indexing system that firing sequence is inactive
        indexingSystem.setFiringSequenceActive(false);
        
        currentShotNumber = 1;
    }

    /**
     * Cancel the firing sequence
     */
    public void cancelFiring() {
        completeFiring();
    }

    /**
     * Check if a firing sequence is currently active
     *
     * @return true if firing sequence is active
     */
    public boolean isFiringActive() {
        return firingSequenceActive;
    }

    /**
     * Get the current shot number in the sequence
     *
     * @return Current shot number
     */
    public int getCurrentShotNumber() {
        return currentShotNumber;
    }

    /**
     * Get elapsed time since firing sequence started
     *
     * @return Elapsed time in milliseconds, or 0 if not firing
     */
    public long getElapsedTime() {
        if (!firingSequenceActive) {
            return 0;
        }
        return System.currentTimeMillis() - firingSequenceStartTime;
    }

    /**
     * Emergency stop - stops firing and resets both systems
     * Uses existing state management methods to maintain consistency
     */
    public void emergencyStop() {
        completeFiring();
        shooter.stop();
        indexingSystem.reset();
    }

    /**
     * Reset the coordinator to initial state
     */
    public void reset() {
        firingSequenceActive = false;
        // Notify indexing system
        indexingSystem.setFiringSequenceActive(false);
        currentShotNumber = 1;
    }
}
