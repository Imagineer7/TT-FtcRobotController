package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.teamcode.util.debug.DebugLogger;
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

    private final IndexingSystem indexingSystem;
    private final Shooter shooter;
    private final DebugLogger debugLogger;

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
    public FiringSequenceCoordinator(IndexingSystem indexingSystem, Shooter shooter) {
        this.indexingSystem = indexingSystem;
        this.shooter = shooter;
        this.debugLogger = new DebugLogger();
        
        // Register all boolean checks for tracking
        debugLogger.registerCheck("firingSequenceActive", "Firing sequence is active");
        debugLogger.registerCheck("hasArtifacts", "Has artifacts to fire");
        debugLogger.registerCheck("shooterReady", "Shooter is ready to fire");
        debugLogger.registerCheck("indexingReady", "Indexing system ready");
        debugLogger.registerCheck("noOperation", "No operation in progress");
        debugLogger.registerCheck("canStartFiring", "All conditions met to start");
    }
    
    /**
     * Get the debug logger for display
     */
    public DebugLogger getDebugLogger() {
        return debugLogger;
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

        // Update all boolean checks
        debugLogger.updateCheck("hasArtifacts", hasArtifacts, "Count: " + indexingSystem.getArtifactCount());
        debugLogger.updateCheck("shooterReady", shooterReady);
        debugLogger.updateCheck("indexingReady", indexingReady);
        debugLogger.updateCheck("noOperation", noOperation);
        
        boolean canStart = hasArtifacts && notFiring && shooterReady && indexingReady && noOperation;
        debugLogger.updateCheck("canStartFiring", canStart);
        
        // Log the boolean tree
        Map<String, Boolean> conditions = new LinkedHashMap<>();
        conditions.put("hasArtifacts", hasArtifacts);
        conditions.put("notFiring", notFiring);
        conditions.put("shooterReady", shooterReady);
        conditions.put("indexingReady", indexingReady);
        conditions.put("noOperation", noOperation);
        debugLogger.logBooleanTree("canStartFiring", conditions, canStart);
        
        // Legacy System.out logging
        System.out.println("=== canStartFiring Check ===");
        System.out.println("  hasArtifacts: " + hasArtifacts + " (count: " + indexingSystem.getArtifactCount() + ")");
        System.out.println("  notFiring: " + notFiring);
        System.out.println("  shooterReady: " + shooterReady);
        System.out.println("  indexingReady: " + indexingReady + " (state: " + indexingSystem.getCurrentState() + ")");
        System.out.println("  noOperation: " + noOperation);
        System.out.println("  RESULT: " + canStart);

        return canStart;
    }

    /**
     * Start an automated firing sequence
     *
     * @return true if firing sequence started successfully
     */
    public boolean startFiring() {
        debugLogger.info("FIRING", "startFiring() called");
        System.out.println("=== startFiring() called ===");
        
        if (indexingSystem.getArtifactCount() == 0) {
            debugLogger.warning("FIRING", "Cannot start - no artifacts");
            System.out.println("  FAILED: No artifacts");
            return false;
        }

        // CRITICAL: Track the moment firingSequenceActive is set to true
        debugLogger.info("FIRING", "🔥 SETTING firingSequenceActive = TRUE");
        firingSequenceActive = true;
        debugLogger.updateCheck("firingSequenceActive", true, "Set by startFiring()");
        
        firingSequenceStartTime = System.currentTimeMillis();
        currentShotNumber = 1;

        debugLogger.info("FIRING", "Firing sequence started successfully");
        System.out.println("  SUCCESS: Firing sequence started!");
        System.out.println("  firingSequenceActive = true");

        // Ensure shooter is spinning up
        if (!shooter.isAtTargetRPM()) {
            debugLogger.debug("FIRING", "Calling shooter.spinUp()");
            System.out.println("  Calling shooter.spinUp()");
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

        debugLogger.debug("FIRING", "update() - FIRING ACTIVE");
        System.out.println("=== FiringCoordinator.update() - ACTIVE ===");

        // Check if we still have artifacts to fire
        if (indexingSystem.getArtifactCount() == 0) {
            debugLogger.info("FIRING", "No more artifacts - completing");
            System.out.println("  No more artifacts - completing");
            completeFiring();
            return;
        }

        // Wait for shooter to be ready
        if (!shooter.isAtTargetRPM()) {
            debugLogger.debug("FIRING", "Waiting for shooter", 
                String.format("Current: %.0f RPM, Target: %.0f RPM", 
                    shooter.getCurrentRPM(), shooter.getTargetRPM()));
            System.out.println("  Waiting for shooter (current RPM: " + shooter.getCurrentRPM() + " / target: " + shooter.getTargetRPM() + ")");
            return; // Wait for shooter to spin up
        }

        debugLogger.debug("FIRING", "Shooter ready! Checking indexing system");
        System.out.println("  Shooter ready! Checking indexing system...");
        System.out.println("  indexingSystem.isReadyToFire(): " + indexingSystem.isReadyToFire());
        System.out.println("  indexingSystem.isOperationInProgress(): " + indexingSystem.isOperationInProgress());

        // Check if indexing system is ready to fire
        if (indexingSystem.isReadyToFire() && !indexingSystem.isOperationInProgress()) {
            debugLogger.info("FIRING", "🔥 FIRING NOW!");
            System.out.println("  FIRING NOW!");
            // Fire the current shot
            boolean fired = indexingSystem.onFireSignal();
            debugLogger.info("FIRING", "onFireSignal() returned: " + fired);
            System.out.println("  onFireSignal() returned: " + fired);
            if (fired) {
                currentShotNumber++;
            }
        } else {
            debugLogger.debug("FIRING", "Indexing system not ready");
            System.out.println("  Indexing system not ready");
        }

        // Safety timeout
        long elapsed = System.currentTimeMillis() - firingSequenceStartTime;
        if (elapsed > firingTimeoutMs) {
            debugLogger.warning("FIRING", "Timeout reached - completing", 
                String.format("Elapsed: %.1fs", elapsed / 1000.0));
            System.out.println("  TIMEOUT - completing firing");
            completeFiring();
        }
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
        debugLogger.info("FIRING", "🛑 SETTING firingSequenceActive = FALSE");
        firingSequenceActive = false;
        debugLogger.updateCheck("firingSequenceActive", false, "Set by completeFiring()");
        currentShotNumber = 1;
    }

    /**
     * Cancel the firing sequence
     */
    public void cancelFiring() {
        debugLogger.warning("FIRING", "🛑 Firing cancelled");
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
        currentShotNumber = 1;
    }
}
