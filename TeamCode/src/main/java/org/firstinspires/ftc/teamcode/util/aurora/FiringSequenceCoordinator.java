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
        this(indexingSystem, shooter, null);
    }

    /**
     * Create a new FiringSequenceCoordinator with shared debug logger
     *
     * @param indexingSystem The indexing system to coordinate
     * @param shooter The shooter to coordinate
     * @param debugLogger Shared debug logger instance (or null to create new)
     */
    public FiringSequenceCoordinator(IndexingSystem indexingSystem, Shooter shooter, DebugLogger debugLogger) {
        this.indexingSystem = indexingSystem;
        this.shooter = shooter;
        this.debugLogger = debugLogger != null ? debugLogger : new DebugLogger();
        
        // Register all boolean checks for tracking
        debugLogger.registerCheck("firingSequenceActive", "Firing sequence is active");
        debugLogger.registerCheck("hasArtifacts", "Has artifacts to fire");
        debugLogger.registerCheck("shooterReady", "Shooter is ready to fire");
        debugLogger.registerCheck("shooterEnabled", "Shooter is enabled");
        debugLogger.registerCheck("shooterAtTargetRPM", "Shooter at target RPM");
        debugLogger.registerCheck("shooterStabilized", "Shooter RPM stabilized");
        debugLogger.registerCheck("indexingReady", "Indexing system ready");
        debugLogger.registerCheck("noOperation", "No operation in progress");
        debugLogger.registerCheck("canStartFiring", "All conditions met to start");

        // Initialize live variables so they're visible immediately
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

        debugLogger.updateLiveVars(vars);
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

        // Update all boolean checks with detailed reasons
        debugLogger.updateCheck("hasArtifacts", hasArtifacts, "Count: " + indexingSystem.getArtifactCount());
        
        // Enhanced shooter ready check with detailed breakdown
        // CRITICAL: Use Shooter methods, NOT DecodeHelper directly, to match isReadyToFire() logic
        boolean shooterEnabled = shooter.isEnabled();
        boolean shooterAtTargetRPM = shooter.isAtTargetRPM();
        boolean shooterStabilized = shooter.isRPMStable();
        String stabilizationInfo = shooter.getDecodeHelper().getStabilizationDebugInfo();
        
        // Also query DecodeHelper directly for comparison/debugging
        boolean decodeAtTarget = shooter.getDecodeHelper().isAtTargetRPM();
        boolean decodeStabilized = shooter.getDecodeHelper().isStabilized();
        if (shooterAtTargetRPM != decodeAtTarget || shooterStabilized != decodeStabilized) {
            debugLogger.warning("MISMATCH", String.format(
                "Shooter vs DecodeHelper mismatch! Shooter(atTarget=%s, stable=%s) vs Decode(atTarget=%s, stable=%s)",
                shooterAtTargetRPM, shooterStabilized, decodeAtTarget, decodeStabilized));
        }
        
        debugLogger.updateCheck("shooterEnabled", shooterEnabled);
        debugLogger.updateCheck("shooterAtTargetRPM", shooterAtTargetRPM, 
            String.format("current=%.0f, target=%.0f, %s", shooter.getCurrentRPM(), shooter.getTargetRPM(), stabilizationInfo));
        debugLogger.updateCheck("shooterStabilized", shooterStabilized, stabilizationInfo);
        
        if (!shooterReady) {
            String reason = String.format("enabled=%s, atTargetRPM=%s, stabilized=%s, currentRPM=%.0f, targetRPM=%.0f, %s", 
                shooterEnabled, shooterAtTargetRPM, shooterStabilized, shooter.getCurrentRPM(), shooter.getTargetRPM(), stabilizationInfo);
            debugLogger.updateCheck("shooterReady", shooterReady, reason);
        } else {
            debugLogger.updateCheck("shooterReady", shooterReady, "Ready to fire");
        }
        
        debugLogger.updateCheck("indexingReady", indexingReady);
        debugLogger.updateCheck("noOperation", noOperation);
        
        boolean canStart = hasArtifacts && notFiring && shooterReady && indexingReady && noOperation;
        debugLogger.updateCheck("canStartFiring", canStart);
        
        // Log the boolean tree with sub-conditions for shooterReady
        Map<String, Boolean> conditions = new LinkedHashMap<>();
        conditions.put("hasArtifacts", hasArtifacts);
        conditions.put("notFiring", notFiring);
        conditions.put("shooterReady", shooterReady);
        if (!shooterReady) {
            // Expand shooter ready into sub-conditions
            conditions.put("  └─ shooterEnabled", shooterEnabled);
            conditions.put("  └─ shooterAtTargetRPM", shooterAtTargetRPM);
            conditions.put("  └─ shooterStabilized", shooterStabilized);
        }
        conditions.put("indexingReady", indexingReady);
        conditions.put("noOperation", noOperation);
        debugLogger.logBooleanTree("canStartFiring", conditions, canStart);

        return canStart;
    }

    /**
     * Start an automated firing sequence
     *
     * @return true if firing sequence started successfully
     */
    public boolean startFiring() {
        debugLogger.info("FIRING", "startFiring() called");

        if (indexingSystem.getArtifactCount() == 0) {
            debugLogger.warning("FIRING", "Cannot start - no artifacts");
            return false;
        }

        // CRITICAL: Track the moment firingSequenceActive is set to true
        debugLogger.info("FIRING", "🔥 SETTING firingSequenceActive = TRUE");
        firingSequenceActive = true;
        debugLogger.updateCheck("firingSequenceActive", true, "Set by startFiring()");
        
        firingSequenceStartTime = System.currentTimeMillis();
        currentShotNumber = 1;

        debugLogger.info("FIRING", "Firing sequence started successfully");

        // Ensure shooter is spinning up
        if (!shooter.isAtTargetRPM()) {
            debugLogger.debug("FIRING", "Calling shooter.spinUp()");
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

        debugLogger.debug("FIRING", "update() - FIRING ACTIVE");

        // Check if we still have artifacts to fire
        if (indexingSystem.getArtifactCount() == 0) {
            debugLogger.info("FIRING", "No more artifacts - completing");
            completeFiring();
            return;
        }

        // Wait for shooter to be ready
        if (!shooter.isAtTargetRPM()) {
            debugLogger.debug("FIRING", "Waiting for shooter", 
                String.format("Current: %.0f RPM, Target: %.0f RPM", 
                    shooter.getCurrentRPM(), shooter.getTargetRPM()));
            return; // Wait for shooter to spin up
        }

        debugLogger.debug("FIRING", "Shooter ready! Checking indexing system");

        // Check if indexing system is ready to fire
        if (indexingSystem.isReadyToFire() && !indexingSystem.isOperationInProgress()) {
            debugLogger.info("FIRING", "🔥 FIRING NOW!");
            // Fire the current shot
            boolean fired = indexingSystem.onFireSignal();
            debugLogger.info("FIRING", "onFireSignal() returned: " + fired);
            if (fired) {
                currentShotNumber++;
            }
        } else {
            debugLogger.debug("FIRING", "Indexing system not ready");
        }

        // Safety timeout
        long elapsed = System.currentTimeMillis() - firingSequenceStartTime;
        if (elapsed > firingTimeoutMs) {
            debugLogger.warning("FIRING", "Timeout reached - completing", 
                String.format("Elapsed: %.1fs", elapsed / 1000.0));
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

        debugLogger.updateLiveVars(vars);
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
