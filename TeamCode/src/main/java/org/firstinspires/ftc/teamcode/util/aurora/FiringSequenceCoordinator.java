package org.firstinspires.ftc.teamcode.util.aurora;

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

        // Debug logging to trace the issue
        System.out.println("=== canStartFiring Check ===");
        System.out.println("  hasArtifacts: " + hasArtifacts + " (count: " + indexingSystem.getArtifactCount() + ")");
        System.out.println("  notFiring: " + notFiring);
        System.out.println("  shooterReady: " + shooterReady);
        System.out.println("  indexingReady: " + indexingReady + " (state: " + indexingSystem.getCurrentState() + ")");
        System.out.println("  noOperation: " + noOperation);
        System.out.println("  RESULT: " + (hasArtifacts && notFiring && shooterReady && indexingReady && noOperation));

        return hasArtifacts && notFiring && shooterReady && indexingReady && noOperation;
    }

    /**
     * Start an automated firing sequence
     *
     * @return true if firing sequence started successfully
     */
    public boolean startFiring() {
        System.out.println("=== startFiring() called ===");
        
        if (indexingSystem.getArtifactCount() == 0) {
            System.out.println("  FAILED: No artifacts");
            return false;
        }

        firingSequenceActive = true;
        firingSequenceStartTime = System.currentTimeMillis();
        currentShotNumber = 1;

        System.out.println("  SUCCESS: Firing sequence started!");
        System.out.println("  firingSequenceActive = true");

        // Ensure shooter is spinning up
        if (!shooter.isAtTargetRPM()) {
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

        System.out.println("=== FiringCoordinator.update() - ACTIVE ===");

        // Check if we still have artifacts to fire
        if (indexingSystem.getArtifactCount() == 0) {
            System.out.println("  No more artifacts - completing");
            completeFiring();
            return;
        }

        // Wait for shooter to be ready
        if (!shooter.isAtTargetRPM()) {
            System.out.println("  Waiting for shooter (current RPM: " + shooter.getCurrentRPM() + " / target: " + shooter.getTargetRPM() + ")");
            return; // Wait for shooter to spin up
        }

        System.out.println("  Shooter ready! Checking indexing system...");
        System.out.println("  indexingSystem.isReadyToFire(): " + indexingSystem.isReadyToFire());
        System.out.println("  indexingSystem.isOperationInProgress(): " + indexingSystem.isOperationInProgress());

        // Check if indexing system is ready to fire
        if (indexingSystem.isReadyToFire() && !indexingSystem.isOperationInProgress()) {
            System.out.println("  FIRING NOW!");
            // Fire the current shot
            boolean fired = indexingSystem.onFireSignal();
            System.out.println("  onFireSignal() returned: " + fired);
            if (fired) {
                currentShotNumber++;
            }
        } else {
            System.out.println("  Indexing system not ready");
        }

        // Safety timeout
        long elapsed = System.currentTimeMillis() - firingSequenceStartTime;
        if (elapsed > firingTimeoutMs) {
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
        firingSequenceActive = false;
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
        currentShotNumber = 1;
    }
}
