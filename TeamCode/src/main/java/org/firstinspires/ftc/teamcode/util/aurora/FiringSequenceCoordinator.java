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
    private static final long FIRING_TIMEOUT_MS = 30000; // 30 seconds

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

        return hasArtifacts && notFiring && shooterReady && indexingReady && noOperation;
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

        firingSequenceActive = true;
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
        if (elapsed > FIRING_TIMEOUT_MS) {
            completeFiring();
        }
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
     */
    public void emergencyStop() {
        firingSequenceActive = false;
        currentShotNumber = 1;
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
