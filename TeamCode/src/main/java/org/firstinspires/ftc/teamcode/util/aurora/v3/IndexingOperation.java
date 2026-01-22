package org.firstinspires.ftc.teamcode.util.aurora.v3;

/**
 * IndexingOperation - Base interface for all indexing operations in v3 system
 *
 * Operations represent atomic, transactional units of work that manipulate
 * the slot ledger. Each operation has a lifecycle:
 * 
 * 1. start() - Begin the operation, start hardware
 * 2. update() - Called every loop, check completion
 * 3. commit() - Apply slot changes when complete
 * 4. cancel() - Abort operation if cancellable
 *
 * Operations are responsible for:
 * - Orchestrating BasicIndexingHelper/BasicFiringHelper calls
 * - Managing operation state and timing
 * - Detecting completion/timeout
 * - Updating slot ledger at commit time only
 */
public interface IndexingOperation {

    /**
     * Start the operation
     * This initiates hardware actions via helpers
     * 
     * @return true if started successfully, false if preconditions failed
     */
    boolean start();

    /**
     * Update the operation (called every loop)
     * Check for completion, timeout, or cancellation
     * 
     * @return true if operation is still in progress, false if complete/failed
     */
    boolean update();

    /**
     * Commit the operation results to the slot ledger
     * This is where slot state changes occur
     * Called automatically when operation completes
     */
    void commit();

    /**
     * Cancel the operation if possible
     * Some operations may not be cancellable once started
     * 
     * @return true if cancelled, false if non-cancellable or already complete
     */
    boolean cancel();

    /**
     * Check if operation is complete (success or failure)
     * 
     * @return true if operation has finished
     */
    boolean isComplete();

    /**
     * Check if operation succeeded
     * Only valid after isComplete() returns true
     * 
     * @return true if operation succeeded, false if failed/cancelled
     */
    boolean isSuccess();

    /**
     * Get operation name for logging/telemetry
     * 
     * @return Human-readable operation name
     */
    String getOperationName();

    /**
     * Get current operation status message
     * 
     * @return Status message for telemetry
     */
    String getStatusMessage();

    /**
     * Get elapsed time since operation started (milliseconds)
     * 
     * @return Elapsed time in ms
     */
    long getElapsedTimeMs();

    /**
     * Check if operation has timed out
     * 
     * @return true if operation exceeded timeout threshold
     */
    boolean isTimedOut();
}
