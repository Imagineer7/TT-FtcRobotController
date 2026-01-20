package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * BaseOperation - Abstract base class for all indexing operations
 *
 * Provides common functionality for operation lifecycle:
 * - State tracking (NOT_STARTED, RUNNING, COMPLETE, FAILED, CANCELLED)
 * - Timeout detection and tracking
 * - Elapsed time calculation
 * - Standard telemetry generation
 * - Automatic commit on success
 *
 * Subclasses must implement:
 * - doStart() - Begin hardware actions
 * - doUpdate() - Check completion (return false when done)
 * - doCommit() - Apply slot changes
 * - getOperationName() - Operation name for logging
 */
public abstract class BaseOperation implements IndexingOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // ENUMS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Operation lifecycle state
     */
    protected enum OperationState {
        NOT_STARTED,  // Operation created but not started
        RUNNING,      // Operation in progress
        COMPLETE,     // Operation completed successfully
        FAILED,       // Operation failed
        CANCELLED     // Operation was cancelled
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    protected final Telemetry telemetry;
    protected final long timeoutMs;

    private OperationState state;
    private long startTime;
    private String statusMessage;
    private RejectReason failureReason;

    // Default timeout
    private static final long DEFAULT_TIMEOUT_MS = 5000;  // 5 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new BaseOperation
     * 
     * @param telemetry Telemetry for logging
     * @param timeoutMs Operation timeout in milliseconds
     */
    protected BaseOperation(Telemetry telemetry, long timeoutMs) {
        this.telemetry = telemetry;
        this.timeoutMs = timeoutMs;
        this.state = OperationState.NOT_STARTED;
        this.startTime = 0;
        this.statusMessage = "Not started";
        this.failureReason = null;
    }

    /**
     * Create a new BaseOperation with default timeout
     * 
     * @param telemetry Telemetry for logging
     */
    protected BaseOperation(Telemetry telemetry) {
        this(telemetry, DEFAULT_TIMEOUT_MS);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API (IndexingOperation interface)
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public final boolean start() {
        if (state != OperationState.NOT_STARTED) {
            statusMessage = "Cannot start - already " + state;
            return false;
        }

        startTime = System.currentTimeMillis();
        state = OperationState.RUNNING;
        statusMessage = "Starting...";

        // Log operation start
        logInfo("Starting " + getOperationName());

        // Call subclass implementation
        boolean started = doStart();
        
        if (!started) {
            state = OperationState.FAILED;
            statusMessage = "Start failed";
            logError("Start failed");
            return false;
        }

        statusMessage = "Running";
        return true;
    }

    @Override
    public final boolean update() {
        if (state != OperationState.RUNNING) {
            return false;  // Not in progress
        }

        // Check timeout
        if (isTimedOut()) {
            state = OperationState.FAILED;
            failureReason = RejectReason.SENSOR_DETECTION_TIMEOUT;  // Generic timeout
            statusMessage = "Timed out after " + getElapsedTimeMs() + "ms";
            logError("Operation timed out");
            return false;
        }

        // Call subclass implementation
        boolean stillRunning = doUpdate();

        if (!stillRunning) {
            // Operation complete
            state = OperationState.COMPLETE;
            statusMessage = "Complete (elapsed: " + getElapsedTimeMs() + "ms)";
            logInfo("Completed successfully");
        }

        return stillRunning;
    }

    @Override
    public final void commit() {
        if (state != OperationState.COMPLETE) {
            logWarn("Commit called but state is " + state + " (expected COMPLETE)");
            return;
        }

        logInfo("Committing slot changes");
        
        // Call subclass implementation
        doCommit();
        
        logInfo("Commit complete");
    }

    @Override
    public final boolean cancel() {
        if (state != OperationState.RUNNING) {
            return false;  // Can only cancel running operations
        }

        // Check if cancellable
        if (!isCancellable()) {
            statusMessage = "Cannot cancel - operation is non-cancellable";
            return false;
        }

        state = OperationState.CANCELLED;
        statusMessage = "Cancelled by user";
        logInfo("Operation cancelled");

        // Call subclass cleanup
        doCancel();

        return true;
    }

    @Override
    public final boolean isComplete() {
        return state == OperationState.COMPLETE || 
               state == OperationState.FAILED || 
               state == OperationState.CANCELLED;
    }

    @Override
    public final boolean isSuccess() {
        return state == OperationState.COMPLETE;
    }

    @Override
    public final String getStatusMessage() {
        return statusMessage;
    }

    @Override
    public final long getElapsedTimeMs() {
        if (startTime == 0) {
            return 0;
        }
        return System.currentTimeMillis() - startTime;
    }

    @Override
    public final boolean isTimedOut() {
        return getElapsedTimeMs() > timeoutMs;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PROTECTED HELPERS (For subclasses)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set status message (visible in telemetry)
     */
    protected void setStatusMessage(String message) {
        this.statusMessage = message;
    }

    /**
     * Mark operation as failed with reason
     */
    protected void fail(RejectReason reason) {
        state = OperationState.FAILED;
        failureReason = reason;
        statusMessage = "Failed: " + reason.getMessage();
        logError("Failed: " + reason);
    }

    /**
     * Get current operation state
     */
    protected OperationState getState() {
        return state;
    }

    /**
     * Get failure reason (null if not failed)
     */
    public RejectReason getFailureReason() {
        return failureReason;
    }

    /**
     * Check if operation is currently running
     */
    protected boolean isRunning() {
        return state == OperationState.RUNNING;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // ABSTRACT METHODS (Subclasses must implement)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Start the operation - begin hardware actions
     * 
     * @return true if started successfully, false if preconditions failed
     */
    protected abstract boolean doStart();

    /**
     * Update the operation - check completion
     * Called every loop while operation is running
     * 
     * @return true if still in progress, false if complete
     */
    protected abstract boolean doUpdate();

    /**
     * Commit the operation results to slot ledger
     * Only called after successful completion
     */
    protected abstract void doCommit();

    /**
     * Get operation name for logging
     * 
     * @return Human-readable operation name
     */
    public abstract String getOperationName();

    // ═══════════════════════════════════════════════════════════════════════
    // OPTIONAL OVERRIDES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if operation can be cancelled
     * Override this if operation is non-cancellable
     * 
     * @return true if cancellable (default), false if not
     */
    protected boolean isCancellable() {
        return true;  // Most operations are cancellable
    }

    /**
     * Cancel cleanup - stop hardware
     * Override this to perform cleanup on cancellation
     */
    protected void doCancel() {
        // Default: no cleanup needed
    }

    // ═══════════════════════════════════════════════════════════════════════
    // LOGGING HELPERS
    // ═══════════════════════════════════════════════════════════════════════

    protected void logInfo(String message) {
        if (telemetry != null) {
            telemetry.addData("[" + getOperationName() + "]", message);
        }
    }

    protected void logWarn(String message) {
        if (telemetry != null) {
            telemetry.addData("⚠️ [" + getOperationName() + "]", message);
        }
    }

    protected void logError(String message) {
        if (telemetry != null) {
            telemetry.addData("❌ [" + getOperationName() + "]", message);
        }
    }

    protected void logDebug(String key, Object value) {
        if (telemetry != null) {
            telemetry.addData("  " + key, value);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Add operation telemetry (call from main loop)
     */
    public void addTelemetry() {
        telemetry.addData("Operation", getOperationName());
        telemetry.addData("State", state);
        telemetry.addData("Status", statusMessage);
        
        if (state == OperationState.RUNNING) {
            long elapsed = getElapsedTimeMs();
            long remaining = timeoutMs - elapsed;
            telemetry.addData("Elapsed", elapsed + "ms / " + timeoutMs + "ms");
            telemetry.addData("Remaining", remaining + "ms");
            
            // Progress bar
            double progress = Math.min(1.0, (double) elapsed / timeoutMs);
            int barLength = 20;
            int filled = (int) (progress * barLength);
            StringBuilder bar = new StringBuilder("[");
            for (int i = 0; i < barLength; i++) {
                bar.append(i < filled ? "=" : " ");
            }
            bar.append("]");
            telemetry.addData("Progress", bar.toString());
        }
        
        if (failureReason != null) {
            telemetry.addData("Failure Reason", failureReason);
        }
    }
}
