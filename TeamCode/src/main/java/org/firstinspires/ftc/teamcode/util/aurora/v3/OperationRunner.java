package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;

/**
 * OperationRunner - Manages operation lifecycle and enforces single operation at a time
 *
 * This class ensures that:
 * - Only one operation runs at a time (transactional execution)
 * - Operations complete their full lifecycle (start → update* → commit)
 * - Automatic commit on successful completion
 * - Proper cleanup on failure or cancellation
 * - Telemetry tracking of current operation
 *
 * Usage:
 * ```java
 * OperationRunner runner = new OperationRunner(telemetry);
 *
 * // Request operation
 * if (runner.start(new CollectOperation(...))) {
 *     // Started successfully
 * }
 *
 * // In loop
 * runner.update();  // Automatically commits on completion
 *
 * // Check status
 * if (runner.isBusy()) {
 *     // Operation in progress
 * }
 * ```
 */
public class OperationRunner {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final Telemetry telemetry;
    private IndexingOperation currentOperation;
    private long operationStartTime;
    private int operationCount;  // Total operations executed (for tracking)

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new OperationRunner
     * 
     * @param telemetry Telemetry for status updates
     */
    public OperationRunner(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.currentOperation = null;
        this.operationStartTime = 0;
        this.operationCount = 0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OPERATION LIFECYCLE
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Start a new operation
     * 
     * @param operation The operation to start
     * @return true if started successfully, false if rejected (already busy or start failed)
     */
    public boolean start(IndexingOperation operation) {
        if (operation == null) {
            logError("Cannot start null operation");
            Dbg.e(LogGroup.PLANNEREX, "ERROR: Null operation");
            return false;
        }

        // Check if already busy
        if (isBusy()) {
            logWarn("Rejected: " + operation.getOperationName() + 
                   " (already running: " + currentOperation.getOperationName() + ")");
            Dbg.w(LogGroup.PLANNEREX, "REJECTED: Already running %s",
                             currentOperation.getOperationName());
            return false;
        }

        Dbg.d(LogGroup.PLANNEREX, "Starting operation: %s", operation.getOperationName());

        // Try to start operation
        boolean started = operation.start();
        
        if (started) {
            currentOperation = operation;
            operationStartTime = System.currentTimeMillis();
            operationCount++;
            logInfo("Started: " + operation.getOperationName() + " (#" + operationCount + ")");
            Dbg.d(LogGroup.PLANNEREX, "STARTED successfully (#%d)", operationCount);
            return true;
        } else {
            logError("Failed to start: " + operation.getOperationName());
            Dbg.e(LogGroup.PLANNEREX, "FAILED to start: %s", operation.getOperationName());
            return false;
        }
    }

    /**
     * Update current operation
     * Automatically commits on successful completion
     * Call this every loop
     */
    public void update() {
        if (currentOperation == null) {
            return;  // No operation running
        }

        // Update operation
        boolean stillRunning = currentOperation.update();
        Dbg.d(LogGroup.PLANNEREX, "update() - stillRunning=%b, op=%s", stillRunning,
                         currentOperation.getOperationName());

        if (!stillRunning) {
            // Operation complete (success or failure)
            long elapsed = System.currentTimeMillis() - operationStartTime;
            
            if (currentOperation.isSuccess()) {
                // Successful completion - commit slot changes
                Dbg.d(LogGroup.PLANNEREX, "Operation SUCCESSFUL, committing...");
                logInfo("Completed: " + currentOperation.getOperationName() +
                       " (elapsed: " + elapsed + "ms)");
                currentOperation.commit();
                logInfo("Committed slot changes");
                Dbg.d(LogGroup.PLANNEREX, "Commit complete");
            } else {
                // Failed or canceled
                String status = currentOperation.isComplete() ? "Failed" : "Cancelled";
                Dbg.w(LogGroup.PLANNEREX, "Operation %s", status);
                logWarn(status + ": " + currentOperation.getOperationName() +
                       " (" + currentOperation.getStatusMessage() + ")");
                
                // Log failure reason if available
                if (currentOperation instanceof BaseOperation) {
                    BaseOperation baseOp = (BaseOperation) currentOperation;
                    if (baseOp.getFailureReason() != null) {
                        logError("Reason: " + baseOp.getFailureReason());
                    }
                }
            }

            // Clear current operation
            Dbg.d(LogGroup.PLANNEREX, "Clearing operation, now idle");
            currentOperation = null;
            operationStartTime = 0;
        }
    }

    /**
     * Cancel current operation
     * 
     * @return true if cancelled, false if no operation or non-cancellable
     */
    public boolean cancel() {
        if (currentOperation == null) {
            return false;  // No operation to cancel
        }

        boolean cancelled = currentOperation.cancel();
        
        if (cancelled) {
            logInfo("Cancelled: " + currentOperation.getOperationName());
            currentOperation = null;
            operationStartTime = 0;
        } else {
            logWarn("Cannot cancel: " + currentOperation.getOperationName() + 
                   " (non-cancellable or already complete)");
        }

        return cancelled;
    }

    /**
     * Force clear current operation (emergency use only)
     * This does NOT call cancel() - just drops the operation
     * Use only when you need to reset the system
     */
    public void forceClear() {
        if (currentOperation != null) {
            logWarn("Force clearing operation: " + currentOperation.getOperationName());
            currentOperation = null;
            operationStartTime = 0;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS QUERIES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if an operation is currently running
     * 
     * @return true if operation in progress
     */
    public boolean isBusy() {
        return currentOperation != null && !currentOperation.isComplete();
    }

    /**
     * Check if runner is idle (no operation)
     * 
     * @return true if idle
     */
    public boolean isIdle() {
        return currentOperation == null;
    }

    /**
     * Get current operation (can be null)
     * 
     * @return Current operation or null if idle
     */
    public IndexingOperation getCurrentOperation() {
        return currentOperation;
    }

    /**
     * Get current operation name
     * 
     * @return Operation name or "IDLE" if no operation
     */
    public String getCurrentOperationName() {
        if (currentOperation == null) {
            return "IDLE";
        }
        return currentOperation.getOperationName();
    }

    /**
     * Get elapsed time for current operation
     * 
     * @return Elapsed time in ms, or 0 if no operation
     */
    public long getElapsedTimeMs() {
        if (currentOperation == null) {
            return 0;
        }
        return currentOperation.getElapsedTimeMs();
    }

    /**
     * Get total number of operations executed (for tracking)
     * 
     * @return Operation count
     */
    public int getOperationCount() {
        return operationCount;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Add operation runner telemetry
     */
    public void addTelemetry() {
        telemetry.addData("═══ Operation Runner ═══", "");
        telemetry.addData("Status", isBusy() ? "BUSY" : "IDLE");
        telemetry.addData("Operations Executed", operationCount);
        
        if (currentOperation != null) {
            telemetry.addData("Current Operation", currentOperation.getOperationName());
            telemetry.addData("  Status", currentOperation.getStatusMessage());
            telemetry.addData("  Elapsed", currentOperation.getElapsedTimeMs() + "ms");
            
            // Add operation-specific telemetry if available
            if (currentOperation instanceof BaseOperation) {
                ((BaseOperation) currentOperation).addTelemetry();
            }
        } else {
            telemetry.addData("Current Operation", "None");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // LOGGING HELPERS
    // ═══════════════════════════════════════════════════════════════════════

    private void logInfo(String message) {
        if (telemetry != null) {
            telemetry.addData("[OpRunner]", message);
        }
    }

    private void logWarn(String message) {
        if (telemetry != null) {
            telemetry.addData("⚠️ [OpRunner]", message);
        }
    }

    private void logError(String message) {
        if (telemetry != null) {
            telemetry.addData("❌ [OpRunner]", message);
        }
    }
}
