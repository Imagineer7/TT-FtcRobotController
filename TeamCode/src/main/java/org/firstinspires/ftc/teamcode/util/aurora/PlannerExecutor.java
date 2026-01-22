package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;

/**
 * PlannerExecutor - Executor component for physically rearranging artifacts
 *
 * RESPONSIBILITIES:
 * - Executes physical rearrangement only when idle
 * - Ignores new planner requests while busy
 * - Uses operation timeouts from IndexConfig
 * - Manages push operations (artifact swaps between intakes and center)
 * - Updates artifact location states after successful motion
 * - Enforces intake mode rules (STORAGE vs NORMAL mode)
 *
 * PUSH OPERATION STEPS:
 * 1. Un-pre-position center artifact (retract uptake servos)
 * 2. Reverse uptake servos (duration from IndexConfig)
 * 3. Set destination intake to STORAGE mode (low holding speed)
 * 4. Feed stored artifact into center (pushes center artifact to opposite intake)
 * 5. Re-pre-position new center artifact
 * 6. Update artifact location states
 * 7. Enforce timeout using IndexConfig.operationTimeout
 *
 * REARRANGEMENT RULES:
 * - Only allowed when artifact count == 2
 * - Only when executor is idle
 * - Only when manualPushMode == false
 *
 * INTAKE MODE RULES:
 * - STORAGE mode: Intake has an artifact, rollers run slowly to retain
 * - NORMAL mode: Intake is empty, rollers return to normal intake behavior
 * - Empty intakes automatically return to normal intake mode
 */
@Deprecated
public class PlannerExecutor {

    // ═══════════════════════════════════════════════════════════════════════
    // EXECUTOR STATE
    // ═══════════════════════════════════════════════════════════════════════

    public enum ExecutorState {
        IDLE,                       // Not executing any operation
        REARRANGING,                // Executing push/rearrangement operation
        FAILED                      // Operation failed or timed out
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    /** Current executor state */
    private ExecutorState state = ExecutorState.IDLE;

    /** Operation start time for timeout tracking */
    private long operationStartTime = 0;

    /** Pending rearrangement request */
    private Artifact pendingDesiredCenter = null;

    /** Rearrangement lockout flag (set when operation fails) */
    private boolean rearrangementLockout = false;

    /** Last artifact count (used to reset lockout when count changes) */
    private int lastArtifactCount = 0;

    /** Reference to IndexConfig for timeout values */
    private IndexingConfig config;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    public PlannerExecutor(IndexingConfig config) {
        this.config = config;
        this.state = ExecutorState.IDLE;
        this.operationStartTime = 0;
        this.pendingDesiredCenter = null;
        this.rearrangementLockout = false;
        this.lastArtifactCount = 0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Request a rearrangement operation
     * This is called by the planner with the desired center artifact
     *
     * @param desiredCenter The artifact that should be in center position
     * @return true if request accepted, false if ignored (busy or locked out)
     */
    public boolean requestRearrangement(Artifact desiredCenter) {
        // Ignore if busy
        if (state != ExecutorState.IDLE) {
            return false;
        }

        // Ignore if locked out
        if (rearrangementLockout) {
            return false;
        }

        // Accept the request
        pendingDesiredCenter = desiredCenter;
        return true;
    }

    /**
     * Update the executor state
     * This should be called from the main update loop
     *
     * @param artifactCount Current artifact count
     * @return true if a rearrangement operation is in progress
     */
    public boolean update(int artifactCount) {
        long currentTime = System.currentTimeMillis();

        // Reset lockout and FAILED state if artifact count changes
        // This allows the system to recover from previous failures
        if (artifactCount != lastArtifactCount) {
            rearrangementLockout = false;
            lastArtifactCount = artifactCount;
            
            // Auto-recover from FAILED state when artifact count changes
            if (state == ExecutorState.FAILED) {
                Dbg.i(LogGroup.PLANNEREX, "Auto-recovering from FAILED state (artifact count changed to %d)", artifactCount);
                state = ExecutorState.IDLE;
                operationStartTime = 0;
                pendingDesiredCenter = null;
            }
        }

        // Check for timeout if operation in progress
        if (state == ExecutorState.REARRANGING) {
            long elapsed = currentTime - operationStartTime;
            if (elapsed > config.getOperationTimeoutMs()) {
                // Timeout - abort operation
                Dbg.w(LogGroup.PLANNEREX, "Operation timeout - entering FAILED state");
                abortOperation();
                return false;
            }
        }

        return state == ExecutorState.REARRANGING;
    }

    /**
     * Check if executor is idle
     * @return true if executor is not busy
     */
    public boolean isIdle() {
        return state == ExecutorState.IDLE;
    }

    /**
     * Check if executor is busy
     * @return true if executor is executing an operation
     */
    public boolean isBusy() {
        return state == ExecutorState.REARRANGING;
    }

    /**
     * Get current executor state
     * @return Current state
     */
    public ExecutorState getState() {
        return state;
    }

    /**
     * Get pending rearrangement request
     * @return Desired center artifact, or null if no pending request
     */
    public Artifact getPendingDesiredCenter() {
        return pendingDesiredCenter;
    }

    /**
     * Start executing a rearrangement operation
     * This should be called by IndexingSystem after validating the request
     *
     * @return true if operation started successfully
     */
    public boolean startRearrangement() {
        if (state != ExecutorState.IDLE) {
            return false;
        }

        if (pendingDesiredCenter == null) {
            return false;
        }

        // Start the operation
        state = ExecutorState.REARRANGING;
        operationStartTime = System.currentTimeMillis();
        return true;
    }

    /**
     * Complete the current rearrangement operation successfully
     */
    public void completeRearrangement() {
        state = ExecutorState.IDLE;
        operationStartTime = 0;
        pendingDesiredCenter = null;
    }

    /**
     * Abort the current operation (timeout or failure)
     */
    public void abortOperation() {
        state = ExecutorState.FAILED;
        operationStartTime = 0;
        pendingDesiredCenter = null;
        rearrangementLockout = true; // Lock out further auto-rearrangement
    }

    /**
     * Reset executor to idle state (for system reset or manual intervention)
     */
    public void reset() {
        state = ExecutorState.IDLE;
        operationStartTime = 0;
        pendingDesiredCenter = null;
        rearrangementLockout = false;
        lastArtifactCount = 0;
    }

    /**
     * Check if rearrangement is locked out
     * @return true if locked out due to previous failure
     */
    public boolean isRearrangementLockedOut() {
        return rearrangementLockout;
    }

    /**
     * Get elapsed time for current operation
     * @return Elapsed time in milliseconds, or 0 if not executing
     */
    public long getOperationElapsedTime() {
        if (state == ExecutorState.REARRANGING && operationStartTime > 0) {
            return System.currentTimeMillis() - operationStartTime;
        }
        return 0;
    }

    /**
     * Get remaining time before timeout
     * @return Remaining time in milliseconds, or 0 if not executing
     */
    public long getOperationRemainingTime() {
        if (state == ExecutorState.REARRANGING && operationStartTime > 0) {
            long elapsed = System.currentTimeMillis() - operationStartTime;
            long timeout = config.getOperationTimeoutMs();
            return Math.max(0, timeout - elapsed);
        }
        return 0;
    }
}
