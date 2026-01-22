package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;

/**
 * KeepAliveWatchdog - Enforces keep-alive shutdown to prevent indefinite spinning
 *
 * **Problem:** In keep-alive mode, shooter stays spinning between shots. If controller doesn't
 * explicitly call cancelFiring(), shooter spins indefinitely (unsafe, wasteful, battery drain).
 *
 * **Solution:** Watchdog automatically detects shutdown conditions and enforces cancelFiring().
 *
 * Shutdown Triggers:
 * 1. **Idle Timeout**: No operation active for > 3 seconds
 * 2. **Trigger Released**: User releases fire button/trigger
 * 3. **Manual Mode Detected**: Manual control override engaged
 * 4. **Explicit Request**: Controller calls shutdown()
 *
 * Usage Pattern (Phase 6 Controller):
 * ```java
 * KeepAliveWatchdog watchdog = new KeepAliveWatchdog(firingHelper, telemetry);
 *
 * // In main update() loop
 * watchdog.update(
 *     triggerPressed: gamepad1.right_trigger > 0.1,
 *     operationActive: operationRunner.isBusy(),
 *     manualModeActive: detectManualMode()
 * );
 *
 * // Watchdog automatically calls firingHelper.cancelFiring() when needed
 * ```
 *
 * Benefits:
 * - **Safety**: Prevents indefinite spinning (overheating, battery drain)
 * - **Automatic**: No controller code needed (just call update())
 * - **Clear Feedback**: Telemetry shows exactly why shutdown occurred
 * - **Testable**: Can verify shutdown logic independently
 *
 * Testing:
 * - Verify idle timeout (3s without activity → shutdown)
 * - Verify trigger release (release button → immediate shutdown)
 * - Verify manual mode (enter manual → immediate shutdown)
 * - Verify normal operation (trigger held + ops running → no shutdown)
 */
public class KeepAliveWatchdog {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final BasicFiringHelper firingHelper;
    private final Telemetry telemetry;

    // Session state
    private boolean sessionActive = false;
    private long lastActivityTime = 0;
    private boolean lastTriggerState = false;
    private String lastShutdownReason = "N/A";

    // Configuration
    private static final long IDLE_TIMEOUT_MS = 3000;  // 3 seconds idle → shutdown
    private boolean enabled = true;

    // Statistics (for debugging/tuning)
    private int totalShutdowns = 0;
    private int idleTimeoutCount = 0;
    private int triggerReleaseCount = 0;
    private int manualModeCount = 0;
    private int explicitShutdownCount = 0;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new KeepAliveWatchdog
     *
     * @param firingHelper BasicFiringHelper to monitor and control
     * @param telemetry Telemetry for warnings and status
     */
    public KeepAliveWatchdog(BasicFiringHelper firingHelper, Telemetry telemetry) {
        this.firingHelper = firingHelper;
        this.telemetry = telemetry;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UPDATE METHOD - CALL EVERY LOOP
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update watchdog - MUST be called every loop
     * Automatically enforces shutdown when conditions met
     *
     * @param triggerPressed Is fire trigger currently pressed?
     * @param operationActive Is a firing operation currently running?
     */
    public void update(boolean triggerPressed, boolean operationActive) {
        update(triggerPressed, operationActive, false);
    }

    /**
     * Update watchdog with manual mode detection
     *
     * @param triggerPressed Is fire trigger currently pressed?
     * @param operationActive Is a firing operation currently running?
     * @param manualModeActive Is manual control mode active?
     */
    public void update(boolean triggerPressed, boolean operationActive, boolean manualModeActive) {
        if (!enabled) return;

        // Check if shooter is actually in keep-alive mode
        boolean shooterInKeepAlive = firingHelper.isReadyForNextShot();

        if (!shooterInKeepAlive) {
            // Shooter not in keep-alive - reset session
            if (sessionActive) {
                sessionActive = false;
                lastShutdownReason = "Session ended (shooter not in keep-alive)";
            }
            return;
        }

        // Shooter is in keep-alive mode
        if (!sessionActive) {
            // Start new session
            sessionActive = true;
            lastActivityTime = System.currentTimeMillis();
            lastTriggerState = triggerPressed;
            return;
        }

        // Update activity timestamp if operation running
        if (operationActive) {
            lastActivityTime = System.currentTimeMillis();
        }

        // Check shutdown conditions
        String shutdownReason = shouldShutdown(triggerPressed, operationActive, manualModeActive);
        
        if (shutdownReason != null) {
            forceShutdown(shutdownReason);
        }

        // Update last trigger state
        lastTriggerState = triggerPressed;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SHUTDOWN LOGIC
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if shutdown is needed
     *
     * @param triggerPressed Current trigger state
     * @param operationActive Current operation state
     * @param manualModeActive Manual mode state
     * @return Shutdown reason if should shutdown, null otherwise
     */
    private String shouldShutdown(boolean triggerPressed, boolean operationActive, boolean manualModeActive) {
        // 1. Manual mode override
        if (manualModeActive) {
            return "Manual mode active (conflict prevention)";
        }

        // 2. Trigger released (falling edge)
        if (lastTriggerState && !triggerPressed) {
            return "Trigger released (user stop)";
        }

        // 3. Idle timeout
        if (!operationActive) {
            long idleTime = System.currentTimeMillis() - lastActivityTime;
            if (idleTime > IDLE_TIMEOUT_MS) {
                return "Idle timeout (" + idleTime + "ms without activity)";
            }
        }

        // No shutdown needed
        return null;
    }

    /**
     * Force shutdown with reason
     *
     * @param reason Human-readable shutdown reason
     */
    private void forceShutdown(String reason) {
        // Cancel firing helper (stops shooter)
        firingHelper.cancelFiring();

        // Record statistics
        totalShutdowns++;
        lastShutdownReason = reason;

        // Count by type
        if (reason.contains("Idle timeout")) {
            idleTimeoutCount++;
        } else if (reason.contains("Trigger released")) {
            triggerReleaseCount++;
        } else if (reason.contains("Manual mode")) {
            manualModeCount++;
        } else {
            explicitShutdownCount++;
        }

        // Log telemetry
        telemetry.addData("⚠️ WATCHDOG SHUTDOWN", reason);
        telemetry.addData("  Total Shutdowns", totalShutdowns);

        // End session
        sessionActive = false;
    }

    /**
     * Manually trigger shutdown
     * Use when controller detects other shutdown conditions
     *
     * @param reason Shutdown reason
     */
    public void shutdown(String reason) {
        if (sessionActive) {
            forceShutdown(reason);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS QUERIES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if watchdog session is active
     *
     * @return true if monitoring a keep-alive session
     */
    public boolean isSessionActive() {
        return sessionActive;
    }

    /**
     * Get time since last activity
     *
     * @return milliseconds since last operation activity
     */
    public long getIdleTime() {
        if (!sessionActive) return 0;
        return System.currentTimeMillis() - lastActivityTime;
    }

    /**
     * Get last shutdown reason
     *
     * @return reason for most recent shutdown
     */
    public String getLastShutdownReason() {
        return lastShutdownReason;
    }

    /**
     * Get total number of shutdowns
     *
     * @return total watchdog shutdowns since creation
     */
    public int getTotalShutdowns() {
        return totalShutdowns;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Enable or disable watchdog
     *
     * @param enabled true to enable, false to disable
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Check if watchdog is enabled
     *
     * @return true if enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Add watchdog status to telemetry
     * Call from controller's addTelemetry() method
     */
    public void addTelemetry() {
        if (!enabled) {
            telemetry.addData("Watchdog", "❌ DISABLED");
            return;
        }

        if (sessionActive) {
            long idleTime = getIdleTime();
            telemetry.addData("🔒 Watchdog", "ACTIVE (idle: " + idleTime + "ms)");
            
            if (idleTime > IDLE_TIMEOUT_MS * 0.8) {
                // Warn when approaching timeout
                long remaining = IDLE_TIMEOUT_MS - idleTime;
                telemetry.addData("  Warning", "Approaching timeout (" + remaining + "ms left)");
            }
        } else {
            telemetry.addData("Watchdog", "Idle (no session)");
        }

        // Statistics
        if (totalShutdowns > 0) {
            telemetry.addData("  Shutdowns", totalShutdowns + 
                            " (idle:" + idleTimeoutCount + 
                            " trigger:" + triggerReleaseCount + 
                            " manual:" + manualModeCount + ")");
            telemetry.addData("  Last Reason", lastShutdownReason);
        }
    }
}
