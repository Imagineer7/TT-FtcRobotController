package org.firstinspires.ftc.teamcode.util.aurora.v3;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShotPlanner;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;
import org.firstinspires.ftc.teamcode.util.debug.PerformanceMonitor;

/**
 * IndexingSystemV3 - Main controller for the v3 indexing system.
 * 
 * Orchestrates all operations (collect, transfer, swap, preposition, fire, eject) via OperationRunner.
 * Integrates with ShotPlanningCoordinator for optimal shot sequences.
 * Manages keep-alive burst firing with KeepAliveWatchdog for safety.
 * Provides public API compatible with old IndexingSystem for drop-in replacement.
 * 
 * Key Features:
 * - Slot-based model (CENTER, FRONT, BACK) - single source of truth
 * - Transactional operations (atomic commits on success)
 * - Sensor fusion with checkpoint-based color classification
 * - Shot planning integration with rearrangement
 * - Keep-alive burst firing (2-3x faster)
 * - Automatic safety enforcement (watchdog)
 * - Manual override detection
 * - Rich telemetry and diagnostics
 * 
 * Usage:
 * <pre>
 * IndexingSystemV3 indexing = new IndexingSystemV3(hardware, config, shooter, telemetry);
 * indexing.setMotifPattern("PPG");
 * 
 * // In loop
 * indexing.update(gamepad1, gamepad2);
 * 
 * // Request operations
 * if (gamepad1.a) indexing.requestCollect(SlotLedger.Slot.FRONT);
 * if (gamepad1.x) indexing.requestFire();
 * </pre>
 * 
 * @author FTC 26581 Tundra Tech
 * @version 3.0
 * @since 2026-01-20
 */
public class IndexingSystemV3 {
    
    // ========== Dependencies ==========
    private final AuroraHardwareConfig hardware;
    private final IndexingConfig config;
    private final Shooter shooter;
    private final Telemetry telemetry;
    
    // Core components
    private final SlotLedger ledger;
    private final OperationRunner runner;
    private final ShotPlanningCoordinator shotPlanner;
    private final KeepAliveWatchdog watchdog;
    
    // Helpers
    private final BasicIndexingHelper indexingHelper;
    private final BasicFiringHelper firingHelper;
    
    // Perception (one per intake + center slot)
    private final IntakePerception frontPerception;
    private final IntakePerception backPerception;
    private final IntakePerception.CenterSlotPerception centerPerception;
    
    // Performance monitoring
    private final PerformanceMonitor performanceMonitor;
    
    // ========== State ==========
    private SystemState currentState;
    private boolean enabled;
    private boolean manualModeActive;
    private boolean huntEnabled;  // Hunt mode: auto-collect when artifacts detected
    private boolean skipColorDetection;  // Skip color detection mode: collect as UNKNOWN immediately (default: true)
    
    // Burst firing state
    private boolean burstFiringActive;
    private long lastOperationCompleteTime;
    private int consecutiveShotsFired;
    private int lastKnownShotCount;  // Track FiringHelper's shot count for ledger updates
    private ArtifactIdentity lastFiredArtifact;  // Track last fired artifact for debug telemetry
    private boolean deferredTransferNeeded;  // True when shot fired but transfer deferred due to busy operation
    private boolean firingButtonHeld;  // Track firing button state for watchdog
    
    // Artifact tracking
    private int nextSequenceId;
    
    // Operation tracking
    private IndexingOperation lastCompletedOperation;
    private boolean wasRunnerBusyLastUpdate;
    
    // Statistics
    private int totalCollections;
    private int totalTransfers;
    private int totalSwaps;
    private int totalShots;
    private int totalEjections;
    
    // Telemetry page switching (Issue 1)
    private int telemetryPage = 0;  // 0, 1, or 2
    
    // Auto-collect cooldown (Issue 3)
    private static final long AUTO_COLLECT_COOLDOWN_MS = 1000;  // 1 second
    private long lastFrontAutoCollectTime = 0;
    private long lastBackAutoCollectTime = 0;
    
    // Detection state tracking (prevents duplicate detection)
    private static final long DETECTION_CONFIRMATION_DELAY_MS = 200;  // Wait 200ms to confirm artifact
    private boolean frontArtifactDetected = false;
    private boolean backArtifactDetected = false;
    private long frontDetectionStartTime = 0;
    private long backDetectionStartTime = 0;

    // Auto-swap feature (for skip mode)
    private boolean autoSwapEnabled = true;  // Default ON - auto-swap 2nd artifact in skip mode
    private boolean wasAutoSwapEnabledLastCheck = true;  // Track state to prevent retroactive swaps

    // Transfer cooldown (prevents ghost artifact detection after transfer)
    private static final long TRANSFER_COOLDOWN_MS = 500;  // 500ms cooldown after transfer
    private long lastFrontTransferTime = 0;
    private long lastBackTransferTime = 0;

    // Roller stall detection (storage mode only)
    private static final int STALL_CHECK_COUNT = 3;  // Check 3 times before giving up
    private static final int STALL_MOVEMENT_THRESHOLD = 10;  // Encoder ticks (very small movement = stalled)
    private static final long STALL_CHECK_INTERVAL_MS = 100;  // Check every 100ms
    private boolean frontRollerStalled = false;
    private boolean backRollerStalled = false;
    private int frontStallCheckCount = 0;
    private int backStallCheckCount = 0;
    private long lastFrontStallCheckTime = 0;
    private long lastBackStallCheckTime = 0;
    private int lastFrontRollerPosition = 0;
    private int lastBackRollerPosition = 0;

    /**
     * System states for high-level coordination.
     */
    public enum SystemState {
        IDLE,                  // No artifacts, no operations
        COLLECTING,            // Collecting artifact into intake
        TRANSFERRING,          // Transferring artifact to center
        READY_TO_FIRE,        // Center occupied, shooter ready, prepositioned
        FIRING,                // Firing shot
        REARRANGING,          // Swapping artifacts for optimal order
        EJECTING,             // Clearing artifacts
        ERROR                 // System error state
    }
    
    /**
     * Constructor - initializes all components.
     * 
     * @param hardware Hardware configuration
     * @param config Indexing configuration
     * @param shooter Shooter subsystem
     * @param telemetry Telemetry for display
     */
    public IndexingSystemV3(AuroraHardwareConfig hardware, IndexingConfig config, 
                            Shooter shooter, Telemetry telemetry) {
        this.hardware = hardware;
        this.config = config;
        this.shooter = shooter;
        this.telemetry = telemetry;
        
        // Initialize core components
        this.ledger = new SlotLedger();
        this.runner = new OperationRunner(telemetry);
        
        // Initialize helpers (new API - no config, no enable/disable)
        this.indexingHelper = new BasicIndexingHelper(hardware, telemetry);
        this.firingHelper = new BasicFiringHelper(shooter, indexingHelper, hardware, telemetry);
        
        // Initialize shot planning
        ShotPlanner planner = new ShotPlanner();
        this.shotPlanner = new ShotPlanningCoordinator(planner, telemetry);
        
        // Initialize watchdog
        this.watchdog = new KeepAliveWatchdog(firingHelper, telemetry);
        
        // Initialize performance monitoring (disabled by default)
        this.performanceMonitor = new PerformanceMonitor(telemetry);
        
        // Initialize perception for intakes (IntakeSide enum, sensors, config)
        // New sensor layout:
        // - goBILDA distance sensor (confirmation)
        // - REV Color V3 sensors (primary detection via color + proximity)
        this.frontPerception = new IntakePerception(
            IntakePerception.IntakeSide.FRONT,
            hardware.getFrontDistanceSensor(),          // goBILDA confirmation
            hardware.getFrontIntakeColorPrimary(),      // Primary REV Color V3
            hardware.getFrontIntakeColorSecondary(),    // Secondary REV Color V3
            config
        );
        
        this.backPerception = new IntakePerception(
            IntakePerception.IntakeSide.BACK,
            hardware.getBackDistanceSensor(),           // goBILDA confirmation
            hardware.getBackIntakeColorPrimary(),       // Primary REV Color V3
            hardware.getBackIntakeColorSecondary(),     // Secondary REV Color V3
            config
        );
        
        // Initialize center slot perception
        this.centerPerception = new IntakePerception.CenterSlotPerception(
            hardware.getCenterDistanceSensor(),         // goBILDA center distance
            hardware.getCenterColorLeft(),              // Left REV Color V3
            hardware.getCenterColorRight(),             // Right REV Color V3
            config
        );
        
        // Initialize state
        this.currentState = SystemState.IDLE;
        this.enabled = false;
        this.manualModeActive = false;
        this.huntEnabled = true;  // Hunt mode ON by default
        this.skipColorDetection = true;  // Skip color detection ON by default (fast collection mode)
        this.burstFiringActive = false;
        this.deferredTransferNeeded = false;  // Initialize deferred transfer flag
        this.lastOperationCompleteTime = System.currentTimeMillis();
        this.consecutiveShotsFired = 0;
        this.lastKnownShotCount = 0;  // Initialize shot count tracker
        this.lastFiredArtifact = null;  // No artifact fired yet
        this.nextSequenceId = 1;
        this.lastCompletedOperation = null;
        this.wasRunnerBusyLastUpdate = false;
        
        telemetry.addData("IndexingSystemV3", "Initialized");
        telemetry.update();
    }
    
    /**
     * Enable the system.
     */
    public void enable() {
        enabled = true;
        telemetry.addData("IndexingSystemV3", "Enabled");
    }
    
    /**
     * Disable the system.
     */
    public void disable() {
        enabled = false;
        // Cancel any running operation
        if (runner.isBusy()) {
            runner.cancel();
        }
        // Stop shooter if spinning
        if (firingHelper.isFiring() || firingHelper.isReadyForNextShot()) {
            firingHelper.cancelFiring();
        }
        burstFiringActive = false;
        telemetry.addData("IndexingSystemV3", "Disabled");
    }
    
    /**
     * Main update loop - MUST be called every iteration.
     * 
     * OpModes should call this method in their loop and handle gamepad inputs separately.
     * 
     * ⚠️ CRITICAL: This method calls firingHelper.update() which internally calls shooter.update()!
     * DO NOT call shooter.update() separately in your OpMode loop!
     * Calling shooter.update() twice per loop will cause the shooter to pulse on/off.
     * 
     * CORRECT OpMode pattern:
     *   while (opModeIsActive()) {
     *       indexing.update();  // ← shooter.update() called here internally via firingHelper
     *       // ... rest of code
     *   }
     * 
     * WRONG OpMode pattern (DO NOT DO THIS):
     *   while (opModeIsActive()) {
     *       indexing.update();  // Calls firingHelper.update() → shooter.update()
     *       shooter.update();   // ❌ WRONG - duplicate call causes pulsing!
     *   }
     */
    public void update() {
        if (!enabled) return;
        
        // Start loop timing
        performanceMonitor.startLoop();
        
        // Update perception (sensor fusion) - only for hunt-eligible intakes
        performanceMonitor.startSection("perception");
        updatePerception();
        performanceMonitor.endSection("perception");
        
        // CRITICAL: Update indexing helper to process timed movements
        // This clears the busy flags when timed movements complete
        performanceMonitor.startSection("indexingHelper");
        indexingHelper.update();
        performanceMonitor.endSection("indexingHelper");
        
        // CRITICAL: Update firing helper to process firing sequences
        // This internally calls shooter.update() - DO NOT call shooter.update() separately!
        performanceMonitor.startSection("firingHelper");
        firingHelper.update();
        performanceMonitor.endSection("firingHelper");
        
        // Update watchdog (automatic safety enforcement)
        // Note: OpMode must call setFiringButtonHeld() to update trigger state
        // CRITICAL: Pass isOperationRunning() which includes physical hardware state,
        // not just runner.isBusy() which only checks the operation state machine
        performanceMonitor.startSection("watchdog");
        watchdog.update(firingButtonHeld, isOperationRunning(), manualModeActive);
        performanceMonitor.endSection("watchdog");
        
        // Capture current operation before update (for completion handling)
        boolean isBusyNow = runner.isBusy();
        if (isBusyNow) {
            lastCompletedOperation = runner.getCurrentOperation();
        }
        
        // Update operation runner (automatic lifecycle management)
        performanceMonitor.startSection("operations");
        runner.update();
        performanceMonitor.endSection("operations");
        
        // Handle operation completion (detect transition from busy to idle)
        if (wasRunnerBusyLastUpdate && !runner.isBusy()) {
            handleOperationComplete();
        }
        wasRunnerBusyLastUpdate = runner.isBusy();
        
        // CRITICAL: Check for subsequent shots AFTER operations complete and ledger updates
        // This ensures transfers have committed their ledger changes before we process shots
        // Otherwise we clear CENTER before the transfer has even set it!
        checkForSubsequentShotFired();
        
        // Update shot planner
        performanceMonitor.startSection("shotPlanner");
        shotPlanner.update(ledger, manualModeActive);
        performanceMonitor.endSection("shotPlanner");
        
        // Track auto-swap state to prevent retroactive swaps when button is released
        // Only update tracking when button state changes to prevent swaps on already-collected artifacts
        wasAutoSwapEnabledLastCheck = autoSwapEnabled;

        // Update system state
        updateSystemState();
        
        // Automatic operations (if not in manual mode)
        if (!manualModeActive && !runner.isBusy()) {
            performanceMonitor.startSection("autoOperations");
            performAutomaticOperations();
            performanceMonitor.endSection("autoOperations");
        }
        
        // End loop timing
        performanceMonitor.endLoop();
    }
    
    // Manual override detection removed - OpModes handle gamepad inputs directly
    
    /**
     * Update perception sensors based on hunt mode eligibility.
     * 
     * Hunt Mode ON: Update perception for empty intakes (eligible to hunt).
     * Hunt Mode OFF: Skip perception updates for empty intakes (sleep mode).
     * 
     * Note: Perception is always skipped during operations (rollers controlled by operations).
     */
    private void updatePerception() {
        // CRITICAL FIX: Always update perception for color sampling during operations
        // Operations need perception to read color sensors at checkpoints
        // Even when operations are running, perception must continue to update
        // so that color sampling (when enabled) can collect fresh sensor readings
        
        // Always update perception - needed for color sampling during operations
        frontPerception.update();
        backPerception.update();
        centerPerception.update();  // Update center slot perception
        
        // During operations: skip hunt-mode hardware control (operation has control)
        if (runner.isBusy()) {
            // Stop hunt-mode hardware when operations take control
            stopHuntingTransferServos();
            stopHuntingRollers();
            return;  // Skip hunt-mode updates below
        }
        
        // Hunt mode: Run intake rollers for hunt-eligible intakes
        // This allows artifacts to be pulled in during hunt mode
        updateHuntingRollers();
        
        // Check for roller stalls in storage mode (non-blocking)
        checkRollerStalls(System.currentTimeMillis());

        // Hunt mode: Run transfer servos in reverse for hunt-eligible intakes
        // This creates a "jiggling" effect that rotates artifacts slightly
        // Helps prevent sensor blind spots from holes in artifacts
        updateHuntingTransferServos();
    }
    
    /**
     * Check if a subsequent shot (non-FireOperation) has fired and update ledger.
     * 
     * When fireNextShot() is called, it fires directly via FiringHelper without creating
     * a FireOperation. This avoids race conditions where FireOperation's doCommit() clears
     * the CENTER slot before the artifact is even loaded.
     * 
     * Instead, we track FiringHelper's shot count and update the ledger when we detect
     * a new shot has actually fired.
     * 
     * CRITICAL: Shot count increments IMMEDIATELY after 300ms uptake feeding completes.
     * At that point, the artifact has been pushed through uptake into the shooter flywheel.
     * We must clear the ledger IMMEDIATELY (artifact is physically gone from CENTER).
     * 
     * However, we queue the next transfer only when NO operations are running to avoid
     * conflicts with ongoing transfers or collections.
     */
    private void checkForSubsequentShotFired() {
        if (!burstFiringActive) {
            return;  // Not in burst mode, nothing to check
        }
        
        // Check if shot count increased
        int currentShotCount = firingHelper.getShotsFiredCount();
        if (currentShotCount > lastKnownShotCount) {
            // Shot fired! Get the artifact before clearing CENTER
            ArtifactIdentity firedArtifact = ledger.getCenter();
            
            Dbg.d(LogGroup.FIRING, "Subsequent shot detected (%d → %d), clearing CENTER",
                             lastKnownShotCount, currentShotCount);

            // CRITICAL: Clear center slot IMMEDIATELY when shot fires
            // The artifact has been physically pushed into the shooter, it's no longer in CENTER
            ledger.setCenter(null);
            
            // Update tracking
            lastKnownShotCount = currentShotCount;
            lastFiredArtifact = firedArtifact;  // Track for telemetry
            consecutiveShotsFired++;
            totalShots++;
            
            // Consume shot from plan
            if (shotPlanner != null) {
                shotPlanner.consumeShot();
                Dbg.d(LogGroup.SHOTPLAN, "Shot consumed from plan");
            }
            
            // Queue next transfer ONLY if no operations running
            // This prevents conflicts with ongoing transfers or collections
            if (!runner.isBusy() && !indexingHelper.isTransferActive()) {
                queueNextShotInBurst();
            } else {
                Dbg.d(LogGroup.FIRING, "Deferring next transfer queue (operation busy)");
                deferredTransferNeeded = true;  // Will queue when operation completes
            }
        }
    }
    
    /**
     * Control intake rollers during hunt mode.
     * Hunt-eligible intakes run rollers at 0.6 power.
     * Non-hunt-eligible intakes stop rollers (unless holding artifact at 0.3 power).
     * Storage mode includes stall detection - if rollers don't move, stop trying.
     */
    private void updateHuntingRollers() {
        // Issue 2 Fix: Don't control hardware if operation is running
        if (runner.isBusy()) {
            return;  // Operations have exclusive control
        }
        
        final double HUNT_ROLLER_POWER = 0.6;     // Hunt mode roller power
        final double STORAGE_ROLLER_POWER = 0.3;  // Storage mode hold power

        // Front intake roller control
        if (isIntakeHuntEligible(SlotLedger.Slot.FRONT)) {
            // Hunt-eligible: run roller at hunt power (0.6)
            indexingHelper.setFrontRollerPower(HUNT_ROLLER_POWER);
        } else if (ledger.isOccupied(SlotLedger.Slot.FRONT)) {
            // Storage intake: run at hold power UNLESS stalled
            if (!frontRollerStalled) {
                indexingHelper.setFrontRollerPower(STORAGE_ROLLER_POWER);
            } else {
                // Stalled - stop trying
                indexingHelper.setFrontRollerPower(0);
                Dbg.d(LogGroup.INTAKE, "FRONT roller stalled in storage mode, stopped");
            }
        } else {
            // Not hunt-eligible and empty: stop roller and reset stall flag
            indexingHelper.setFrontRollerPower(0);
            if (frontRollerStalled) {
                frontRollerStalled = false;
                frontStallCheckCount = 0;
                Dbg.d(LogGroup.INTAKE, "FRONT intake empty, stall flag reset");
            }
        }
        
        // Back intake roller control
        if (isIntakeHuntEligible(SlotLedger.Slot.BACK)) {
            // Hunt-eligible: run roller at hunt power (0.6)
            indexingHelper.setBackRollerPower(HUNT_ROLLER_POWER);
        } else if (ledger.isOccupied(SlotLedger.Slot.BACK)) {
            // Storage intake: run at hold power UNLESS stalled
            if (!backRollerStalled) {
                indexingHelper.setBackRollerPower(STORAGE_ROLLER_POWER);
            } else {
                // Stalled - stop trying
                indexingHelper.setBackRollerPower(0);
                Dbg.d(LogGroup.INTAKE, "BACK roller stalled in storage mode, stopped");
            }
        } else {
            // Not hunt-eligible and empty: stop roller and reset stall flag
            indexingHelper.setBackRollerPower(0);
            if (backRollerStalled) {
                backRollerStalled = false;
                backStallCheckCount = 0;
                Dbg.d(LogGroup.INTAKE, "BACK intake empty, stall flag reset");
            }
        }
    }
    
    /**
     * Stop hunt-mode rollers.
     * Called when operations take control.
     */
    /**
     * Stop hunt-mode rollers.
     * Called when operations take control.
     *
     * CRITICAL: When operations are running, DO NOT touch the rollers!
     * Operations have exclusive control and set roller powers directly every loop.
     * If we set them to 0 here, we'll fight with the operation and cause intermittent behavior.
     *
     * This method is now a NO-OP - operations control hardware, we don't interfere.
     */
    private void stopHuntingRollers() {
        // DO NOTHING - let operations have exclusive control
        // Previously, this was setting rollers to 0, which fought with operation control
    }
    
    /**
     * Control transfer servos for hunt mode.
     * 
     * When hunt-eligible (waiting to collect), run transfer servos FORWARD at moderate speed.
     * This helps guide artifacts toward center and keeps them from sitting in the intake.
     * The forward motion (toward center) combined with roller motion helps position
     * artifacts for better sensor detection.
     *
     * Forward power means the servo gently pushes artifact toward center.
     * Combined with roller motion, this helps maintain artifact position.
     *
     * Power is adjusted based on system state:
     * - No artifacts (0/3): Full power (-0.35) for aggressive collection
     * - Has artifacts (1-2/3): Reduced power (-0.2) for gentler handling
     *
     * IMPORTANT: Only applies hunt-mode power when no timed movement is active.
     * Operations use timed movements for transfers, and we must not interfere.
     */
    private void updateHuntingTransferServos() {
        // Issue 2 Fix: Don't control hardware if operation is running
        if (runner.isBusy()) {
            return;  // Operations have exclusive control
        }
        
        // Power for forward motion (negative = forward toward center)
        // Adjust power based on whether system already has artifacts
        final double HUNT_TRANSFER_FULL_POWER = -0.35;    // When system empty (0 artifacts)
        final double HUNT_TRANSFER_REDUCED_POWER = -0.2;  // When system has artifacts (1-2 artifacts)

        // Select power based on current artifact count
        boolean hasArtifacts = ledger.getArtifactCount() > 0;
        double huntTransferPower = hasArtifacts ? HUNT_TRANSFER_REDUCED_POWER : HUNT_TRANSFER_FULL_POWER;

        // Front intake: run transfer servo forward if hunt-eligible
        // BUT: Don't interfere if operation has active timed movement
        if (isIntakeHuntEligible(SlotLedger.Slot.FRONT) && !indexingHelper.isFrontTransferBusy()) {
            indexingHelper.setFrontTransferPower(huntTransferPower);
        } else if (!indexingHelper.isFrontTransferBusy()) {
            // Not hunt-eligible and no operation: stop transfer servo
            // (If operation is busy, let it control the servo)
            indexingHelper.setFrontTransferPower(0);
        }
        
        // Back intake: run transfer servo forward if hunt-eligible
        // BUT: Don't interfere if operation has active timed movement
        if (isIntakeHuntEligible(SlotLedger.Slot.BACK) && !indexingHelper.isBackTransferBusy()) {
            indexingHelper.setBackTransferPower(huntTransferPower);
        } else if (!indexingHelper.isBackTransferBusy()) {
            // Not hunt-eligible and no operation: stop transfer servo
            // (If operation is busy, let it control the servo)
            indexingHelper.setBackTransferPower(0);
        }
    }
    
    /**
     * Stop hunt-mode transfer servos.
     * Called when operations take control of servos.
     * 
     * CRITICAL: When operations are running, DO NOT touch the servos at all!
     * Operations have exclusive control and set servo powers directly every loop.
     * If we set them to 0 here, we'll fight with the operation and cause intermittent behavior.
     *
     * This method is now a NO-OP - operations control servos, we don't interfere.
     */
    private void stopHuntingTransferServos() {
        // DO NOTHING - let operations have exclusive control
        // Previously, this was setting servos to 0, which fought with operation control
        // and caused intermittent servo behavior (run, stop, run, stop pattern)
    }
    
    /**
     * Check if intake is eligible to hunt for artifacts.
     * 
     * Eligible when:
     * - Hunt mode ON
     * - Slot is empty (not storing an artifact)
     * - System not full (has capacity)
     * - No operation running (not busy)
     * 
     * @param slot FRONT or BACK intake
     * @return true if intake should hunt (run rollers, poll sensors)
     */
    private boolean isIntakeHuntEligible(SlotLedger.Slot slot) {
        if (!huntEnabled) {
            return false;  // Hunt mode OFF - sleep
        }
        
        if (slot == SlotLedger.Slot.CENTER) {
            return false;  // Center slot never hunts
        }
        
        // Check if slot is occupied
        boolean slotOccupied = (slot == SlotLedger.Slot.FRONT) ? 
                              ledger.isFrontOccupied() : ledger.isBackOccupied();
        
        if (slotOccupied) {
            return false;  // Can't hunt if storing an artifact
        }
        
        if (ledger.isFull()) {
            return false;  // Can't hunt if system full
        }
        
        if (runner.isBusy()) {
            return false;  // Can't hunt if operation running
        }
        
        return true;  // Eligible to hunt!
    }
    
    /**
     * Check for roller stalls in storage mode.
     * If rollers don't move when power is applied, mark as stalled after 3 checks.
     * Only monitors intakes in storage mode (occupied but not hunt-eligible).
     * Non-blocking check every 100ms.
     */
    private void checkRollerStalls(long currentTime) {
        // Only check in storage mode (not during operations)
        if (runner.isBusy()) return;

        // Check FRONT roller if in storage mode
        if (ledger.isOccupied(SlotLedger.Slot.FRONT) && !isIntakeHuntEligible(SlotLedger.Slot.FRONT) && !frontRollerStalled) {
            if ((currentTime - lastFrontStallCheckTime) >= STALL_CHECK_INTERVAL_MS) {
                // Time to check
                int currentPosition = indexingHelper.getFrontRollerPosition();
                int movement = Math.abs(currentPosition - lastFrontRollerPosition);

                if (movement < STALL_MOVEMENT_THRESHOLD) {
                    // Not moving enough - increment stall count
                    frontStallCheckCount++;
                    Dbg.d(LogGroup.INTAKE, "FRONT roller stall check %d/%d (movement=%d ticks)",
                          frontStallCheckCount, STALL_CHECK_COUNT, movement);

                    if (frontStallCheckCount >= STALL_CHECK_COUNT) {
                        // Stalled after multiple checks
                        frontRollerStalled = true;
                        Dbg.w(LogGroup.INTAKE, "FRONT roller STALLED in storage mode (no movement detected)");
                    }
                } else {
                    // Moving - reset stall count
                    frontStallCheckCount = 0;
                }

                lastFrontRollerPosition = currentPosition;
                lastFrontStallCheckTime = currentTime;
            }
        }

        // Check BACK roller if in storage mode
        if (ledger.isOccupied(SlotLedger.Slot.BACK) && !isIntakeHuntEligible(SlotLedger.Slot.BACK) && !backRollerStalled) {
            if ((currentTime - lastBackStallCheckTime) >= STALL_CHECK_INTERVAL_MS) {
                // Time to check
                int currentPosition = indexingHelper.getBackRollerPosition();
                int movement = Math.abs(currentPosition - lastBackRollerPosition);

                if (movement < STALL_MOVEMENT_THRESHOLD) {
                    // Not moving enough - increment stall count
                    backStallCheckCount++;
                    Dbg.d(LogGroup.INTAKE, "BACK roller stall check %d/%d (movement=%d ticks)",
                          backStallCheckCount, STALL_CHECK_COUNT, movement);

                    if (backStallCheckCount >= STALL_CHECK_COUNT) {
                        // Stalled after multiple checks
                        backRollerStalled = true;
                        Dbg.w(LogGroup.INTAKE, "BACK roller STALLED in storage mode (no movement detected)");
                    }
                } else {
                    // Moving - reset stall count
                    backStallCheckCount = 0;
                }

                lastBackRollerPosition = currentPosition;
                lastBackStallCheckTime = currentTime;
            }
        }
    }

    /**
     * Handle operation completion.
     */
    private void handleOperationComplete() {
        IndexingOperation lastOp = lastCompletedOperation;
        if (lastOp == null) return;
        
        Dbg.d(LogGroup.PLANNEREX, "handleOperationComplete: %s, success=%b",
                         lastOp.getOperationName(), lastOp.isSuccess());

        // Only process successful operations
        if (!lastOp.isSuccess()) return;
        
        // Update statistics
        if (lastOp instanceof CollectOperation) {
            totalCollections++;
            Dbg.d(LogGroup.INDEXING, "Collection completed, calling handlePostCollection()");

            // Post-Collection Logic: Automatically handle artifact placement
            // This is the CRITICAL missing piece - after collecting, we need to:
            // - 1st artifact: Transfer to center
            // - 2nd artifact: Check if swap needed (shot planning), otherwise stay in intake
            // - 3rd artifact: Stay in intake
            // 
            // IMPORTANT: During burst firing, skip only the 1st artifact transfer logic
            // (burst firing manages CENTER transfers). But still handle 2nd/3rd artifacts
            // (they need to be added to storage or swapped).
            // 
            // The collected artifact is already in the ledger, so we just need to decide
            // whether to transfer/swap it or leave it in the intake.
            if (!burstFiringActive || ledger.getArtifactCount() >= 2) {
                // Call handlePostCollection if:
                // - Not in burst mode (normal operation), OR
                // - In burst mode but this is 2nd or 3rd artifact (handle storage/swap logic)
                handlePostCollection();
            } else {
                // In burst mode and this is 1st artifact - skip to avoid conflict with burst transfers
                Dbg.d(LogGroup.INDEXING, "Skipping handlePostCollection for 1st artifact (burst firing active)");
            }
            
        } else if (lastOp instanceof TransferOperation) {
            totalTransfers++;
            Dbg.d(LogGroup.TRANSFER, "Transfer completed");

            // CRITICAL: Reset perception for the source intake to prevent false detections
            // After an artifact physically moves away, sensors may still show presence briefly
            // Resetting clears stale sensor data and prevents immediate false re-collection
            String opName = lastOp.getOperationName();
            if (opName.contains("FRONT")) {
                frontPerception.reset();
                Dbg.d(LogGroup.INTAKE, "Reset FRONT perception after transfer");
            } else if (opName.contains("BACK")) {
                backPerception.reset();
                Dbg.d(LogGroup.INTAKE, "Reset BACK perception after transfer");
            }
            
            // If we deferred a transfer, queue it now
            // This happens when a shot fired while an operation was running
            // We need to transfer the next artifact to CENTER regardless of burst mode state
            if (deferredTransferNeeded) {
                Dbg.d(LogGroup.FIRING, "Queuing deferred transfer (burstActive=%b)", burstFiringActive);
                if (burstFiringActive) {
                    queueNextShotInBurst();
                } else {
                    // Burst was cancelled but we still need to fill CENTER
                    // Use auto-transfer logic in performAutomaticOperations
                    Dbg.d(LogGroup.TRANSFER, "Burst cancelled, letting auto-transfer handle it");
                }
                deferredTransferNeeded = false;
            }
            
        } else if (lastOp instanceof SwapOperation) {
            totalSwaps++;
            Dbg.d(LogGroup.TRANSFER, "Swap completed");
        } else if (lastOp instanceof FireOperation) {
            FireOperation fireOp = (FireOperation) lastOp;
            if (!fireOp.wasCancelledBeforeShot()) {
                totalShots++;
                consecutiveShotsFired++;
                lastFiredArtifact = fireOp.getFiredArtifact();  // Track for telemetry
                Dbg.i(LogGroup.FIRING, "Shot fired, total=%d", totalShots);

                // If burst firing, queue next transfer if more shots needed
                if (burstFiringActive && shouldContinueBurst()) {
                    queueNextShotInBurst();
                }
            } else {
                // Cancelled before shot - end burst
                burstFiringActive = false;
                firingHelper.cancelFiring();
                Dbg.i(LogGroup.FIRING, "Fire cancelled before shot, ending burst");
            }
        } else if (lastOp instanceof EjectOperation) {
            totalEjections++;
            burstFiringActive = false;  // Ejection ends burst
            Dbg.d(LogGroup.EJECT, "Ejection completed");
        }
    }
    
    /**
     * Handle post-collection logic to automatically place collected artifacts.
     * 
     * Collection Rules:
     * - 1st artifact: Transfer to center (ready to fire)
     * - 2nd artifact: Stay in intake UNLESS shot planner says swap
     * - 3rd artifact: Stay in intake (system full)
     * 
     * Storage Mode: Artifacts in intakes are in "storage mode" - rollers run at
     * hold power to retain artifact, not collect power.
     */
    private void handlePostCollection() {
        int artifactCount = ledger.getArtifactCount();
        Dbg.d(LogGroup.INDEXING, "handlePostCollection: artifactCount=%d", artifactCount);

        if (artifactCount == 1) {
            // FIRST ARTIFACT: Transfer to center immediately
            // Find which intake has the artifact
            SlotLedger.Slot sourceSlot = null;
            if (ledger.isFrontOccupied()) {
                sourceSlot = SlotLedger.Slot.FRONT;
            } else if (ledger.isBackOccupied()) {
                sourceSlot = SlotLedger.Slot.BACK;
            }
            
            if (sourceSlot != null) {
                Dbg.d(LogGroup.TRANSFER, "1st artifact - requesting transfer from %s", sourceSlot);
                // Queue transfer to center
                boolean success = requestTransfer(sourceSlot);
                Dbg.d(LogGroup.TRANSFER, "Transfer request %s", success ? "SUCCESSFUL" : "FAILED");
            } else {
                Dbg.w(LogGroup.INDEXING, "WARNING: 1st artifact but no occupied intake found!");
            }
            
        } else if (artifactCount == 2) {
            // SECOND ARTIFACT: Check if swap needed for optimal shot order
            Dbg.d(LogGroup.SHOTPLAN, "2nd artifact - checking shot planner...");
            // The shot planner will determine if we need to rearrange
            if (shotPlanner.isRearrangementNeeded()) {
                SlotLedger.Slot swapSlot = shotPlanner.getRearrangementSlot();
                if (swapSlot != null) {
                    Dbg.d(LogGroup.SHOTPLAN, "Shot planner recommends swap with %s", swapSlot);
                    // Swap needed - do it now
                    boolean success = requestSwap(swapSlot);
                    Dbg.d(LogGroup.TRANSFER, "Swap request %s", success ? "SUCCESSFUL" : "FAILED");
                }
            } else {
                Dbg.d(LogGroup.SHOTPLAN, "No swap needed, artifact stays in storage");
            }
            // If no swap needed, artifact stays in intake (storage mode)
            // Hunt-mode will automatically run rollers at storage power
            
        } else if (artifactCount == 3) {
            // THIRD ARTIFACT: System full, stays in intake (storage mode)
            Dbg.d(LogGroup.INDEXING, "3rd artifact - system full, stays in storage");
            // Nothing to do - artifact is already committed to slot
            // Hunt-mode will run rollers at storage power
        }
    }
    
    /**
     * Update system state based on current conditions.
     */
    private void updateSystemState() {
        if (runner.isBusy()) {
            // State follows active operation
            IndexingOperation current = runner.getCurrentOperation();
            if (current instanceof CollectOperation) {
                currentState = SystemState.COLLECTING;
            } else if (current instanceof TransferOperation) {
                currentState = SystemState.TRANSFERRING;
            } else if (current instanceof SwapOperation) {
                currentState = SystemState.REARRANGING;
            } else if (current instanceof FireOperation) {
                currentState = SystemState.FIRING;
            } else if (current instanceof EjectOperation) {
                currentState = SystemState.EJECTING;
            }
        } else {
            // Idle or ready
            if (ledger.isCenterOccupied() && shooter.isReadyToFire()) {
                currentState = SystemState.READY_TO_FIRE;
            } else if (ledger.getArtifactCount() > 0) {
                currentState = SystemState.IDLE;  // Has artifacts but not ready
            } else {
                currentState = SystemState.IDLE;
            }
        }
    }
    
    /**
     * Perform automatic operations based on system state.
     */
    private void performAutomaticOperations() {
        // Issue 3 Fix: Add cooldown to prevent repeated auto-collections
        long currentTime = System.currentTimeMillis();
        
        // Auto-transfer to CENTER when empty (not in burst firing mode)
        // Burst firing handles its own transfers via queueNextShotInBurst()
        if (!ledger.isCenterOccupied() && !burstFiringActive && !firingHelper.isFiring()) {
            // CENTER is empty and we're not actively firing - try to fill it
            SlotLedger.Slot transferSlot = shotPlanner.getNextTransferSlot(ledger);
            
            if (transferSlot == null) {
                // Shot planner doesn't have a next shot (plan exhausted)
                // Fall back to any available artifact
                if (ledger.isFrontOccupied()) {
                    transferSlot = SlotLedger.Slot.FRONT;
                } else if (ledger.isBackOccupied()) {
                    transferSlot = SlotLedger.Slot.BACK;
                }
            }
            
            if (transferSlot != null) {
                Dbg.d(LogGroup.TRANSFER, "Auto-transfer to CENTER: %s → CENTER", transferSlot);
                requestTransfer(transferSlot);
                return;  // Skip other auto-operations this loop
            }
        }
        
        // Auto-detect and handle artifacts in intakes
        // Detection is separate from collection to prevent duplicates
        handleIntakeDetection(SlotLedger.Slot.FRONT, frontPerception, currentTime);
        handleIntakeDetection(SlotLedger.Slot.BACK, backPerception, currentTime);

        // Auto-rearrange if shot planner detects benefit
        if (shotPlanner.isRearrangementNeeded() && ledger.getArtifactCount() == 2) {
            SlotLedger.Slot swapSlot = shotPlanner.getRearrangementSlot();
            if (swapSlot != null) {
                requestSwap(swapSlot);
            }
        }
    }
    
    // ========== Detection and Collection Logic ==========

    /**
     * Handle artifact detection in an intake.
     * Prevents duplicate detection by tracking detection state per intake.
     *
     * Detection Flow:
     * 1. Artifact detected → Start confirmation timer (200ms)
     * 2. After confirmation delay → Add to ledger and decide next action
     * 3. If first artifact → Transfer to center immediately
     * 4. If second artifact → Check shot plan, swap if beneficial, else store
     * 5. If third artifact → Store in intake (storage mode)
     *
     * @param slot FRONT or BACK intake
     * @param perception IntakePerception for this intake
     * @param currentTime Current time in milliseconds
     */
    private void handleIntakeDetection(SlotLedger.Slot slot, IntakePerception perception, long currentTime) {
        if (!huntEnabled) return;  // Hunt mode OFF
        if (!isIntakeHuntEligible(slot)) return;  // Not eligible for detection
        if (runner.isBusy()) return;  // Operation already running

        boolean isFront = (slot == SlotLedger.Slot.FRONT);

        // Check if intake hardware is busy (transfer in progress)
        if (isFront && indexingHelper.isFrontTransferBusy()) return;
        if (!isFront && indexingHelper.isBackTransferBusy()) return;

        // Check transfer cooldown (prevents ghost detection after transfer)
        long lastTransferTime = isFront ? lastFrontTransferTime : lastBackTransferTime;
        if ((currentTime - lastTransferTime) < TRANSFER_COOLDOWN_MS) {
            return;  // Still in cooldown period
        }

        boolean detected = isFront ? frontArtifactDetected : backArtifactDetected;
        long detectionStartTime = isFront ? frontDetectionStartTime : backDetectionStartTime;

        // Check if artifact detected by perception
        boolean artifactPresent = perception.getFastPresence();
        IntakePerception.PresenceConfidence confidence = perception.getPresenceConfidence();
        IntakePerception.PresenceConfidence requiredConfidence = skipColorDetection ?
            IntakePerception.PresenceConfidence.MEDIUM : IntakePerception.PresenceConfidence.HIGH;

        // State machine: Artifact detection → Confirmation → Collection → Transfer/Storage
        if (!detected && artifactPresent && confidence.ordinal() >= requiredConfidence.ordinal()) {
            // NEW DETECTION: Start confirmation timer
            if (isFront) {
                frontArtifactDetected = true;
                frontDetectionStartTime = currentTime;
            } else {
                backArtifactDetected = true;
                backDetectionStartTime = currentTime;
            }
            Dbg.d(LogGroup.INTAKE, "%s: Artifact detected, starting confirmation (conf=%s)", slot, confidence);
        } else if (detected) {
            // DETECTION IN PROGRESS: Check confirmation timer
            long elapsedSinceDetection = currentTime - detectionStartTime;

            if (!artifactPresent || confidence.ordinal() < requiredConfidence.ordinal()) {
                // Lost detection - reset
                if (isFront) {
                    frontArtifactDetected = false;
                } else {
                    backArtifactDetected = false;
                }
                Dbg.d(LogGroup.INTAKE, "%s: Lost detection, resetting", slot);
            } else if (elapsedSinceDetection >= DETECTION_CONFIRMATION_DELAY_MS || skipColorDetection) {
                // CONFIRMED: Handle collection
                if (skipColorDetection) {
                    // Skip mode: collect immediately without color detection
                    handleArtifactCollection(slot, ArtifactIdentity.ColorClass.UNKNOWN, currentTime);
                } else {
                    // Normal mode: sample color first
                    // CRITICAL: Enable color sampling before reading!
                    perception.enableColorSampling();
                    perception.update();  // Update to get fresh sensor readings

                    ArtifactIdentity.ColorClass color = perception.getBestColorClass();
                    double colorConfidence = perception.getBestColorConfidence();

                    // Disable color sampling after reading
                    perception.disableColorSampling();

                    Dbg.d(LogGroup.INTAKE, "%s: Color sampled: %s (conf=%.2f)", slot, color, colorConfidence);
                    handleArtifactCollection(slot, color, currentTime);
                }

                // Reset detection state (prevents duplicate detection)
                if (isFront) {
                    frontArtifactDetected = false;
                } else {
                    backArtifactDetected = false;
                }
            }
            // else: still waiting for confirmation delay
        }
    }

    /**
     * Handle confirmed artifact collection.
     * Determines what to do with the artifact based on current robot state.
     *
     * Logic:
     * - If 3 artifacts → Ignore (system full)
     * - If 0 artifacts → Add to ledger, transfer to center immediately
     * - If 1 artifact (in center) → Add to ledger, check shot plan:
     *   - If center needs swapping → Swap
     *   - Else → Store in intake
     * - If 2 artifacts → Add to ledger, store in intake (storage mode)
     *
     * @param slot FRONT or BACK intake
     * @param color Detected color (or UNKNOWN if skip mode)
     * @param currentTime Current time in milliseconds
     */
    private void handleArtifactCollection(SlotLedger.Slot slot, ArtifactIdentity.ColorClass color, long currentTime) {
        int currentCount = ledger.getArtifactCount();

        // Check if system full
        if (currentCount >= 3) {
            Dbg.d(LogGroup.INTAKE, "%s: System full (3 artifacts), ignoring detection", slot);
            return;
        }

        // Add artifact to ledger immediately
        ArtifactIdentity artifact = ArtifactIdentity.createFromSensor(color, 1.0, nextSequenceId++);
        ledger.set(slot, artifact);
        totalCollections++;
        Dbg.i(LogGroup.INTAKE, "%s: Artifact added to ledger (%s, id=%d)", slot, color, artifact.getSequenceId());

        // Decide next action based on current state
        if (currentCount == 0) {
            // FIRST ARTIFACT: Transfer to center immediately
            Dbg.i(LogGroup.INTAKE, "%s: First artifact, transferring to center", slot);

            // Start transfer cooldown to prevent ghost detection
            if (slot == SlotLedger.Slot.FRONT) {
                lastFrontTransferTime = currentTime;
            } else {
                lastBackTransferTime = currentTime;
            }

            // Reset perception to clear sensor state before transfer
            IntakePerception perception = (slot == SlotLedger.Slot.FRONT) ? frontPerception : backPerception;
            perception.resetPresenceDetection();

            requestTransfer(slot);
        } else if (currentCount == 1) {
            // SECOND ARTIFACT: Decide behavior based on skip mode and auto-swap setting

            // Auto-swap feature for skip mode: When enabled, 2nd artifact pushes 1st to opposite intake
            if (skipColorDetection && autoSwapEnabled && wasAutoSwapEnabledLastCheck && ledger.isCenterOccupied()) {
                // AUTO-SWAP MODE: Push 1st artifact (in CENTER) to opposite intake from 2nd artifact
                // This allows rapid collection: 1st → CENTER, 2nd collected → swap → 2nd in CENTER, 1st in opposite
                Dbg.i(LogGroup.INTAKE, "%s: Auto-swap enabled (skip mode), swapping 2nd artifact to center", slot);

                // Start transfer cooldown for swap operation
                if (slot == SlotLedger.Slot.FRONT) {
                    lastFrontTransferTime = currentTime;
                } else {
                    lastBackTransferTime = currentTime;
                }

                // Reset perception before swap
                IntakePerception perception = (slot == SlotLedger.Slot.FRONT) ? frontPerception : backPerception;
                perception.resetPresenceDetection();

                // Request swap: 2nd artifact (in slot) ↔ 1st artifact (in CENTER)
                // Result: 2nd → CENTER, 1st → opposite intake (automatic by SwapOperation)
                requestSwap(slot);

            } else if (ledger.isCenterOccupied() && !skipColorDetection) {
                // NORMAL MODE: Check shot plan to see if we want a different color in center
                ArtifactIdentity centerArtifact = ledger.getCenter();
                ArtifactIdentity.ColorClass desiredCenterColor = shotPlanner.getDesiredCenterColor();

                if (desiredCenterColor != null &&
                    centerArtifact.getColorClass() != desiredCenterColor &&
                    artifact.getColorClass() == desiredCenterColor) {
                    // Beneficial swap: new artifact is what we want in center
                    Dbg.i(LogGroup.INTAKE, "%s: Beneficial swap detected (want %s, have %s), swapping",
                          slot, desiredCenterColor, centerArtifact.getColorClass());

                    // Start transfer cooldown for swap operation
                    if (slot == SlotLedger.Slot.FRONT) {
                        lastFrontTransferTime = currentTime;
                    } else {
                        lastBackTransferTime = currentTime;
                    }

                    // Reset perception before swap
                    IntakePerception perception = (slot == SlotLedger.Slot.FRONT) ? frontPerception : backPerception;
                    perception.resetPresenceDetection();

                    requestSwap(slot);
                } else {
                    // No swap needed: store in intake
                    Dbg.i(LogGroup.INTAKE, "%s: Second artifact, storing in intake (no beneficial swap)", slot);
                    setIntakeStorageMode(slot, true);
                }
            } else {
                // Skip mode with auto-swap OFF, or center not occupied: just store
                String reason = !autoSwapEnabled ? "auto-swap disabled" :
                               !skipColorDetection ? "not skip mode" : "center empty";
                Dbg.i(LogGroup.INTAKE, "%s: Second artifact, storing in intake (%s)", slot, reason);
                setIntakeStorageMode(slot, true);
            }
        } else {
            // THIRD ARTIFACT: Store in intake (storage mode)
            Dbg.i(LogGroup.INTAKE, "%s: Third artifact, storing in intake (storage mode)", slot);
            setIntakeStorageMode(slot, true);
        }
    }

    /**
     * Set intake storage mode (run rollers at hold power to retain artifact).
     *
     * @param slot FRONT or BACK intake
     * @param storageMode true to enable storage mode, false to disable
     */
    private void setIntakeStorageMode(SlotLedger.Slot slot, boolean storageMode) {
        if (storageMode) {
            double storagePower = 0.3;  // Hold power for storage (reduced from 0.4)
            if (slot == SlotLedger.Slot.FRONT) {
                indexingHelper.setFrontRollerPower(storagePower);
            } else {
                indexingHelper.setBackRollerPower(storagePower);
            }
            Dbg.d(LogGroup.INTAKE, "%s: Storage mode enabled (power=%.1f)", slot, storagePower);
        } else {
            // Stop rollers
            if (slot == SlotLedger.Slot.FRONT) {
                indexingHelper.setFrontRollerPower(0.0);
            } else {
                indexingHelper.setBackRollerPower(0.0);
            }
            Dbg.d(LogGroup.INTAKE, "%s: Storage mode disabled", slot);
        }
    }

    // ========== Public API - Operation Requests ==========
    
    /**
     * Request collection from specified intake.
     * 
     * @param slot FRONT or BACK intake
     * @return true if operation started, false if rejected
     */
    public boolean requestCollect(SlotLedger.Slot slot) {
        if (!enabled) return false;
        if (slot == SlotLedger.Slot.CENTER) return false;  // Invalid slot
        
        IntakePerception perception = (slot == SlotLedger.Slot.FRONT) ? frontPerception : backPerception;
        
        CollectOperation op = new CollectOperation(
            ledger, perception, indexingHelper, config, slot, nextSequenceId++, skipColorDetection, telemetry
        );
        
        return runner.start(op);
    }
    
    /**
     * Request transfer from intake to center.
     * 
     * @param slot FRONT or BACK intake
     * @return true if operation started, false if rejected
     */
    public boolean requestTransfer(SlotLedger.Slot slot) {
        if (!enabled) return false;
        if (slot == SlotLedger.Slot.CENTER) return false;  // Invalid slot
        
        TransferOperation op = new TransferOperation(
            ledger, centerPerception, indexingHelper, slot, telemetry
        );
        
        boolean started = runner.start(op);

        // If transfer started, set cooldown timestamp and reset perception
        if (started) {
            long currentTime = System.currentTimeMillis();
            if (slot == SlotLedger.Slot.FRONT) {
                lastFrontTransferTime = currentTime;
                // Reset perception to prevent ghost detection
                frontPerception.resetPresenceDetection();
            } else {
                lastBackTransferTime = currentTime;
                // Reset perception to prevent ghost detection
                backPerception.resetPresenceDetection();
            }

            Dbg.d(LogGroup.INTAKE, "%s: Transfer started, cooldown active for %dms", slot, TRANSFER_COOLDOWN_MS);
        }

        return started;
    }
    
    /**
     * Request swap between center and intake.
     * 
     * @param intakeSlot FRONT or BACK intake to swap with center
     * @return true if operation started, false if rejected
     */
    public boolean requestSwap(SlotLedger.Slot intakeSlot) {
        if (!enabled) return false;
        if (intakeSlot == SlotLedger.Slot.CENTER) return false;  // Invalid slot
        
        SwapOperation op = new SwapOperation(
            ledger, indexingHelper, config, intakeSlot, telemetry
        );
        
        boolean started = runner.start(op);

        // If swap started, set cooldown timestamp and reset perception on BOTH intakes
        // During swap: artifact from intakeSlot goes to center, artifact from center goes to opposite
        // Both intakes need cooldown to prevent ghost detection
        if (started) {
            long currentTime = System.currentTimeMillis();

            // Set cooldown on BOTH intakes
            lastFrontTransferTime = currentTime;
            lastBackTransferTime = currentTime;

            // Reset perception on BOTH intakes to prevent ghost detection
            frontPerception.resetPresenceDetection();
            backPerception.resetPresenceDetection();

            Dbg.d(LogGroup.INTAKE, "Swap started: %s ↔ CENTER, cooldown active on BOTH intakes for %dms",
                  intakeSlot, TRANSFER_COOLDOWN_MS);
        }

        return started;
    }
    
    /**
     * Request preposition (move center artifact to firing position).
     * 
     * @return true if operation started, false if rejected
     */
    public boolean requestPreposition() {
        if (!enabled) return false;
        
        PrepositionOperation op = new PrepositionOperation(
            ledger, indexingHelper, false, telemetry  // false = not yet prepositioned
        );
        
        return runner.start(op);
    }
    
    /**
     * Request single fire shot.
     * 
     * @return true if operation started, false if rejected
     */
    public boolean requestFire() {
        return requestFire(true, null);
    }
    
    /**
     * Request fire shot with specified RPM (keep-alive mode enabled).
     * 
     * This method starts the firing sequence using BasicFiringHelper's built-in functionality.
     * The FiringHelper will automatically:
     * - Spin up the shooter to target RPM
     * - Feed the artifact when ready
     * - Keep the shooter spinning (keep-alive mode)
     * - Track shot completion
     * 
     * After the first shot, use fireNextShot() when isReadyForNextShot() returns true.
     * 
     * @param rpm Target RPM for shooter
     * @return true if started successfully, false if already firing or no artifact
     */
    public boolean requestFire(double rpm) {
        if (!enabled) return false;
        
        // Check if CENTER has an artifact
        if (!ledger.isCenterOccupied()) {
            telemetry.addData("⚠️ Fire", "CENTER empty - cannot fire");
            return false;
        }
        
        // Start firing with keep-alive mode using BasicFiringHelper
        // This handles spinup automatically and keeps shooter spinning for follow-up shots
        boolean started = firingHelper.startFiring(rpm, "CUSTOM", true);
        
        if (started) {
            burstFiringActive = true;
            consecutiveShotsFired = 0;
            firingHelper.resetShotDetection();
        }
        
        return started;
    }
    
    /**
     * Request fire shot with optional keep-alive and cancellation callback.
     * 
     * @param keepAlive Enable keep-alive mode (shooter stays spun)
     * @param shouldContinue Optional callback for mid-operation cancellation
     * @return true if operation started, false if rejected
     */
    public boolean requestFire(boolean keepAlive, FireOperation.ShouldContinueCallback shouldContinue) {
        if (!enabled) return false;
        
        double rpm = shooter.getTargetRPM();  // Use current shooter RPM
        
        FireOperation op = new FireOperation(
            ledger, firingHelper, shooter, rpm, keepAlive, shotPlanner, shouldContinue, telemetry
        );
        
        boolean started = runner.start(op);
        if (started && keepAlive) {
            burstFiringActive = true;
            consecutiveShotsFired = 0;
            // Initialize shot count tracking for subsequent shot detection
            lastKnownShotCount = firingHelper.getShotsFiredCount();
        }
        
        return started;
    }
    
    /**
     * Request burst firing sequence (keep-alive mode, multiple shots).
     * Shooter stays spun between shots, transfers happen automatically.
     * 
     * @param shouldContinue Callback checked every loop (e.g., trigger held)
     * @return true if burst started, false if rejected
     */
    public boolean requestBurstFire(FireOperation.ShouldContinueCallback shouldContinue) {
        boolean started = requestFire(true, shouldContinue);
        if (started) {
            firingHelper.resetShotDetection();  // Reset counter for burst sequence
        }
        return started;
    }
    
    /**
     * Request eject operation.
     * 
     * @param mode ALL, CENTER, or SOFTWARE_CLEAR
     * @return true if operation started, false if rejected
     */
    public boolean requestEject(EjectOperation.EjectMode mode) {
        if (!enabled) return false;
        
        EjectOperation op = new EjectOperation(
            ledger, firingHelper, indexingHelper, mode, telemetry
        );
        
        boolean started = runner.start(op);
        if (started) {
            // Cancel burst firing if ejecting
            if (burstFiringActive) {
                firingHelper.cancelFiring();
                burstFiringActive = false;
            }
        }
        
        return started;
    }
    
    // ========== Burst Firing Helpers ==========
    
    /**
     * Check if burst firing should continue.
     */
    private boolean shouldContinueBurst() {
        // Continue if:
        // 1. Burst is active
        // 2. Still have artifacts to fire (in intakes)
        // 3. Shot plan not exhausted (if using planner)
        return burstFiringActive && 
               (ledger.isFrontOccupied() || ledger.isBackOccupied()) &&
               consecutiveShotsFired < 5;  // Max 5 shots per burst (safety)
    }
    
    /**
     * Queue next shot in burst sequence.
     */
    private void queueNextShotInBurst() {
        // Determine which slot to transfer next
        SlotLedger.Slot nextSlot = shotPlanner.getNextTransferSlot(ledger);
        
        if (nextSlot == null) {
            // Shot planner doesn't have a next shot (plan exhausted)
            // Fall back to any available artifact (handles mid-burst collections)
            if (ledger.isFrontOccupied()) {
                nextSlot = SlotLedger.Slot.FRONT;
                Dbg.d(LogGroup.FIRING, "Shot plan exhausted, using FRONT artifact");
            } else if (ledger.isBackOccupied()) {
                nextSlot = SlotLedger.Slot.BACK;
                Dbg.d(LogGroup.FIRING, "Shot plan exhausted, using BACK artifact");
            }
        }
        
        if (nextSlot != null) {
            // Transfer artifact to center
            requestTransfer(nextSlot);
            // Note: Firing will happen automatically after transfer completes
            // (via automatic operations or explicit request in next update)
        } else {
            // No artifacts in intakes to transfer
            // If CENTER is occupied, keep burst active (still have artifact to fire)
            // Only end burst if CENTER is also empty (truly no more artifacts)
            if (!ledger.isCenterOccupied()) {
                // No more artifacts anywhere - end burst
                Dbg.d(LogGroup.FIRING, "No more artifacts to transfer or fire - ending burst");
                burstFiringActive = false;
                firingHelper.cancelFiring();
            } else {
                // CENTER still has artifact to fire - keep burst active
                Dbg.d(LogGroup.FIRING, "No artifacts to transfer, but CENTER occupied - keeping burst active");
            }
        }
    }
    
    // ========== Public API - Configuration ==========
    
    /**
     * Set motif pattern for shot planning.
     * 
     * @param pattern "PPG", "PGP", or "GPP"
     */
    public void setMotifPattern(String pattern) {
        shotPlanner.setMotifPattern(pattern);
    }
    
    /**
     * Set hunt mode (controls auto-collection).
     * 
     * Hunt Mode ON: Empty intakes run rollers, poll sensors, auto-collect artifacts.
     * Hunt Mode OFF (Sleep): Empty intakes stop rollers and sensors, no auto-collect.
     *                        Storage intakes (with artifacts) still maintain hold power.
     *                        All other operations (fire, transfer, swap, eject) still work.
     * 
     * @param enabled true for ON (active), false for OFF (sleep)
     */
    public void setHuntEnabled(boolean enabled) {
        this.huntEnabled = enabled;
        telemetry.addData("Hunt Mode", enabled ? "🔍 ON (Active)" : "💤 OFF (Sleep)");
    }
    
    /**
     * Toggle hunt mode between ON and OFF.
     * 
     * @return new hunt mode state (true = ON, false = OFF)
     */
    public boolean toggleHuntEnabled() {
        huntEnabled = !huntEnabled;
        telemetry.addData("Hunt Mode", huntEnabled ? "🔍 ON (Active)" : "💤 OFF (Sleep)");
        return huntEnabled;
    }
    
    /**
     * Check if hunt mode is enabled.
     * 
     * @return true if hunt mode ON (active), false if OFF (sleep)
     */
    public boolean isHuntEnabled() {
        return huntEnabled;
    }
    
    /**
     * Set skip color detection mode (fast collection mode).
     * 
     * When enabled: Artifacts collected immediately as UNKNOWN after presence detection.
     *               Faster collection (~200ms vs ~1400ms) but no color classification.
     * 
     * When disabled: Full color detection with sampling, jiggling if needed.
     *                Slower but provides accurate color classification.
     * 
     * Default: ON (enabled) for fastest collection speed.
     * 
     * @param enabled true to skip color detection (fast mode), false for full color detection
     */
    public void setSkipColorDetection(boolean enabled) {
        this.skipColorDetection = enabled;
        Dbg.i(LogGroup.INTAKE, "Skip color detection: %s", enabled ? "ON (fast mode)" : "OFF (full detection)");
        telemetry.addData("Fast Collect Mode", enabled ? "⚡ ON (Skip Color)" : "🎨 OFF (Detect Color)");
    }
    
    /**
     * Toggle skip color detection mode.
     * 
     * @return new skip mode state (true = skip ON/fast mode, false = skip OFF/full detection)
     */
    public boolean toggleSkipColorDetection() {
        skipColorDetection = !skipColorDetection;
        Dbg.i(LogGroup.INTAKE, "Skip color detection toggled: %s", skipColorDetection ? "ON (fast)" : "OFF (full)");
        telemetry.addData("Fast Collect Mode", skipColorDetection ? "⚡ ON (Skip Color)" : "🎨 OFF (Detect Color)");
        return skipColorDetection;
    }
    
    /**
     * Check if skip color detection mode is enabled.
     * 
     * @return true if skip mode ON (fast collection), false if OFF (full color detection)
     */
    public boolean isSkipColorDetection() {
        return skipColorDetection;
    }
    
    /**
     * Set auto-swap mode (for skip mode).
     *
     * When enabled + skip mode ON: 2nd artifact collection automatically swaps with 1st artifact.
     *   - 1st artifact → CENTER
     *   - 2nd artifact collected → automatic swap → 2nd in CENTER, 1st in opposite intake
     *   - 3rd artifact → stays in intake (storage mode)
     *
     * When disabled: 2nd artifact stays in its intake (normal V3 behavior).
     *   - 1st artifact → CENTER
     *   - 2nd artifact collected → stays in intake
     *   - 3rd artifact → stays in intake (storage mode)
     *
     * NOTE: Auto-swap only works in skip mode (skipColorDetection = true).
     *       In normal mode, shot planner controls swaps based on motif patterns.
     *
     * Default: ON (enabled) for rapid collection workflow.
     *
     * @param enabled true to enable auto-swap, false to disable
     */
    public void setAutoSwapEnabled(boolean enabled) {
        this.autoSwapEnabled = enabled;
        Dbg.i(LogGroup.INTAKE, "Auto-swap: %s", enabled ? "ON (auto-swap 2nd artifact)" : "OFF (store 2nd)");
        telemetry.addData("Auto-Swap", enabled ? "✓ ON" : "✗ OFF");
    }

    /**
     * Check if auto-swap mode is enabled.
     *
     * @return true if auto-swap ON, false if OFF
     */
    public boolean isAutoSwapEnabled() {
        return autoSwapEnabled;
    }

    /**
     * Enable color sampling at current checkpoint.
     * Used by operations to control when color is read.
     */
    public void enableColorSampling(SlotLedger.Slot slot) {
        if (slot == SlotLedger.Slot.FRONT) {
            frontPerception.enableColorSampling();
        } else if (slot == SlotLedger.Slot.BACK) {
            backPerception.enableColorSampling();
        }
    }
    
    /**
     * Disable color sampling.
     */
    public void disableColorSampling(SlotLedger.Slot slot) {
        if (slot == SlotLedger.Slot.FRONT) {
            frontPerception.disableColorSampling();
        } else if (slot == SlotLedger.Slot.BACK) {
            backPerception.disableColorSampling();
        }
    }
    
    /**
     * Manually inject an artifact into a slot (for testing/operator override).
     * 
     * This bypasses normal sensor detection and creates a "phantom" artifact that
     * the system will collect. It forces the perception system to detect an artifact
     * of the specified color, then initiates a collection operation.
     * 
     * Use cases:
     * - Testing shot planning without physical artifacts
     * - Operator override when sensors malfunction
     * - Debugging collection/transfer logic
     * - Simulating various artifact configurations
     * 
     * Safety checks:
     * - Target slot must be empty
     * - System must not be full (< 3 artifacts)
     * - No operation must be running
     * - Cannot inject into CENTER (only FRONT or BACK)
     * 
     * @param slot Which slot to inject artifact into (FRONT or BACK only)
     * @param color Color of the artifact (PURPLE or GREEN recommended, not UNKNOWN)
     * @return true if successfully queued, false if rejected
     */
    public boolean addManualArtifact(SlotLedger.Slot slot, ArtifactIdentity.ColorClass color) {
        // Validate slot (can only manually add to intake slots)
        if (slot == SlotLedger.Slot.CENTER) {
            telemetry.addData("❌ Manual Add", "Cannot add directly to CENTER");
            return false;
        }
        
        // Check if slot already occupied
        if (ledger.isOccupied(slot)) {
            telemetry.addData("❌ Manual Add", slot + " already occupied");
            return false;
        }
        
        // Check if system is full
        if (ledger.isFull()) {
            telemetry.addData("❌ Manual Add", "System full (3/3)");
            return false;
        }
        
        // Check if operation is running
        if (runner.isBusy()) {
            telemetry.addData("❌ Manual Add", "Operation in progress");
            return false;
        }
        
        // Get perception for this slot
        IntakePerception perception = (slot == SlotLedger.Slot.FRONT) ? frontPerception : backPerception;
        
        // Force perception to report artifact with operator-specified color
        perception.forceDetection(color);
        
        // Now request collect (will use forced detection)
        boolean success = requestCollect(slot);
        
        if (success) {
            telemetry.addData("✅ Manual Add", color + " → " + slot + " (queued)");
        } else {
            // Failed to queue - clear forced detection
            perception.clearForcedDetection();
            telemetry.addData("❌ Manual Add", "Failed to queue collection");
        }
        
        return success;
    }
    
    // ========== Public API - State Queries ==========
    
    public boolean isEnabled() { return enabled; }
    public boolean isBusy() { return runner.isBusy(); }
    public boolean isOperationActive() { return runner.isBusy(); }
    public String getCurrentOperationName() { return runner.getCurrentOperationName(); }
    public boolean isManualMode() { return manualModeActive; }
    public boolean isBurstFiring() { return burstFiringActive; }
    public SystemState getCurrentState() { return currentState; }
    public SlotLedger getLedger() { return ledger; }
    public int getArtifactCount() { return ledger.getArtifactCount(); }
    public boolean hasArtifactInCenter() { return ledger.isCenterOccupied(); }
    public boolean isReadyToFire() { return currentState == SystemState.READY_TO_FIRE; }
    
    // Multiple artifact detection (per intake)
    /**
     * Check if FRONT intake has multiple artifacts of different colors.
     * This is a RELIABLE detection based on opposite color sensor readings.
     *
     * @return true if front intake has two artifacts of different colors
     */
    public boolean frontIntakeHasMultipleDifferentColors() {
        return frontPerception.hasMultipleDifferentColors();
    }

    /**
     * Check if BACK intake has multiple artifacts of different colors.
     * This is a RELIABLE detection based on opposite color sensor readings.
     *
     * @return true if back intake has two artifacts of different colors
     */
    public boolean backIntakeHasMultipleDifferentColors() {
        return backPerception.hasMultipleDifferentColors();
    }

    /**
     * Check if FRONT intake has multiple artifacts of same color.
     * This is LESS RELIABLE and should be used with caution.
     *
     * @return true if front intake likely has two artifacts of same color
     */
    public boolean frontIntakeHasMultipleSameColor() {
        return frontPerception.hasMultipleSameColor();
    }

    /**
     * Check if BACK intake has multiple artifacts of same color.
     * This is LESS RELIABLE and should be used with caution.
     *
     * @return true if back intake likely has two artifacts of same color
     */
    public boolean backIntakeHasMultipleSameColor() {
        return backPerception.hasMultipleSameColor();
    }

    /**
     * Check if FRONT intake has any multiple artifacts (different OR same color).
     *
     * @return true if front intake has multiple artifacts detected
     */
    public boolean frontIntakeHasMultipleArtifacts() {
        return frontPerception.hasMultipleArtifacts();
    }

    /**
     * Check if BACK intake has any multiple artifacts (different OR same color).
     *
     * @return true if back intake has multiple artifacts detected
     */
    public boolean backIntakeHasMultipleArtifacts() {
        return backPerception.hasMultipleArtifacts();
    }

    /**
     * Check if ANY intake has multiple artifacts.
     * Useful for general alerting or intake jam detection.
     *
     * @return true if either front or back intake has multiple artifacts
     */
    public boolean anyIntakeHasMultipleArtifacts() {
        return frontPerception.hasMultipleArtifacts() || backPerception.hasMultipleArtifacts();
    }

    /**
     * Get which intake(s) have multiple artifacts.
     *
     * @return String description of which intakes have multiple artifacts, or "NONE"
     */
    public String getMultipleArtifactStatus() {
        boolean frontMultiple = frontPerception.hasMultipleArtifacts();
        boolean backMultiple = backPerception.hasMultipleArtifacts();

        if (!frontMultiple && !backMultiple) {
            return "NONE";
        } else if (frontMultiple && backMultiple) {
            return "BOTH";
        } else if (frontMultiple) {
            return "FRONT";
        } else {
            return "BACK";
        }
    }

    // Presence Confidence Scoring
    /**
     * Get FRONT intake presence confidence score (0.0-1.0).
     * Numerical representation of presence confidence (more granular than enum).
     *
     * Score Weighting:
     * - Confirmation sensor: 0.15
     * - Left proximity: 0.25
     * - Right proximity: 0.25
     * - Left color: 0.10
     * - Right color: 0.10
     *
     * @return Front intake confidence score 0.0-1.0
     */
    public double getFrontIntakeConfidenceScore() {
        return frontPerception.getConfidenceScore();
    }

    /**
     * Get BACK intake presence confidence score (0.0-1.0).
     * Numerical representation of presence confidence (more granular than enum).
     *
     * @return Back intake confidence score 0.0-1.0
     */
    public double getBackIntakeConfidenceScore() {
        return backPerception.getConfidenceScore();
    }

    /**
     * Get confidence score for specified intake.
     *
     * @param slot FRONT or BACK (CENTER not supported)
     * @return Confidence score 0.0-1.0, or 0.0 if slot is CENTER
     */
    public double getIntakeConfidenceScore(SlotLedger.Slot slot) {
        switch (slot) {
            case FRONT:
                return frontPerception.getConfidenceScore();
            case BACK:
                return backPerception.getConfidenceScore();
            default:
                return 0.0;  // CENTER or invalid
        }
    }

    /**
     * Get highest confidence score across both intakes.
     * Useful for determining system-wide artifact presence strength.
     *
     * @return Maximum confidence score from either intake (0.0-1.0)
     */
    public double getMaxIntakeConfidenceScore() {
        return Math.max(frontPerception.getConfidenceScore(), backPerception.getConfidenceScore());
    }

    /**
     * Get average confidence score across both intakes.
     * Useful for overall system assessment.
     *
     * @return Average confidence score (0.0-1.0)
     */
    public double getAverageIntakeConfidenceScore() {
        return (frontPerception.getConfidenceScore() + backPerception.getConfidenceScore()) / 2.0;
    }

    // Statistics
    public int getTotalCollections() { return totalCollections; }
    public int getTotalTransfers() { return totalTransfers; }
    public int getTotalSwaps() { return totalSwaps; }
    public int getTotalShots() { return totalShots; }
    public int getTotalEjections() { return totalEjections; }
    
    // Telemetry page control (Issue 1)
    public int getTelemetryPage() { return telemetryPage; }
    public void nextTelemetryPage() { 
        telemetryPage = (telemetryPage + 1) % 3;  // Cycle 0->1->2->0
    }
    
    // Performance monitoring control
    public PerformanceMonitor getPerformanceMonitor() { return performanceMonitor; }
    public void setPerformanceMonitoringEnabled(boolean enabled) { 
        performanceMonitor.enable(enabled); 
    }
    public boolean isPerformanceMonitoringEnabled() { 
        return performanceMonitor.isEnabled(); 
    }
    
    // ========== Public API - State Setters (OpMode-Controlled) ==========
    
    /**
     * Set manual mode state (OpMode-controlled).
     * 
     * OpModes should call this method to indicate when manual controls are active.
     * When manual mode is activated, burst firing is automatically cancelled.
     * 
     * @param active true if manual controls are being used, false if auto mode
     */
    public void setManualModeActive(boolean active) {
        boolean wasActive = manualModeActive;
        manualModeActive = active;
        
        // Cancel burst firing if manual override just activated
        if (active && !wasActive) {
            if (burstFiringActive) {
                firingHelper.cancelFiring();
                burstFiringActive = false;
                telemetry.addData("⚠️ MANUAL OVERRIDE", "Burst cancelled");
            }
        }
        
        // Log state change
        if (active != wasActive) {
            telemetry.addData("Manual Mode", active ? "⚠️ ACTIVE" : "Auto");
        }
    }
    
    /**
     * Set watchdog trigger state (OpMode-controlled).
     * 
     * OpModes should call this method to indicate when the fire trigger is pressed.
     * This allows the watchdog to detect trigger release and auto-cancel burst firing.
     * 
     * @param triggerPressed true if fire trigger is pressed, false if released
     */
    public void setWatchdogTriggerState(boolean triggerPressed) {
        setFiringButtonHeld(triggerPressed);
    }
    
    /**
     * Cancel burst firing mode.
     * 
     * Stops the shooter and ends burst firing sequence.
     * Any operations in progress (transfers) will complete normally.
     * 
     * Use this when:
     * - User releases fire button
     * - Manual override activated
     * - Emergency stop needed
     */
    public void cancelBurstFiring() {
        if (burstFiringActive) {
            firingHelper.cancelFiring();
            burstFiringActive = false;
            // Clear deferred transfer flag since we're cancelling burst mode
            // If there's a pending transfer, the auto-transfer logic will handle it
            deferredTransferNeeded = false;
            telemetry.addData("🛑 Burst Firing", "Cancelled");
            Dbg.i(LogGroup.FIRING, "Burst firing cancelled by OpMode request");
        }
    }
    
    // ========== Shooter Control API ==========
    
    /**
     * Set the firing button held state for automatic keep-alive management.
     * Call this every loop with the current button state.
     * 
     * OpModes should use this to track the firing button state.
     * BasicFiringHelper uses this to automatically stop when button is released.
     * 
     * @param held true if firing button is currently pressed, false otherwise
     */
    public void setFiringButtonHeld(boolean held) {
        firingHelper.setButtonHeld(held);
    }
    
    /**
     * Check if ready to fire the next shot (shooter spun up and waiting).
     * Use this after starting firing to know when you can fire the next shot.
     * 
     * IMPORTANT: You should also check isOperationRunning() to ensure any transfers
     * have completed before calling fireNextShot(). Otherwise you may fire before
     * the artifact is physically loaded into CENTER.
     * 
     * @return true if in READY_TO_FIRE state, false otherwise
     */
    public boolean isReadyForNextShot() {
        return firingHelper.isReadyForNextShot();
    }
    
    /**
     * Check if an operation is currently running (collect, transfer, swap, fire).
     * Use this to determine if it's safe to start a new operation.
     * 
     * For burst firing, check this before calling fireNextShot() to ensure
     * any transfer operations have completed and the artifact is physically in CENTER.
     * 
     * CRITICAL: This checks BOTH the operation runner AND the physical hardware state.
     * The runner can finish while hardware (timed movements in BasicIndexingHelper) is still active.
     * 
     * @return true if operation running OR hardware still active, false if completely idle
     */
    public boolean isOperationRunning() {
        return runner.isBusy() || indexingHelper.isTransferActive();
    }
    
    /**
     * Fire the next shot (only works when in READY_TO_FIRE state).
     * Call this when isReadyForNextShot() returns true and you have an artifact ready.
     * 
     * This is used for subsequent shots after the first one in keep-alive mode.
     * 
     * CRITICAL: This fires directly via FiringHelper without creating a FireOperation.
     * The ledger is updated asynchronously when we detect the shot fired (via shot count).
     * This avoids race conditions where FireOperation's doCommit() would clear CENTER
     * before the artifact is even loaded from the transfer.
     * 
     * @return true if shot started, false if not ready
     */
    public boolean fireNextShot() {
        // Fire directly via FiringHelper (don't create FireOperation)
        // Ledger will be updated in checkForSubsequentShotFired() when shot actually fires
        return firingHelper.fireShot();
    }
    
    /**
     * Check if shooter is ready to fire (at target RPM and stable).
     * 
     * @return true if shooter ready, false otherwise
     */
    public boolean isShooterReady() {
        return firingHelper.isShooterReady();
    }
    
    /**
     * Get current shooter RPM.
     * 
     * @return current RPM
     */
    public double getShooterCurrentRPM() {
        return firingHelper.getCurrentRPM();
    }
    
    /**
     * Get target shooter RPM.
     * 
     * @return target RPM
     */
    public double getShooterTargetRPM() {
        return firingHelper.getTargetRPM();
    }
    
    // ========== Telemetry ==========
    
    /**
     * Add telemetry display with paged output.
     * Provides detailed information about system state, sensors, operations, etc.
     * 
     * Usage in OpMode:
     * - Call indexing.addTelemetry() at end of loop
     * - Telemetry from IndexingSystemV3 will appear BEFORE OpMode telemetry (display order)
     */
    public void addTelemetry() {
        // Issue 1 Fix: Implement actual page switching
        switch (telemetryPage) {
            case 0:
                addTelemetryPage1();
                break;
            case 1:
                addTelemetryPage2();
                break;
            case 2:
                addTelemetryPage3();
                break;
        }
    }
    
    /**
     * Telemetry Page 1: Overview - System state, slot ledger, operations, shot planning
     */
    private void addTelemetryPage1() {
        telemetry.addLine("========== INDEXING V3 (Page 1/3) ==========");
        telemetry.addData("State", currentState);
        telemetry.addData("Enabled", enabled ? "✓" : "✗");
        telemetry.addData("Hunt Mode", huntEnabled ? "🔍 ON" : "💤 OFF");

        // Show hunt mode transfer servo power (adaptive based on artifact count)
        if (huntEnabled) {
            int artifactCount = ledger.getArtifactCount();
            String transferPowerInfo;
            if (artifactCount == 0) {
                transferPowerInfo = "Full (-0.35)";
            } else {
                transferPowerInfo = "Reduced (-0.2) [" + artifactCount + " artifact" + (artifactCount > 1 ? "s" : "") + "]";
            }
            telemetry.addData("Hunt Power", transferPowerInfo);
        }

        telemetry.addData("Manual Mode", manualModeActive ? "⚠️ YES" : "No");
        telemetry.addData("Burst Firing", burstFiringActive ? "🔥 YES (" + consecutiveShotsFired + ")" : "No");

        // Show auto-swap status (only relevant in skip mode)
        if (skipColorDetection) {
            telemetry.addData("Auto-Swap", autoSwapEnabled ? "✓ ON" : "✗ OFF");
        }

        telemetry.addData("Last Fired", lastFiredArtifact != null ?
            lastFiredArtifact.getColorClass() + " " + String.format("%.0f%%", lastFiredArtifact.getColorConfidence() * 100) : "None");
        telemetry.addLine();
        
        // Slot ledger
        telemetry.addLine("--- Slot Ledger ("+ledger.getArtifactCount()+"/3) ---");
        telemetry.addData("CENTER", ledger.isCenterOccupied() ? 
            ledger.getCenter().getColorClass() + " " + String.format("%.0f%%", ledger.getCenter().getColorConfidence() * 100) : "EMPTY");
        telemetry.addData("FRONT", ledger.isFrontOccupied() ? 
            ledger.getFront().getColorClass() + " " + String.format("%.0f%%", ledger.getFront().getColorConfidence() * 100) : "EMPTY");
        telemetry.addData("BACK", ledger.isBackOccupied() ? 
            ledger.getBack().getColorClass() + " " + String.format("%.0f%%", ledger.getBack().getColorConfidence() * 100) : "EMPTY");
        telemetry.addLine();
        
        // Operations
        telemetry.addLine("--- Current Operation ---");
        if (runner.isBusy()) {
            telemetry.addData("Op", runner.getCurrentOperationName());
            telemetry.addData("Status", runner.getCurrentOperation().getStatusMessage());
        } else {
            telemetry.addData("Op", "IDLE");
        }
        telemetry.addData("Total Ops", runner.getOperationCount());
        
        // Shot planning
        if (shotPlanner.isRearrangementNeeded()) {
            telemetry.addLine();
            telemetry.addData("⚠️ REARRANGE", shotPlanner.getRearrangementSlot() + " ↔ CENTER");
        }
        telemetry.addData("Shot Plan", shotPlanner.getShotPlanString());
        telemetry.addLine();
    }
    
    /**
     * Telemetry Page 2: Sensors & Perception - Intake status, sensor data, hardware state
     */
    private void addTelemetryPage2() {
        telemetry.addLine("========== SENSORS (Page 2/3) ==========");
        
        // Front intake perception
        telemetry.addLine("--- FRONT Intake ---");
        telemetry.addData("Hunt Eligible", isIntakeHuntEligible(SlotLedger.Slot.FRONT) ? "✓ YES" : "✗ No");
        telemetry.addData("Fast Presence", frontPerception.getFastPresence() ? "✓ DETECTED" : "✗ Empty");
        telemetry.addData("Stable Presence", frontPerception.getStablePresence() ? "✓ DETECTED" : "✗ Empty");
        telemetry.addData("Confidence", frontPerception.getPresenceConfidence());
        telemetry.addData("Best Color", frontPerception.getBestColorClass() + " (" + 
            String.format("%.0f%%", frontPerception.getBestColorConfidence() * 100) + ")");
        telemetry.addData("Roller Busy", indexingHelper.isFrontRollerBusy() ? "✓ YES" : "No");
        telemetry.addData("Transfer Busy", indexingHelper.isFrontTransferBusy() ? "✓ YES" : "No");
        
        // Raw sensor hints (updated for new sensor layout)
        telemetry.addData("Raw: Confirmation", frontPerception.isConfirmationDetected() ? "✓" : "✗");
        telemetry.addData("Raw: PrimaryProx", frontPerception.isPrimaryProximityDetected() ? "✓" : "✗");
        telemetry.addData("Raw: SecondaryProx", frontPerception.isSecondaryProximityDetected() ? "✓" : "✗");
        telemetry.addData("Raw: PrimaryColor", frontPerception.colorSeesArtifact_Primary() ? "✓" : "✗");
        telemetry.addData("Raw: SecondaryColor", frontPerception.colorSeesArtifact_Secondary() ? "✓" : "✗");
        telemetry.addLine();
        
        // Back intake perception
        telemetry.addLine("--- BACK Intake ---");
        telemetry.addData("Hunt Eligible", isIntakeHuntEligible(SlotLedger.Slot.BACK) ? "✓ YES" : "✗ No");
        telemetry.addData("Fast Presence", backPerception.getFastPresence() ? "✓ DETECTED" : "✗ Empty");
        telemetry.addData("Stable Presence", backPerception.getStablePresence() ? "✓ DETECTED" : "✗ Empty");
        telemetry.addData("Confidence", backPerception.getPresenceConfidence());
        telemetry.addData("Best Color", backPerception.getBestColorClass() + " (" + 
            String.format("%.0f%%", backPerception.getBestColorConfidence() * 100) + ")");
        telemetry.addData("Roller Busy", indexingHelper.isBackRollerBusy() ? "✓ YES" : "No");
        telemetry.addData("Transfer Busy", indexingHelper.isBackTransferBusy() ? "✓ YES" : "No");
        
        // Raw sensor hints (updated for new sensor layout)
        telemetry.addData("Raw: Confirmation", backPerception.isConfirmationDetected() ? "✓" : "✗");
        telemetry.addData("Raw: PrimaryProx", backPerception.isPrimaryProximityDetected() ? "✓" : "✗");
        telemetry.addData("Raw: SecondaryProx", backPerception.isSecondaryProximityDetected() ? "✓" : "✗");
        telemetry.addData("Raw: PrimaryColor", backPerception.colorSeesArtifact_Primary() ? "✓" : "✗");
        telemetry.addData("Raw: SecondaryColor", backPerception.colorSeesArtifact_Secondary() ? "✓" : "✗");
        telemetry.addLine();
    }
    
    /**
     * Telemetry Page 3: Statistics & Watchdog - Counters, operation history, safety status
     */
    private void addTelemetryPage3() {
        telemetry.addLine("========== STATS (Page 3/3) ==========");
        telemetry.addData("Collections", totalCollections);
        telemetry.addData("Transfers", totalTransfers);
        telemetry.addData("Swaps", totalSwaps);
        telemetry.addData("Shots Fired", totalShots);
        telemetry.addData("Ejections", totalEjections);
        telemetry.addLine();
        
        // Performance monitoring (if enabled)
        if (performanceMonitor.isEnabled()) {
            performanceMonitor.addTelemetry();
            telemetry.addLine();
        }
        
        // Watchdog status
        telemetry.addLine("--- KeepAlive Watchdog ---");
        watchdog.addTelemetry();
    }
}
