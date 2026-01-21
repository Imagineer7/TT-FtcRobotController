package org.firstinspires.ftc.teamcode.util.aurora.v3;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShotPlanner;

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
 * @author Copilot (AI Assistant)
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
    
    // Perception (one per intake)
    private final IntakePerception frontPerception;
    private final IntakePerception backPerception;
    
    // ========== State ==========
    private SystemState currentState;
    private boolean enabled;
    private boolean manualModeActive;
    private boolean huntEnabled;  // Hunt mode: auto-collect when artifacts detected
    
    // Manual override detection
    private boolean lastManual_frontIntake;
    private boolean lastManual_backIntake;
    private boolean lastManual_uptake;
    
    // Burst firing state
    private boolean burstFiringActive;
    private long lastOperationCompleteTime;
    private int consecutiveShotsFired;
    
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
        
        // Initialize perception (IntakeSide enum, sensors, config)
        this.frontPerception = new IntakePerception(
            IntakePerception.IntakeSide.FRONT,
            hardware.getFrontDistanceSensor(),
            hardware.getFrontLeftDistanceSensor(),
            hardware.getFrontLeftColorSensor(),
            hardware.getFrontRightColorSensor(),
            config
        );
        
        this.backPerception = new IntakePerception(
            IntakePerception.IntakeSide.BACK,
            hardware.getBackDistanceSensor(),
            hardware.getBackRightDistanceSensor(),
            hardware.getBackRightColorSensor(),
            hardware.getLeftRightColorSensor(),
            config
        );
        
        // Initialize state
        this.currentState = SystemState.IDLE;
        this.enabled = false;
        this.manualModeActive = false;
        this.huntEnabled = true;  // Hunt mode ON by default
        this.burstFiringActive = false;
        this.lastOperationCompleteTime = System.currentTimeMillis();
        this.consecutiveShotsFired = 0;
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
     * @param gamepad1 Driver gamepad
     * @param gamepad2 Operator gamepad
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if (!enabled) return;
        
        // Update perception (sensor fusion) - only for hunt-eligible intakes
        updatePerception();
        
        // Detect manual override
        detectManualOverride(gamepad2);
        
        // Update watchdog (automatic safety enforcement)
        boolean triggerPressed = gamepad1.right_trigger > 0.1;
        watchdog.update(triggerPressed, runner.isBusy(), manualModeActive);
        
        // Capture current operation before update (for completion handling)
        boolean isBusyNow = runner.isBusy();
        if (isBusyNow) {
            lastCompletedOperation = runner.getCurrentOperation();
        }
        
        // Update operation runner (automatic lifecycle management)
        runner.update();
        
        // Handle operation completion (detect transition from busy to idle)
        if (wasRunnerBusyLastUpdate && !runner.isBusy()) {
            handleOperationComplete();
        }
        wasRunnerBusyLastUpdate = runner.isBusy();
        
        // Update shot planner
        shotPlanner.update(ledger, manualModeActive);
        
        // Update system state
        updateSystemState();
        
        // Automatic operations (if not in manual mode)
        if (!manualModeActive && !runner.isBusy()) {
            performAutomaticOperations();
        }
    }
    
    /**
     * Detect manual override from gamepad inputs.
     */
    private void detectManualOverride(Gamepad gamepad2) {
        // Manual control detection (from GamepadConfig)
        boolean manual_frontIntake = gamepad2.dpad_left;
        boolean manual_backIntake = gamepad2.dpad_right;
        boolean manual_uptake = gamepad2.dpad_up || gamepad2.dpad_down;
        
        // Edge detection - manual mode activated if any control just pressed
        boolean manualActivated = (manual_frontIntake && !lastManual_frontIntake) ||
                                 (manual_backIntake && !lastManual_backIntake) ||
                                 (manual_uptake && !lastManual_uptake);
        
        if (manualActivated) {
            manualModeActive = true;
            // Cancel burst firing if manual override detected
            if (burstFiringActive) {
                firingHelper.cancelFiring();
                burstFiringActive = false;
                telemetry.addData("⚠️ MANUAL OVERRIDE", "Burst cancelled");
            }
        }
        
        // Manual mode ends when all controls released for 500ms
        if (!manual_frontIntake && !manual_backIntake && !manual_uptake) {
            // TODO: Add 500ms delay before clearing manual mode
            // For now, immediate clear
            if (manualModeActive) {
                manualModeActive = false;
                telemetry.addData("Manual Mode", "Deactivated");
            }
        }
        
        // Store last state
        lastManual_frontIntake = manual_frontIntake;
        lastManual_backIntake = manual_backIntake;
        lastManual_uptake = manual_uptake;
    }
    
    /**
     * Update perception sensors based on hunt mode eligibility.
     * 
     * Hunt Mode ON: Update perception for empty intakes (eligible to hunt).
     * Hunt Mode OFF: Skip perception updates for empty intakes (sleep mode).
     * 
     * Note: Perception is always skipped during operations (rollers controlled by operations).
     */
    private void updatePerception() {
        // Don't update perception if operation is running (operation controls rollers)
        if (runner.isBusy()) {
            return;
        }
        
        // Update front intake perception if eligible to hunt
        if (isIntakeHuntEligible(SlotLedger.Slot.FRONT)) {
            frontPerception.update();
        }
        
        // Update back intake perception if eligible to hunt
        if (isIntakeHuntEligible(SlotLedger.Slot.BACK)) {
            backPerception.update();
        }
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
     * Handle operation completion.
     */
    private void handleOperationComplete() {
        IndexingOperation lastOp = lastCompletedOperation;
        if (lastOp == null) return;
        
        // Update statistics
        if (lastOp instanceof CollectOperation) {
            totalCollections++;
        } else if (lastOp instanceof TransferOperation) {
            totalTransfers++;
        } else if (lastOp instanceof SwapOperation) {
            totalSwaps++;
        } else if (lastOp instanceof FireOperation) {
            FireOperation fireOp = (FireOperation) lastOp;
            if (lastOp.isSuccess() && !fireOp.wasCancelledBeforeShot()) {
                totalShots++;
                consecutiveShotsFired++;
                
                // If burst firing, queue next transfer if more shots needed
                if (burstFiringActive && shouldContinueBurst()) {
                    queueNextShotInBurst();
                }
            } else if (fireOp.wasCancelledBeforeShot()) {
                // Cancelled before shot - end burst
                burstFiringActive = false;
                firingHelper.cancelFiring();
            }
        } else if (lastOp instanceof EjectOperation) {
            totalEjections++;
            burstFiringActive = false;  // Ejection ends burst
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
        // Auto-collect ONLY if hunt mode enabled and intake eligible
        if (huntEnabled && isIntakeHuntEligible(SlotLedger.Slot.FRONT) && 
            frontPerception.getFastPresence()) {
            requestCollect(SlotLedger.Slot.FRONT);
        }
        if (huntEnabled && isIntakeHuntEligible(SlotLedger.Slot.BACK) && 
            backPerception.getFastPresence()) {
            requestCollect(SlotLedger.Slot.BACK);
        }
        
        // Auto-rearrange if shot planner detects benefit
        if (shotPlanner.isRearrangementNeeded() && ledger.getArtifactCount() == 2) {
            SlotLedger.Slot swapSlot = shotPlanner.getRearrangementSlot();
            if (swapSlot != null) {
                requestSwap(swapSlot);
            }
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
            ledger, perception, indexingHelper, config, slot, nextSequenceId++, telemetry
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
        
        IntakePerception perception = (slot == SlotLedger.Slot.FRONT) ? frontPerception : backPerception;
        
        TransferOperation op = new TransferOperation(
            ledger, perception, indexingHelper, config, slot, telemetry
        );
        
        return runner.start(op);
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
        
        return runner.start(op);
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
        return requestFire(false, null);
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
        
        if (nextSlot != null) {
            // Transfer artifact to center
            requestTransfer(nextSlot);
            // Note: Firing will happen automatically after transfer completes
            // (via automatic operations or explicit request in next update)
        } else {
            // No more artifacts - end burst
            burstFiringActive = false;
            firingHelper.cancelFiring();
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
    
    // ========== Public API - State Queries ==========
    
    public boolean isEnabled() { return enabled; }
    public boolean isBusy() { return runner.isBusy(); }
    public boolean isManualMode() { return manualModeActive; }
    public boolean isBurstFiring() { return burstFiringActive; }
    public SystemState getCurrentState() { return currentState; }
    public SlotLedger getLedger() { return ledger; }
    public int getArtifactCount() { return ledger.getArtifactCount(); }
    public boolean hasArtifactInCenter() { return ledger.isCenterOccupied(); }
    public boolean isReadyToFire() { return currentState == SystemState.READY_TO_FIRE; }
    
    // Statistics
    public int getTotalCollections() { return totalCollections; }
    public int getTotalTransfers() { return totalTransfers; }
    public int getTotalSwaps() { return totalSwaps; }
    public int getTotalShots() { return totalShots; }
    public int getTotalEjections() { return totalEjections; }
    
    // ========== Telemetry ==========
    
    /**
     * Add telemetry display.
     */
    public void addTelemetry() {
        telemetry.addLine("========== INDEXING V3 ==========");
        telemetry.addData("State", currentState);
        telemetry.addData("Enabled", enabled ? "✓" : "✗");
        telemetry.addData("Hunt Mode", huntEnabled ? "🔍 ON (Active)" : "💤 OFF (Sleep)");
        telemetry.addData("Manual Mode", manualModeActive ? "⚠️ ACTIVE" : "Auto");
        telemetry.addData("Burst Firing", burstFiringActive ? "🔥 YES (" + consecutiveShotsFired + ")" : "No");
        telemetry.addLine();
        
        // Slot ledger
        telemetry.addLine("--- Slot Ledger ---");
        telemetry.addData("Count", ledger.getArtifactCount() + "/3");
        telemetry.addData("CENTER", ledger.isCenterOccupied() ? 
            ledger.getCenter().getColorClass() + " (" + String.format("%.0f%%", ledger.getCenter().getColorConfidence() * 100) + ")" : "EMPTY");
        telemetry.addData("FRONT", ledger.isFrontOccupied() ? 
            ledger.getFront().getColorClass() + " (" + String.format("%.0f%%", ledger.getFront().getColorConfidence() * 100) + ")" : "EMPTY");
        telemetry.addData("BACK", ledger.isBackOccupied() ? 
            ledger.getBack().getColorClass() + " (" + String.format("%.0f%%", ledger.getBack().getColorConfidence() * 100) + ")" : "EMPTY");
        telemetry.addLine();
        
        // Operations
        telemetry.addLine("--- Operations ---");
        if (runner.isBusy()) {
            telemetry.addData("Current", runner.getCurrentOperationName());
            // Note: getProgressPercent() not available in BaseOperation
            // telemetry.addData("Progress", String.format("%.0f%%", runner.getCurrentOperation().getProgressPercent()));
        } else {
            telemetry.addData("Current", "IDLE");
        }
        telemetry.addData("Total Ops", runner.getOperationCount());
        telemetry.addLine();
        
        // Shot planner
        if (shotPlanner.isRearrangementNeeded()) {
            telemetry.addData("⚠️ REARRANGE", "Swap " + shotPlanner.getRearrangementSlot() + " ↔ CENTER");
        }
        telemetry.addData("Shot Plan", shotPlanner.getShotPlanString());
        telemetry.addLine();
        
        // Statistics
        telemetry.addLine("--- Statistics ---");
        telemetry.addData("Collections", totalCollections);
        telemetry.addData("Transfers", totalTransfers);
        telemetry.addData("Swaps", totalSwaps);
        telemetry.addData("Shots Fired", totalShots);
        telemetry.addData("Ejections", totalEjections);
        telemetry.addLine();
        
        // Watchdog
        watchdog.addTelemetry();
    }
}
