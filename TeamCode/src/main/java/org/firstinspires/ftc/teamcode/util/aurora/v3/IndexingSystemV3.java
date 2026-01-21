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
    
    // Telemetry page switching (Issue 1)
    private int telemetryPage = 0;  // 0, 1, or 2
    
    // Auto-collect cooldown (Issue 3)
    private static final long AUTO_COLLECT_COOLDOWN_MS = 1000;  // 1 second
    private long lastFrontAutoCollectTime = 0;
    private long lastBackAutoCollectTime = 0;
    
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
     * OpModes should call this method in their loop and handle gamepad inputs separately.
     */
    public void update() {
        if (!enabled) return;
        
        // Update perception (sensor fusion) - only for hunt-eligible intakes
        updatePerception();
        
        // CRITICAL: Update indexing helper to process timed movements
        // This clears the busy flags when timed movements complete
        indexingHelper.update();
        
        // Update watchdog (automatic safety enforcement)
        // Note: OpMode must call setWatchdogTriggerState() to update trigger state
        watchdog.update(false, runner.isBusy(), manualModeActive);
        
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
        // Don't update perception if operation is running (operation controls rollers)
        if (runner.isBusy()) {
            // Stop hunt-mode hardware when operations take control
            stopHuntingTransferServos();
            stopHuntingRollers();
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
        
        // Run intake rollers for hunt-eligible intakes
        // This allows artifacts to be pulled in during hunt mode
        updateHuntingRollers();
        
        // Run transfer servos in reverse for hunt-eligible intakes
        // This creates a "jiggling" effect that rotates artifacts slightly
        // Helps prevent sensor blind spots from holes in artifacts
        updateHuntingTransferServos();
    }
    
    /**
     * Control intake rollers during hunt mode.
     * Hunt-eligible intakes run rollers at collect power.
     * Non-hunt-eligible intakes stop rollers (unless holding artifact).
     */
    private void updateHuntingRollers() {
        // Issue 2 Fix: Don't control hardware if operation is running
        if (runner.isBusy()) {
            return;  // Operations have exclusive control
        }
        
        // Front intake roller control
        if (isIntakeHuntEligible(SlotLedger.Slot.FRONT)) {
            // Hunt-eligible: run roller at collect power
            indexingHelper.setFrontRollerPower(config.getIntakeRollerPower());
        } else if (ledger.isOccupied(SlotLedger.Slot.FRONT)) {
            // Storage intake: run at hold power to retain artifact
            indexingHelper.setFrontRollerPower(config.getIntakeStoragePower());
        } else {
            // Not hunt-eligible and empty: stop roller
            indexingHelper.setFrontRollerPower(0);
        }
        
        // Back intake roller control
        if (isIntakeHuntEligible(SlotLedger.Slot.BACK)) {
            // Hunt-eligible: run roller at collect power
            indexingHelper.setBackRollerPower(config.getIntakeRollerPower());
        } else if (ledger.isOccupied(SlotLedger.Slot.BACK)) {
            // Storage intake: run at hold power to retain artifact
            indexingHelper.setBackRollerPower(config.getIntakeStoragePower());
        } else {
            // Not hunt-eligible and empty: stop roller
            indexingHelper.setBackRollerPower(0);
        }
    }
    
    /**
     * Stop hunt-mode rollers.
     * Called when operations take control.
     */
    private void stopHuntingRollers() {
        // Only stop if not busy with timed movements
        if (!indexingHelper.isFrontRollerBusy()) {
            indexingHelper.setFrontRollerPower(0);
        }
        if (!indexingHelper.isBackRollerBusy()) {
            indexingHelper.setBackRollerPower(0);
        }
    }
    
    /**
     * Control transfer servos for hunt mode.
     * 
     * When hunt-eligible (waiting to collect), run transfer servos in reverse.
     * This creates a back-and-forth motion with the roller that rotates the artifact slightly,
     * helping prevent holes in the artifact from lining up with sensors (blind spots).
     * 
     * Reverse power means the servo pushes artifact away from center (eject direction).
     * Combined with forward roller motion, this creates a "jiggling" effect.
     * 
     * IMPORTANT: Only applies hunt-mode power when no timed movement is active.
     * Operations use timed movements for transfers, and we must not interfere.
     */
    private void updateHuntingTransferServos() {
        // Issue 2 Fix: Don't control hardware if operation is running
        if (runner.isBusy()) {
            return;  // Operations have exclusive control
        }
        
        // Power for reverse motion (negative = eject direction)
        // Lower value (-0.3 to -0.4) creates gentle jiggling without ejecting artifact
        final double HUNT_TRANSFER_REVERSE_POWER = 0.35;
        
        // Front intake: run transfer servo in reverse if hunt-eligible
        // BUT: Don't interfere if operation has active timed movement
        if (isIntakeHuntEligible(SlotLedger.Slot.FRONT) && !indexingHelper.isFrontTransferBusy()) {
            indexingHelper.setFrontTransferPower(HUNT_TRANSFER_REVERSE_POWER);
        } else if (!indexingHelper.isFrontTransferBusy()) {
            // Not hunt-eligible and no operation: stop transfer servo
            // (If operation is busy, let it control the servo)
            indexingHelper.setFrontTransferPower(0);
        }
        
        // Back intake: run transfer servo in reverse if hunt-eligible
        // BUT: Don't interfere if operation has active timed movement
        if (isIntakeHuntEligible(SlotLedger.Slot.BACK) && !indexingHelper.isBackTransferBusy()) {
            indexingHelper.setBackTransferPower(HUNT_TRANSFER_REVERSE_POWER);
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
     * Only stops servos if they don't have active timed movements.
     * Operations use timed movements, so we respect those.
     */
    private void stopHuntingTransferServos() {
        // Only stop if no timed movement is active
        if (!indexingHelper.isFrontTransferBusy()) {
            indexingHelper.setFrontTransferPower(0);
        }
        if (!indexingHelper.isBackTransferBusy()) {
            indexingHelper.setBackTransferPower(0);
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
        
        System.out.println("[IndexingV3] handleOperationComplete: " + lastOp.getOperationName() + 
                         ", success=" + lastOp.isSuccess());
        
        // Only process successful operations
        if (!lastOp.isSuccess()) return;
        
        // Update statistics
        if (lastOp instanceof CollectOperation) {
            totalCollections++;
            System.out.println("[IndexingV3] Collection completed, calling handlePostCollection()");
            
            // Post-Collection Logic: Automatically handle artifact placement
            // This is the CRITICAL missing piece - after collecting, we need to:
            // - 1st artifact: Transfer to center
            // - 2nd artifact: Check if swap needed (shot planning), otherwise stay in intake
            // - 3rd artifact: Stay in intake
            handlePostCollection();
            
        } else if (lastOp instanceof TransferOperation) {
            totalTransfers++;
            System.out.println("[IndexingV3] Transfer completed");
            
            // CRITICAL: Reset perception for the source intake to prevent false detections
            // After an artifact physically moves away, sensors may still show presence briefly
            // Resetting clears stale sensor data and prevents immediate false re-collection
            String opName = lastOp.getOperationName();
            if (opName.contains("FRONT")) {
                frontPerception.reset();
                System.out.println("[IndexingV3] Reset FRONT perception after transfer");
            } else if (opName.contains("BACK")) {
                backPerception.reset();
                System.out.println("[IndexingV3] Reset BACK perception after transfer");
            }
            
        } else if (lastOp instanceof SwapOperation) {
            totalSwaps++;
            System.out.println("[IndexingV3] Swap completed");
        } else if (lastOp instanceof FireOperation) {
            FireOperation fireOp = (FireOperation) lastOp;
            if (!fireOp.wasCancelledBeforeShot()) {
                totalShots++;
                consecutiveShotsFired++;
                System.out.println("[IndexingV3] Shot fired, total=" + totalShots);
                
                // If burst firing, queue next transfer if more shots needed
                if (burstFiringActive && shouldContinueBurst()) {
                    queueNextShotInBurst();
                }
            } else {
                // Cancelled before shot - end burst
                burstFiringActive = false;
                firingHelper.cancelFiring();
                System.out.println("[IndexingV3] Fire cancelled before shot, ending burst");
            }
        } else if (lastOp instanceof EjectOperation) {
            totalEjections++;
            burstFiringActive = false;  // Ejection ends burst
            System.out.println("[IndexingV3] Ejection completed");
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
        System.out.println("[IndexingV3] handlePostCollection: artifactCount=" + artifactCount);
        
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
                System.out.println("[IndexingV3] 1st artifact - requesting transfer from " + sourceSlot);
                // Queue transfer to center
                boolean success = requestTransfer(sourceSlot);
                System.out.println("[IndexingV3] Transfer request " + (success ? "SUCCESSFUL" : "FAILED"));
            } else {
                System.out.println("[IndexingV3] WARNING: 1st artifact but no occupied intake found!");
            }
            
        } else if (artifactCount == 2) {
            // SECOND ARTIFACT: Check if swap needed for optimal shot order
            System.out.println("[IndexingV3] 2nd artifact - checking shot planner...");
            // The shot planner will determine if we need to rearrange
            if (shotPlanner.isRearrangementNeeded()) {
                SlotLedger.Slot swapSlot = shotPlanner.getRearrangementSlot();
                if (swapSlot != null) {
                    System.out.println("[IndexingV3] Shot planner recommends swap with " + swapSlot);
                    // Swap needed - do it now
                    boolean success = requestSwap(swapSlot);
                    System.out.println("[IndexingV3] Swap request " + (success ? "SUCCESSFUL" : "FAILED"));
                }
            } else {
                System.out.println("[IndexingV3] No swap needed, artifact stays in storage");
            }
            // If no swap needed, artifact stays in intake (storage mode)
            // Hunt-mode will automatically run rollers at storage power
            
        } else if (artifactCount == 3) {
            // THIRD ARTIFACT: System full, stays in intake (storage mode)
            System.out.println("[IndexingV3] 3rd artifact - system full, stays in storage");
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
        
        // Auto-collect ONLY if hunt mode enabled and intake eligible
        if (huntEnabled && isIntakeHuntEligible(SlotLedger.Slot.FRONT) && 
            frontPerception.getFastPresence() &&
            (currentTime - lastFrontAutoCollectTime) >= AUTO_COLLECT_COOLDOWN_MS) {
            if (requestCollect(SlotLedger.Slot.FRONT)) {
                lastFrontAutoCollectTime = currentTime;
            }
        }
        if (huntEnabled && isIntakeHuntEligible(SlotLedger.Slot.BACK) && 
            backPerception.getFastPresence() &&
            (currentTime - lastBackAutoCollectTime) >= AUTO_COLLECT_COOLDOWN_MS) {
            if (requestCollect(SlotLedger.Slot.BACK)) {
                lastBackAutoCollectTime = currentTime;
            }
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
    
    // Telemetry page control (Issue 1)
    public int getTelemetryPage() { return telemetryPage; }
    public void nextTelemetryPage() { 
        telemetryPage = (telemetryPage + 1) % 3;  // Cycle 0->1->2->0
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
        // Update watchdog with current trigger state
        watchdog.update(triggerPressed, runner.isBusy(), manualModeActive);
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
        telemetry.addData("Manual Mode", manualModeActive ? "⚠️ YES" : "No");
        telemetry.addData("Burst Firing", burstFiringActive ? "🔥 YES (" + consecutiveShotsFired + ")" : "No");
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
        
        // Watchdog status
        telemetry.addLine("--- KeepAlive Watchdog ---");
        watchdog.addTelemetry();
    }
}
