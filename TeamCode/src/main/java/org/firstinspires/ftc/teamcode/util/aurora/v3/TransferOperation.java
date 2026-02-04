package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;

/**
 * TransferOperation - Transfer artifact from intake slot to center using sensor-based completion
 *
 * This operation uses real-time center slot detection to determine when the artifact has
 * successfully transferred, eliminating the need for fixed-duration timers.
 *
 * Transfer Phases:
 * 1. Un-pre-position (150ms) - Clear uptake area
 * 2. Transfer - Run hardware at full power until artifact detected in center
 * 3. Detection confirmation - Wait 200ms of stable detection before stopping
 * 4. Pre-position (100ms) - Position artifact for firing
 * 5. Settle delay (200ms) - Wait for artifact to settle
 * 6. Color resample (Checkpoint 2) - Update artifact color with improved reading
 * 7. Commit - Move artifact from source to center in ledger
 *
 * Sensor-Based Transfer Benefits:
 * - No wasted time (stops as soon as artifact detected)
 * - No under-transfer (continues until confirmed detection)
 * - Adaptive to different artifact speeds/weights
 * - Timeout protection (4 seconds max)
 *
 * Preconditions:
 * - Source slot occupied
 * - Center slot empty
 *
 * Hardware: Direct control via BasicIndexingHelper (rollers + transfer servos + injectors)
 */
public class TransferOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final IntakePerception.CenterSlotPerception centerPerception;
    private final BasicIndexingHelper helper;
    private final SlotLedger.Slot sourceSlot;

    private ArtifactIdentity transferredArtifact;
    private boolean hardwareComplete;

    // Sensor-based transfer detection
    private boolean centerDetectionStarted;
    private long centerDetectionStartTime;

    // Hardware state tracking
    private boolean hardwareRunning;
    private boolean prepositionStarted;  // Track if we already started pre-positioning

    // Transfer detection duration (how long artifact must be detected before stopping)
    // CRITICAL: This must be SHORT because artifacts move fast through center!
    // 50ms = ~2-3 loop cycles - enough to debounce but not so long artifact passes through
    private static final long CENTER_DETECTION_DURATION_MS = 50;

    // Transfer timeout
    private static final long TRANSFER_TIMEOUT_MS = 4000;  // 4 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new TransferOperation
     *
     * @param ledger Slot ledger to update
     * @param centerPerception Center slot perception for transfer detection
     * @param helper BasicIndexingHelper for hardware control
     * @param sourceSlot Which slot to transfer from (FRONT or BACK)
     * @param telemetry Telemetry for logging
     */
    public TransferOperation(SlotLedger ledger,
                            IntakePerception.CenterSlotPerception centerPerception,
                            BasicIndexingHelper helper,
                            SlotLedger.Slot sourceSlot,
                            Telemetry telemetry) {
        super(telemetry, TRANSFER_TIMEOUT_MS);
        this.ledger = ledger;
        this.centerPerception = centerPerception;
        this.helper = helper;
        this.sourceSlot = sourceSlot;
        this.transferredArtifact = null;
        this.hardwareComplete = false;
        this.centerDetectionStarted = false;
        this.centerDetectionStartTime = 0;
        this.hardwareRunning = false;
        this.prepositionStarted = false;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OPERATION IMPLEMENTATION
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    protected boolean doStart() {
        Dbg.i(LogGroup.TRANSFER, "TransferOperation.doStart() - source=%s", sourceSlot);

        // Check preconditions
        if (!ledger.isOccupied(sourceSlot)) {
            Dbg.w(LogGroup.TRANSFER, "Transfer REJECTED - source slot %s is EMPTY", sourceSlot);
            fail(RejectReason.SLOT_EMPTY);
            setStatusMessage("Source slot " + sourceSlot + " is empty");
            return false;
        }

        if (ledger.isCenterOccupied()) {
            Dbg.w(LogGroup.TRANSFER, "Transfer REJECTED - center slot is OCCUPIED");
            fail(RejectReason.CENTER_OCCUPIED);
            setStatusMessage("Center slot is occupied");
            return false;
        }

        // Get artifact from source slot
        transferredArtifact = ledger.get(sourceSlot);
        Dbg.d(LogGroup.TRANSFER, "Artifact to transfer: %s", transferredArtifact);

        // Start un-pre-positioning (clear uptake area)
        helper.unPrePositionArtifacts();
        Dbg.d(LogGroup.TRANSFER, "Started un-pre-positioning (150ms)");

        setStatusMessage("Un-pre-positioning for transfer from " + sourceSlot);
        return true;
    }

    @Override
    protected boolean doUpdate() {
        // Update center perception every loop
        centerPerception.update();

        // Phase 1: Wait for un-pre-positioning to complete
        if (!hardwareRunning && helper.isUptakeBusy()) {
            setStatusMessage("Un-pre-positioning...");
            return true;  // Still un-pre-positioning
        }

        // Phase 2: Run transfer hardware until artifact detected in center
        if (!hardwareComplete) {
            // Start hardware if not already running
            if (!hardwareRunning) {
                Dbg.i(LogGroup.TRANSFER, "=== STARTING TRANSFER HARDWARE ===");
                Dbg.d(LogGroup.TRANSFER, "  Source: %s", sourceSlot);

                if (sourceSlot == SlotLedger.Slot.FRONT) {
                    helper.runFrontIntake(true, 1.0);  // Full power
                    helper.setInjectorPower(1.0);
                    Dbg.d(LogGroup.TRANSFER, "  Hardware: Front intake + injector (1.0 power)");
                } else if (sourceSlot == SlotLedger.Slot.BACK) {
                    helper.runBackIntake(true, 1.0);  // Full power
                    helper.setInjectorPower(-1.0);
                    Dbg.d(LogGroup.TRANSFER, "  Hardware: Back intake + injector (-1.0 power)");
                }
                hardwareRunning = true;
                setStatusMessage("Transfer started: " + sourceSlot + " → CENTER");
            }

            // Check for artifact detection in center (STRICT: Both proximity sensors)
            boolean bothSensorsDetect = centerPerception.isBothProximitySensorsDetecting();

            // Log detection state every 100ms for visibility
            long now = System.currentTimeMillis();
            if (now % 100 < 20) {  // Log approximately every 100ms
                Dbg.d(LogGroup.TRANSFER, "Detection check: bothSensors=%s, detectionStarted=%s",
                      bothSensorsDetect, centerDetectionStarted);
            }

            if (bothSensorsDetect) {
                if (!centerDetectionStarted) {
                    // First detection - start timer
                    centerDetectionStarted = true;
                    centerDetectionStartTime = System.currentTimeMillis();
                    Dbg.i(LogGroup.TRANSFER, "✓ BOTH SENSORS DETECTING - Starting %dms confirmation timer", CENTER_DETECTION_DURATION_MS);
                    setStatusMessage("Both sensors detect - confirming...");
                } else {
                    // Check if detection has been stable for required duration
                    long detectionDuration = System.currentTimeMillis() - centerDetectionStartTime;

                    // Log progress every 20ms during confirmation (more frequent for fast detection)
                    if (detectionDuration % 20 < 10) {
                        Dbg.d(LogGroup.TRANSFER, "Confirmation progress: %dms / %dms",
                              detectionDuration, CENTER_DETECTION_DURATION_MS);
                    }

                    if (detectionDuration >= CENTER_DETECTION_DURATION_MS) {
                        // Artifact confirmed in center - STOP TRANSFER IMMEDIATELY
                        Dbg.i(LogGroup.TRANSFER, "=== STOPPING TRANSFER HARDWARE ===");
                        Dbg.i(LogGroup.TRANSFER, "  Reason: Confirmed detection (both sensors for %dms)", detectionDuration);

                        if (sourceSlot == SlotLedger.Slot.FRONT) {
                            helper.runFrontIntake(false, 0);
                            helper.setInjectorPower(0);
                            Dbg.d(LogGroup.TRANSFER, "  Stopped: Front intake + injector");
                        } else {
                            helper.runBackIntake(false, 0);
                            helper.setInjectorPower(0);
                            Dbg.d(LogGroup.TRANSFER, "  Stopped: Back intake + injector");
                        }

                        hardwareRunning = false;
                        hardwareComplete = true;
                        setStatusMessage("Transfer complete - artifact in center");
                        logInfo("Artifact detected in center after " + detectionDuration + "ms (both sensors)");
                    } else {
                        long remaining = CENTER_DETECTION_DURATION_MS - detectionDuration;
                        setStatusMessage("Confirming (" + detectionDuration + "/" + CENTER_DETECTION_DURATION_MS + "ms)");
                    }
                }
            } else {
                // Not both sensors detecting - reset timer and continue
                if (centerDetectionStarted) {
                    centerDetectionStarted = false;
                    Dbg.w(LogGroup.TRANSFER, "✗ LOST DETECTION (not both sensors) - Resetting timer, continuing transfer");
                    setStatusMessage("Lost detection (not both), continuing...");
                } else {
                    // Log periodically while waiting
                    if (now % 100 < 20) {  // Log every ~100ms while waiting
                        Dbg.d(LogGroup.TRANSFER, "Waiting for both sensors to detect...");
                    }
                    setStatusMessage("Transferring (waiting for both sensors)...");
                }
            }

            return true;  // Still transferring
        }

        // Phase 3: Pre-position artifact after hardware complete
        if (hardwareComplete && !prepositionStarted && !helper.isUptakeBusy()) {
            // Start pre-positioning ONCE
            helper.prePositionArtifacts();
            prepositionStarted = true;
            Dbg.d(LogGroup.TRANSFER, "Started pre-positioning (100ms)");
            setStatusMessage("Pre-positioning artifact");
        }

        // Phase 4: Wait for pre-positioning to complete
        if (hardwareComplete && prepositionStarted && helper.isUptakeBusy()) {
            setStatusMessage("Pre-positioning...");
            return true;
        }

        // Phase 5: Complete immediately after pre-positioning
        // Color was already detected at checkpoint 1 (during collection), no resampling needed
        if (hardwareComplete && prepositionStarted && !helper.isUptakeBusy()) {
            setStatusMessage("Transfer complete");
            return false;  // Done
        }

        return true;  // Still running (waiting for pre-positioning to complete)
    }

    @Override
    protected void doCommit() {
        Dbg.i(LogGroup.TRANSFER, "=== COMMITTING TRANSFER ===");
        Dbg.d(LogGroup.TRANSFER, "  Moving artifact: %s → CENTER", sourceSlot);
        Dbg.d(LogGroup.TRANSFER, "  Artifact: %s", transferredArtifact);

        // Move artifact from source to center
        ledger.clear(sourceSlot);
        ledger.setCenter(transferredArtifact);
        
        Dbg.i(LogGroup.TRANSFER, "Transfer committed successfully");
        Dbg.d(LogGroup.TRANSFER, "  Ledger state: %s", ledger.toSnapshot());

        logInfo("Committed transfer: " + sourceSlot + " → CENTER");
        logDebug("Artifact", transferredArtifact.toString());
        logDebug("Slot Ledger", ledger.toSnapshot());
    }

    @Override
    public String getOperationName() {
        return "Transfer[" + sourceSlot + "→CENTER]";
    }

    @Override
    protected void doCancel() {
        Dbg.w(LogGroup.TRANSFER, "=== CANCELLING TRANSFER ===");
        Dbg.d(LogGroup.TRANSFER, "  Source: %s, hardwareRunning: %s", sourceSlot, hardwareRunning);

        // Stop all transfer hardware immediately
        if (sourceSlot == SlotLedger.Slot.FRONT) {
            helper.runFrontIntake(false, 0);
            Dbg.d(LogGroup.TRANSFER, "  Stopped: Front intake");
        } else {
            helper.runBackIntake(false, 0);
            Dbg.d(LogGroup.TRANSFER, "  Stopped: Back intake");
        }
        helper.setInjectorPower(0);
        helper.setUptakePower(0);
        Dbg.d(LogGroup.TRANSFER, "  Stopped: Injectors + uptake");

        hardwareRunning = false;
        Dbg.i(LogGroup.TRANSFER, "Transfer cancelled, hardware stopped");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        telemetry.addData("Source Slot", sourceSlot);
        telemetry.addData("Hardware Running", hardwareRunning);
        telemetry.addData("Hardware Complete", hardwareComplete);

        // Center detection status (detailed for debugging)
        telemetry.addData("Center Any Detected", centerPerception.isArtifactPresent());
        telemetry.addData("Center BOTH Prox", centerPerception.isBothProximitySensorsDetecting() ? "✓✓ YES" : "✗ NO");
        telemetry.addData("Center Confidence", centerPerception.getConfidence());

        if (centerDetectionStarted) {
            long duration = System.currentTimeMillis() - centerDetectionStartTime;
            telemetry.addData("Detection Timer", duration + "/" + CENTER_DETECTION_DURATION_MS + "ms");
        }

        if (transferredArtifact != null) {
            telemetry.addData("Artifact Color", transferredArtifact.getColorClass());
            telemetry.addData("Confidence", String.format("%.2f", 
                             transferredArtifact.getColorConfidence()));
        }
    }
}
