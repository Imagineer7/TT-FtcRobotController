package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;

/**
 * CollectOperation - Collect artifact from front or back intake
 *
 * This operation demonstrates checkpoint-based color classification:
 * 1. Wait for edge detection (fast 30ms debounce)
 * 2. Start intake hardware
 * 3. Wait COLOR_CLASSIFICATION_DELAY for artifact to settle
 * 4. Sample color at checkpoint (Checkpoint 1)
 * 5. Continue running intake until complete
 * 6. Commit artifact to target slot
 *
 * Preconditions:
 * - System not full (< 3 artifacts)
 * - Target slot empty
 * - Artifact detected by perception
 *
 * Hardware: Runs intake rollers + transfer + bottom servos
 */
public class CollectOperation extends BaseOperation {

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final SlotLedger ledger;
    private final IntakePerception perception;
    private final BasicIndexingHelper helper;
    private final IndexingConfig config;
    private final SlotLedger.Slot targetSlot;
    private final int sequenceId;

    private ArtifactIdentity collectedArtifact;
    private boolean colorSampled;
    private long edgeDetectTime;

    // Sampling state machine
    private enum SamplingState {
        WAITING_HARDWARE_DELAY,     // Waiting for hardware checkpoint
        SETTLING,                    // Artifact settling after hardware stop
        SAMPLING_INITIAL,            // Initial color sampling
        JIGGLING,                    // Jiggling artifact to improve detection
        SETTLING_AFTER_JIGGLE,       // Settling after jiggle
        SAMPLING_AFTER_JIGGLE,       // Re-sampling after jiggle
        COMPLETE                     // Sampling complete
    }
    
    private SamplingState samplingState = SamplingState.WAITING_HARDWARE_DELAY;
    private long samplingStateStartTime;
    private int jiggleCycleCount;
    private int totalSampleCount;

    // Timing constants (non-blocking)
    private static final long COLOR_CLASSIFICATION_DELAY_MS = 150;
    private static final long SETTLE_DELAY_MS = 50;           // Artifact settling time
    private static final long INITIAL_SAMPLING_MS = 150;      // Initial color sampling duration
    private static final long JIGGLE_DURATION_MS = 900;       // Total jiggle duration
    private static final long JIGGLE_CYCLE_MS = 300;          // Time per jiggle cycle
    private static final long RESAMPLE_DURATION_MS = 150;     // Re-sampling duration
    private static final long SAMPLE_INTERVAL_MS = 15;        // Time between perception updates
    
    // Validation thresholds
    private static final double MIN_COLOR_CONFIDENCE = 0.5;   // Minimum confidence to commit
    private static final double JIGGLE_TRIGGER_CONFIDENCE = 0.60;  // Trigger jiggle if below this
    
    // Collection timeout
    private static final long COLLECTION_TIMEOUT_MS = 3000;  // 3 seconds

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new CollectOperation
     *
     * @param ledger Slot ledger to update
     * @param perception Intake perception for sensor fusion
     * @param helper BasicIndexingHelper for hardware control
     * @param config IndexingConfig for parameters
     * @param targetSlot Which slot to collect into (FRONT or BACK)
     * @param sequenceId Unique sequence ID for this artifact
     * @param telemetry Telemetry for logging
     */
    public CollectOperation(SlotLedger ledger,
                           IntakePerception perception,
                           BasicIndexingHelper helper,
                           IndexingConfig config,
                           SlotLedger.Slot targetSlot,
                           int sequenceId,
                           Telemetry telemetry) {
        super(telemetry, COLLECTION_TIMEOUT_MS);
        this.ledger = ledger;
        this.perception = perception;
        this.helper = helper;
        this.config = config;
        this.targetSlot = targetSlot;
        this.sequenceId = sequenceId;
        this.collectedArtifact = null;
        this.colorSampled = false;
        this.edgeDetectTime = 0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OPERATION IMPLEMENTATION
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    protected boolean doStart() {
        Dbg.d(LogGroup.INTAKE, "doStart() called for %s", targetSlot);

        // Check preconditions
        if (ledger.isFull()) {
            Dbg.w(LogGroup.INTAKE, "REJECTED: System full");
            fail(RejectReason.SYSTEM_FULL);
            return false;
        }

        if (ledger.isOccupied(targetSlot)) {
            Dbg.w(LogGroup.INTAKE, "REJECTED: Slot occupied");
            fail(RejectReason.SLOT_OCCUPIED);
            return false;
        }

        // Check if artifact is actually detected (use fast presence for responsive start)
        if (!perception.getFastPresence()) {
            Dbg.w(LogGroup.INTAKE, "REJECTED: No fast presence detected");
            fail(RejectReason.SENSOR_DETECTION_TIMEOUT);
            setStatusMessage("No artifact detected at " + targetSlot);
            return false;
        }

        // Record edge detection time for color checkpoint
        edgeDetectTime = System.currentTimeMillis();
        Dbg.d(LogGroup.INTAKE, "Fast presence detected, starting hardware...");

        // Start intake hardware
        boolean started;
        if (targetSlot == SlotLedger.Slot.FRONT) {
            helper.runFrontIntakeTimed(true, config.getIntakeRollerPower(), 
                                      config.getIntakeRollerTimeMs());
            started = true;
            Dbg.d(LogGroup.INTAKE, "Started FRONT intake hardware");
        } else if (targetSlot == SlotLedger.Slot.BACK) {
            helper.runBackIntakeTimed(true, config.getIntakeRollerPower(), 
                                     config.getIntakeRollerTimeMs());
            started = true;
            Dbg.d(LogGroup.INTAKE, "Started BACK intake hardware");
        } else {
            Dbg.e(LogGroup.INTAKE, "REJECTED: Invalid target slot");
            fail(RejectReason.INVALID_PARAMETERS);
            setStatusMessage("Invalid target slot: " + targetSlot);
            return false;
        }

        setStatusMessage("Collecting to " + targetSlot);
        Dbg.d(LogGroup.INTAKE, "doStart() completed successfully");
        return started;
    }

    @Override
    protected boolean doUpdate() {
        // Non-blocking state machine for color sampling
        // Called repeatedly in loop - processes one step per call
        
        long currentTime = System.currentTimeMillis();
        long elapsedInState = currentTime - samplingStateStartTime;
        
        switch (samplingState) {
            case WAITING_HARDWARE_DELAY:
                // Wait for color classification checkpoint
                long elapsedSinceEdge = currentTime - edgeDetectTime;
                
                if (elapsedSinceEdge >= COLOR_CLASSIFICATION_DELAY_MS) {
                    Dbg.d(LogGroup.INTAKE, "Color checkpoint reached, stopping hardware");

                    // Stop hardware for color sampling
                    stopHardware();
                    
                    // Transition to settling
                    samplingState = SamplingState.SETTLING;
                    samplingStateStartTime = currentTime;
                    Dbg.d(LogGroup.INTAKE, "→ SETTLING state");
                } else {
                    long remaining = COLOR_CLASSIFICATION_DELAY_MS - elapsedSinceEdge;
                    setStatusMessage("Waiting for settle (" + remaining + "ms)");
                }
                break;
                
            case SETTLING:
                // Wait for artifact to stop vibrating
                if (elapsedInState >= SETTLE_DELAY_MS) {
                    Dbg.d(LogGroup.INTAKE, "Artifact settled, starting initial sampling");

                    // Enable color sampling
                    perception.enableColorSampling();
                    totalSampleCount = 0;
                    
                    // Transition to initial sampling
                    samplingState = SamplingState.SAMPLING_INITIAL;
                    samplingStateStartTime = currentTime;
                    Dbg.d(LogGroup.INTAKE, "→ SAMPLING_INITIAL state");
                } else {
                    setStatusMessage("Settling artifact...");
                }
                break;
                
            case SAMPLING_INITIAL:
                // Continuously sample color sensors
                if (elapsedInState % SAMPLE_INTERVAL_MS < 5) {  // Sample every ~15ms
                    perception.update();
                    totalSampleCount++;
                }
                
                if (elapsedInState >= INITIAL_SAMPLING_MS) {
                    // Get sampled color
                    ArtifactIdentity.ColorClass color = perception.getBestColorClass();
                    double confidence = perception.getBestColorConfidence();
                    
                    Dbg.d(LogGroup.INTAKE, "Initial sampling complete: %d samples, color=%s, conf=%.2f",
                                     totalSampleCount, color, confidence);

                    // Check if jiggle needed
                    if (confidence < JIGGLE_TRIGGER_CONFIDENCE && color == ArtifactIdentity.ColorClass.UNKNOWN) {
                        Dbg.d(LogGroup.INTAKE, "Low confidence, starting jiggle routine");
                        logWarn("Low confidence (" + String.format("%.2f", confidence) + "), jiggling");
                        
                        jiggleCycleCount = 0;
                        samplingState = SamplingState.JIGGLING;
                        samplingStateStartTime = currentTime;
                        Dbg.d(LogGroup.INTAKE, "→ JIGGLING state");
                    } else {
                        // Good enough, complete sampling
                        collectedArtifact = ArtifactIdentity.createFromSensor(color, confidence, sequenceId);
                        colorSampled = true;
                        
                        perception.disableColorSampling();
                        samplingState = SamplingState.COMPLETE;
                        Dbg.d(LogGroup.INTAKE, "→ COMPLETE (initial sample sufficient)");

                        logInfo("Color checkpoint: " + color + " (conf=" + String.format("%.2f", confidence) + ")");
                        setStatusMessage("Collected " + color + " artifact");
                    }
                } else {
                    setStatusMessage("Sampling color... (" + totalSampleCount + ")");
                }
                break;
                
            case JIGGLING:
                // Jiggle artifact to rotate for better sensor view
                long jiggleElapsed = elapsedInState;
                int currentCycle = (int)(jiggleElapsed / JIGGLE_CYCLE_MS);
                
                if (currentCycle != jiggleCycleCount) {
                    jiggleCycleCount = currentCycle;
                }
                
                // Alternate transfer direction each cycle
                double transferPower = (jiggleCycleCount % 2 == 0) ? -0.5 : 0.5;
                
                if (targetSlot == SlotLedger.Slot.FRONT) {
                    helper.setFrontRollerPower(0.8);
                    helper.setFrontTransferPower(transferPower);
                } else {
                    helper.setBackRollerPower(0.8);
                    helper.setBackTransferPower(transferPower);
                }
                
                // Continue sampling during jiggle
                perception.update();
                
                if (jiggleElapsed >= JIGGLE_DURATION_MS) {
                    Dbg.d(LogGroup.INTAKE, "Jiggle complete (%d cycles)", jiggleCycleCount);

                    // Stop hardware
                    stopHardware();
                    
                    // Transition to settling after jiggle
                    samplingState = SamplingState.SETTLING_AFTER_JIGGLE;
                    samplingStateStartTime = currentTime;
                    Dbg.d(LogGroup.INTAKE, "→ SETTLING_AFTER_JIGGLE state");
                } else {
                    setStatusMessage("Jiggling artifact... (" + jiggleCycleCount + ")");
                }
                break;
                
            case SETTLING_AFTER_JIGGLE:
                // Wait for artifact to settle after jiggle
                if (elapsedInState >= SETTLE_DELAY_MS) {
                    Dbg.d(LogGroup.INTAKE, "Artifact settled, re-sampling");

                    totalSampleCount = 0;
                    
                    // Transition to re-sampling
                    samplingState = SamplingState.SAMPLING_AFTER_JIGGLE;
                    samplingStateStartTime = currentTime;
                    Dbg.d(LogGroup.INTAKE, "→ SAMPLING_AFTER_JIGGLE state");
                }
                break;
                
            case SAMPLING_AFTER_JIGGLE:
                // Re-sample color after jiggle
                if (elapsedInState % SAMPLE_INTERVAL_MS < 5) {
                    perception.update();
                    totalSampleCount++;
                }
                
                if (elapsedInState >= RESAMPLE_DURATION_MS) {
                    // Get final color
                    ArtifactIdentity.ColorClass color = perception.getBestColorClass();
                    double confidence = perception.getBestColorConfidence();
                    
                    Dbg.d(LogGroup.INTAKE, "Re-sampling complete: %d samples, color=%s, conf=%.2f",
                                     totalSampleCount, color, confidence);

                    collectedArtifact = ArtifactIdentity.createFromSensor(color, confidence, sequenceId);
                    colorSampled = true;
                    
                    perception.disableColorSampling();
                    samplingState = SamplingState.COMPLETE;
                    Dbg.d(LogGroup.INTAKE, "→ COMPLETE (after jiggle)");

                    logInfo("Color after jiggle: " + color + " (conf=" + String.format("%.2f", confidence) + ")");
                    setStatusMessage("Collected " + color + " artifact");
                } else {
                    setStatusMessage("Re-sampling... (" + totalSampleCount + ")");
                }
                break;
                
            case COMPLETE:
                // Sampling complete, wait for hardware to finish
                break;
        }
        
        // Check if hardware is done (regardless of sampling state)
        boolean stillBusy = isHardwareBusy();
        
        if (!stillBusy && samplingState != SamplingState.COMPLETE) {
            // Edge case: hardware finished before sampling complete
            Dbg.w(LogGroup.INTAKE, "Hardware done before sampling, forcing completion");

            if (samplingState == SamplingState.JIGGLING) {
                stopHardware();
            }
            
            // Force sample now
            if (!colorSampled) {
                perception.enableColorSampling();
                
                // Quick sample
                for (int i = 0; i < 5; i++) {
                    perception.update();
                }
                
                ArtifactIdentity.ColorClass color = perception.getBestColorClass();
                double confidence = perception.getBestColorConfidence();
                collectedArtifact = ArtifactIdentity.createFromSensor(color, confidence, sequenceId);
                colorSampled = true;
                
                perception.disableColorSampling();
                logWarn("Forced color sample at hardware completion");
            }
            
            samplingState = SamplingState.COMPLETE;
        }
        
        if (!stillBusy && samplingState == SamplingState.COMPLETE) {
            Dbg.d(LogGroup.INTAKE, "Collection complete");
            return false;  // Done
        }
        
        return true;  // Still running
    }
    
    /**
     * Stop all hardware for the target slot
     */
    private void stopHardware() {
        if (targetSlot == SlotLedger.Slot.FRONT) {
            helper.setFrontRollerPower(0);
            helper.setFrontTransferPower(0);
            helper.setFrontBottomIntakePower(0);
        } else {
            helper.setBackRollerPower(0);
            helper.setBackTransferPower(0);
            helper.setBackBottomIntakePower(0);
        }
    }

    /**
     * Check if hardware is still busy
     */
    private boolean isHardwareBusy() {
        if (targetSlot == SlotLedger.Slot.FRONT) {
            return helper.isFrontIntakeBusy();
        } else {
            return helper.isBackIntakeBusy();
        }
    }

    @Override
    protected void doCommit() {
        Dbg.d(LogGroup.INTAKE, "doCommit() called");

        // Commit artifact to slot ledger
        if (collectedArtifact == null) {
            // Failsafe: create UNKNOWN artifact if something went wrong
            logWarn("No artifact sampled - creating UNKNOWN");
            collectedArtifact = ArtifactIdentity.createUnknown(sequenceId);
            Dbg.w(LogGroup.INTAKE, "WARNING: No artifact sampled, created UNKNOWN");
        }
        
        // CRITICAL: Validate color confidence before committing
        // Only commit if confidence meets minimum threshold
        double confidence = collectedArtifact.getColorConfidence();
        if (confidence < MIN_COLOR_CONFIDENCE) {
            logWarn("Color confidence too low (" + String.format("%.2f", confidence) + 
                   " < " + MIN_COLOR_CONFIDENCE + "), REJECTING collection");
            Dbg.w(LogGroup.INTAKE, "REJECTED: Color confidence %.2f below threshold", confidence);

            // Don't commit - reject the operation
            fail(RejectReason.SENSOR_DETECTION_TIMEOUT);  // Use existing reject reason
            setStatusMessage("Color conf too low (" + String.format("%.2f", confidence) + ")");
            
            // Clear forced detection if active
            if (perception.isForcedDetectionActive()) {
                perception.clearForcedDetection();
            }
            return;
        }

        ledger.set(targetSlot, collectedArtifact);
        Dbg.i(LogGroup.INTAKE, "Committed %s (%s) to %s", collectedArtifact.getColorClass(),
                         collectedArtifact.getColorConfidence(), targetSlot);

        // Clear forced detection if it was used
        if (perception.isForcedDetectionActive()) {
            perception.clearForcedDetection();
            logInfo("Cleared forced detection after collection");
            Dbg.d(LogGroup.INTAKE, "Cleared forced detection");
        }
        
        logInfo("Committed " + collectedArtifact.getColorClass() + 
               " artifact to " + targetSlot + " (conf=" + String.format("%.2f", confidence) + ")");
        logDebug("Artifact", collectedArtifact.toString());
        logDebug("Slot Ledger", ledger.toSnapshot());
    }

    @Override
    public String getOperationName() {
        return "Collect[" + targetSlot + "]";
    }

    @Override
    protected void doCancel() {
        // Stop intake hardware
        if (targetSlot == SlotLedger.Slot.FRONT) {
            helper.stopFrontIntake();
        } else {
            helper.stopBackIntake();
        }
        
        // Clear forced detection on cancel
        if (perception.isForcedDetectionActive()) {
            perception.clearForcedDetection();
            logInfo("Cleared forced detection after cancel");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void addTelemetry() {
        super.addTelemetry();
        
        telemetry.addData("Target Slot", targetSlot);
        telemetry.addData("Color Sampled", colorSampled);
        
        if (colorSampled && collectedArtifact != null) {
            telemetry.addData("Detected Color", collectedArtifact.getColorClass());
            telemetry.addData("Confidence", String.format("%.2f", 
                             collectedArtifact.getColorConfidence()));
        }
        
        if (edgeDetectTime > 0) {
            long elapsedSinceEdge = System.currentTimeMillis() - edgeDetectTime;
            telemetry.addData("Edge Delay", elapsedSinceEdge + "ms / " + 
                             COLOR_CLASSIFICATION_DELAY_MS + "ms");
        }
    }
}
