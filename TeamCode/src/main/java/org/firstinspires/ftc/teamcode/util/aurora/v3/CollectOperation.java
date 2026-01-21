package org.firstinspires.ftc.teamcode.util.aurora.v3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;

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

    // Color classification delay (Checkpoint 1 policy)
    private static final long COLOR_CLASSIFICATION_DELAY_MS = 150;
    
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
        System.out.println("[CollectOp] doStart() called for " + targetSlot);
        
        // Check preconditions
        if (ledger.isFull()) {
            System.out.println("[CollectOp] REJECTED: System full");
            fail(RejectReason.SYSTEM_FULL);
            return false;
        }

        if (ledger.isOccupied(targetSlot)) {
            System.out.println("[CollectOp] REJECTED: Slot occupied");
            fail(RejectReason.SLOT_OCCUPIED);
            return false;
        }

        // Check if artifact is actually detected (use fast presence for responsive start)
        if (!perception.getFastPresence()) {
            System.out.println("[CollectOp] REJECTED: No fast presence detected");
            fail(RejectReason.SENSOR_DETECTION_TIMEOUT);
            setStatusMessage("No artifact detected at " + targetSlot);
            return false;
        }

        // Record edge detection time for color checkpoint
        edgeDetectTime = System.currentTimeMillis();
        System.out.println("[CollectOp] Fast presence detected, starting hardware...");

        // Start intake hardware
        boolean started;
        if (targetSlot == SlotLedger.Slot.FRONT) {
            helper.runFrontIntakeTimed(true, config.getIntakeRollerPower(), 
                                      config.getIntakeRollerTimeMs());
            started = true;
            System.out.println("[CollectOp] Started FRONT intake hardware");
        } else if (targetSlot == SlotLedger.Slot.BACK) {
            helper.runBackIntakeTimed(true, config.getIntakeRollerPower(), 
                                     config.getIntakeRollerTimeMs());
            started = true;
            System.out.println("[CollectOp] Started BACK intake hardware");
        } else {
            System.out.println("[CollectOp] REJECTED: Invalid target slot");
            fail(RejectReason.INVALID_PARAMETERS);
            setStatusMessage("Invalid target slot: " + targetSlot);
            return false;
        }

        setStatusMessage("Collecting to " + targetSlot);
        System.out.println("[CollectOp] doStart() completed successfully");
        return started;
    }

    @Override
    protected boolean doUpdate() {
        System.out.println("[CollectOp] doUpdate() - colorSampled=" + colorSampled);
        
        // Checkpoint 1: Color classification after delay
        if (!colorSampled) {
            long elapsedSinceEdge = System.currentTimeMillis() - edgeDetectTime;
            
            if (elapsedSinceEdge >= COLOR_CLASSIFICATION_DELAY_MS) {
                System.out.println("[CollectOp] Color checkpoint reached, sampling...");
                
                // Enable color sampling
                perception.enableColorSampling();
                
                // CRITICAL: Color sensors need time to get accurate readings
                // REV color sensors have latency - give them time to capture fresh data
                // Without this delay, sensors return stale/inaccurate readings (UNKNOWN)
                try {
                    Thread.sleep(100);  // 100ms for sensors to stabilize
                    System.out.println("[CollectOp] Waited 100ms for color sensors to stabilize");
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                
                // Now call update() to read the fresh sensor data
                perception.update();
                
                // Get the freshly sampled color
                ArtifactIdentity.ColorClass color = perception.getBestColorClass();
                double confidence = perception.getBestColorConfidence();
                
                // Disable color sampling
                perception.disableColorSampling();
                
                collectedArtifact = ArtifactIdentity.createFromSensor(
                    color, confidence, sequenceId
                );
                
                colorSampled = true;
                
                System.out.println("[CollectOp] Color sampled: " + color + " (conf=" + 
                                 String.format("%.2f", confidence) + ")");
                
                // Log color classification
                logInfo("Color checkpoint: " + color + " (conf=" + 
                       String.format("%.2f", confidence) + ")");
                setStatusMessage("Collected " + color + " artifact");
            } else {
                // Still waiting for delay
                long remaining = COLOR_CLASSIFICATION_DELAY_MS - elapsedSinceEdge;
                setStatusMessage("Waiting for settle (" + remaining + "ms)");
                // System.out.println("[CollectOp] Waiting " + remaining + "ms for settle");
            }
        }

        // Check if hardware is done
        boolean stillBusy;
        if (targetSlot == SlotLedger.Slot.FRONT) {
            stillBusy = helper.isFrontIntakeBusy();
        } else {
            stillBusy = helper.isBackIntakeBusy();
        }
        
        System.out.println("[CollectOp] Hardware busy check: " + stillBusy);

        if (!stillBusy) {
            // Hardware complete
            if (!colorSampled) {
                System.out.println("[CollectOp] WARNING: Hardware completed before color delay!");
                
                // Edge case: hardware finished before color delay
                // Sample color now
                perception.enableColorSampling();
                
                // Give sensors time to get fresh readings
                try {
                    Thread.sleep(100);  // 100ms for sensors to stabilize
                    System.out.println("[CollectOp] (Edge case) Waited 100ms for color sensors");
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                
                perception.update();  // Read fresh sensor data
                ArtifactIdentity.ColorClass color = perception.getBestColorClass();
                double confidence = perception.getBestColorConfidence();
                perception.disableColorSampling();
                
                collectedArtifact = ArtifactIdentity.createFromSensor(
                    color, confidence, sequenceId
                );
                
                colorSampled = true;
                logWarn("Color sampled at hardware completion (delay not full)");
            }
            
            System.out.println("[CollectOp] Collection hardware complete, returning false (done)");
            setStatusMessage("Collection hardware complete");
            return false;  // Done
        }

        return true;  // Still running
    }

    @Override
    protected void doCommit() {
        System.out.println("[CollectOp] doCommit() called");
        
        // Commit artifact to slot ledger
        if (collectedArtifact == null) {
            // Failsafe: create UNKNOWN artifact if something went wrong
            logWarn("No artifact sampled - creating UNKNOWN");
            collectedArtifact = ArtifactIdentity.createUnknown(sequenceId);
            System.out.println("[CollectOp] WARNING: No artifact sampled, created UNKNOWN");
        }

        ledger.set(targetSlot, collectedArtifact);
        System.out.println("[CollectOp] Committed " + collectedArtifact.getColorClass() + 
                         " to " + targetSlot);
        
        // Clear forced detection if it was used
        if (perception.isForcedDetectionActive()) {
            perception.clearForcedDetection();
            logInfo("Cleared forced detection after collection");
            System.out.println("[CollectOp] Cleared forced detection");
        }
        
        logInfo("Committed " + collectedArtifact.getColorClass() + 
               " artifact to " + targetSlot);
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
