package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import java.util.ArrayList;
import java.util.List;

/**
 * IndexingSystem - Push-Based Artifact Indexing System for Aurora Robot
 *
 * This system manages the collection, storage, and firing of up to 3 artifacts using
 * a push-based mechanical indexing mechanism. The system respects physical constraints
 * where injector servos move artifacts between transfer system and center storage.
 *
 * INDEXING RULES:
 * 1. First Artifact:
 *    - Goes into center storage
 *    - Cannot be moved unless pushed by the second artifact
 *
 * 2. Second Artifact:
 *    - Enters center storage
 *    - Pushes first artifact into the opposite intake for storage
 *    - Remains in center storage (becomes forced first shot)
 *
 * 3. Third Artifact:
 *    - Stored in the same intake it was collected from
 *    - Does not push any other artifact
 *
 * EARLY FIRE HANDLING:
 * - One artifact: Transfer to center and fire
 * - Two artifacts: Can rearrange based on motif pattern using empty intake
 *   Example: Green in center, Purple in back, pattern wants Purple first
 *   Solution: Push green to front intake, pull purple to center
 * - Three artifacts: Fire in center (no rearrangement possible)
 *
 * SHOT PLANNING:
 * - Motif pattern (PPG, PGP, or GPP) set by limelight camera determines shot order
 * - With 2 artifacts: Can rearrange to match desired first shot
 * - With 3 artifacts: First shot is mechanically forced (in center)
 * - Software plans shots 2 and 3 based on motif pattern and artifact colors
 */
public class IndexingSystem {

    // ═══════════════════════════════════════════════════════════════════════
    // SYSTEM STATE
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Current state of the indexing system
     */
    public enum SystemState {
        IDLE,                   // No operations in progress
        COLLECTING,             // Actively collecting an artifact
        TRANSFERRING,           // Transferring artifact to center
        PUSHING,                // Pushing artifact to storage
        READY_TO_FIRE,          // Artifact in center, ready to fire
        FIRING,                 // Actively firing an artifact
        ERROR                   // System error detected
    }

    /**
     * Source intake for artifact collection
     */
    public enum IntakeSource {
        FRONT,
        BACK,
        UNKNOWN
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final AuroraHardwareConfig hardware;
    private final IndexingConfig config;
    private final Telemetry telemetry;
    private final Shooter shooter;

    // System state
    private SystemState currentState;
    private long stateStartTime;

    // Artifact memory - tracks all artifacts in the system
    private final List<Artifact> artifacts;
    private int nextCollectionOrder;

    // Current operation tracking
    private Artifact artifactInCenter;
    private Artifact artifactInFrontIntake;
    private Artifact artifactInBackIntake;
    private IntakeSource lastIntakeSource;

    // Timing and operation state
    private long operationStartTime;
    private boolean operationInProgress;

    // Shot planning
    private Artifact plannedSecondShot;
    private Artifact plannedThirdShot;
    private Artifact plannedFirstShot;  // For external access

    // Motif pattern for shot order (determined by limelight camera)
    private String motifPattern = "PPG"; // Default pattern: Purple, Purple, Green
    private boolean motifPatternSet = false;

    // Safety and error tracking
    private String lastError;
    private int errorCount;

    // Color detection delay tracking
    private long frontArtifactFirstDetected = 0;
    private long backArtifactFirstDetected = 0;
    private Artifact frontPendingArtifact = null;
    private Artifact backPendingArtifact = null;

    // Uptake servo coordination state
    private boolean uptakeServoPrePositioned = false;
    private long uptakeServoActionTime = 0;
    private boolean uptakeServoPrePositionedForCurrentArtifact = false; // Tracks if we've pre-positioned for current center artifact
    private long lastSensorCheck = 0;
    private static final long SENSOR_CHECK_INTERVAL = 50; // 50ms = 20Hz

    // Automatic detection state
    private boolean autoDetectionEnabled = true;

    // Debug message storage for opmode display
    private final java.util.concurrent.ConcurrentLinkedQueue<String> debugMessages = new java.util.concurrent.ConcurrentLinkedQueue<>();
    private static final int MAX_DEBUG_MESSAGES = 10; // Keep last 10 messages
    private final long debugStartTime = System.currentTimeMillis(); // Reference time for relative timestamps

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new IndexingSystem
     * @param hardware The Aurora hardware configuration
     * @param config The indexing configuration parameters
     * @param shooter The shooter subsystem
     * @param telemetry The telemetry system for logging
     */
    public IndexingSystem(AuroraHardwareConfig hardware, IndexingConfig config, Shooter shooter, Telemetry telemetry) {
        try {
            this.hardware = hardware;
            this.config = config;
            this.shooter = shooter;
            this.telemetry = telemetry;

            this.currentState = SystemState.IDLE;
            this.stateStartTime = System.currentTimeMillis();

            this.artifacts = new ArrayList<>();
            this.nextCollectionOrder = 1;

            this.artifactInCenter = null;
            this.artifactInFrontIntake = null;
            this.artifactInBackIntake = null;
            this.lastIntakeSource = IntakeSource.UNKNOWN;

            this.operationStartTime = 0;
            this.operationInProgress = false;

            this.plannedSecondShot = null;
            this.plannedThirdShot = null;

            this.lastError = "";
            this.errorCount = 0;

            // Don't initialize hardware automatically - wait for enable() call
            // This prevents motors from running during initialization phase

            // Log successful initialization
            if (telemetry != null && config != null && config.isDebugTelemetry()) {
                telemetry.addLine("✅ IndexingSystem initialized successfully");
            }
        } catch (Exception e) {
            // Log initialization error
            if (telemetry != null) {
                telemetry.addLine("❌ IndexingSystem init error: " + e.getMessage());
                telemetry.update();
            }
            // Set safe defaults
            this.currentState = SystemState.ERROR;
            this.lastError = "Initialization failed: " + e.getMessage();
            throw new RuntimeException("IndexingSystem initialization failed", e);
        }
    }
    
    /**
     * Initialize hardware - start rollers running continuously
     */
    private void initializeHardware() {
        try {
            // Both intakes start in collection mode (full power, rolling inward)
            // These will gracefully handle missing motors
            setIntakeCollectionMode(IntakeSource.FRONT);
            setIntakeCollectionMode(IntakeSource.BACK);

            // All servos start in idle position
            // This will gracefully handle missing servos
            resetAllServos();

            if (telemetry != null && config != null && config.isDebugTelemetry()) {
                // Count available hardware
                int availableMotors = 0;
                if (hardware != null) {
                    if (hardware.getFrontRollerMotor() != null) availableMotors++;
                    if (hardware.getBackRollerMotor() != null) availableMotors++;
                }

                int availableServos = 0;
                if (hardware != null) {
                    if (hardware.getInjectorServoLeft() != null) availableServos++;
                    if (hardware.getInjectorServoRight() != null) availableServos++;
                    if (hardware.getUptakeServoL() != null) availableServos++;
                    if (hardware.getUptakeServoR() != null) availableServos++;
                    if (hardware.getFrontTransferServo() != null) availableServos++;
                    if (hardware.getBackTransferServo() != null) availableServos++;
                }

                telemetry.addLine(String.format("Hardware: %d motors, %d servos available",
                    availableMotors, availableServos));
            }

        } catch (Exception e) {
            setError("Hardware initialization failed: " + e.getMessage());
            if (telemetry != null) {
                telemetry.addLine("❌ Hardware init error: " + e.getMessage());
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC API METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Enable the indexing system - starts hardware operation
     * Call this after initialization but before starting operation
     */
    public void enable() {
        try {
            initializeHardware();
            if (telemetry != null && config != null && config.isDebugTelemetry()) {
                telemetry.addLine("✅ IndexingSystem enabled successfully");
            }
        } catch (Exception e) {
            setError("Enable failed: " + e.getMessage());
            if (telemetry != null) {
                telemetry.addLine("❌ IndexingSystem enable error: " + e.getMessage());
            }
        }
    }

    /**
     * Disable the indexing system - stops all motors and servos
     * Call this when stopping the OpMode
     */
    public void disable() {
        // Stop all intake motors
        setIntakePower(IntakeSource.FRONT, 0.0);
        setIntakePower(IntakeSource.BACK, 0.0);

        // Stop all servos
        resetAllServos();
    }

    /**
     * Called when an artifact is detected at an intake
     * @param artifact The detected artifact with color and source information
     * @return true if artifact collection started successfully
     */
    public boolean onArtifactDetected(Artifact artifact, IntakeSource source) {
        // Safety check: can't collect more than 3 artifacts
        if (getArtifactCount() >= IndexingConfig.MAX_ARTIFACTS) {
            setError("Cannot collect artifact: system full (3/3 artifacts)");
            return false;
        }

        // Safety check: can't start new collection while operation in progress
        if (operationInProgress) {
            if (config.isDebugTelemetry()) {
                telemetry.addLine("Cannot collect: operation in progress");
            }
            return false;
        }

        // Update artifact with correct collection order
        Artifact collectionArtifact = new Artifact(
            artifact.getColor(),
            Artifact.Location.UNKNOWN,
            nextCollectionOrder
        );

        lastIntakeSource = source;
        startArtifactCollection(collectionArtifact);
        return true;
    }

    /**
     * Called when an artifact is first detected at an intake - starts color detection delay
     * This replaces the immediate onArtifactDetected call for better color readings
     * @param source Which intake detected the artifact
     * @return true if detection delay started
     */
    public boolean onArtifactFirstDetected(IntakeSource source) {
        // Safety checks
        if (getArtifactCount() >= IndexingConfig.MAX_ARTIFACTS) {
            return false;
        }

        if (operationInProgress) {
            return false;
        }

        // Detect color NOW while artifact is in front of sensor
        Artifact.Color detectedColor = detectArtifactColor(source);

        // Only proceed if we detected a valid color
        if (detectedColor != Artifact.Color.PURPLE && detectedColor != Artifact.Color.GREEN) {
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine("🔍 " + source + ": No valid color detected, waiting...");
            }
            return false;
        }

        // Start color detection delay for the appropriate intake
        // Store the detected color so we don't need to re-detect after delay
        long currentTime = System.currentTimeMillis();

        if (source == IntakeSource.FRONT) {
            if (frontPendingArtifact == null) { // Only start if not already pending
                frontArtifactFirstDetected = currentTime;
                // Store the detected color in the pending artifact
                frontPendingArtifact = new Artifact(detectedColor, Artifact.Location.UNKNOWN, 0);

                if (config.isDebugTelemetry() && telemetry != null) {
                    telemetry.addLine("🔍 FRONT: Detected " + detectedColor + " - waiting 0.8s...");
                }
            }
        } else if (source == IntakeSource.BACK) {
            if (backPendingArtifact == null) { // Only start if not already pending
                backArtifactFirstDetected = currentTime;
                // Store the detected color in the pending artifact
                backPendingArtifact = new Artifact(detectedColor, Artifact.Location.UNKNOWN, 0);

                if (config.isDebugTelemetry() && telemetry != null) {
                    telemetry.addLine("🔍 BACK: Detected " + detectedColor + " - waiting 0.8s...");
                }
            }
        }

        return true;
    }

    /**
     * Called to cancel pending artifact detection (artifact removed before delay completed)
     * @param source Which intake to cancel
     */
    public void cancelPendingArtifact(IntakeSource source) {
        if (source == IntakeSource.FRONT) {
            frontArtifactFirstDetected = 0;
            frontPendingArtifact = null;
            if (config.isDebugTelemetry()) {
                telemetry.addLine("🔍 FRONT: Artifact removed before collection");
            }
        } else if (source == IntakeSource.BACK) {
            backArtifactFirstDetected = 0;
            backPendingArtifact = null;
            if (config.isDebugTelemetry()) {
                telemetry.addLine("🔍 BACK: Artifact removed before collection");
            }
        }
    }

    /**
     * Check and update color detection delays - call this from your update loop
     * This handles the 0.8-second delay before actually starting collection
     */
    public void updateColorDetectionDelays() {
        long currentTime = System.currentTimeMillis();

        // Check front intake delay
        if (frontPendingArtifact != null) {
            if (currentTime - frontArtifactFirstDetected >= config.getColorDetectionDelayMs()) {
                // Delay complete - use the color that was detected when artifact first arrived
                Artifact.Color detectedColor = frontPendingArtifact.getColor();

                if (detectedColor != Artifact.Color.UNKNOWN) {
                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine("🔍 FRONT: CREATING ARTIFACT - PRE STATE:");
                        telemetry.addLine(String.format("   NextOrder: %d, ArtifactCount: %d, ListSize: %d",
                            nextCollectionOrder, getArtifactCount(), artifacts.size()));
                        telemetry.addLine(String.format("   State: %s, OpInProgress: %s",
                            currentState, operationInProgress));
                        for (int i = 0; i < artifacts.size(); i++) {
                            Artifact a = artifacts.get(i);
                            telemetry.addLine(String.format("   Artifact[%d]: #%d %s at %s",
                                i, a.getCollectionOrder(), a.getColor(), a.getLocation()));
                        }
                    }

                    // SAFETY: For third artifact detection, explicitly ensure collection order is 3
                    int collectionOrder = nextCollectionOrder;
                    if (currentState == SystemState.READY_TO_FIRE && getArtifactCount() == 2 &&
                        artifactInCenter != null && (artifactInFrontIntake != null || artifactInBackIntake != null)) {
                        // This is definitely the third artifact - ensure it gets order 3
                        if (collectionOrder != 3) {
                            if (config.isDebugTelemetry() && telemetry != null) {
                                telemetry.addLine(String.format("⚠️ FRONT: Correcting collection order from %d to 3 for third artifact", collectionOrder));
                            }
                            collectionOrder = 3;
                        }
                    }

                    Artifact finalArtifact = new Artifact(detectedColor, Artifact.Location.UNKNOWN, collectionOrder);
                    lastIntakeSource = IntakeSource.FRONT;

                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine(String.format("🔍 FRONT: Created artifact #%d (nextOrder=%d, totalCount=%d)",
                            finalArtifact.getCollectionOrder(), nextCollectionOrder, getArtifactCount()));
                    }

                    // Clear pending state BEFORE starting collection
                    frontPendingArtifact = null;
                    frontArtifactFirstDetected = 0;

                    // Start collection - this activates the transfer servos and injectors
                    startArtifactCollection(finalArtifact);

                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine("🔍 FRONT: Starting collection of " + detectedColor + " artifact");
                    }
                } else {
                    // Color was unknown - shouldn't happen but handle gracefully
                    frontPendingArtifact = null;
                    frontArtifactFirstDetected = 0;
                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine("🔍 FRONT: Collection cancelled - no valid color");
                    }
                }
            }
        }

        // Check back intake delay
        if (backPendingArtifact != null) {
            if (currentTime - backArtifactFirstDetected >= config.getColorDetectionDelayMs()) {
                // Delay complete - use the color that was detected when artifact first arrived
                Artifact.Color detectedColor = backPendingArtifact.getColor();

                if (detectedColor != Artifact.Color.UNKNOWN) {
                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine("🔍 BACK: CREATING ARTIFACT - PRE STATE:");
                        telemetry.addLine(String.format("   NextOrder: %d, ArtifactCount: %d, ListSize: %d",
                            nextCollectionOrder, getArtifactCount(), artifacts.size()));
                        telemetry.addLine(String.format("   State: %s, OpInProgress: %s",
                            currentState, operationInProgress));
                        for (int i = 0; i < artifacts.size(); i++) {
                            Artifact a = artifacts.get(i);
                            telemetry.addLine(String.format("   Artifact[%d]: #%d %s at %s",
                                i, a.getCollectionOrder(), a.getColor(), a.getLocation()));
                        }
                    }

                    // SAFETY: For third artifact detection, explicitly ensure collection order is 3
                    int collectionOrder = nextCollectionOrder;
                    if (currentState == SystemState.READY_TO_FIRE && getArtifactCount() == 2 &&
                        artifactInCenter != null && (artifactInFrontIntake != null || artifactInBackIntake != null)) {
                        // This is definitely the third artifact - ensure it gets order 3
                        if (collectionOrder != 3) {
                            if (config.isDebugTelemetry() && telemetry != null) {
                                telemetry.addLine(String.format("⚠️ BACK: Correcting collection order from %d to 3 for third artifact", collectionOrder));
                            }
                            collectionOrder = 3;
                        }
                    }

                    Artifact finalArtifact = new Artifact(detectedColor, Artifact.Location.UNKNOWN, collectionOrder);
                    lastIntakeSource = IntakeSource.BACK;

                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine(String.format("🔍 BACK: Created artifact #%d (nextOrder=%d, totalCount=%d)",
                            finalArtifact.getCollectionOrder(), nextCollectionOrder, getArtifactCount()));
                    }

                    // Clear pending state BEFORE starting collection
                    backPendingArtifact = null;
                    backArtifactFirstDetected = 0;

                    // Start collection - this activates the transfer servos and injectors
                    startArtifactCollection(finalArtifact);

                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine("🔍 BACK: Starting collection of " + detectedColor + " artifact");
                    }
                } else {
                    // Color was unknown - shouldn't happen but handle gracefully
                    backPendingArtifact = null;
                    backArtifactFirstDetected = 0;
                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine("🔍 BACK: Collection cancelled - no valid color");
                    }
                }
            }
        }
    }

    /**
     * Check if an intake has a pending artifact (waiting for color detection delay)
     * @param source Which intake to check
     * @return true if artifact is pending color detection
     */
    public boolean hasPendingArtifact(IntakeSource source) {
        if (source == IntakeSource.FRONT) {
            return frontPendingArtifact != null;
        } else if (source == IntakeSource.BACK) {
            return backPendingArtifact != null;
        }
        return false;
    }

    /**
     * Get remaining color detection delay time for an intake
     * @param source Which intake to check
     * @return Remaining delay in milliseconds, or 0 if no pending artifact
     */
    public long getRemainingColorDelay(IntakeSource source) {
        long currentTime = System.currentTimeMillis();

        if (source == IntakeSource.FRONT && frontPendingArtifact != null) {
            long elapsed = currentTime - frontArtifactFirstDetected;
            long remaining = config.getColorDetectionDelayMs() - elapsed;
            return Math.max(0, remaining);
        } else if (source == IntakeSource.BACK && backPendingArtifact != null) {
            long elapsed = currentTime - backArtifactFirstDetected;
            long remaining = config.getColorDetectionDelayMs() - elapsed;
            return Math.max(0, remaining);
        }

        return 0;
    }

    /**
     * Called when fire signal is issued
     * @return true if firing started successfully
     * TODO: Implement firing logic
     */
    public boolean onFireSignal() {
        // Stub: Firing logic removed - needs to be reimplemented
        if (config.isDebugTelemetry()) {
            telemetry.addLine("Fire signal received - firing logic not implemented");
        }
        return false;
    }
    
    /**
     * Set the motif pattern for shot ordering
     * This should be called by the limelight camera system to determine shot order.
     * @param pattern One of "PPG", "PGP", or "GPP" where P=Purple, G=Green
     * @return true if pattern is valid and set
     * TODO: Implement shot planning logic
     */
    public boolean setMotifPattern(String pattern) {
        if (pattern == null) {
            return false;
        }
        
        String normalized = pattern.trim().toUpperCase();
        if (normalized.equals("PPG") || normalized.equals("PGP") || normalized.equals("GPP")) {
            this.motifPattern = normalized;
            this.motifPatternSet = true;
            
            if (config.isDebugTelemetry()) {
                telemetry.addLine("Motif pattern set: " + normalized);
            }
            
            // Shot planning removed - needs to be reimplemented
            
            return true;
        }
        
        if (config.isDebugTelemetry()) {
            telemetry.addLine("Invalid motif pattern: " + pattern);
        }
        return false;
    }
    
    /**
     * Get the current motif pattern
     * @return The motif pattern string (PPG, PGP, or GPP)
     */
    public String getMotifPattern() {
        return motifPattern;
    }
    
    /**
     * Check if motif pattern has been set by limelight
     * @return true if pattern has been explicitly set
     */
    public boolean isMotifPatternSet() {
        return motifPatternSet;
    }

    /**
     * Periodic update method - call this regularly from OpMode loop
     * Handles state machine transitions and ongoing operations
     */
    public void update() {
        long currentTime = System.currentTimeMillis();
        long stateElapsedTime = currentTime - stateStartTime;

        // Update color detection delays first
        updateColorDetectionDelays();

        // Update uptake servo pre-positioning timeout
        updateUptakeServoTimeout(currentTime);

        // Handle automatic sensor monitoring and detection
        if (autoDetectionEnabled) {
            handleAutomaticDetection(currentTime);
        }

        // Shot planning removed - needs to be reimplemented

        // State machine processing FIRST (this may reset operationStartTime during state transitions)
        switch (currentState) {
            case COLLECTING:
                // Recalculate elapsed time for this specific state
                long collectingElapsed = System.currentTimeMillis() - operationStartTime;
                if (config.isDebugTelemetry() && telemetry != null) {
                    telemetry.addData("⏱️ COLLECTING", String.format("%.1fs / %.1fs",
                        collectingElapsed / 1000.0, config.getIntakeRollerTimeMs() / 1000.0));
                }
                updateCollecting(collectingElapsed);
                break;

            case TRANSFERRING:
                // Recalculate elapsed time for this specific state
                long transferringElapsed = System.currentTimeMillis() - operationStartTime;
                if (config.isDebugTelemetry() && telemetry != null) {
                    long totalTransferTime = config.getTransferServoTimeMs() + config.getCenterAcceptTimeMs();
                    telemetry.addData("⏱️ TRANSFERRING", String.format("%.1fs / %.1fs",
                        transferringElapsed / 1000.0, totalTransferTime / 1000.0));
                }
                updateTransferring(transferringElapsed);
                break;

            case PUSHING:
                // Recalculate elapsed time for this specific state
                long pushingElapsed = System.currentTimeMillis() - operationStartTime;
                if (config.isDebugTelemetry() && telemetry != null) {
                    long totalPushTime = config.getPushStartDelayMs() +
                                        config.getSecondArtifactPushTimeMs() +
                                        config.getStorageIntakeAcceptTimeMs();
                    telemetry.addData("⏱️ PUSHING", String.format("%.1fs / %.1fs",
                        pushingElapsed / 1000.0, totalPushTime / 1000.0));
                }
                updatePushing(pushingElapsed);
                break;

            case FIRING:
                // Firing logic removed - needs to be reimplemented
                if (config.isDebugTelemetry() && telemetry != null) {
                    telemetry.addLine("⚠️ FIRING state encountered - not implemented");
                }
                // Reset to idle to prevent system from getting stuck
                resetToIdle();
                break;

            case READY_TO_FIRE:
                // Ensure uptake servos are pre-positioned when ready to fire
                // Only start pre-positioning if not already in progress and haven't completed for this artifact
                if (!uptakeServoPrePositioned && !operationInProgress && artifactInCenter != null &&
                    !uptakeServoPrePositionedForCurrentArtifact) {

                    if (config.isDebugTelemetry() && telemetry != null) {
                        String msg = "🔧 STATE: READY_TO_FIRE starting pre-position";
                        telemetry.addLine(msg);
                        addDebugMessage(msg);
                    }
                    setUptakeServoPrePosition(true);
                    // Don't set the flag here - let timeout completion set it
                }
                break;

            case IDLE:
                // Ensure uptake servos are pre-positioned if there's an artifact in center
                // Only start pre-positioning if not already in progress and haven't completed for this artifact
                if (!uptakeServoPrePositioned && !operationInProgress && artifactInCenter != null &&
                    !uptakeServoPrePositionedForCurrentArtifact) {

                    if (config.isDebugTelemetry() && telemetry != null) {
                        String msg = "🔧 STATE: IDLE starting pre-position";
                        telemetry.addLine(msg);
                        addDebugMessage(msg);
                    }
                    setUptakeServoPrePosition(true);
                    // Don't set the flag here - let timeout completion set it
                } else if (config.isDebugTelemetry() && telemetry != null && artifactInCenter != null) {
                    // DEBUG: Track every few seconds when IDLE state is checking pre-positioning
                    long debugTime = System.currentTimeMillis();
                    if (debugTime % 2000 < 50) { // Log approximately every 2 seconds
                        String debugMsg = String.format("🔧 IDLE: checking pre-position (prePos:%s, completed:%s)",
                            uptakeServoPrePositioned, uptakeServoPrePositionedForCurrentArtifact);
                        addDebugMessage(debugMsg);
                    }
                }
                break;

            case ERROR:
                // Try auto-recovery if enabled
                if (config.isEnableAutoRecovery() && stateElapsedTime > 1000) {
                    resetToIdle();
                }
                break;
        }

        // Check for operation timeout AFTER state machine processing
        // This uses the NEW operationStartTime if a state transition occurred
        if (operationInProgress) {
            long newOperationElapsedTime = System.currentTimeMillis() - operationStartTime;
            if (newOperationElapsedTime > config.getOperationTimeoutMs()) {
                if (config.isDebugTelemetry() && telemetry != null) {
                    telemetry.addLine("⚠️ TIMEOUT in state: " + currentState +
                        " after " + (newOperationElapsedTime / 1000.0) + "s");
                }
                setError("Operation timeout in state: " + currentState);
                resetToIdle();
                return;
            }
        }

        // Update telemetry if debug enabled
        if (config.isDebugTelemetry()) {
            updateTelemetry();
        }
    }

    /**
     * Reset the indexing system to initial state
     * Clears all artifact memory and resets counters
     */
    public void reset() {
        artifacts.clear();
        nextCollectionOrder = 1;
        artifactInCenter = null;
        artifactInFrontIntake = null;
        artifactInBackIntake = null;
        lastIntakeSource = IntakeSource.UNKNOWN;
        plannedSecondShot = null;
        plannedThirdShot = null;
        lastError = "";
        errorCount = 0;

        // Clear pending color detection delays
        frontArtifactFirstDetected = 0;
        backArtifactFirstDetected = 0;
        frontPendingArtifact = null;
        backPendingArtifact = null;

        // Reset uptake servo coordination state
        uptakeServoPrePositioned = false;
        uptakeServoActionTime = 0;

        resetToIdle();
        
        // Reinitialize hardware to restart rollers
        initializeHardware();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CORE INDEXING LOGIC - COLLECTION AND STORAGE
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Start collecting an artifact from an intake
     */
    private void startArtifactCollection(Artifact artifact) {
        // Clear any stale pending artifacts to prevent conflicts
        clearPendingArtifacts();

        changeState(SystemState.COLLECTING);
        operationInProgress = true;
        operationStartTime = System.currentTimeMillis();

        // Add artifact to tracking
        artifacts.add(artifact);

        // Start hardware for collection - but third artifact doesn't need transfer hardware
        if (artifact.getCollectionOrder() == 3) {
            // Third artifact: Only keep intake rollers running, no transfer servos/injectors
            executeThirdArtifactCollectionHardware();
        } else {
            // First and second artifacts: Full transfer hardware
            executeCollectionHardware();
        }

        if (config.isDebugTelemetry()) {
            telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            telemetry.addLine(String.format("🔄 COLLECTING ARTIFACT #%d", artifact.getCollectionOrder()));
            telemetry.addLine(String.format("   Color: %s", artifact.getColor()));
            telemetry.addLine(String.format("   Source: %s intake", lastIntakeSource));
            telemetry.addLine(String.format("   Total Count: %d/%d", getArtifactCount(), IndexingConfig.MAX_ARTIFACTS));

            // Show what will happen next and hardware activation
            switch (artifact.getCollectionOrder()) {
                case 1:
                    telemetry.addLine("   Next: Transfer to center storage");
                    telemetry.addLine("   Hardware: Transfer servos + Injectors ON");
                    break;
                case 2:
                    telemetry.addLine("   Next: Push first to storage, move to center");
                    telemetry.addLine("   Hardware: Transfer servos + Injectors ON");
                    break;
                case 3:
                    telemetry.addLine("   Next: Store in collection intake");
                    telemetry.addLine("   Hardware: NO transfer/injector activation");
                    break;
            }
            telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        }
    }

    /**
     * Update collection state
     */
    private void updateCollecting(long elapsedTime) {
        // Collection completes after intake roller time
        if (elapsedTime >= config.getIntakeRollerTimeMs()) {
            // Rollers continue running (don't stop), just transition state
            completeCollection();
        }
    }

    /**
     * Complete artifact collection and determine next action
     */
    private void completeCollection() {
        if (artifacts.isEmpty()) {
            setError("completeCollection called but no artifacts in list!");
            return;
        }

        Artifact artifact = artifacts.get(artifacts.size() - 1);

        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine("✅ Collection complete - artifact #" + artifact.getCollectionOrder());
            telemetry.addLine(String.format("   Total artifacts: %d, Next collection order: %d",
                getArtifactCount(), nextCollectionOrder));
        }

        switch (artifact.getCollectionOrder()) {
            case 1:
                // First artifact: goes to center storage
                if (config.isDebugTelemetry() && telemetry != null) {
                    telemetry.addLine("   → Taking path: startTransferToCenter (first artifact)");
                }
                startTransferToCenter(artifact);
                break;

            case 2:
                // Second artifact: behavior depends on manual push mode
                if (config.isManualPushMode()) {
                    // Manual push mode: second artifact stays in intake
                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine("   → Taking path: storeSecondArtifactInIntake (manual push mode)");
                    }
                    storeSecondArtifactInIntake(artifact);
                } else {
                    // Auto push mode: goes to center, will push first to opposite intake
                    if (config.isDebugTelemetry() && telemetry != null) {
                        telemetry.addLine("   → Taking path: startSecondArtifactIndexing (auto push mode)");
                    }
                    startSecondArtifactIndexing(artifact);
                }
                break;

            case 3:
                // Third artifact: stays in same intake it was collected from
                if (config.isDebugTelemetry() && telemetry != null) {
                    telemetry.addLine("   → Taking path: storeThirdArtifact (third artifact)");
                }
                storeThirdArtifact(artifact);
                break;

            default:
                setError("Invalid collection order: " + artifact.getCollectionOrder());
                break;
        }
    }

    /**
     * Transfer artifact to center storage
     */
    private void startTransferToCenter(Artifact artifact) {
        changeState(SystemState.TRANSFERRING);
        operationStartTime = System.currentTimeMillis();

        // Start hardware for transfer
        executeTransferHardware();

        if (config.isDebugTelemetry() && telemetry != null) {
            long totalTime = config.getTransferServoTimeMs() + config.getCenterAcceptTimeMs();
            telemetry.addLine("🔄 TRANSFERRING to center (will take " + (totalTime / 1000.0) + "s)");
            telemetry.addLine("   State: " + currentState + ", opInProgress: " + operationInProgress);
        }
    }

    /**
     * Update transferring state
     */
    private void updateTransferring(long elapsedTime) {
        if (elapsedTime >= config.getTransferServoTimeMs() + config.getCenterAcceptTimeMs()) {
            // Return transfer servos to idle
            setInjectorServos(false);
            setIntakeTransferServo(lastIntakeSource, false);
            completeTransferToCenter();
        }
    }

    /**
     * Complete transfer to center
     */
    private void completeTransferToCenter() {
        Artifact artifact = artifacts.get(artifacts.size() - 1);
        
        // Update artifact location
        Artifact updatedArtifact = artifact.withLocation(Artifact.Location.CENTER_STORAGE);
        artifacts.set(artifacts.size() - 1, updatedArtifact);
        artifactInCenter = updatedArtifact;

        // Reset uptake pre-position flag for new center artifact
        uptakeServoPrePositionedForCurrentArtifact = false;

        nextCollectionOrder++;

        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine(String.format("🔢 nextCollectionOrder incremented to %d (after first transfer)",
                nextCollectionOrder));
        }

        // ENSURE all servos are turned off after transfer
        resetAllServos();

        // Update intake modes after artifact placement
        updateIntakeModes();

        // Pre-positioning will be handled by the state machine logic in update()
        // No need to explicitly start it here to avoid conflicts

        if (updatedArtifact.getCollectionOrder() == 1) {
            // First artifact in center, ready for more collection
            changeState(SystemState.IDLE);
            operationInProgress = false;

            if (config.isDebugTelemetry()) {
                telemetry.addLine("✅ First artifact transfer complete - servos stopped");
                telemetry.addLine(String.format("   Center: %s #%d - ready for collection",
                    updatedArtifact.getColor(), updatedArtifact.getCollectionOrder()));
            }
        } else {
            // Ready to fire
            changeState(SystemState.READY_TO_FIRE);
            operationInProgress = false;

            if (config.isDebugTelemetry()) {
                telemetry.addLine("✅ Artifact transfer complete - ready to fire");
                telemetry.addLine(String.format("   Center: %s #%d",
                    updatedArtifact.getColor(), updatedArtifact.getCollectionOrder()));
            }
        }
    }

    /**
     * Handle second artifact indexing (pushes first to opposite intake)
     */
    private void startSecondArtifactIndexing(Artifact secondArtifact) {
        if (artifactInCenter == null || artifactInCenter.getCollectionOrder() != 1) {
            setError("Cannot index second artifact: first artifact not in center");
            return;
        }

        // Clear any stale pending artifacts to prevent conflicts during push
        clearPendingArtifacts();

        // Retract uptake servos during push operation to avoid interference
        if (uptakeServoPrePositioned) {
            retractUptakeServos();
        }

        // Determine opposite intake from where second artifact came
        Artifact.Location oppositeIntake = (lastIntakeSource == IntakeSource.FRONT)
            ? Artifact.Location.BACK_INTAKE
            : Artifact.Location.FRONT_INTAKE;

        // IMMEDIATELY update storage references to prevent auto-detection conflicts
        // The first artifact will be pushed to the opposite intake
        Artifact firstArtifact = artifactInCenter;
        Artifact movedFirst = firstArtifact.withLocation(oppositeIntake);

        // Update artifact list
        for (int i = 0; i < artifacts.size(); i++) {
            if (artifacts.get(i).getCollectionOrder() == 1) {
                artifacts.set(i, movedFirst);
                break;
            }
        }

        // Update storage references IMMEDIATELY
        if (oppositeIntake == Artifact.Location.FRONT_INTAKE) {
            artifactInFrontIntake = movedFirst;
        } else {
            artifactInBackIntake = movedFirst;
        }

        // Update intake modes immediately to prevent false detection
        updateIntakeModes();

        changeState(SystemState.PUSHING);
        operationStartTime = System.currentTimeMillis();

        // Start hardware for push operation
        executePushHardware();

        if (config.isDebugTelemetry()) {
            telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            telemetry.addLine("🔄 STARTING SECOND ARTIFACT INDEXING");
            telemetry.addLine(String.format("   Second artifact: %s from %s",
                secondArtifact.getColor(), lastIntakeSource));
            telemetry.addLine(String.format("   First artifact: %s → %s",
                firstArtifact.getColor(), oppositeIntake));
            telemetry.addLine("   Action: Push first out, move second to center");
            telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        }
    }

    /**
     * Update pushing state
     */
    private void updatePushing(long elapsedTime) {
        long totalPushTime = config.getPushStartDelayMs() + 
                            config.getSecondArtifactPushTimeMs() + 
                            config.getStorageIntakeAcceptTimeMs();

        if (elapsedTime >= totalPushTime) {
            // Reset servos to idle
            resetAllServos();
            completePushOperation();
        }
    }

    /**
     * Complete push operation
     * Handles normal second artifact indexing (first artifact pushed to storage, second to center)
     */
    private void completePushOperation() {
        // For normal second artifact collection, always complete second artifact indexing
        // The first artifact was already moved to storage in startSecondArtifactIndexing
        completeSecondArtifactIndexing();
    }
    
    /**
     * Complete normal second artifact indexing
     */
    private void completeSecondArtifactIndexing() {
        Artifact secondArtifact = artifacts.get(artifacts.size() - 1);

        // Move second artifact to center (first artifact already moved at start of push)
        Artifact secondInCenter = secondArtifact.withLocation(Artifact.Location.CENTER_STORAGE);
        artifacts.set(artifacts.size() - 1, secondInCenter);
        artifactInCenter = secondInCenter;

        // Reset uptake pre-position flag for new center artifact
        uptakeServoPrePositionedForCurrentArtifact = false;

        nextCollectionOrder++;

        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine(String.format("🔢 nextCollectionOrder incremented to %d (after second indexing)",
                nextCollectionOrder));
        }

        // ENSURE all servos are turned off after push operation
        resetAllServos();

        // Update intake modes (should already be correct from startSecondArtifactIndexing)
        updateIntakeModes();

        // Pre-positioning will be handled by the state machine logic in update()
        // No need to explicitly start it here to avoid conflicts

        // Shot planning removed - needs to be reimplemented

        changeState(SystemState.READY_TO_FIRE);
        operationInProgress = false;

        if (config.isDebugTelemetry()) {
            telemetry.addLine("✅ Second artifact indexing complete - servos stopped");
            telemetry.addLine(String.format("   Center: %s #%d", secondInCenter.getColor(), secondInCenter.getCollectionOrder()));

            // Show where first artifact went
            if (artifactInFrontIntake != null) {
                telemetry.addLine(String.format("   Front Storage: %s #%d", artifactInFrontIntake.getColor(), artifactInFrontIntake.getCollectionOrder()));
            }
            if (artifactInBackIntake != null) {
                telemetry.addLine(String.format("   Back Storage: %s #%d", artifactInBackIntake.getColor(), artifactInBackIntake.getCollectionOrder()));
            }
        }
    }
    
    /**
     * Complete two-artifact rearrangement
     * Center artifact pushed to empty intake, storage artifact pulled to center
     */
    private void completeTwoArtifactRearrangement() {
        // Find the artifact that was in storage (now being pulled to center)
        Artifact storageArtifact = null;
        Artifact centerArtifact = artifactInCenter;
        
        if (lastIntakeSource == IntakeSource.FRONT) {
            storageArtifact = artifactInFrontIntake;
        } else if (lastIntakeSource == IntakeSource.BACK) {
            storageArtifact = artifactInBackIntake;
        }
        
        if (storageArtifact == null || centerArtifact == null) {
            setError("Rearrangement failed: missing artifacts");
            return;
        }
        
        // Determine empty intake (opposite of storage source)
        Artifact.Location emptyIntake = (lastIntakeSource == IntakeSource.FRONT) 
            ? Artifact.Location.BACK_INTAKE 
            : Artifact.Location.FRONT_INTAKE;
        
        // Move center artifact to empty intake
        Artifact movedToEmpty = centerArtifact.withLocation(emptyIntake);
        for (int i = 0; i < artifacts.size(); i++) {
            if (artifacts.get(i).equals(centerArtifact)) {
                artifacts.set(i, movedToEmpty);
                break;
            }
        }
        
        // Update storage references for moved artifact
        if (emptyIntake == Artifact.Location.FRONT_INTAKE) {
            artifactInFrontIntake = movedToEmpty;
        } else {
            artifactInBackIntake = movedToEmpty;
        }
        
        // Clear the old storage location
        if (lastIntakeSource == IntakeSource.FRONT) {
            artifactInFrontIntake = null;
        } else {
            artifactInBackIntake = null;
        }
        
        // Move storage artifact to center
        Artifact movedToCenter = storageArtifact.withLocation(Artifact.Location.CENTER_STORAGE);
        for (int i = 0; i < artifacts.size(); i++) {
            if (artifacts.get(i).equals(storageArtifact)) {
                artifacts.set(i, movedToCenter);
                break;
            }
        }
        artifactInCenter = movedToCenter;
        
        // Reset uptake pre-position flag for new center artifact
        uptakeServoPrePositionedForCurrentArtifact = false;

        // Update intake modes
        updateIntakeModes();
        
        // Pre-positioning will be handled by the state machine logic in update()
        // No need to explicitly start it here to avoid conflicts

        // Now ready to fire the desired artifact
        changeState(SystemState.READY_TO_FIRE);
        operationInProgress = false;
        
        if (config.isDebugTelemetry()) {
            telemetry.addLine("Rearrangement complete: storage->center, center->empty");
        }
    }

    /**
     * Store third artifact in its collection intake
     */
    private void storeThirdArtifact(Artifact thirdArtifact) {
        Artifact.Location storageLocation = (lastIntakeSource == IntakeSource.FRONT)
            ? Artifact.Location.FRONT_INTAKE
            : Artifact.Location.BACK_INTAKE;

        // Check if intake is already occupied
        if ((storageLocation == Artifact.Location.FRONT_INTAKE && artifactInFrontIntake != null) ||
            (storageLocation == Artifact.Location.BACK_INTAKE && artifactInBackIntake != null)) {
            setError("Cannot store third artifact: intake already occupied");
            return;
        }

        Artifact stored = thirdArtifact.withLocation(storageLocation);
        artifacts.set(artifacts.size() - 1, stored);

        if (storageLocation == Artifact.Location.FRONT_INTAKE) {
            artifactInFrontIntake = stored;
        } else {
            artifactInBackIntake = stored;
        }

        nextCollectionOrder++;

        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine(String.format("🔢 nextCollectionOrder incremented to %d (after third storage)",
                nextCollectionOrder));
        }

        // IMPORTANT: Make sure all servos are idle since third artifact doesn't transfer
        resetAllServos();

        // Update intake modes (this intake now in storage mode)
        updateIntakeModes();

        // Shot planning removed - needs to be reimplemented

        // Keep system in READY_TO_FIRE state since we still have an artifact in center
        // Only go to IDLE if no artifact in center (which shouldn't happen for third artifact)
        if (artifactInCenter != null) {
            changeState(SystemState.READY_TO_FIRE);
        } else {
            changeState(SystemState.IDLE);
        }
        operationInProgress = false;

        if (config.isDebugTelemetry()) {
            telemetry.addLine(String.format("✅ Third artifact stored in %s", storageLocation));
            telemetry.addLine("   System remains READY_TO_FIRE with center artifact");
            telemetry.addLine("   All servos reset to idle - no transfer occurred");
        }
    }

    /**
     * Store second artifact in its collection intake (manual push mode)
     * Similar to storeThirdArtifact but for second artifact in manual mode
     */
    private void storeSecondArtifactInIntake(Artifact secondArtifact) {
        Artifact.Location storageLocation = (lastIntakeSource == IntakeSource.FRONT)
            ? Artifact.Location.FRONT_INTAKE
            : Artifact.Location.BACK_INTAKE;

        // Check if intake is already occupied (shouldn't happen in manual mode)
        if ((storageLocation == Artifact.Location.FRONT_INTAKE && artifactInFrontIntake != null) ||
            (storageLocation == Artifact.Location.BACK_INTAKE && artifactInBackIntake != null)) {
            setError("Cannot store second artifact: intake already occupied");
            return;
        }

        Artifact stored = secondArtifact.withLocation(storageLocation);
        artifacts.set(artifacts.size() - 1, stored);

        if (storageLocation == Artifact.Location.FRONT_INTAKE) {
            artifactInFrontIntake = stored;
        } else {
            artifactInBackIntake = stored;
        }

        nextCollectionOrder++;

        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine(String.format("🔢 nextCollectionOrder incremented to %d (after second storage - manual mode)",
                nextCollectionOrder));
        }

        // IMPORTANT: Make sure all servos are idle since second artifact doesn't transfer in manual mode
        resetAllServos();

        // Update intake modes (this intake now in storage mode)
        updateIntakeModes();

        // Shot planning removed - needs to be reimplemented

        // System goes to READY_TO_FIRE with first artifact still in center
        changeState(SystemState.READY_TO_FIRE);
        operationInProgress = false;

        if (config.isDebugTelemetry()) {
            telemetry.addLine(String.format("✅ Second artifact stored in %s (manual push mode)", storageLocation));
            telemetry.addLine("   System READY_TO_FIRE with first artifact in center");
            telemetry.addLine("   All servos reset to idle - no transfer occurred");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIRING LOGIC - REMOVED
    // ═══════════════════════════════════════════════════════════════════════
    // All firing logic has been removed and needs to be reimplemented
    // The following stubs are placeholders for future implementation

    /**
     * ═══════════════════════════════════════════════════════════════════════
     * Manual push functionality - only available when exactly 2 artifacts present
     * Pushes first artifact from center to opposite intake, moves second from storage to center
     * @return true if manual push started successfully
     */
    public boolean onManualPush() {
        // Can only manual push with exactly 2 artifacts
        if (getArtifactCount() != 2) {
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine(String.format("Cannot manual push: need exactly 2 artifacts, have %d", getArtifactCount()));
            }
            return false;
        }

        // Must be in READY_TO_FIRE state
        if (currentState != SystemState.READY_TO_FIRE) {
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine("Cannot manual push: system not ready to fire");
            }
            return false;
        }

        // Can't push during another operation
        if (operationInProgress) {
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine("Cannot manual push: operation in progress");
            }
            return false;
        }

        // Must have artifact in center and one in storage
        if (artifactInCenter == null) {
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine("Cannot manual push: no artifact in center");
            }
            return false;
        }

        // Find the artifact in storage and determine source
        IntakeSource storageSource = IntakeSource.UNKNOWN;
        if (artifactInFrontIntake != null && artifactInBackIntake == null) {
            storageSource = IntakeSource.FRONT;
        } else if (artifactInBackIntake != null && artifactInFrontIntake == null) {
            storageSource = IntakeSource.BACK;
        } else {
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine("Cannot manual push: invalid storage configuration");
            }
            return false;
        }

        // Start manual push operation (similar to normal second artifact indexing)
        Artifact centerArtifact = artifactInCenter;
        Artifact storageArtifact = (storageSource == IntakeSource.FRONT) ? artifactInFrontIntake : artifactInBackIntake;

        // Determine opposite intake from storage source
        Artifact.Location oppositeIntake = (storageSource == IntakeSource.FRONT)
            ? Artifact.Location.BACK_INTAKE
            : Artifact.Location.FRONT_INTAKE;

        // IMMEDIATELY update storage references to prevent auto-detection conflicts
        // The center artifact will be pushed to the opposite intake
        Artifact movedCenter = centerArtifact.withLocation(oppositeIntake);

        // Update artifact list
        for (int i = 0; i < artifacts.size(); i++) {
            if (artifacts.get(i).equals(centerArtifact)) {
                artifacts.set(i, movedCenter);
                break;
            }
        }

        // Update storage references IMMEDIATELY
        if (oppositeIntake == Artifact.Location.FRONT_INTAKE) {
            artifactInFrontIntake = movedCenter;
        } else {
            artifactInBackIntake = movedCenter;
        }

        // Clear the old storage location (storage artifact will move to center)
        if (storageSource == IntakeSource.FRONT) {
            artifactInFrontIntake = null;
        } else {
            artifactInBackIntake = null;
        }

        // Update intake modes immediately to prevent false detection
        updateIntakeModes();

        changeState(SystemState.PUSHING);
        operationInProgress = true;
        operationStartTime = System.currentTimeMillis();

        // Set lastIntakeSource to the storage source for proper hardware control
        lastIntakeSource = storageSource;

        // Start hardware for manual push operation
        executePushHardware();

        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            telemetry.addLine("🔄 MANUAL PUSH OPERATION");
            telemetry.addLine(String.format("   Center artifact: %s → %s",
                centerArtifact.getColor(), oppositeIntake));
            telemetry.addLine(String.format("   Storage artifact: %s from %s → center",
                storageArtifact.getColor(), storageSource));
            telemetry.addLine("   Action: Manual push - center out, storage to center");
            telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        }

        return true;
    }

    /**
     * Get manual push mode status
     * @return true if manual push mode is enabled
     */
    public boolean isManualPushMode() {
        return config.isManualPushMode();
    }

    /**
     * Set manual push mode (can also be controlled via gamepad)
     * @param enabled true to enable manual push mode
     */
    public void setManualPushMode(boolean enabled) {
        config.setManualPushMode(enabled);
        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine("Manual push mode " + (enabled ? "ENABLED" : "DISABLED"));
            if (enabled) {
                telemetry.addLine("   Second artifacts will stay in intake");
                telemetry.addLine("   Use manual push when ready");
            } else {
                telemetry.addLine("   Second artifacts will auto-push to center");
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATE MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════

    private void changeState(SystemState newState) {
        currentState = newState;
        stateStartTime = System.currentTimeMillis();
    }

    private void resetToIdle() {
        // CRITICAL: Stop all servos before going to idle
        resetAllServos();

        // Explicitly reset uptake servos and their state
        resetUptakeServos();

        currentState = SystemState.IDLE;
        operationInProgress = false;
        stateStartTime = System.currentTimeMillis();

        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine("🔄 System reset to IDLE - servos stopped");
        }
    }

    private void setError(String error) {
        lastError = error;
        errorCount++;
        currentState = SystemState.ERROR;
        operationInProgress = false;
        
        if (telemetry != null) {
            telemetry.addLine("⚠️ Indexing Error: " + error);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS - SYSTEM STATE
    // ═══════════════════════════════════════════════════════════════════════

    public SystemState getCurrentState() { return currentState; }
    public int getArtifactCount() {
        int count = 0;
        for (Artifact a : artifacts) {
            if (a.getLocation() != Artifact.Location.FIRED) {
                count++;
            }
        }
        return count;
    }

    public Artifact getArtifactInCenter() { return artifactInCenter; }
    public Artifact getArtifactInFrontIntake() { return artifactInFrontIntake; }
    public Artifact getArtifactInBackIntake() { return artifactInBackIntake; }
    
    public List<Artifact> getAllArtifacts() { return new ArrayList<>(artifacts); }
    
    public boolean isReadyToFire() {
        // Ready to fire if we have an artifact in center and are in IDLE or READY_TO_FIRE state
        // IDLE state allows continued artifact collection while still being able to fire
        // READY_TO_FIRE state is the traditional ready state
        return (currentState == SystemState.IDLE || currentState == SystemState.READY_TO_FIRE) 
               && artifactInCenter != null;
    }
    
    public boolean isOperationInProgress() { return operationInProgress; }
    
    /**
     * Get uptake servo pre-positioning status
     * @return true if uptake servos are currently pre-positioned
     */
    public boolean isUptakeServoPrePositioned() { return uptakeServoPrePositioned; }

    /**
     * Get uptake servo action time for debug purposes
     * @return action time in milliseconds, or 0 if not active
     */
    public long getUptakeServoActionTime() { return uptakeServoActionTime; }

    /**
     * Get uptake servo completion flag for this artifact
     * @return true if pre-positioning has been completed for current center artifact
     */
    public boolean isUptakeServoCompletedForCurrentArtifact() {
        return uptakeServoPrePositionedForCurrentArtifact;
    }

    /**
     * Get remaining time for uptake servo pre-positioning
     * @return remaining time in milliseconds, or 0 if not active
     */
    public long getUptakeServoRemainingTime() {
        if (!uptakeServoPrePositioned || uptakeServoActionTime <= 0) {
            return 0;
        }
        long elapsed = System.currentTimeMillis() - uptakeServoActionTime;
        long timeout = ShooterConfig.UPTAKE_PREPOSITION_TIMEOUT_MS;
        return Math.max(0, timeout - elapsed);
    }

    /**
     * Get elapsed time for uptake servo pre-positioning
     * @return elapsed time in milliseconds, or 0 if not active
     */
    public long getUptakeServoElapsedTime() {
        if (!uptakeServoPrePositioned || uptakeServoActionTime <= 0) {
            return 0;
        }
        return System.currentTimeMillis() - uptakeServoActionTime;
    }

    public Shooter getShooter() { return shooter; }

    /**
     * Enable or disable automatic artifact detection
     * @param enabled true to enable auto-detection, false to disable
     */
    public void setAutoDetectionEnabled(boolean enabled) {
        this.autoDetectionEnabled = enabled;
        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine("Auto-detection " + (enabled ? "ENABLED" : "DISABLED"));
        }
    }

    /**
     * Check if automatic detection is enabled
     * @return true if auto-detection is active
     */
    public boolean isAutoDetectionEnabled() {
        return autoDetectionEnabled;
    }

    public double getFireFeedTime() {
        return config.getFireFeedTime();
    }

    public String getLastError() { return lastError; }
    public int getErrorCount() { return errorCount; }

    public Artifact getPlannedSecondShot() { return plannedSecondShot; }
    public Artifact getPlannedThirdShot() { return plannedThirdShot; }
    public Artifact getPlannedFirstShot() { return plannedFirstShot; }

    /**
     * Add a debug message to the queue for opmode display
     */
    private void addDebugMessage(String message) {
        // Add timestamp to message for precise timing analysis
        long currentTime = System.currentTimeMillis();
        long relativeTime = currentTime - debugStartTime;
        String timestampedMessage = String.format("[+%dms] %s", relativeTime, message);
        debugMessages.offer(timestampedMessage);
        // Keep only the most recent messages
        while (debugMessages.size() > MAX_DEBUG_MESSAGES) {
            debugMessages.poll();
        }
    }

    /**
     * Get recent debug messages for opmode display
     * @return List of recent debug messages
     */
    public java.util.List<String> getRecentDebugMessages() {
        return new java.util.ArrayList<>(debugMessages);
    }

    /**
     * Clear debug message history
     */
    public void clearDebugMessages() {
        debugMessages.clear();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE CONTROL METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Run intake roller motor at specified power
     * Rollers are the main collection mechanism - they roll inward continuously.
     * They run at full power during collection and slower when intake is in storage mode.
     * @param source Which intake to run
     * @param power Motor power (0.0 to 1.0, always inward/positive)
     */
    private void setIntakePower(IntakeSource source, double power) {
        if (hardware == null) return;

        try {
            if (source == IntakeSource.FRONT && hardware.getFrontRollerMotor() != null) {
                hardware.getFrontRollerMotor().setPower(power);
            } else if (source == IntakeSource.BACK && hardware.getBackRollerMotor() != null) {
                hardware.getBackRollerMotor().setPower(power);
            }
        } catch (Exception e) {
            setError("Failed to set intake power: " + e.getMessage());
        }
    }
    
    /**
     * Set intake to storage mode - rollers run slower to hold artifact
     * @param source Which intake to set to storage mode
     */
    private void setIntakeStorageMode(IntakeSource source) {
        // Run rollers at configured storage power to maintain artifact in storage
        setIntakePower(source, config.getIntakeStoragePower());
    }
    
    /**
     * Set intake to collection mode - rollers run at full speed
     * @param source Which intake to set to collection mode
     */
    private void setIntakeCollectionMode(IntakeSource source) {
        // Run rollers at full power for collection
        setIntakePower(source, config.getIntakeRollerPower());
    }

    /**
     * Set injector servo power (CRServos)
     * These servos move artifacts between transfer system and center storage,
     * completing the move from intake transfer into center,
     * and also push artifacts out of center into an empty intake.
     * Note: Direction depends on which intake is being used.
     * @param active true to activate transfer (run at power), false for idle (stop)
     * @param source which intake is being used (affects servo direction)
     */
    private void setInjectorServos(boolean active, IntakeSource source) {
        if (hardware == null) return;

        try {
            double basePower = active ? config.getTransferServoPower() : config.getTransferServoIdlePower();

            // Reverse servo directions when collecting from back intake
            boolean reverseDirection = (source == IntakeSource.BACK);

            if (hardware.getInjectorServoLeft() != null) {
                // Left servo base configuration: reversed (facing opposite direction)
                // For back intake: reverse this again (double negative = positive)
                double leftPower = reverseDirection ? basePower : -basePower;
                hardware.getInjectorServoLeft().setPower(leftPower);
            }
            if (hardware.getInjectorServoRight() != null) {
                // Right servo base configuration: normal
                // For back intake: reverse this
                double rightPower = reverseDirection ? -basePower : basePower;
                hardware.getInjectorServoRight().setPower(rightPower);
            }
        } catch (Exception e) {
            setError("Failed to set injector servos: " + e.getMessage());
        }
    }

    /**
     * Set injector servos without source context (uses front intake behavior)
     * @param active true to activate transfer, false for idle
     */
    private void setInjectorServos(boolean active) {
        setInjectorServos(active, IntakeSource.FRONT);
    }

    /**
     * Set uptake servo power (CRServos)
     * These servos feed artifacts from the center slot UP into the shooter.
     * They are the only servos that can move an artifact sitting in center slot.
     * @param active true to activate uptake (run at power), false for idle (stop)
     */
    private void setUptakeServos(boolean active) {
        if (hardware == null) return;

        try {
            double power = active ? config.getTransferServoPower() : config.getTransferServoIdlePower();

            // DEBUG: Track ALL calls to this method, especially when setting to 0 power
            if (config.isDebugTelemetry() && telemetry != null) {
                StackTraceElement[] stack = Thread.currentThread().getStackTrace();
                String caller = "UNKNOWN";
                if (stack.length > 2) {
                    StackTraceElement element = stack[2];
                    caller = element.getMethodName() + ":" + element.getLineNumber();
                }

                // Always log when setting to 0 power, and log other calls if pre-positioning is active
                if (!active || uptakeServoPrePositioned) {
                    String callMsg = String.format("🔧 📞 setUptakeServos(%s, power=%.2f) called by %s", active, power, caller);
                    addDebugMessage(callMsg);
                }
            }

            // DEBUG: Track if this method is being called when pre-positioning is active (CONFLICT)
            if (!active && uptakeServoPrePositioned && config.isDebugTelemetry() && telemetry != null) {
                StackTraceElement[] stack = Thread.currentThread().getStackTrace();
                String caller = "UNKNOWN";
                if (stack.length > 2) {
                    StackTraceElement element = stack[2];
                    caller = element.getMethodName() + ":" + element.getLineNumber();
                }
                String message = "⚠️ CONFLICT: setUptakeServos(false) called during pre-positioning by " + caller;
                telemetry.addLine(message);
                addDebugMessage(message);
            }

            if (hardware.getUptakeServoL() != null) {
                hardware.getUptakeServoL().setPower(power);
                if (config.isDebugTelemetry() && telemetry != null && (!active || uptakeServoPrePositioned)) {
                    String msg = String.format("🔧 🎛️ LEFT SERVO (setUptakeServos): Set to %.2f power", power);
                    addDebugMessage(msg);
                }
            }
            if (hardware.getUptakeServoR() != null) {
                hardware.getUptakeServoR().setPower(power);
                if (config.isDebugTelemetry() && telemetry != null && (!active || uptakeServoPrePositioned)) {
                    String msg = String.format("🔧 🎛️ RIGHT SERVO (setUptakeServos): Set to %.2f power", power);
                    addDebugMessage(msg);
                }
            }

        } catch (Exception e) {
            setError("Failed to set uptake servos: " + e.getMessage());
        }
    }

    /**
     * Set intake transfer servo power (CRServo)
     * These servos transfer artifacts from the intake into the center.
     * @param source Which intake transfer servo to control
     * @param active true to activate transfer (run at power), false for idle (stop)
     */
    private void setIntakeTransferServo(IntakeSource source, boolean active) {
        if (hardware == null) return;

        try {
            double power = active ? config.getTransferServoPower() : config.getTransferServoIdlePower();

            if (source == IntakeSource.FRONT && hardware.getFrontTransferServo() != null) {
                hardware.getFrontTransferServo().setPower(power);
            } else if (source == IntakeSource.BACK && hardware.getBackTransferServo() != null) {
                hardware.getBackTransferServo().setPower(power);
            }
        } catch (Exception e) {
            setError("Failed to set intake transfer servo: " + e.getMessage());
        }
    }

    /**
     * Set uptake servos to pre-position mode (slightly up for quick firing)
     * @param enable true to pre-position up, false to return to idle
     */
    private void setUptakeServoPrePosition(boolean enable) {
        if (hardware == null || shooter == null) return;


        try {
            if (enable) {
                // Start pre-positioning with gentle upward power
                double power = ShooterConfig.UPTAKE_PREPOSITION_POWER; // Use config value

                if (hardware.getUptakeServoL() != null) {
                    hardware.getUptakeServoL().setPower(power);
                    if (config.isDebugTelemetry() && telemetry != null) {
                        String msg = String.format("🔧 ⚡ LEFT SERVO: Set to %.2f power", power);
                        addDebugMessage(msg);
                    }
                }
                if (hardware.getUptakeServoR() != null) {
                    hardware.getUptakeServoR().setPower(power);
                    if (config.isDebugTelemetry() && telemetry != null) {
                        String msg = String.format("🔧 ⚡ RIGHT SERVO: Set to %.2f power", power);
                        addDebugMessage(msg);
                    }
                }

                uptakeServoPrePositioned = true;
                uptakeServoActionTime = System.currentTimeMillis();

                if (config.isDebugTelemetry() && telemetry != null) {
                    String message = String.format("🔧 ✅ UPTAKE START: Pre-positioning started (actionTime: %d)", uptakeServoActionTime);
                    telemetry.addLine(message);
                    addDebugMessage(message);
                }
            } else {
                // Stop pre-positioning - return to idle
                if (hardware.getUptakeServoL() != null) {
                    hardware.getUptakeServoL().setPower(0.0);
                    if (config.isDebugTelemetry() && telemetry != null) {
                        String msg = "🔧 🔌 LEFT SERVO: Set to 0.0 power (STOPPED)";
                        addDebugMessage(msg);
                    }
                }
                if (hardware.getUptakeServoR() != null) {
                    hardware.getUptakeServoR().setPower(0.0);
                    if (config.isDebugTelemetry() && telemetry != null) {
                        String msg = "🔧 🔌 RIGHT SERVO: Set to 0.0 power (STOPPED)";
                        addDebugMessage(msg);
                    }
                }

                uptakeServoPrePositioned = false;

                // DEBUG: Track what's calling this to stop pre-positioning
                if (config.isDebugTelemetry() && telemetry != null) {
                    StackTraceElement[] stack = Thread.currentThread().getStackTrace();
                    String caller = "UNKNOWN";
                    if (stack.length > 2) {
                        StackTraceElement element = stack[2]; // Skip getStackTrace() and this method
                        caller = element.getMethodName() + ":" + element.getLineNumber();
                    }
                    String message = "🔧 ❌ UPTAKE STOP: Pre-positioning ended by " + caller;
                    telemetry.addLine(message);
                    addDebugMessage(message);
                }
            }
        } catch (Exception e) {
            setError("Failed to control uptake servos: " + e.getMessage());
        }
    }

    /**
     * Set uptake servos to retract mode (down, away from shooter)
     * Used during push operations to avoid interference
     */
    private void retractUptakeServos() {
        if (hardware == null) return;

        try {
            double retractPower = -0.5; // Retract downward

            if (hardware.getUptakeServoL() != null) {
                hardware.getUptakeServoL().setPower(retractPower);
                if (config.isDebugTelemetry() && telemetry != null) {
                    String msg = String.format("🔧 ⬇️ LEFT SERVO (retract): Set to %.2f power (RETRACT)", retractPower);
                    addDebugMessage(msg);
                }
            }
            if (hardware.getUptakeServoR() != null) {
                hardware.getUptakeServoR().setPower(retractPower);
                if (config.isDebugTelemetry() && telemetry != null) {
                    String msg = String.format("🔧 ⬇️ RIGHT SERVO (retract): Set to %.2f power (RETRACT)", retractPower);
                    addDebugMessage(msg);
                }
            }

            uptakeServoPrePositioned = false;
            // FIXED: Clear action time instead of resetting it to prevent timing interference
            uptakeServoActionTime = 0;

            if (config.isDebugTelemetry() && telemetry != null) {
                String message = "🔧 ⚠️ UPTAKE RETRACTED: Push operation interference!";
                telemetry.addLine(message);
                addDebugMessage(message);
            }
        } catch (Exception e) {
            setError("Failed to retract uptake servos: " + e.getMessage());
        }
    }

    /**
     * Update uptake servo pre-positioning timeout
     * Automatically stops pre-positioning after the configured time to prevent continuous running
     */
    private void updateUptakeServoTimeout(long currentTime) {
        if (uptakeServoPrePositioned && uptakeServoActionTime > 0) {
            // Check if pre-positioning timeout has elapsed
            long elapsedTime = currentTime - uptakeServoActionTime;
            long timeoutMs = ShooterConfig.UPTAKE_PREPOSITION_TIMEOUT_MS; // Use config value

            // Enhanced debug telemetry to track timing issues
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addData("🔧 Uptake Timer", String.format("%.0f/%.0fms (%.1fs)",
                    (double)elapsedTime, (double)timeoutMs, elapsedTime / 1000.0));
                telemetry.addData("🔧 Times", String.format("Current: %d, Action: %d, Diff: %d",
                    currentTime, uptakeServoActionTime, elapsedTime));
            }

            // Use >= for timeout to ensure we don't overshoot significantly
            // The slight overshoot (590ms vs 500ms) is normal due to loop timing
            if (elapsedTime >= timeoutMs) {
                // Add debug info BEFORE calling setUptakeServoPrePosition
                if (config.isDebugTelemetry() && telemetry != null) {
                    String debugMsg = String.format("🔧 ⏰ TIMEOUT TRIGGER: %.0fms elapsed (>= %.0fms timeout)",
                        (double)elapsedTime, (double)timeoutMs);
                    telemetry.addLine(debugMsg);
                    addDebugMessage(debugMsg);
                }

                // Pre-positioning timeout reached - stop servos
                setUptakeServoPrePosition(false);

                // Mark as completed for this artifact so it won't restart
                uptakeServoPrePositionedForCurrentArtifact = true;

                if (config.isDebugTelemetry() && telemetry != null) {
                    String timeoutMsg = String.format("🔧 ✅ TIMEOUT COMPLETE: Pre-positioning ran for %.0fms (target: %.0fms)",
                        (double)elapsedTime, (double)timeoutMs);
                    telemetry.addLine(timeoutMsg);
                    addDebugMessage(timeoutMsg);
                }
            }
        } else if (config.isDebugTelemetry() && telemetry != null && uptakeServoPrePositioned) {
            // If pre-positioned but no action time, that's suspicious
            telemetry.addData("🔧 Uptake Warning", "Pre-positioned but no action time!");
        }
    }
    /**
     * Reset only transfer and injector servos after operations
     * Uptake servos are managed separately by pre-positioning logic
     */
    private void resetAllServos() {
        setInjectorServos(false); // Stop (power = 0)
        setIntakeTransferServo(IntakeSource.FRONT, false);
        setIntakeTransferServo(IntakeSource.BACK, false);

        // Don't reset uptake servos here - let pre-positioning logic handle them
        // This prevents interference with the 500ms pre-positioning cycle
    }

    /**
     * Explicitly stop and reset uptake servos (for system reset, errors, etc.)
     */
    private void resetUptakeServos() {
        if (hardware != null) {
            try {
                if (hardware.getUptakeServoL() != null) {
                    hardware.getUptakeServoL().setPower(0.0);
                    if (config.isDebugTelemetry() && telemetry != null) {
                        String msg = "🔧 🔄 LEFT SERVO (resetUptakeServos): Set to 0.0 power (RESET)";
                        addDebugMessage(msg);
                    }
                }
                if (hardware.getUptakeServoR() != null) {
                    hardware.getUptakeServoR().setPower(0.0);
                    if (config.isDebugTelemetry() && telemetry != null) {
                        String msg = "🔧 🔄 RIGHT SERVO (resetUptakeServos): Set to 0.0 power (RESET)";
                        addDebugMessage(msg);
                    }
                }
            } catch (Exception e) {
                // Ignore errors during servo reset
            }
        }

        // DEBUG: Track when this method is called
        if (config.isDebugTelemetry() && telemetry != null) {
            StackTraceElement[] stack = Thread.currentThread().getStackTrace();
            String caller = "UNKNOWN";
            if (stack.length > 2) {
                StackTraceElement element = stack[2];
                caller = element.getMethodName() + ":" + element.getLineNumber();
            }
            String message = "🔧 🔄 UPTAKE RESET: Called by " + caller;
            telemetry.addLine(message);
            addDebugMessage(message);
        }

        uptakeServoPrePositioned = false;
        uptakeServoActionTime = 0;
        uptakeServoPrePositionedForCurrentArtifact = false;
    }
    
    /**
     * Set intake modes based on current artifact storage
     * Intakes with artifacts run in storage mode (slower), empty intakes run normally
     */
    private void updateIntakeModes() {
        // Front intake mode
        if (artifactInFrontIntake != null) {
            setIntakeStorageMode(IntakeSource.FRONT);
        } else {
            setIntakeCollectionMode(IntakeSource.FRONT);
        }
        
        // Back intake mode
        if (artifactInBackIntake != null) {
            setIntakeStorageMode(IntakeSource.BACK);
        } else {
            setIntakeCollectionMode(IntakeSource.BACK);
        }
    }

    /**
     * Check if artifact is detected by distance and color sensors
     * An artifact is detected if:
     * - Distance < 10cm (configurable) using goBILDA laser sensor in analog mode
     * - Color is GREEN or PURPLE (valid artifact colors only)
     * @param source Which intake to check
     * @return true if artifact detected
     */
    private boolean isArtifactDetected(IntakeSource source) {
        if (hardware == null) return false;

        try {
            // Check distance first using goBILDA laser sensor (analog mode)
            // Sensor outputs 0-3.3V corresponding to 0-1000mm
            double distanceMM = Double.MAX_VALUE;
            if (source == IntakeSource.FRONT) {
                distanceMM = hardware.getFrontDistanceMM();
            } else if (source == IntakeSource.BACK) {
                distanceMM = hardware.getBackDistanceMM();
            }
            
            // Convert threshold from cm to mm for comparison
            double thresholdMM = config.getArtifactDetectionDistance() * 10.0;
            
            // Check if distance is within threshold (default 100mm = 10cm)
            if (distanceMM < 0 || distanceMM >= thresholdMM) {
                return false; // Sensor not available or too far, no artifact
            }
            
            // Check color to ensure it's a valid artifact (GREEN or PURPLE only)
            Artifact.Color detectedColor = detectArtifactColor(source);
            if (detectedColor != Artifact.Color.GREEN && detectedColor != Artifact.Color.PURPLE) {
                return false; // Invalid color - not a valid artifact
            }
            
            // Distance is close and color is GREEN or PURPLE - valid artifact detected
            return true;
        } catch (Exception e) {
            // Sensor not available or error
        }

        return false;
    }

    /**
     * Detect artifact color from color sensors
     * NOTE: Front Left and Back Right sensors replaced with REV 2m distance sensors
     * Uses remaining REV Color Sensor V3 per intake for accurate color data
     * @param source Which intake sensor to check
     * @return Detected artifact color (PURPLE, GREEN, or UNKNOWN)
     */
    private Artifact.Color detectArtifactColor(IntakeSource source) {
        if (hardware == null) return Artifact.Color.UNKNOWN;

        try {
            // Collect readings from available color sensors for the intake (REV Color Sensor V3)
            List<com.qualcomm.robotcore.hardware.NormalizedColorSensor> sensors = new ArrayList<>();
            
            if (source == IntakeSource.FRONT) {
                // NOTE: Front Left replaced with REV 2m distance sensor
                if (hardware.getFrontLeftColorSensor() != null) sensors.add(hardware.getFrontLeftColorSensor());
                if (hardware.getFrontRightColorSensor() != null) sensors.add(hardware.getFrontRightColorSensor());
                if (hardware.getFrontCenterColorSensor() != null) sensors.add(hardware.getFrontCenterColorSensor());
            } else if (source == IntakeSource.BACK) {
                // NOTE: Back Right replaced with REV 2m distance sensor
                if (hardware.getBackRightColorSensor() != null) sensors.add(hardware.getBackRightColorSensor());
                if (hardware.getLeftRightColorSensor() != null) sensors.add(hardware.getLeftRightColorSensor());
                if (hardware.getBackCenterColorSensor() != null) sensors.add(hardware.getBackCenterColorSensor());
            }

            if (sensors.isEmpty()) {
                if (config.isDebugTelemetry() && telemetry != null) {
                    telemetry.addLine("⚠️ No color sensors available for " + source + " intake");
                }
                return Artifact.Color.UNKNOWN;
            }

            // Collect normalized color readings from all available sensors (0-1 range)
            float totalRed = 0, totalGreen = 0, totalBlue = 0;
            for (com.qualcomm.robotcore.hardware.NormalizedColorSensor sensor : sensors) {
                NormalizedRGBA colors = sensor.getNormalizedColors();
                totalRed += colors.red;
                totalGreen += colors.green;
                totalBlue += colors.blue;
            }

            // Average the readings
            float avgRed = totalRed / sensors.size();
            float avgGreen = totalGreen / sensors.size();
            float avgBlue = totalBlue / sensors.size();

            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine(String.format("🎨 %s color: R%.2f G%.2f B%.2f (%d sensors)",
                    source, avgRed, avgGreen, avgBlue, sensors.size()));
            }

            // Use IndexingConfig color detection with measured RGB thresholds
            String detectedColor = config.detectArtifactColor(avgRed, avgGreen, avgBlue);

            if ("PURPLE".equals(detectedColor)) {
                // Verify confidence meets minimum threshold
                double confidence = config.calculateColorConfidence(avgRed, avgGreen, avgBlue, "PURPLE");
                if (confidence >= config.getColorDetectionMinScore()) {
                    return Artifact.Color.PURPLE;
                }
            } else if ("GREEN".equals(detectedColor)) {
                // Verify confidence meets minimum threshold
                double confidence = config.calculateColorConfidence(avgRed, avgGreen, avgBlue, "GREEN");
                if (confidence >= config.getColorDetectionMinScore()) {
                    return Artifact.Color.GREEN;
                }
            }
        } catch (Exception e) {
            // Sensor not available or error
        }

        return Artifact.Color.UNKNOWN;
    }

    /**
     * Execute hardware actions for collection state
     * Rollers continue running, intake transfer servo moves artifact to center,
     * injector servos accept and complete the transfer.
     */
    private void executeCollectionHardware() {
        // Intake rollers already running continuously (in collection mode)
        setIntakeCollectionMode(lastIntakeSource);
        
        // Activate intake transfer servo to move artifact from intake to center
        setIntakeTransferServo(lastIntakeSource, true);
        
        // Activate injector servos to accept artifact from intake transfer
        setInjectorServos(true, lastIntakeSource);
    }

    /**
     * Execute hardware actions for third artifact collection
     * Third artifact should NOT be transferred - it stays in the intake
     * Only keep intake rollers running to hold the artifact in place
     */
    private void executeThirdArtifactCollectionHardware() {
        // Keep intake rollers running at collection speed temporarily
        setIntakeCollectionMode(lastIntakeSource);

        // DO NOT activate transfer servos or injectors for third artifact
        // Third artifact stays in the intake where it was collected

        if (config.isDebugTelemetry() && telemetry != null) {
            telemetry.addLine("🔄 Third artifact: No transfer hardware activated");
            telemetry.addLine("   Artifact will remain in collection intake");
        }
    }

    /**
     * Execute hardware actions for transferring state
     * Continue the transfer process with servos active
     */
    private void executeTransferHardware() {
        // Intake rollers continue running to push artifact through
        setIntakeCollectionMode(lastIntakeSource);
        
        // Keep transfer servos active
        setIntakeTransferServo(lastIntakeSource, true);
        setInjectorServos(true, lastIntakeSource);
    }

    /**
     * Execute hardware actions for pushing state
     * Second artifact pushes first artifact from center into opposite (empty) intake.
     * Center servos push the artifact out, opposite intake accepts it.
     */
    private void executePushHardware() {
        IntakeSource oppositeIntake = (lastIntakeSource == IntakeSource.FRONT) 
            ? IntakeSource.BACK 
            : IntakeSource.FRONT;
        
        // Collecting intake continues at collection speed
        setIntakeCollectionMode(lastIntakeSource);
        
        // Opposite (empty) intake runs to accept pushed artifact
        setIntakeCollectionMode(oppositeIntake);
        
        // Activate intake transfer servo on collecting side
        setIntakeTransferServo(lastIntakeSource, true);
        
        // Opposite intake transfer servo ready to receive
        setIntakeTransferServo(oppositeIntake, true);
        
        // Injector servos push artifact out to opposite intake
        // Use the source intake (where second artifact came from) for servo direction
        setInjectorServos(true, lastIntakeSource);
    }

    // executeFiringHardware() method removed - needs to be reimplemented

    /**
     * Handle automatic artifact detection - monitors sensors and triggers collection
     * This runs internally during update() and manages the entire detection process
     */
    private void handleAutomaticDetection(long currentTime) {
        // Check sensors at controlled intervals to prevent spam
        if (currentTime - lastSensorCheck > SENSOR_CHECK_INTERVAL) {
            lastSensorCheck = currentTime;

            handleFrontIntakeAutoDetection();
            handleBackIntakeAutoDetection();
        }
    }

    /**
     * Handle automatic front intake detection
     */
    private void handleFrontIntakeAutoDetection() {
        boolean artifactPresent = isArtifactDetectedAtIntake(IntakeSource.FRONT);
        boolean intakeEmpty = artifactInFrontIntake == null;
        boolean systemIdle = currentState == SystemState.IDLE;
        boolean noOperationInProgress = !operationInProgress;

        // Allow third artifact collection when in READY_TO_FIRE state with 2 artifacts
        // Additional safety: ensure we have one in center and one in storage, with front intake empty for third
        boolean canCollectThirdArtifact = (currentState == SystemState.READY_TO_FIRE) &&
                                        (getArtifactCount() == 2) &&
                                        !operationInProgress &&
                                        (artifactInCenter != null) &&  // Must have artifact in center
                                        (artifactInFrontIntake == null) &&  // Front must be empty for collection
                                        (artifactInBackIntake != null);  // Back must have the stored second artifact

        if (config.isDebugTelemetry() && telemetry != null && canCollectThirdArtifact && artifactPresent && intakeEmpty) {
            telemetry.addLine("🔍 FRONT: THIRD ARTIFACT DETECTION TRIGGER");
            telemetry.addLine(String.format("   State: %s, Count: %d, NextOrder: %d",
                currentState, getArtifactCount(), nextCollectionOrder));
            telemetry.addLine(String.format("   Center: %s, Front: %s, Back: %s",
                artifactInCenter != null ? artifactInCenter.getColor() : "Empty",
                artifactInFrontIntake != null ? artifactInFrontIntake.getColor() : "Empty",
                artifactInBackIntake != null ? artifactInBackIntake.getColor() : "Empty"));
        }

        // Start new detection when:
        // 1. System is completely IDLE with no operations, OR
        // 2. System is READY_TO_FIRE with 2 artifacts (third artifact collection)
        if (artifactPresent && intakeEmpty && frontPendingArtifact == null &&
            (systemIdle && noOperationInProgress || canCollectThirdArtifact)) {
            onArtifactFirstDetected(IntakeSource.FRONT);
        }
        // Do NOT cancel pending artifacts during operations - let them complete
        // This prevents conflicts when artifacts are being moved by the system
    }

    /**
     * Handle automatic back intake detection
     */
    private void handleBackIntakeAutoDetection() {
        boolean artifactPresent = isArtifactDetectedAtIntake(IntakeSource.BACK);
        boolean intakeEmpty = artifactInBackIntake == null;
        boolean systemIdle = currentState == SystemState.IDLE;
        boolean noOperationInProgress = !operationInProgress;

        // Allow third artifact collection when in READY_TO_FIRE state with 2 artifacts
        // Additional safety: ensure we have one in center and one in storage, with back intake empty for third
        boolean canCollectThirdArtifact = (currentState == SystemState.READY_TO_FIRE) &&
                                        (getArtifactCount() == 2) &&
                                        !operationInProgress &&
                                        (artifactInCenter != null) &&  // Must have artifact in center
                                        (artifactInBackIntake == null) &&  // Back must be empty for collection
                                        (artifactInFrontIntake != null);  // Front must have the stored second artifact

        if (config.isDebugTelemetry() && telemetry != null && canCollectThirdArtifact && artifactPresent && intakeEmpty) {
            telemetry.addLine("🔍 BACK: THIRD ARTIFACT DETECTION TRIGGER");
            telemetry.addLine(String.format("   State: %s, Count: %d, NextOrder: %d",
                currentState, getArtifactCount(), nextCollectionOrder));
            telemetry.addLine(String.format("   Center: %s, Front: %s, Back: %s",
                artifactInCenter != null ? artifactInCenter.getColor() : "Empty",
                artifactInFrontIntake != null ? artifactInFrontIntake.getColor() : "Empty",
                artifactInBackIntake != null ? artifactInBackIntake.getColor() : "Empty"));
        }

        // Start new detection when:
        // 1. System is completely IDLE with no operations, OR
        // 2. System is READY_TO_FIRE with 2 artifacts (third artifact collection)
        if (artifactPresent && intakeEmpty && backPendingArtifact == null &&
            (systemIdle && noOperationInProgress || canCollectThirdArtifact)) {
            onArtifactFirstDetected(IntakeSource.BACK);
        }
        // Do NOT cancel pending artifacts during operations - let them complete
        // This prevents conflicts when artifacts are being moved by the system
    }

    /**
     * Check if artifact is detected at the specified intake using multiple sensor types
     * Uses both original laser sensors and new REV 2m distance sensors for better detection
     * @param source Which intake to check
     * @return true if something is detected within distance threshold
     */
    private boolean isArtifactDetectedAtIntake(IntakeSource source) {
        if (hardware == null) return false;

        try {
            boolean laserDetection = false;
            boolean revDetection = false;

            // Check original goBILDA laser sensors (converted to cm)
            double laserDistanceCm = Double.MAX_VALUE;
            if (source == IntakeSource.FRONT) {
                laserDistanceCm = hardware.getFrontDistanceMM() / 10.0; // Convert mm to cm
            } else if (source == IntakeSource.BACK) {
                laserDistanceCm = hardware.getBackDistanceMM() / 10.0; // Convert mm to cm
            }

            if (laserDistanceCm >= 0 && laserDistanceCm < config.getArtifactDetectionDistance()) {
                laserDetection = true;
            }

            // Check NEW REV 2m distance sensors (if enabled and available)
            double revDistanceCm = Double.MAX_VALUE;
            if (config.getUseRevDistanceSensors()) {
                if (source == IntakeSource.FRONT) {
                    // Front intake uses front left REV sensor
                    revDistanceCm = hardware.getFrontLeftDistanceCM();
                } else if (source == IntakeSource.BACK) {
                    // Back intake uses back right REV sensor
                    revDistanceCm = hardware.getBackRightDistanceCM();
                }

                if (revDistanceCm >= 0 && revDistanceCm < config.getRevSensorDetectionThreshold()) {
                    revDetection = true;
                }
            }

            // Combine sensor readings based on configuration
            if (config.getUseRevDistanceSensors()) {
                // Use weighted combination of both sensor types
                double revWeight = config.getRevSensorWeight();
                double laserWeight = 1.0 - revWeight;

                // If either sensor detects with sufficient confidence, return true
                boolean combinedDetection = (revDetection && revWeight > 0.5) ||
                                          (laserDetection && laserWeight > 0.5) ||
                                          (revDetection && laserDetection); // Both agree

                if (config.isDebugTelemetry() && telemetry != null && combinedDetection) {
                    telemetry.addLine(String.format("🔍 %s: Laser=%.1fcm Rev=%.1fcm",
                        source, laserDistanceCm, revDistanceCm));
                }

                return combinedDetection;
            } else {
                // Use only original laser sensors
                return laserDetection;
            }

        } catch (Exception e) {
            // Sensor error - don't trigger false detection
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine("⚠️ Sensor error in detection: " + e.getMessage());
            }
            return false;
        }
    }

    /**
     * ═══════════════════════════════════════════════════════════════════════
     * Clear any pending artifacts when starting a new operation
     * This prevents stale detection delays from interfering with operations
     */
    private void clearPendingArtifacts() {
        if (frontPendingArtifact != null) {
            frontPendingArtifact = null;
            frontArtifactFirstDetected = 0;
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine("Cleared pending front artifact for operation");
            }
        }
        if (backPendingArtifact != null) {
            backPendingArtifact = null;
            backArtifactFirstDetected = 0;
            if (config.isDebugTelemetry() && telemetry != null) {
                telemetry.addLine("Cleared pending back artifact for operation");
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("🤖 INDEXING SYSTEM");
        telemetry.addData("State", currentState);
        telemetry.addData("Artifacts", String.format("%d/%d (list size: %d)",
            getArtifactCount(), IndexingConfig.MAX_ARTIFACTS, artifacts.size()));
        telemetry.addData("Operation", operationInProgress ? "IN PROGRESS" : "IDLE");
        telemetry.addLine("");

        // Artifact Locations with collection order
        telemetry.addLine("📦 ARTIFACT LOCATIONS:");
        telemetry.addData("  Center", artifactInCenter != null ?
            String.format("%s #%d", artifactInCenter.getColor(), artifactInCenter.getCollectionOrder()) : "Empty");
        telemetry.addData("  Front Intake", artifactInFrontIntake != null ?
            String.format("%s #%d", artifactInFrontIntake.getColor(), artifactInFrontIntake.getCollectionOrder()) : "Empty");
        telemetry.addData("  Back Intake", artifactInBackIntake != null ?
            String.format("%s #%d", artifactInBackIntake.getColor(), artifactInBackIntake.getCollectionOrder()) : "Empty");

        // Show collection order progress
        telemetry.addData("  Next Collection", "#" + nextCollectionOrder);
        telemetry.addLine("");

        // Distance Sensor Readings
        telemetry.addLine("📏 DISTANCE SENSORS:");

        // Original laser sensors
        double frontLaserMM = hardware.getFrontDistanceMM();
        double backLaserMM = hardware.getBackDistanceMM();
        telemetry.addData("  Laser Front", frontLaserMM >= 0 ?
            String.format("%.1fmm", frontLaserMM) : "N/A");
        telemetry.addData("  Laser Back", backLaserMM >= 0 ?
            String.format("%.1fmm", backLaserMM) : "N/A");

        // NEW: REV 2m Distance Sensors
        if (config.getUseRevDistanceSensors()) {
            double frontRevCM = hardware.getFrontLeftDistanceCM();
            double backRevCM = hardware.getBackRightDistanceCM();
            telemetry.addData("  REV Front Left", frontRevCM >= 0 ?
                String.format("%.1fcm%s", frontRevCM, frontRevCM < config.getRevSensorDetectionThreshold() ? " 🔍" : "") : "N/A");
            telemetry.addData("  REV Back Right", backRevCM >= 0 ?
                String.format("%.1fcm%s", backRevCM, backRevCM < config.getRevSensorDetectionThreshold() ? " 🔍" : "") : "N/A");
            telemetry.addData("  Detection Threshold", String.format("%.1fcm", config.getRevSensorDetectionThreshold()));
        }

        // Pending Detection Status
        if (frontPendingArtifact != null || backPendingArtifact != null) {
            telemetry.addLine("");
            telemetry.addLine("⏱️ PENDING DETECTION:");
            if (frontPendingArtifact != null) {
                long remaining = getRemainingColorDelay(IntakeSource.FRONT);
                telemetry.addData("  Front", String.format("%s - %.1fs remaining",
                    frontPendingArtifact.getColor(), remaining / 1000.0));
            }
            if (backPendingArtifact != null) {
                long remaining = getRemainingColorDelay(IntakeSource.BACK);
                telemetry.addData("  Back", String.format("%s - %.1fs remaining",
                    backPendingArtifact.getColor(), remaining / 1000.0));
            }
        }

        // Show all collected artifacts (for debugging)
        if (!artifacts.isEmpty()) {
            telemetry.addLine("");
            telemetry.addLine("📋 ALL ARTIFACTS:");
            for (Artifact a : artifacts) {
                telemetry.addData("  #" + a.getCollectionOrder(),
                    String.format("%s at %s", a.getColor(), a.getLocation()));
            }
        }

        // Uptake servo coordination status
        telemetry.addLine("");
        telemetry.addLine("🔧 UPTAKE SERVO STATUS:");
        telemetry.addData("  Pre-Positioned", uptakeServoPrePositioned ? "YES" : "NO");
        if (uptakeServoActionTime > 0) {
            long timeSinceAction = System.currentTimeMillis() - uptakeServoActionTime;
            telemetry.addData("  Last Action", String.format("%.1fs ago", timeSinceAction / 1000.0));
        }

        if (lastError != null && !lastError.isEmpty()) {
            telemetry.addLine("");
            telemetry.addData("⚠️ Last Error", lastError);
        }
    }

    /**
     * Get a summary string of the current system state
     */
    public String getStatusSummary() {
        return String.format("State: %s | Artifacts: %d/%d | Center: %s",
            currentState,
            getArtifactCount(),
            IndexingConfig.MAX_ARTIFACTS,
            artifactInCenter != null ? artifactInCenter.getColor().toString() : "Empty"
        );
    }
}
