package org.firstinspires.ftc.teamcode.util.aurora;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * SimpleIndexingSystem - Manual Control Artifact Indexing System for Aurora Robot
 *
 * This is a simplified, non-smart indexing system that provides direct driver control
 * over artifact management. Unlike the full IndexingSystem, this version:
 * - Does NOT use color sensors for artifact identification
 * - Does NOT automatically plan shot order
 * - Does NOT sort or rearrange artifacts based on color
 *
 * The driver is responsible for all decision-making about:
 * - Which intake to use for collection
 * - When to fire artifacts
 * - Which intake to use as the next shot source
 *
 * ARTIFACT FLOW:
 * 1. Empty Robot: Rollers always running, waiting for artifact
 * 2. First Artifact: Distance threshold passed (<10cm), transfer to center slot
 * 3. Second Artifact: Keep in intake as storage (don't push to center automatically)
 *    - Driver can manually request to move it to center (pushes first out opposite side)
 * 4. Third Artifact: Must stay in collection intake (no room elsewhere)
 *
 * MANUAL CONTROL METHODS:
 * - fireCenter(): Fire whatever is in center (if present)
 * - transferToCenter(IntakeSource): Move artifact from specified intake to center
 *    (pushes current center artifact to opposite intake if present)
 * - selectNextShot(IntakeSource): Mark which intake should provide next shot
 *
 * STORAGE BEHAVIOR:
 * - Intakes with stored artifacts keep rollers running (holds artifact in place)
 * - Empty intakes run at full collection speed
 */
public class SimpleIndexingSystem {

    // ═══════════════════════════════════════════════════════════════════════
    // SYSTEM STATE
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Current state of the indexing system
     */
    public enum SystemState {
        IDLE,                   // No operations in progress, ready for input
        COLLECTING,             // Actively collecting an artifact (detected by distance)
        TRANSFERRING,           // Transferring artifact to center
        PUSHING,                // Pushing center artifact to storage while pulling new one
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
    // CONSTANTS
    // ═══════════════════════════════════════════════════════════════════════

    /** Maximum artifacts the system can hold */
    public static final int MAX_ARTIFACTS = 3;

    /** Distance threshold for artifact detection in cm */
    private static final double DETECTION_DISTANCE_CM = 10.0;

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

    // Artifact tracking (simple presence tracking, no color)
    private boolean artifactInCenter;
    private boolean artifactInFrontIntake;
    private boolean artifactInBackIntake;

    // Current operation tracking
    private IntakeSource lastIntakeSource;
    private IntakeSource selectedNextShotSource;
    private long operationStartTime;
    private boolean operationInProgress;

    // Safety and error tracking
    private String lastError;
    private int errorCount;
    private boolean debugEnabled;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new SimpleIndexingSystem
     * @param hardware The Aurora hardware configuration
     * @param config The indexing configuration parameters
     * @param shooter The shooter subsystem
     * @param telemetry The telemetry system for logging
     */
    public SimpleIndexingSystem(AuroraHardwareConfig hardware, IndexingConfig config,
                                 Shooter shooter, Telemetry telemetry) {
        this.hardware = hardware;
        this.config = config;
        this.shooter = shooter;
        this.telemetry = telemetry;

        this.currentState = SystemState.IDLE;
        this.stateStartTime = System.currentTimeMillis();

        this.artifactInCenter = false;
        this.artifactInFrontIntake = false;
        this.artifactInBackIntake = false;

        this.lastIntakeSource = IntakeSource.UNKNOWN;
        this.selectedNextShotSource = IntakeSource.UNKNOWN;
        this.operationStartTime = 0;
        this.operationInProgress = false;

        this.lastError = "";
        this.errorCount = 0;
        this.debugEnabled = config.isDebugTelemetry();

        // Initialize hardware: Start rollers running continuously
        initializeHardware();
    }

    /**
     * Initialize hardware - start rollers running continuously
     */
    private void initializeHardware() {
        // Both intakes start in collection mode (full power, rolling inward)
        setIntakeCollectionMode(IntakeSource.FRONT);
        setIntakeCollectionMode(IntakeSource.BACK);

        // All servos start in idle position
        resetAllServos();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MAIN UPDATE LOOP
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Periodic update method - call this regularly from OpMode loop
     * Handles state machine transitions and ongoing operations
     */
    public void update() {
        long currentTime = System.currentTimeMillis();
        long stateElapsedTime = currentTime - stateStartTime;
        long operationElapsedTime = currentTime - operationStartTime;

        // Check for operation timeout
        if (operationInProgress && operationElapsedTime > config.getOperationTimeoutMs()) {
            setError("Operation timeout in state: " + currentState);
            resetToIdle();
            return;
        }

        // State machine processing
        switch (currentState) {
            case COLLECTING:
                updateCollecting(operationElapsedTime);
                break;

            case TRANSFERRING:
                updateTransferring(operationElapsedTime);
                break;

            case PUSHING:
                updatePushing(operationElapsedTime);
                break;

            case FIRING:
                updateFiring(operationElapsedTime);
                break;

            case IDLE:
                // Check for automatic artifact detection
                checkForArtifactDetection();
                break;

            case ERROR:
                // Try auto-recovery if enabled
                if (config.isEnableAutoRecovery() && stateElapsedTime > 1000) {
                    resetToIdle();
                }
                break;
        }

        // Update telemetry if debug enabled
        if (debugEnabled) {
            updateTelemetry();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // AUTOMATIC ARTIFACT DETECTION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check for artifact detection at intakes (distance sensor only)
     * Called during IDLE state
     */
    private void checkForArtifactDetection() {
        if (operationInProgress) return;

        // Check front intake
        if (!artifactInFrontIntake && isArtifactDetected(IntakeSource.FRONT)) {
            onArtifactDetected(IntakeSource.FRONT);
            return;
        }

        // Check back intake
        if (!artifactInBackIntake && isArtifactDetected(IntakeSource.BACK)) {
            onArtifactDetected(IntakeSource.BACK);
        }
    }

    /**
     * Check if artifact is detected by distance sensor only (<10cm)
     * @param source Which intake sensor to check
     * @return true if artifact detected
     */
    private boolean isArtifactDetected(IntakeSource source) {
        if (hardware == null) return false;

        try {
            double distanceMM = Double.MAX_VALUE;
            if (source == IntakeSource.FRONT) {
                distanceMM = hardware.getFrontDistanceMM();
            } else if (source == IntakeSource.BACK) {
                distanceMM = hardware.getBackDistanceMM();
            }

            // Convert threshold from cm to mm
            double thresholdMM = DETECTION_DISTANCE_CM * 10.0;

            // Artifact detected if distance is within threshold
            return distanceMM >= 0 && distanceMM < thresholdMM;
        } catch (Exception e) {
            // Sensor not available or error
        }
        return false;
    }

    /**
     * Called when an artifact is detected at an intake
     * @param source Which intake detected the artifact
     */
    private void onArtifactDetected(IntakeSource source) {
        // Safety check: can't collect more than 3 artifacts
        if (getArtifactCount() >= MAX_ARTIFACTS) {
            if (debugEnabled) {
                telemetry.addLine("Cannot collect: system full (3/3 artifacts)");
            }
            return;
        }

        // Safety check: can't start new collection while operation in progress
        if (operationInProgress) {
            if (debugEnabled) {
                telemetry.addLine("Cannot collect: operation in progress");
            }
            return;
        }

        lastIntakeSource = source;
        startArtifactCollection(source);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // COLLECTION AND TRANSFER LOGIC
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Start collecting an artifact from an intake
     */
    private void startArtifactCollection(IntakeSource source) {
        int currentCount = getArtifactCount();

        if (currentCount == 0) {
            // First artifact: collect and transfer to center
            changeState(SystemState.COLLECTING);
            operationInProgress = true;
            operationStartTime = System.currentTimeMillis();
            executeCollectionHardware();

            if (debugEnabled) {
                telemetry.addLine("Collecting first artifact from " + source);
            }
        } else if (currentCount == 1) {
            // Second artifact: keep in intake as storage
            // Mark as stored immediately (rollers keep running to hold it)
            if (source == IntakeSource.FRONT) {
                artifactInFrontIntake = true;
            } else {
                artifactInBackIntake = true;
            }
            updateIntakeModes();

            if (debugEnabled) {
                telemetry.addLine("Second artifact stored in " + source + " intake");
            }
        } else if (currentCount == 2) {
            // Third artifact: must stay in collection intake
            if (source == IntakeSource.FRONT) {
                artifactInFrontIntake = true;
            } else {
                artifactInBackIntake = true;
            }
            updateIntakeModes();

            if (debugEnabled) {
                telemetry.addLine("Third artifact stored in " + source + " intake");
            }
        }
    }

    /**
     * Update collection state (first artifact only)
     */
    private void updateCollecting(long elapsedTime) {
        if (elapsedTime >= config.getIntakeRollerTimeMs()) {
            // Start transfer to center
            changeState(SystemState.TRANSFERRING);
            operationStartTime = System.currentTimeMillis();
            executeTransferHardware();
        }
    }

    /**
     * Update transferring state
     */
    private void updateTransferring(long elapsedTime) {
        if (elapsedTime >= config.getTransferServoTimeMs() + config.getCenterAcceptTimeMs()) {
            // Transfer complete
            setInjectorServos(false);
            setIntakeTransferServo(lastIntakeSource, false);
            completeTransferToCenter();
        }
    }

    /**
     * Complete transfer to center
     */
    private void completeTransferToCenter() {
        artifactInCenter = true;

        // Clear the intake that the artifact came from
        if (lastIntakeSource == IntakeSource.FRONT) {
            artifactInFrontIntake = false;
        } else if (lastIntakeSource == IntakeSource.BACK) {
            artifactInBackIntake = false;
        }

        updateIntakeModes();
        changeState(SystemState.IDLE);
        operationInProgress = false;

        if (debugEnabled) {
            telemetry.addLine("Artifact now in center slot");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MANUAL CONTROL METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Fire whatever is currently in the center slot
     * @return true if firing started successfully
     */
    public boolean fireCenter() {
        if (!artifactInCenter) {
            if (debugEnabled) {
                telemetry.addLine("Cannot fire: no artifact in center");
            }
            return false;
        }

        if (operationInProgress) {
            if (debugEnabled) {
                telemetry.addLine("Cannot fire: operation in progress");
            }
            return false;
        }

        return startFiring();
    }

    /**
     * Manually transfer an artifact from specified intake to center
     * If center is occupied, pushes current center artifact to opposite intake
     * @param source The intake to pull from
     * @return true if transfer started successfully
     */
    public boolean transferToCenter(IntakeSource source) {
        if (source == IntakeSource.UNKNOWN) {
            if (debugEnabled) {
                telemetry.addLine("Cannot transfer: unknown source");
            }
            return false;
        }

        // Check if source intake has an artifact
        boolean hasArtifact = (source == IntakeSource.FRONT) ? artifactInFrontIntake : artifactInBackIntake;
        if (!hasArtifact) {
            if (debugEnabled) {
                telemetry.addLine("Cannot transfer: no artifact in " + source + " intake");
            }
            return false;
        }

        if (operationInProgress) {
            if (debugEnabled) {
                telemetry.addLine("Cannot transfer: operation in progress");
            }
            return false;
        }

        lastIntakeSource = source;

        if (artifactInCenter) {
            // Need to push current center artifact to opposite intake first
            IntakeSource opposite = (source == IntakeSource.FRONT) ? IntakeSource.BACK : IntakeSource.FRONT;
            boolean oppositeHasArtifact = (opposite == IntakeSource.FRONT) ? artifactInFrontIntake : artifactInBackIntake;

            if (oppositeHasArtifact) {
                if (debugEnabled) {
                    telemetry.addLine("Cannot transfer: opposite intake occupied, no room to push");
                }
                return false;
            }

            // Start push operation (swap center with source intake)
            return startPushSwap(source, opposite);
        } else {
            // Center is empty, simple transfer
            changeState(SystemState.TRANSFERRING);
            operationInProgress = true;
            operationStartTime = System.currentTimeMillis();
            executeTransferHardware();

            if (debugEnabled) {
                telemetry.addLine("Transferring from " + source + " to center");
            }
            return true;
        }
    }

    /**
     * Start a push swap operation: push center to opposite, pull source to center
     * @param source Intake to pull from
     * @param opposite Intake to push center artifact to
     * @return true if operation started
     */
    private boolean startPushSwap(IntakeSource source, IntakeSource opposite) {
        changeState(SystemState.PUSHING);
        operationInProgress = true;
        operationStartTime = System.currentTimeMillis();
        executePushHardware();

        if (debugEnabled) {
            telemetry.addLine("Push swap: center->" + opposite + ", " + source + "->center");
        }
        return true;
    }

    /**
     * Update pushing state
     */
    private void updatePushing(long elapsedTime) {
        long totalPushTime = config.getPushStartDelayMs() +
                             config.getSecondArtifactPushTimeMs() +
                             config.getStorageIntakeAcceptTimeMs();

        if (elapsedTime >= totalPushTime) {
            resetAllServos();
            completePushSwap();
        }
    }

    /**
     * Complete push swap operation
     */
    private void completePushSwap() {
        // Center artifact moved to opposite intake
        IntakeSource opposite = (lastIntakeSource == IntakeSource.FRONT) ? IntakeSource.BACK : IntakeSource.FRONT;
        if (opposite == IntakeSource.FRONT) {
            artifactInFrontIntake = true;
        } else {
            artifactInBackIntake = true;
        }

        // Source intake artifact now in center
        if (lastIntakeSource == IntakeSource.FRONT) {
            artifactInFrontIntake = false;
        } else {
            artifactInBackIntake = false;
        }

        // Center still has an artifact (the one pulled from source)
        artifactInCenter = true;

        updateIntakeModes();
        changeState(SystemState.IDLE);
        operationInProgress = false;

        if (debugEnabled) {
            telemetry.addLine("Push swap complete");
        }
    }

    /**
     * Select which intake should provide the next shot
     * This is informational only - driver must still call transferToCenter and fireCenter
     * @param source The intake to use for next shot
     */
    public void selectNextShot(IntakeSource source) {
        selectedNextShotSource = source;
        if (debugEnabled) {
            telemetry.addLine("Next shot selected from: " + source);
        }
    }

    /**
     * Get the currently selected next shot source
     * @return The selected intake source, or UNKNOWN if not set
     */
    public IntakeSource getSelectedNextShotSource() {
        return selectedNextShotSource;
    }

    /**
     * Convenience method: load selected intake to center and fire
     * Combines transferToCenter and fireCenter operations
     * @return true if operation started
     */
    public boolean loadAndFire() {
        if (artifactInCenter) {
            // Already loaded, just fire
            return fireCenter();
        }

        if (selectedNextShotSource != IntakeSource.UNKNOWN) {
            // Transfer selected source to center
            if (transferToCenter(selectedNextShotSource)) {
                // Transfer started, will need to call fireCenter after transfer completes
                return true;
            }
        }

        if (debugEnabled) {
            telemetry.addLine("Cannot load and fire: no source selected or transfer failed");
        }
        return false;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIRING LOGIC
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Start firing the artifact in center storage
     */
    private boolean startFiring() {
        if (!artifactInCenter) {
            return false;
        }

        changeState(SystemState.FIRING);
        operationInProgress = true;
        operationStartTime = System.currentTimeMillis();
        executeFiringHardware();

        if (debugEnabled) {
            telemetry.addLine("Firing artifact from center");
        }

        return true;
    }

    /**
     * Update firing state
     */
    private void updateFiring(long elapsedTime) {
        if (elapsedTime >= config.getFireFeedTimeMs()) {
            setUptakeServos(false);
            completeFiring();
        }
    }

    /**
     * Complete firing operation
     */
    private void completeFiring() {
        artifactInCenter = false;

        updateIntakeModes();
        changeState(SystemState.IDLE);
        operationInProgress = false;

        if (debugEnabled) {
            telemetry.addLine("Firing complete - center now empty");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MANUAL SERVO CONTROL (for advanced driver control)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Manually run the front intake transfer servo
     * @param active true to activate, false to stop
     */
    public void manualFrontTransfer(boolean active) {
        if (!operationInProgress) {
            setIntakeTransferServo(IntakeSource.FRONT, active);
        }
    }

    /**
     * Manually run the back intake transfer servo
     * @param active true to activate, false to stop
     */
    public void manualBackTransfer(boolean active) {
        if (!operationInProgress) {
            setIntakeTransferServo(IntakeSource.BACK, active);
        }
    }

    /**
     * Manually run the injector servos
     * @param active true to activate, false to stop
     */
    public void manualInjector(boolean active) {
        if (!operationInProgress) {
            setInjectorServos(active);
        }
    }

    /**
     * Manually run the uptake servos (feed to shooter)
     * @param active true to activate, false to stop
     */
    public void manualUptake(boolean active) {
        if (!operationInProgress) {
            setUptakeServos(active);
        }
    }

    /**
     * Manually set front intake roller power
     * @param power Power level (0.0 to 1.0, negative for reverse)
     */
    public void manualFrontRoller(double power) {
        if (!operationInProgress) {
            setIntakePower(IntakeSource.FRONT, power);
        }
    }

    /**
     * Manually set back intake roller power
     * @param power Power level (0.0 to 1.0, negative for reverse)
     */
    public void manualBackRoller(double power) {
        if (!operationInProgress) {
            setIntakePower(IntakeSource.BACK, power);
        }
    }

    /**
     * Check if manual control is available (no operation in progress)
     * @return true if manual control can be used
     */
    public boolean isManualControlAvailable() {
        return !operationInProgress;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE CONTROL METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Run intake roller motor at specified power
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
     */
    private void setIntakeStorageMode(IntakeSource source) {
        setIntakePower(source, config.getIntakeRollerPower() * 0.5);
    }

    /**
     * Set intake to collection mode - rollers run at full speed
     */
    private void setIntakeCollectionMode(IntakeSource source) {
        setIntakePower(source, config.getIntakeRollerPower());
    }

    /**
     * Set injector servo power (CRServos)
     */
    private void setInjectorServos(boolean active) {
        if (hardware == null) return;

        try {
            double power = active ? config.getTransferServoPower() : config.getTransferServoIdlePower();

            if (hardware.getInjectorServoLeft() != null) {
                hardware.getInjectorServoLeft().setPower(power);
            }
            if (hardware.getInjectorServoRight() != null) {
                hardware.getInjectorServoRight().setPower(power);
            }
        } catch (Exception e) {
            setError("Failed to set injector servos: " + e.getMessage());
        }
    }

    /**
     * Set uptake servo power (CRServos) - feeds to shooter
     */
    private void setUptakeServos(boolean active) {
        if (hardware == null) return;

        try {
            double power = active ? config.getTransferServoPower() : config.getTransferServoIdlePower();

            if (hardware.getUptakeServoL() != null) {
                hardware.getUptakeServoL().setPower(power);
            }
            if (hardware.getUptakeServoR() != null) {
                hardware.getUptakeServoR().setPower(power);
            }
        } catch (Exception e) {
            setError("Failed to set uptake servos: " + e.getMessage());
        }
    }

    /**
     * Set intake transfer servo power (CRServo)
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
     * Stop all transfer servos
     */
    private void resetAllServos() {
        setInjectorServos(false);
        setUptakeServos(false);
        setIntakeTransferServo(IntakeSource.FRONT, false);
        setIntakeTransferServo(IntakeSource.BACK, false);
    }

    /**
     * Update intake modes based on current artifact storage
     */
    private void updateIntakeModes() {
        if (artifactInFrontIntake) {
            setIntakeStorageMode(IntakeSource.FRONT);
        } else {
            setIntakeCollectionMode(IntakeSource.FRONT);
        }

        if (artifactInBackIntake) {
            setIntakeStorageMode(IntakeSource.BACK);
        } else {
            setIntakeCollectionMode(IntakeSource.BACK);
        }
    }

    /**
     * Execute hardware actions for collection state
     */
    private void executeCollectionHardware() {
        setIntakeCollectionMode(lastIntakeSource);
        setIntakeTransferServo(lastIntakeSource, true);
        setInjectorServos(true);
    }

    /**
     * Execute hardware actions for transferring state
     */
    private void executeTransferHardware() {
        setIntakeCollectionMode(lastIntakeSource);
        setIntakeTransferServo(lastIntakeSource, true);
        setInjectorServos(true);
    }

    /**
     * Execute hardware actions for pushing state
     */
    private void executePushHardware() {
        IntakeSource oppositeIntake = (lastIntakeSource == IntakeSource.FRONT)
            ? IntakeSource.BACK
            : IntakeSource.FRONT;

        setIntakeCollectionMode(lastIntakeSource);
        setIntakeCollectionMode(oppositeIntake);
        setIntakeTransferServo(lastIntakeSource, true);
        setIntakeTransferServo(oppositeIntake, true);
        setInjectorServos(true);
    }

    /**
     * Execute hardware actions for firing state
     */
    private void executeFiringHardware() {
        if (shooter != null && !shooter.isReadyToFire()) {
            if (debugEnabled) {
                telemetry.addLine("Waiting for shooter to be ready...");
            }
            return;
        }

        if (shooter != null) {
            shooter.fire();
        }

        setUptakeServos(true);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATE MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════

    private void changeState(SystemState newState) {
        currentState = newState;
        stateStartTime = System.currentTimeMillis();
    }

    private void resetToIdle() {
        currentState = SystemState.IDLE;
        operationInProgress = false;
        stateStartTime = System.currentTimeMillis();
    }

    private void setError(String error) {
        lastError = error;
        errorCount++;
        currentState = SystemState.ERROR;
        operationInProgress = false;

        if (telemetry != null) {
            telemetry.addLine("⚠️ SimpleIndexing Error: " + error);
        }
    }

    /**
     * Reset the indexing system to initial state
     */
    public void reset() {
        artifactInCenter = false;
        artifactInFrontIntake = false;
        artifactInBackIntake = false;
        lastIntakeSource = IntakeSource.UNKNOWN;
        selectedNextShotSource = IntakeSource.UNKNOWN;
        lastError = "";
        errorCount = 0;
        resetToIdle();
        initializeHardware();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS - SYSTEM STATE
    // ═══════════════════════════════════════════════════════════════════════

    public SystemState getCurrentState() { return currentState; }

    public int getArtifactCount() {
        int count = 0;
        if (artifactInCenter) count++;
        if (artifactInFrontIntake) count++;
        if (artifactInBackIntake) count++;
        return count;
    }

    public boolean hasArtifactInCenter() { return artifactInCenter; }
    public boolean hasArtifactInFrontIntake() { return artifactInFrontIntake; }
    public boolean hasArtifactInBackIntake() { return artifactInBackIntake; }

    public boolean isReadyToFire() {
        return currentState == SystemState.IDLE && artifactInCenter && !operationInProgress;
    }

    public boolean isOperationInProgress() { return operationInProgress; }

    public String getLastError() { return lastError; }
    public int getErrorCount() { return errorCount; }

    public Shooter getShooter() { return shooter; }

    /**
     * Enable or disable debug telemetry
     */
    public void setDebugEnabled(boolean enabled) {
        this.debugEnabled = enabled;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("🤖 SIMPLE INDEXING SYSTEM");
        telemetry.addData("State", currentState);
        telemetry.addData("Artifacts", String.format("%d/%d", getArtifactCount(), MAX_ARTIFACTS));
        telemetry.addLine("");

        telemetry.addData("Center", artifactInCenter ? "🟢 LOADED" : "⚪ Empty");
        telemetry.addData("Front Intake", artifactInFrontIntake ? "🟢 STORED" : "⚪ Empty");
        telemetry.addData("Back Intake", artifactInBackIntake ? "🟢 STORED" : "⚪ Empty");

        if (selectedNextShotSource != IntakeSource.UNKNOWN) {
            telemetry.addData("Next Shot From", selectedNextShotSource);
        }

        if (isReadyToFire()) {
            telemetry.addLine("🎯 READY TO FIRE");
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
        return String.format("State: %s | Artifacts: %d/%d | Center: %s | Ready: %s",
            currentState,
            getArtifactCount(),
            MAX_ARTIFACTS,
            artifactInCenter ? "Yes" : "No",
            isReadyToFire() ? "Yes" : "No"
        );
    }
}

