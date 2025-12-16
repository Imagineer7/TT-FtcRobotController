package org.firstinspires.ftc.teamcode.util.aurora;

/**
 * IndexingConfig - Centralized configuration for artifact indexing system
 *
 * This class provides tunable parameters for the push-based artifact indexing system.
 * All timing values and thresholds can be adjusted here without changing core logic.
 *
 * Key Parameters:
 * - Intake operation timings
 * - Transfer operation timings
 * - Sensor detection thresholds
 * - Push-based movement timings
 */
public class IndexingConfig {

    // ═══════════════════════════════════════════════════════════════════════
    // INTAKE SYSTEM TIMING PARAMETERS (seconds)
    // ═══════════════════════════════════════════════════════════════════════

    /** Time to run intake rollers to collect an artifact (seconds) */
    private double intakeRollerTime = 0.5;

    /** Power level for intake rollers (0.0 to 1.0) */
    private double intakeRollerPower = 0.8;

    /** Time to run intake rollers in reverse to eject (seconds) */
    private double intakeEjectTime = 0.3;

    /** Power level for intake ejection (-1.0 to 0.0) */
    private double intakeEjectPower = -0.6;

    // ═══════════════════════════════════════════════════════════════════════
    // TRANSFER SYSTEM TIMING PARAMETERS (seconds)
    // ═══════════════════════════════════════════════════════════════════════

    /** Time for transfer servo operation to move artifact to center (seconds) */
    private double transferServoTime = 0.4;

    /** Power for transfer servo when active - CRServo runs at this power (-1.0 to 1.0) */
    private double transferServoPower = 0.7;

    /** Idle power for transfer servo (0.0 = stopped) */
    private double transferServoIdlePower = 0.0;

    /** Time for center rollers to accept artifact during push (seconds) */
    private double centerAcceptTime = 0.3;

    /** Power for center rollers during artifact acceptance (0.0 to 1.0) */
    private double centerRollerPower = 0.7;

    // ═══════════════════════════════════════════════════════════════════════
    // PUSH-BASED INDEXING TIMING PARAMETERS (seconds)
    // ═══════════════════════════════════════════════════════════════════════

    /** Time for first artifact to settle in center storage (seconds) */
    private double firstArtifactSettleTime = 0.2;

    /** Time for second artifact push operation (pushes first to opposite intake) (seconds) */
    private double secondArtifactPushTime = 0.6;

    /** Delay before starting push operation (seconds) */
    private double pushStartDelay = 0.1;

    /** Time for artifact to fully enter storage intake (seconds) */
    private double storageIntakeAcceptTime = 0.4;

    // ═══════════════════════════════════════════════════════════════════════
    // FIRING SYSTEM TIMING PARAMETERS (seconds)
    // ═══════════════════════════════════════════════════════════════════════

    /** Time to run feed system to fire an artifact (seconds) */
    private double fireFeedTime = 0.25;

    /** Power for feed system during firing (0.0 to 1.0) */
    private double fireFeedPower = 1.0;

    /** Delay after firing before next operation can start (seconds) */
    private double postFireDelay = 0.3;

    /** Minimum time shooter must be running before firing (seconds) */
    private double minShooterSpinupTime = 1.0;

    // ═══════════════════════════════════════════════════════════════════════
    // SENSOR DETECTION PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Distance threshold for artifact detection (centimeters) - artifacts detected when distance < 10cm */
    private double artifactDetectionDistance = 10.0;

    /** Minimum time sensor must detect artifact before confirming (seconds) */
    private double sensorDebounceTime = 0.05;

    /** Color sensor confidence threshold (0.0 to 1.0) */
    private double colorConfidenceThreshold = 0.6;

    // ═══════════════════════════════════════════════════════════════════════
    // STATE MACHINE PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Polling interval for sensor updates (seconds) */
    private double sensorPollingInterval = 0.02;

    /** Maximum time to wait for an operation to complete (seconds) */
    private double operationTimeout = 3.0;

    /** Enable/disable debug telemetry output */
    private boolean debugTelemetry = true;

    // ═══════════════════════════════════════════════════════════════════════
    // SAFETY PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Enable safety checks (prevents physically impossible operations) */
    private boolean enableSafetyChecks = true;

    /** Maximum number of artifacts that can be stored */
    public static final int MAX_ARTIFACTS = 3;

    /** Enable automatic recovery from illegal states */
    private boolean enableAutoRecovery = true;

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS AND SETTERS
    // ═══════════════════════════════════════════════════════════════════════

    // Intake System
    public double getIntakeRollerTime() { return intakeRollerTime; }
    public void setIntakeRollerTime(double intakeRollerTime) { 
        this.intakeRollerTime = intakeRollerTime; 
    }

    public double getIntakeRollerPower() { return intakeRollerPower; }
    public void setIntakeRollerPower(double intakeRollerPower) { 
        this.intakeRollerPower = intakeRollerPower; 
    }

    public double getIntakeEjectTime() { return intakeEjectTime; }
    public void setIntakeEjectTime(double intakeEjectTime) { 
        this.intakeEjectTime = intakeEjectTime; 
    }

    public double getIntakeEjectPower() { return intakeEjectPower; }
    public void setIntakeEjectPower(double intakeEjectPower) { 
        this.intakeEjectPower = intakeEjectPower; 
    }

    // Transfer System
    public double getTransferServoTime() { return transferServoTime; }
    public void setTransferServoTime(double transferServoTime) { 
        this.transferServoTime = transferServoTime; 
    }

    public double getTransferServoPower() { return transferServoPower; }
    public void setTransferServoPower(double power) {
        this.transferServoPower = power;
    }

    public double getTransferServoIdlePower() { return transferServoIdlePower; }
    public void setTransferServoIdlePower(double power) {
        this.transferServoIdlePower = power;
    }

    public double getCenterAcceptTime() { return centerAcceptTime; }
    public void setCenterAcceptTime(double centerAcceptTime) { 
        this.centerAcceptTime = centerAcceptTime; 
    }

    public double getCenterRollerPower() { return centerRollerPower; }
    public void setCenterRollerPower(double centerRollerPower) { 
        this.centerRollerPower = centerRollerPower; 
    }

    // Push-based Indexing
    public double getFirstArtifactSettleTime() { return firstArtifactSettleTime; }
    public void setFirstArtifactSettleTime(double time) { 
        this.firstArtifactSettleTime = time; 
    }

    public double getSecondArtifactPushTime() { return secondArtifactPushTime; }
    public void setSecondArtifactPushTime(double time) { 
        this.secondArtifactPushTime = time; 
    }

    public double getPushStartDelay() { return pushStartDelay; }
    public void setPushStartDelay(double pushStartDelay) { 
        this.pushStartDelay = pushStartDelay; 
    }

    public double getStorageIntakeAcceptTime() { return storageIntakeAcceptTime; }
    public void setStorageIntakeAcceptTime(double time) { 
        this.storageIntakeAcceptTime = time; 
    }

    // Firing System
    public double getFireFeedTime() { return fireFeedTime; }
    public void setFireFeedTime(double fireFeedTime) { 
        this.fireFeedTime = fireFeedTime; 
    }

    public double getFireFeedPower() { return fireFeedPower; }
    public void setFireFeedPower(double fireFeedPower) { 
        this.fireFeedPower = fireFeedPower; 
    }

    public double getPostFireDelay() { return postFireDelay; }
    public void setPostFireDelay(double postFireDelay) { 
        this.postFireDelay = postFireDelay; 
    }

    public double getMinShooterSpinupTime() { return minShooterSpinupTime; }
    public void setMinShooterSpinupTime(double time) { 
        this.minShooterSpinupTime = time; 
    }

    // Sensor Detection
    public double getArtifactDetectionDistance() { return artifactDetectionDistance; }
    public void setArtifactDetectionDistance(double distance) { 
        this.artifactDetectionDistance = distance; 
    }

    public double getSensorDebounceTime() { return sensorDebounceTime; }
    public void setSensorDebounceTime(double time) { 
        this.sensorDebounceTime = time; 
    }

    public double getColorConfidenceThreshold() { return colorConfidenceThreshold; }
    public void setColorConfidenceThreshold(double threshold) { 
        this.colorConfidenceThreshold = threshold; 
    }

    // State Machine
    public double getSensorPollingInterval() { return sensorPollingInterval; }
    public void setSensorPollingInterval(double interval) { 
        this.sensorPollingInterval = interval; 
    }

    public double getOperationTimeout() { return operationTimeout; }
    public void setOperationTimeout(double timeout) { 
        this.operationTimeout = timeout; 
    }

    public boolean isDebugTelemetry() { return debugTelemetry; }
    public void setDebugTelemetry(boolean enabled) { 
        this.debugTelemetry = enabled; 
    }

    // Safety
    public boolean isEnableSafetyChecks() { return enableSafetyChecks; }
    public void setEnableSafetyChecks(boolean enabled) { 
        this.enableSafetyChecks = enabled; 
    }

    public boolean isEnableAutoRecovery() { return enableAutoRecovery; }
    public void setEnableAutoRecovery(boolean enabled) { 
        this.enableAutoRecovery = enabled; 
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Convert seconds to milliseconds for timing operations
     */
    public long getIntakeRollerTimeMs() { return (long)(intakeRollerTime * 1000); }
    public long getTransferServoTimeMs() { return (long)(transferServoTime * 1000); }
    public long getCenterAcceptTimeMs() { return (long)(centerAcceptTime * 1000); }
    public long getFirstArtifactSettleTimeMs() { return (long)(firstArtifactSettleTime * 1000); }
    public long getSecondArtifactPushTimeMs() { return (long)(secondArtifactPushTime * 1000); }
    public long getPushStartDelayMs() { return (long)(pushStartDelay * 1000); }
    public long getStorageIntakeAcceptTimeMs() { return (long)(storageIntakeAcceptTime * 1000); }
    public long getFireFeedTimeMs() { return (long)(fireFeedTime * 1000); }
    public long getPostFireDelayMs() { return (long)(postFireDelay * 1000); }
    public long getMinShooterSpinupTimeMs() { return (long)(minShooterSpinupTime * 1000); }
    public long getSensorDebounceTimeMs() { return (long)(sensorDebounceTime * 1000); }
    public long getSensorPollingIntervalMs() { return (long)(sensorPollingInterval * 1000); }
    public long getOperationTimeoutMs() { return (long)(operationTimeout * 1000); }

    /**
     * Reset all parameters to default values
     */
    public void resetToDefaults() {
        intakeRollerTime = 0.5;
        intakeRollerPower = 0.8;
        intakeEjectTime = 0.3;
        intakeEjectPower = -0.6;
        transferServoTime = 0.4;
        transferServoPower = 0.7;
        transferServoIdlePower = 0.0;
        centerAcceptTime = 0.3;
        centerRollerPower = 0.7;
        firstArtifactSettleTime = 0.2;
        secondArtifactPushTime = 0.6;
        pushStartDelay = 0.1;
        storageIntakeAcceptTime = 0.4;
        fireFeedTime = 0.25;
        fireFeedPower = 1.0;
        postFireDelay = 0.3;
        minShooterSpinupTime = 1.0;
        artifactDetectionDistance = 10.0;
        sensorDebounceTime = 0.05;
        colorConfidenceThreshold = 0.6;
        sensorPollingInterval = 0.02;
        operationTimeout = 3.0;
        debugTelemetry = true;
        enableSafetyChecks = true;
        enableAutoRecovery = true;
    }
}
