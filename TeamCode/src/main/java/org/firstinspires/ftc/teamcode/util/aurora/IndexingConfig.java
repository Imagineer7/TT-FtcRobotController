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
    private double intakeRollerTime = 0.8;

    /** Power level for intake rollers (0.0 to 1.0) */
    private double intakeRollerPower = 1.0;

    /** Power level for intake rollers when in storage mode - holding artifacts (0.0 to 1.0) */
    private double intakeStoragePower = 0.65;

    /** Time to run intake rollers in reverse to eject (seconds) */
    private double intakeEjectTime = 0.8;

    /** Power level for intake ejection (-1.0 to 0.0) */
    private double intakeEjectPower = -0.8;

    // ═══════════════════════════════════════════════════════════════════════
    // TRANSFER SYSTEM TIMING PARAMETERS (seconds)
    // ═══════════════════════════════════════════════════════════════════════

    /** Time for transfer servo operation to move artifact to center (seconds) */
    private double transferServoTime = 2.0;

    /** Power for transfer servo when active - CRServo runs at this power (-1.0 to 1.0) */
    private double transferServoPower = -1.0;  // Negative to reverse direction

    /** Idle power for transfer servo (0.0 = stopped) */
    private double transferServoIdlePower = 0.0;

    /** Time for center rollers to accept artifact during push (seconds) */
    private double centerAcceptTime = 0.8;

    /** Power for center rollers during artifact acceptance (0.0 to 1.0) */
    private double centerRollerPower = 0.8;

    // ═══════════════════════════════════════════════════════════════════════
    // PUSH-BASED INDEXING TIMING PARAMETERS (seconds)
    // ═══════════════════════════════════════════════════════════════════════

    /** Time for first artifact to settle in center storage (seconds) */
    private double firstArtifactSettleTime = 0.5;

    /** Time for second artifact push operation (pushes first to opposite intake) (seconds) */
    private double secondArtifactPushTime = 2.5;

    /** Delay before starting push operation (seconds) */
    private double pushStartDelay = 0.2;

    /** Time for artifact to fully enter storage intake (seconds) */
    private double storageIntakeAcceptTime = 0.8;

    // ═══════════════════════════════════════════════════════════════════════
    // FIRING SYSTEM TIMING PARAMETERS (seconds)
    // ═══════════════════════════════════════════════════════════════════════

    /** Time to run feed system to fire an artifact (seconds) */
    private double fireFeedTime = 0.8;

    /** Power for feed system during firing (0.0 to 1.0) */
    private double fireFeedPower = 1.0;

    /** Delay after firing before next operation can start (seconds) */
    private double postFireDelay = 0.3;

    /** Minimum time shooter must be running before firing (seconds) */
    private double minShooterSpinupTime = 0.5;

    // ═══════════════════════════════════════════════════════════════════════
    // SENSOR DETECTION PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Distance threshold for artifact detection (centimeters) - artifacts detected when distance < 10cm */
    private double artifactDetectionDistance = 10.0;

    /** NEW: REV 2m Distance Sensor Parameters */
    /** Enable/disable REV 2m distance sensors for enhanced detection */
    private boolean useRevDistanceSensors = true;

    /** Baseline distance reading when no artifact present (centimeters) */
    private double revSensorBaselineDistance = 25.0;

    /** Threshold below baseline that indicates artifact presence (centimeters) */
    private double revSensorDetectionThreshold = 15.0;  // Artifact detected when < 15cm

    /** Weight given to REV sensors vs original laser sensors (0.0 to 1.0) */
    private double revSensorWeight = 0.3;  // 30% REV sensors, 70% laser sensors

    /** Minimum time sensor must detect artifact before confirming (seconds) */
    private double sensorDebounceTime = 0.1;

    /** Color sensor confidence threshold (0.0 to 1.0) */
    private double colorConfidenceThreshold = 0.6;

    // ═══════════════════════════════════════════════════════════════════════
    // COLOR DETECTION PARAMETERS (Pattern-based scoring from measured RGB values)
    // ═══════════════════════════════════════════════════════════════════════

    /** Minimum confidence score for color detection (0.0 to 1.0) */
    private double colorDetectionMinScore = 0.6;

    /** Time to wait after artifact detection for stable color reading (seconds) */
    private double colorDetectionDelay = 0.4;

    // ═══════════════════════════════════════════════════════════════════════
    // STATE MACHINE PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Polling interval for sensor updates (seconds) */
    private double sensorPollingInterval = 0.02;

    /** Maximum time to wait for an operation to complete (seconds) */
    private double operationTimeout = 4.0;

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

    /** Manual push mode - when enabled, second artifact stays in intake instead of auto-pushing first to storage */
    private boolean manualPushMode = true; //Push disabled by default to match new game strategy

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

    public double getIntakeStoragePower() { return intakeStoragePower; }
    public void setIntakeStoragePower(double intakeStoragePower) {
        this.intakeStoragePower = intakeStoragePower;
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

    // NEW: REV 2m Distance Sensor Parameters
    public boolean getUseRevDistanceSensors() { return useRevDistanceSensors; }
    public void setUseRevDistanceSensors(boolean use) {
        this.useRevDistanceSensors = use;
    }

    public double getRevSensorBaselineDistance() { return revSensorBaselineDistance; }
    public void setRevSensorBaselineDistance(double distance) {
        this.revSensorBaselineDistance = distance;
    }

    public double getRevSensorDetectionThreshold() { return revSensorDetectionThreshold; }
    public void setRevSensorDetectionThreshold(double threshold) {
        this.revSensorDetectionThreshold = threshold;
    }

    public double getRevSensorWeight() { return revSensorWeight; }
    public void setRevSensorWeight(double weight) {
        this.revSensorWeight = Math.max(0.0, Math.min(1.0, weight)); // Clamp 0-1
    }

    // Color Detection Pattern System
    public double getColorDetectionMinScore() { return colorDetectionMinScore; }
    public void setColorDetectionMinScore(double value) { this.colorDetectionMinScore = value; }

    public double getColorDetectionDelay() { return colorDetectionDelay; }
    public void setColorDetectionDelay(double delay) { this.colorDetectionDelay = delay; }

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

    public boolean isManualPushMode() { return manualPushMode; }
    public void setManualPushMode(boolean enabled) {
        this.manualPushMode = enabled;
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
    public long getColorDetectionDelayMs() { return (long)(colorDetectionDelay * 1000); }

    /**
     * Reset all parameters to default values
     */
    public void resetToDefaults() {
        intakeRollerTime = 0.5;
        intakeRollerPower = 0.8;
        intakeStoragePower = 0.65;
        intakeEjectTime = 0.3;
        intakeEjectPower = -0.6;
        transferServoTime = 0.4;
        transferServoPower = -0.7;  // Negative to reverse direction
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

        // Pattern-based color detection
        colorDetectionMinScore = 0.6;
        colorDetectionDelay = 0.8;

        sensorPollingInterval = 0.02;
        operationTimeout = 3.0;
        debugTelemetry = true;
        enableSafetyChecks = true;
        enableAutoRecovery = true;
        manualPushMode = false;
    }

    /**
     * Determine artifact color based on characteristic RGB patterns
     * Purple: Red ≈ Green, Blue > Red/Green (R:0.1-0.15, G:0.1-0.15, B:0.15-0.3)
     * Green: Green >> Red/Blue (R:0.32, G:1.00, B:0.88)
     * @param red Normalized red value (0.0 to 1.0)
     * @param green Normalized green value (0.0 to 1.0)
     * @param blue Normalized blue value (0.0 to 1.0)
     * @return Detected color string or "UNKNOWN" if no match
     */
    public String detectArtifactColor(double red, double green, double blue) {
        // Minimum brightness threshold
        double maxValue = Math.max(Math.max(red, green), blue);
        if (maxValue < 0.05) return "UNKNOWN"; // Too dark to detect

        // Calculate scores for both colors
        double purpleScore = calculatePurpleScore(red, green, blue);
        double greenScore = calculateGreenScore(red, green, blue);

        // Determine best match - green wins ties since it should be more distinctive
        if (greenScore >= colorDetectionMinScore && greenScore > purpleScore) {
            return "GREEN";
        } else if (purpleScore >= colorDetectionMinScore) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }

    /**
     * Calculate confidence score for a specific color
     */
    public double calculateColorConfidence(double red, double green, double blue, String targetColor) {
        if ("PURPLE".equals(targetColor)) {
            return calculatePurpleScore(red, green, blue);
        } else if ("GREEN".equals(targetColor)) {
            return calculateGreenScore(red, green, blue);
        }
        return 0.0;
    }

    /**
     * Calculate purple detection score based on characteristic pattern:
     * - Red ≈ Green (similar values)
     * - Blue > Red and Blue > Green
     * - Values in range: R:0.1-0.15, G:0.1-0.15, B:0.15-0.3
     */
    private double calculatePurpleScore(double red, double green, double blue) {
        double score = 0.0;

        // Check if red ≈ green (should be similar for purple)
        double rgDiff = Math.abs(red - green);
        double rgSimilarity = Math.max(0.0, 1.0 - (rgDiff / 0.1)); // Penalize if difference > 0.1
        score += rgSimilarity * 0.4; // 40% weight

        // Check if blue > red and blue > green (blue should be highest)
        if (blue > red && blue > green) {
            double blueAdvantage = Math.min(blue - Math.max(red, green), 0.2) / 0.2; // Normalize advantage
            score += blueAdvantage * 0.3; // 30% weight
        }

        // Check if values are in expected ranges
        double redMatch = 0.0, greenMatch = 0.0, blueMatch = 0.0;

        if (red >= 0.08 && red <= 0.18) redMatch = 1.0;   // Expanded range around 0.1-0.15
        else redMatch = Math.max(0.0, 1.0 - Math.abs(red - 0.125) / 0.125);

        if (green >= 0.08 && green <= 0.18) greenMatch = 1.0; // Expanded range around 0.1-0.15
        else greenMatch = Math.max(0.0, 1.0 - Math.abs(green - 0.125) / 0.125);

        if (blue >= 0.12 && blue <= 0.35) blueMatch = 1.0;   // Expanded range around 0.15-0.3
        else blueMatch = Math.max(0.0, 1.0 - Math.abs(blue - 0.225) / 0.225);

        score += (redMatch + greenMatch + blueMatch) / 3.0 * 0.3; // 30% weight for range matching

        return Math.min(score, 1.0);
    }

    /**
     * Calculate green detection score based on NEW measured pattern:
     * - Green > Red and Green > Blue (but not dramatically)
     * - Values: R:0.05-0.1, G:0.2-0.35, B:0.1-0.2
     * - Green is dominant but values are much lower than expected
     */
    private double calculateGreenScore(double red, double green, double blue) {
        double score = 0.0;

        // Green should be dominant (higher than red and blue)
        if (green > red && green > blue) {
            // Green dominance is less dramatic than before
            double greenDominance = Math.min(green - Math.max(red, blue), 0.2) / 0.2;
            score += greenDominance * 0.4; // 40% weight for green dominance
        }

        // Check if green is at least 2x red (characteristic of green artifacts)
        if (green >= red * 1.5 && green >= blue * 1.2) {
            score += 0.3; // 30% bonus for proper ratios
        }

        // Range matching based on NEW measurements: R:0.05-0.1, G:0.2-0.35, B:0.1-0.2
        double redMatch = 0.0, greenMatch = 0.0, blueMatch = 0.0;

        // Red should be low (0.05-0.1)
        if (red >= 0.03 && red <= 0.12) redMatch = 1.0;
        else redMatch = Math.max(0.0, 1.0 - Math.abs(red - 0.075) / 0.075);

        // Green should be in middle range (0.2-0.35)
        if (green >= 0.15 && green <= 0.4) greenMatch = 1.0;
        else greenMatch = Math.max(0.0, 1.0 - Math.abs(green - 0.275) / 0.275);

        // Blue should be low-medium (0.1-0.2)
        if (blue >= 0.08 && blue <= 0.25) blueMatch = 1.0;
        else blueMatch = Math.max(0.0, 1.0 - Math.abs(blue - 0.15) / 0.15);

        score += (redMatch + greenMatch + blueMatch) / 3.0 * 0.3; // 30% weight for range matching

        return Math.min(score, 1.0);
    }
}
