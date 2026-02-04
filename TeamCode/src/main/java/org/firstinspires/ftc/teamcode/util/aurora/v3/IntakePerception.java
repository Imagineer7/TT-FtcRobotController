package org.firstinspires.ftc.teamcode.util.aurora.v3;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;

/**
 * IntakePerception - Sensor fusion for a single intake (front or back)
 *
 * This class fuses multiple sensors per intake to produce derived signals:
 * - leftProximity (REV Color V3 proximity detection - left sensor)
 * - rightProximity (REV Color V3 proximity detection - right sensor)
 * - confirmationDistance (goBILDA distance sensor - confirmation)
 * - colorSeesArtifact (color sensor confidence)
 * - artifactHint (combined presence signal)
 * - presenceConfidence (LOW/MED/HIGH)
 *
 * Key principles:
 * - REV Color V3 sensors (using proximity) are primary detectors
 * - goBILDA distance sensor is confirmation signal
 * - Sensors provide hints, not truth
 * - Debounce ensures stable signals
 * - Color classification only at stable checkpoints
 *
 * Hardware Layout:
 * - 2× REV Color Sensor V3 (left and right, each with color + proximity)
 * - 1× goBILDA distance sensor (confirmation)
 */
public class IntakePerception {

    // ═══════════════════════════════════════════════════════════════════════
    // ENUMS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Presence confidence level
     */
    public enum PresenceConfidence {
        NONE,      // No sensors detect artifact
        LOW,       // One sensor detects
        MEDIUM,    // Two sensors detect
        HIGH       // Three or more sensors detect
    }

    /**
     * Intake side identifier
     */
    public enum IntakeSide {
        FRONT,
        BACK
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELDS
    // ═══════════════════════════════════════════════════════════════════════

    private final IntakeSide side;
    private final IndexingConfig config;

    // Hardware sensors
    private final AnalogInput confirmationSensor;           // goBILDA distance (confirmation)
    private final NormalizedColorSensor leftColorSensor;    // Left REV Color V3 (color + proximity)
    private final NormalizedColorSensor rightColorSensor;   // Right REV Color V3 (color + proximity)

    // Derived signal state
    private boolean confirmationDetected;        // goBILDA distance sensor
    private boolean leftProximityDetected;       // REV Color V3 proximity (left)
    private boolean rightProximityDetected;      // REV Color V3 proximity (right)
    private boolean colorSeesArtifact_left;      // Left color confidence
    private boolean colorSeesArtifact_right;     // Right color confidence

    // Hysteresis state (not needed anymore but kept for compatibility)
    private boolean hysteresisState;  // General hysteresis state
    
    // Debounce state
    private long lastRawHintChangeTime;     // When raw hint last changed
    private boolean fastPresence;           // Fast-debounced presence (30ms)
    private boolean stablePresence;         // Slow-debounced presence (100ms)
    private boolean lastRawHint;

    // Color classification
    private ArtifactIdentity.ColorClass lastColorClass;
    private double lastColorConfidence;
    private boolean samplingEnabled;        // Gates color re-sampling to checkpoints
    
    // Multiple artifact detection
    private boolean multipleDifferentColorsDetected;  // Two artifacts of different colors
    private boolean multipleSameColorDetected;        // Two artifacts of same color (less reliable)

    // Manual override state (for testing/operator override)
    private boolean forcedDetectionActive;
    private ArtifactIdentity.ColorClass forcedColor;
    
    // Sensor caching (performance optimization)
    // Cache sensor readings to avoid duplicate I2C calls within same loop
    private boolean sensorsCached;          // True if readings are from current loop
    private double cachedConfirmationVoltage;
    private double cachedLeftDistance;
    private double cachedRightDistance;
    // Color readings cached in updateColorSensors (needed for sampling logic)

    // Constants
    private static final double CONFIRMATION_THRESHOLD_CM = 10.0;  // Artifact detected when < 10cm (goBILDA)
    private static final double PROXIMITY_THRESHOLD_CM = 7.0;       // REV Color V3 proximity threshold (7cm or less = artifact detected)
    private static final double MAX_LASER_VOLTS = 3.3;
    private static final double MAX_LASER_DISTANCE_MM = 1000.0;
    
    // Multiple artifact detection constants
    private static final double MULTIPLE_SAME_COLOR_PROXIMITY_THRESHOLD = 5.0;  // Both sensors < 5cm = likely two artifacts of same color
    private static final double COLOR_SCORE_OPPOSITE_THRESHOLD = 0.3;  // Score difference threshold to detect opposite colors

    // Debounce timing
    private static final long EDGE_DETECTION_DEBOUNCE_MS = 30;    // Fast response for entry/exit
    private static final long STABLE_PRESENCE_DEBOUNCE_MS = 100;  // Confirm still present

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new IntakePerception for one intake
     *
     * @param side Which intake (FRONT or BACK)
     * @param confirmationSensor goBILDA distance sensor (confirmation)
     * @param leftColorSensor Left REV Color V3 sensor (color + proximity)
     * @param rightColorSensor Right REV Color V3 sensor (color + proximity)
     * @param config IndexingConfig for thresholds
     */
    public IntakePerception(IntakeSide side,
                           AnalogInput confirmationSensor,
                           NormalizedColorSensor leftColorSensor,
                           NormalizedColorSensor rightColorSensor,
                           IndexingConfig config) {
        this.side = side;
        this.confirmationSensor = confirmationSensor;
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.config = config;

        // Initialize state
        this.confirmationDetected = false;
        this.leftProximityDetected = false;
        this.rightProximityDetected = false;
        this.colorSeesArtifact_left = false;
        this.colorSeesArtifact_right = false;
        this.hysteresisState = false;
        this.fastPresence = false;
        this.stablePresence = false;
        this.lastRawHint = false;
        this.lastRawHintChangeTime = System.currentTimeMillis();
        this.lastColorClass = ArtifactIdentity.ColorClass.UNKNOWN;
        this.lastColorConfidence = 0.0;
        this.samplingEnabled = false;
        this.forcedDetectionActive = false;
        this.forcedColor = ArtifactIdentity.ColorClass.UNKNOWN;
        this.sensorsCached = false;
        this.cachedConfirmationVoltage = 0.0;
        this.cachedLeftDistance = 0.0;
        this.cachedRightDistance = 0.0;
        this.multipleDifferentColorsDetected = false;
        this.multipleSameColorDetected = false;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UPDATE METHOD (Call every loop)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update all sensor readings and derived signals
     * Call this every loop
     * 
     * PERFORMANCE: Reads sensors once per loop and caches values.
     * Subsequent calls within same loop use cached values (no duplicate I2C).
     */
    public void update() {
        // Read sensors once per loop (cache for this iteration)
        readSensorsOnce();
        
        // Update confirmation sensor (goBILDA distance)
        updateConfirmationSensor();

        // Update REV Color V3 proximity sensors (primary detectors)
        updateProximitySensors();

        // Update color sensors (for color classification)
        updateColorSensors();

        // Update debounced hints (edge detection + stable presence)
        updateDebouncedHints();
        
        // Mark that sensors have been cached for this loop
        // Will be cleared at start of next update() call
        sensorsCached = true;
    }
    
    /**
     * Read all sensors once and cache values for this loop iteration.
     * This prevents duplicate I2C calls if update() is called multiple times.
     * 
     * PERFORMANCE: Single I2C transaction per sensor per loop.
     */
    private void readSensorsOnce() {
        // Always read sensors fresh each update() call
        // The sensorsCached flag prevents duplicate reads if same method called twice
        // But we reset it each update() to get fresh readings each loop
        sensorsCached = false;  // Reset at start of update cycle
        
        // Read confirmation sensor (goBILDA distance - analog)
        try {
            cachedConfirmationVoltage = (confirmationSensor != null) 
                ? confirmationSensor.getVoltage() 
                : 0.0;
        } catch (Exception e) {
            cachedConfirmationVoltage = 0.0;
        }
        
        // Read primary proximity sensor (REV Color V3 distance)
        try {
            cachedLeftDistance = (leftColorSensor instanceof DistanceSensor)
                ? ((DistanceSensor) leftColorSensor).getDistance(DistanceUnit.CM)
                : 999.0;  // Far away = not detected
        } catch (Exception e) {
            cachedLeftDistance = 999.0;
        }
        
        // Read secondary proximity sensor (REV Color V3 distance)
        try {
            cachedRightDistance = (rightColorSensor instanceof DistanceSensor)
                ? ((DistanceSensor) rightColorSensor).getDistance(DistanceUnit.CM)
                : 999.0;
        } catch (Exception e) {
            cachedRightDistance = 999.0;
        }
        
        // Note: Color sensor readings are NOT cached here because:
        // 1. They're only read when samplingEnabled=true (at checkpoints)
        // 2. They need fresh readings for accurate color classification
        // 3. Color reads are less frequent than proximity reads
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SENSOR UPDATE METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update confirmation sensor (goBILDA distance sensor)
     * Converts voltage to distance and checks threshold
     * 
     * PERFORMANCE: Uses cached voltage reading from readSensorsOnce()
     */
    private void updateConfirmationSensor() {
        if (confirmationSensor == null) {
            confirmationDetected = false;
            return;
        }

        try {
            // Use cached voltage reading (already read in readSensorsOnce)
            double voltage = cachedConfirmationVoltage;
            double distanceMm = (voltage / MAX_LASER_VOLTS) * MAX_LASER_DISTANCE_MM;
            double distanceCm = distanceMm / 10.0;

            // Artifact detected when distance < threshold
            confirmationDetected = (distanceCm < CONFIRMATION_THRESHOLD_CM && distanceCm > 0.5);  // > 0.5 filters noise
        } catch (Exception e) {
            confirmationDetected = false;  // Sensor error = assume empty
        }
    }

    /**
     * Update REV Color V3 proximity sensors (primary detection method)
     * Uses the distance interface available on REV Color Sensor V3
     * Reference: SensorColor.java line 211-212
     * 
     * PERFORMANCE: Uses cached distance readings from readSensorsOnce()
     */
    private void updateProximitySensors() {
        // Update left sensor proximity (use cached reading)
        if (leftColorSensor instanceof DistanceSensor) {
            try {
                double distanceCm = cachedLeftDistance;
                leftProximityDetected = (distanceCm < PROXIMITY_THRESHOLD_CM && distanceCm > 0.1);
            } catch (Exception e) {
                leftProximityDetected = false;
            }
        } else {
            leftProximityDetected = false;
        }

        // Update right sensor proximity (use cached reading)
        if (rightColorSensor instanceof DistanceSensor) {
            try {
                double distanceCm = cachedRightDistance;
                rightProximityDetected = (distanceCm < PROXIMITY_THRESHOLD_CM && distanceCm > 0.1);
            } catch (Exception e) {
                rightProximityDetected = false;
            }
        } else {
            rightProximityDetected = false;
        }
    }

    /**
     * Update color sensors and check for high-confidence artifact detection
     * Uses color classification logic from IndexingConfig
     * Color detection is now GATED by proximity - prevents false positives from high-gain sensors
     */
    private void updateColorSensors() {
        // Color sensors only contribute when proximity confirms physical presence
        colorSeesArtifact_left = checkColorSensorWithProximity(leftColorSensor, leftProximityDetected);
        colorSeesArtifact_right = checkColorSensorWithProximity(rightColorSensor, rightProximityDetected);

        // Only update best color classification when sampling is enabled (at checkpoints)
        if (samplingEnabled) {
            updateBestColorClassification();
        }
        // Otherwise keep last sampled color (don't resample while moving)

        // Update multiple artifact detection
        updateMultipleArtifactDetection();
    }

    /**
     * Detect if there are multiple artifacts in the intake.
     *
     * Two Different Colors:
     * - Pattern: Left and right sensors show OPPOSITE color scores
     * - Left sensor high purple + Right sensor high green (or vice versa)
     * - Difference in purple/green scores between sensors > threshold
     *
     * Two Same Color (less reliable):
     * - Pattern: Both proximity sensors < 5cm
     * - Both left and right proximity readings are very close
     * - Color scores are similar (both detect same color)
     */
    private void updateMultipleArtifactDetection() {
        // Reset flags
        multipleDifferentColorsDetected = false;
        multipleSameColorDetected = false;

        // Get color scores from both sensors
        double[] leftScores = getLeftColorScores();
        double[] rightScores = getRightColorScores();

        if (leftScores == null || rightScores == null) {
            return;  // Can't detect without color scores
        }

        double leftPurple = leftScores[0];
        double leftGreen = leftScores[1];
        double rightPurple = rightScores[0];
        double rightGreen = rightScores[1];

        // Check for DIFFERENT colors (reliable detection)
        // Pattern 1: Left sees purple, Right sees green
        boolean leftPurpleRightGreen = (leftPurple > leftGreen + COLOR_SCORE_OPPOSITE_THRESHOLD) &&
                                        (rightGreen > rightPurple + COLOR_SCORE_OPPOSITE_THRESHOLD);

        // Pattern 2: Left sees green, Right sees purple
        boolean leftGreenRightPurple = (leftGreen > leftPurple + COLOR_SCORE_OPPOSITE_THRESHOLD) &&
                                        (rightPurple > rightGreen + COLOR_SCORE_OPPOSITE_THRESHOLD);

        multipleDifferentColorsDetected = leftPurpleRightGreen || leftGreenRightPurple;

        // Check for SAME color (less reliable, proximity-based)
        // Both proximity sensors must detect something close
        if (leftProximityDetected && rightProximityDetected) {
            // Use cached distance readings
            boolean bothVeryClose = (cachedLeftDistance < MULTIPLE_SAME_COLOR_PROXIMITY_THRESHOLD &&
                                     cachedLeftDistance > 0.1) &&
                                    (cachedRightDistance < MULTIPLE_SAME_COLOR_PROXIMITY_THRESHOLD &&
                                     cachedRightDistance > 0.1);

            // Check that color scores agree (both see same color)
            boolean colorScoresAgree = Math.abs(leftPurple - rightPurple) < 0.2 &&
                                       Math.abs(leftGreen - rightGreen) < 0.2;

            multipleSameColorDetected = bothVeryClose && colorScoresAgree;
        }
    }

    /**
     * Check if a color sensor detects an artifact with high confidence.
     * Color detection is GATED by proximity - must have physical presence first.
     * This prevents false positives from high-gain color sensors detecting ambient light/reflections.
     *
     * @param sensor Color sensor to check
     * @param proximityDetected Whether corresponding proximity sensor detects artifact
     * @return true if both proximity AND color confidence are high
     */
    private boolean checkColorSensorWithProximity(NormalizedColorSensor sensor, boolean proximityDetected) {
        // GATE: Color detection requires proximity confirmation
        // Without proximity, high-gain sensors can trigger false positives from ambient conditions
        if (!proximityDetected) return false;
        if (sensor == null) return false;

        try {
            double red = sensor.getNormalizedColors().red;
            double green = sensor.getNormalizedColors().green;
            double blue = sensor.getNormalizedColors().blue;

            // Check for minimum brightness
            double maxValue = Math.max(Math.max(red, green), blue);
            if (maxValue < 0.05) return false;  // Too dark

            // Calculate purple and green scores
            double purpleScore = config.calculateColorConfidence(red, green, blue, "PURPLE");
            double greenScore = config.calculateColorConfidence(red, green, blue, "GREEN");

            // Artifact detected if either color has high confidence
            double maxScore = Math.max(purpleScore, greenScore);
            return maxScore >= config.getColorConfidenceThreshold();
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Update best color classification from both sensors
     * Called during stable checkpoints (not continuously)
     */
    private void updateBestColorClassification() {
        // If forced detection is active, use forced color with max confidence
        if (forcedDetectionActive) {
            lastColorClass = forcedColor;
            lastColorConfidence = 1.0;
            return;
        }
        
        // Get readings from both sensors
        ColorObservation leftObs = getColorObservation(leftColorSensor);
        ColorObservation rightObs = getColorObservation(rightColorSensor);

        // Select best observation (highest confidence)
        ColorObservation bestObs = (leftObs.confidence > rightObs.confidence) ? leftObs : rightObs;

        // Update cached classification
        lastColorClass = bestObs.colorClass;
        lastColorConfidence = bestObs.confidence;
    }

    /**
     * Get color observation from a single sensor
     */
    private ColorObservation getColorObservation(NormalizedColorSensor sensor) {
        if (sensor == null) {
            return new ColorObservation(ArtifactIdentity.ColorClass.UNKNOWN, 0.0);
        }

        try {
            double red = sensor.getNormalizedColors().red;
            double green = sensor.getNormalizedColors().green;
            double blue = sensor.getNormalizedColors().blue;

            // Calculate confidence for BOTH colors
            double purpleConfidence = config.calculateColorConfidence(red, green, blue, "PURPLE");
            double greenConfidence = config.calculateColorConfidence(red, green, blue, "GREEN");

            // Get classification from config (may return UNKNOWN)
            String colorStr = config.detectArtifactColor(red, green, blue);
            ArtifactIdentity.ColorClass colorClass = parseColorString(colorStr);

            // Use the best confidence score (highest of purple or green)
            double bestConfidence = Math.max(purpleConfidence, greenConfidence);

            // If confidence is high enough but classification said UNKNOWN, override with best color
            if (colorClass == ArtifactIdentity.ColorClass.UNKNOWN && bestConfidence >= config.getColorConfidenceThreshold()) {
                colorClass = (greenConfidence > purpleConfidence) ?
                    ArtifactIdentity.ColorClass.GREEN : ArtifactIdentity.ColorClass.PURPLE;
            }

            return new ColorObservation(colorClass, bestConfidence);
        } catch (Exception e) {
            return new ColorObservation(ArtifactIdentity.ColorClass.UNKNOWN, 0.0);
        }
    }

    /**
     * Update debounced hints with split strategy
     * - Fast presence: 30ms stability for responsive detection
     * - Stable presence: 100ms stability for confirmation
     */
    private void updateDebouncedHints() {
        boolean currentRawHint = getRawArtifactHint();
        long now = System.currentTimeMillis();

        // Track when raw hint changes
        if (currentRawHint != lastRawHint) {
            lastRawHintChangeTime = now;
            lastRawHint = currentRawHint;
        }

        // Calculate how long signal has been stable
        long stableDuration = now - lastRawHintChangeTime;

        // Fast presence: requires 30ms stability
        if (stableDuration >= EDGE_DETECTION_DEBOUNCE_MS) {
            fastPresence = currentRawHint;
        }

        // Stable presence: requires 100ms stability
        if (stableDuration >= STABLE_PRESENCE_DEBOUNCE_MS) {
            stablePresence = currentRawHint;
        }
    }

    /**
     * Get raw artifact hint (before debounce)
     * True if any sensor indicates presence OR forced detection is active
     * 
     * Detection priority:
     * 1. Left/Right REV Color V3 proximity (main detectors)
     * 2. goBILDA confirmation sensor (secondary confirmation)
     * 3. Color confidence detection (GATED by proximity - prevents false positives)
     *
     * Note: Color detection is now gated by proximity, so colorSeesArtifact_left/right
     * will only be true if proximity sensors also confirm physical presence.
     * This prevents high-gain color sensors from triggering on ambient light.
     */
    private boolean getRawArtifactHint() {
        return forcedDetectionActive || 
               leftProximityDetected ||
               rightProximityDetected ||
               confirmationDetected ||
               colorSeesArtifact_left ||
               colorSeesArtifact_right;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC QUERIES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Enable color sampling (call at checkpoint start)
     */
    public void enableColorSampling() {
        samplingEnabled = true;
    }
    
    /**
     * Disable color sampling (call after checkpoint)
     */
    public void disableColorSampling() {
        samplingEnabled = false;
    }
    
    /**
     * Check if color sampling is enabled
     */
    public boolean isSamplingEnabled() {
        return samplingEnabled;
    }
    
    /**
     * Reset presence detection state.
     * Call this after transfers/swaps to clear sensor state and prevent ghost detection.
     * Resets debounced presence flags and timestamps to force fresh detection.
     */
    public void resetPresenceDetection() {
        fastPresence = false;
        stablePresence = false;
        lastRawHint = false;
        lastRawHintChangeTime = System.currentTimeMillis();
    }

    /**
     * Get fast presence (30ms debounce)
     * Use this for responsive detection (e.g., triggering collection start)
     */
    public boolean getFastPresence() {
        return fastPresence;
    }
    
    /**
     * Get stable presence (100ms debounce)
     * Use this for confirmed "still present" checks
     */
    public boolean getStablePresence() {
        return stablePresence;
    }
    
    /**
     * Get artifact hint (uses stable presence)
     * True if any sensor indicates presence (after full 100ms debounce)
     */
    public boolean getArtifactHint() {
        return stablePresence;
    }

    /**
     * Check if confirmation sensor detects artifact (goBILDA distance)
     */
    public boolean isConfirmationDetected() {
        return confirmationDetected;
    }

    /**
     * Check if primary proximity sensor detects artifact (REV Color V3)
     */
    public boolean isLeftProximityDetected() {
        return leftProximityDetected;
    }

    /**
     * Check if secondary proximity sensor detects artifact (REV Color V3)
     */
    public boolean isRightProximityDetected() {
        return rightProximityDetected;
    }

    // Backward compatibility
    @Deprecated
    public boolean isPrimaryProximityDetected() { return isLeftProximityDetected(); }
    @Deprecated
    public boolean isSecondaryProximityDetected() { return isRightProximityDetected(); }

    /**
     * Check if color sensor sees artifact (left)
     */
    public boolean colorSeesArtifact_Left() {
        return colorSeesArtifact_left;
    }

    /**
     * Check if color sensor sees artifact (right)
     */
    public boolean colorSeesArtifact_Right() {
        return colorSeesArtifact_right;
    }

    /**
     * Get presence confidence level
     * Based on number of sensors detecting artifact
     */
    public PresenceConfidence getPresenceConfidence() {
        int sensorCount = 0;
        if (confirmationDetected) sensorCount++;
        if (leftProximityDetected) sensorCount++;
        if (rightProximityDetected) sensorCount++;
        if (colorSeesArtifact_left) sensorCount++;
        if (colorSeesArtifact_right) sensorCount++;

        if (sensorCount == 0) return PresenceConfidence.NONE;
        if (sensorCount == 1) return PresenceConfidence.LOW;
        if (sensorCount <= 2) return PresenceConfidence.MEDIUM;
        return PresenceConfidence.HIGH;
    }

    /**
     * Get presence confidence as a numerical score (0.0-1.0)
     * More granular than the enum-based getPresenceConfidence()
     *
     * Scoring calculation:
     * - Base: 0% for no sensors
     * - Each sensor adds weight (confirmation=0.15, proximities=0.25 each, colors=0.10 each)
     * - Color detection is GATED by proximity (requires physical presence confirmation)
     * - Max total: 1.0 (all 5 sensors detecting)
     * - Provides smooth confidence scaling
     *
     * Score Ranges:
     * - 0.0: No sensors
     * - 0.15-0.25: Low confidence (1 sensor)
     * - 0.40-0.60: Medium confidence (2 sensors)
     * - 0.65-1.00: High confidence (3+ sensors)
     *
     * Proximity Gating:
     * - Color sensors only contribute when proximity confirms physical presence
     * - Prevents false positives from high-gain sensors detecting ambient light
     * - Maintains high gain for accurate color detection while ensuring reliability
     *
     * @return Presence confidence score 0.0-1.0
     */
    public double getPresenceConfidenceScore() {
        double score = 0.0;

        // Confirmation sensor (goBILDA laser) - reliable
        if (confirmationDetected) score += 0.15;

        // Proximity sensors (REV Color V3) - most reliable for presence
        if (leftProximityDetected) score += 0.25;
        if (rightProximityDetected) score += 0.25;

        // Color detection signals - GATED by proximity (already enforced in checkColorSensorWithProximity)
        // Color only adds weight when proximity confirms physical presence
        if (colorSeesArtifact_left) score += 0.10;
        if (colorSeesArtifact_right) score += 0.10;

        // Clamp to 0.0-1.0 range
        return Math.min(score, 1.0);
    }

    /**
     * Get best color classification from fused sensors
     * Only call at stable checkpoints (after delay)
     */
    public ArtifactIdentity.ColorClass getBestColorClass() {
        return lastColorClass;
    }

    /**
     * Get best color confidence from fused sensors
     */
    public double getBestColorConfidence() {
        return lastColorConfidence;
    }

    /**
     * Check if multiple artifacts of DIFFERENT colors are detected in this intake.
     *
     * Detection Pattern:
     * - Left sensor shows high score for one color (purple or green)
     * - Right sensor shows high score for opposite color
     * - Score difference between sensors exceeds threshold
     *
     * This is a RELIABLE detection method because the color pattern is distinct.
     *
     * @return true if two artifacts of different colors are likely present
     */
    public boolean hasMultipleDifferentColors() {
        return multipleDifferentColorsDetected;
    }

    /**
     * Check if multiple artifacts of SAME color are detected in this intake.
     *
     * Detection Pattern:
     * - Both proximity sensors read < 5cm
     * - Color scores from both sensors are similar (agree on same color)
     *
     * This is LESS RELIABLE because:
     * - Single artifact entering can also trigger both proximity sensors
     * - Proximity readings may vary slightly (one 5cm, one 6cm)
     * - Use with caution and additional confirmation
     *
     * @return true if two artifacts of same color are likely present (less reliable)
     */
    public boolean hasMultipleSameColor() {
        return multipleSameColorDetected;
    }

    /**
     * Check if ANY multiple artifacts are detected (different OR same color).
     *
     * @return true if either multiple different colors OR multiple same color detected
     */
    public boolean hasMultipleArtifacts() {
        return multipleDifferentColorsDetected || multipleSameColorDetected;
    }

    /**
     * Get presence confidence score (for external access).
     *
     * Public wrapper for getPresenceConfidenceScore() to allow other classes to retrieve
     * the numerical confidence score without having direct access to IntakePerception.
     *
     * Score Range: 0.0 (no sensors) to 1.0 (all sensors detecting)
     *
     * Weighting:
     * - Confirmation sensor (goBILDA): 0.15
     * - Left proximity (REV Color V3): 0.25
     * - Right proximity (REV Color V3): 0.25
     * - Left color detection: 0.10
     * - Right color detection: 0.10
     *
     * @return Presence confidence score 0.0-1.0
     */
    public double getConfidenceScore() {
        return getPresenceConfidenceScore();
    }

    /**
     * Get intake side
     */
    public IntakeSide getSide() {
        return side;
    }
    
    /**
     * Get raw RGB values from left color sensor (for debugging)
     * Returns array [red, green, blue] or null if sensor unavailable
     */
    public double[] getLeftColorRaw() {
        if (leftColorSensor == null) return null;
        try {
            return new double[] {
                leftColorSensor.getNormalizedColors().red,
                leftColorSensor.getNormalizedColors().green,
                leftColorSensor.getNormalizedColors().blue
            };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get raw RGB values from right color sensor (for debugging)
     * Returns array [red, green, blue] or null if sensor unavailable
     */
    public double[] getRightColorRaw() {
        if (rightColorSensor == null) return null;
        try {
            return new double[] {
                rightColorSensor.getNormalizedColors().red,
                rightColorSensor.getNormalizedColors().green,
                rightColorSensor.getNormalizedColors().blue
            };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get calculated purple and green confidence scores from left sensor
     * Returns array [purpleScore, greenScore] or null if sensor unavailable
     */
    public double[] getLeftColorScores() {
        if (leftColorSensor == null) return null;
        try {
            double red = leftColorSensor.getNormalizedColors().red;
            double green = leftColorSensor.getNormalizedColors().green;
            double blue = leftColorSensor.getNormalizedColors().blue;
            double purpleScore = config.calculateColorConfidence(red, green, blue, "PURPLE");
            double greenScore = config.calculateColorConfidence(red, green, blue, "GREEN");
            return new double[] { purpleScore, greenScore };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get calculated purple and green confidence scores from right sensor
     * Returns array [purpleScore, greenScore] or null if sensor unavailable
     */
    public double[] getRightColorScores() {
        if (rightColorSensor == null) return null;
        try {
            double red = rightColorSensor.getNormalizedColors().red;
            double green = rightColorSensor.getNormalizedColors().green;
            double blue = rightColorSensor.getNormalizedColors().blue;
            double purpleScore = config.calculateColorConfidence(red, green, blue, "PURPLE");
            double greenScore = config.calculateColorConfidence(red, green, blue, "GREEN");
            return new double[] { purpleScore, greenScore };
        } catch (Exception e) {
            return null;
        }
    }
    
    // Backward compatibility methods for deprecated test classes
    @Deprecated
    public double[] getPrimaryColorRaw() { return getLeftColorRaw(); }
    @Deprecated
    public double[] getSecondaryColorRaw() { return getRightColorRaw(); }
    @Deprecated
    public double[] getPrimaryColorScores() { return getLeftColorScores(); }
    @Deprecated
    public double[] getSecondaryColorScores() { return getRightColorScores(); }
    @Deprecated
    public double[] getOutwardColorScores() { return getLeftColorScores(); }
    @Deprecated
    public double[] getMouthColorScores() { return getRightColorScores(); }
    @Deprecated
    public double[] getOutwardColorRaw() { return getLeftColorRaw(); }
    @Deprecated
    public double[] getMouthColorRaw() { return getRightColorRaw(); }
    @Deprecated
    public boolean isFrontBlocked() { return isConfirmationDetected(); }
    @Deprecated
    public boolean isMouthOccupied() { return isLeftProximityDetected() || isRightProximityDetected(); }
    @Deprecated
    public boolean colorSeesArtifact_Outward() { return colorSeesArtifact_Left(); }
    @Deprecated
    public boolean colorSeesArtifact_Mouth() { return colorSeesArtifact_Right(); }
    @Deprecated
    public boolean colorSeesArtifact_Primary() { return colorSeesArtifact_Left(); }
    @Deprecated
    public boolean colorSeesArtifact_Secondary() { return colorSeesArtifact_Right(); }
    @Deprecated
    public void calibrateRevSensorBaseline() { /* No-op - calibration not needed with new sensors */ }

    /**
     * Force detection of an artifact with specified color (for testing/operator override).
     * This bypasses normal sensor detection and makes the system believe an artifact
     * is present. The forced detection is automatically cleared after it's used in
     * a collection operation.
     * 
     * @param color Color to force detect (PURPLE or GREEN)
     */
    public void forceDetection(ArtifactIdentity.ColorClass color) {
        if (color == ArtifactIdentity.ColorClass.UNKNOWN) {
            // Don't allow forcing UNKNOWN - that defeats the purpose
            return;
        }
        
        forcedDetectionActive = true;
        forcedColor = color;
        
        // Immediately trigger presence signals
        fastPresence = true;
        stablePresence = true;
        lastRawHint = true;
        lastRawHintChangeTime = System.currentTimeMillis();
    }
    
    /**
     * Clear forced detection.
     * Called automatically by CollectOperation after artifact is collected.
     */
    public void clearForcedDetection() {
        forcedDetectionActive = false;
        forcedColor = ArtifactIdentity.ColorClass.UNKNOWN;
    }
    
    /**
     * Check if forced detection is active.
     */
    public boolean isForcedDetectionActive() {
        return forcedDetectionActive;
    }
    
    /**
     * Reset perception state to clear stale sensor data.
     * 
     * This should be called after operations that physically move artifacts away from the intake
     * (e.g., after a successful transfer operation) to prevent false detections from lingering
     * sensor signals.
     * 
     * Resets:
     * - Presence flags (fast, stable)
     * - Raw sensor hints (all detection flags)
     * - Debounce timing
     * - Forced detection state
     * 
     * Does NOT reset:
     * - Configuration
     */
    public void reset() {
        // Clear all presence flags
        fastPresence = false;
        stablePresence = false;
        lastRawHint = false;
        
        // Reset debounce timing
        lastRawHintChangeTime = System.currentTimeMillis();
        
        // Clear raw sensor hints
        confirmationDetected = false;
        leftProximityDetected = false;
        rightProximityDetected = false;
        colorSeesArtifact_left = false;
        colorSeesArtifact_right = false;
        hysteresisState = false;
        
        // Clear forced detection
        forcedDetectionActive = false;
        forcedColor = ArtifactIdentity.ColorClass.UNKNOWN;
        
        // Clear multiple artifact detection
        multipleDifferentColorsDetected = false;
        multipleSameColorDetected = false;

        // Note: samplingEnabled state is preserved (operations control this)
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get telemetry snapshot for debugging
     */
    public String getTelemetrySnapshot() {
        StringBuilder sb = new StringBuilder();
        sb.append(side).append(" Intake:\n");
        
        // Confirmation sensor (goBILDA distance)
        sb.append("  confirmationDetected: ").append(confirmationDetected);
        if (confirmationSensor != null) {
            double distCm = (confirmationSensor.getVoltage() / MAX_LASER_VOLTS) * MAX_LASER_DISTANCE_MM / 10.0;
            sb.append(" (").append(String.format("%.1f", distCm)).append("cm)");
        }
        sb.append("\n");
        
        // Proximity sensors (REV Color V3)
        sb.append("  leftProximity: ").append(leftProximityDetected);
        if (leftColorSensor instanceof DistanceSensor) {
            try {
                double distCm = ((DistanceSensor) leftColorSensor).getDistance(DistanceUnit.CM);
                sb.append(" (").append(String.format("%.1f", distCm)).append("cm)");
            } catch (Exception e) {
                sb.append(" (error)");
            }
        }
        sb.append("\n");
        
        sb.append("  rightProximity: ").append(rightProximityDetected);
        if (rightColorSensor instanceof DistanceSensor) {
            try {
                double distCm = ((DistanceSensor) rightColorSensor).getDistance(DistanceUnit.CM);
                sb.append(" (").append(String.format("%.1f", distCm)).append("cm)");
            } catch (Exception e) {
                sb.append(" (error)");
            }
        }
        sb.append("\n");
        
        sb.append("  colorSeesArtifact: left=").append(colorSeesArtifact_left)
          .append(", right=").append(colorSeesArtifact_right).append("\n");
        sb.append("  fastPresence: ").append(fastPresence).append(" (30ms debounce)\n");
        sb.append("  stablePresence: ").append(stablePresence).append(" (100ms debounce)\n");
        sb.append("  presenceConfidence: ").append(getPresenceConfidence()).append("\n");
        sb.append("  samplingEnabled: ").append(samplingEnabled).append("\n");
        sb.append("  bestColor: ").append(lastColorClass)
          .append(" (conf=").append(String.format("%.2f", lastColorConfidence)).append(")\n");

        // Add raw color sensor values
        double[] leftRaw = getLeftColorRaw();
        if (leftRaw != null) {
            sb.append("  leftColorRaw: R=").append(String.format("%.3f", leftRaw[0]))
              .append(", G=").append(String.format("%.3f", leftRaw[1]))
              .append(", B=").append(String.format("%.3f", leftRaw[2])).append("\n");
        }

        double[] rightRaw = getRightColorRaw();
        if (rightRaw != null) {
            sb.append("  rightColorRaw: R=").append(String.format("%.3f", rightRaw[0]))
              .append(", G=").append(String.format("%.3f", rightRaw[1]))
              .append(", B=").append(String.format("%.3f", rightRaw[2])).append("\n");
        }

        // Add color scores
        double[] leftScores = getLeftColorScores();
        if (leftScores != null) {
            sb.append("  leftScores: Purple=").append(String.format("%.2f", leftScores[0]))
              .append(", Green=").append(String.format("%.2f", leftScores[1])).append("\n");
        }

        double[] rightScores = getRightColorScores();
        if (rightScores != null) {
            sb.append("  rightScores: Purple=").append(String.format("%.2f", rightScores[0]))
              .append(", Green=").append(String.format("%.2f", rightScores[1])).append("\n");
        }

        // Add multiple artifact detection
        sb.append("  multipleArtifacts: ");
        if (multipleDifferentColorsDetected) {
            sb.append("✓ DIFFERENT COLORS (reliable)");
        } else if (multipleSameColorDetected) {
            sb.append("⚠ SAME COLOR (less reliable)");
        } else {
            sb.append("✗ SINGLE OR NONE");
        }

        return sb.toString();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HELPER CLASSES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Color observation from a single sensor
     */
    private static class ColorObservation {
        final ArtifactIdentity.ColorClass colorClass;
        final double confidence;

        ColorObservation(ArtifactIdentity.ColorClass colorClass, double confidence) {
            this.colorClass = colorClass;
            this.confidence = confidence;
        }
    }

    /**
     * Parse color string to ColorClass enum
     */
    private ArtifactIdentity.ColorClass parseColorString(String colorStr) {
        if ("GREEN".equals(colorStr)) return ArtifactIdentity.ColorClass.GREEN;
        if ("PURPLE".equals(colorStr)) return ArtifactIdentity.ColorClass.PURPLE;
        return ArtifactIdentity.ColorClass.UNKNOWN;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CENTER SLOT PERCEPTION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * CenterSlotPerception - Sensor fusion for the center slot / uptake area
     * 
     * This inner class handles detection for the center storage position using:
     * - 1× goBILDA distance sensor (confirmation)
     * - 2× REV Color Sensor V3 (primary detection via color + proximity)
     * 
     * Similar to IntakePerception but specifically for the center slot.
     */
    public static class CenterSlotPerception {
        private final AnalogInput centerDistanceSensor;
        private final NormalizedColorSensor leftColorSensor;
        private final NormalizedColorSensor rightColorSensor;
        private final IndexingConfig config;

        // Detection state
        private boolean centerConfirmation;
        private boolean leftProximityDetected;
        private boolean rightProximityDetected;
        private boolean leftColorDetected;
        private boolean rightColorDetected;

        // Debounced signals
        private boolean artifactPresent;
        private long lastChangeTime;
        
        // Sensor caching (performance optimization)
        private double cachedCenterVoltage;
        private double cachedLeftDistance;
        private double cachedRightDistance;

        // Constants
        private static final double CENTER_CONFIRMATION_THRESHOLD_CM = 10.0;
        private static final double CENTER_LEFT_PROXIMITY_THRESHOLD_CM = 6.0;   // Left sensor threshold (increased from 5.1 to account for actual positioning)
        private static final double CENTER_RIGHT_PROXIMITY_THRESHOLD_CM = 3.5;  // Right sensor threshold
        private static final double MAX_LASER_VOLTS = 3.3;
        private static final double MAX_LASER_DISTANCE_MM = 1000.0;
        private static final long CENTER_DEBOUNCE_MS = 50;

        /**
         * Create center slot perception
         * @param centerDistanceSensor goBILDA distance sensor
         * @param leftColorSensor Left REV Color V3 sensor
         * @param rightColorSensor Right REV Color V3 sensor
         * @param config IndexingConfig
         */
        public CenterSlotPerception(AnalogInput centerDistanceSensor,
                                   NormalizedColorSensor leftColorSensor,
                                   NormalizedColorSensor rightColorSensor,
                                   IndexingConfig config) {
            this.centerDistanceSensor = centerDistanceSensor;
            this.leftColorSensor = leftColorSensor;
            this.rightColorSensor = rightColorSensor;
            this.config = config;
            this.centerConfirmation = false;
            this.leftProximityDetected = false;
            this.rightProximityDetected = false;
            this.leftColorDetected = false;
            this.rightColorDetected = false;
            this.artifactPresent = false;
            this.lastChangeTime = System.currentTimeMillis();
            this.cachedCenterVoltage = 0.0;
            this.cachedLeftDistance = 0.0;
            this.cachedRightDistance = 0.0;
        }

        /**
         * Update all center slot sensors
         * PERFORMANCE: Reads sensors once per call
         */
        public void update() {
            // Read sensors once
            readCenterSensorsOnce();
            
            // Update confirmation sensor (goBILDA distance)
            updateCenterConfirmation();

            // Update proximity sensors (REV Color V3)
            boolean wasLeftDetecting = leftProximityDetected;
            boolean wasRightDetecting = rightProximityDetected;
            updateCenterProximity();

            // Log sensor state changes
            if (leftProximityDetected != wasLeftDetecting) {
                Dbg.d(LogGroup.SENSORS, "Center LEFT proximity changed: %s → %s (dist=%.2fcm)",
                      wasLeftDetecting, leftProximityDetected, cachedLeftDistance);
            }
            if (rightProximityDetected != wasRightDetecting) {
                Dbg.d(LogGroup.SENSORS, "Center RIGHT proximity changed: %s → %s (dist=%.2fcm)",
                      wasRightDetecting, rightProximityDetected, cachedRightDistance);
            }

            // Update color detection
            updateCenterColors();

            // Update debounced artifact presence
            boolean wasPresent = artifactPresent;
            updateArtifactPresence();

            // Log presence state changes
            if (artifactPresent != wasPresent) {
                Dbg.i(LogGroup.SENSORS, "Center artifact presence changed: %s → %s",
                      wasPresent, artifactPresent);
            }
        }
        
        /**
         * Read all center sensors once and cache
         * PERFORMANCE: Single I2C read per sensor
         */
        private void readCenterSensorsOnce() {
            // Read center confirmation sensor
            try {
                cachedCenterVoltage = (centerDistanceSensor != null)
                    ? centerDistanceSensor.getVoltage()
                    : 0.0;
            } catch (Exception e) {
                cachedCenterVoltage = 0.0;
                Dbg.w(LogGroup.SENSORS, "Center distance sensor read error: %s", e.getMessage());
            }
            
            // Read left proximity
            try {
                cachedLeftDistance = (leftColorSensor instanceof DistanceSensor)
                    ? ((DistanceSensor) leftColorSensor).getDistance(DistanceUnit.CM)
                    : 999.0;
            } catch (Exception e) {
                cachedLeftDistance = 999.0;
                Dbg.w(LogGroup.SENSORS, "Center LEFT proximity read error: %s", e.getMessage());
            }
            
            // Read right proximity
            try {
                cachedRightDistance = (rightColorSensor instanceof DistanceSensor)
                    ? ((DistanceSensor) rightColorSensor).getDistance(DistanceUnit.CM)
                    : 999.0;
            } catch (Exception e) {
                cachedRightDistance = 999.0;
                Dbg.w(LogGroup.SENSORS, "Center RIGHT proximity read error: %s", e.getMessage());
            }

            // Log raw readings periodically (every ~500ms) for debugging
            long now = System.currentTimeMillis();
            if (now % 500 < 20) {
                Dbg.d(LogGroup.SENSORS, "Center raw readings: LEFT=%.2fcm, RIGHT=%.2fcm, confirm=%.2fV",
                      cachedLeftDistance, cachedRightDistance, cachedCenterVoltage);
            }
        }

        private void updateCenterConfirmation() {
            if (centerDistanceSensor == null) {
                centerConfirmation = false;
                return;
            }

            try {
                // Use cached voltage
                double voltage = cachedCenterVoltage;
                double distanceMm = (voltage / MAX_LASER_VOLTS) * MAX_LASER_DISTANCE_MM;
                double distanceCm = distanceMm / 10.0;
                centerConfirmation = (distanceCm < CENTER_CONFIRMATION_THRESHOLD_CM && distanceCm > 0.5);
            } catch (Exception e) {
                centerConfirmation = false;
            }
        }

        private void updateCenterProximity() {
            // Left sensor proximity (use cached reading) - 6.0cm threshold
            if (leftColorSensor instanceof DistanceSensor) {
                try {
                    double distCm = cachedLeftDistance;
                    leftProximityDetected = (distCm < CENTER_LEFT_PROXIMITY_THRESHOLD_CM && distCm > 0.1);
                } catch (Exception e) {
                    leftProximityDetected = false;
                }
            } else {
                leftProximityDetected = false;
            }

            // Right sensor proximity (use cached reading) - 3.5cm threshold
            if (rightColorSensor instanceof DistanceSensor) {
                try {
                    double distCm = cachedRightDistance;
                    rightProximityDetected = (distCm < CENTER_RIGHT_PROXIMITY_THRESHOLD_CM && distCm > 0.1);
                } catch (Exception e) {
                    rightProximityDetected = false;
                }
            } else {
                rightProximityDetected = false;
            }
        }

        private void updateCenterColors() {
            leftColorDetected = checkColorSensor(leftColorSensor);
            rightColorDetected = checkColorSensor(rightColorSensor);
        }

        private boolean checkColorSensor(NormalizedColorSensor sensor) {
            if (sensor == null) return false;
            try {
                double red = sensor.getNormalizedColors().red;
                double green = sensor.getNormalizedColors().green;
                double blue = sensor.getNormalizedColors().blue;
                double maxValue = Math.max(Math.max(red, green), blue);
                if (maxValue < 0.05) return false;
                double purpleScore = config.calculateColorConfidence(red, green, blue, "PURPLE");
                double greenScore = config.calculateColorConfidence(red, green, blue, "GREEN");
                double maxScore = Math.max(purpleScore, greenScore);
                return maxScore >= config.getColorConfidenceThreshold();
            } catch (Exception e) {
                return false;
            }
        }

        private void updateArtifactPresence() {
            boolean rawPresence = centerConfirmation || leftProximityDetected || 
                                 rightProximityDetected || leftColorDetected || rightColorDetected;
            
            long now = System.currentTimeMillis();
            if (rawPresence != artifactPresent) {
                if ((now - lastChangeTime) >= CENTER_DEBOUNCE_MS) {
                    artifactPresent = rawPresence;
                    lastChangeTime = now;
                }
            } else {
                lastChangeTime = now;
            }
        }

        /**
         * Check if artifact is present in center slot
         */
        public boolean isArtifactPresent() {
            return artifactPresent;
        }

        /**
         * Check if BOTH proximity sensors detect artifact
         * This is a stricter check for transfer completion
         * @return true only if both left AND right proximity sensors detect
         */
        public boolean isBothProximitySensorsDetecting() {
            boolean both = leftProximityDetected && rightProximityDetected;

            // Log detailed state periodically (every ~200ms)
            long now = System.currentTimeMillis();
            if (now % 200 < 20) {  // Approximately every 200ms
                Dbg.d(LogGroup.SENSORS, "Center sensor check: LEFT=%s (%.2fcm<%s?), RIGHT=%s (%.2fcm<%s?), BOTH=%s",
                      leftProximityDetected ? "✓" : "✗",
                      cachedLeftDistance,
                      CENTER_LEFT_PROXIMITY_THRESHOLD_CM,
                      rightProximityDetected ? "✓" : "✗",
                      cachedRightDistance,
                      CENTER_RIGHT_PROXIMITY_THRESHOLD_CM,
                      both ? "✓✓ YES" : "✗ NO");
            }

            return both;
        }

        /**
         * Get confidence level based on sensor agreement
         */
        public IntakePerception.PresenceConfidence getConfidence() {
            int count = 0;
            if (centerConfirmation) count++;
            if (leftProximityDetected) count++;
            if (rightProximityDetected) count++;
            if (leftColorDetected) count++;
            if (rightColorDetected) count++;

            if (count == 0) return IntakePerception.PresenceConfidence.NONE;
            if (count == 1) return IntakePerception.PresenceConfidence.LOW;
            if (count <= 2) return IntakePerception.PresenceConfidence.MEDIUM;
            return IntakePerception.PresenceConfidence.HIGH;
        }

        /**
         * Get telemetry snapshot
         */
        public String getTelemetrySnapshot() {
            StringBuilder sb = new StringBuilder();
            sb.append("Center Slot:\n");
            sb.append("  confirmation: ").append(centerConfirmation);
            if (centerDistanceSensor != null) {
                double distCm = (centerDistanceSensor.getVoltage() / MAX_LASER_VOLTS) * MAX_LASER_DISTANCE_MM / 10.0;
                sb.append(" (").append(String.format("%.1f", distCm)).append("cm)");
            }
            sb.append("\n");
            sb.append("  leftProximity: ").append(leftProximityDetected).append("\n");
            sb.append("  rightProximity: ").append(rightProximityDetected).append("\n");
            sb.append("  artifactPresent: ").append(artifactPresent).append("\n");
            sb.append("  confidence: ").append(getConfidence());
            return sb.toString();
        }

        /**
         * Reset center slot perception
         */
        public void reset() {
            centerConfirmation = false;
            leftProximityDetected = false;
            rightProximityDetected = false;
            leftColorDetected = false;
            rightColorDetected = false;
            artifactPresent = false;
            lastChangeTime = System.currentTimeMillis();
        }
    }
}
