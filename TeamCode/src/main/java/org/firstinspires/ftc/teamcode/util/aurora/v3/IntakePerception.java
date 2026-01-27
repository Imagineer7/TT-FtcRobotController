package org.firstinspires.ftc.teamcode.util.aurora.v3;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;

/**
 * IntakePerception - Sensor fusion for a single intake (front or back)
 *
 * This class fuses multiple sensors per intake to produce derived signals:
 * - primaryProximity (REV Color V3 proximity detection - primary)
 * - secondaryProximity (REV Color V3 proximity detection - secondary)
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
 * - 2× REV Color Sensor V3 (primary detection via color + proximity)
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
    private final NormalizedColorSensor primaryColorSensor;    // Primary REV Color V3 (color + proximity)
    private final NormalizedColorSensor secondaryColorSensor;  // Secondary REV Color V3 (color + proximity)

    // Derived signal state
    private boolean confirmationDetected;        // goBILDA distance sensor
    private boolean primaryProximityDetected;    // REV Color V3 proximity (primary)
    private boolean secondaryProximityDetected;  // REV Color V3 proximity (secondary)
    private boolean colorSeesArtifact_primary;   // Primary color confidence
    private boolean colorSeesArtifact_secondary; // Secondary color confidence

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
    
    // Manual override state (for testing/operator override)
    private boolean forcedDetectionActive;
    private ArtifactIdentity.ColorClass forcedColor;

    // Constants
    private static final double CONFIRMATION_THRESHOLD_CM = 10.0;  // Artifact detected when < 10cm (goBILDA)
    private static final double PROXIMITY_THRESHOLD_CM = 5.0;       // REV Color V3 proximity threshold
    private static final double MAX_LASER_VOLTS = 3.3;
    private static final double MAX_LASER_DISTANCE_MM = 1000.0;
    
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
     * @param primaryColorSensor Primary REV Color V3 sensor (color + proximity)
     * @param secondaryColorSensor Secondary REV Color V3 sensor (color + proximity)
     * @param config IndexingConfig for thresholds
     */
    public IntakePerception(IntakeSide side,
                           AnalogInput confirmationSensor,
                           NormalizedColorSensor primaryColorSensor,
                           NormalizedColorSensor secondaryColorSensor,
                           IndexingConfig config) {
        this.side = side;
        this.confirmationSensor = confirmationSensor;
        this.primaryColorSensor = primaryColorSensor;
        this.secondaryColorSensor = secondaryColorSensor;
        this.config = config;

        // Initialize state
        this.confirmationDetected = false;
        this.primaryProximityDetected = false;
        this.secondaryProximityDetected = false;
        this.colorSeesArtifact_primary = false;
        this.colorSeesArtifact_secondary = false;
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
    }
    
    /**
     * DEPRECATED: Old constructor for backward compatibility
     * Maps old sensor parameters to new layout
     * 
     * @deprecated Use new constructor with updated sensor layout
     */
    @Deprecated
    public IntakePerception(IntakeSide side,
                           AnalogInput laserSensor,
                           DistanceSensor revSensor,  // REMOVED - ignored
                           NormalizedColorSensor outwardColorSensor,
                           NormalizedColorSensor mouthColorSensor,
                           IndexingConfig config) {
        // Call new constructor, mapping old sensors to new ones
        this(side, laserSensor, outwardColorSensor, mouthColorSensor, config);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UPDATE METHOD (Call every loop)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update all sensor readings and derived signals
     * Call this every loop
     */
    public void update() {
        // Update confirmation sensor (goBILDA distance)
        updateConfirmationSensor();

        // Update REV Color V3 proximity sensors (primary detectors)
        updateProximitySensors();

        // Update color sensors (for color classification)
        updateColorSensors();

        // Update debounced hints (edge detection + stable presence)
        updateDebouncedHints();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SENSOR UPDATE METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update confirmation sensor (goBILDA distance sensor)
     * Converts voltage to distance and checks threshold
     */
    private void updateConfirmationSensor() {
        if (confirmationSensor == null) {
            confirmationDetected = false;
            return;
        }

        try {
            double voltage = confirmationSensor.getVoltage();
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
     */
    private void updateProximitySensors() {
        // Update primary sensor proximity
        if (primaryColorSensor instanceof DistanceSensor) {
            try {
                double distanceCm = ((DistanceSensor) primaryColorSensor).getDistance(DistanceUnit.CM);
                primaryProximityDetected = (distanceCm < PROXIMITY_THRESHOLD_CM && distanceCm > 0.1);
            } catch (Exception e) {
                primaryProximityDetected = false;
            }
        } else {
            primaryProximityDetected = false;
        }

        // Update secondary sensor proximity
        if (secondaryColorSensor instanceof DistanceSensor) {
            try {
                double distanceCm = ((DistanceSensor) secondaryColorSensor).getDistance(DistanceUnit.CM);
                secondaryProximityDetected = (distanceCm < PROXIMITY_THRESHOLD_CM && distanceCm > 0.1);
            } catch (Exception e) {
                secondaryProximityDetected = false;
            }
        } else {
            secondaryProximityDetected = false;
        }
    }

    /**
     * Update color sensors and check for high-confidence artifact detection
     * Uses color classification logic from IndexingConfig
     */
    private void updateColorSensors() {
        colorSeesArtifact_primary = checkColorSensor(primaryColorSensor);
        colorSeesArtifact_secondary = checkColorSensor(secondaryColorSensor);

        // Only update best color classification when sampling is enabled (at checkpoints)
        if (samplingEnabled) {
            updateBestColorClassification();
        }
        // Otherwise keep last sampled color (don't resample while moving)
    }

    /**
     * Check if a color sensor detects an artifact with high confidence
     */
    private boolean checkColorSensor(NormalizedColorSensor sensor) {
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
        ColorObservation primaryObs = getColorObservation(primaryColorSensor);
        ColorObservation secondaryObs = getColorObservation(secondaryColorSensor);

        // Select best observation (highest confidence)
        ColorObservation bestObs = (primaryObs.confidence > secondaryObs.confidence) ? primaryObs : secondaryObs;

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
     * 1. Primary/Secondary REV Color V3 proximity (main detectors)
     * 2. goBILDA confirmation sensor (secondary confirmation)
     * 3. Color confidence detection
     */
    private boolean getRawArtifactHint() {
        return forcedDetectionActive || 
               primaryProximityDetected || 
               secondaryProximityDetected ||
               confirmationDetected ||
               colorSeesArtifact_primary || 
               colorSeesArtifact_secondary;
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
    public boolean isPrimaryProximityDetected() {
        return primaryProximityDetected;
    }

    /**
     * Check if secondary proximity sensor detects artifact (REV Color V3)
     */
    public boolean isSecondaryProximityDetected() {
        return secondaryProximityDetected;
    }

    /**
     * Check if color sensor sees artifact (primary)
     */
    public boolean colorSeesArtifact_Primary() {
        return colorSeesArtifact_primary;
    }

    /**
     * Check if color sensor sees artifact (secondary)
     */
    public boolean colorSeesArtifact_Secondary() {
        return colorSeesArtifact_secondary;
    }

    /**
     * Get presence confidence level
     * Based on number of sensors detecting artifact
     */
    public PresenceConfidence getPresenceConfidence() {
        int sensorCount = 0;
        if (confirmationDetected) sensorCount++;
        if (primaryProximityDetected) sensorCount++;
        if (secondaryProximityDetected) sensorCount++;
        if (colorSeesArtifact_primary) sensorCount++;
        if (colorSeesArtifact_secondary) sensorCount++;

        if (sensorCount == 0) return PresenceConfidence.NONE;
        if (sensorCount == 1) return PresenceConfidence.LOW;
        if (sensorCount <= 2) return PresenceConfidence.MEDIUM;
        return PresenceConfidence.HIGH;
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
     * Get intake side
     */
    public IntakeSide getSide() {
        return side;
    }
    
    /**
     * Get raw RGB values from primary color sensor (for debugging)
     * Returns array [red, green, blue] or null if sensor unavailable
     */
    public double[] getPrimaryColorRaw() {
        if (primaryColorSensor == null) return null;
        try {
            return new double[] {
                primaryColorSensor.getNormalizedColors().red,
                primaryColorSensor.getNormalizedColors().green,
                primaryColorSensor.getNormalizedColors().blue
            };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get raw RGB values from secondary color sensor (for debugging)
     * Returns array [red, green, blue] or null if sensor unavailable
     */
    public double[] getSecondaryColorRaw() {
        if (secondaryColorSensor == null) return null;
        try {
            return new double[] {
                secondaryColorSensor.getNormalizedColors().red,
                secondaryColorSensor.getNormalizedColors().green,
                secondaryColorSensor.getNormalizedColors().blue
            };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get calculated purple and green confidence scores from primary sensor
     * Returns array [purpleScore, greenScore] or null if sensor unavailable
     */
    public double[] getPrimaryColorScores() {
        if (primaryColorSensor == null) return null;
        try {
            double red = primaryColorSensor.getNormalizedColors().red;
            double green = primaryColorSensor.getNormalizedColors().green;
            double blue = primaryColorSensor.getNormalizedColors().blue;
            double purpleScore = config.calculateColorConfidence(red, green, blue, "PURPLE");
            double greenScore = config.calculateColorConfidence(red, green, blue, "GREEN");
            return new double[] { purpleScore, greenScore };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get calculated purple and green confidence scores from secondary sensor
     * Returns array [purpleScore, greenScore] or null if sensor unavailable
     */
    public double[] getSecondaryColorScores() {
        if (secondaryColorSensor == null) return null;
        try {
            double red = secondaryColorSensor.getNormalizedColors().red;
            double green = secondaryColorSensor.getNormalizedColors().green;
            double blue = secondaryColorSensor.getNormalizedColors().blue;
            double purpleScore = config.calculateColorConfidence(red, green, blue, "PURPLE");
            double greenScore = config.calculateColorConfidence(red, green, blue, "GREEN");
            return new double[] { purpleScore, greenScore };
        } catch (Exception e) {
            return null;
        }
    }
    
    // Backward compatibility methods for test classes
    @Deprecated
    public double[] getOutwardColorScores() { return getPrimaryColorScores(); }
    @Deprecated
    public double[] getMouthColorScores() { return getSecondaryColorScores(); }
    @Deprecated
    public double[] getOutwardColorRaw() { return getPrimaryColorRaw(); }
    @Deprecated
    public double[] getMouthColorRaw() { return getSecondaryColorRaw(); }
    @Deprecated
    public boolean isFrontBlocked() { return isConfirmationDetected(); }
    @Deprecated
    public boolean isMouthOccupied() { return isPrimaryProximityDetected() || isSecondaryProximityDetected(); }
    @Deprecated
    public boolean colorSeesArtifact_Outward() { return colorSeesArtifact_Primary(); }
    @Deprecated
    public boolean colorSeesArtifact_Mouth() { return colorSeesArtifact_Secondary(); }
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
        primaryProximityDetected = false;
        secondaryProximityDetected = false;
        colorSeesArtifact_primary = false;
        colorSeesArtifact_secondary = false;
        hysteresisState = false;
        
        // Clear forced detection
        forcedDetectionActive = false;
        forcedColor = ArtifactIdentity.ColorClass.UNKNOWN;
        
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
        sb.append("  primaryProximity: ").append(primaryProximityDetected);
        if (primaryColorSensor instanceof DistanceSensor) {
            try {
                double distCm = ((DistanceSensor) primaryColorSensor).getDistance(DistanceUnit.CM);
                sb.append(" (").append(String.format("%.1f", distCm)).append("cm)");
            } catch (Exception e) {
                sb.append(" (error)");
            }
        }
        sb.append("\n");
        
        sb.append("  secondaryProximity: ").append(secondaryProximityDetected);
        if (secondaryColorSensor instanceof DistanceSensor) {
            try {
                double distCm = ((DistanceSensor) secondaryColorSensor).getDistance(DistanceUnit.CM);
                sb.append(" (").append(String.format("%.1f", distCm)).append("cm)");
            } catch (Exception e) {
                sb.append(" (error)");
            }
        }
        sb.append("\n");
        
        sb.append("  colorSeesArtifact: primary=").append(colorSeesArtifact_primary)
          .append(", secondary=").append(colorSeesArtifact_secondary).append("\n");
        sb.append("  fastPresence: ").append(fastPresence).append(" (30ms debounce)\n");
        sb.append("  stablePresence: ").append(stablePresence).append(" (100ms debounce)\n");
        sb.append("  presenceConfidence: ").append(getPresenceConfidence()).append("\n");
        sb.append("  samplingEnabled: ").append(samplingEnabled).append("\n");
        sb.append("  bestColor: ").append(lastColorClass)
          .append(" (conf=").append(String.format("%.2f", lastColorConfidence)).append(")\n");

        // Add raw color sensor values
        double[] primaryRaw = getPrimaryColorRaw();
        if (primaryRaw != null) {
            sb.append("  primaryColorRaw: R=").append(String.format("%.3f", primaryRaw[0]))
              .append(", G=").append(String.format("%.3f", primaryRaw[1]))
              .append(", B=").append(String.format("%.3f", primaryRaw[2])).append("\n");
        }

        double[] secondaryRaw = getSecondaryColorRaw();
        if (secondaryRaw != null) {
            sb.append("  secondaryColorRaw: R=").append(String.format("%.3f", secondaryRaw[0]))
              .append(", G=").append(String.format("%.3f", secondaryRaw[1]))
              .append(", B=").append(String.format("%.3f", secondaryRaw[2])).append("\n");
        }

        // Add color scores
        double[] primaryScores = getPrimaryColorScores();
        if (primaryScores != null) {
            sb.append("  primaryScores: Purple=").append(String.format("%.2f", primaryScores[0]))
              .append(", Green=").append(String.format("%.2f", primaryScores[1])).append("\n");
        }

        double[] secondaryScores = getSecondaryColorScores();
        if (secondaryScores != null) {
            sb.append("  secondaryScores: Purple=").append(String.format("%.2f", secondaryScores[0]))
              .append(", Green=").append(String.format("%.2f", secondaryScores[1]));
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

        // Constants
        private static final double CENTER_CONFIRMATION_THRESHOLD_CM = 10.0;
        private static final double CENTER_PROXIMITY_THRESHOLD_CM = 5.0;
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
        }

        /**
         * Update all center slot sensors
         */
        public void update() {
            // Update confirmation sensor (goBILDA distance)
            updateCenterConfirmation();

            // Update proximity sensors (REV Color V3)
            updateCenterProximity();

            // Update color detection
            updateCenterColors();

            // Update debounced artifact presence
            updateArtifactPresence();
        }

        private void updateCenterConfirmation() {
            if (centerDistanceSensor == null) {
                centerConfirmation = false;
                return;
            }

            try {
                double voltage = centerDistanceSensor.getVoltage();
                double distanceMm = (voltage / MAX_LASER_VOLTS) * MAX_LASER_DISTANCE_MM;
                double distanceCm = distanceMm / 10.0;
                centerConfirmation = (distanceCm < CENTER_CONFIRMATION_THRESHOLD_CM && distanceCm > 0.5);
            } catch (Exception e) {
                centerConfirmation = false;
            }
        }

        private void updateCenterProximity() {
            // Left sensor proximity
            if (leftColorSensor instanceof DistanceSensor) {
                try {
                    double distCm = ((DistanceSensor) leftColorSensor).getDistance(DistanceUnit.CM);
                    leftProximityDetected = (distCm < CENTER_PROXIMITY_THRESHOLD_CM && distCm > 0.1);
                } catch (Exception e) {
                    leftProximityDetected = false;
                }
            } else {
                leftProximityDetected = false;
            }

            // Right sensor proximity
            if (rightColorSensor instanceof DistanceSensor) {
                try {
                    double distCm = ((DistanceSensor) rightColorSensor).getDistance(DistanceUnit.CM);
                    rightProximityDetected = (distCm < CENTER_PROXIMITY_THRESHOLD_CM && distCm > 0.1);
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
