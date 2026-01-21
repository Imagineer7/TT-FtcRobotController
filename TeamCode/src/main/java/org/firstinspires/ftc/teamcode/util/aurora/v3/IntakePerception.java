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
 * - frontBlocked (outward laser distance sensor)
 * - mouthOccupied (REV 2m ToF sensor with hysteresis)
 * - colorSeesArtifact (color sensor confidence)
 * - artifactHint (combined presence signal)
 * - presenceConfidence (LOW/MED/HIGH)
 *
 * Key principles:
 * - Sensors provide hints, not truth
 * - Hysteresis prevents oscillation
 * - Debounce ensures stable signals
 * - Color classification only at stable checkpoints
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
    private final AnalogInput laserSensor;              // 0-3.3V = 0-1000mm
    private final DistanceSensor revSensor;             // REV 2m ToF
    private final NormalizedColorSensor outwardColorSensor;  // Outward-facing
    private final NormalizedColorSensor mouthColorSensor;    // Mouth-mounted

    // Derived signal state
    private boolean frontBlocked;
    private boolean mouthOccupied;
    private boolean colorSeesArtifact_outward;
    private boolean colorSeesArtifact_mouth;

    // Hysteresis state for REV sensor
    private boolean revSensorHysteresisState;  // Current state (occupied or not)
    private double revSensorBaseline;          // Calibrated baseline distance

    // Debounce state
    private long lastRawHintChangeTime;     // When raw hint last changed
    private boolean fastPresence;           // Fast-debounced presence (30ms)
    private boolean stablePresence;         // Slow-debounced presence (100ms)
    private boolean lastRawHint;

    // Color classification
    private ArtifactIdentity.ColorClass lastColorClass;
    private double lastColorConfidence;
    private boolean samplingEnabled;        // Gates color re-sampling to checkpoints
    
    // Baseline calibration state
    private long lastBaselineUpdateTime;
    private int emptyReadingCount;
    
    // Manual override state (for testing/operator override)
    private boolean forcedDetectionActive;
    private ArtifactIdentity.ColorClass forcedColor;

    // Constants (will be moved to config)
    private static final double LASER_THRESHOLD_CM = 10.0;  // Artifact detected when < 10cm
    private static final double REV_HYSTERESIS_ENTER_DELTA = 7.0;  // cm below baseline to enter occupied state
    private static final double REV_HYSTERESIS_EXIT_DELTA = 3.0;   // cm below baseline to exit occupied state
    private static final double MAX_LASER_VOLTS = 3.3;
    private static final double MAX_LASER_DISTANCE_MM = 1000.0;
    
    // Debounce timing
    private static final long EDGE_DETECTION_DEBOUNCE_MS = 30;    // Fast response for entry/exit
    private static final long STABLE_PRESENCE_DEBOUNCE_MS = 100;  // Confirm still present
    
    // Baseline calibration
    private static final long BASELINE_UPDATE_INTERVAL_MS = 500;  // Check every 500ms
    private static final int EMPTY_READINGS_REQUIRED = 5;         // Need 5 consecutive empty readings

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new IntakePerception for one intake
     *
     * @param side Which intake (FRONT or BACK)
     * @param laserSensor Outward-facing laser distance sensor
     * @param revSensor REV 2m ToF distance sensor (across mouth)
     * @param outwardColorSensor Color sensor next to laser
     * @param mouthColorSensor Color sensor opposite REV sensor
     * @param config IndexingConfig for thresholds
     */
    public IntakePerception(IntakeSide side,
                           AnalogInput laserSensor,
                           DistanceSensor revSensor,
                           NormalizedColorSensor outwardColorSensor,
                           NormalizedColorSensor mouthColorSensor,
                           IndexingConfig config) {
        this.side = side;
        this.laserSensor = laserSensor;
        this.revSensor = revSensor;
        this.outwardColorSensor = outwardColorSensor;
        this.mouthColorSensor = mouthColorSensor;
        this.config = config;

        // Initialize state
        this.frontBlocked = false;
        this.mouthOccupied = false;
        this.colorSeesArtifact_outward = false;
        this.colorSeesArtifact_mouth = false;
        this.revSensorHysteresisState = false;
        this.fastPresence = false;
        this.stablePresence = false;
        this.lastRawHint = false;
        this.lastRawHintChangeTime = System.currentTimeMillis();
        this.lastColorClass = ArtifactIdentity.ColorClass.UNKNOWN;
        this.lastColorConfidence = 0.0;
        this.samplingEnabled = false;
        this.lastBaselineUpdateTime = System.currentTimeMillis();
        this.emptyReadingCount = 0;
        this.forcedDetectionActive = false;
        this.forcedColor = ArtifactIdentity.ColorClass.UNKNOWN;

        // Calibrate REV sensor baseline
        calibrateRevSensorBaseline();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UPDATE METHOD (Call every loop)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update all sensor readings and derived signals
     * Call this every loop
     */
    public void update() {
        // Update laser distance sensor (frontBlocked)
        updateLaserSensor();

        // Update REV 2m sensor with hysteresis (mouthOccupied)
        updateRevSensor();

        // Update color sensors (colorSeesArtifact)
        updateColorSensors();

        // Update debounced hints (edge detection + stable presence)
        updateDebouncedHints();
        
        // Update baseline calibration (continuous when confidently empty)
        updateBaselineCalibration();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SENSOR UPDATE METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update laser distance sensor reading
     * Converts voltage to distance and checks threshold
     */
    private void updateLaserSensor() {
        if (laserSensor == null) {
            frontBlocked = false;
            return;
        }

        try {
            double voltage = laserSensor.getVoltage();
            double distanceMm = (voltage / MAX_LASER_VOLTS) * MAX_LASER_DISTANCE_MM;
            double distanceCm = distanceMm / 10.0;

            // Artifact detected when distance < threshold
            frontBlocked = (distanceCm < LASER_THRESHOLD_CM && distanceCm > 0.5);  // > 0.5 filters noise
        } catch (Exception e) {
            frontBlocked = false;  // Sensor error = assume empty
        }
    }

    /**
     * Update REV 2m distance sensor with hysteresis
     * Prevents oscillation when artifact is near threshold
     * 
     * Hysteresis: 
     * - Enter occupied when distance < baseline - ENTER_DELTA (artifact gets close)
     * - Exit occupied when distance > baseline - EXIT_DELTA (artifact moves away)
     * - ENTER_DELTA > EXIT_DELTA ensures no oscillation in the band
     */
    private void updateRevSensor() {
        if (revSensor == null || !config.getUseRevDistanceSensors()) {
            mouthOccupied = false;
            return;
        }

        try {
            double distanceCm = revSensor.getDistance(DistanceUnit.CM);

            // Hysteresis thresholds
            double enterThreshold = revSensorBaseline - REV_HYSTERESIS_ENTER_DELTA;  // e.g., 25 - 7 = 18cm
            double exitThreshold = revSensorBaseline - REV_HYSTERESIS_EXIT_DELTA;    // e.g., 25 - 3 = 22cm

            if (revSensorHysteresisState) {
                // Currently occupied - check if artifact left (distance > exitThreshold)
                if (distanceCm > exitThreshold) {
                    revSensorHysteresisState = false;
                }
            } else {
                // Currently empty - check if artifact entered (distance < enterThreshold)
                if (distanceCm < enterThreshold && distanceCm > 0.5) {  // > 0.5 filters invalid readings
                    revSensorHysteresisState = true;
                }
            }

            mouthOccupied = revSensorHysteresisState;
        } catch (Exception e) {
            mouthOccupied = false;  // Sensor error = assume empty
        }
    }

    /**
     * Update color sensors and check for high-confidence artifact detection
     * Uses color classification logic from IndexingConfig
     */
    private void updateColorSensors() {
        colorSeesArtifact_outward = checkColorSensor(outwardColorSensor);
        colorSeesArtifact_mouth = checkColorSensor(mouthColorSensor);

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
        ColorObservation outwardObs = getColorObservation(outwardColorSensor);
        ColorObservation mouthObs = getColorObservation(mouthColorSensor);

        // Select best observation (highest confidence)
        ColorObservation bestObs = (outwardObs.confidence > mouthObs.confidence) ? outwardObs : mouthObs;

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
     * Update REV sensor baseline calibration
     * Only updates when confidently empty and rollers are off
     */
    private void updateBaselineCalibration() {
        if (revSensor == null || !config.getUseRevDistanceSensors()) {
            return;
        }
        
        long now = System.currentTimeMillis();
        if ((now - lastBaselineUpdateTime) < BASELINE_UPDATE_INTERVAL_MS) {
            return;  // Not time to check yet
        }
        
        lastBaselineUpdateTime = now;
        
        // Only update baseline when confidently empty
        // (no artifact hint, and stable for a while)
        if (!stablePresence && !fastPresence) {
            try {
                double currentDistance = revSensor.getDistance(DistanceUnit.CM);
                
                // Valid reading in reasonable range
                if (currentDistance > 10.0 && currentDistance < 100.0) {
                    emptyReadingCount++;
                    
                    if (emptyReadingCount >= EMPTY_READINGS_REQUIRED) {
                        // Drift baseline slowly toward current reading
                        // Use exponential moving average with alpha = 0.1
                        double alpha = 0.1;
                        revSensorBaseline = alpha * currentDistance + (1 - alpha) * revSensorBaseline;
                    }
                } else {
                    emptyReadingCount = 0;
                }
            } catch (Exception e) {
                emptyReadingCount = 0;
            }
        } else {
            // Artifact present - reset empty counter
            emptyReadingCount = 0;
        }
    }

    /**
     * Get raw artifact hint (before debounce)
     * True if any sensor indicates presence OR forced detection is active
     */
    private boolean getRawArtifactHint() {
        return forcedDetectionActive || frontBlocked || mouthOccupied || 
               colorSeesArtifact_outward || colorSeesArtifact_mouth;
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
     * Check if front is blocked (laser sensor)
     */
    public boolean isFrontBlocked() {
        return frontBlocked;
    }

    /**
     * Check if mouth is occupied (REV sensor with hysteresis)
     */
    public boolean isMouthOccupied() {
        return mouthOccupied;
    }

    /**
     * Check if color sensor sees artifact (outward)
     */
    public boolean colorSeesArtifact_Outward() {
        return colorSeesArtifact_outward;
    }

    /**
     * Check if color sensor sees artifact (mouth)
     */
    public boolean colorSeesArtifact_Mouth() {
        return colorSeesArtifact_mouth;
    }

    /**
     * Get presence confidence level
     */
    public PresenceConfidence getPresenceConfidence() {
        int sensorCount = 0;
        if (frontBlocked) sensorCount++;
        if (mouthOccupied) sensorCount++;
        if (colorSeesArtifact_outward) sensorCount++;
        if (colorSeesArtifact_mouth) sensorCount++;

        if (sensorCount == 0) return PresenceConfidence.NONE;
        if (sensorCount == 1) return PresenceConfidence.LOW;
        if (sensorCount == 2) return PresenceConfidence.MEDIUM;
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
     * Get raw RGB values from outward color sensor (for debugging)
     * Returns array [red, green, blue] or null if sensor unavailable
     */
    public double[] getOutwardColorRaw() {
        if (outwardColorSensor == null) return null;
        try {
            return new double[] {
                outwardColorSensor.getNormalizedColors().red,
                outwardColorSensor.getNormalizedColors().green,
                outwardColorSensor.getNormalizedColors().blue
            };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get raw RGB values from mouth color sensor (for debugging)
     * Returns array [red, green, blue] or null if sensor unavailable
     */
    public double[] getMouthColorRaw() {
        if (mouthColorSensor == null) return null;
        try {
            return new double[] {
                mouthColorSensor.getNormalizedColors().red,
                mouthColorSensor.getNormalizedColors().green,
                mouthColorSensor.getNormalizedColors().blue
            };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get calculated purple and green confidence scores from outward sensor
     * Returns array [purpleScore, greenScore] or null if sensor unavailable
     */
    public double[] getOutwardColorScores() {
        if (outwardColorSensor == null) return null;
        try {
            double red = outwardColorSensor.getNormalizedColors().red;
            double green = outwardColorSensor.getNormalizedColors().green;
            double blue = outwardColorSensor.getNormalizedColors().blue;
            double purpleScore = config.calculateColorConfidence(red, green, blue, "PURPLE");
            double greenScore = config.calculateColorConfidence(red, green, blue, "GREEN");
            return new double[] { purpleScore, greenScore };
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Get calculated purple and green confidence scores from mouth sensor
     * Returns array [purpleScore, greenScore] or null if sensor unavailable
     */
    public double[] getMouthColorScores() {
        if (mouthColorSensor == null) return null;
        try {
            double red = mouthColorSensor.getNormalizedColors().red;
            double green = mouthColorSensor.getNormalizedColors().green;
            double blue = mouthColorSensor.getNormalizedColors().blue;
            double purpleScore = config.calculateColorConfidence(red, green, blue, "PURPLE");
            double greenScore = config.calculateColorConfidence(red, green, blue, "GREEN");
            return new double[] { purpleScore, greenScore };
        } catch (Exception e) {
            return null;
        }
    }

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

    // ═══════════════════════════════════════════════════════════════════════
    // CALIBRATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Calibrate REV sensor baseline (call when intake is empty)
     * Takes average of 10 readings
     */
    public void calibrateRevSensorBaseline() {
        if (revSensor == null || !config.getUseRevDistanceSensors()) {
            revSensorBaseline = config.getRevSensorBaselineDistance();
            return;
        }

        try {
            double sum = 0;
            int validReadings = 0;

            for (int i = 0; i < 10; i++) {
                double reading = revSensor.getDistance(DistanceUnit.CM);
                if (reading > 0.5 && reading < 100.0) {  // Filter invalid readings
                    sum += reading;
                    validReadings++;
                }
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
            }

            if (validReadings > 0) {
                revSensorBaseline = sum / validReadings;
            } else {
                revSensorBaseline = config.getRevSensorBaselineDistance();  // Use default
            }
        } catch (Exception e) {
            revSensorBaseline = config.getRevSensorBaselineDistance();  // Use default on error
        }
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
        sb.append("  frontBlocked: ").append(frontBlocked);
        if (laserSensor != null) {
            double distCm = (laserSensor.getVoltage() / MAX_LASER_VOLTS) * MAX_LASER_DISTANCE_MM / 10.0;
            sb.append(" (").append(String.format("%.1f", distCm)).append("cm)");
        }
        sb.append("\n");
        
        sb.append("  mouthOccupied: ").append(mouthOccupied);
        if (revSensor != null && config.getUseRevDistanceSensors()) {
            try {
                double distCm = revSensor.getDistance(DistanceUnit.CM);
                sb.append(" (").append(String.format("%.1f", distCm)).append("cm, baseline ")
                  .append(String.format("%.1f", revSensorBaseline)).append("cm)");
            } catch (Exception e) {
                sb.append(" (error)");
            }
        }
        sb.append("\n");
        
        sb.append("  colorSeesArtifact: outward=").append(colorSeesArtifact_outward)
          .append(", mouth=").append(colorSeesArtifact_mouth).append("\n");
        sb.append("  fastPresence: ").append(fastPresence).append(" (30ms debounce)\n");
        sb.append("  stablePresence: ").append(stablePresence).append(" (100ms debounce)\n");
        sb.append("  presenceConfidence: ").append(getPresenceConfidence()).append("\n");
        sb.append("  samplingEnabled: ").append(samplingEnabled).append("\n");
        sb.append("  bestColor: ").append(lastColorClass)
          .append(" (conf=").append(String.format("%.2f", lastColorConfidence)).append(")\n");

        // Add raw color sensor values
        double[] outwardRaw = getOutwardColorRaw();
        if (outwardRaw != null) {
            sb.append("  outwardColorRaw: R=").append(String.format("%.3f", outwardRaw[0]))
              .append(", G=").append(String.format("%.3f", outwardRaw[1]))
              .append(", B=").append(String.format("%.3f", outwardRaw[2])).append("\n");
        }

        double[] mouthRaw = getMouthColorRaw();
        if (mouthRaw != null) {
            sb.append("  mouthColorRaw: R=").append(String.format("%.3f", mouthRaw[0]))
              .append(", G=").append(String.format("%.3f", mouthRaw[1]))
              .append(", B=").append(String.format("%.3f", mouthRaw[2])).append("\n");
        }

        // Add color scores
        double[] outwardScores = getOutwardColorScores();
        if (outwardScores != null) {
            sb.append("  outwardScores: Purple=").append(String.format("%.2f", outwardScores[0]))
              .append(", Green=").append(String.format("%.2f", outwardScores[1])).append("\n");
        }

        double[] mouthScores = getMouthColorScores();
        if (mouthScores != null) {
            sb.append("  mouthScores: Purple=").append(String.format("%.2f", mouthScores[0]))
              .append(", Green=").append(String.format("%.2f", mouthScores[1]));
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
}
