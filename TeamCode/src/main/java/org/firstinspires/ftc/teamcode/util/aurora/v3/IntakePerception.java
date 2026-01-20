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
    private long lastEdgeDetectionTime;
    private long lastStablePresenceTime;
    private boolean edgeDetected;           // Fast edge detection for entry events
    private boolean stablePresence;         // Slower stable presence confirmation
    private boolean lastRawHint;

    // Color classification
    private ArtifactIdentity.ColorClass lastColorClass;
    private double lastColorConfidence;
    
    // Baseline calibration state
    private long lastBaselineUpdateTime;
    private int emptyReadingCount;

    // Constants (will be moved to config)
    private static final double LASER_THRESHOLD_CM = 10.0;  // Artifact detected when < 10cm
    private static final double REV_HYSTERESIS_HIGH_OFFSET = 7.0;  // cm above baseline = exit threshold
    private static final double REV_HYSTERESIS_LOW_OFFSET = 3.0;   // cm below baseline = enter threshold
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
        this.edgeDetected = false;
        this.stablePresence = false;
        this.lastRawHint = false;
        this.lastEdgeDetectionTime = System.currentTimeMillis();
        this.lastStablePresenceTime = System.currentTimeMillis();
        this.lastColorClass = ArtifactIdentity.ColorClass.UNKNOWN;
        this.lastColorConfidence = 0.0;
        this.lastBaselineUpdateTime = System.currentTimeMillis();
        this.emptyReadingCount = 0;

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
     */
    private void updateRevSensor() {
        if (revSensor == null || !config.getUseRevDistanceSensors()) {
            mouthOccupied = false;
            return;
        }

        try {
            double distanceCm = revSensor.getDistance(DistanceUnit.CM);

            // Hysteresis thresholds
            double enterThreshold = revSensorBaseline - REV_HYSTERESIS_LOW_OFFSET;   // e.g., 25 - 7 = 18cm
            double exitThreshold = revSensorBaseline - REV_HYSTERESIS_HIGH_OFFSET;   // e.g., 25 - 3 = 22cm

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

        // Update best color classification (for checkpoints)
        updateBestColorClassification();
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

            String colorStr = config.detectArtifactColor(red, green, blue);
            ArtifactIdentity.ColorClass colorClass = parseColorString(colorStr);
            double confidence = config.calculateColorConfidence(red, green, blue, colorStr);

            return new ColorObservation(colorClass, confidence);
        } catch (Exception e) {
            return new ColorObservation(ArtifactIdentity.ColorClass.UNKNOWN, 0.0);
        }
    }

    /**
     * Update debounced hints with split strategy
     * - Edge detection: fast (30ms) for catching entry/exit events
     * - Stable presence: slower (100ms) for confirming artifact still there
     */
    private void updateDebouncedHints() {
        boolean currentRawHint = getRawArtifactHint();
        long now = System.currentTimeMillis();

        // Edge detection: fast response for entry/exit events
        if (currentRawHint != lastRawHint) {
            // Edge detected (signal changed)
            if ((now - lastEdgeDetectionTime) >= EDGE_DETECTION_DEBOUNCE_MS) {
                // Debounce time elapsed - recognize edge
                edgeDetected = currentRawHint;
                lastEdgeDetectionTime = now;
            }
        } else {
            // Signal stable
            lastEdgeDetectionTime = now;
        }

        // Stable presence: slower confirmation for "still present"
        if (currentRawHint != stablePresence) {
            // Presence state wants to change
            if ((now - lastStablePresenceTime) >= STABLE_PRESENCE_DEBOUNCE_MS) {
                // Debounce time elapsed - update stable presence
                stablePresence = currentRawHint;
                lastStablePresenceTime = now;
            }
        } else {
            // State stable
            lastStablePresenceTime = now;
        }

        lastRawHint = currentRawHint;
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
        if (!stablePresence && !edgeDetected) {
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
     * True if any sensor indicates presence
     */
    private boolean getRawArtifactHint() {
        return frontBlocked || mouthOccupied || colorSeesArtifact_outward || colorSeesArtifact_mouth;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC QUERIES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get fast edge detection (for initial entry detection)
     * Use this to trigger "first detect" events
     */
    public boolean getEdgeDetected() {
        return edgeDetected;
    }
    
    /**
     * Get stable presence (for confirming artifact still there)
     * Use this for "confirm still present" checks after delay
     */
    public boolean getStablePresence() {
        return stablePresence;
    }
    
    /**
     * Get debounced artifact hint (uses stable presence)
     * True if any sensor indicates presence (after full debounce)
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
        sb.append("  edgeDetected: ").append(edgeDetected).append(" (fast, 30ms debounce)\n");
        sb.append("  stablePresence: ").append(stablePresence).append(" (slow, 100ms debounce)\n");
        sb.append("  presenceConfidence: ").append(getPresenceConfidence()).append("\n");
        sb.append("  bestColor: ").append(lastColorClass)
          .append(" (conf=").append(String.format("%.2f", lastColorConfidence)).append(")");
        
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
