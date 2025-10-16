package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.vision.AuroraAprilTagLocalizer;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;

/**
 * Aurora Localization Integration Manager
 *
 * Integrates the new AuroraAprilTagLocalizer with the existing Aurora positioning system.
 * Provides seamless position updates, fallback handling, and system coordination.
 *
 * Features:
 * - Real-time position fusion between vision and encoders
 * - Automatic fallback when vision is lost
 * - Position validation and confidence tracking
 * - Aurora system compatibility
 *
 * @author Aurora Team
 */
public class AuroraLocalizationManager implements AuroraAprilTagLocalizer.AuroraPositionCallback {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final FieldMap fieldMap;

    // Localization components
    private AuroraAprilTagLocalizer aprilTagLocalizer;
    private EncoderManager encoderManager; // Your existing encoder system

    // Position state
    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentHeading = 0.0;
    private boolean hasVisionPosition = false;
    private double visionConfidence = 0.0;
    private long lastVisionUpdate = 0;

    // Configuration
    private static final double MIN_VISION_CONFIDENCE = 0.6;
    private static final long VISION_TIMEOUT_MS = 1000;
    private boolean useVisionCorrection = true;
    private double visionWeight = 0.7; // How much to trust vision vs encoders

    // Aurora integration
    private AuroraPositionUpdateListener auroraListener;

    /**
     * Interface for Aurora system to receive position updates
     */
    public interface AuroraPositionUpdateListener {
        void onPositionUpdated(double x, double y, double heading, PositionSource source);
        void onPositionSourceChanged(PositionSource oldSource, PositionSource newSource);
    }

    /**
     * Position source enumeration
     */
    public enum PositionSource {
        VISION_PRIMARY,    // High-confidence vision
        VISION_ENCODER,    // Vision + encoder fusion
        ENCODER_ONLY,      // Vision lost, using encoders
        UNKNOWN           // No reliable position
    }

    /**
     * Initialize the Aurora Localization Manager
     */
    public AuroraLocalizationManager(HardwareMap hardwareMap, Telemetry telemetry,
                                   FieldMap.Alliance alliance, String webcamName) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.fieldMap = new FieldMap(alliance);

        // Initialize localization systems
        initializeLocalizationSystems(alliance, webcamName);
    }

    /**
     * Initialize all localization subsystems
     */
    private void initializeLocalizationSystems(FieldMap.Alliance alliance, String webcamName) {
        try {
            // Initialize AprilTag localizer with our calibrated settings
            aprilTagLocalizer = new AuroraAprilTagLocalizer(hardwareMap, telemetry, alliance, webcamName);
            aprilTagLocalizer.setPositionCallback(this);

            // Initialize encoder manager (assuming it exists in your Aurora system)
            // encoderManager = new EncoderManager(hardwareMap, telemetry);

            telemetry.addData("Aurora Localization", "Initialized successfully");

        } catch (Exception e) {
            telemetry.addData("Aurora Localization Error", e.getMessage());
        }
    }

    /**
     * Set Aurora system listener
     */
    public void setAuroraListener(AuroraPositionUpdateListener listener) {
        this.auroraListener = listener;
    }

    /**
     * Update localization - call this in your main Aurora loop
     */
    public void updateLocalization() {
        // Update AprilTag localization
        boolean visionUpdated = false;
        if (aprilTagLocalizer != null) {
            visionUpdated = aprilTagLocalizer.updatePosition();
        }

        // Update encoder position (if you have encoder manager)
        // encoderManager.updatePosition();

        // Determine current position source and notify Aurora
        PositionSource currentSource = determinePositionSource();
        updateAuroraPosition(currentSource);
    }

    /**
     * Determine the most reliable position source
     */
    private PositionSource determinePositionSource() {
        if (hasVisionPosition && visionConfidence >= MIN_VISION_CONFIDENCE) {
            if (System.currentTimeMillis() - lastVisionUpdate < 200) {
                return PositionSource.VISION_PRIMARY;
            } else {
                return PositionSource.VISION_ENCODER;
            }
        } else if (System.currentTimeMillis() - lastVisionUpdate < VISION_TIMEOUT_MS) {
            return PositionSource.VISION_ENCODER;
        } else {
            return PositionSource.ENCODER_ONLY;
        }
    }

    /**
     * Update Aurora system with current position
     */
    private void updateAuroraPosition(PositionSource source) {
        if (auroraListener != null) {
            auroraListener.onPositionUpdated(currentX, currentY, currentHeading, source);
        }
    }

    /**
     * AprilTag position callback implementation
     */
    @Override
    public void onPositionUpdate(double x, double y, double heading, double confidence, long timestamp) {
        // Fuse vision with existing position
        if (useVisionCorrection) {
            if (hasVisionPosition) {
                // Smooth position update
                double weight = Math.min(confidence * visionWeight, 0.9);
                currentX = currentX * (1 - weight) + x * weight;
                currentY = currentY * (1 - weight) + y * weight;
                currentHeading = normalizeAngle(currentHeading * (1 - weight) + heading * weight);
            } else {
                // First vision update - accept directly
                currentX = x;
                currentY = y;
                currentHeading = heading;
            }
        }

        hasVisionPosition = true;
        visionConfidence = confidence;
        lastVisionUpdate = timestamp;

        telemetry.addData("Vision Position", "X: %.1f, Y: %.1f, H: %.1f°", currentX, currentY, currentHeading);
        telemetry.addData("Vision Confidence", "%.2f", confidence);
    }

    /**
     * AprilTag position lost callback
     */
    @Override
    public void onPositionLost() {
        hasVisionPosition = false;
        visionConfidence = 0.0;

        telemetry.addData("Vision Status", "Position Lost - Using Encoders");

        if (auroraListener != null) {
            auroraListener.onPositionSourceChanged(PositionSource.VISION_PRIMARY, PositionSource.ENCODER_ONLY);
        }
    }

    /**
     * Get current position for Aurora system
     */
    public double[] getCurrentPosition() {
        return new double[]{currentX, currentY, currentHeading};
    }

    /**
     * Set robot position (for initialization or correction)
     */
    public void setPosition(double x, double y, double heading) {
        currentX = x;
        currentY = y;
        currentHeading = normalizeAngle(heading);

        // Reset encoder position if available
        // if (encoderManager != null) {
        //     encoderManager.setPosition(x, y, heading);
        // }
    }

    /**
     * Get localization status for Aurora dashboard
     */
    public LocalizationStatus getStatus() {
        AuroraAprilTagLocalizer.AuroraLocalizationStatus visionStatus =
            aprilTagLocalizer != null ? aprilTagLocalizer.getStatus() : null;

        return new LocalizationStatus(
            currentX, currentY, currentHeading,
            hasVisionPosition, visionConfidence,
            determinePositionSource(),
            visionStatus
        );
    }

    /**
     * Enable/disable vision correction
     */
    public void setVisionCorrectionEnabled(boolean enabled) {
        this.useVisionCorrection = enabled;
        telemetry.addData("Vision Correction", enabled ? "ENABLED" : "DISABLED");
    }

    /**
     * Set vision weight for position fusion
     */
    public void setVisionWeight(double weight) {
        this.visionWeight = Math.max(0.0, Math.min(1.0, weight));
    }

    /**
     * Get field zone information
     */
    public String getCurrentZone() {
        return fieldMap.getZoneInfo(currentX, currentY);
    }

    /**
     * Pause/resume vision processing
     */
    public void pauseVision() {
        if (aprilTagLocalizer != null) {
            aprilTagLocalizer.pauseStreaming();
        }
    }

    public void resumeVision() {
        if (aprilTagLocalizer != null) {
            aprilTagLocalizer.resumeStreaming();
        }
    }

    /**
     * Cleanup resources
     */
    public void shutdown() {
        if (aprilTagLocalizer != null) {
            aprilTagLocalizer.shutdown();
        }
    }

    /**
     * Normalize angle to [-180, 180] degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Comprehensive localization status
     */
    public static class LocalizationStatus {
        public final double x, y, heading;
        public final boolean hasVision;
        public final double visionConfidence;
        public final PositionSource currentSource;
        public final AuroraAprilTagLocalizer.AuroraLocalizationStatus visionStatus;

        public LocalizationStatus(double x, double y, double heading, boolean hasVision,
                                double visionConfidence, PositionSource currentSource,
                                AuroraAprilTagLocalizer.AuroraLocalizationStatus visionStatus) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.hasVision = hasVision;
            this.visionConfidence = visionConfidence;
            this.currentSource = currentSource;
            this.visionStatus = visionStatus;
        }
    }
}
