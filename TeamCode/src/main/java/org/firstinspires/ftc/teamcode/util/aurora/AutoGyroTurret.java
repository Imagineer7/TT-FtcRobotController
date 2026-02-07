package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * AutoGyroTurret - Field-relative turret control with direct servo control
 *
 * This class maintains a field-relative heading for the turret, automatically
 * compensating for robot rotation to keep the turret pointing in the same
 * direction relative to the field.
 *
 * Direct Servo Position Mapping (FTC Legal - No Servo Tuner):
 * Based on measured calibration points:
 * - Servo position 0.05 = 0° turret (FORWARD-FACING)
 * - Servo position 0.37 = 180° turret (backward)
 * - Servo position 0.71 = 360° turret (full rotation)
 *
 * Linear Mapping Formula:
 * - Servo range: 0.05 to 0.71 (0.66 span)
 * - Turret range: 0° to 360° (full rotation)
 * - Turret angle = (servo position - 0.05) / 0.66 × 360°
 * - Servo position = (turret angle / 360°) × 0.66 + 0.05
 *
 * No intermediate servo angle calculations - direct position-to-angle mapping!
 *
 * Key Features:
 * - Field-relative targeting (turret maintains heading regardless of robot orientation)
 * - Shortest path calculation with automatic wraparound
 * - Direct servo control without intermediate classes
 * - Full 360° rotation capability
 * - Simple linear mapping from calibration measurements
 *
 * Usage:
 * 1. Initialize: autoGyro = new AutoGyroTurret(hardwareMap, telemetry)
 * 2. Set target: autoGyro.setFieldRelativeHeading(90.0, robotHeading) // Point east
 * 3. Update loop: autoGyro.update(robotHeading)
 *
 * The turret will automatically adjust as the robot rotates to maintain
 * the field-relative heading.
 */
public class AutoGyroTurret {

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTANTS
    // ═══════════════════════════════════════════════════════════════════════

    /** Turret servo device name in hardware map */
    private static final String TURRET_SERVO_NAME = "TurretLeft";

    /**
     * Direct Servo Position to Turret Angle Mapping (Measured Calibration)
     *
     * Calibration measurements:
     * - Servo 0.05 = 0° turret
     * - Servo 0.37 = 180° turret (forward-facing)
     * - Servo 0.71 = 360° turret
     *
     * Linear mapping:
     * - Servo range: 0.05 to 0.71 (0.66 span)
     * - Turret range: 0° to 360° (360° span)
     * - Formula: turretAngle = (servoPos - 0.05) / 0.66 × 360°
     * - Inverse: servoPos = (turretAngle / 360°) × 0.66 + 0.05
     */

    /** Servo position at 0° turret (measured) */
    private static final double SERVO_POS_ZERO_DEGREES = 0.05;

    /** Servo position at 360° turret (measured) */
    private static final double SERVO_POS_360_DEGREES = 0.71;

    /** Servo position at 180° turret / forward-facing (measured) */
    private static final double SERVO_POS_180_DEGREES = 0.37;

    /** Servo position range for full turret rotation */
    private static final double SERVO_POSITION_RANGE = SERVO_POS_360_DEGREES - SERVO_POS_ZERO_DEGREES; // 0.66

    /** Maximum turret rotation available */
    private static final double MAX_TURRET_ROTATION = 360.0; // Full rotation available

    /**
     * Forward offset in turret degrees
     * **IMPORTANT**: The physical turret is mounted 180° backwards!
     * - Logical 0° would be servo 0.05, but that points BACKWARD on our robot
     * - Logical 180° = servo 0.37, which points FORWARD on our robot
     * So we add 180° offset to all logical angles to compensate for physical mounting
     */
    private static final double FORWARD_OFFSET_TURRET_DEGREES = 180.0;

    /**
     * Logical turret rotation range (we still work in 0-360° logical space)
     * Physical range (~601°) is mapped to logical 360° for easier field-relative math
     */
    private static final double LOGICAL_TURRET_RANGE = 360.0;

    /**
     * Minimum logical turret rotation
     */
    private static final double MIN_TURRET_ROTATION = 0.0;

    /** Default tolerance for "at target" detection */
    private static final double DEFAULT_TOLERANCE = 2.0; // degrees

    /** Default settling time after wraparound movement (milliseconds) */
    private static final long DEFAULT_SETTLING_TIME_MS = 1500;

    /**
     * Wraparound deadband - Minimum wraparound movement required to trigger wraparound
     * This prevents tiny movements from triggering wraparound near 0°/360° boundary.
     * Example: With 50° deadband:
     *   - 359° -> 1° (wrap would be 2°): 2° < 50° → NO wraparound, use direct
     *   - 340° -> 20° (wrap would be 40°): 40° < 50° → NO wraparound, use direct
     *   - 330° -> 30° (wrap would be 60°): 60° > 50° → YES wraparound allowed
     * Only allows wraparound if the wraparound movement itself exceeds this threshold.
     * Higher values = less sensitive, more stable near boundary.
     */
    private static final double DEFAULT_WRAPAROUND_DEADBAND = 50.0; // degrees

    /**
     * Hysteresis threshold for path switching
     * Once a path (direct or wraparound) is chosen, it takes this much MORE savings
     * to switch to the other path. This prevents oscillation when paths are nearly equal.
     * Example: With 10° hysteresis, if using direct path, wraparound needs to save 10° more to switch.
     */
    private static final double DEFAULT_PATH_HYSTERESIS = 20.0; // degrees

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE REFERENCES
    // ═══════════════════════════════════════════════════════════════════════

    private final Servo turretServo;
    private final Telemetry telemetry;
    private boolean initialized = false;

    /** Current turret angle (0-360°) tracked by this class */
    private double currentTurretAngle = 0.0;

    // ═══════════════════════════════════════════════════════════════════════
    // STATE TRACKING
    // ═══════════════════════════════════════════════════════════════════════

    /** Target heading relative to field (0° = north, 90° = east) */
    private double fieldRelativeTargetHeading = 0.0;

    /** Whether auto-gyro mode is active */
    private boolean enabled = false;

    /** Lock-on mode: continuously updates turret position */
    private boolean lockOnMode = true;

    /** Last known robot heading for change detection (normalized to 0-360) */
    private double lastRobotHeading = 0.0;

    /** Last RAW robot heading before normalization (for boundary crossing detection) */
    private double lastRawRobotHeading = 0.0;

    /** Hysteresis for boundary crossing - degrees past boundary required to accept crossing */
    private static final double BOUNDARY_CROSSING_HYSTERESIS = 15.0; // degrees

    /** Flag indicating we're near the 0°/360° boundary and using hysteresis */
    private boolean usingBoundaryCrossingHysteresis = false;

    /** Initial turret position when field heading was set */
    private double initialTurretPosition = 0.0;

    /** Position tolerance for "at target" detection */
    private double positionTolerance = DEFAULT_TOLERANCE;

    /** Settling time after wraparound movement (milliseconds) */
    private long settlingTimeMs = DEFAULT_SETTLING_TIME_MS;

    /** Wraparound deadband (degrees) - minimum wraparound movement to allow wraparound */
    private double wraparoundDeadband = DEFAULT_WRAPAROUND_DEADBAND;

    /** Path hysteresis (degrees) - stickiness to prevent oscillation */
    private double pathHysteresis = DEFAULT_PATH_HYSTERESIS;

    /** Timestamp when wraparound movement started (0 = not moving) */
    private long wraparoundStartTime = 0;

    /** Flag to prevent repeated wraparound timing triggers */
    private boolean isCurrentlyWrapping = false;

    /** Last path choice: true = wraparound, false = direct */
    private boolean lastPathWasWraparound = false;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new AutoGyroTurret controller with direct servo control
     * @param hardwareMap The hardware map to get the turret servo from
     * @param telemetry The OpMode's telemetry for logging
     */
    public AutoGyroTurret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize servo
        Servo servo = null;
        try {
            servo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);

            // Read initial position and convert to logical angle
            double servoPosition = servo.getPosition();
            currentTurretAngle = servoPositionToLogicalAngle(servoPosition);

            initialized = true;
            log(String.format("AutoGyroTurret initialized - Direct mapping: servo 0.05-0.71 = turret 0-360°, forward at 0.05 (0°)"));
        } catch (Exception e) {
            logWarning("Failed to initialize turret servo: " + e.getMessage());
            initialized = false;
        }

        turretServo = servo;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MAIN UPDATE LOOP
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update the turret to maintain field-relative heading
     * Call this every loop from your OpMode
     *
     * @param currentRobotHeading Current robot heading in degrees (0° = north, positive = clockwise)
     */
    public void update(double currentRobotHeading) {
        if (!enabled) {
            return;
        }

        if (!initialized || turretServo == null) {
            logWarning("Turret servo not initialized - cannot update");
            return;
        }

        // Store original for debugging
        double originalRobotHeading = currentRobotHeading;

        // CRITICAL FIX: Normalize robot heading with boundary crossing hysteresis
        // This handles IMU output in -180 to +180 range and prevents oscillation at boundary
        currentRobotHeading = normalizeRobotHeadingWithHysteresis(currentRobotHeading);

        // Debug: Log if we're using hysteresis
        if (usingBoundaryCrossingHysteresis) {
            log(String.format("⚠ Hysteresis active: raw %.1f°, normalized held at %.1f°",
                originalRobotHeading, currentRobotHeading));
        }

        // Calculate required robot-relative turret angle
        double requiredTurretAngle = calculateRobotRelativeAngle(
            fieldRelativeTargetHeading,
            currentRobotHeading
        );

        // Apply the shortest path calculation
        double targetAngle = calculateShortestPath(
            currentTurretAngle,
            requiredTurretAngle
        );

        // Set turret to target angle
        setTurretAngle(targetAngle);

        // Store for next update
        lastRobotHeading = currentRobotHeading;
    }

    /**
     * Maintain robot-relative forward position when auto-gyro is disabled
     * This keeps the turret pointing forward relative to the robot
     * Call this every loop when auto-gyro is disabled
     *
     * @param currentRobotHeading Current robot heading in degrees (0° = north, positive = clockwise)
     */
    public void maintainRobotRelativeForward(double currentRobotHeading) {
        if (!initialized || turretServo == null) {
            return;
        }

        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);

        // Keep turret at 0° robot-relative (forward)
        // Field-relative target = robot heading (turret points same direction as robot)
        fieldRelativeTargetHeading = currentRobotHeading;

        // Calculate robot-relative angle (should be 0° for forward)
        double requiredTurretAngle = calculateRobotRelativeAngle(
            fieldRelativeTargetHeading,
            currentRobotHeading
        );

        // Apply shortest path to get to forward position
        double targetAngle = calculateShortestPath(
            currentTurretAngle,
            requiredTurretAngle
        );

        // Set turret to target angle
        setTurretAngle(targetAngle);

        // Store for next update
        lastRobotHeading = currentRobotHeading;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SERVO CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Convert logical turret angle (0-360°) to servo position (0.0-1.0)
     *
     * Logical angle mapping (with forward offset = 180°):
     * - Logical 0° = Forward-facing (compensating for physical mounting) = 180° physical turret = 0.37 servo
     * - Logical 90° = Right = 270° physical turret = 0.545 servo
     * - Logical 180° = Backward = 0° physical turret = 0.05 servo
     * - Logical 270° = Left = 90° physical turret = 0.215 servo
     *
     * Formula:
     * 1. Add forward offset: physicalTurret = (logical + 180°) mod 360°
     * 2. Convert to servo: servoPos = (physicalTurret / 360°) × 0.66 + 0.05
     *
     * @param logicalAngle Logical turret angle in degrees (0-360°, where 0° = forward)
     * @return Servo position (0.0-1.0)
     */
    private double logicalAngleToServoPosition(double logicalAngle) {
        // Normalize to 0-360° range
        logicalAngle = normalizeAngle360(logicalAngle);

        // Apply forward offset: logical 0° = forward = 180° physical turret
        double physicalTurretAngle = normalizeAngle360(logicalAngle + FORWARD_OFFSET_TURRET_DEGREES);

        // Direct linear mapping: physical turret angle → servo position
        double servoPosition = (physicalTurretAngle / MAX_TURRET_ROTATION) * SERVO_POSITION_RANGE + SERVO_POS_ZERO_DEGREES;

        // Clamp to valid servo range
        return Math.max(0.0, Math.min(1.0, servoPosition));
    }

    /**
     * Convert servo position (0.0-1.0) to logical turret angle (0-360°)
     *
     * Reverse mapping:
     * 1. Servo to physical turret: physicalTurret = (servoPos - 0.05) / 0.66 × 360°
     * 2. Remove forward offset: logical = (physicalTurret - 180°) mod 360°
     *
     * @param servoPosition Servo position (0.0-1.0)
     * @return Logical turret angle in degrees (0-360°, where 0° = forward)
     */
    private double servoPositionToLogicalAngle(double servoPosition) {
        // Direct linear mapping: servo position → physical turret angle
        double physicalTurretAngle = (servoPosition - SERVO_POS_ZERO_DEGREES) / SERVO_POSITION_RANGE * MAX_TURRET_ROTATION;

        // Remove forward offset: physical 180° = logical 0° (forward)
        double logicalAngle = normalizeAngle360(physicalTurretAngle - FORWARD_OFFSET_TURRET_DEGREES);

        // Normalize to 0-360° range
        return normalizeAngle360(logicalAngle);
    }

    /**
     * Set turret to specific angle using direct servo control
     * @param angle Target angle in degrees (0-360°)
     */
    private void setTurretAngle(double angle) {
        if (!initialized || turretServo == null) {
            logWarning("Cannot set angle - not initialized");
            return;
        }

        // Normalize to 0-360° range
        angle = normalizeAngle360(angle);

        // Convert logical angle to servo position using direct mapping
        double servoPosition = logicalAngleToServoPosition(angle);

        // Debug logging
        log(String.format("Setting turret: %.1f° logical -> %.3f servo pos", angle, servoPosition));

        // Set servo
        turretServo.setPosition(servoPosition);

        // Update tracking
        currentTurretAngle = angle;
    }

    /**
     * Get current turret angle by reading servo position
     * @return Current turret angle in degrees (0-360°)
     */
    private double getTurretAngle() {
        if (!initialized || turretServo == null) {
            return currentTurretAngle; // Return last known value
        }

        // Read servo position and convert to logical angle
        double servoPosition = turretServo.getPosition();
        currentTurretAngle = servoPositionToLogicalAngle(servoPosition);

        return currentTurretAngle;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // FIELD-RELATIVE CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set the target heading relative to the field
     * The turret will maintain this heading regardless of robot orientation
     *
     * @param fieldHeading Target heading in degrees (0° = north, 90° = east, positive = clockwise)
     * @param currentRobotHeading Current robot heading for immediate positioning
     */
    public void setFieldRelativeHeading(double fieldHeading, double currentRobotHeading) {
        // Normalize both angles to 0-360 range
        fieldRelativeTargetHeading = normalizeAngle360(fieldHeading);
        currentRobotHeading = normalizeAngle360(currentRobotHeading);
        lastRobotHeading = currentRobotHeading;

        // Store initial position for reference
        initialTurretPosition = getTurretAngle();

        // Immediately update turret
        if (enabled) {
            update(currentRobotHeading);
        }

        log(String.format("Field heading set to %.1f° (robot at %.1f°)",
            fieldRelativeTargetHeading, currentRobotHeading));
    }

    /**
     * Set field-relative heading based on current turret position
     * "Locks" the current turret direction as the field-relative target
     *
     * @param currentRobotHeading Current robot heading
     */
    public void lockCurrentHeading(double currentRobotHeading) {
        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);

        double currentTurretAngle = getTurretAngle();
        double fieldHeading = currentTurretAngle + currentRobotHeading;
        setFieldRelativeHeading(fieldHeading, currentRobotHeading);
        log(String.format("Locked current heading: %.1f° field-relative", fieldHeading));
    }

    /**
     * Point turret at a specific field coordinate
     *
     * @param targetX Target X coordinate (field-relative, inches)
     * @param targetY Target Y coordinate (field-relative, inches)
     * @param robotX Current robot X position (inches)
     * @param robotY Current robot Y position (inches)
     * @param currentRobotHeading Current robot heading
     */
    public void pointAtFieldPosition(double targetX, double targetY,
                                     double robotX, double robotY,
                                     double currentRobotHeading) {
        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);

        // Calculate angle to target (field-relative)
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double fieldHeading = Math.toDegrees(Math.atan2(deltaX, deltaY));

        setFieldRelativeHeading(fieldHeading, currentRobotHeading);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // ANGLE CALCULATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Calculate robot-relative angle from field-relative heading
     *
     * @param fieldHeading Target heading relative to field
     * @param robotHeading Current robot heading relative to field
     * @return Required turret angle relative to robot (0-360° range)
     */
    private double calculateRobotRelativeAngle(double fieldHeading, double robotHeading) {
        // Ensure both angles are normalized to 0-360° range
        fieldHeading = normalizeAngle360(fieldHeading);
        robotHeading = normalizeAngle360(robotHeading);

        // Robot-relative angle = field heading - robot heading
        double robotRelative = fieldHeading - robotHeading;

        // Normalize to 0-360° range for 360° tuned mode
        return normalizeAngle360(robotRelative);
    }

    /**
     * Calculate the shortest path from current position to target in 0-360° range
     *
     * With 360° tuned mode:
     * - Turret can be at any position 0-360°
     * - We can wrap around (360° = 0°) for shortest path
     * - Example: To go from 350° to 10°, it's shorter to go +20° (wrap) than -340°
     *
     * Wraparound Deadband:
     * - Only allows wraparound if the wraparound movement itself exceeds deadband
     * - Prevents tiny movements from triggering wraparound near boundary
     * - Example: With 30° deadband:
     *   * 359° -> 1°: direct=2°, wrap=358° → wrap too large, use direct (2°) ✓
     *   * 1° -> 359°: direct=358°, wrap=2° → wrap < 30°, use direct (358°) ✓
     *   * 340° -> 20°: direct=40°, wrap=320° → wrap too large, use direct (40°) ✓
     *   * 20° -> 340°: direct=320°, wrap=40° → wrap > 30°, wraparound allowed ✓
     * - This allows wraparound for significant boundary crossings only
     *
     * Path Hysteresis:
     * - Prevents oscillation when direct and wraparound paths are nearly equal
     * - Once a path is chosen, it "sticks" until the other path is significantly better
     * - Example: With 20° hysteresis, need 20° better to switch paths
     *
     * Algorithm:
     * 1. Normalize target to 0-360° range
     * 2. Calculate direct delta and wraparound delta
     * 3. Check if wraparound movement exceeds deadband threshold
     * 4. If wraparound < deadband: NEVER wraparound (use direct)
     * 5. If wraparound >= deadband: Apply hysteresis to choose path
     * 6. Apply chosen delta, normalizing result to 0-360°
     *
     * @param currentAngle Current turret angle (0-360°)
     * @param targetAngle Desired turret angle (can be any value, will be normalized)
     * @return Optimal target angle in 0-360° range
     */
    private double calculateShortestPath(double currentAngle, double targetAngle) {
        // Normalize both angles to 0-360° range
        currentAngle = normalizeAngle360(currentAngle);
        targetAngle = normalizeAngle360(targetAngle);

        // Calculate direct delta (how far to rotate)
        double directDelta = targetAngle - currentAngle;

        // Calculate wraparound delta (going the other way around the circle)
        double wraparoundDelta;
        if (directDelta > 0) {
            // Target is clockwise from current
            // Wraparound = go counter-clockwise (negative direction)
            wraparoundDelta = directDelta - 360.0;
        } else {
            // Target is counter-clockwise from current
            // Wraparound = go clockwise (positive direction)
            wraparoundDelta = directDelta + 360.0;
        }

        // Get absolute values for comparison
        double absDirectDelta = Math.abs(directDelta);
        double absWraparoundDelta = Math.abs(wraparoundDelta);

        // Choose the path based on deadband and hysteresis
        double chosenDelta;
        boolean willWrapAround = false;

        // KEY LOGIC: Only allow wraparound if the wraparound movement itself exceeds deadband
        // This prevents tiny movements from triggering wraparound near the boundary
        if (absWraparoundDelta < wraparoundDeadband) {
            // Wraparound movement is too small - always use direct path
            chosenDelta = directDelta;
            log(String.format("Direct (wraparound too small): %.1f° -> %.1f° (wrap: %.1f° < %.1f° deadband, using direct: %.1f°)",
                currentAngle, targetAngle, absWraparoundDelta, wraparoundDeadband, absDirectDelta));
        } else {
            // Wraparound movement exceeds deadband - apply hysteresis to choose path
            double effectiveHysteresis = pathHysteresis;

            // Check if we should switch paths based on hysteresis
            if (lastPathWasWraparound) {
                // Currently using wraparound - need significant benefit to switch to direct
                if (absDirectDelta < absWraparoundDelta - effectiveHysteresis) {
                    // Direct is significantly shorter
                    chosenDelta = directDelta;
                    log(String.format("Switch to Direct: %.1f° -> %.1f° (direct: %.1f° << wrap: %.1f°)",
                        currentAngle, targetAngle, absDirectDelta, absWraparoundDelta));
                } else {
                    // Stay on wraparound
                    chosenDelta = wraparoundDelta;
                    willWrapAround = true;
                    log(String.format("Stay Wraparound: %.1f° -> %.1f° (wrap: %.1f° vs direct: %.1f°)",
                        currentAngle, targetAngle, absWraparoundDelta, absDirectDelta));
                }
            } else {
                // Currently using direct path - need significant benefit to switch to wraparound
                if (absWraparoundDelta < absDirectDelta - effectiveHysteresis) {
                    // Wraparound is significantly shorter
                    chosenDelta = wraparoundDelta;
                    willWrapAround = true;
                    log(String.format("Switch to Wraparound: %.1f° -> %.1f° (wrap: %.1f° << direct: %.1f°)",
                        currentAngle, targetAngle, absWraparoundDelta, absDirectDelta));
                } else {
                    // Stay on direct
                    chosenDelta = directDelta;
                    log(String.format("Stay Direct: %.1f° -> %.1f° (direct: %.1f° vs wrap: %.1f°)",
                        currentAngle, targetAngle, absDirectDelta, absWraparoundDelta));
                }
            }
        }


        // Update last path choice for next iteration's hysteresis
        lastPathWasWraparound = willWrapAround;

        // Start wraparound timing if this is a new wraparound movement
        // Only trigger once per wraparound operation
        if (willWrapAround && !isCurrentlyWrapping) {
            wraparoundStartTime = System.currentTimeMillis();
            isCurrentlyWrapping = true;
            log(String.format("Wraparound started - turret busy for %d ms", settlingTimeMs));
        } else if (!willWrapAround && isCurrentlyWrapping) {
            // Movement changed to non-wraparound, reset flag
            isCurrentlyWrapping = false;
        }

        // Apply delta and normalize result
        return normalizeAngle360(currentAngle + chosenDelta);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Normalize angle to -180 to +180 range
     */
    private double normalizeAngle180(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /**
     * Normalize angle to 0 to 360 range
     */
    private double normalizeAngle360(double angle) {
        while (angle < 0) angle += 360;
        while (angle >= 360) angle -= 360;
        return angle;
    }

    /**
     * Smart normalization with boundary crossing hysteresis
     * Prevents oscillation when robot heading bounces around 0°/360° boundary
     *
     * @param rawHeading Raw robot heading (may be in -180 to +180 range)
     * @return Normalized heading (0-360 range) with hysteresis applied
     */
    private double normalizeRobotHeadingWithHysteresis(double rawHeading) {
        // Simple normalization first
        double normalized = normalizeAngle360(rawHeading);

        // If this is the first call, just accept it
        if (lastRawRobotHeading == 0.0 && lastRobotHeading == 0.0) {
            lastRawRobotHeading = rawHeading;
            lastRobotHeading = normalized;
            return normalized;
        }

        // Calculate how much the raw heading changed
        double rawDelta = rawHeading - lastRawRobotHeading;

        // Detect if raw heading crossed ±180° boundary
        boolean rawHeadingCrossedBoundary = Math.abs(rawDelta) > 180;

        // If raw heading crossed boundary (e.g., 179° → -179°)
        if (rawHeadingCrossedBoundary) {
            // Calculate what the normalized heading would be
            double potentialNormalized = normalizeAngle360(rawHeading);

            // Check if this would cause a large jump in normalized heading
            double normalizedDelta = potentialNormalized - lastRobotHeading;

            // Normalize the delta to -180 to +180 range to get shortest angle
            while (normalizedDelta > 180) normalizedDelta -= 360;
            while (normalizedDelta <= -180) normalizedDelta += 360;

            double absNormalizedDelta = Math.abs(normalizedDelta);

            // If the normalized jump is large (> 180°), we're at the boundary
            if (absNormalizedDelta > 180) {
                // Determine which side of boundary we're on
                boolean crossingFromNegativeToPositive = (lastRawRobotHeading < -90 && rawHeading > 90);
                boolean crossingFromPositiveToNegative = (lastRawRobotHeading > 90 && rawHeading < -90);

                if (crossingFromNegativeToPositive || crossingFromPositiveToNegative) {
                    // We're crossing the ±180° boundary
                    // Check if we've moved far enough past the boundary to accept the crossing

                    if (crossingFromNegativeToPositive) {
                        // Raw heading went from negative (e.g., -179°) to positive (e.g., 179°)
                        // Normalized: 181° → 179°
                        // Only accept if raw heading is > (180° - hysteresis) OR < (-180° + hysteresis)
                        if (rawHeading > (180.0 - BOUNDARY_CROSSING_HYSTERESIS)) {
                            // Not far enough past boundary, hold old normalized value
                            usingBoundaryCrossingHysteresis = true;
                            log(String.format("Boundary hysteresis: holding normalized %.1f° (raw: %.1f° not past threshold)",
                                lastRobotHeading, rawHeading));
                            return lastRobotHeading; // Don't accept crossing yet
                        }
                    } else {
                        // Raw heading went from positive to negative
                        // Only accept if raw heading is < -(180° - hysteresis) OR > (180° - hysteresis)
                        if (rawHeading < -(180.0 - BOUNDARY_CROSSING_HYSTERESIS)) {
                            // Not far enough past boundary, hold old normalized value
                            usingBoundaryCrossingHysteresis = true;
                            log(String.format("Boundary hysteresis: holding normalized %.1f° (raw: %.1f° not past threshold)",
                                lastRobotHeading, rawHeading));
                            return lastRobotHeading; // Don't accept crossing yet
                        }
                    }

                    // If we get here, we've moved far enough past boundary to accept the crossing
                    usingBoundaryCrossingHysteresis = false;
                    log(String.format("Boundary crossing accepted: raw %.1f° → %.1f° (normalized: %.1f° → %.1f°)",
                        lastRawRobotHeading, rawHeading, lastRobotHeading, potentialNormalized));
                }
            }
        } else {
            // Normal small movement, no boundary crossing
            usingBoundaryCrossingHysteresis = false;
        }

        // Update tracking variables
        lastRawRobotHeading = rawHeading;
        lastRobotHeading = normalized;

        return normalized;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATE CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Enable auto-gyro mode
     * Turret will maintain field-relative heading
     */
    public void enable() {
        enabled = true;
        log("Auto-gyro enabled");
    }

    /**
     * Disable auto-gyro mode
     * Turret control returns to manual
     */
    public void disable() {
        enabled = false;
        log("Auto-gyro disabled");
    }

    /**
     * Toggle auto-gyro mode
     * @return New enabled state
     */
    public boolean toggle() {
        if (enabled) {
            disable();
        } else {
            enable();
        }
        return enabled;
    }

    /**
     * Enable lock-on mode
     * Continuously updates turret position (default behavior)
     */
    public void enableLockOn() {
        lockOnMode = true;
    }

    /**
     * Disable lock-on mode
     * Turret only moves when explicitly commanded
     */
    public void disableLockOn() {
        lockOnMode = false;
    }

    /**
     * Reset to center position pointing forward
     * Logical 0° = forward-facing (with 180° offset for physical mounting)
     * This maps to 180° physical turret = servo position 0.37
     * @param currentRobotHeading Current robot heading
     */
    public void resetToForward(double currentRobotHeading) {
        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);

        // Set field heading to match robot heading
        // This makes turret robot-relative angle = 0° (logical)
        // Which maps to 180° physical turret (forward-facing due to mounting offset)
        setFieldRelativeHeading(currentRobotHeading, currentRobotHeading);
        log("Reset to forward (logical 0° = 180° physical turret = servo 0.37)");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS QUERIES
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if auto-gyro mode is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Check if lock-on mode is active
     */
    public boolean isLockOnMode() {
        return lockOnMode;
    }

    /**
     * Check if turret is at the target field heading
     * @param currentRobotHeading Current robot heading for calculation
     */
    public boolean isAtTarget(double currentRobotHeading) {
        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);

        double requiredAngle = calculateRobotRelativeAngle(
            fieldRelativeTargetHeading,
            currentRobotHeading
        );
        double currentAngle = getTurretAngle();
        double error = Math.abs(normalizeAngle180(requiredAngle - currentAngle));
        return error < positionTolerance;
    }

    /**
     * Check if turret is busy (moving during wraparound)
     *
     * This method returns true if the turret is currently performing a wraparound
     * movement and the settling time has not elapsed. Other systems can use this
     * to wait for the turret to be ready before executing actions.
     *
     * @return true if turret is busy with wraparound movement, false if ready
     */
    public boolean isBusy() {
        // If no wraparound has been triggered, not busy
        if (wraparoundStartTime == 0) {
            return false;
        }

        // Check if settling time has elapsed
        long elapsed = System.currentTimeMillis() - wraparoundStartTime;
        if (elapsed >= settlingTimeMs) {
            // Settling complete, reset timing
            wraparoundStartTime = 0;
            isCurrentlyWrapping = false;
            return false;
        }

        // Still settling
        return true;
    }

    /**
     * Get remaining busy time in milliseconds
     * @return Milliseconds remaining until turret is ready (0 if not busy)
     */
    public long getRemainingBusyTime() {
        if (!isBusy()) {
            return 0;
        }
        long elapsed = System.currentTimeMillis() - wraparoundStartTime;
        return Math.max(0, settlingTimeMs - elapsed);
    }

    /**
     * Force clear busy state
     * Use with caution - this overrides the settling timer
     */
    public void clearBusyState() {
        wraparoundStartTime = 0;
        isCurrentlyWrapping = false;
        log("Busy state cleared manually");
    }

    /**
     * Get current field-relative target heading
     */
    public double getFieldRelativeHeading() {
        return fieldRelativeTargetHeading;
    }

    /**
     * Get current field-relative actual heading
     * @param currentRobotHeading Current robot heading
     */
    public double getCurrentFieldHeading(double currentRobotHeading) {
        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);
        return normalizeAngle360(getTurretAngle() + currentRobotHeading);
    }

    /**
     * Get heading error (field-relative)
     * @param currentRobotHeading Current robot heading
     * @return Error in degrees (positive = target is clockwise)
     */
    public double getHeadingError(double currentRobotHeading) {
        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);
        double currentFieldHeading = getCurrentFieldHeading(currentRobotHeading);
        return normalizeAngle180(fieldRelativeTargetHeading - currentFieldHeading);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set position tolerance for "at target" detection
     * @param tolerance Tolerance in degrees
     */
    public void setPositionTolerance(double tolerance) {
        this.positionTolerance = tolerance;
    }

    /**
     * Get position tolerance
     */
    public double getPositionTolerance() {
        return positionTolerance;
    }

    /**
     * Set settling time after wraparound movement
     * This determines how long isBusy() returns true after a wraparound
     *
     * @param milliseconds Settling time in milliseconds (default: 1000ms)
     */
    public void setSettlingTime(long milliseconds) {
        this.settlingTimeMs = milliseconds;
        log(String.format("Settling time set to %d ms", milliseconds));
    }

    /**
     * Get current settling time configuration
     * @return Settling time in milliseconds
     */
    public long getSettlingTime() {
        return settlingTimeMs;
    }

    /**
     * Set wraparound deadband - minimum wraparound movement required to allow wraparound
     * This prevents tiny movements from triggering wraparound near the 0°/360° boundary.
     * Example: With 30° deadband, wraparound only happens if the wraparound movement itself is > 30°.
     * This allows wraparound for significant boundary crossings (e.g., 40° wraparound) while blocking
     * tiny movements (e.g., 2° wraparound).
     *
     * @param degrees Deadband threshold in degrees (default: 30.0°)
     */
    public void setWraparoundDeadband(double degrees) {
        this.wraparoundDeadband = degrees;
        log(String.format("Wraparound deadband set to %.1f°", degrees));
    }

    /**
     * Get current wraparound deadband configuration
     * @return Deadband threshold in degrees
     */
    public double getWraparoundDeadband() {
        return wraparoundDeadband;
    }

    /**
     * Set path hysteresis - prevents oscillation between direct and wraparound paths
     * Once a path is chosen, the other path must be better by this amount to switch.
     * This solves jittering when paths are nearly equal.
     *
     * @param degrees Hysteresis amount in degrees (default: 10.0°)
     */
    public void setPathHysteresis(double degrees) {
        this.pathHysteresis = degrees;
        log(String.format("Path hysteresis set to %.1f°", degrees));
    }

    /**
     * Get current path hysteresis configuration
     * @return Hysteresis amount in degrees
     */
    public double getPathHysteresis() {
        return pathHysteresis;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS DISPLAY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get status summary string
     * @param currentRobotHeading Current robot heading for calculations
     */
    public String getStatusSummary(double currentRobotHeading) {
        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);
        String busyStatus = isBusy() ? String.format(" | BUSY (%dms)", getRemainingBusyTime()) : "";
        return String.format("AutoGyro: %s | Target: %.1f° field | Current: %.1f° field | Error: %.1f° | Turret: %.1f°%s",
            enabled ? "ON" : "OFF",
            fieldRelativeTargetHeading,
            getCurrentFieldHeading(currentRobotHeading),
            getHeadingError(currentRobotHeading),
            getTurretAngle(),
            busyStatus
        );
    }

    /**
     * Add detailed telemetry data
     * @param currentRobotHeading Current robot heading for calculations
     */
    public void addTelemetry(double currentRobotHeading) {
        // Normalize robot heading to 0-360 range
        currentRobotHeading = normalizeAngle360(currentRobotHeading);

        telemetry.addData("AutoGyro Mode", enabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Field Target", "%.1f°", fieldRelativeTargetHeading);
        telemetry.addData("Field Current", "%.1f°", getCurrentFieldHeading(currentRobotHeading));
        telemetry.addData("Field Error", "%.1f°", getHeadingError(currentRobotHeading));
        telemetry.addData("Robot Heading", "%.1f°", currentRobotHeading);
        telemetry.addData("Turret Angle", "%.1f°", getTurretAngle());
        telemetry.addData("At Target", isAtTarget(currentRobotHeading) ? "YES" : "NO");

        if (isBusy()) {
            telemetry.addData("Status", "⚠ BUSY (settling: %dms remaining)", getRemainingBusyTime());
        } else {
            telemetry.addData("Status", "✓ READY");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // LOGGING
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Log info message
     */
    private void log(String message) {
        if (telemetry != null) {
            telemetry.addData("AutoGyroTurret", message);
        }
    }

    /**
     * Log warning message
     */
    private void logWarning(String message) {
        if (telemetry != null) {
            telemetry.addData("⚠ AutoGyroTurret", message);
        }
    }
}
