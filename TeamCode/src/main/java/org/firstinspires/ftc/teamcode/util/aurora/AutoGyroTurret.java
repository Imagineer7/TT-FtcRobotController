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
 * Direct Servo Control (360° Tuned Mode):
 * - Servo position 0.0 = 0° turret (forward)
 * - Servo position 0.5 = 180° turret (backward)
 * - Servo position 1.0 = 360° turret (forward, full rotation)
 * - Simple formula: servoPosition = turretAngle / 360.0
 *
 * Key Features:
 * - Field-relative targeting (turret maintains heading regardless of robot orientation)
 * - Shortest path calculation with automatic wraparound
 * - Direct servo control without intermediate classes
 * - Full 360° rotation capability
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

    /** Maximum turret rotation for 360° tuned mode - full rotation available! */
    private static final double MAX_TURRET_ROTATION = 360.0;

    /** Minimum turret rotation (0° is forward, we can go full circle) */
    private static final double MIN_TURRET_ROTATION = 0.0;

    /** Default tolerance for "at target" detection */
    private static final double DEFAULT_TOLERANCE = 2.0; // degrees

    /** Default settling time after wraparound movement (milliseconds) */
    private static final long DEFAULT_SETTLING_TIME_MS = 1500;

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

    /** Last known robot heading for change detection */
    private double lastRobotHeading = 0.0;

    /** Initial turret position when field heading was set */
    private double initialTurretPosition = 0.0;

    /** Position tolerance for "at target" detection */
    private double positionTolerance = DEFAULT_TOLERANCE;

    /** Settling time after wraparound movement (milliseconds) */
    private long settlingTimeMs = DEFAULT_SETTLING_TIME_MS;

    /** Timestamp when wraparound movement started (0 = not moving) */
    private long wraparoundStartTime = 0;

    /** Flag to prevent repeated wraparound timing triggers */
    private boolean isCurrentlyWrapping = false;

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

            // Read initial position and convert to angle
            double servoPosition = servo.getPosition();
            currentTurretAngle = servoPosition * 360.0;

            initialized = true;
            log("AutoGyroTurret initialized (servo direct control)");
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

        // Calculate required robot-relative turret angle
        double requiredTurretAngle = calculateRobotRelativeAngle(
            fieldRelativeTargetHeading,
            currentRobotHeading
        );

        // Apply shortest path calculation
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

        // Direct mapping: servo position = turret angle / 360
        double servoPosition = angle / 360.0;

        // Clamp to valid servo range
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // Debug logging
        log(String.format("Setting turret: %.1f° -> servo pos %.3f", angle, servoPosition));

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

        // Read servo position and convert to angle
        double servoPosition = turretServo.getPosition();
        currentTurretAngle = servoPosition * 360.0;

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
        // Normalize to 0-360 range
        fieldRelativeTargetHeading = normalizeAngle360(fieldHeading);
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
        // Robot-relative angle = field heading - robot heading
        double robotRelative = fieldHeading - robotHeading;

        // Normalize to 0-360° range for 360° tuned mode
        return normalizeAngle360(robotRelative);
    }

    /**
     * Calculate shortest path from current position to target in 0-360° range
     *
     * With 360° tuned mode:
     * - Turret can be at any position 0-360°
     * - We can wrap around (360° = 0°) for shortest path
     * - Example: To go from 350° to 10°, it's shorter to go +20° (wrap) than -340°
     *
     * Algorithm:
     * 1. Normalize target to 0-360° range
     * 2. Calculate direct delta and wraparound delta
     * 3. Choose shortest path
     * 4. Apply to current position, normalizing result to 0-360°
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

        // Choose the shortest path
        double chosenDelta;
        boolean willWrapAround = false;

        if (Math.abs(directDelta) <= Math.abs(wraparoundDelta)) {
            chosenDelta = directDelta;
        } else {
            chosenDelta = wraparoundDelta;
            willWrapAround = true;
            log(String.format("Wraparound: %.1f° -> %.1f° (delta: %.1f°)",
                currentAngle, targetAngle, wraparoundDelta));
        }

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
        double resultAngle = normalizeAngle360(currentAngle + chosenDelta);

        return resultAngle;
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
     * @param currentRobotHeading Current robot heading
     */
    public void resetToForward(double currentRobotHeading) {
        setFieldRelativeHeading(currentRobotHeading, currentRobotHeading);
        log("Reset to forward");
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
        return normalizeAngle360(getTurretAngle() + currentRobotHeading);
    }

    /**
     * Get heading error (field-relative)
     * @param currentRobotHeading Current robot heading
     * @return Error in degrees (positive = target is clockwise)
     */
    public double getHeadingError(double currentRobotHeading) {
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

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS DISPLAY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get status summary string
     * @param currentRobotHeading Current robot heading for calculations
     */
    public String getStatusSummary(double currentRobotHeading) {
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
