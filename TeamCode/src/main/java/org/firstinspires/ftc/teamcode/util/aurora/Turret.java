package org.firstinspires.ftc.teamcode.util.aurora;

// This class will be for controlling the turret on the robot. The turret is a rotating mechanism with a hood(non adjustable) for aiming at the goals no matter the orientation of the robot.
// It will provide methods for rotating the turret to specific angles, aiming at targets, and possibly controlling the hood angle if needed.
// No dependencies on other AURORA classes, except for AuroraHardwareConfig if necessary.
// No advanced features, just basic turret control functionality.

//The turret uses a gobilda servo that can be continuous rotation or a position servo depending on the need. Not sure if the servo can track position while still being able to continuously rotate.
// If it is a continuous rotation servo, we will need to implement a way to track the position using encoders or other sensors.
// If it is a position servo, we can directly set the position to aim at targets. However it may have limited range of motion.

//In position mode the servo has 300 degrees of rotation, however the gear on the servo is much smaller than the turret gear, so the the servo has to rotate multiple times to get the turret to rotate fully once.
//We might get around this by using a continuous rotation servo and tracking position with encoders.
//Include methods for manual control of the turret as well as automatic aiming at targets.
//Include tracking position using timed movements with continuous rotation servos or direct position control with position servos.

//If we get a 5-turn position servo we will be able to get 1800 degrees of rotation. Depending on our servo to turret gear ratio we may be able to get full 360 degree rotation of the turret with a position servo.
/*Our current servo specs are:
Weight 	58g
Gear Ratio 	67:1
Output Shaft Style 	25 Tooth Spline
Voltage Range 	4.8V ~ 7.4V
No-Load Speed (4.8V) 	0.055 sec/60° (180RPM)
No-Load Speed (6.0V) 	0.043 sec/60° (230RPM)
No-Load Speed (7.4V) 	0.035 sec/60° (290RPM)
Stall Torque (4.8V) 	4.0 kg.cm (55 oz-in)
Stall Torque (6.0V) 	4.7 kg.cm (65 oz-in)
Stall Torque (7.4V) 	5.4 kg.cm (75 oz-in)
No-Load Current (4.8V) 	190mA
No-Load Current (6.0V) 	200mA
No-Load Current (7.4V) 	230mA
Stall Current (4.8V) 	2000mA
Stall Current (6.0V) 	2500mA
Stall Current (7.4V) 	3000mA
Max PWM Range (Default) 	500 - 2500μsec
Max PWM Range (Continuous) 	1000 - 2000μsec
Travel per µsec (Default) 	0.15°/μsec
Max Travel (Default) 	300°
Pulse Amplitude 	3-5V
Direction 	Clockwise w/ Increasing PWM Signal
Deadband Width 	4μsec
Motor Type 	Brushed DC
Feedback Style 	5KΩ Potentiometer
Output Shaft Support 	Dual Ball Bearing
Gear Material 	Steel
Wire Length 	300mm
Wire Gauge 	22 AWG
Connector Type 	3-Pos TJC8 Servo Connector [MH-FC]
 */

/* The 5 turn servo specs are:
Weight 	60g (2.12oz)
Gear Ratio 	135:1
Output Shaft Style 	H25T (25 Tooth) Spline
Voltage Range 	4.8V - 7.4V
No-Load Speed (4.8V) 	0.11 sec/60° (90RPM)
No-Load Speed (6.0V) 	0.09 sec/60° (115RPM)
No-Load Speed (7.4V) 	0.07 sec/60° (145RPM)
Stall Torque (4.8V) 	110 oz-in (7.9 kg.cm)
Stall Torque (6.0V) 	130 oz-in (9.3 kg.cm)
Stall Torque (7.4V) 	150 oz-in (10.8 kg.cm)
No-Load Current (4.8V) 	190mA
No-Load Current (6.0V) 	200mA
No-Load Current (7.4V) 	230mA
Stall Current (4.8V) 	2,000mA
Stall Current (6.0V) 	2,500mA
Stall Current (7.4V) 	3,000mA
Max PWM Range 	500-2500μsec
Max PWM Range (Continuous) 	900-2100µsec
Travel per μsec 	0.90°/μsec
Max Rotation (Default Mode) 	5 Turns (1800°)
Pulse Amplitude 	3-5V
Deadband Width 	4μsec
Motor Type 	Brushed DC
Feedback Style 	5KΩ Potentiometer
Output Shaft Support 	Dual Ball Bearing
Gear Material 	Steel
Wire Length 	11.81" (300mm)
Wire Gauge 	22AWG
Connector Type 	3-Pos TJC8 Servo Connector [MH-FC]
Servo Size 	Standard
Direction with Increasing PWM Signal 	Clockwise
 */

//Turret gear has 108 teeth
//Servo gear has 37 teeth
//Gear ratio is 108/37 = 2.92
//So for every full rotation of the turret, the servo needs to rotate 2.92 times.
//For 360 degrees of turret rotation, servo needs to rotate 360 * 2.92 = 1051.4 degrees.
//With a 300 degree max rotation servo, we would need multiple rotations of the servo to achieve full turret rotation.
//With a 5 turn (1800 degree) servo, we can achieve full turret rotation with some margin.
//We will need to implement a way to track the servo position to know the turret angle.

//In position mode on a 300 degree servo, we can only achieve about 102.7 degrees of turret rotation before hitting the servo limits.
//In position mode on a 5 turn servo, we can achieve full 360 degree turret rotation with room to spare.

//So implement both options in the code, with a way to switch between continuous rotation mode and position mode depending on the servo type used.
//Use a variable at the top of the class to set the mode. CONTINUOUS_ROTATION_MODE, POSITION_MODE, 5_TURN_MODE
//Implement methods for rotating to specific angles, manual control, and tracking position based on the mode
//In continuous rotation mode, use timed movements to estimate position by tracking time and speed. Have variable for tuning timed movements.

//Robot configuration has the servo as "turret_servo".
//Depending on the mode set in the code, initialize the servo accordingly.

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret - Controls the rotating turret mechanism for aiming
 *
 * This class provides control for a servo-driven turret system with support for
 * different servo types and operating modes.
 *
 * Hardware Specifications:
 * - Turret gear: 108 teeth
 * - Servo gear: 37 teeth
 * - Gear ratio: 2.92 (servo must rotate 2.92x for full turret rotation)
 * - For 360° turret rotation: servo needs 1051.4° rotation
 *
 * Supported Servo Modes:
 * - POSITION_MODE_300: Standard 300° servo (limited to ~103° turret rotation)
 * - POSITION_MODE_5TURN: 5-turn 1800° servo (full 360° turret rotation)
 * - CONTINUOUS_ROTATION: CR servo with time-based position estimation
 */
public class Turret {

    // ═══════════════════════════════════════════════════════════════════════
    // SERVO MODE CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Turret servo operating mode
     */
    public enum TurretMode {
        POSITION_MODE_300,      // Standard 300° position servo (limited rotation)
        POSITION_MODE_5TURN,    // 5-turn 1800° position servo (full rotation)
        CONTINUOUS_ROTATION     // Continuous rotation with time-based tracking
    }

    /** CONFIGURE THIS: Set the mode for your servo type */
    private static final TurretMode SERVO_MODE = TurretMode.POSITION_MODE_300;

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE CONSTANTS
    // ═══════════════════════════════════════════════════════════════════════

    /** Servo device name in robot configuration */
    public static final String TURRET_SERVO_NAME = "turret_servo";

    /** Gear ratio: turret teeth / servo teeth */
    private static final double GEAR_RATIO = 108.0 / 37.0; // = 2.92

    /** Servo degrees needed for full turret rotation */
    private static final double SERVO_DEGREES_PER_TURRET_ROTATION = 360.0 * GEAR_RATIO; // = 1051.4°

    /** Maximum servo travel in each mode */
    private static final double SERVO_MAX_POSITION_300 = 300.0;
    private static final double SERVO_MAX_POSITION_5TURN = 1800.0;

    /** Maximum turret rotation in each mode */
    private static final double MAX_TURRET_ANGLE_300 = SERVO_MAX_POSITION_300 / GEAR_RATIO; // ~103°
    private static final double MAX_TURRET_ANGLE_5TURN = SERVO_MAX_POSITION_5TURN / GEAR_RATIO; // ~617°

    // ═══════════════════════════════════════════════════════════════════════
    // TUNING PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    /** Continuous rotation speed (degrees per second) - CALIBRATED */
    // Measured: 360° in 0.34 seconds = 1059 °/s
    // Note: Hardware limitation - commands shorter than 0.3s are not accurate
    private double continuousRotationSpeed = 1059.0; // Calibrated turret speed

    /** Movement deadband for "at target" detection */
    private double positionTolerance = 2.0; // degrees

    /** Default turret rotation speed for manual control (0.0 to 1.0) */
    private double defaultRotationPower = 0.8;

    /** Center position offset calibration (degrees) */
    private double centerOffsetCalibration = 0.0;

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE INSTANCES
    // ═══════════════════════════════════════════════════════════════════════

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private Servo positionServo;        // For position modes
    private CRServo continuousServo;    // For continuous rotation mode

    // ═══════════════════════════════════════════════════════════════════════
    // STATE TRACKING
    // ═══════════════════════════════════════════════════════════════════════

    private boolean initialized = false;
    private double currentTurretAngle = 0.0;    // Current turret angle in degrees
    private double targetTurretAngle = 0.0;     // Target turret angle in degrees
    private long lastUpdateTime = 0;            // For continuous rotation tracking
    private boolean enabled = true;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new Turret controller
     * @param hardwareMap The OpMode's hardwareMap
     * @param telemetry The OpMode's telemetry
     */
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Initialize the turret hardware
     * @return true if initialization successful
     */
    public boolean initialize() {
        try {
            if (SERVO_MODE == TurretMode.CONTINUOUS_ROTATION) {
                continuousServo = hardwareMap.get(CRServo.class, TURRET_SERVO_NAME);
                continuousServo.setPower(0.0);
                log("Turret initialized in CONTINUOUS ROTATION mode");
            } else {
                positionServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
                // Start at center position
                setToCenter();
                log("Turret initialized in " + SERVO_MODE + " mode");
            }

            lastUpdateTime = System.currentTimeMillis();
            initialized = true;
            return true;

        } catch (Exception e) {
            log("ERROR: Failed to initialize turret: " + e.getMessage());
            initialized = false;
            return false;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UPDATE LOOP
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Update turret state (call periodically from OpMode loop)
     * Required for continuous rotation mode position tracking
     */
    public void update() {
        if (!initialized || !enabled) {
            return;
        }

        if (SERVO_MODE == TurretMode.CONTINUOUS_ROTATION) {
            updateContinuousRotationTracking();
        } else {
            updatePositionServoTracking();
        }
    }

    /**
     * Update position tracking for continuous rotation mode
     */
    private void updateContinuousRotationTracking() {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // seconds
        lastUpdateTime = currentTime;

        // Estimate position change based on time and speed
        double currentPower = continuousServo.getPower();
        double estimatedMovement = currentPower * continuousRotationSpeed * deltaTime;
        currentTurretAngle += estimatedMovement;

        // Wrap angle to 0-360 range
        currentTurretAngle = normalizeAngle(currentTurretAngle);

        // Auto-stop if at target
        if (Math.abs(currentTurretAngle - targetTurretAngle) < positionTolerance) {
            continuousServo.setPower(0.0);
        }
    }

    /**
     * Update position tracking for position servo modes
     */
    private void updatePositionServoTracking() {
        // Position servos handle movement automatically
        // Just sync our tracking with actual servo position
        double servoPosition = positionServo.getPosition();
        double servoDegrees = servoPosition * getMaxServoTravel();
        currentTurretAngle = servoDegreesToTurretDegrees(servoDegrees);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // POSITION CONTROL METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set turret to specific angle
     * @param angle Target angle in degrees (0° = forward, positive = clockwise)
     * @return true if command accepted
     */
    public boolean setAngle(double angle) {
        if (!initialized || !enabled) {
            return false;
        }

        // Clamp angle to valid range
        double clampedAngle = clampAngle(angle);
        targetTurretAngle = clampedAngle;

        if (SERVO_MODE == TurretMode.CONTINUOUS_ROTATION) {
            setAngleContinuous(clampedAngle);
        } else {
            setAnglePosition(clampedAngle);
        }

        return true;
    }

    /**
     * Set angle using position servo
     */
    private void setAnglePosition(double turretAngle) {
        double servoDegrees = turretDegreesToServoDegrees(turretAngle);
        double servoPosition = servoDegrees / getMaxServoTravel();
        positionServo.setPosition(servoPosition);
        currentTurretAngle = turretAngle;
    }

    /**
     * Set angle using continuous rotation servo
     */
    private void setAngleContinuous(double targetAngle) {
        double angleDelta = targetAngle - currentTurretAngle;

        // Determine shortest rotation direction
        if (angleDelta > 180) {
            angleDelta -= 360;
        } else if (angleDelta < -180) {
            angleDelta += 360;
        }

        // Set power based on direction and distance
        double power = Math.signum(angleDelta) * defaultRotationPower;

        // Reduce power when close to target
        double distance = Math.abs(angleDelta);
        if (distance < 10) {
            power *= 0.3; // Slow down near target
        }

        continuousServo.setPower(power);
    }

    /**
     * Rotate turret by relative angle
     * @param deltaAngle Angle to rotate (positive = clockwise)
     */
    public void rotateBy(double deltaAngle) {
        setAngle(currentTurretAngle + deltaAngle);
    }

    /**
     * Set turret to center (0° forward)
     */
    public void setToCenter() {
        setAngle(centerOffsetCalibration);
    }

    /**
     * Set turret to face forward relative to robot
     */
    public void setToForward() {
        setAngle(0.0);
    }

    /**
     * Set turret to face backward
     */
    public void setToBackward() {
        setAngle(180.0);
    }

    /**
     * Set turret to face left
     */
    public void setToLeft() {
        setAngle(90.0);
    }

    /**
     * Set turret to face right
     */
    public void setToRight() {
        setAngle(-90.0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MANUAL CONTROL METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Manually rotate turret with specified power
     * @param power Rotation power (-1.0 to 1.0, positive = clockwise)
     */
    public void rotate(double power) {
        if (!initialized || !enabled) {
            return;
        }

        if (SERVO_MODE == TurretMode.CONTINUOUS_ROTATION) {
            continuousServo.setPower(power);
        } else {
            // For position servos, translate power to position change
            double movement = power * 5.0; // degrees per update
            setAngle(currentTurretAngle + movement);
        }
    }

    /**
     * Stop turret movement
     */
    public void stop() {
        if (SERVO_MODE == TurretMode.CONTINUOUS_ROTATION && continuousServo != null) {
            continuousServo.setPower(0.0);
        }
        targetTurretAngle = currentTurretAngle;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // AIMING METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Aim turret at a target position
     * @param targetX Target X coordinate (field-relative)
     * @param targetY Target Y coordinate (field-relative)
     * @param robotX Current robot X position
     * @param robotY Current robot Y position
     * @param robotHeading Current robot heading in degrees
     */
    public void aimAt(double targetX, double targetY, double robotX, double robotY, double robotHeading) {
        // Calculate angle to target (field-relative)
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double angleToTarget = Math.toDegrees(Math.atan2(deltaX, deltaY));

        // Convert to robot-relative angle
        double robotRelativeAngle = angleToTarget - robotHeading;

        // Set turret to aim at target
        setAngle(robotRelativeAngle);
    }

    /**
     * Aim at target with offset adjustment
     * @param targetAngle Target angle in field coordinates
     * @param robotHeading Current robot heading
     * @param offsetDegrees Additional offset for trajectory compensation
     */
    public void aimAtWithOffset(double targetAngle, double robotHeading, double offsetDegrees) {
        double robotRelativeAngle = targetAngle - robotHeading + offsetDegrees;
        setAngle(robotRelativeAngle);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATE QUERY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if turret is at target angle
     * @return true if at target within tolerance
     */
    public boolean isAtTarget() {
        return Math.abs(currentTurretAngle - targetTurretAngle) < positionTolerance;
    }

    /**
     * Check if turret is initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Check if turret is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Get current turret angle
     * @return Current angle in degrees
     */
    public double getCurrentAngle() {
        return currentTurretAngle;
    }

    /**
     * Get target turret angle
     * @return Target angle in degrees
     */
    public double getTargetAngle() {
        return targetTurretAngle;
    }

    /**
     * Get angle error (difference between current and target)
     * @return Angle error in degrees
     */
    public double getAngleError() {
        return targetTurretAngle - currentTurretAngle;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CONFIGURATION METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Enable turret operation
     */
    public void enable() {
        enabled = true;
    }

    /**
     * Disable turret operation
     */
    public void disable() {
        stop();
        enabled = false;
    }

    /**
     * Set rotation speed for continuous mode
     * @param degreesPerSecond Speed in degrees per second
     */
    public void setContinuousRotationSpeed(double degreesPerSecond) {
        this.continuousRotationSpeed = degreesPerSecond;
    }

    /**
     * Set position tolerance
     * @param tolerance Tolerance in degrees
     */
    public void setPositionTolerance(double tolerance) {
        this.positionTolerance = tolerance;
    }

    /**
     * Set default rotation power for manual control
     * @param power Power value (0.0 to 1.0)
     */
    public void setDefaultRotationPower(double power) {
        this.defaultRotationPower = Math.min(1.0, Math.max(0.0, power));
    }

    /**
     * Calibrate center position
     * @param offsetDegrees Offset from physical center
     */
    public void calibrateCenter(double offsetDegrees) {
        this.centerOffsetCalibration = offsetDegrees;
    }

    /**
     * Reset turret angle tracking to zero
     */
    public void resetAngleTracking() {
        currentTurretAngle = 0.0;
        targetTurretAngle = 0.0;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Convert turret degrees to servo degrees
     */
    private double turretDegreesToServoDegrees(double turretDegrees) {
        return turretDegrees * GEAR_RATIO;
    }

    /**
     * Convert servo degrees to turret degrees
     */
    private double servoDegreesToTurretDegrees(double servoDegrees) {
        return servoDegrees / GEAR_RATIO;
    }

    /**
     * Get maximum servo travel for current mode
     */
    private double getMaxServoTravel() {
        return (SERVO_MODE == TurretMode.POSITION_MODE_5TURN)
            ? SERVO_MAX_POSITION_5TURN
            : SERVO_MAX_POSITION_300;
    }

    /**
     * Get maximum turret angle for current mode
     */
    private double getMaxTurretAngle() {
        if (SERVO_MODE == TurretMode.CONTINUOUS_ROTATION) {
            return 360.0; // Full rotation
        } else if (SERVO_MODE == TurretMode.POSITION_MODE_5TURN) {
            return MAX_TURRET_ANGLE_5TURN;
        } else {
            return MAX_TURRET_ANGLE_300;
        }
    }

    /**
     * Clamp angle to valid range for current mode
     */
    private double clampAngle(double angle) {
        if (SERVO_MODE == TurretMode.CONTINUOUS_ROTATION) {
            return normalizeAngle(angle); // Allow full 360° rotation
        } else {
            double maxAngle = getMaxTurretAngle() / 2.0;
            return Math.max(-maxAngle, Math.min(maxAngle, angle));
        }
    }

    /**
     * Normalize angle to 0-360 range
     */
    private double normalizeAngle(double angle) {
        while (angle < 0) angle += 360;
        while (angle >= 360) angle -= 360;
        return angle;
    }

    /**
     * Log message to telemetry
     */
    private void log(String message) {
        if (telemetry != null) {
            telemetry.addLine("Turret: " + message);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS INFORMATION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get status summary string
     * @return Status summary
     */
    public String getStatusSummary() {
        return String.format("Turret: %s | Angle: %.1f° → %.1f° | %s",
            enabled ? "ON" : "OFF",
            currentTurretAngle,
            targetTurretAngle,
            isAtTarget() ? "AT TARGET" : "MOVING"
        );
    }

    /**
     * Get current mode
     * @return The active turret mode
     */
    public TurretMode getMode() {
        return SERVO_MODE;
    }
}
