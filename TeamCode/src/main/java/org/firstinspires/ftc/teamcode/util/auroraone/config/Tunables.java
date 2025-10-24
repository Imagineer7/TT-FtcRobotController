package org.firstinspires.ftc.teamcode.util.auroraone.config;

/**
 * AURORA ONE - Tunable Configuration Constants
 *
 * This class contains all tunable parameters for the Aurora One robot system.
 * These values can be adjusted for different field conditions, robot configurations,
 * or performance requirements.
 */
public class Tunables {

    // =========================================================================================
    // FIELD DIMENSIONS
    // =========================================================================================

    /**
     * Field width in inches (standard FTC field)
     */
    public static final double FIELD_WIDTH = 144.0; // 12 feet

    /**
     * Field length in inches (standard FTC field)
     */
    public static final double FIELD_LENGTH = 144.0; // 12 feet

    /**
     * Tile size in inches (standard FTC field tiles)
     */
    public static final double TILE_SIZE = 24.0; // 2 feet per tile

    /**
     * Field margin for boundary checking (inches)
     */
    public static final double FIELD_MARGIN = 6.0;

    // =========================================================================================
    // STATE MACHINE TIMEOUTS
    // =========================================================================================

    /**
     * Maximum time allowed for collection sequence before timeout (seconds)
     */
    public static final double COLLECTION_TIMEOUT = 5.0;

    /**
     * Maximum time allowed for shooting sequence before timeout (seconds)
     */
    public static final double SHOOTING_TIMEOUT = 10.0;

    /**
     * Maximum time allowed for positioning operations (seconds)
     */
    public static final double POSITIONING_TIMEOUT = 8.0;

    /**
     * Maximum time allowed for alignment operations (seconds)
     */
    public static final double ALIGNMENT_TIMEOUT = 3.0;

    // =========================================================================================
    // DRIVE SYSTEM TUNING
    // =========================================================================================

    /**
     * Maximum drive power (0.0 to 1.0)
     */
    public static final double MAX_DRIVE_POWER = 0.9;

    /**
     * Maximum turn power (0.0 to 1.0)
     */
    public static final double MAX_TURN_POWER = 0.8;

    /**
     * Drive power scaling for precision mode
     */
    public static final double PRECISION_DRIVE_SCALE = 0.3;

    /**
     * Minimum drive power to overcome static friction
     */
    public static final double MIN_DRIVE_POWER = 0.1;

    /**
     * Maximum drive power for precision mode (0.0 to 1.0)
     */
    public static final double DRIVE_MAX_SPEED_PRECISION = 0.3;

    /**
     * Maximum drive power for normal mode (0.0 to 1.0)
     */
    public static final double DRIVE_MAX_SPEED_NORMAL = 0.7;

    /**
     * Maximum drive power for sport mode (0.0 to 1.0)
     */
    public static final double DRIVE_MAX_SPEED_SPORT = 1.0;

    /**
     * Maximum drive power for efficiency mode (0.0 to 1.0)
     */
    public static final double DRIVE_MAX_SPEED_EFFICIENCY = 0.6;

    /**
     * Maximum drive power for auto-adaptive mode (0.0 to 1.0)
     */
    public static final double DRIVE_MAX_SPEED_AUTO_ADAPTIVE = 0.8;

    /**
     * Nominal battery voltage for power calculations (volts)
     */
    public static final double DRIVE_NOMINAL_VOLTAGE = 12.0;

    /**
     * Low voltage threshold for battery protection (volts)
     */
    public static final double DRIVE_LOW_VOLTAGE_THRESHOLD = 11.0;

    /**
     * Duration before sustained low voltage triggers protection (seconds)
     */
    public static final double DRIVE_LOW_VOLTAGE_DURATION = 3.0;

    /**
     * Power multiplier when battery voltage is low
     */
    public static final double DRIVE_LOW_BATTERY_POWER_MULTIPLIER = 0.7;

    /**
     * Joystick deadzone threshold (0.0 to 1.0)
     */
    public static final double DRIVE_DEADZONE = 0.05;

    /**
     * Rotation sensitivity multiplier
     */
    public static final double DRIVE_ROTATION_SENSITIVITY = 0.8;

    /**
     * Maximum acceleration limit to prevent wheel slip (power units per second)
     */
    public static final double DRIVE_ACCELERATION_LIMIT = 3.0;

    /**
     * Threshold for detecting sharp turns (yaw input magnitude)
     */
    public static final double DRIVE_SHARP_TURN_THRESHOLD = 0.7;

    /**
     * Minimum movement threshold for analytics (combined input magnitude)
     */
    public static final double DRIVE_MOVEMENT_THRESHOLD = 0.1;

    /**
     * Fine movement scale factor for precision control
     */
    public static final double DRIVE_FINE_MOVEMENT_SCALE = 0.2;

    // =========================================================================================
    // SHOOTER SYSTEM TUNING
    // =========================================================================================

    /**
     * Target RPM for shooter motors
     */
    public static final double SHOOTER_TARGET_RPM = 3000.0;

    /**
     * RPM tolerance for considering shooter "at speed"
     */
    public static final double SHOOTER_RPM_TOLERANCE = 50.0;

    /**
     * Time to wait for shooter to spin up (seconds)
     */
    public static final double SHOOTER_SPINUP_TIME = 2.0;

    /**
     * High goal shooting RPM for maximum range
     */
    public static final double SHOOTER_HIGH_GOAL_RPM = 4000.0;

    /**
     * Mid goal shooting RPM for balanced power and accuracy
     */
    public static final double SHOOTER_MID_GOAL_RPM = 3000.0;

    /**
     * Low goal shooting RPM for close range, high accuracy
     */
    public static final double SHOOTER_LOW_GOAL_RPM = 2000.0;

    /**
     * Maximum possible RPM for the shooter system
     */
    public static final double SHOOTER_MAX_RPM = 5000.0;

    /**
     * Time for shooter to spin up to target RPM (seconds)
     */
    public static final double SHOOTER_SPIN_UP_TIME = 1.5;

    /**
     * Delay between feed servo activation and deactivation (seconds)
     */
    public static final double SHOOTER_FEED_DELAY = 0.5;

    /**
     * Power for feed servos (0.0 to 1.0)
     */
    public static final double SHOOTER_FEED_SERVO_POWER = 0.8;

    /**
     * Maximum power adjustment per control loop iteration
     */
    public static final double SHOOTER_MAX_POWER_ADJUSTMENT = 0.1;

    // =========================================================================================
    // SHOOTER PID CONTROL CONSTANTS
    // =========================================================================================

    /**
     * Proportional gain for shooter RPM control
     */
    public static final double SHOOTER_KP = 0.0005;

    /**
     * Integral gain for shooter RPM control
     */
    public static final double SHOOTER_KI = 0.0001;

    /**
     * Derivative gain for shooter RPM control
     */
    public static final double SHOOTER_KD = 0.00001;

    // =========================================================================================
    // COLLECTOR SYSTEM TUNING
    // =========================================================================================

    /**
     * Power for intake motor
     */
    public static final double INTAKE_POWER = 0.8;

    /**
     * Power for outtake motor
     */
    public static final double OUTTAKE_POWER = -0.8;

    /**
     * Time to run collector after element detection (seconds)
     */
    public static final double COLLECTION_HOLD_TIME = 0.5;

    // =========================================================================================
    // SENSOR THRESHOLDS
    // =========================================================================================

    /**
     * Distance threshold for obstacle detection (inches)
     */
    public static final double OBSTACLE_DETECTION_DISTANCE = 12.0;

    /**
     * Distance threshold for element detection (inches)
     */
    public static final double ELEMENT_DETECTION_DISTANCE = 3.0;

    /**
     * Minimum confidence for vision target detection (0.0 to 1.0)
     */
    public static final double VISION_CONFIDENCE_THRESHOLD = 0.7;

    // =========================================================================================
    // AUTONOMOUS TUNING
    // =========================================================================================

    /**
     * Default autonomous movement speed
     */
    public static final double AUTO_DRIVE_SPEED = 0.6;

    /**
     * Default autonomous turn speed
     */
    public static final double AUTO_TURN_SPEED = 0.4;

    /**
     * Maximum autonomous velocity for position validation (inches/second)
     */
    public static final double AUTO_MAX_VELOCITY = 60.0;

    /**
     * Position tolerance for autonomous movements (inches)
     */
    public static final double AUTO_POSITION_TOLERANCE = 1.0;

    /**
     * Heading tolerance for autonomous turns (degrees)
     */
    public static final double AUTO_HEADING_TOLERANCE = 2.0;

    // =========================================================================================
    // PERFORMANCE TUNING
    // =========================================================================================

    /**
     * Maximum acceptable update time for state machine (seconds)
     */
    public static final double MAX_UPDATE_TIME = 0.020; // 20ms

    /**
     * Maximum acceptable loop time for main robot loop (seconds)
     */
    public static final double MAX_LOOP_TIME = 0.050; // 50ms

    /**
     * Frequency for telemetry updates (Hz)
     */
    public static final double TELEMETRY_UPDATE_RATE = 10.0;

    // =========================================================================================
    // TELEOP CONFIGURATION
    // =========================================================================================

    /**
     * Default field-relative mode setting for teleop
     */
    public static final boolean TELEOP_FIELD_RELATIVE_DEFAULT = true;

    // =========================================================================================
    // DEBUG CONFIGURATION
    // =========================================================================================

    /**
     * Global debug mode flag
     */
    public static final boolean DEBUG_ENABLED = false;

    /**
     * Check if debug is enabled for a specific subsystem
     * @param subsystem The subsystem name to check
     * @return true if debug is enabled for this subsystem
     */
    public static boolean isDebugEnabled(String subsystem) {
        // For now, return global debug flag
        // In the future, this could be expanded to per-subsystem debug flags
        return DEBUG_ENABLED;
    }

    // =========================================================================================
    // SAFETY LIMITS
    // =========================================================================================

    /**
     * Emergency stop activation distance (inches)
     */
    public static final double EMERGENCY_STOP_DISTANCE = 6.0;

    /**
     * Maximum tilt angle before emergency stop (degrees)
     */
    public static final double MAX_TILT_ANGLE = 30.0;

    /**
     * Minimum battery voltage for operation (volts)
     */
    public static final double MIN_BATTERY_VOLTAGE = 11.0;

    // =========================================================================================
    // LOCALIZATION SYSTEM CONFIGURATION
    // =========================================================================================

    /**
     * Localization mode constants
     */
    public static final int LOCALIZATION_MODE_SENSOR_FUSION = 1;
    public static final int LOCALIZATION_MODE_ODOMETRY_VISION_INIT = 2;
    public static final int LOCALIZATION_MODE_ODOMETRY_ONLY = 3;

    /**
     * Default localization mode (1=Sensor Fusion, 2=Odometry+Vision Init, 3=Odometry Only)
     */
    public static final int LOCALIZATION_DEFAULT_MODE = LOCALIZATION_MODE_SENSOR_FUSION;

    /**
     * Default starting position on field (inches)
     */
    public static final double LOCALIZATION_DEFAULT_START_X = 0.0;
    public static final double LOCALIZATION_DEFAULT_START_Y = 0.0;
    public static final double LOCALIZATION_DEFAULT_START_HEADING = 0.0;

    /**
     * Update rates for localization system (seconds)
     */
    public static final double LOCALIZATION_FUSION_UPDATE_RATE = 0.02; // 50Hz fusion rate
    public static final double LOCALIZATION_TELEMETRY_UPDATE_RATE = 0.1; // 10Hz telemetry rate

    /**
     * Sensor fusion weights (vision, odometry) - must sum to 1.0
     * Odometry is heavily favored for post-initialization tracking
     */
    public static final double LOCALIZATION_VISION_WEIGHT = 0.15;
    public static final double LOCALIZATION_ODOMETRY_WEIGHT = 0.85;

    /**
     * Get normalized fusion weights for sensor fusion
     */
    public static double[] getNormalizedFusionWeights() {
        double total = LOCALIZATION_VISION_WEIGHT + LOCALIZATION_ODOMETRY_WEIGHT;
        return new double[]{
            LOCALIZATION_VISION_WEIGHT / total,
            LOCALIZATION_ODOMETRY_WEIGHT / total
        };
    }

    /**
     * Confidence and validation thresholds
     */
    public static final double LOCALIZATION_CONFIDENCE_THRESHOLD = 0.7;
    public static final double LOCALIZATION_MIN_VISION_CONFIDENCE = 0.5;
    public static final int LOCALIZATION_MIN_VISION_DETECTIONS = 3;

    /**
     * Timeout values (seconds)
     */
    public static final double LOCALIZATION_VISION_TIMEOUT = 2.0;
    public static final double LOCALIZATION_STALENESS_TIMEOUT = 1.0;

    /**
     * Position validation limits
     */
    public static final double LOCALIZATION_MAX_POSITION_JUMP = 24.0; // 24 inches max jump
    public static final double LOCALIZATION_VELOCITY_SANITY_CHECK = 120.0; // 120 in/s max velocity
    public static final double LOCALIZATION_TIP_DETECTION_THRESHOLD = 720.0; // 720 deg/s max angular velocity

    /**
     * Field constraints and validation
     */
    public static final boolean LOCALIZATION_USE_FIELD_CONSTRAINTS = true;
    public static final boolean LOCALIZATION_DETAILED_LOGGING = false;

    // =========================================================================================
    // VISION SYSTEM CONFIGURATION
    // =========================================================================================

    /**
     * Camera resolution settings
     */
    public static final int VISION_CAMERA_WIDTH = 640;
    public static final int VISION_CAMERA_HEIGHT = 480;
    public static final int VISION_CAMERA_FPS = 30;

    /**
     * Camera exposure and gain settings
     */
    public static final double VISION_EXPOSURE_MS = 6.0; // 6ms exposure
    public static final int VISION_GAIN = 250; // Camera gain

    /**
     * Vision detection thresholds
     */
    public static final double VISION_DETECTION_CONFIDENCE_THRESHOLD = 0.6;
    public static final double VISION_MAX_DETECTION_DISTANCE = 72.0; // 72 inches max detection range

    /**
     * Communication timeout for vision system (seconds)
     */
    public static final double COMMUNICATION_TIMEOUT = 3.0;

    /**
     * Sensor update period for position validation (seconds)
     */
    public static final double SENSOR_UPDATE_PERIOD = 0.02; // 50Hz

    // =========================================================================================
    // Private constructor to prevent instantiation
    private Tunables() {
        throw new UnsupportedOperationException("Tunables is a utility class and cannot be instantiated");
    }
}
