/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Robot Lift Controller with Performance Monitoring and Auto-Compensation
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;
import java.util.Queue;

/**
 * RobotLiftController - Advanced dual-motor lift system with performance monitoring
 *
 * Features:
 * - Dual motor synchronization (robotLiftLeft & robotLiftRight)
 * - Encoder-based position limits (configurable min/max positions)
 * - Performance monitoring (power usage, speed, load detection)
 * - Automatic gravity compensation (detects and prevents unintended drops)
 * - Load-based performance analysis
 * - Safety limits and emergency stop
 *
 * SETUP INSTRUCTIONS:
 *
 * 1. FINDING NO-LOAD SPEED:
 *    - Remove any load from the lift (detach mechanism if possible)
 *    - Run the lift at 100% power for a known distance
 *    - Measure how long it takes to move 1000 ticks
 *    - Calculate: EXPECTED_TICKS_PER_SECOND_NO_LOAD = 1000 / time_in_seconds
 *    - Example: If it takes 0.5 seconds to move 1000 ticks at 100% power
 *      then EXPECTED_TICKS_PER_SECOND_NO_LOAD = 1000 / 0.5 = 2000
 *
 * 2. FINDING POSITION LIMITS:
 *    - Reset encoders with lift at lowest safe position
 *    - Manually move lift to highest safe position
 *    - Note the encoder value - this is your MAX_POSITION
 *    - MIN_POSITION should typically be 0 or slightly above 0
 *
 * 3. TUNING GRAVITY COMPENSATION:
 *    - Start with GRAVITY_HOLD_POWER = 0.1
 *    - Hold lift at mid-height with no input
 *    - If it drops, increase power by 0.05
 *    - If it rises, decrease power by 0.05
 *    - Optimal value is when lift stays stationary with no driver input
 *
 * 4. TUNING LOAD THRESHOLDS:
 *    - Run lift with normal game piece load
 *    - Check telemetry for "Load Factor"
 *    - NORMAL_LOAD_THRESHOLD should be ~120% of average loaded performance
 *    - HEAVY_LOAD_THRESHOLD should be ~150% of average loaded performance
 *
 * Usage Example in AURORATeleOp:
 * <pre>
 * // In class declaration:
 * private RobotLiftController liftController;
 *
 * // In runOpMode() initialization:
 * liftController = new RobotLiftController(hardwareMap, telemetry);
 * liftController.initialize();
 *
 * // In main loop:
 * liftController.update(gamepad2.left_stick_y);
 *
 * // Or with preset positions:
 * if (gamepad2.dpad_up) liftController.moveToPosition(RobotLiftController.POSITION_HIGH);
 * if (gamepad2.dpad_down) liftController.moveToPosition(RobotLiftController.POSITION_LOW);
 *
 * // Optional: Display lift telemetry
 * liftController.addTelemetry();
 * </pre>
 */
public class RobotLiftController {

    // ========================================================================================
    // HARDWARE CONFIGURATION - Adjust these to match your robot
    // ========================================================================================

    /** Hardware device names */
    private static final String LIFT_LEFT_MOTOR = "robotLiftLeft";
    private static final String LIFT_RIGHT_MOTOR = "robotLiftRight";

    // ========================================================================================
    // POSITION LIMITS - MUST BE CALIBRATED FOR YOUR ROBOT
    // ========================================================================================

    /**
     * Minimum safe position in encoder ticks
     * Set this to prevent the lift from going too low
     * CALIBRATION: Set lift to lowest safe position and note encoder value
     */
    private static final int MIN_POSITION = 0;

    /**
     * Maximum safe position in encoder ticks
     * Set this to prevent the lift from going too high
     * CALIBRATION: Reset encoders at bottom, manually move to top, note encoder value
     */
    private static final int MAX_POSITION = 5800;  // ADJUST THIS VALUE

    /**
     * Soft limit buffer zone (ticks before hard limit)
     * Lift will slow down when entering this zone
     */
    private static final int SOFT_LIMIT_BUFFER = 200;

    // ========================================================================================
    // PRESET POSITIONS - Convenient height presets for common operations
    // ========================================================================================

    /** Preset position: Lowest safe position */
    public static final int POSITION_GROUND = MIN_POSITION;

    /** Preset position: Low scoring position */
    public static final int POSITION_LOW = 800;  // ADJUST THIS VALUE

    /** Preset position: Medium scoring position */
    public static final int POSITION_MID = 2000;  // ADJUST THIS VALUE

    /** Preset position: High scoring position */
    public static final int POSITION_HIGH = 4500;  // ADJUST THIS VALUE

    // ========================================================================================
    // PERFORMANCE MONITORING CONSTANTS - Calibrate for accurate load detection
    // ========================================================================================

    /**
     * Expected encoder ticks per second at 100% power with NO LOAD
     * CALIBRATION INSTRUCTIONS:
     * 1. Remove all load from lift
     * 2. Run at 100% power and time how long it takes to move 1000 ticks
     * 3. Calculate: EXPECTED_TICKS_PER_SECOND_NO_LOAD = 1000 / time_in_seconds
     * 4. Example: If it takes 0.5 sec -> 1000/0.5 = 2000 ticks/sec
     */
    private static final double EXPECTED_TICKS_PER_SECOND_NO_LOAD = 651.3;  // CALIBRATE THIS

    /**
     * Load factor threshold for normal operation
     * Load factor = expected_speed / actual_speed
     * 1.0 = no load, >1.0 = loaded
     * CALIBRATION: Run with typical game piece, check telemetry "Load Factor"
     * Set this to ~120% of normal loaded value (e.g., if normal is 1.3, use 1.5)
     */
    private static final double NORMAL_LOAD_THRESHOLD = 1.5;  // TUNE THIS VALUE

    /**
     * Load factor threshold for heavy load detection
     * CALIBRATION: Set to ~150% of normal loaded value
     */
    private static final double HEAVY_LOAD_THRESHOLD = 2.0;  // TUNE THIS VALUE

    /**
     * Minimum speed (ticks/sec) below which we don't calculate load
     * Prevents divide-by-zero and false positives when moving slowly
     */
    private static final double MIN_SPEED_FOR_LOAD_CALC = 50.0;

    /**
     * Stall detection threshold
     * If power > this value but speed < MIN_SPEED_FOR_LOAD_CALC, lift is considered stalled
     */
    private static final double STALL_DETECTION_POWER_THRESHOLD = 0.2;

    // ========================================================================================
    // MOTOR POWER CONSTANTS
    // ========================================================================================

    /**
     * Maximum motor power (0.0 to 1.0)
     * Reduce this if motors are too aggressive or draw too much current
     */
    private static final double MAX_MOTOR_POWER = 1.0;

    /**
     * Minimum motor power to overcome static friction
     * Motors won't move below this power level
     */
    private static final double MIN_MOTOR_POWER = 0.05;

    /**
     * Power multiplier when in soft limit zone
     * Reduces speed near limits for safety
     */
    private static final double SOFT_LIMIT_POWER_MULTIPLIER = 0.5;

    /**
     * Initial gravity compensation power (0.0 to 1.0)
     * Starting value for automatic PID adjustment
     * System will automatically tune this value during operation
     * REDUCED to prevent initial movement issues
     */
    private static final double INITIAL_GRAVITY_HOLD_POWER = 0.0;  // Starting point - will auto-adjust

    /**
     * Minimum gravity compensation power
     * Prevents PID from adjusting too low
     */
    private static final double MIN_GRAVITY_HOLD_POWER = 0.0;

    /**
     * Maximum gravity compensation power
     * Prevents PID from adjusting too high
     */
    private static final double MAX_GRAVITY_HOLD_POWER = 0.5;

    // ========================================================================================
    // AUTOMATIC GRAVITY COMPENSATION PID CONSTANTS
    // ========================================================================================

    /**
     * Proportional gain for gravity compensation PID
     * How aggressively to respond to position drift
     * Higher = more aggressive correction, may oscillate
     * Lower = slower correction, may drift more
     */
    private static final double GRAVITY_PID_KP = 0.0001;  // TUNE THIS VALUE

    /**
     * Integral gain for gravity compensation PID
     * Eliminates steady-state error (persistent drift)
     * Higher = faster elimination of drift, may cause overshoot
     * Lower = slower drift correction, more stable
     */
    private static final double GRAVITY_PID_KI = 0.00002;  // TUNE THIS VALUE

    /**
     * Derivative gain for gravity compensation PID
     * Reduces oscillation and overshoot
     * Higher = more damping, may be sluggish
     * Lower = less damping, may oscillate
     */
    private static final double GRAVITY_PID_KD = 0.0005;  // TUNE THIS VALUE

    /**
     * Maximum allowed integral accumulation (wind-up prevention)
     * Prevents integral term from growing too large
     */
    private static final double GRAVITY_PID_MAX_INTEGRAL = 0.1;

    /**
     * Position error threshold (ticks) for gravity compensation
     * Lift position must stay within this range when holding
     */
    private static final int GRAVITY_HOLD_POSITION_TOLERANCE = 15;

    /**
     * Time required (seconds) to be stationary before auto-tuning activates
     * Prevents PID from adjusting during intentional movements
     */
    private static final double GRAVITY_TUNE_ACTIVATION_TIME = 0.5;

    // ========================================================================================
    // POSITION CONTROL CONSTANTS
    // ========================================================================================

    /** Position tolerance for "at target" detection (encoder ticks) */
    private static final int POSITION_TOLERANCE = 20;

    /** Proportional gain for position control (power per tick error) */
    private static final double POSITION_KP = 0.001;  // TUNE THIS VALUE

    /** Minimum power for position control movements */
    private static final double MIN_POSITION_POWER = 0.1;

    /** Maximum power for position control movements */
    private static final double MAX_POSITION_POWER = 0.8;

    // ========================================================================================
    // DROP DETECTION CONSTANTS
    // ========================================================================================

    /**
     * Minimum position change (ticks) to count as unintended drop
     * Must be negative (dropping down)
     */
    private static final int DROP_DETECTION_THRESHOLD = -10;

    /**
     * Time window (milliseconds) for drop detection
     * Checks if position has decreased over this time period
     */
    private static final double DROP_DETECTION_WINDOW_MS = 100.0;

    // ========================================================================================
    // PERFORMANCE MONITORING CONSTANTS
    // ========================================================================================

    /** Number of samples for moving average calculations */
    private static final int PERFORMANCE_SAMPLE_SIZE = 20;

    /** Update interval for performance calculations (milliseconds) */
    private static final double PERFORMANCE_UPDATE_INTERVAL_MS = 100.0;

    // ========================================================================================
    // MOTOR SYNCHRONIZATION CONSTANTS
    // ========================================================================================

    /**
     * Maximum allowed position difference between left and right motors (encoder ticks)
     * If motors differ by more than this, slower motor will be sped up to catch up
     */
    private static final int MAX_MOTOR_DESYNC = 50;

    /**
     * Power adjustment factor for motor synchronization
     * When motors are out of sync, lagging motor gets extra power and leading motor is reduced
     */
    private static final double SYNC_POWER_ADJUSTMENT = 0.15;

    /**
     * Minimum position difference to trigger synchronization (ticks)
     * Prevents constant micro-adjustments
     */
    private static final int MIN_SYNC_THRESHOLD = 10;

    // ========================================================================================
    // GAMEPAD VIBRATION PATTERNS (in milliseconds)
    // ========================================================================================

    /**
     * Short single pulse - soft limit reached
     */
    private static final int VIBRATION_SOFT_LIMIT = 100;

    /**
     * Double pulse - hard limit reached
     */
    private static final int VIBRATION_HARD_LIMIT_DURATION = 200;
    private static final int VIBRATION_HARD_LIMIT_PAUSE = 100;

    /**
     * Long continuous - stall/jam detected
     */
    private static final int VIBRATION_STALL_DURATION = 500;

    /**
     * Triple pulse - motor desynchronization warning
     */
    private static final int VIBRATION_DESYNC_PULSE = 150;
    private static final int VIBRATION_DESYNC_PAUSE = 80;

    /**
     * Quick pulse - position reached
     */
    private static final int VIBRATION_POSITION_REACHED = 150;

    /**
     * Minimum time between vibration warnings (ms) to prevent spam
     */
    private static final double VIBRATION_COOLDOWN_MS = 1000.0;

    // ========================================================================================
    // HARDWARE COMPONENTS
    // ========================================================================================

    private DcMotorEx liftMotorLeft;
    private DcMotorEx liftMotorRight;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private Gamepad gamepad1;  // Driver gamepad
    private Gamepad gamepad2;  // Operator gamepad (for dual-driver mode)

    // ========================================================================================
    // STATE TRACKING
    // ========================================================================================

    private boolean initialized = false;
    private boolean emergencyStop = false;

    // Individual motor position tracking
    private int leftMotorPosition = 0;
    private int rightMotorPosition = 0;
    private int motorPositionDifference = 0;  // left - right
    private boolean motorsDesynchronized = false;

    // Combined position tracking
    private int currentPosition = 0;  // Average of both motors
    private int targetPosition = 0;
    private boolean positionControlActive = false;

    // Drop detection
    private int lastPosition = 0;
    private ElapsedTime dropDetectionTimer = new ElapsedTime();
    private boolean unintendedDropDetected = false;

    // Automatic gravity compensation PID
    private double currentCompensationPower = INITIAL_GRAVITY_HOLD_POWER;
    private int holdTargetPosition = 0;  // Position to hold when stationary
    private double gravityPidIntegral = 0.0;  // Integral accumulator
    private double gravityPidLastError = 0.0;  // Previous error for derivative
    private ElapsedTime holdTimer = new ElapsedTime();  // Time spent holding
    private boolean gravityPidActive = false;  // Whether PID is currently tuning
    private ElapsedTime gravityPidTimer = new ElapsedTime();  // For PID update rate

    // Power and direction tracking
    private double lastCommandedPower = 0.0;
    private double currentAppliedPower = 0.0;
    private String currentDirection = "Stopped";  // "Up", "Down", or "Stopped"

    // ========================================================================================
    // PERFORMANCE MONITORING
    // ========================================================================================

    private ElapsedTime performanceTimer = new ElapsedTime();
    private Queue<Double> speedHistory = new LinkedList<>();
    private Queue<Double> powerHistory = new LinkedList<>();
    private Queue<Double> loadFactorHistory = new LinkedList<>();

    // Performance metrics
    private double currentSpeed = 0.0;  // ticks per second
    private double averageSpeed = 0.0;
    private double peakSpeed = 0.0;

    private double currentLoadFactor = 1.0;  // 1.0 = no load, >1.0 = loaded
    private double averageLoadFactor = 1.0;
    private double peakLoadFactor = 1.0;

    private double averagePower = 0.0;
    private double totalEnergyUsed = 0.0;  // Approximate (power * time)

    // Position tracking for speed calculation
    private int lastPositionForSpeed = 0;
    private ElapsedTime speedTimer = new ElapsedTime();

    // Load status
    public enum LoadStatus {
        IDLE("Idle", "○"),              // No power applied
        NO_LOAD("No Load", "✓"),        // Moving freely with minimal resistance
        NORMAL_LOAD("Normal", "◆"),     // Normal operating load
        HEAVY_LOAD("Heavy", "⚠"),       // Heavy load detected
        OVERLOAD("Overload", "⚠⚠"),     // Excessive load
        STALLED("STALLED", "⛔"),       // Power applied but not moving (stuck/jammed)
        UNKNOWN("Unknown", "?");         // Initial state or error

        private final String displayName;
        private final String symbol;

        LoadStatus(String displayName, String symbol) {
            this.displayName = displayName;
            this.symbol = symbol;
        }

        public String getDisplayName() { return displayName; }
        public String getSymbol() { return symbol; }
    }

    private LoadStatus currentLoadStatus = LoadStatus.UNKNOWN;

    // ========================================================================================
    // VIBRATION FEEDBACK STATE
    // ========================================================================================

    private ElapsedTime vibrationCooldownTimer = new ElapsedTime();
    private boolean vibrationsEnabled = true;
    private boolean lastStallState = false;
    private boolean lastDesyncState = false;
    private boolean lastSoftLimitState = false;
    private int lastAutoPositionTarget = -1;

    // ========================================================================================
    // CONSTRUCTOR
    // ========================================================================================

    /**
     * Create a new RobotLiftController
     * @param hardwareMap FTC hardware map for device access
     * @param telemetry Telemetry for status reporting
     */
    public RobotLiftController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = null;
        this.gamepad2 = null;
    }

    /**
     * Create a new RobotLiftController with gamepad vibration support
     * @param hardwareMap FTC hardware map for device access
     * @param telemetry Telemetry for status reporting
     * @param gamepad1 Driver gamepad for vibration feedback (can be null)
     * @param gamepad2 Operator gamepad for vibration feedback (can be null)
     */
    public RobotLiftController(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Set gamepads for vibration feedback (can be called after construction)
     * @param gamepad1 Driver gamepad (can be null)
     * @param gamepad2 Operator gamepad (can be null)
     */
    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Enable or disable gamepad vibration feedback
     * @param enabled true to enable vibrations, false to disable
     */
    public void setVibrationsEnabled(boolean enabled) {
        this.vibrationsEnabled = enabled;
    }

    /**
     * Check if vibrations are enabled
     * @return true if enabled, false otherwise
     */
    public boolean areVibrationsEnabled() {
        return vibrationsEnabled;
    }

    // ========================================================================================
    // INITIALIZATION
    // ========================================================================================

    /**
     * Initialize the lift controller and configure motors
     * Call this in OpMode initialization, before waitForStart()
     * @return true if initialization successful, false otherwise
     */
    public boolean initialize() {
        try {
            telemetry.addLine("Initializing Lift Controller...");
            telemetry.update();

            // Get motor hardware
            liftMotorLeft = hardwareMap.get(DcMotorEx.class, LIFT_LEFT_MOTOR);
            liftMotorRight = hardwareMap.get(DcMotorEx.class, LIFT_RIGHT_MOTOR);

            // Configure motors
            // Assuming motors are mirrored, one needs to be reversed
            liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);
            liftMotorRight.setDirection(DcMotor.Direction.REVERSE);  // ADJUST IF NEEDED

            // Set motor behavior
            liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Use encoders for position tracking
            liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize state
            currentPosition = 0;
            lastPosition = 0;
            lastPositionForSpeed = 0;
            targetPosition = 0;
            holdTargetPosition = 0;

            // Start timers
            dropDetectionTimer.reset();
            performanceTimer.reset();
            speedTimer.reset();
            holdTimer.reset();
            gravityPidTimer.reset();

            // Initialize PID state
            gravityPidIntegral = 0.0;
            gravityPidLastError = 0.0;
            gravityPidActive = false;
            currentCompensationPower = INITIAL_GRAVITY_HOLD_POWER;

            initialized = true;

            telemetry.addLine("✅ Lift Controller Initialized");
            telemetry.addData("Left Motor", LIFT_LEFT_MOTOR);
            telemetry.addData("Right Motor", LIFT_RIGHT_MOTOR);
            telemetry.addData("Position Limits", "%d to %d ticks", MIN_POSITION, MAX_POSITION);
            telemetry.addData("Auto Gravity PID", "ENABLED");
            telemetry.addData("Initial Hold Power", "%.2f", INITIAL_GRAVITY_HOLD_POWER);
            telemetry.update();

            return true;

        } catch (Exception e) {
            telemetry.addLine("❌ Lift Controller Init Failed!");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            initialized = false;
            return false;
        }
    }

    // ========================================================================================
    // MAIN UPDATE METHOD
    // ========================================================================================

    /**
     * Update the lift controller - call this every loop
     * @param controlInput Driver input from joystick (-1.0 to 1.0, negative = down, positive = up)
     */
    public void update(double controlInput) {
        if (!initialized) {
            return;
        }

        // Update current position from encoders (average of both motors)
        updatePosition();

        // Update performance monitoring
        updatePerformanceMetrics();

        // Update automatic gravity compensation with PID
        updateAutomaticGravityCompensation(controlInput);

        // Determine motor power based on mode
        double powerToApply;

        if (emergencyStop) {
            powerToApply = 0.0;
            positionControlActive = false;
        } else if (positionControlActive && Math.abs(controlInput) < 0.1) {
            // Position control mode (moving to preset)
            powerToApply = calculatePositionControlPower();

            // Check if we've reached target
            if (Math.abs(currentPosition - targetPosition) < POSITION_TOLERANCE) {
                positionControlActive = false;
                powerToApply = 0.0;
            }
        } else {
            // Manual control mode
            positionControlActive = false;
            powerToApply = calculateManualControlPower(controlInput);
        }

        // Apply power to motors
        applyMotorPower(powerToApply);

        // Store for next iteration
        lastCommandedPower = controlInput;
        currentAppliedPower = powerToApply;
    }

    // ========================================================================================
    // POSITION TRACKING
    // ========================================================================================

    /**
     * Update current position from motor encoders
     * Tracks both motors individually and detects desynchronization
     */
    private void updatePosition() {
        // Read individual motor positions
        leftMotorPosition = liftMotorLeft.getCurrentPosition();
        rightMotorPosition = liftMotorRight.getCurrentPosition();

        // Calculate position difference (positive = left ahead, negative = right ahead)
        motorPositionDifference = leftMotorPosition - rightMotorPosition;

        // Check if motors are significantly out of sync
        motorsDesynchronized = Math.abs(motorPositionDifference) > MIN_SYNC_THRESHOLD;

        // Use average of both motors for overall position tracking
        currentPosition = (leftMotorPosition + rightMotorPosition) / 2;
    }

    /**
     * Get left motor position
     * @return Left motor position in encoder ticks
     */
    public int getLeftMotorPosition() {
        return leftMotorPosition;
    }

    /**
     * Get right motor position
     * @return Right motor position in encoder ticks
     */
    public int getRightMotorPosition() {
        return rightMotorPosition;
    }

    /**
     * Get position difference between motors
     * @return Position difference (left - right) in encoder ticks
     */
    public int getMotorPositionDifference() {
        return motorPositionDifference;
    }

    /**
     * Check if motors are desynchronized
     * @return true if position difference exceeds threshold
     */
    public boolean areMotorsDesynchronized() {
        return motorsDesynchronized;
    }

    /**
     * Get current lift position
     * @return Current position in encoder ticks
     */
    public int getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Get current position as percentage of total range
     * @return Position as percentage (0.0 to 100.0)
     */
    public double getPositionPercentage() {
        return 100.0 * (currentPosition - MIN_POSITION) / (MAX_POSITION - MIN_POSITION);
    }

    /**
     * Check if lift is at minimum position
     */
    public boolean isAtMinimum() {
        return currentPosition <= (MIN_POSITION + POSITION_TOLERANCE);
    }

    /**
     * Check if lift is at maximum position
     */
    public boolean isAtMaximum() {
        return currentPosition >= (MAX_POSITION - POSITION_TOLERANCE);
    }

    /**
     * Check if lift is in soft limit zone (near limits)
     */
    private boolean isInSoftLimitZone() {
        return currentPosition < (MIN_POSITION + SOFT_LIMIT_BUFFER) ||
               currentPosition > (MAX_POSITION - SOFT_LIMIT_BUFFER);
    }

    // ========================================================================================
    // PRESET POSITION CONTROL
    // ========================================================================================

    /**
     * Move lift to a preset position
     * @param position Target position in encoder ticks
     */
    public void moveToPosition(int position) {
        if (!initialized || emergencyStop) {
            return;
        }

        // Clamp to valid range
        targetPosition = Range.clip(position, MIN_POSITION, MAX_POSITION);
        positionControlActive = true;
    }

    /**
     * Move to ground position (lowest safe position)
     */
    public void moveToGround() {
        moveToPosition(POSITION_GROUND);
    }

    /**
     * Move to low scoring position
     */
    public void moveToLow() {
        moveToPosition(POSITION_LOW);
    }

    /**
     * Move to medium scoring position
     */
    public void moveToMid() {
        moveToPosition(POSITION_MID);
    }

    /**
     * Move to high scoring position
     */
    public void moveToHigh() {
        moveToPosition(POSITION_HIGH);
    }

    /**
     * Cancel position control and return to manual mode
     */
    public void cancelPositionControl() {
        positionControlActive = false;
    }

    /**
     * Check if currently executing position control
     */
    public boolean isPositionControlActive() {
        return positionControlActive;
    }

    /**
     * Calculate power for position control using proportional control
     */
    private double calculatePositionControlPower() {
        int error = targetPosition - currentPosition;

        // Proportional control
        double power = error * POSITION_KP;

        // Clamp to reasonable range
        power = Range.clip(power, -MAX_POSITION_POWER, MAX_POSITION_POWER);

        // Ensure minimum power to overcome friction
        if (Math.abs(power) > 0.01 && Math.abs(power) < MIN_POSITION_POWER) {
            power = Math.signum(power) * MIN_POSITION_POWER;
        }

        return power;
    }

    // ========================================================================================
    // MANUAL CONTROL
    // ========================================================================================

    /**
     * Calculate motor power for manual control
     * Includes safety limits and gravity compensation
     */
    private double calculateManualControlPower(double controlInput) {
        double power = 0.0;

        // Check if driver is providing input
        boolean driverInputActive = Math.abs(controlInput) > 0.05;

        if (driverInputActive) {
            // Driver is actively controlling lift
            power = controlInput * MAX_MOTOR_POWER;

            // Reduce power in soft limit zones
            if (isInSoftLimitZone()) {
                power *= SOFT_LIMIT_POWER_MULTIPLIER;
            }

            // Hard limits - prevent movement beyond range
            if (currentPosition <= MIN_POSITION && power < 0) {
                power = 0;  // Can't go lower
            }
            if (currentPosition >= MAX_POSITION && power > 0) {
                power = 0;  // Can't go higher
            }

        } else {
            // No driver input - apply gravity compensation only if not at bottom
            // At minimum position, no holding force is needed (lift rests on hard stop)
            if (currentPosition > (MIN_POSITION + GRAVITY_HOLD_POSITION_TOLERANCE)) {
                power = currentCompensationPower;
            } else {
                power = 0.0;  // No compensation needed at bottom
            }
        }

        return power;
    }

    // ========================================================================================
    // AUTOMATIC GRAVITY COMPENSATION WITH PID
    // ========================================================================================

    /**
     * Automatic gravity compensation using PID control
     * Continuously adjusts hold power to maintain position when stationary
     * This replaces manual drop detection with intelligent auto-tuning
     */
    private void updateAutomaticGravityCompensation(double controlInput) {
        // Check if driver is actively controlling lift
        boolean driverInputActive = Math.abs(controlInput) > 0.05;

        // Check if lift is at or near minimum position (no compensation needed)
        boolean atMinimumPosition = currentPosition <= (MIN_POSITION + GRAVITY_HOLD_POSITION_TOLERANCE);

        if (driverInputActive) {
            // Driver is actively moving lift - disable PID tuning
            gravityPidActive = false;
            gravityPidIntegral = 0.0;  // Reset integral to prevent wind-up
            holdTimer.reset();
            holdTargetPosition = currentPosition;  // Update target for when they stop
            return;
        }

        // If at minimum position, disable PID and reset compensation
        if (atMinimumPosition) {
            gravityPidActive = false;
            gravityPidIntegral = 0.0;
            currentCompensationPower = 0.0;  // No compensation at bottom
            holdTimer.reset();
            return;
        }

        // Driver has released controls - prepare for hold mode
        double timeSinceRelease = holdTimer.seconds();

        // Wait for lift to settle before activating PID
        if (timeSinceRelease < GRAVITY_TUNE_ACTIVATION_TIME) {
            // Still settling, just use current compensation
            holdTargetPosition = currentPosition;  // Keep updating target during settle
            return;
        }

        // Activate PID control
        if (!gravityPidActive) {
            gravityPidActive = true;
            holdTargetPosition = currentPosition;  // Lock in the hold position
            gravityPidTimer.reset();
        }

        // Calculate position error
        int positionError = holdTargetPosition - currentPosition;

        // Check for significant drop (safety check)
        if (positionError > DROP_DETECTION_THRESHOLD * -1) {  // Positive error = dropping
            unintendedDropDetected = true;
        } else if (Math.abs(positionError) <= GRAVITY_HOLD_POSITION_TOLERANCE) {
            unintendedDropDetected = false;
        }

        // Only run PID at reasonable update rate (prevent high-frequency adjustments)
        if (gravityPidTimer.milliseconds() < 50.0) {  // 20Hz update rate
            return;
        }

        double dt = gravityPidTimer.seconds();
        gravityPidTimer.reset();

        // PID Calculations
        // P term: Proportional to current error
        double pTerm = GRAVITY_PID_KP * positionError;

        // I term: Accumulated error over time (eliminates steady-state drift)
        gravityPidIntegral += positionError * dt;
        // Prevent integral wind-up
        gravityPidIntegral = Range.clip(gravityPidIntegral,
            -GRAVITY_PID_MAX_INTEGRAL, GRAVITY_PID_MAX_INTEGRAL);
        double iTerm = GRAVITY_PID_KI * gravityPidIntegral;

        // D term: Rate of change of error (dampens oscillation)
        double errorRate = (positionError - gravityPidLastError) / dt;
        double dTerm = GRAVITY_PID_KD * errorRate;
        gravityPidLastError = positionError;

        // Calculate total PID adjustment
        double pidAdjustment = pTerm + iTerm + dTerm;

        // Update compensation power with PID adjustment
        currentCompensationPower += pidAdjustment;

        // Clamp to safe range
        currentCompensationPower = Range.clip(currentCompensationPower,
            MIN_GRAVITY_HOLD_POWER, MAX_GRAVITY_HOLD_POWER);
    }

    /**
     * Check if an unintended drop is currently detected
     */
    public boolean isUnintendedDropDetected() {
        return unintendedDropDetected;
    }

    /**
     * Get current gravity compensation power being applied
     */
    public double getCurrentCompensationPower() {
        return currentCompensationPower;
    }

    /**
     * Manually set gravity compensation power (overrides PID)
     * Use this for manual tuning or to disable compensation
     * @param power Power to apply (0.0 to MAX_GRAVITY_HOLD_POWER)
     */
    public void setManualCompensationPower(double power) {
        gravityPidActive = false;
        gravityPidIntegral = 0.0;
        currentCompensationPower = Range.clip(power, MIN_GRAVITY_HOLD_POWER, MAX_GRAVITY_HOLD_POWER);
    }

    /**
     * Disable all gravity compensation (PID and manual)
     * Use this if experiencing unwanted movement
     */
    public void disableGravityCompensation() {
        gravityPidActive = false;
        gravityPidIntegral = 0.0;
        currentCompensationPower = 0.0;
    }

    /**
     * Reset and re-enable automatic PID gravity compensation
     */
    public void resetGravityCompensation() {
        gravityPidActive = false;
        gravityPidIntegral = 0.0;
        gravityPidLastError = 0.0;
        currentCompensationPower = INITIAL_GRAVITY_HOLD_POWER;
        holdTimer.reset();
    }

    // ========================================================================================
    // MOTOR CONTROL
    // ========================================================================================

    /**
     * Apply power to both lift motors with automatic synchronization
     * If motors are out of sync, adjusts power to each motor individually to re-sync
     */
    private void applyMotorPower(double power) {
        if (!initialized) {
            return;
        }

        double leftPower = power;
        double rightPower = power;

        // Check if motors are desynchronized and need correction
        if (motorsDesynchronized && Math.abs(power) > 0.05) {
            // Calculate sync adjustment based on position difference
            // Positive difference = left is ahead, slow down left and speed up right
            // Negative difference = right is ahead, speed up left and slow down right

            double syncAdjustment = SYNC_POWER_ADJUSTMENT;

            // Check if difference is critical (near max desync limit)
            if (Math.abs(motorPositionDifference) > MAX_MOTOR_DESYNC) {
                syncAdjustment *= 1.5;  // More aggressive correction
            }

            if (motorPositionDifference > 0) {
                // Left motor is ahead - slow it down, speed up right
                leftPower = power * (1.0 - syncAdjustment);
                rightPower = power * (1.0 + syncAdjustment);
            } else {
                // Right motor is ahead - speed up left, slow down right
                leftPower = power * (1.0 + syncAdjustment);
                rightPower = power * (1.0 - syncAdjustment);
            }

            // Ensure we don't reverse direction when synchronizing
            if (Math.signum(leftPower) != Math.signum(power)) {
                leftPower = 0;
            }
            if (Math.signum(rightPower) != Math.signum(power)) {
                rightPower = 0;
            }

            // Clamp to valid power range
            leftPower = Range.clip(leftPower, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);
            rightPower = Range.clip(rightPower, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);
        }

        // Apply power to motors
        liftMotorLeft.setPower(leftPower);
        liftMotorRight.setPower(rightPower);
    }

    /**
     * Stop both motors immediately
     */
    public void stop() {
        applyMotorPower(0.0);
        positionControlActive = false;
    }

    /**
     * Emergency stop - immediately halt all motion
     */
    public void emergencyStop() {
        emergencyStop = true;
        stop();
    }

    /**
     * Clear emergency stop and resume normal operation
     */
    public void clearEmergencyStop() {
        emergencyStop = false;
    }

    /**
     * Check if emergency stop is active
     */
    public boolean isEmergencyStopped() {
        return emergencyStop;
    }

    // ========================================================================================
    // PERFORMANCE MONITORING
    // ========================================================================================

    /**
     * Update performance metrics and calculate load factors
     */
    private void updatePerformanceMetrics() {
        // Only update at specified interval to reduce CPU load
        if (performanceTimer.milliseconds() < PERFORMANCE_UPDATE_INTERVAL_MS) {
            return;
        }

        // Calculate current speed (ticks per second)
        double deltaTime = speedTimer.seconds();
        if (deltaTime > 0.001) {  // Avoid division by zero
            int deltaPosition = currentPosition - lastPositionForSpeed;
            currentSpeed = Math.abs(deltaPosition / deltaTime);

            // Determine movement direction for telemetry
            if (Math.abs(currentAppliedPower) < 0.05) {
                currentDirection = "Stopped";
            } else if (deltaPosition > 5) {
                currentDirection = "Up ↑";
            } else if (deltaPosition < -5) {
                currentDirection = "Down ↓";
            } else if (Math.abs(currentAppliedPower) > STALL_DETECTION_POWER_THRESHOLD) {
                currentDirection = "Stalled";
            } else {
                currentDirection = "Holding";
            }

            // Update speed history
            speedHistory.offer(currentSpeed);
            if (speedHistory.size() > PERFORMANCE_SAMPLE_SIZE) {
                speedHistory.poll();
            }

            // Calculate average speed
            averageSpeed = speedHistory.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(0.0);

            // Track peak speed
            peakSpeed = Math.max(peakSpeed, currentSpeed);

            // Update position tracking
            lastPositionForSpeed = currentPosition;
            speedTimer.reset();
        }

        // Calculate load factor based on movement state
        double absPower = Math.abs(currentAppliedPower);

        if (absPower < MIN_MOTOR_POWER) {
            // No power applied - idle state
            currentLoadStatus = LoadStatus.IDLE;
            currentLoadFactor = 1.0;

        } else if (currentSpeed < MIN_SPEED_FOR_LOAD_CALC) {
            // Power applied but not moving much
            if (absPower > STALL_DETECTION_POWER_THRESHOLD) {
                // Significant power but no movement - STALLED
                currentLoadStatus = LoadStatus.STALLED;
                currentLoadFactor = 99.9;  // Extremely high load factor
            } else {
                // Low power, low speed - could be starting/stopping or gravity compensation
                currentLoadStatus = LoadStatus.UNKNOWN;
            }

        } else {
            // Moving significantly - calculate load factor
            // Expected speed at current power level (works for both up and down)
            double expectedSpeed = EXPECTED_TICKS_PER_SECOND_NO_LOAD * absPower;

            // Load factor = how much slower we're moving than expected
            // 1.0 = no load (moving at expected speed)
            // >1.0 = loaded (moving slower than expected)
            currentLoadFactor = expectedSpeed / currentSpeed;

            // Update load factor history
            loadFactorHistory.offer(currentLoadFactor);
            if (loadFactorHistory.size() > PERFORMANCE_SAMPLE_SIZE) {
                loadFactorHistory.poll();
            }

            // Calculate average load factor
            averageLoadFactor = loadFactorHistory.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(1.0);

            // Track peak load
            peakLoadFactor = Math.max(peakLoadFactor, currentLoadFactor);

            // Determine load status based on factor
            updateLoadStatus();
        }

        // Track power usage
        powerHistory.offer(Math.abs(currentAppliedPower));
        if (powerHistory.size() > PERFORMANCE_SAMPLE_SIZE) {
            powerHistory.poll();
        }

        // Calculate average power
        averagePower = powerHistory.stream()
            .mapToDouble(Double::doubleValue)
            .average()
            .orElse(0.0);

        // Approximate energy usage (power * time)
        totalEnergyUsed += Math.abs(currentAppliedPower) * (PERFORMANCE_UPDATE_INTERVAL_MS / 1000.0);

        performanceTimer.reset();
    }

    /**
     * Update load status based on current load factor
     * Only called when lift is actively moving (not idle or stalled)
     */
    private void updateLoadStatus() {
        if (currentLoadFactor < 1.2) {
            currentLoadStatus = LoadStatus.NO_LOAD;
        } else if (currentLoadFactor < NORMAL_LOAD_THRESHOLD) {
            currentLoadStatus = LoadStatus.NORMAL_LOAD;
        } else if (currentLoadFactor < HEAVY_LOAD_THRESHOLD) {
            currentLoadStatus = LoadStatus.HEAVY_LOAD;
        } else if (currentLoadFactor < 90.0) {  // Don't override STALLED status
            currentLoadStatus = LoadStatus.OVERLOAD;
        }
        // If load factor is extremely high (>90), leave as STALLED
    }

    /**
     * Reset all performance statistics
     */
    public void resetPerformanceStats() {
        speedHistory.clear();
        powerHistory.clear();
        loadFactorHistory.clear();

        averageSpeed = 0.0;
        peakSpeed = 0.0;
        averageLoadFactor = 1.0;
        peakLoadFactor = 1.0;
        averagePower = 0.0;
        totalEnergyUsed = 0.0;

        currentLoadStatus = LoadStatus.UNKNOWN;
    }

    // ========================================================================================
    // GETTERS - Performance Metrics
    // ========================================================================================

    public double getCurrentSpeed() { return currentSpeed; }
    public double getAverageSpeed() { return averageSpeed; }
    public double getPeakSpeed() { return peakSpeed; }

    public double getCurrentLoadFactor() { return currentLoadFactor; }
    public double getAverageLoadFactor() { return averageLoadFactor; }
    public double getPeakLoadFactor() { return peakLoadFactor; }

    public LoadStatus getCurrentLoadStatus() { return currentLoadStatus; }
    public String getCurrentDirection() { return currentDirection; }

    public double getCurrentPower() { return currentAppliedPower; }
    public double getAveragePower() { return averagePower; }
    public double getTotalEnergyUsed() { return totalEnergyUsed; }

    // Gravity PID getters
    public boolean isGravityPidActive() { return gravityPidActive; }
    public double getGravityCompensationPower() { return currentCompensationPower; }
    public int getHoldTargetPosition() { return holdTargetPosition; }
    public double getGravityPidIntegral() { return gravityPidIntegral; }

    // ========================================================================================
    // TELEMETRY
    // ========================================================================================

    /**
     * Add basic lift telemetry to display
     */
    public void addTelemetry() {
        if (!initialized) {
            telemetry.addLine("⚠ Lift: Not Initialized");
            return;
        }

        telemetry.addLine("=== LIFT SYSTEM ===");

        // Position info
        telemetry.addData("Position", "%d ticks (%.1f%%)",
            currentPosition, getPositionPercentage());

        // Motor synchronization status
        if (motorsDesynchronized) {
            telemetry.addData("Motor Sync", "⚠ DESYNCED by %d ticks", Math.abs(motorPositionDifference));
            telemetry.addData("L/R Positions", "%d / %d", leftMotorPosition, rightMotorPosition);
        } else {
            telemetry.addData("Motor Sync", "✓ OK");
        }

        // Status
        if (emergencyStop) {
            telemetry.addLine("⚠⚠ EMERGENCY STOP ACTIVE ⚠⚠");
        } else if (positionControlActive) {
            telemetry.addData("Mode", "AUTO → %d ticks", targetPosition);
        } else {
            telemetry.addData("Mode", "MANUAL");
        }

        // Power and Direction
        telemetry.addData("Power", "%.2f (%s)", currentAppliedPower, currentDirection);

        // Automatic gravity compensation status
        if (gravityPidActive) {
            telemetry.addData("Auto Hold", "ACTIVE (%.3f)", currentCompensationPower);
            int holdError = holdTargetPosition - currentPosition;
            telemetry.addData("Hold Error", "%d ticks", holdError);
        } else {
            telemetry.addData("Gravity Comp", "%.3f", currentCompensationPower);
        }

        // Drop detection warning
        if (unintendedDropDetected) {
            telemetry.addLine("⚠ Significant drop detected!");
        }

        // Load status with special handling for STALLED and IDLE
        if (currentLoadStatus == LoadStatus.STALLED) {
            telemetry.addData("Load", "⛔ STALLED - CHECK FOR JAM!");
        } else if (currentLoadStatus == LoadStatus.IDLE) {
            telemetry.addData("Load", "○ Idle");
        } else {
            telemetry.addData("Load", "%s %s (%.2fx)",
                currentLoadStatus.getSymbol(),
                currentLoadStatus.getDisplayName(),
                currentLoadFactor);
        }
    }

    /**
     * Add detailed performance telemetry
     */
    public void addDetailedTelemetry() {
        addTelemetry();

        telemetry.addLine();
        telemetry.addLine("=== MOTOR SYNCHRONIZATION ===");
        telemetry.addData("Left Position", "%d ticks", leftMotorPosition);
        telemetry.addData("Right Position", "%d ticks", rightMotorPosition);
        telemetry.addData("Difference", "%d ticks", motorPositionDifference);
        telemetry.addData("Status", motorsDesynchronized ? "⚠ OUT OF SYNC" : "✓ SYNCHRONIZED");
        if (motorsDesynchronized) {
            telemetry.addLine("Auto-correcting: " + (motorPositionDifference > 0 ? "Slowing LEFT" : "Slowing RIGHT"));
        }

        telemetry.addLine();
        telemetry.addLine("=== LIFT PERFORMANCE ===");
        telemetry.addData("Direction", currentDirection);

        // Speed metrics
        telemetry.addData("Speed", "%.0f ticks/sec", currentSpeed);
        telemetry.addData("Avg Speed", "%.0f ticks/sec", averageSpeed);
        telemetry.addData("Peak Speed", "%.0f ticks/sec", peakSpeed);

        // Load metrics with status
        telemetry.addData("Load Status", "%s %s",
            currentLoadStatus.getSymbol(), currentLoadStatus.getDisplayName());
        if (currentLoadStatus == LoadStatus.STALLED) {
            telemetry.addLine("⚠️ MOTOR STALLED - POSSIBLE JAM!");
        } else if (currentLoadStatus != LoadStatus.IDLE && currentLoadStatus != LoadStatus.UNKNOWN) {
            telemetry.addData("Load Factor", "%.2fx (avg: %.2fx)",
                currentLoadFactor, averageLoadFactor);
            telemetry.addData("Peak Load", "%.2fx", peakLoadFactor);
        }

        // Power metrics
        telemetry.addData("Avg Power", "%.2f", averagePower);
        telemetry.addData("Energy Used", "%.1f", totalEnergyUsed);

        // Gravity PID info
        telemetry.addLine();
        telemetry.addLine("=== AUTO GRAVITY PID ===");
        telemetry.addData("PID Active", gravityPidActive ? "YES" : "NO");
        telemetry.addData("Comp Power", "%.4f", currentCompensationPower);
        if (gravityPidActive) {
            telemetry.addData("Hold Target", "%d ticks", holdTargetPosition);
            telemetry.addData("PID Integral", "%.5f", gravityPidIntegral);
        }

        // Limits
        telemetry.addData("Limits", "%d to %d", MIN_POSITION, MAX_POSITION);
        telemetry.addData("At Limit", "%s | %s",
            isAtMinimum() ? "MIN" : "---",
            isAtMaximum() ? "MAX" : "---");
    }

    /**
     * Add calibration helper telemetry
     * Use this during setup to find proper constants
     */
    public void addCalibrationTelemetry() {
        telemetry.addLine("=== LIFT CALIBRATION ===");
        telemetry.addData("Position", "%d ticks", currentPosition);
        telemetry.addData("Speed", "%.0f ticks/sec", currentSpeed);
        telemetry.addData("Power", "%.2f", currentAppliedPower);
        telemetry.addData("Load Factor", "%.2fx", currentLoadFactor);
        telemetry.addLine();
        telemetry.addLine("=== AUTO GRAVITY PID ===");
        telemetry.addData("PID Active", gravityPidActive);
        telemetry.addData("Comp Power", "%.4f", currentCompensationPower);
        telemetry.addData("Hold Target", "%d", holdTargetPosition);
        telemetry.addData("Position Error", "%d", holdTargetPosition - currentPosition);
        telemetry.addLine();
        telemetry.addLine("CALIBRATION STEPS:");
        telemetry.addLine("1. Move to lowest → note position → set MIN_POSITION");
        telemetry.addLine("2. Move to highest → note position → set MAX_POSITION");
        telemetry.addLine("3. Remove load, run at 100% → time 1000 ticks → set NO_LOAD_SPEED");
        telemetry.addLine("4. Add load → check load factor → set LOAD_THRESHOLDS");
        telemetry.addLine("5. Gravity auto-tunes via PID - watch Comp Power stabilize!");
    }

    /**
     * Get a status summary string
     */
    public String getStatusString() {
        if (!initialized) return "Not Initialized";
        if (emergencyStop) return "EMERGENCY STOP";
        if (positionControlActive) return "AUTO";
        return "MANUAL";
    }

    // ========================================================================================
    // GAMEPAD VIBRATION FEEDBACK
    // ========================================================================================

    /**
     * Update vibration feedback based on current lift state
     * Call this in your main loop AFTER update()
     */
    public void updateVibrationFeedback() {
        if (!vibrationsEnabled || (gamepad1 == null && gamepad2 == null)) {
            return;
        }

        // Check for stall condition
        boolean isStalled = (currentLoadStatus == LoadStatus.STALLED);
        if (isStalled && !lastStallState) {
            vibrateStallWarning();
        }
        lastStallState = isStalled;

        // Check for motor desynchronization
        if (motorsDesynchronized && !lastDesyncState && Math.abs(motorPositionDifference) > MAX_MOTOR_DESYNC) {
            vibrateDesyncWarning();
        }
        lastDesyncState = motorsDesynchronized;

        // Check for soft limit zone
        boolean inSoftLimit = isInSoftLimitZone();
        if (inSoftLimit && !lastSoftLimitState) {
            vibrateSoftLimit();
        }
        lastSoftLimitState = inSoftLimit;

        // Check for hard limits
        if (currentPosition <= MIN_POSITION && currentAppliedPower < -0.1) {
            vibrateHardLimit();
        } else if (currentPosition >= MAX_POSITION && currentAppliedPower > 0.1) {
            vibrateHardLimit();
        }

        // Check for auto-position reached
        if (positionControlActive && Math.abs(currentPosition - targetPosition) < POSITION_TOLERANCE) {
            if (lastAutoPositionTarget != targetPosition) {
                vibratePositionReached();
                lastAutoPositionTarget = targetPosition;
            }
        } else {
            lastAutoPositionTarget = -1;
        }
    }

    /**
     * Vibrate for soft limit warning (entering soft zone)
     */
    private void vibrateSoftLimit() {
        if (!canVibrate()) return;
        vibrateGamepads(VIBRATION_SOFT_LIMIT);
        vibrationCooldownTimer.reset();
    }

    /**
     * Vibrate for hard limit warning (at absolute limit)
     * Double pulse pattern
     */
    private void vibrateHardLimit() {
        if (!canVibrate()) return;
        vibrateGamepads(VIBRATION_HARD_LIMIT_DURATION);
        // Note: Second pulse would need to be handled in a separate thread or with rumble effects
        vibrationCooldownTimer.reset();
    }

    /**
     * Vibrate for stall/jam warning
     * Long continuous vibration
     */
    private void vibrateStallWarning() {
        if (!canVibrate()) return;
        vibrateGamepads(VIBRATION_STALL_DURATION);
        vibrationCooldownTimer.reset();
    }

    /**
     * Vibrate for motor desynchronization warning
     * Triple pulse pattern
     */
    private void vibrateDesyncWarning() {
        if (!canVibrate()) return;
        vibrateGamepads(VIBRATION_DESYNC_PULSE);
        vibrationCooldownTimer.reset();
    }

    /**
     * Vibrate when auto-position is reached
     * Quick confirmation pulse
     */
    private void vibratePositionReached() {
        if (!canVibrate()) return;
        vibrateGamepads(VIBRATION_POSITION_REACHED);
        vibrationCooldownTimer.reset();
    }

    /**
     * Check if enough time has passed since last vibration
     */
    private boolean canVibrate() {
        return vibrationCooldownTimer.milliseconds() > VIBRATION_COOLDOWN_MS;
    }

    /**
     * Send vibration to connected gamepads
     * @param durationMs Duration of vibration in milliseconds
     */
    private void vibrateGamepads(int durationMs) {
        // Vibrate gamepad1 if available
        if (gamepad1 != null) {
            gamepad1.rumble(durationMs);
        }
        // Vibrate gamepad2 if available (for dual-driver mode)
        if (gamepad2 != null) {
            gamepad2.rumble(durationMs);
        }
    }

    /**
     * Send custom vibration pattern to gamepads
     * @param leftRumble Left motor intensity (0.0 to 1.0)
     * @param rightRumble Right motor intensity (0.0 to 1.0)
     * @param durationMs Duration in milliseconds
     */
    public void vibrateCustomPattern(double leftRumble, double rightRumble, int durationMs) {
        if (!vibrationsEnabled || feedbackManager == null) return;
        feedbackManager.rumbleCustom(leftRumble, rightRumble, durationMs);
    }

    /**
     * Stop all gamepad vibrations immediately
     */
    public void stopVibrations() {
        if (feedbackManager != null) {
            feedbackManager.stopAll();
        }
    }

    /**
     * Check if feedback is currently playing
     */
    public boolean isFeedbackPlaying() {
        return feedbackManager != null && feedbackManager.isPlaying();
    }

    /**
     * Get name of currently playing feedback
     */
    public String getCurrentFeedbackName() {
        return feedbackManager != null ? feedbackManager.getCurrentFeedbackName() : "None";
    }
}

