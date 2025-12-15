package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * IntelMechanumDrive - Advanced Mecanum Drive Controller for AURORA System
 *
 * This class provides intelligent mecanum drive control with the following features:
 * - Robot-centric and field-centric driving modes
 * - Smooth acceleration and deceleration
 * - IMU integration for orientation-based driving
 * - Power scaling and speed modes
 * - Compatible with AuroraHardwareConfig for easy integration
 *
 * Design Philosophy:
 * - Standalone operation with optional AuroraHardwareConfig integration
 * - No circular dependencies within AURORA package
 * - Easy plug-and-play setup for any mecanum drive robot
 *
 * References org.firstinspires.ftc.teamcode.util.MechanumDrive for baseline implementation
 */
public class IntelMechanumDrive {

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTANTS
    // ═══════════════════════════════════════════════════════════════════════

    private static final double SLOW_MODE_SCALE = 0.33;  // 33% power in slow mode
    private static final double NORMAL_MODE_SCALE = 1.0;  // 100% power in normal mode
    private static final double TURBO_MODE_SCALE = 1.0;   // 100% power in turbo mode (can be increased if needed)

    private static final double DPAD_MOVE_SPEED = 0.4;  // Speed for D-pad fine control

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE COMPONENTS
    // ═══════════════════════════════════════════════════════════════════════

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private IMU imu;

    private Gamepad gamepad;
    private GamepadConfig gamepadConfig;  // Optional: Use GamepadConfig for standardized controls

    // ═══════════════════════════════════════════════════════════════════════
    // DRIVE STATE
    // ═══════════════════════════════════════════════════════════════════════

    private DriveMode driveMode = DriveMode.ROBOT_CENTRIC;
    private SpeedMode speedMode = SpeedMode.NORMAL;

    // Motor power values
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double backLeftPower = 0;
    private double backRightPower = 0;

    // Input values
    private double axial = 0;      // Forward/backward
    private double lateral = 0;    // Strafe left/right
    private double yaw = 0;        // Rotation

    // ═══════════════════════════════════════════════════════════════════════
    // ENUMS
    // ═══════════════════════════════════════════════════════════════════════

    public enum DriveMode {
        ROBOT_CENTRIC,    // Standard mecanum drive relative to robot
        FIELD_CENTRIC     // Drive relative to field using IMU
    }

    public enum SpeedMode {
        SLOW,      // 33% power for precise control
        NORMAL,    // 100% power for standard operation
        TURBO      // 100% power (or higher if configured)
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTORS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Constructor using AuroraHardwareConfig (Recommended)
     *
     * @param config Initialized AuroraHardwareConfig instance
     * @param gamepad Gamepad for driver control
     */
    public IntelMechanumDrive(AuroraHardwareConfig config, Gamepad gamepad) {
        this.frontLeftMotor = config.getFrontLeftMotor();
        this.frontRightMotor = config.getFrontRightMotor();
        this.backLeftMotor = config.getBackLeftMotor();
        this.backRightMotor = config.getBackRightMotor();
        this.imu = config.getIMU();
        this.gamepad = gamepad;
        this.gamepadConfig = null;

        configureMotors();
    }

    /**
     * Constructor using AuroraHardwareConfig with GamepadConfig (Recommended for standardized controls)
     *
     * @param config Initialized AuroraHardwareConfig instance
     * @param gamepadConfig GamepadConfig for standardized driver controls
     */
    public IntelMechanumDrive(AuroraHardwareConfig config, GamepadConfig gamepadConfig) {
        this.frontLeftMotor = config.getFrontLeftMotor();
        this.frontRightMotor = config.getFrontRightMotor();
        this.backLeftMotor = config.getBackLeftMotor();
        this.backRightMotor = config.getBackRightMotor();
        this.imu = config.getIMU();
        this.gamepadConfig = gamepadConfig;
        this.gamepad = gamepadConfig.getDriverGamepad();  // Fallback for legacy code

        configureMotors();
    }

    /**
     * Constructor using individual motor references (Backup/Standalone mode)
     *
     * @param frontLeft Front left motor
     * @param frontRight Front right motor
     * @param backLeft Back left motor
     * @param backRight Back right motor
     * @param imu IMU sensor (can be null if field-centric is not needed)
     * @param gamepad Gamepad for driver control
     */
    public IntelMechanumDrive(DcMotor frontLeft, DcMotor frontRight,
                              DcMotor backLeft, DcMotor backRight,
                              IMU imu, Gamepad gamepad) {
        this.frontLeftMotor = frontLeft;
        this.frontRightMotor = frontRight;
        this.backLeftMotor = backLeft;
        this.backRightMotor = backRight;
        this.imu = imu;
        this.gamepad = gamepad;

        configureMotors();
    }

    /**
     * Configure motor directions and behavior
     * Based on AuroraHardwareConfig and MechanumDrive implementations
     */
    private void configureMotors() {
        // Set motor directions (matching AuroraHardwareConfig configuration)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to BRAKE for better control
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders for direct power control
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MAIN DRIVE METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Main drive method - call this in your OpMode loop
     * Reads gamepad inputs and calculates motor powers
     */
    public void drive() {
        readInputs();
        calculateMotorPowers();
        applyMotorPowers();
    }

    /**
     * Read gamepad inputs and process them
     */
    private void readInputs() {
        // Use GamepadConfig if available, otherwise use direct gamepad access
        if (gamepadConfig != null) {
            // Read inputs from GamepadConfig (standardized controls)
            axial = gamepadConfig.getAxial();
            lateral = gamepadConfig.getLateral();
            yaw = gamepadConfig.getYaw();

            // Apply speed scaling
            applySpeedScaling();

        } else {
            // Fallback to legacy direct gamepad reading
            // Check for analog stick input (left stick for forward/backward, triggers for strafe, right stick for rotation)
            if (hasAnalogInput()) {
                // Forward/backward from left stick Y
                axial = gamepad.left_stick_y;

                // Strafe from triggers (right trigger = right, left trigger = left)
                lateral = -(gamepad.right_trigger - gamepad.left_trigger);

                // Rotation from right stick X
                yaw = -gamepad.right_stick_x;

                // Apply speed scaling
                applySpeedScaling();

            } else {
                // Fall back to D-pad for fine control
                processDpadInput();
                yaw = 0;  // No rotation from D-pad
            }
        }
    }

    /**
     * Check if gamepad has analog input
     */
    private boolean hasAnalogInput() {
        return (gamepad.left_stick_y != 0) ||
               (gamepad.right_stick_x != 0) ||
               (gamepad.right_trigger != 0) ||
               (gamepad.left_trigger != 0);
    }

    /**
     * Process D-pad input for fine control
     */
    private void processDpadInput() {
        // Forward/backward
        if (gamepad.dpad_up) {
            axial = -DPAD_MOVE_SPEED;
        } else if (gamepad.dpad_down) {
            axial = DPAD_MOVE_SPEED;
        } else {
            axial = 0;
        }

        // Strafe left/right
        if (gamepad.dpad_right) {
            lateral = -DPAD_MOVE_SPEED;
        } else if (gamepad.dpad_left) {
            lateral = DPAD_MOVE_SPEED;
        } else {
            lateral = 0;
        }
    }

    /**
     * Apply speed mode scaling to inputs
     */
    private void applySpeedScaling() {
        double scale = getSpeedScale();
        axial *= scale;
        lateral *= scale;
        yaw *= scale;
    }

    /**
     * Get the speed scale factor based on current speed mode
     */
    private double getSpeedScale() {
        switch (speedMode) {
            case SLOW:
                return SLOW_MODE_SCALE;
            case TURBO:
                return TURBO_MODE_SCALE;
            case NORMAL:
            default:
                return NORMAL_MODE_SCALE;
        }
    }

    /**
     * Calculate motor powers based on drive mode
     */
    private void calculateMotorPowers() {
        if (driveMode == DriveMode.FIELD_CENTRIC && imu != null) {
            calculateFieldCentricPowers();
        } else {
            calculateRobotCentricPowers();
        }

        // Normalize powers to ensure no value exceeds 1.0
        normalizePowers();
    }

    /**
     * Calculate robot-centric motor powers (standard mecanum)
     * Formula based on org.firstinspires.ftc.teamcode.util.MechanumDrive
     */
    private void calculateRobotCentricPowers() {
        frontLeftPower = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower = axial - lateral + yaw;
        backRightPower = axial + lateral - yaw;
    }

    /**
     * Calculate field-centric motor powers using IMU heading
     */
    private void calculateFieldCentricPowers() {
        // Get robot heading from IMU
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate input vector by heading angle
        double rotatedAxial = axial * Math.cos(-heading) - lateral * Math.sin(-heading);
        double rotatedLateral = axial * Math.sin(-heading) + lateral * Math.cos(-heading);

        // Calculate powers using rotated inputs
        frontLeftPower = rotatedAxial + rotatedLateral + yaw;
        frontRightPower = rotatedAxial - rotatedLateral - yaw;
        backLeftPower = rotatedAxial - rotatedLateral + yaw;
        backRightPower = rotatedAxial + rotatedLateral - yaw;
    }

    /**
     * Normalize motor powers to ensure no value exceeds 1.0
     */
    private void normalizePowers() {
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }
    }

    /**
     * Apply calculated motor powers to physical motors
     */
    private void applyMotorPowers() {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MODE CONTROL METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Toggle between robot-centric and field-centric modes
     */
    public void toggleDriveMode() {
        driveMode = (driveMode == DriveMode.ROBOT_CENTRIC) ?
                    DriveMode.FIELD_CENTRIC : DriveMode.ROBOT_CENTRIC;
    }

    /**
     * Set drive mode explicitly
     */
    public void setDriveMode(DriveMode mode) {
        this.driveMode = mode;
    }

    /**
     * Cycle through speed modes: NORMAL -> SLOW -> TURBO -> NORMAL
     */
    public void cycleSpeedMode() {
        switch (speedMode) {
            case NORMAL:
                speedMode = SpeedMode.SLOW;
                break;
            case SLOW:
                speedMode = SpeedMode.TURBO;
                break;
            case TURBO:
                speedMode = SpeedMode.NORMAL;
                break;
        }
    }

    /**
     * Toggle between normal and slow speed modes
     */
    public void toggleSlowMode() {
        speedMode = (speedMode == SpeedMode.SLOW) ? SpeedMode.NORMAL : SpeedMode.SLOW;
    }

    /**
     * Set speed mode explicitly
     */
    public void setSpeedMode(SpeedMode mode) {
        this.speedMode = mode;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Stop all motors immediately
     */
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * Set motor powers directly (for autonomous or advanced control)
     *
     * @param forward Forward/backward power (-1.0 to 1.0)
     * @param strafe Left/right power (-1.0 to 1.0)
     * @param turn Rotation power (-1.0 to 1.0)
     */
    public void setMechanumPowers(double forward, double strafe, double turn) {
        axial = forward;
        lateral = strafe;
        yaw = turn;

        calculateMotorPowers();
        applyMotorPowers();
    }

    /**
     * Reset IMU heading (for field-centric mode)
     */
    public void resetHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // GETTERS
    // ═══════════════════════════════════════════════════════════════════════

    public DriveMode getDriveMode() { return driveMode; }
    public SpeedMode getSpeedMode() { return speedMode; }

    public double getFrontLeftPower() { return frontLeftPower; }
    public double getFrontRightPower() { return frontRightPower; }
    public double getBackLeftPower() { return backLeftPower; }
    public double getBackRightPower() { return backRightPower; }

    public double getCurrentHeading() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        return 0;
    }
}
