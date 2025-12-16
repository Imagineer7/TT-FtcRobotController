package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

/**
 * AURORA V2 Unified Hardware Configuration
 *
 * This class serves as the single source of truth for all hardware device mappings
 * in the AURORA system. It provides:
 * - Centralized hardware device name constants
 * - Automatic hardware initialization with error handling
 * - Consistent hardware configuration across all OpModes
 * - Easy maintenance - change device names in one place
 *
 * Usage:
 *   AuroraHardwareConfig hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
 *   hardware.initializeWithOdometry();  // For TeleOp
 *   // OR
 *   hardware.initialize();  // For Autonomous without odometry
 *
 *   DcMotor shooter = hardware.getShooterMotor();
 *   DcMotor frontLeft = hardware.getFrontLeftMotor();
 */
public class AuroraHardwareConfig {

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE DEVICE NAMES - Update these to match your Driver Station config
    // ═══════════════════════════════════════════════════════════════════════

    // Drive System Motors
    public static final String FRONT_LEFT_MOTOR = "frontLeft";
    public static final String FRONT_RIGHT_MOTOR = "frontRight";
    public static final String BACK_LEFT_MOTOR = "backLeft";
    public static final String BACK_RIGHT_MOTOR = "backRight";

    // Shooter System
    public static final String SHOOTER_MOTOR = "shooter";
    public static final String FEED_SERVO_1 = "servo1";
    public static final String FEED_SERVO_2 = "servo2";
    public static final String LIGHT_SERVO = "light";  // Optional

    // Intake and Indexing System
    public static final String FRONT_ROLLER_MOTOR = "frontRollerMotor";
    public static final String BACK_ROLLER_MOTOR = "backRollerMotor";
    public static final String FRONT_TRANSFER_SERVO = "frontTransferServo";
    public static final String BACK_TRANSFER_SERVO = "backTransferServo";
    public static final String TRANSFER_SERVO_CL = "transferServoCL";
    public static final String TRANSFER_SERVO_CR = "transferServoCR";
    
    // Artifact Detection Sensors
    public static final String FRONT_DISTANCE_SENSOR = "frontDist";
    public static final String BACK_DISTANCE_SENSOR = "backDist";
    public static final String FRONT_LEFT_COLOR_SENSOR = "frontLeftColor";
    public static final String FRONT_RIGHT_COLOR_SENSOR = "frontRightColor";
    public static final String BACK_RIGHT_COLOR_SENSOR = "backRightColor";
    public static final String LEFT_RIGHT_COLOR_SENSOR = "leftRightColor";
    public static final String FRONT_CENTER_COLOR_SENSOR = "frontCenterColor";
    public static final String BACK_CENTER_COLOR_SENSOR = "backCenterColor";

    // Sensors
    public static final String IMU_SENSOR = "imu";
    public static final String ODOMETRY_COMPUTER = "odo";

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE CONFIGURATION PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    // IMU Orientation Configuration
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // Odometry Pod Offsets (in inches from robot center)
    private static final double ODOMETRY_X_OFFSET = 4.71;   // Right from center
    private static final double ODOMETRY_Y_OFFSET = -6.62;  // Forward from center

    // Odometry Pod Directions
    private static final GoBildaPinpointDriver.EncoderDirection FORWARD_POD_DIRECTION =
        GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection STRAFE_POD_DIRECTION =
        GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE DEVICE INSTANCES
    // ═══════════════════════════════════════════════════════════════════════

    // Core Hardware
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Drive Motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Shooter System
    private DcMotor shooterMotor;
    private CRServo feedServo1;
    private CRServo feedServo2;
    private Servo lightServo;  // Optional

    // Intake and Indexing System
    private DcMotor frontRollerMotor;
    private DcMotor backRollerMotor;
    private Servo frontTransferServo;
    private Servo backTransferServo;
    private Servo transferServoCL;
    private Servo transferServoCR;
    
    // Artifact Detection Sensors
    // goBILDA Laser Distance Sensors (Analog Mode: 0-3.3V = 0-1000mm)
    private AnalogInput frontDistanceSensor;
    private AnalogInput backDistanceSensor;
    private ColorSensor frontLeftColorSensor;
    private ColorSensor frontRightColorSensor;
    private ColorSensor backRightColorSensor;
    private ColorSensor leftRightColorSensor;
    private ColorSensor frontCenterColorSensor;
    private ColorSensor backCenterColorSensor;
    
    // Distance sensor calibration constants (for goBILDA laser sensors in analog mode)
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    // Sensors
    private IMU imu;
    private GoBildaPinpointDriver odometry;
    private VoltageSensor voltageSensor;

    // Initialization Status
    private boolean driveSystemInitialized = false;
    private boolean shooterSystemInitialized = false;
    private boolean indexingSystemInitialized = false;
    private boolean imuInitialized = false;
    private boolean odometryInitialized = false;

    // Error Messages
    private String driveInitError = "";
    private String shooterInitError = "";
    private String indexingInitError = "";
    private String imuInitError = "";
    private String odometryInitError = "";

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create a new AuroraHardwareConfig instance
     * @param hardwareMap The OpMode's hardwareMap
     * @param telemetry The OpMode's telemetry
     */
    public AuroraHardwareConfig(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // INITIALIZATION METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Initialize all hardware systems WITHOUT odometry
     * Use this for Autonomous OpModes that don't need continuous position tracking
     */
    public void initialize() {
        telemetry.addLine("🤖 Initializing AURORA Hardware...");
        telemetry.update();

        initializeDriveSystem();
        initializeShooterSystem();
        initializeIndexingSystem();
        initializeIMU();
        initializeVoltageSensor();

        telemetry.addLine("✅ AURORA Hardware Initialization Complete");
        telemetry.addLine(getInitializationSummary());
        telemetry.update();
    }

    /**
     * Initialize all hardware systems WITH odometry
     * Use this for TeleOp OpModes that need position tracking
     */
    public void initializeWithOdometry() {
        telemetry.addLine("🤖 Initializing AURORA Hardware with Odometry...");
        telemetry.update();

        initializeDriveSystem();
        initializeShooterSystem();
        initializeIndexingSystem();
        initializeIMU();
        initializeOdometry();
        initializeVoltageSensor();

        telemetry.addLine("✅ AURORA Hardware Initialization Complete");
        telemetry.addLine(getInitializationSummary());
        telemetry.update();
    }

    /**
     * Initialize the drive system (4 mecanum motors)
     */
    private void initializeDriveSystem() {
        try {
            // Initialize motors
            frontLeftMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR);
            frontRightMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR);
            backLeftMotor = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR);
            backRightMotor = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR);

            // Set motor directions (typical mecanum configuration)
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            // Set zero power behavior
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set run mode
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            driveSystemInitialized = true;
            telemetry.addLine("  ✅ Drive System");

        } catch (Exception e) {
            driveSystemInitialized = false;
            driveInitError = e.getMessage();
            telemetry.addLine("  ❌ Drive System: " + driveInitError);
        }
    }

    /**
     * Initialize the shooter system (motor and servos)
     */
    private void initializeShooterSystem() {
        try {
            // Initialize shooter motor
            shooterMotor = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR);
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize feed servos
            feedServo1 = hardwareMap.get(CRServo.class, FEED_SERVO_1);
            feedServo2 = hardwareMap.get(CRServo.class, FEED_SERVO_2);

            // Initialize optional light servo
            try {
                lightServo = hardwareMap.get(Servo.class, LIGHT_SERVO);
            } catch (Exception e) {
                lightServo = null;
                telemetry.addLine("  ⚠️ Light servo not found (optional)");
            }

            shooterSystemInitialized = true;
            telemetry.addLine("  ✅ Shooter System");

        } catch (Exception e) {
            shooterSystemInitialized = false;
            shooterInitError = e.getMessage();
            telemetry.addLine("  ❌ Shooter System: " + shooterInitError);
        }
    }

    /**
     * Initialize the intake and indexing system
     */
    private void initializeIndexingSystem() {
        try {
            // Initialize roller motors
            frontRollerMotor = hardwareMap.get(DcMotor.class, FRONT_ROLLER_MOTOR);
            backRollerMotor = hardwareMap.get(DcMotor.class, BACK_ROLLER_MOTOR);

            // Set motor directions
            frontRollerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRollerMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            // Set zero power behavior
            frontRollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set run mode
            frontRollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize transfer servos
            frontTransferServo = hardwareMap.get(Servo.class, FRONT_TRANSFER_SERVO);
            backTransferServo = hardwareMap.get(Servo.class, BACK_TRANSFER_SERVO);
            transferServoCL = hardwareMap.get(Servo.class, TRANSFER_SERVO_CL);
            transferServoCR = hardwareMap.get(Servo.class, TRANSFER_SERVO_CR);

            // Initialize distance sensors (goBILDA Laser Distance Sensors in analog mode)
            // These sensors output 0-3.3V corresponding to 0-1000mm distance
            try {
                frontDistanceSensor = hardwareMap.get(AnalogInput.class, FRONT_DISTANCE_SENSOR);
            } catch (Exception e) {
                frontDistanceSensor = null;
                telemetry.addLine("  ⚠️ Front distance sensor not found (optional)");
            }

            try {
                backDistanceSensor = hardwareMap.get(AnalogInput.class, BACK_DISTANCE_SENSOR);
            } catch (Exception e) {
                backDistanceSensor = null;
                telemetry.addLine("  ⚠️ Back distance sensor not found (optional)");
            }

            // Initialize color sensors (optional, may not all be present)
            try {
                frontLeftColorSensor = hardwareMap.get(ColorSensor.class, FRONT_LEFT_COLOR_SENSOR);
            } catch (Exception e) {
                frontLeftColorSensor = null;
                telemetry.addLine("  ⚠️ Front left color sensor not found (optional)");
            }

            try {
                frontRightColorSensor = hardwareMap.get(ColorSensor.class, FRONT_RIGHT_COLOR_SENSOR);
            } catch (Exception e) {
                frontRightColorSensor = null;
                telemetry.addLine("  ⚠️ Front right color sensor not found (optional)");
            }

            try {
                backRightColorSensor = hardwareMap.get(ColorSensor.class, BACK_RIGHT_COLOR_SENSOR);
            } catch (Exception e) {
                backRightColorSensor = null;
                telemetry.addLine("  ⚠️ Back right color sensor not found (optional)");
            }

            try {
                leftRightColorSensor = hardwareMap.get(ColorSensor.class, LEFT_RIGHT_COLOR_SENSOR);
            } catch (Exception e) {
                leftRightColorSensor = null;
                telemetry.addLine("  ⚠️ Left right color sensor not found (optional)");
            }

            try {
                frontCenterColorSensor = hardwareMap.get(ColorSensor.class, FRONT_CENTER_COLOR_SENSOR);
            } catch (Exception e) {
                frontCenterColorSensor = null;
                telemetry.addLine("  ⚠️ Front center color sensor not found (optional)");
            }

            try {
                backCenterColorSensor = hardwareMap.get(ColorSensor.class, BACK_CENTER_COLOR_SENSOR);
            } catch (Exception e) {
                backCenterColorSensor = null;
                telemetry.addLine("  ⚠️ Back center color sensor not found (optional)");
            }

            indexingSystemInitialized = true;
            telemetry.addLine("  ✅ Indexing System");

        } catch (Exception e) {
            indexingSystemInitialized = false;
            indexingInitError = e.getMessage();
            telemetry.addLine("  ❌ Indexing System: " + indexingInitError);
        }
    }

    /**
     * Initialize the IMU sensor
     */
    private void initializeIMU() {
        try {
            imu = hardwareMap.get(IMU.class, IMU_SENSOR);

            // Configure IMU orientation
            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                IMU_LOGO_DIRECTION,
                IMU_USB_DIRECTION
            );

            IMU.Parameters imuParameters = new IMU.Parameters(orientation);
            imu.initialize(imuParameters);

            imuInitialized = true;
            telemetry.addLine("  ✅ IMU (Logo: " + IMU_LOGO_DIRECTION + ", USB: " + IMU_USB_DIRECTION + ")");

        } catch (Exception e) {
            imuInitialized = false;
            imuInitError = e.getMessage();
            telemetry.addLine("  ❌ IMU: " + imuInitError);
        }
    }

    /**
     * Initialize the odometry system (goBILDA Pinpoint)
     */
    private void initializeOdometry() {
        try {
            odometry = hardwareMap.get(GoBildaPinpointDriver.class, ODOMETRY_COMPUTER);

            // Configure odometry offsets (in inches)
            odometry.setOffsets(ODOMETRY_X_OFFSET, ODOMETRY_Y_OFFSET, DistanceUnit.INCH);

            // Configure encoder directions
            odometry.setEncoderDirections(FORWARD_POD_DIRECTION, STRAFE_POD_DIRECTION);

            // Reset position
            odometry.resetPosAndIMU();

            odometryInitialized = true;
            telemetry.addLine("  ✅ Odometry (Pinpoint)");

        } catch (Exception e) {
            odometryInitialized = false;
            odometryInitError = e.getMessage();
            telemetry.addLine("  ⚠️ Odometry: " + odometryInitError);
        }
    }

    /**
     * Initialize voltage sensor (always available on control hub)
     */
    private void initializeVoltageSensor() {
        try {
            // Get voltage sensor from the hardware map
            // The control hub always has a voltage sensor
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                voltageSensor = sensor;
                break;
            }
        } catch (Exception e) {
            voltageSensor = null;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE ACCESSORS
    // ═══════════════════════════════════════════════════════════════════════

    // Drive Motors
    public DcMotor getFrontLeftMotor() { return frontLeftMotor; }
    public DcMotor getFrontRightMotor() { return frontRightMotor; }
    public DcMotor getBackLeftMotor() { return backLeftMotor; }
    public DcMotor getBackRightMotor() { return backRightMotor; }

    // Shooter System
    public DcMotor getShooterMotor() { return shooterMotor; }
    public CRServo getFeedServo1() { return feedServo1; }
    public CRServo getFeedServo2() { return feedServo2; }
    public Servo getLightServo() { return lightServo; }  // May be null

    // Intake and Indexing System
    public DcMotor getFrontRollerMotor() { return frontRollerMotor; }
    public DcMotor getBackRollerMotor() { return backRollerMotor; }
    public Servo getFrontTransferServo() { return frontTransferServo; }
    public Servo getBackTransferServo() { return backTransferServo; }
    public Servo getTransferServoCL() { return transferServoCL; }
    public Servo getTransferServoCR() { return transferServoCR; }
    
    // Artifact Detection Sensors
    public AnalogInput getFrontDistanceSensor() { return frontDistanceSensor; }
    public AnalogInput getBackDistanceSensor() { return backDistanceSensor; }
    
    /**
     * Get distance reading from front sensor in millimeters
     * Converts analog voltage (0-3.3V) to distance (0-1000mm)
     * @return Distance in mm, or -1 if sensor not available
     */
    public double getFrontDistanceMM() {
        if (frontDistanceSensor == null) return -1;
        double volts = frontDistanceSensor.getVoltage();
        return (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
    }
    
    /**
     * Get distance reading from back sensor in millimeters
     * Converts analog voltage (0-3.3V) to distance (0-1000mm)
     * @return Distance in mm, or -1 if sensor not available
     */
    public double getBackDistanceMM() {
        if (backDistanceSensor == null) return -1;
        double volts = backDistanceSensor.getVoltage();
        return (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
    }
    
    public ColorSensor getFrontLeftColorSensor() { return frontLeftColorSensor; }
    public ColorSensor getFrontRightColorSensor() { return frontRightColorSensor; }
    public ColorSensor getBackRightColorSensor() { return backRightColorSensor; }
    public ColorSensor getLeftRightColorSensor() { return leftRightColorSensor; }
    public ColorSensor getFrontCenterColorSensor() { return frontCenterColorSensor; }
    public ColorSensor getBackCenterColorSensor() { return backCenterColorSensor; }

    // Sensors
    public IMU getIMU() { return imu; }
    public GoBildaPinpointDriver getOdometry() { return odometry; }
    public VoltageSensor getVoltageSensor() { return voltageSensor; }

    // Core
    public HardwareMap getHardwareMap() { return hardwareMap; }

    // ═══════════════════════════════════════════════════════════════════════
    // STATUS CHECKERS
    // ═══════════════════════════════════════════════════════════════════════

    public boolean isDriveSystemInitialized() { return driveSystemInitialized; }
    public boolean isShooterSystemInitialized() { return shooterSystemInitialized; }
    public boolean isIndexingSystemInitialized() { return indexingSystemInitialized; }
    public boolean isIMUInitialized() { return imuInitialized; }
    public boolean isOdometryInitialized() { return odometryInitialized; }

    public String getDriveInitError() { return driveInitError; }
    public String getShooterInitError() { return shooterInitError; }
    public String getIndexingInitError() { return indexingInitError; }
    public String getIMUInitError() { return imuInitError; }
    public String getOdometryInitError() { return odometryInitError; }

    /**
     * Check if all critical systems are operational
     */
    public boolean isSystemHealthy() {
        return driveSystemInitialized && shooterSystemInitialized && indexingSystemInitialized && imuInitialized;
    }

    /**
     * Get a summary of initialization status
     */
    public String getInitializationSummary() {
        StringBuilder summary = new StringBuilder();
        summary.append("\n📊 Hardware Status:\n");
        summary.append("  Drive: ").append(driveSystemInitialized ? "✅" : "❌").append("\n");
        summary.append("  Shooter: ").append(shooterSystemInitialized ? "✅" : "❌").append("\n");
        summary.append("  Indexing: ").append(indexingSystemInitialized ? "✅" : "❌").append("\n");
        summary.append("  IMU: ").append(imuInitialized ? "✅" : "❌").append("\n");
        summary.append("  Odometry: ").append(odometryInitialized ? "✅" : "⚠️").append("\n");
        return summary.toString();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Reset the IMU heading to zero
     */
    public void resetIMUHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    /**
     * Reset odometry position to (0, 0) with heading 0
     */
    public void resetOdometryPosition() {
        if (odometry != null) {
            odometry.resetPosAndIMU();
        }
    }

    /**
     * Get current battery voltage
     */
    public double getBatteryVoltage() {
        if (voltageSensor != null) {
            return voltageSensor.getVoltage();
        }
        return 12.0;  // Default fallback
    }

    /**
     * Stop all motors (emergency stop)
     */
    public void stopAllMotors() {
        if (frontLeftMotor != null) frontLeftMotor.setPower(0);
        if (frontRightMotor != null) frontRightMotor.setPower(0);
        if (backLeftMotor != null) backLeftMotor.setPower(0);
        if (backRightMotor != null) backRightMotor.setPower(0);
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (feedServo1 != null) feedServo1.setPower(0);
        if (feedServo2 != null) feedServo2.setPower(0);
        if (frontRollerMotor != null) frontRollerMotor.setPower(0);
        if (backRollerMotor != null) backRollerMotor.setPower(0);
    }
}
