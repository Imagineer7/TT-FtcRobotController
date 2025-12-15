package org.firstinspires.ftc.teamcode.util.aurora;

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
    public static final String FRONT_INTAKE_MOTOR = "frontIntake";
    public static final String BACK_INTAKE_MOTOR = "backIntake";
    public static final String CENTER_ROLLER_MOTOR = "centerRoller";
    public static final String TRANSFER_SERVO = "transferServo";
    
    // Artifact Detection Sensors
    public static final String FRONT_DISTANCE_SENSOR = "frontDistance";
    public static final String BACK_DISTANCE_SENSOR = "backDistance";
    public static final String CENTER_DISTANCE_SENSOR = "centerDistance";
    public static final String FRONT_COLOR_SENSOR = "frontColor";
    public static final String BACK_COLOR_SENSOR = "backColor";
    public static final String CENTER_COLOR_SENSOR = "centerColor";

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
    private DcMotor frontIntakeMotor;
    private DcMotor backIntakeMotor;
    private DcMotor centerRollerMotor;
    private Servo transferServo;
    
    // Artifact Detection Sensors
    private DistanceSensor frontDistanceSensor;
    private DistanceSensor backDistanceSensor;
    private DistanceSensor centerDistanceSensor;
    private ColorSensor frontColorSensor;
    private ColorSensor backColorSensor;
    private ColorSensor centerColorSensor;

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
            // Initialize intake motors
            frontIntakeMotor = hardwareMap.get(DcMotor.class, FRONT_INTAKE_MOTOR);
            backIntakeMotor = hardwareMap.get(DcMotor.class, BACK_INTAKE_MOTOR);
            centerRollerMotor = hardwareMap.get(DcMotor.class, CENTER_ROLLER_MOTOR);

            // Set motor directions
            frontIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            centerRollerMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            // Set zero power behavior
            frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            centerRollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set run mode
            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            centerRollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize transfer servo
            transferServo = hardwareMap.get(Servo.class, TRANSFER_SERVO);

            // Initialize distance sensors (optional, may not all be present)
            try {
                frontDistanceSensor = hardwareMap.get(DistanceSensor.class, FRONT_DISTANCE_SENSOR);
            } catch (Exception e) {
                frontDistanceSensor = null;
                telemetry.addLine("  ⚠️ Front distance sensor not found (optional)");
            }

            try {
                backDistanceSensor = hardwareMap.get(DistanceSensor.class, BACK_DISTANCE_SENSOR);
            } catch (Exception e) {
                backDistanceSensor = null;
                telemetry.addLine("  ⚠️ Back distance sensor not found (optional)");
            }

            try {
                centerDistanceSensor = hardwareMap.get(DistanceSensor.class, CENTER_DISTANCE_SENSOR);
            } catch (Exception e) {
                centerDistanceSensor = null;
                telemetry.addLine("  ⚠️ Center distance sensor not found (optional)");
            }

            // Initialize color sensors (optional, may not all be present)
            try {
                frontColorSensor = hardwareMap.get(ColorSensor.class, FRONT_COLOR_SENSOR);
            } catch (Exception e) {
                frontColorSensor = null;
                telemetry.addLine("  ⚠️ Front color sensor not found (optional)");
            }

            try {
                backColorSensor = hardwareMap.get(ColorSensor.class, BACK_COLOR_SENSOR);
            } catch (Exception e) {
                backColorSensor = null;
                telemetry.addLine("  ⚠️ Back color sensor not found (optional)");
            }

            try {
                centerColorSensor = hardwareMap.get(ColorSensor.class, CENTER_COLOR_SENSOR);
            } catch (Exception e) {
                centerColorSensor = null;
                telemetry.addLine("  ⚠️ Center color sensor not found (optional)");
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
    public DcMotor getFrontIntakeMotor() { return frontIntakeMotor; }
    public DcMotor getBackIntakeMotor() { return backIntakeMotor; }
    public DcMotor getCenterRollerMotor() { return centerRollerMotor; }
    public Servo getTransferServo() { return transferServo; }
    
    // Artifact Detection Sensors
    public DistanceSensor getFrontDistanceSensor() { return frontDistanceSensor; }
    public DistanceSensor getBackDistanceSensor() { return backDistanceSensor; }
    public DistanceSensor getCenterDistanceSensor() { return centerDistanceSensor; }
    public ColorSensor getFrontColorSensor() { return frontColorSensor; }
    public ColorSensor getBackColorSensor() { return backColorSensor; }
    public ColorSensor getCenterColorSensor() { return centerColorSensor; }

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
        if (frontIntakeMotor != null) frontIntakeMotor.setPower(0);
        if (backIntakeMotor != null) backIntakeMotor.setPower(0);
        if (centerRollerMotor != null) centerRollerMotor.setPower(0);
    }
}
