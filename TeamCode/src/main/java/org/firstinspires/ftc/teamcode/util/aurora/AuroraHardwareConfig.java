package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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
    public static final String FRONT_LEFT_MOTOR = "Left Front";
    public static final String FRONT_RIGHT_MOTOR = "Right Front";
    public static final String BACK_LEFT_MOTOR = "Left Back";
    public static final String BACK_RIGHT_MOTOR = "Right Back";

    // Shooter System
    public static final String LEFT_SHOOTER_MOTOR = "Shooter Front";
    public static final String RIGHT_SHOOTER_MOTOR = "Shooter Back";
    public static final String LIGHT_SERVO = "RGB Light Back";  // Optional

    // Turret System
    public static final String TURRET_SERVO = "Turret Left";

    // Intake and Indexing System
    public static final String FRONT_ROLLER_MOTOR = "TopIntakeFront";
    public static final String BACK_ROLLER_MOTOR = "TopIntakeBack";
    public static final String FRONT_BOTTOM_INTAKE_SERVO = "BottomIntakeFront";  // Assists front roller (opposite direction)
    public static final String BACK_BOTTOM_INTAKE_SERVO = "BottomIntakeBack";    // Assists back roller (opposite direction)

    // Transfer System Servos
    public static final String FRONT_TRANSFER_SERVO = "TransferSystemFront";
    public static final String BACK_TRANSFER_SERVO = "TransferSystemBack";

    // Uptake System Servos (feed artifacts from center slot up into shooter)
    public static final String UPTAKE_SERVO_L = "UptakeTransferLeft";
    public static final String UPTAKE_SERVO_R = "UptakeTransferRight";

    // Injector System (moves artifacts between transfer system and center storage)
    public static final String INJECTOR_SERVO_LEFT = "InjectorSystemLeft";
    public static final String INJECTOR_SERVO_RIGHT = "InjectorSystemRight";

    // Artifact Detection Sensors
    public static final String FRONT_DISTANCE_SENSOR = "LaserSensorFront";
    public static final String BACK_DISTANCE_SENSOR = "LaserSensorBack";

    // NEW: REV 2m Distance Sensors for enhanced artifact detection
    public static final String FRONT_LEFT_DISTANCE_SENSOR = "DistSensorLeftFront";
    public static final String BACK_RIGHT_DISTANCE_SENSOR = "DistSensorRightBack";

    public static final String FRONT_LEFT_COLOR_SENSOR = "ColorSensorLeftFront";
    public static final String FRONT_RIGHT_COLOR_SENSOR = "ColorSensorRightFront";
    public static final String BACK_RIGHT_COLOR_SENSOR = "ColorSensorRightBack";
    public static final String LEFT_RIGHT_COLOR_SENSOR = "ColorSensorLeftBack";
    public static final String FRONT_CENTER_COLOR_SENSOR = "ColorSensorFront";
    public static final String BACK_CENTER_COLOR_SENSOR = "ColorSensorBack";

    // Sensors
    public static final String IMU_SENSOR = "imu";
    public static final String ODOMETRY_COMPUTER = "odo";

    // ═══════════════════════════════════════════════════════════════════════
    // HARDWARE CONFIGURATION PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════

    // IMU Orientation Configuration
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
        RevHubOrientationOnRobot.UsbFacingDirection.UP;

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
    private DcMotor leftShooterMotor;
    private DcMotor rightShooterMotor;
    private Servo lightServo;  // Optional

    // Turret System
    private Servo turretServo;  // Can also be CRServo depending on Turret mode

    // Intake and Indexing System
    private DcMotor frontRollerMotor;
    private DcMotor backRollerMotor;
    private CRServo frontBottomIntakeServo;  // Assists front roller (opposite direction)
    private CRServo backBottomIntakeServo;   // Assists back roller (opposite direction)
    private CRServo frontTransferServo;
    private CRServo backTransferServo;
    private CRServo uptakeServoL;            // Feeds artifacts from center up into shooter
    private CRServo uptakeServoR;            // Feeds artifacts from center up into shooter
    private CRServo injectorServoLeft;       // Moves artifacts between transfer and center storage
    private CRServo injectorServoRight;      // Moves artifacts between transfer and center storage

    // Artifact Detection Sensors
    // goBILDA Laser Distance Sensors (Analog Mode: 0-3.3V = 0-1000mm)
    private AnalogInput frontDistanceSensor;
    private AnalogInput backDistanceSensor;

    // NEW: REV 2m Distance Sensors for enhanced artifact detection
    // These face parallel with intake rollers, read ~25cm unless artifact present
    private DistanceSensor frontLeftDistanceSensor;
    private DistanceSensor backRightDistanceSensor;

    // REV Color Sensor V3 (Normalized RGB values 0-1)
    private NormalizedColorSensor frontLeftColorSensor;   // NOTE: Replaced with distance sensor
    private NormalizedColorSensor frontRightColorSensor;
    private NormalizedColorSensor backRightColorSensor;   // NOTE: Replaced with distance sensor
    private NormalizedColorSensor leftRightColorSensor;
    private NormalizedColorSensor frontCenterColorSensor;
    private NormalizedColorSensor backCenterColorSensor;
    
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
    private boolean turretSystemInitialized = false;
    private boolean indexingSystemInitialized = false;
    private boolean imuInitialized = false;
    private boolean odometryInitialized = false;

    // Error Messages
    private String driveInitError = "";
    private String shooterInitError = "";
    private String turretInitError = "";
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
        initializeTurretSystem();
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
        initializeTurretSystem();
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
     * Initialize the shooter system (dual motors and servos)
     */
    private void initializeShooterSystem() {
        try {
            // Initialize shooter motors
            leftShooterMotor = hardwareMap.get(DcMotor.class, LEFT_SHOOTER_MOTOR);
            rightShooterMotor = hardwareMap.get(DcMotor.class, RIGHT_SHOOTER_MOTOR);

            // STOP MOTORS IMMEDIATELY to prevent auto-start
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);

            // Note: Motor directions are configured by DecodeHelper using ShooterConfig
            // Do not set directions here to avoid conflicts

            leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Initialize optional light servo
            try {
                lightServo = hardwareMap.get(Servo.class, LIGHT_SERVO);
            } catch (Exception e) {
                lightServo = null;
                telemetry.addLine("  ⚠️ Light servo not found (optional)");
            }

            shooterSystemInitialized = true;
            telemetry.addLine("  ✅ Shooter System (Dual Motor)");

        } catch (Exception e) {
            shooterSystemInitialized = false;
            shooterInitError = e.getMessage();
            telemetry.addLine("  ❌ Shooter System: " + shooterInitError);
        }
    }

    /**
     * Initialize the turret system (servo)
     * Note: The Turret class handles the servo as either position or continuous rotation
     */
    private void initializeTurretSystem() {
        try {
            // Try to initialize as position servo first (most common)
            // The Turret class will handle it based on its SERVO_MODE configuration
            turretServo = hardwareMap.get(Servo.class, TURRET_SERVO);

            turretSystemInitialized = true;
            telemetry.addLine("  ✅ Turret System");

        } catch (Exception e) {
            // If position servo fails, turret might be configured as CRServo
            // The Turret class will handle the actual hardware access
            turretSystemInitialized = false;
            turretInitError = e.getMessage();
            telemetry.addLine("  ⚠️ Turret System: Not configured or " + turretInitError);
        }
    }

    /**
     * Initialize the intake and indexing system
     */
    private void initializeIndexingSystem() {
        boolean motorsAndServosOk = true;

        try {
            // Initialize roller motors
            frontRollerMotor = hardwareMap.get(DcMotor.class, FRONT_ROLLER_MOTOR);
            backRollerMotor = hardwareMap.get(DcMotor.class, BACK_ROLLER_MOTOR);

            // Set motor directions - REVERSE to flip intake direction
            frontRollerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRollerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // Set zero power behavior
            frontRollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set run mode
            frontRollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize bottom intake assist servos (run opposite direction of main rollers)
            frontBottomIntakeServo = hardwareMap.get(CRServo.class, FRONT_BOTTOM_INTAKE_SERVO);
            backBottomIntakeServo = hardwareMap.get(CRServo.class, BACK_BOTTOM_INTAKE_SERVO);

            // Initialize transfer servos (CRServos - Continuous Rotation)
            frontTransferServo = hardwareMap.get(CRServo.class, FRONT_TRANSFER_SERVO);
            backTransferServo = hardwareMap.get(CRServo.class, BACK_TRANSFER_SERVO);

            // Initialize uptake servos (feed artifacts from center up into shooter)
            uptakeServoL = hardwareMap.get(CRServo.class, UPTAKE_SERVO_L);
            uptakeServoR = hardwareMap.get(CRServo.class, UPTAKE_SERVO_R);

            // Initialize injector servos (move artifacts between transfer system and center storage)
            injectorServoLeft = hardwareMap.get(CRServo.class, INJECTOR_SERVO_LEFT);
            injectorServoRight = hardwareMap.get(CRServo.class, INJECTOR_SERVO_RIGHT);


        } catch (Exception e) {
            motorsAndServosOk = false;
            indexingInitError = e.getMessage();
            telemetry.addLine("  ❌ Indexing motors/servos: " + indexingInitError);
            telemetry.addLine("  Check Driver Station config for:");
            telemetry.addLine("    TopIntakeFront, TopIntakeBack (motors)");
            telemetry.addLine("    TransferSystemFront, TransferSystemBack");
            telemetry.addLine("    UptakeTransferLeft, UptakeTransferRight");
            telemetry.addLine("    InjectorSystemLeft, InjectorSystemRight");
            telemetry.addLine("    BottomIntakeFront, BottomIntakeBack");
        }

        // Initialize distance sensors (goBILDA Laser Distance Sensors in analog mode)
        // These sensors output 0-3.3V corresponding to 0-1000mm distance
        // SEPARATE from motor/servo initialization so sensor failures don't break everything
        try {
            frontDistanceSensor = hardwareMap.get(AnalogInput.class, FRONT_DISTANCE_SENSOR);
            telemetry.addLine("  ✅ Front distance sensor");
        } catch (Exception e) {
            frontDistanceSensor = null;
            telemetry.addLine("  ⚠️ Front distance sensor: " + e.getMessage());
        }

        try {
            backDistanceSensor = hardwareMap.get(AnalogInput.class, BACK_DISTANCE_SENSOR);
            telemetry.addLine("  ✅ Back distance sensor");
        } catch (Exception e) {
            backDistanceSensor = null;
            telemetry.addLine("  ⚠️ Back distance sensor: " + e.getMessage());
        }

        // Initialize NEW REV 2m Distance Sensors for enhanced artifact detection
        // These sensors face parallel with intake rollers and provide ~25cm baseline reading
        try {
            frontLeftDistanceSensor = hardwareMap.get(DistanceSensor.class, FRONT_LEFT_DISTANCE_SENSOR);
            telemetry.addLine("  ✅ Front left REV 2m distance sensor");
        } catch (Exception e) {
            frontLeftDistanceSensor = null;
            telemetry.addLine("  ⚠️ Front left REV 2m distance sensor: " + e.getMessage());
        }

        try {
            backRightDistanceSensor = hardwareMap.get(DistanceSensor.class, BACK_RIGHT_DISTANCE_SENSOR);
            telemetry.addLine("  ✅ Back right REV 2m distance sensor");
        } catch (Exception e) {
            backRightDistanceSensor = null;
            telemetry.addLine("  ⚠️ Back right REV 2m distance sensor: " + e.getMessage());
        }

        // Initialize color sensors (REV Color Sensor V3 - optional, may not all be present)
        // IMPORTANT: Set gain to increase detection range (default is too low)
        // REV Color Sensor V3 needs gain adjustment to detect colors at useful distances
        // SEPARATE from motor/servo initialization so sensor failures don't break everything
        // NOTE: Front Left and Back Right are temporarily replaced with REV 2m distance sensors
        try {
            frontLeftColorSensor = hardwareMap.get(NormalizedColorSensor.class, FRONT_LEFT_COLOR_SENSOR);
            if (frontLeftColorSensor != null) {
                frontLeftColorSensor.setGain(50);  // Increased gain for better detection range
                telemetry.addLine("  ✅ Front left color sensor (gain=50)");
            }
        } catch (Exception e) {
            frontLeftColorSensor = null;
            telemetry.addLine("  ⚠️ Front left color sensor: " + e.getMessage() + " (temp: REV 2m distance)");
        }

        try {
            frontRightColorSensor = hardwareMap.get(NormalizedColorSensor.class, FRONT_RIGHT_COLOR_SENSOR);
            if (frontRightColorSensor != null) {
                frontRightColorSensor.setGain(50);  // Increased gain for better detection range
                telemetry.addLine("  ✅ Front right color sensor (gain=50)");
            }
        } catch (Exception e) {
            frontRightColorSensor = null;
            telemetry.addLine("  ⚠️ Front right color sensor: " + e.getMessage());
        }

        try {
            backRightColorSensor = hardwareMap.get(NormalizedColorSensor.class, BACK_RIGHT_COLOR_SENSOR);
            if (backRightColorSensor != null) {
                backRightColorSensor.setGain(50);  // Increased gain for better detection range
                telemetry.addLine("  ✅ Back right color sensor (gain=50)");
            }
        } catch (Exception e) {
            backRightColorSensor = null;
            telemetry.addLine("  ⚠️ Back right color sensor: " + e.getMessage() + " (temp: REV 2m distance)");
        }

        try {
            leftRightColorSensor = hardwareMap.get(NormalizedColorSensor.class, LEFT_RIGHT_COLOR_SENSOR);
            if (leftRightColorSensor != null) {
                leftRightColorSensor.setGain(50);  // Increased gain for better detection range
                telemetry.addLine("  ✅ Left back color sensor (gain=50)");
            }
        } catch (Exception e) {
            leftRightColorSensor = null;
            telemetry.addLine("  ⚠️ Left back color sensor: " + e.getMessage());
        }

        try {
            frontCenterColorSensor = hardwareMap.get(NormalizedColorSensor.class, FRONT_CENTER_COLOR_SENSOR);
            if (frontCenterColorSensor != null) {
                frontCenterColorSensor.setGain(50);  // Increased gain for better detection range
                telemetry.addLine("  ✅ Front center color sensor (gain=50)");
            }
        } catch (Exception e) {
            frontCenterColorSensor = null;
            telemetry.addLine("  ⚠️ Front center color sensor: " + e.getMessage());
        }

        try {
            backCenterColorSensor = hardwareMap.get(NormalizedColorSensor.class, BACK_CENTER_COLOR_SENSOR);
            if (backCenterColorSensor != null) {
                backCenterColorSensor.setGain(50);  // Increased gain for better detection range
                telemetry.addLine("  ✅ Back center color sensor (gain=50)");
            }
        } catch (Exception e) {
            backCenterColorSensor = null;
            telemetry.addLine("  ⚠️ Back center color sensor: " + e.getMessage());
        }

        // Mark system as initialized if motors and servos are OK
        // Sensors are optional and their failure doesn't prevent system from being "initialized"
        indexingSystemInitialized = motorsAndServosOk;

        if (motorsAndServosOk) {
            telemetry.addLine("  ✅ Indexing System (motors & servos OK)");
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
    public DcMotor getLeftShooterMotor() { return leftShooterMotor; }
    public DcMotor getRightShooterMotor() { return rightShooterMotor; }
    public Servo getLightServo() { return lightServo; }  // May be null

    // Turret System
    public Servo getTurretServo() { return turretServo; }  // May be null if not configured

    // Intake and Indexing System
    public DcMotor getFrontRollerMotor() { return frontRollerMotor; }
    public DcMotor getBackRollerMotor() { return backRollerMotor; }
    public CRServo getFrontBottomIntakeServo() { return frontBottomIntakeServo; }
    public CRServo getBackBottomIntakeServo() { return backBottomIntakeServo; }
    public CRServo getFrontTransferServo() { return frontTransferServo; }
    public CRServo getBackTransferServo() { return backTransferServo; }
    public CRServo getUptakeServoL() { return uptakeServoL; }
    public CRServo getUptakeServoR() { return uptakeServoR; }
    public CRServo getInjectorServoLeft() { return injectorServoLeft; }
    public CRServo getInjectorServoRight() { return injectorServoRight; }

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

    // NEW: REV 2m Distance Sensors for enhanced artifact detection
    public DistanceSensor getFrontLeftDistanceSensor() { return frontLeftDistanceSensor; }
    public DistanceSensor getBackRightDistanceSensor() { return backRightDistanceSensor; }

    /**
     * Get distance reading from front left REV 2m sensor in centimeters
     * These sensors face parallel with intake rollers, ~25cm baseline when empty
     * @return Distance in cm, or -1 if sensor not available
     */
    public double getFrontLeftDistanceCM() {
        if (frontLeftDistanceSensor == null) return -1;
        try {
            return frontLeftDistanceSensor.getDistance(DistanceUnit.CM);
        } catch (Exception e) {
            return -1;
        }
    }

    /**
     * Get distance reading from back right REV 2m sensor in centimeters
     * These sensors face parallel with intake rollers, ~25cm baseline when empty
     * @return Distance in cm, or -1 if sensor not available
     */
    public double getBackRightDistanceCM() {
        if (backRightDistanceSensor == null) return -1;
        try {
            return backRightDistanceSensor.getDistance(DistanceUnit.CM);
        } catch (Exception e) {
            return -1;
        }
    }

    public NormalizedColorSensor getFrontLeftColorSensor() { return frontLeftColorSensor; }
    public NormalizedColorSensor getFrontRightColorSensor() { return frontRightColorSensor; }
    public NormalizedColorSensor getBackRightColorSensor() { return backRightColorSensor; }
    public NormalizedColorSensor getLeftRightColorSensor() { return leftRightColorSensor; }
    public NormalizedColorSensor getFrontCenterColorSensor() { return frontCenterColorSensor; }
    public NormalizedColorSensor getBackCenterColorSensor() { return backCenterColorSensor; }

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
    public boolean isTurretSystemInitialized() { return turretSystemInitialized; }
    public boolean isIndexingSystemInitialized() { return indexingSystemInitialized; }
    public boolean isIMUInitialized() { return imuInitialized; }
    public boolean isOdometryInitialized() { return odometryInitialized; }

    public String getDriveInitError() { return driveInitError; }
    public String getShooterInitError() { return shooterInitError; }
    public String getTurretInitError() { return turretInitError; }
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
        summary.append("  Turret: ").append(turretSystemInitialized ? "✅" : "⚠️").append("\n");
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
     * Stop all motors and servos (emergency stop)
     */
    public void stopAllMotors() {
        // Stop drive motors
        if (frontLeftMotor != null) frontLeftMotor.setPower(0);
        if (frontRightMotor != null) frontRightMotor.setPower(0);
        if (backLeftMotor != null) backLeftMotor.setPower(0);
        if (backRightMotor != null) backRightMotor.setPower(0);

        // Stop shooter motors
        if (leftShooterMotor != null) leftShooterMotor.setPower(0);
        if (rightShooterMotor != null) rightShooterMotor.setPower(0);

        // Stop intake rollers
        if (frontRollerMotor != null) frontRollerMotor.setPower(0);
        if (backRollerMotor != null) backRollerMotor.setPower(0);

        // Stop bottom intake servos
        if (frontBottomIntakeServo != null) frontBottomIntakeServo.setPower(0);
        if (backBottomIntakeServo != null) backBottomIntakeServo.setPower(0);

        // Stop transfer servos
        if (frontTransferServo != null) frontTransferServo.setPower(0);
        if (backTransferServo != null) backTransferServo.setPower(0);

        // Stop uptake servos
        if (uptakeServoL != null) uptakeServoL.setPower(0);
        if (uptakeServoR != null) uptakeServoR.setPower(0);

        // Stop injector servos
        if (injectorServoLeft != null) injectorServoLeft.setPower(0);
        if (injectorServoRight != null) injectorServoRight.setPower(0);
    }
}
