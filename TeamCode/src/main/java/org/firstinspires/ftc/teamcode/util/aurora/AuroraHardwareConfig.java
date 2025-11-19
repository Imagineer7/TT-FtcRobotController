/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Unified Hardware Configuration for AURORA System
 * Centralizes all hardware mapping in one place for easy maintenance
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

/**
 * AuroraHardwareConfig - Unified Hardware Configuration
 * 
 * This class centralizes all hardware device mapping for the AURORA robot system.
 * Instead of calling hardwareMap.get() throughout different classes, all hardware
 * initialization happens here in one place.
 * 
 * Benefits:
 * - Single source of truth for all hardware device names
 * - Easy to update hardware configuration in one place
 * - Graceful error handling with detailed logging
 * - Consistent initialization across all OpModes
 * - Clear documentation of all robot hardware
 * 
 * Usage:
 * <pre>
 * AuroraHardwareConfig hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
 * hardware.initialize();  // Or initializeWithOdometry() for TeleOp
 * 
 * // Then pass hardware to subsystems:
 * SmartMechanumDrive drive = new SmartMechanumDrive(hardware);
 * EnhancedDecodeHelper shooter = new EnhancedDecodeHelper(hardware);
 * </pre>
 */
public class AuroraHardwareConfig {
    
    // ========================================================================================
    // HARDWARE DEVICE NAMES - Change these to match your robot configuration
    // ========================================================================================
    
    // Drive Motors
    public static final String FRONT_LEFT_MOTOR = "frontLeft";
    public static final String FRONT_RIGHT_MOTOR = "frontRight";
    public static final String BACK_LEFT_MOTOR = "backLeft";
    public static final String BACK_RIGHT_MOTOR = "backRight";
    
    // Shooter System
    public static final String SHOOTER_MOTOR = "shooter";
    public static final String FEED_SERVO_1 = "servo1";
    public static final String FEED_SERVO_2 = "servo2";
    public static final String LIGHT_SERVO = "light";
    
    // Odometry
    public static final String ODOMETRY_COMPUTER = "odo";
    
    // ========================================================================================
    // HARDWARE COMPONENTS - Public accessors
    // ========================================================================================
    
    // Drive system motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    
    // Shooter system components
    private DcMotor shooterMotor;
    private CRServo feedServo1;
    private CRServo feedServo2;
    private Servo lightServo;
    
    // Sensors
    private VoltageSensor voltageSensor;
    private GoBildaPinpointDriver odometry;
    
    // ========================================================================================
    // INITIALIZATION STATE TRACKING
    // ========================================================================================
    
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    
    // Initialization status flags
    private boolean driveSystemInitialized = false;
    private boolean shooterSystemInitialized = false;
    private boolean odometryInitialized = false;
    
    // Error messages for debugging
    private String driveInitError = "";
    private String shooterInitError = "";
    private String odometryInitError = "";
    
    // ========================================================================================
    // ODOMETRY CONFIGURATION - Adjust these for your robot
    // ========================================================================================
    
    // Odometry pod offsets (in inches)
    // X offset: how far sideways from center is the forward pod (right is positive)
    // Y offset: how far forward from center is the strafe pod (forward is positive)
    private static final double ODOMETRY_X_OFFSET = 4.71;  // inches
    private static final double ODOMETRY_Y_OFFSET = -6.62; // inches
    
    // Pod directions
    private static final GoBildaPinpointDriver.EncoderDirection FORWARD_POD_DIRECTION = 
        GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection STRAFE_POD_DIRECTION = 
        GoBildaPinpointDriver.EncoderDirection.REVERSED;
    
    /**
     * Constructor - Creates hardware config object
     * Call initialize() or initializeWithOdometry() after construction
     * 
     * @param hardwareMap The FTC hardware map
     * @param telemetry Telemetry for logging initialization status
     */
    public AuroraHardwareConfig(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
    
    /**
     * Initialize all hardware WITHOUT odometry (for Autonomous OpModes)
     * Odometry is optional and may not be needed in autonomous if using time-based or
     * encoder-based movement instead of continuous position tracking
     */
    public void initialize() {
        initializeDriveSystem();
        initializeShooterSystem();
        initializeVoltageSensor();
        
        telemetry.addLine("✅ Hardware initialization complete (without odometry)");
        telemetry.update();
    }
    
    /**
     * Initialize all hardware WITH odometry (for TeleOp OpModes)
     * Use this when you need continuous position tracking during the match
     */
    public void initializeWithOdometry() {
        initializeDriveSystem();
        initializeShooterSystem();
        initializeVoltageSensor();
        initializeOdometry();
        
        telemetry.addLine("✅ Hardware initialization complete (with odometry)");
        telemetry.update();
    }
    
    /**
     * Initialize drive system motors with error handling
     */
    private void initializeDriveSystem() {
        try {
            telemetry.addLine("Initializing drive system...");
            telemetry.update();
            
            frontLeftMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR);
            frontRightMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR);
            backLeftMotor = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR);
            backRightMotor = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR);
            
            // Configure motor behavior
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            driveSystemInitialized = true;
            driveInitError = "";
            telemetry.addLine("✅ Drive system initialized successfully");
            telemetry.addLine("   Motors: " + FRONT_LEFT_MOTOR + ", " + FRONT_RIGHT_MOTOR + ", " + 
                             BACK_LEFT_MOTOR + ", " + BACK_RIGHT_MOTOR);
            
        } catch (IllegalArgumentException e) {
            driveSystemInitialized = false;
            driveInitError = "Motor not found: " + e.getMessage();
            telemetry.addLine("⚠️ Drive system failed: " + driveInitError);
            telemetry.addLine("   Check motor names in hardware configuration:");
            telemetry.addLine("   Expected: " + FRONT_LEFT_MOTOR + ", " + FRONT_RIGHT_MOTOR + ", " + 
                             BACK_LEFT_MOTOR + ", " + BACK_RIGHT_MOTOR);
        } catch (Exception e) {
            driveSystemInitialized = false;
            driveInitError = e.getClass().getSimpleName() + ": " + e.getMessage();
            telemetry.addLine("⚠️ Drive system failed: " + driveInitError);
        }
        telemetry.update();
    }
    
    /**
     * Initialize shooter system components with error handling
     */
    private void initializeShooterSystem() {
        try {
            telemetry.addLine("Initializing shooter system...");
            telemetry.update();
            
            shooterMotor = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR);
            feedServo1 = hardwareMap.get(CRServo.class, FEED_SERVO_1);
            feedServo2 = hardwareMap.get(CRServo.class, FEED_SERVO_2);
            
            // Configure shooter motor
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Light servo is optional
            try {
                lightServo = hardwareMap.get(Servo.class, LIGHT_SERVO);
                telemetry.addLine("   Light indicator found");
            } catch (Exception e) {
                lightServo = null;
                telemetry.addLine("   Light indicator not found (optional)");
            }
            
            shooterSystemInitialized = true;
            shooterInitError = "";
            telemetry.addLine("✅ Shooter system initialized successfully");
            telemetry.addLine("   Motor: " + SHOOTER_MOTOR);
            telemetry.addLine("   Servos: " + FEED_SERVO_1 + ", " + FEED_SERVO_2);
            
        } catch (IllegalArgumentException e) {
            shooterSystemInitialized = false;
            shooterInitError = "Device not found: " + e.getMessage();
            telemetry.addLine("⚠️ Shooter system failed: " + shooterInitError);
            telemetry.addLine("   Check device names in hardware configuration:");
            telemetry.addLine("   Expected: " + SHOOTER_MOTOR + ", " + FEED_SERVO_1 + ", " + FEED_SERVO_2);
        } catch (Exception e) {
            shooterSystemInitialized = false;
            shooterInitError = e.getClass().getSimpleName() + ": " + e.getMessage();
            telemetry.addLine("⚠️ Shooter system failed: " + shooterInitError);
        }
        telemetry.update();
    }
    
    /**
     * Initialize voltage sensor with error handling
     */
    private void initializeVoltageSensor() {
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            telemetry.addLine("✅ Voltage sensor initialized");
        } catch (Exception e) {
            voltageSensor = null;
            telemetry.addLine("⚠️ Voltage sensor not found (optional)");
        }
        telemetry.update();
    }
    
    /**
     * Initialize odometry computer with error handling
     */
    private void initializeOdometry() {
        try {
            telemetry.addLine("Initializing odometry computer...");
            telemetry.update();
            
            odometry = hardwareMap.get(GoBildaPinpointDriver.class, ODOMETRY_COMPUTER);
            
            // Configure odometry
            odometry.setOffsets(ODOMETRY_X_OFFSET, ODOMETRY_Y_OFFSET, DistanceUnit.INCH);
            odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odometry.setEncoderDirections(FORWARD_POD_DIRECTION, STRAFE_POD_DIRECTION);
            
            // Reset position and calibrate IMU
            odometry.resetPosAndIMU();
            
            odometryInitialized = true;
            odometryInitError = "";
            telemetry.addLine("✅ Odometry initialized successfully");
            telemetry.addData("   Device", ODOMETRY_COMPUTER);
            telemetry.addData("   Version", odometry.getDeviceVersion());
            telemetry.addData("   Status", odometry.getDeviceStatus());
            telemetry.addData("   X Offset", "%.2f in", ODOMETRY_X_OFFSET);
            telemetry.addData("   Y Offset", "%.2f in", ODOMETRY_Y_OFFSET);
            
        } catch (IllegalArgumentException e) {
            odometry = null;
            odometryInitialized = false;
            odometryInitError = "Device not found: " + e.getMessage();
            telemetry.addLine("⚠️ Odometry device '" + ODOMETRY_COMPUTER + "' not found in hardware configuration!");
            telemetry.addLine("");
            telemetry.addLine("To fix this:");
            telemetry.addLine("1. Go to Robot Configuration on Driver Station");
            telemetry.addLine("2. Add an I2C device named '" + ODOMETRY_COMPUTER + "'");
            telemetry.addLine("3. Set device type to 'goBILDA Pinpoint'");
        } catch (Exception e) {
            odometry = null;
            odometryInitialized = false;
            odometryInitError = e.getClass().getSimpleName() + ": " + e.getMessage();
            telemetry.addLine("⚠️ Odometry initialization failed!");
            telemetry.addLine("   Error: " + odometryInitError);
        }
        telemetry.update();
    }
    
    // ========================================================================================
    // PUBLIC ACCESSORS - Use these to get hardware components
    // ========================================================================================
    
    // Drive motors
    public DcMotor getFrontLeftMotor() { return frontLeftMotor; }
    public DcMotor getFrontRightMotor() { return frontRightMotor; }
    public DcMotor getBackLeftMotor() { return backLeftMotor; }
    public DcMotor getBackRightMotor() { return backRightMotor; }
    
    // Shooter components
    public DcMotor getShooterMotor() { return shooterMotor; }
    public CRServo getFeedServo1() { return feedServo1; }
    public CRServo getFeedServo2() { return feedServo2; }
    public Servo getLightServo() { return lightServo; }
    
    // Sensors
    public VoltageSensor getVoltageSensor() { return voltageSensor; }
    public GoBildaPinpointDriver getOdometry() { return odometry; }
    
    // Original hardware map (for cases where direct access is still needed)
    public HardwareMap getHardwareMap() { return hardwareMap; }
    
    // ========================================================================================
    // STATUS CHECKING - Use these to check if hardware initialized successfully
    // ========================================================================================
    
    public boolean isDriveSystemInitialized() { return driveSystemInitialized; }
    public boolean isShooterSystemInitialized() { return shooterSystemInitialized; }
    public boolean isOdometryInitialized() { return odometryInitialized; }
    
    public String getDriveInitError() { return driveInitError; }
    public String getShooterInitError() { return shooterInitError; }
    public String getOdometryInitError() { return odometryInitError; }
    
    /**
     * Check if all critical systems are initialized
     * @return true if drive and shooter are both initialized
     */
    public boolean isFullyInitialized() {
        return driveSystemInitialized && shooterSystemInitialized;
    }
    
    /**
     * Check if any system failed to initialize
     * @return true if any initialization error occurred
     */
    public boolean hasInitializationErrors() {
        return !driveInitError.isEmpty() || !shooterInitError.isEmpty() || !odometryInitError.isEmpty();
    }
    
    /**
     * Get a summary of initialization status
     * @return Multi-line string describing initialization status
     */
    public String getInitializationSummary() {
        StringBuilder summary = new StringBuilder();
        summary.append("=== Hardware Initialization Summary ===\n");
        summary.append("Drive System: ").append(driveSystemInitialized ? "✅ OK" : "❌ FAILED").append("\n");
        if (!driveInitError.isEmpty()) {
            summary.append("  Error: ").append(driveInitError).append("\n");
        }
        summary.append("Shooter System: ").append(shooterSystemInitialized ? "✅ OK" : "❌ FAILED").append("\n");
        if (!shooterInitError.isEmpty()) {
            summary.append("  Error: ").append(shooterInitError).append("\n");
        }
        summary.append("Odometry: ").append(odometryInitialized ? "✅ OK" : "⚠️ Not Initialized").append("\n");
        if (!odometryInitError.isEmpty()) {
            summary.append("  Error: ").append(odometryInitError).append("\n");
        }
        return summary.toString();
    }
}
