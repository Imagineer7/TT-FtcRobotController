package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MechanumDrive;
import org.firstinspires.ftc.teamcode.util.AprilTagMultiTool;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * AdvancedPositioningHelper - Precision Robot Localization System (v2.1)
 * 
 * This class integrates multiple positioning systems for robust and accurate
 * robot localization, now supporting both traditional and advanced odometry systems:
 * 
 * Positioning Systems Priority:
 * 1. Unified Dead Wheel Odometry (Primary) - Supports both traditional 3-wheel and Pinpoint 2-pod
 * 2. AprilTag Vision (Secondary) - Absolute position corrections  
 * 3. Motor Encoders (Fallback) - Basic positioning when dead wheels fail
 * 4. IMU (Heading) - Heading corrections and backup
 * 
 * New Features v2.1:
 * - Dual Camera Support: Front and back cameras for maximum AprilTag coverage
 * - Automatic camera switching based on detection quality
 * - Motor Encoder + Camera configurations (no dead wheels required)
 * - Configurable camera positions relative to robot center
 * - Automatic AprilTag-based initial localization (optional, default enabled)
 * - Robot can automatically determine starting position using visible AprilTags
 * - Runtime relocalization for periodic position corrections
 * - Comprehensive AprilTag field position database for DECODE
 * 
 * Features v2.0:
 * - Automatic detection of traditional vs. Pinpoint odometry systems
 * - Support for goBILDA Pinpoint 2-pod system with built-in IMU
 * - Velocity data available with Pinpoint system
 * - Unified interface for both odometry types
 * - Automatic fallback and sensor fusion
 * 
 * AprilTag Auto-Localization:
 * - Scans for AprilTags during initialization (3-second window)
 * - Uses best detection to calculate robot's field position
 * - Falls back to (0,0,0°) if no suitable tags found
 * - Can be disabled by passing attemptAprilTagLocalization=false
 * - Runtime relocalization available via attemptAprilTagRelocalization()
 * 
 * DECODE Field Configuration (Official FTC Coordinate System):
 * - Field Size: 141" × 141" square field  
 * - Origin (0,0,0): Field center, on the floor
 * - X-axis: Positive toward audience (front of field)
 * - Y-axis: Positive from Red Wall (left from audience) toward Blue Alliance (right from audience)
 * - Z-axis: Positive vertically upward from floor tiles
 * - Red Wall: Left side as seen from audience (-Y side)
 * - Blue Alliance: Right side as seen from audience (+Y side)
 * - Goal AprilTags: Both on back wall (opposite audience, -X side), 45° angles toward center
 *   • Red Goal (ID 24): (-58.3727, 55.6425, 29.5) heading 315° (back-right corner)
 *   • Blue Goal (ID 20): (-58.3727, -55.6425, 29.5) heading 45° (back-left corner)
 * - Heading: 0° = facing positive Y direction (toward Blue Alliance)
 * 
 * Usage Examples:
 * 
 * // OPTION 1: Motor encoders + single camera (basic setup)
 * positionHelper.initialize("Webcam 1");
 * 
 * // OPTION 2: Motor encoders + dual cameras (recommended for best AprilTag coverage)
 * positionHelper.initializeDualCamera("Webcam 1", "Webcam 2", 
 *     6.0, 0.0, 8.0, 0.0,     // Front camera: 6" forward, 8" up, facing forward
 *     -6.0, 0.0, 8.0, 180.0); // Back camera: 6" back, 8" up, facing backward
 * 
 * // OPTION 3: Motor encoders only (no cameras)
 * positionHelper.initialize(null, false, false);
 * positionHelper.resetPosition(-12.0, -52.5, 90.0); // Red Alliance start
 * 
 * // OPTION 4: Try dead wheel odometry with dual camera fallback
 * positionHelper.initializeDualCamera("Webcam 1", "Webcam 2",
 *     6.0, 0.0, 8.0, 0.0,     // Front camera positioning
 *     -6.0, 0.0, 8.0, 180.0,  // Back camera positioning
 *     true, true);             // Try dead wheels, enable auto-localization
 * 
 * // Autonomous OpMode Example:
 * public class YourAutonomous extends LinearOpMode {
 *     AdvancedPositioningHelper positionHelper = new AdvancedPositioningHelper(this);
 *     
 *     public void runOpMode() {
 *         // Initialize with dual cameras for maximum coverage
 *         positionHelper.initializeDualCamera("Webcam 1", "Webcam 2",
 *             6.0, 0.0, 8.0, 0.0,     // Front: 6" forward, 8" up
 *             -6.0, 0.0, 8.0, 180.0); // Back: 6" back, 8" up, facing backward
 *         
 *         waitForStart();
 *         
 *         while (opModeIsActive()) {
 *             positionHelper.updatePosition(); // Update position from all sensors
 *             
 *             // Move to a position
 *             if (positionHelper.goToPosition(24.0, 24.0, 90.0, 0.6)) {
 *                 // Reached target position
 *             }
 *             
 *             // Switch to best camera for AprilTag corrections
 *             positionHelper.switchToBestCamera();
 *             
 *             positionHelper.updateTelemetry();
 *             telemetry.update();
 *         }
 *     }
 * }
 * 
 * // Runtime relocalization during autonomous (uses best available camera)
 * if (positionHelper.attemptAprilTagRelocalization()) {
 *     // Position successfully corrected using best camera view
 * }
 * 
 * // Camera management
 * if (positionHelper.switchToBestCamera()) {
 *     // Automatically switched to camera with better AprilTag view
 * }
 * 
 * // Get comprehensive status
 * telemetry.addData("Camera Status", positionHelper.getAprilTagStatus());
 * telemetry.addData("Camera Info", positionHelper.getCameraInfo());
 * 
 * @author FTC Team
 * @version 2.1 - Added AprilTag Auto-Localization
 */
public class AdvancedPositioningHelper {
    
    // Hardware references
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    
    // Drive motors
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    
    // Optional: Use existing MechanumDrive class for motor control (can be null)
    private MechanumDrive mechanumDrive = null;
    
    // Sensors
    private IMU imu;
    private AprilTagMultiTool aprilTagUtil;
    
    // Camera configuration
    private boolean useDualCameras = false;
    private String frontCameraName = null;
    private String backCameraName = null;
    private String currentActiveCamera = "front"; // Track which camera is currently active
    private ElapsedTime lastCameraSwitchTime = new ElapsedTime(); // Prevent excessive switching
    private double minSwitchInterval = 0.5; // Minimum 500ms between camera switches
    private double frontCameraX = 0.0;    // Front camera X offset from robot center (inches)
    private double frontCameraY = 0.0;    // Front camera Y offset from robot center (inches)
    private double frontCameraZ = 0.0;    // Front camera Z offset from robot center (inches)
    private double frontCameraYaw = 0.0;  // Front camera yaw angle (degrees)
    private double frontCameraPitch = -90.0; // Front camera pitch angle (degrees, -90 = horizontal)
    private double frontCameraRoll = 0.0; // Front camera roll angle (degrees)
    private double backCameraX = 0.0;     // Back camera X offset from robot center (inches)
    private double backCameraY = 0.0;     // Back camera Y offset from robot center (inches)
    private double backCameraZ = 0.0;     // Back camera Z offset from robot center (inches)
    private double backCameraYaw = 180.0; // Back camera yaw angle (degrees, 180 = facing backward)
    private double backCameraPitch = -90.0; // Back camera pitch angle (degrees)
    private double backCameraRoll = 0.0;  // Back camera roll angle (degrees)
    
    // Unified dead wheel odometry system (auto-detects traditional vs. Pinpoint)
    private DeadWheelOdometry deadWheelOdometry;
    private boolean useDeadWheels = false;
    private DeadWheelOdometry.OdometryType odometryType = DeadWheelOdometry.OdometryType.NONE_DETECTED;
    
    // Position tracking variables
    private double currentX = 0.0;      // Field X position (inches)
    private double currentY = 0.0;      // Field Y position (inches)
    private double currentHeading = 0.0; // Robot heading (degrees)
    
    // Encoder tracking (for motor encoder odometry)
    private int lastLeftFrontPos = 0;
    private int lastRightFrontPos = 0;
    private int lastLeftBackPos = 0;
    private int lastRightBackPos = 0;
    
    // IMU calibration
    private double imuOffset = 0.0;
    private boolean imuCalibrated = false;
    
    // AprilTag correction
    private ElapsedTime lastAprilTagUpdate = new ElapsedTime();
    private double aprilTagConfidence = 0.0;
    
    // Robot constants (based on your motor and wheel specifications)
    // 
    // DRIVE MOTOR SPECIFICATIONS:
    // - Type: Brushed DC Motor with Planetary Gearbox
    // - Gear Ratio: 13.7:1 (formula: (1+(46/17)) * (1+(46/17)))
    // - Encoder Resolution: 384.5 PPR at output shaft (formula: 13.7 * 28 = 383.6, actual 384.5)
    // - No-Load Speed: 435 RPM @ 12VDC
    // - Stall Torque: 18.7 kg.cm (260 oz-in)
    // - Encoder Type: Relative Quadrature, Magnetic (Hall Effect)
    // 
    // WHEEL SPECIFICATIONS:
    // - Diameter: 104mm (4.094 inches)
    // 
    // NOTE: These constants are for DRIVE MOTOR odometry (fallback positioning).
    // Dead wheel odometry uses separate encoders with different specifications (see DeadWheelOdometry.java)
    public static final double COUNTS_PER_MOTOR_REV = 384.5;  // PPR at output shaft with 13.7:1 gearbox
    public static final double DRIVE_GEAR_REDUCTION = 1.0;    // No additional gearing (direct drive)
    public static final double WHEEL_DIAMETER_INCHES = 4.094; // 104mm = 4.094 inches
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double ROBOT_WIDTH = 18.0;   // Distance between left and right wheels (measure your robot)
    public static final double ROBOT_LENGTH = 18.0;  // Distance between front and back wheels (measure your robot)
    
    // DECODE Field constants (measured field dimensions)
    public static final double FIELD_WIDTH = 141.0;        // 3580mm = ~141" inside perimeter walls
    public static final double FIELD_LENGTH = 141.0;       // Square field configuration
    public static final double FIELD_CENTER_TO_WALL = 70.5; // 141/2 = 70.5 inches from center to wall
    public static final double TILE_SIZE_INCHES = 23.5;    // Standard FTC tile size
    public static final double PERIMETER_HEIGHT = 12.3;    // 313mm perimeter height in inches
    
    // DECODE Field Coordinate System positions (Official FTC coordinates)
    public static final double BACK_WALL_X = -70.5;            // Back wall X coordinate (opposite audience)
    public static final double AUDIENCE_WALL_X = 70.5;         // Audience wall X coordinate (front of field)
    public static final double RED_WALL_Y = -70.5;             // Red Wall Y coordinate (left from audience)
    public static final double BLUE_ALLIANCE_Y = 70.5;         // Blue Alliance Y coordinate (right from audience)
    
    // Goal AprilTag positions (Official DECODE coordinates)
    // Both goals are on the back wall (opposite audience side)
    public static final double RED_GOAL_X = -58.3727;          // Red Goal X coordinate (back wall)
    public static final double RED_GOAL_Y = 55.6425;           // Red Goal Y coordinate (toward Blue Alliance side)
    public static final double BLUE_GOAL_X = -58.3727;         // Blue Goal X coordinate (back wall, same as Red)
    public static final double BLUE_GOAL_Y = -55.6425;         // Blue Goal Y coordinate (toward Red Wall side)
    public static final double GOAL_HEIGHT = 29.5;             // AprilTag vertical center height
    public static final double APRILTAG_SIZE = 8.125;          // AprilTag square size in inches
    
    // Alliance starting areas (typical robot placement)
    public static final double RED_ALLIANCE_START_X = -35.0;   // Red Alliance starting area X
    public static final double RED_ALLIANCE_START_Y = 60.0;    // Red Alliance starting area Y
    public static final double BLUE_ALLIANCE_START_X = 35.0;   // Blue Alliance starting area X
    public static final double BLUE_ALLIANCE_START_Y = -60.0;  // Blue Alliance starting area Y
    
    // Movement constants
    public static final double POSITION_TOLERANCE = 1.0;  // inches
    public static final double HEADING_TOLERANCE = 2.0;   // degrees
    public static final double MAX_DRIVE_SPEED = 0.8;
    public static final double MIN_DRIVE_SPEED = 0.2;
    public static final double MAX_TURN_SPEED = 0.6;
    public static final double MIN_TURN_SPEED = 0.15;
    
    // Sensor fusion weights (adjust based on confidence)
    private double encoderWeight = 0.7;
    private double imuWeight = 0.2;
    private double aprilTagWeight = 0.1;
    
    /**
     * Constructor
     */
    public AdvancedPositioningHelper(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }
    
    // ========================================
    // INITIALIZATION METHODS
    // ========================================
    
    /**
     * Initialize the positioning system with motor encoders and single camera
     * @param webcamName Name of the single webcam for AprilTag detection
     */
    public void initialize(String webcamName) {
        initialize(webcamName, false, true);
    }
    
    /**
     * Initialize with motor encoders and single camera with optional auto-localization
     * @param webcamName Name of the webcam for AprilTag detection
     * @param attemptDeadWheels Whether to attempt dead wheel odometry initialization (false for motor encoders only)
     * @param attemptAprilTagLocalization Whether to try determining initial position from AprilTags
     */
    public void initialize(String webcamName, boolean attemptDeadWheels, boolean attemptAprilTagLocalization) {
        this.useDualCameras = false;
        this.frontCameraName = webcamName;
        this.backCameraName = null;
        
        initializeMotors();
        initializeIMU();
        initializeAprilTag(webcamName);
        
        if (attemptDeadWheels) {
            initializeUnifiedOdometry();
        }
        
        // Try to determine initial position using AprilTags if available
        boolean initialPositionSet = false;
        if (attemptAprilTagLocalization && aprilTagUtil != null) {
            initialPositionSet = attemptAprilTagInitialLocalization();
        }
        
        // Fall back to default position if AprilTag localization failed
        if (!initialPositionSet) {
            resetPosition(0, 0, 0);
            telemetry.addData("Initial Position", "Default (0, 0, 0°) - No AprilTag fix");
        }
        
        telemetry.addData("Advanced Positioning", "Initialized v2.1 - Single Camera");
        telemetry.addData("Camera System", "Single: " + webcamName);
        if (useDeadWheels) {
            telemetry.addData("Odometry System", odometryType + " (" + deadWheelOdometry.getSystemStatus() + ")");
        } else {
            telemetry.addData("Odometry System", "Motor Encoders Only");
        }
        telemetry.update();
    }
    
    /**
     * Initialize with motor encoders and dual cameras for maximum AprilTag coverage
     * @param frontCameraName Name of the front-facing camera
     * @param backCameraName Name of the back-facing camera
     * @param frontX Front camera X offset from robot center (inches, + = forward)
     * @param frontY Front camera Y offset from robot center (inches, + = left)
     * @param frontZ Front camera Z offset from robot center (inches, + = up)
     * @param frontYaw Front camera yaw angle (degrees, 0 = facing forward)
     * @param backX Back camera X offset from robot center (inches, + = forward)
     * @param backY Back camera Y offset from robot center (inches, + = left)
     * @param backZ Back camera Z offset from robot center (inches, + = up)
     * @param backYaw Back camera yaw angle (degrees, 180 = facing backward)
     */
    public void initializeDualCamera(String frontCameraName, String backCameraName,
                                   double frontX, double frontY, double frontZ, double frontYaw,
                                   double backX, double backY, double backZ, double backYaw) {
        initializeDualCamera(frontCameraName, backCameraName, 
                           frontX, frontY, frontZ, frontYaw, -90.0, 0.0,
                           backX, backY, backZ, backYaw, -90.0, 0.0,
                           false, true);
    }
    
    /**
     * Initialize with motor encoders and dual cameras with full orientation control
     * @param frontCameraName Name of the front-facing camera
     * @param backCameraName Name of the back-facing camera
     * @param frontX Front camera X offset from robot center (inches)
     * @param frontY Front camera Y offset from robot center (inches)
     * @param frontZ Front camera Z offset from robot center (inches)
     * @param frontYaw Front camera yaw angle (degrees)
     * @param frontPitch Front camera pitch angle (degrees, -90 = horizontal)
     * @param frontRoll Front camera roll angle (degrees)
     * @param backX Back camera X offset from robot center (inches)
     * @param backY Back camera Y offset from robot center (inches)
     * @param backZ Back camera Z offset from robot center (inches)
     * @param backYaw Back camera yaw angle (degrees)
     * @param backPitch Back camera pitch angle (degrees, -90 = horizontal)
     * @param backRoll Back camera roll angle (degrees)
     * @param attemptDeadWheels Whether to attempt dead wheel odometry initialization
     * @param attemptAprilTagLocalization Whether to try determining initial position from AprilTags
     */
    public void initializeDualCamera(String frontCameraName, String backCameraName,
                                   double frontX, double frontY, double frontZ, double frontYaw, double frontPitch, double frontRoll,
                                   double backX, double backY, double backZ, double backYaw, double backPitch, double backRoll,
                                   boolean attemptDeadWheels, boolean attemptAprilTagLocalization) {
        // Store dual camera configuration
        this.useDualCameras = true;
        this.frontCameraName = frontCameraName;
        this.backCameraName = backCameraName;
        this.frontCameraX = frontX;
        this.frontCameraY = frontY;
        this.frontCameraZ = frontZ;
        this.frontCameraYaw = frontYaw;
        this.frontCameraPitch = frontPitch;
        this.frontCameraRoll = frontRoll;
        this.backCameraX = backX;
        this.backCameraY = backY;
        this.backCameraZ = backZ;
        this.backCameraYaw = backYaw;
        this.backCameraPitch = backPitch;
        this.backCameraRoll = backRoll;
        
        initializeMotors();
        initializeIMU();
        initializeDualAprilTag();
        
        if (attemptDeadWheels) {
            initializeUnifiedOdometry();
        }
        
        // Try to determine initial position using AprilTags if available
        boolean initialPositionSet = false;
        if (attemptAprilTagLocalization && aprilTagUtil != null) {
            initialPositionSet = attemptDualCameraAprilTagLocalization();
        }
        
        // Fall back to default position if AprilTag localization failed
        if (!initialPositionSet) {
            resetPosition(0, 0, 0);
            telemetry.addData("Initial Position", "Default (0, 0, 0°) - No AprilTag fix");
        }
        
        telemetry.addData("Advanced Positioning", "Initialized v2.1 - Dual Camera");
        telemetry.addData("Front Camera", frontCameraName + String.format(" (%.1f, %.1f, %.1f) @%.0f°", frontX, frontY, frontZ, frontYaw));
        telemetry.addData("Back Camera", backCameraName + String.format(" (%.1f, %.1f, %.1f) @%.0f°", backX, backY, backZ, backYaw));
        if (useDeadWheels) {
            telemetry.addData("Odometry System", odometryType + " (" + deadWheelOdometry.getSystemStatus() + ")");
        } else {
            telemetry.addData("Odometry System", "Motor Encoders Only");
        }
        telemetry.update();
    }
    
    /**
     * Initialize with unified odometry system (auto-detects hardware)
     */
    public void initialize(String webcamName, boolean attemptDeadWheels) {
        initialize(webcamName, attemptDeadWheels, true);
    }
    

    
    /**
     * Initialize with custom traditional 3-wheel parameters
     */
    public void initialize(String webcamName, double trackWidth, double horizontalOffset) {
        initializeMotors();
        initializeIMU();
        initializeAprilTag(webcamName);
        
        // Force traditional 3-wheel system with custom parameters
        try {
            deadWheelOdometry = new DeadWheelOdometry(hardwareMap, telemetry, trackWidth, horizontalOffset);
            useDeadWheels = deadWheelOdometry.isInitialized();
            odometryType = DeadWheelOdometry.OdometryType.THREE_WHEEL_TRADITIONAL;
            
            telemetry.addData("Traditional Odometry", "Initialized with custom parameters");
            telemetry.addData("Track Width", String.format("%.2f\"", trackWidth));
            telemetry.addData("Horizontal Offset", String.format("%.2f\"", horizontalOffset));
        } catch (Exception e) {
            telemetry.addData("Traditional Odometry Error", e.getMessage());
            useDeadWheels = false;
            odometryType = DeadWheelOdometry.OdometryType.NONE_DETECTED;
        }
        
        resetPosition(0, 0, 0);
        
        telemetry.addData("Advanced Positioning", "Initialized with Custom Traditional System");
        telemetry.addData("System Status", useDeadWheels ? "Traditional Dead Wheels Active" : "Motor Encoders Fallback");
        telemetry.update();
    }
    
    /**
     * Initialize drive motors with proper encoder reset (FTC best practices)
     */
    private void initializeMotors() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        
        // Set directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Set brake behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoder positions to zero (FTC best practice)
        resetEncoders();
        
        // Store initial positions (should be zero after reset)
        updateEncoderPositions();
    }
    
    /**
     * Reset motor encoders to zero (FTC best practice)
     * Always call this at the beginning of OpModes to ensure encoders start at zero
     */
    public void resetEncoders() {
        // Stop and reset all motor encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Turn motors back on for use (required after STOP_AND_RESET_ENCODER)
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Reset position tracking variables
        lastLeftFrontPos = 0;
        lastRightFrontPos = 0;
        lastLeftBackPos = 0;
        lastRightBackPos = 0;
        
        telemetry.addData("Encoders", "Reset to zero");
    }
    
    /**
     * Initialize IMU
     */
    private void initializeIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(revOrientation));
        
        // Reset IMU yaw
        imu.resetYaw();
        imuCalibrated = true;
    }
    
    /**
     * Initialize AprilTag system with single camera
     */
    private void initializeAprilTag(String webcamName) {
        if (webcamName != null) {
            aprilTagUtil = new AprilTagMultiTool(hardwareMap, true, webcamName, null);
            
            // Set camera position relative to robot center (default positioning if not specified)
            Position cameraPosition = new Position(DistanceUnit.INCH, frontCameraX, frontCameraY, frontCameraZ, 0);
            YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, frontCameraYaw, frontCameraPitch, frontCameraRoll, 0);
            aprilTagUtil.setCameraPose(cameraPosition, cameraOrientation);
            
            aprilTagUtil.resumeStreaming();
        }
    }
    
    /**
     * Initialize AprilTag system with dual cameras
     */
    private void initializeDualAprilTag() {
        if (frontCameraName != null && backCameraName != null) {
            aprilTagUtil = new AprilTagMultiTool(hardwareMap, true, frontCameraName, backCameraName);
            
            // Set front camera as default (we'll switch as needed)
            Position frontCameraPosition = new Position(DistanceUnit.INCH, frontCameraX, frontCameraY, frontCameraZ, 0);
            YawPitchRollAngles frontCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, frontCameraYaw, frontCameraPitch, frontCameraRoll, 0);
            aprilTagUtil.setCameraPose(frontCameraPosition, frontCameraOrientation);
            
            aprilTagUtil.resumeStreaming();
            
            telemetry.addData("Dual Camera Init", "✓ Front: " + frontCameraName + ", Back: " + backCameraName);
        } else if (frontCameraName != null) {
            // Fall back to single camera if back camera fails
            initializeAprilTag(frontCameraName);
            telemetry.addData("Camera Init", "✓ Single front camera fallback");
        }
    }
    
    /**
     * Attempt to determine initial robot position using AprilTag detection
     * This method will scan for AprilTags for a few seconds and use the best detection
     * to calculate the robot's starting position on the field.
     * 
     * @return true if initial position was successfully determined from AprilTags
     */
    private boolean attemptAprilTagInitialLocalization() {
        if (aprilTagUtil == null) {
            telemetry.addData("AprilTag Localization", "✗ No camera available");
            return false;
        }
        
        telemetry.addData("AprilTag Localization", "Scanning for tags...");
        telemetry.update();
        
        ElapsedTime scanTimer = new ElapsedTime();
        AprilTagDetection bestDetection = null;
        double bestConfidence = 0.0;
        int detectionCount = 0;
        
        // Scan for AprilTags for up to 3 seconds
        while (scanTimer.seconds() < 3.0 && !opMode.isStopRequested()) {
            List<AprilTagDetection> detections = aprilTagUtil.getDetections();
            
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.ftcPose != null) {
                    detectionCount++;
                    double confidence = calculateAprilTagConfidence(detection);
                    
                    telemetry.addData("Found Tag", "ID:%d Range:%.1f\" Conf:%.2f", 
                                    detection.id, detection.ftcPose.range, confidence);
                    
                    if (confidence > bestConfidence) {
                        bestDetection = detection;
                        bestConfidence = confidence;
                    }
                }
            }
            
            telemetry.addData("Scan Progress", "%.1fs (found %d tags)", scanTimer.seconds(), detectionCount);
            telemetry.update();
            
            // Small delay to allow camera processing
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        
        // Try to calculate initial position from best detection
        if (bestDetection != null && bestConfidence > 0.3) {
            double[] initialPos = calculateRobotPositionFromTag(bestDetection);
            
            if (initialPos != null) {
                resetPosition(initialPos[0], initialPos[1], initialPos[2]);
                
                telemetry.addData("AprilTag Localization", "✓ SUCCESS");
                telemetry.addData("Initial Position", "X:%.1f Y:%.1f H:%.1f°", 
                                initialPos[0], initialPos[1], initialPos[2]);
                telemetry.addData("Based on Tag", "ID:%d Range:%.1f\" Conf:%.2f", 
                                bestDetection.id, bestDetection.ftcPose.range, bestConfidence);
                
                return true;
            }
        }
        
        // Failed to get good localization
        telemetry.addData("AprilTag Localization", "✗ FAILED");
        if (detectionCount == 0) {
            telemetry.addData("Reason", "No AprilTags detected");
        } else if (bestConfidence <= 0.3) {
            telemetry.addData("Reason", "Low confidence (%.2f < 0.3)", bestConfidence);
        } else {
            telemetry.addData("Reason", "Position calculation failed");
        }
        
        return false;
    }
    
    /**
     * Attempt dual camera AprilTag localization by trying both cameras
     * @return true if initial position was successfully determined from AprilTags
     */
    private boolean attemptDualCameraAprilTagLocalization() {
        if (aprilTagUtil == null) {
            telemetry.addData("AprilTag Localization", "✗ No cameras available");
            return false;
        }
        
        telemetry.addData("Dual Camera Localization", "Scanning both cameras...");
        telemetry.update();
        
        ElapsedTime scanTimer = new ElapsedTime();
        AprilTagDetection bestDetection = null;
        double bestConfidence = 0.0;
        int detectionCount = 0;
        String bestCamera = "none";
        
        // Scan for AprilTags using both cameras for up to 4 seconds
        while (scanTimer.seconds() < 4.0 && !opMode.isStopRequested()) {
            // Try front camera first
            switchToFrontCamera();
            List<AprilTagDetection> frontDetections = aprilTagUtil.getDetections();
            
            for (AprilTagDetection detection : frontDetections) {
                if (detection.metadata != null) {
                    detectionCount++;
                    double confidence = calculateAprilTagConfidence(detection);
                    
                    if (confidence > bestConfidence) {
                        bestDetection = detection;
                        bestConfidence = confidence;
                        bestCamera = "front";
                    }
                    
                    telemetry.addData("Front Cam Tag " + detection.id, 
                        "Conf: %.2f Dist: %.1f\"", confidence, detection.ftcPose.range);
                }
            }
            
            // Try back camera second
            switchToBackCamera();
            List<AprilTagDetection> backDetections = aprilTagUtil.getDetections();
            
            for (AprilTagDetection detection : backDetections) {
                if (detection.metadata != null) {
                    detectionCount++;
                    double confidence = calculateAprilTagConfidence(detection);
                    
                    if (confidence > bestConfidence) {
                        bestDetection = detection;
                        bestConfidence = confidence;
                        bestCamera = "back";
                    }
                    
                    telemetry.addData("Back Cam Tag " + detection.id, 
                        "Conf: %.2f Dist: %.1f\"", confidence, detection.ftcPose.range);
                }
            }
            
            telemetry.addData("Best Detection", bestCamera + " cam, conf: %.2f", bestConfidence);
            telemetry.update();
            
            // Small delay to allow camera processing
            try {
                Thread.sleep(75);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        
        // Try to calculate initial position from best detection
        if (bestDetection != null && bestConfidence > 0.3) {
            // Make sure we're using the camera that detected the best tag
            if (bestCamera.equals("front")) {
                switchToFrontCamera();
            } else {
                switchToBackCamera();
            }
            
            double[] initialPos = calculateRobotPositionFromTag(bestDetection);
            
            if (initialPos != null) {
                resetPosition(initialPos[0], initialPos[1], initialPos[2]);
                
                telemetry.addData("Dual Cam Localization", "✓ SUCCESS via " + bestCamera + " camera");
                telemetry.addData("Initial Position", "X:%.1f Y:%.1f H:%.1f°", 
                                initialPos[0], initialPos[1], initialPos[2]);
                telemetry.addData("Detection Quality", "Tag %d, conf: %.2f", bestDetection.id, bestConfidence);
                
                return true;
            }
        }
        
        // Failed to localize
        telemetry.addData("Dual Cam Localization", "✗ FAILED");
        telemetry.addData("Detections Found", "%d total from both cameras", detectionCount);
        
        if (detectionCount == 0) {
            telemetry.addData("Reason", "No AprilTags detected on either camera");
        } else if (bestConfidence <= 0.3) {
            telemetry.addData("Reason", "Low confidence (%.2f < 0.3) on both cameras", bestConfidence);
        } else {
            telemetry.addData("Reason", "Position calculation failed");
        }
        
        return false;
    }
    
    /**
     * Initialize unified odometry system (auto-detects traditional vs. Pinpoint)
     */
    private void initializeUnifiedOdometry() {
        try {
            deadWheelOdometry = new DeadWheelOdometry(hardwareMap, telemetry);
            useDeadWheels = deadWheelOdometry.isInitialized();
            odometryType = deadWheelOdometry.getSystemType();
            
            if (useDeadWheels) {
                telemetry.addData("Unified Odometry", "✓ " + odometryType);
                telemetry.addData("System Status", deadWheelOdometry.getSystemStatus());
                
                if (deadWheelOdometry.isPinpointSystem()) {
                    telemetry.addData("Pinpoint Features", "2-pod system, built-in IMU, velocity data");
                } else if (deadWheelOdometry.isTraditionalSystem()) {
                    telemetry.addData("Traditional Features", "3-wheel system, proven reliability");
                }
            } else {
                telemetry.addData("Unified Odometry", "✗ No systems detected");
            }
            
        } catch (Exception e) {
            telemetry.addData("Odometry Error", e.getMessage());
            useDeadWheels = false;
            odometryType = DeadWheelOdometry.OdometryType.NONE_DETECTED;
        }
    }
    
    // ========================================
    // POSITION TRACKING
    // ========================================
    
    /**
     * Update robot position using all available sensors
     */
    public void updatePosition() {
        if (useDeadWheels && deadWheelOdometry != null && deadWheelOdometry.isInitialized()) {
            // Use dead wheel odometry for primary position tracking
            updateDeadWheelOdometry();
        } else {
            // Fall back to motor encoder odometry
            updateEncoderOdometry();
        }
        
        updateIMUHeading();
        updateAprilTagCorrection();
    }
    
    /**
     * Update position using dead wheel odometry (most accurate)
     */
    private void updateDeadWheelOdometry() {
        if (deadWheelOdometry == null || !deadWheelOdometry.isInitialized()) return;
        
        // Update dead wheel position
        deadWheelOdometry.updatePosition();
        
        // Get position from dead wheels with sensor fusion weights
        double[] deadWheelPos = deadWheelOdometry.getPosition();
        double deadWheelX = deadWheelPos[0];
        double deadWheelY = deadWheelPos[1];
        double deadWheelHeading = deadWheelPos[2];
        
        // Apply sensor fusion (dead wheels get highest weight for X/Y)
        currentX = currentX * (1 - 0.9) + deadWheelX * 0.9;
        currentY = currentY * (1 - 0.9) + deadWheelY * 0.9;
        
        // Note: Heading is primarily updated by IMU for better accuracy
    }
    
    /**
     * Update position using motor encoder odometry
     */
    private void updateEncoderOdometry() {
        // Get current encoder positions
        int leftFrontPos = leftFrontDrive.getCurrentPosition();
        int rightFrontPos = rightFrontDrive.getCurrentPosition();
        int leftBackPos = leftBackDrive.getCurrentPosition();
        int rightBackPos = rightBackDrive.getCurrentPosition();
        
        // Calculate deltas
        int deltaLeftFront = leftFrontPos - lastLeftFrontPos;
        int deltaRightFront = rightFrontPos - lastRightFrontPos;
        int deltaLeftBack = leftBackPos - lastLeftBackPos;
        int deltaRightBack = rightBackPos - lastRightBackPos;
        
        // Convert to inches
        double leftFrontInches = deltaLeftFront / COUNTS_PER_INCH;
        double rightFrontInches = deltaRightFront / COUNTS_PER_INCH;
        double leftBackInches = deltaLeftBack / COUNTS_PER_INCH;
        double rightBackInches = deltaRightBack / COUNTS_PER_INCH;
        
        // Calculate robot movement (mechanum drive kinematics)
        double deltaForward = (leftFrontInches + rightFrontInches + leftBackInches + rightBackInches) / 4.0;
        double deltaStrafe = (-leftFrontInches + rightFrontInches + leftBackInches - rightBackInches) / 4.0;
        
        // Convert robot-relative movement to field coordinates using DECODE coordinate system
        // Heading 0° = facing positive Y (toward Blue Alliance)
        // Heading 90° = facing positive X (toward audience)
        double headingRad = Math.toRadians(currentHeading);
        double deltaX = deltaForward * Math.sin(headingRad) + deltaStrafe * Math.cos(headingRad);
        double deltaY = deltaForward * Math.cos(headingRad) - deltaStrafe * Math.sin(headingRad);
        
        // Update position with encoder weight
        currentX += deltaX * encoderWeight;
        currentY += deltaY * encoderWeight;
        
        // Update stored positions
        updateEncoderPositions();
    }
    
    /**
     * Update encoder position storage
     */
    private void updateEncoderPositions() {
        lastLeftFrontPos = leftFrontDrive.getCurrentPosition();
        lastRightFrontPos = rightFrontDrive.getCurrentPosition();
        lastLeftBackPos = leftBackDrive.getCurrentPosition();
        lastRightBackPos = rightBackDrive.getCurrentPosition();
    }
    
    /**
     * Update heading using IMU
     */
    private void updateIMUHeading() {
        if (imuCalibrated) {
            double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double correctedHeading = imuHeading + imuOffset;
            
            // Apply sensor fusion
            currentHeading = (currentHeading * (1 - imuWeight)) + (correctedHeading * imuWeight);
            
            // Normalize to -180 to 180
            while (currentHeading > 180) currentHeading -= 360;
            while (currentHeading <= -180) currentHeading += 360;
        }
    }
    
    /**
     * Update position using AprilTag corrections (optimized dual camera switching)
     */
    private void updateAprilTagCorrection() {
        if (aprilTagUtil == null) return;
        
        AprilTagDetection bestDetection = null;
        double bestConfidence = 0.0;
        String bestCamera = "none";
        
        if (useDualCameras && frontCameraName != null && backCameraName != null) {
            // Optimized dual camera strategy: 
            // 1. Try current active camera first
            // 2. Only switch if current camera has poor/no detections AND enough time has passed
            
            List<AprilTagDetection> currentDetections = aprilTagUtil.getDetections();
            
            // Check current active camera first
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    double confidence = calculateAprilTagConfidence(detection);
                    if (confidence > bestConfidence) {
                        bestDetection = detection;
                        bestConfidence = confidence;
                        bestCamera = currentActiveCamera;
                    }
                }
            }
            
            // Only switch cameras if:
            // 1. Current camera has poor detection (confidence < 0.3) OR no detections
            // 2. Enough time has passed since last switch (prevents rapid switching)
            boolean shouldTrySwitching = (bestConfidence < 0.3 || currentDetections.isEmpty()) && 
                                       lastCameraSwitchTime.seconds() > minSwitchInterval;
            
            if (shouldTrySwitching) {
                // Try the other camera
                String otherCamera = currentActiveCamera.equals("front") ? "back" : "front";
                
                if (otherCamera.equals("front")) {
                    switchToFrontCamera();
                } else {
                    switchToBackCamera();
                }
                
                // Small delay to allow camera to stabilize
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                
                List<AprilTagDetection> otherDetections = aprilTagUtil.getDetections();
                
                // Check if other camera has better detections
                double otherBestConfidence = 0.0;
                AprilTagDetection otherBestDetection = null;
                
                for (AprilTagDetection detection : otherDetections) {
                    if (detection.metadata != null) {
                        double confidence = calculateAprilTagConfidence(detection);
                        if (confidence > otherBestConfidence) {
                            otherBestDetection = detection;
                            otherBestConfidence = confidence;
                        }
                    }
                }
                
                // Use other camera if significantly better (20% improvement threshold)
                if (otherBestConfidence > bestConfidence * 1.2) {
                    bestDetection = otherBestDetection;
                    bestConfidence = otherBestConfidence;
                    bestCamera = otherCamera;
                    currentActiveCamera = otherCamera;
                    lastCameraSwitchTime.reset();
                } else {
                    // Switch back to original camera if no improvement
                    if (currentActiveCamera.equals("front")) {
                        switchToFrontCamera();
                    } else {
                        switchToBackCamera();
                    }
                }
            }
            
        } else {
            // Single camera mode
            List<AprilTagDetection> detections = aprilTagUtil.getDetections();
            
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    double confidence = calculateAprilTagConfidence(detection);
                    if (confidence > bestConfidence) {
                        bestDetection = detection;
                        bestConfidence = confidence;
                        bestCamera = "single";
                    }
                }
            }
        }
        
        if (bestDetection != null && bestConfidence > 0.2) { // Lower threshold for runtime corrections
            // Calculate robot position from AprilTag
            double[] robotPos = calculateRobotPositionFromTag(bestDetection);
            
            if (robotPos != null) {
                // Apply correction with confidence weighting
                double weight = aprilTagWeight * bestConfidence;
                
                currentX = (currentX * (1 - weight)) + (robotPos[0] * weight);
                currentY = (currentY * (1 - weight)) + (robotPos[1] * weight);
                currentHeading = (currentHeading * (1 - weight)) + (robotPos[2] * weight);
                
                aprilTagConfidence = bestConfidence;
                lastAprilTagUpdate.reset();
            }
        }
    }
    
    /**
     * Calculate robot position from AprilTag detection
     * This method uses the AprilTag's known field position and the robot's relative
     * position to the tag to calculate the robot's field coordinates.
     */
    private double[] calculateRobotPositionFromTag(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null || detection.metadata == null) {
            return null;
        }
        
        // Get known field positions for DECODE AprilTags
        double[] tagFieldPos = getAprilTagFieldPosition(detection.id);
        if (tagFieldPos == null) {
            return null; // Unknown tag ID
        }
        
        // Get robot's position relative to the tag (in tag's coordinate system)
        double tagX = detection.ftcPose.x;      // Robot X relative to tag
        double tagY = detection.ftcPose.y;      // Robot Y relative to tag
        double tagYaw = detection.ftcPose.yaw;  // Robot heading relative to tag
        
        // Known field position of the tag
        double tagFieldX = tagFieldPos[0];
        double tagFieldY = tagFieldPos[1];
        double tagFieldHeading = tagFieldPos[2]; // Tag's heading on field
        
        // Convert robot position from tag coordinates to field coordinates
        // Account for tag's orientation on the field
        double tagHeadingRad = Math.toRadians(tagFieldHeading);
        
        // Rotate robot's relative position by tag's field heading
        double robotFieldX = tagFieldX + (tagX * Math.cos(tagHeadingRad) - tagY * Math.sin(tagHeadingRad));
        double robotFieldY = tagFieldY + (tagX * Math.sin(tagHeadingRad) + tagY * Math.cos(tagHeadingRad));
        
        // Calculate robot's field heading
        double robotFieldHeading = normalizeAngle(tagFieldHeading + Math.toDegrees(tagYaw));
        
        return new double[]{robotFieldX, robotFieldY, robotFieldHeading};
    }
    
    /**
     * Get the known field position of an AprilTag for DECODE field
     * Based on official FTC DECODE field coordinates
     * @param tagId The AprilTag ID
     * @return Array containing [x, y, heading] of the tag on the field, or null if unknown
     */
    private double[] getAprilTagFieldPosition(int tagId) {
        // Official DECODE field AprilTag positions in inches
        // Coordinate system: (0,0,0) = field center on floor
        // +X = toward audience (front), +Y = from Red Wall (left) to Blue Alliance (right), +Z = up
        // AprilTag size: 8.125" square (36h11 family)
        switch (tagId) {
            // GOAL APRILTAGS (Primary localization targets)
            // Both goal tags are on the back wall (opposite audience side)
            // Goal tags are positioned at 45° angles in corners, facing toward field center
            case 20: // Blue Alliance Goal
                // Position: Back wall, toward Red Wall side (left from audience view)
                // Red Goal is at (-58.3727, 55.6425), Blue Goal is at (-58.3727, -55.6425)
                return new double[]{-58.3727, -55.6425, 45.0}; // 45° angle facing toward center
                
            case 24: // Red Alliance Goal (Official coordinates)
                // Position: Back wall, toward Blue Alliance side (right from audience view)
                return new double[]{-58.3727, 55.6425, 315.0}; // 45° angle facing toward center (-45°)
            
            // FIELD STRUCTURE APRILTAGS (if present on DECODE field)
            // Note: These are estimated positions - verify with actual field measurements
            case 1: // Potential Red Alliance side structure
                return new double[]{-70.5, -35.0, 90.0}; // Red alliance wall area
            case 2: // Potential Red Alliance center structure  
                return new double[]{-70.5, 0.0, 90.0};   // Red alliance wall center
            case 3: // Potential Red Alliance right structure
                return new double[]{-70.5, 35.0, 90.0};  // Red alliance wall area
            case 4: // Potential Blue Alliance side structure
                return new double[]{70.5, -35.0, 270.0}; // Blue alliance wall area
            case 5: // Potential Blue Alliance center structure
                return new double[]{70.5, 0.0, 270.0};   // Blue alliance wall center
            case 6: // Potential Blue Alliance right structure
                return new double[]{70.5, 35.0, 270.0};  // Blue alliance wall area
            case 7: // Potential left side structure
                return new double[]{0.0, -70.5, 0.0};    // Left wall, center
            case 8: // Potential right side structure
                return new double[]{0.0, 70.5, 180.0};   // Right wall, center
            case 9: // Potential center field structure
                return new double[]{0.0, 0.0, 0.0};      // Field center
            case 10: // Additional structure tag
                return new double[]{-35.0, -35.0, 45.0}; // Red quadrant
            case 11: // Additional structure tag
                return new double[]{35.0, -35.0, 135.0}; // Blue quadrant
            case 12: // Additional structure tag
                return new double[]{35.0, 35.0, 225.0};  // Blue quadrant
            case 13: // Additional structure tag
                return new double[]{-35.0, 35.0, 315.0}; // Red quadrant
                
            default:
                // Unknown tag ID - cannot calculate position
                telemetry.addData("Unknown AprilTag", "ID: " + tagId + " (not in DECODE database)");
                return null;
        }
    }
    
    /**
     * Calculate confidence in AprilTag detection
     * Optimized for DECODE field with 8.125" AprilTags
     */
    private double calculateAprilTagConfidence(AprilTagDetection detection) {
        // Confidence based on range and detection quality
        double rangeConfidence = Math.max(0, 1.0 - (detection.ftcPose.range / 72.0)); // Decrease with distance (larger field)
        double sizeConfidence = Math.min(1.0, detection.metadata.tagsize / APRILTAG_SIZE); // Normalize to DECODE tag size (8.125")
        
        // Boost confidence for goal tags (primary localization targets)
        double tagTypeBoost = 1.0;
        if (detection.id == 20 || detection.id == 24) { // Goal AprilTags
            tagTypeBoost = 1.2; // 20% confidence boost for goal tags
        }
        
        double baseConfidence = rangeConfidence * sizeConfidence * tagTypeBoost;
        return Math.min(1.0, baseConfidence); // Cap at 1.0
    }
    
    // ========================================
    // POSITION CONTROL
    // ========================================
    
    /**
     * Move robot to specific field position
     * @param targetX Target X coordinate (inches)
     * @param targetY Target Y coordinate (inches)
     * @param targetHeading Target heading (degrees)
     * @param maxSpeed Maximum movement speed
     * @return true when position is reached
     */
    public boolean goToPosition(double targetX, double targetY, double targetHeading, double maxSpeed) {
        updatePosition();
        
        // Calculate distance and angle to target
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // Calculate heading error
        double headingError = normalizeAngle(targetHeading - currentHeading);
        
        // Check if we've reached the target
        if (distanceToTarget < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
            stopMotors();
            return true;
        }
        
        // Calculate drive powers
        double drivePower = calculateDrivePower(distanceToTarget, maxSpeed);
        double turnPower = calculateTurnPower(headingError, maxSpeed);
        
        // Convert field-relative movement to robot-relative coordinates
        // Account for DECODE coordinate system where 0° heading = positive Y direction (toward Blue Alliance)
        double fieldAngleToTarget = Math.toDegrees(Math.atan2(deltaX, deltaY)); // Angle in field coordinates
        double robotAngleToTarget = normalizeAngle(fieldAngleToTarget - currentHeading);
        
        // Calculate robot-relative movement components
        double forward = drivePower * Math.cos(Math.toRadians(robotAngleToTarget));
        double strafe = drivePower * Math.sin(Math.toRadians(robotAngleToTarget));
        
        // Apply movement
        // DIRECTION FIX OPTION 3: If forward/backward are reversed, negate the forward component:
        setMechanumPowers(forward, strafe, turnPower);  // TO FIX: Change to (-forward, strafe, turnPower)
        
        // Debug telemetry for movement troubleshooting
        telemetry.addData("=== MOVEMENT DEBUG ===", "");
        telemetry.addData("Target", String.format("(%.1f, %.1f, %.0f°)", targetX, targetY, targetHeading));
        telemetry.addData("Current", String.format("(%.1f, %.1f, %.0f°)", currentX, currentY, currentHeading));
        telemetry.addData("Delta", String.format("(%.1f, %.1f)", deltaX, deltaY));
        telemetry.addData("Distance", String.format("%.1f\"", distanceToTarget));
        telemetry.addData("Field Angle", String.format("%.1f°", fieldAngleToTarget));
        telemetry.addData("Robot Angle", String.format("%.1f°", robotAngleToTarget));
        telemetry.addData("Powers", String.format("F:%.2f S:%.2f T:%.2f", forward, strafe, turnPower));
        
        return false;
    }

    /** Simple cardinal directions in the ROBOT frame. */
    public enum MoveDir {
        FORWARD, BACKWARD, LEFT, RIGHT
    }

    /**
     * Move a given distance in the ROBOT frame.
     * Angle convention (robot-relative): 0° = forward, +90° = right, -90° = left, 180° = backward.
     *
     * @param inches               distance to move (>= 0)
     * @param robotDirectionDeg    direction in robot frame (degrees, see convention above)
     * @param maxSpeed             max drive speed (0..1)
     * @param timeoutSeconds       safety timeout in seconds
     * @param holdHeading          if true, robot keeps its current heading
     * @return true if reached target within tolerance before timeout
     */
    public boolean moveInchesRobot(double inches,
                                double robotDirectionDeg,
                                double maxSpeed,
                                double timeoutSeconds,
                                boolean holdHeading) {
        // Current pose becomes the reference
        updatePosition();
        double startX = currentX;
        double startY = currentY;
        double startHeading = currentHeading;

        // Convert robot-relative to field-relative direction
        // Field convention in this class: 0° = +Y (toward Blue), +90° = +X (toward audience)
        // Robot-relative 0° (forward) is aligned with current heading.
        double fieldDirectionDeg = normalizeAngle(robotDirectionDeg + startHeading);

        // Compute target in field frame
        double rad = Math.toRadians(fieldDirectionDeg);
        double targetX = startX + inches * Math.sin(rad); // note: X uses sin (matches goToPosition usage)
        double targetY = startY + inches * Math.cos(rad); // note: Y uses cos

        // Keep inside field bounds (just in case)
        double[] clamped = clampToField(targetX, targetY);
        targetX = clamped[0];
        targetY = clamped[1];

        double targetHeading = holdHeading ? startHeading : currentHeading;

        return driveToTargetWithTimeout(targetX, targetY, targetHeading, maxSpeed, timeoutSeconds);
    }

    /**
     * Move a given distance in the FIELD frame.
     * Field convention: 0° = +Y (toward Blue), +90° = +X (toward audience).
     *
     * @param inches             distance to move (>= 0)
     * @param fieldDirectionDeg  absolute field direction (degrees, see convention)
     * @param targetHeading      heading to hold while moving (use getCurrentHeading() to maintain)
     * @param maxSpeed           max drive speed (0..1)
     * @param timeoutSeconds     safety timeout in seconds
     * @return true if reached target within tolerance before timeout
     */
    public boolean moveInchesField(double inches,
                                double fieldDirectionDeg,
                                double targetHeading,
                                double maxSpeed,
                                double timeoutSeconds) {
        updatePosition();
        double startX = currentX;
        double startY = currentY;

        double rad = Math.toRadians(normalizeAngle(fieldDirectionDeg));
        double targetX = startX + inches * Math.sin(rad);
        double targetY = startY + inches * Math.cos(rad);

        double[] clamped = clampToField(targetX, targetY);
        targetX = clamped[0];
        targetY = clamped[1];

        return driveToTargetWithTimeout(targetX, targetY, targetHeading, maxSpeed, timeoutSeconds);
    }

    /**
     * Convenience: move in a cardinal direction in the ROBOT frame.
     * FORWARD/BACKWARD translate; LEFT/RIGHT strafe.
     *
     * @param dir            FORWARD, BACKWARD, LEFT, RIGHT (robot frame)
     * @param inches         distance to move (>= 0)
     * @param maxSpeed       max drive speed (0..1)
     * @param timeoutSeconds safety timeout
     * @return true if reached target within tolerance before timeout
     */
    public boolean moveCardinalRobot(MoveDir dir,
                                    double inches,
                                    double maxSpeed,
                                    double timeoutSeconds) {
        double angleDeg;
        // DIRECTION FIX OPTION 1: If forward/backward are reversed, swap these angle values:
        switch (dir) {
            case FORWARD:  angleDeg = 0;    break;  // TO FIX: Change to 180 (swap with BACKWARD)
            case BACKWARD: angleDeg = 180;  break;  // TO FIX: Change to 0 (swap with FORWARD)
            case RIGHT:    angleDeg = 90;   break;  // positive strafe in your kinematics
            case LEFT:     angleDeg = -90;  break;
            default:       angleDeg = 0;            // TO FIX: Change to 180 if you swap FORWARD
        }
        // Hold current heading by default in a cardinal move
        return moveInchesRobot(inches, angleDeg, maxSpeed, timeoutSeconds, true);
    }

    /**
     * Core loop: drives to (targetX, targetY, targetHeading) using goToPosition(), with timeout+settle window.
     */
    private boolean driveToTargetWithTimeout(double targetX,
                                            double targetY,
                                            double targetHeading,
                                            double maxSpeed,
                                            double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settle = new ElapsedTime();
        boolean inside = false;

        // Small settle time ensures we truly arrived (helps with oscillation)
        final double SETTLE_REQUIRED_S = 0.15;

        while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
            boolean reached = goToPosition(targetX, targetY, targetHeading, maxSpeed);

            // Track continuous time inside tolerance box
            if (reached) {
                if (!inside) { inside = true; settle.reset(); }
                if (settle.seconds() >= SETTLE_REQUIRED_S) {
                    stopMotors();
                    return true;
                }
            } else {
                inside = false; // left the tolerance box, reset settle window
            }
        }

        // Timeout: stop and report failure
        stopMotors();
        telemetry.addData("moveInches", "Timed out (%.2fs). Target=(%.1f,%.1f,%.0f°)",
                timeoutSeconds, targetX, targetY, targetHeading);
        return false;
    }
    
    /**
     * Rotate robot to specific heading
     * @param targetHeading Target heading (degrees)
     * @param maxSpeed Maximum turn speed
     * @return true when heading is reached
     */
    public boolean rotateToHeading(double targetHeading, double maxSpeed) {
        updatePosition();
        
        double headingError = normalizeAngle(targetHeading - currentHeading);
        
        if (Math.abs(headingError) < HEADING_TOLERANCE) {
            stopMotors();
            return true;
        }
        
        double turnPower = calculateTurnPower(headingError, maxSpeed);
        setMechanumPowers(0, 0, turnPower);
        
        return false;
    }
    
    /**
     * Calculate drive power based on distance
     */
    private double calculateDrivePower(double distance, double maxSpeed) {
        // Proportional control with minimum speed
        double power = distance * 0.1; // Adjust gain as needed
        power = Math.max(MIN_DRIVE_SPEED, Math.min(maxSpeed, Math.abs(power)));
        return power;
    }
    
    /**
     * Calculate turn power based on heading error
     */
    private double calculateTurnPower(double headingError, double maxSpeed) {
        // Proportional control with minimum speed
        double power = headingError * 0.02; // Adjust gain as needed
        power = Math.max(-maxSpeed, Math.min(maxSpeed, power));
        
        if (Math.abs(power) < MIN_TURN_SPEED && Math.abs(headingError) > HEADING_TOLERANCE) {
            power = (power >= 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
        }
        
        return power;
    }
    
    /**
     * Set mechanum drive powers with normalization
     * Can optionally use existing MechanumDrive class if provided
     */
    private void setMechanumPowers(double forward, double strafe, double rotate) {
        if (mechanumDrive != null) {
            // Use existing MechanumDrive class power calculation and normalization
            mechanumDrive.leftFrontPower = forward + strafe + rotate;
            mechanumDrive.rightFrontPower = forward - strafe - rotate;
            mechanumDrive.leftBackPower = forward - strafe + rotate;
            mechanumDrive.rightBackPower = forward + strafe - rotate;
            
            // Apply MechanumDrive's normalization
            double max = Math.max(Math.abs(mechanumDrive.leftFrontPower), Math.abs(mechanumDrive.rightFrontPower));
            max = Math.max(max, Math.abs(mechanumDrive.leftBackPower));
            max = Math.max(max, Math.abs(mechanumDrive.rightBackPower));
            if (max > 1.0) {
                mechanumDrive.leftFrontPower /= max;
                mechanumDrive.rightFrontPower /= max;
                mechanumDrive.leftBackPower /= max;
                mechanumDrive.rightBackPower /= max;
            }
            
            mechanumDrive.setMotorPowers();
        } else {
            // Use built-in motor control (current implementation)
            double leftFrontPower = forward + strafe + rotate;
            double rightFrontPower = forward - strafe - rotate;
            double leftBackPower = forward - strafe + rotate;
            double rightBackPower = forward + strafe - rotate;
            
            // Normalize powers
            double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower),
                    Math.max(Math.abs(rightFrontPower),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));
            
            leftFrontDrive.setPower(leftFrontPower / max);
            rightFrontDrive.setPower(rightFrontPower / max);
            leftBackDrive.setPower(leftBackPower / max);
            rightBackDrive.setPower(rightBackPower / max);
        }
    }
    
    /**
     * Public method to set mecanum drive powers (for advanced movement control)
     * @param forward Forward power (-1.0 to 1.0)
     * @param strafe Strafe power (-1.0 to 1.0, positive = right)  
     * @param rotate Rotation power (-1.0 to 1.0, positive = clockwise)
     */
    public void setDrivePowers(double forward, double strafe, double rotate) {
        setMechanumPowers(forward, strafe, rotate);
    }
    
    /**
     * Stop all motors
     */
    public void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    
    // ========================================
    // POSITION MANAGEMENT
    // ========================================
    
    /**
     * Reset robot position and all sensors (FTC best practice)
     * This should be called at the beginning of autonomous OpModes
     */
    public void resetPosition(double x, double y, double heading) {
        currentX = x; 
        currentY = y;
        currentHeading = heading;
        
        // Reset IMU if needed
        if (imuCalibrated) {
            imu.resetYaw();
            imuOffset = heading;
        }
        
        // Reset appropriate odometry system
        if (useDeadWheels && deadWheelOdometry != null && deadWheelOdometry.isInitialized()) {
            deadWheelOdometry.resetPosition(x, y, heading);
        } else {
            // Reset motor encoders to zero
            resetEncoders();
        }
        
        telemetry.addData("Position Reset", String.format("X:%.1f Y:%.1f H:%.1f°", x, y, heading));
        telemetry.addData("Odometry System", isUsingDeadWheels() ? "Dead Wheels" : "Motor Encoders");
    }
    
    /**
     * Set position without resetting encoders (for manual position correction)
     */
    public void setPosition(double x, double y, double heading) {
        currentX = x;
        currentY = y; 
        currentHeading = heading;
        
        // Update appropriate odometry system
        if (useDeadWheels && deadWheelOdometry != null && deadWheelOdometry.isInitialized()) {
            deadWheelOdometry.setPosition(x, y, heading);
        } else {
            // Update motor encoder position tracking to current values
            updateEncoderPositions();
        }
        
        telemetry.addData("Position Set", String.format("X:%.1f Y:%.1f H:%.1f°", x, y, heading));
    }
    
    /**
     * Attempt to relocalize robot position using AprilTags (can be called during runtime)
     * This is useful for periodic position corrections during autonomous or teleop
     * 
     * @return true if position was successfully updated from AprilTag detection
     */
    public boolean attemptAprilTagRelocalization() {
        return attemptAprilTagInitialLocalization();
    }
    
    /**
     * Get information about available AprilTags for localization
     * Useful for debugging and field setup verification
     */
    public void displayAprilTagInfo() {
        telemetry.addData("=== APRILTAG LOCALIZATION INFO ===", "");
        telemetry.addData("Camera Available", aprilTagUtil != null);
        
        if (aprilTagUtil != null) {
            List<AprilTagDetection> detections = aprilTagUtil.getDetections();
            telemetry.addData("Tags Currently Visible", detections.size());
            
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.ftcPose != null) {
                    double confidence = calculateAprilTagConfidence(detection);
                    double[] tagPos = getAprilTagFieldPosition(detection.id);
                    
                    telemetry.addData("Tag " + detection.id, 
                                    "Range:%.1f\" Conf:%.2f %s", 
                                    detection.ftcPose.range, 
                                    confidence,
                                    tagPos != null ? "✓" : "✗Unknown");
                }
            }
            
            telemetry.addData("", "");
            telemetry.addData("DECODE Field AprilTags (Official):", "");
            telemetry.addData("Tag 20", "Blue Goal (-58.37, -55.64) @ 45°");
            telemetry.addData("Tag 24", "Red Goal (-58.37, 55.64) @ 315°");
            telemetry.addData("Goal Wall", "Both on back wall (opposite audience)");
            telemetry.addData("Goal Orientation", "45° angles facing field center");
            telemetry.addData("Goal Height", "29.5\" (tag center)");
            telemetry.addData("Tags 1-13", "Field structures (estimated positions)");
            telemetry.addData("AprilTag Size", "8.125\" square (36h11 family)");
            telemetry.addData("", "");
            telemetry.addData("DECODE Field Orientation:", "");
            telemetry.addData("Origin", "(0,0,0) = field center on floor");
            telemetry.addData("X-axis", "+X toward audience (front)");
            telemetry.addData("Y-axis", "+Y from Red Wall (left) to Blue Alliance (right)");
            telemetry.addData("Z-axis", "+Z vertically upward");
            telemetry.addData("Red Wall", "Left side as seen from audience (-Y)");
            telemetry.addData("Blue Alliance", "Right side as seen from audience (+Y)");
        } else {
            telemetry.addData("Status", "No camera configured");
        }
    }
    
    // ========================================
    // GETTERS AND STATUS
    // ========================================
    
    /**
     * Get current X position
     */
    public double getCurrentX() {
        return currentX;
    }
    
    /**
     * Get current Y position
     */
    public double getCurrentY() {
        return currentY;
    }
    
    /**
     * Get current heading
     */
    public double getCurrentHeading() {
        return currentHeading;
    }
    
    /**
     * Get current FTC-compliant position as array [x, y, heading_rad]
     */
    public double[] getCurrentPose() {
        return new double[]{currentX, currentY, Math.toRadians(currentHeading)};
    }
    
    /**
     * Check if IMU calibration is complete
     */
    public boolean isImuCalibrated() {
        return imuCalibrated;
    }
    
    /**
     * Check if dead wheel odometry is being used
     */
    public boolean isUsingDeadWheels() {
        return useDeadWheels && deadWheelOdometry != null && deadWheelOdometry.isInitialized();
    }
    
    /**
     * Get reference to dead wheel odometry system (for advanced usage)
     */
    public DeadWheelOdometry getDeadWheelOdometry() {
        return deadWheelOdometry;
    }
    
    /**
     * Get current odometry system status
     */
    public String getOdometryStatus() {
        if (isUsingDeadWheels()) {
            return "Dead Wheels: " + deadWheelOdometry.getSystemStatus();
        } else {
            return "Motor Encoders: Active";
        }
    }
    
    /**
     * Get distance to target position
     */
    public double getDistanceToTarget(double targetX, double targetY) {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    /**
     * Get heading error to target
     */
    public double getHeadingError(double targetHeading) {
        return normalizeAngle(targetHeading - currentHeading);
    }
    
    /**
     * Check if AprilTags are being detected
     */
    public boolean hasAprilTagFix() {
        return aprilTagUtil != null && lastAprilTagUpdate.seconds() < 2.0 && aprilTagConfidence > 0.5;
    }
    
    // ========================================
    // CALIBRATION AND TUNING
    // ========================================
    
    /**
     * Calibrate IMU heading
     */
    public void calibrateIMU(double knownHeading) {
        if (imuCalibrated) {
            double imuReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            imuOffset = knownHeading - imuReading;
            currentHeading = knownHeading;
        }
    }
    
    /**
     * Update sensor fusion weights based on confidence
     */
    public void updateSensorWeights(double encoderConf, double imuConf, double aprilTagConf) {
        double total = encoderConf + imuConf + aprilTagConf;
        if (total > 0) {
            encoderWeight = encoderConf / total;
            imuWeight = imuConf / total;
            aprilTagWeight = aprilTagConf / total;
        }
    }
    
    // ========================================
    // TELEMETRY
    // ========================================
    
    /**
     * Update telemetry with positioning data
     */
    public void updateTelemetry() {
        telemetry.addData("=== POSITIONING ===", "");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", currentX, currentY, currentHeading);
        telemetry.addData("Odometry", getOdometryStatus());
        
        if (isUsingDeadWheels()) {
            telemetry.addData("Dead Wheel Status", deadWheelOdometry.getSystemStatus());
        } else {
            telemetry.addData("Motor Encoders", "LF:%d RF:%d LB:%d RB:%d",
                    leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.addData("Sensor Weights", "E:%.2f I:%.2f A:%.2f", encoderWeight, imuWeight, aprilTagWeight);
        }
        
        telemetry.addData("AprilTag", "Conf:%.2f Time:%.1fs", aprilTagConfidence, lastAprilTagUpdate.seconds());
        
        // IMU data
        if (imuCalibrated) {
            double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU", "Raw:%.1f° Offset:%.1f°", imuHeading, imuOffset);
        }
    }
    
    /**
     * Add custom telemetry
     */
    public void addTelemetry(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
    }
    
    // ========================================
    // UTILITY FUNCTIONS
    // ========================================
    
    /**
     * Normalize angle to -180 to 180 range
     */
    public static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
    
    /**
     * Convert field angle to robot heading
     * In DECODE system: 0° = positive Y direction (toward Blue Alliance)
     * @param fieldAngle Angle in field coordinates (degrees)
     * @return Robot heading (degrees)
     */
    public static double fieldAngleToHeading(double fieldAngle) {
        return normalizeAngle(fieldAngle);
    }
    
    /**
     * Get position relative to Red Alliance starting area (DECODE configuration)
     * @param x Field X coordinate
     * @param y Field Y coordinate
     * @return Distance from Red Alliance area
     */
    public static double getDistanceFromRedAlliance(double x, double y) {
        // Red Alliance is at negative Y values (left side from audience in DECODE)
        double redAllianceX = 0.0; // Center of field in X direction
        double redAllianceY = RED_WALL_Y; // Near Red Wall (left side)
        return Math.sqrt(Math.pow(x - redAllianceX, 2) + Math.pow(y - redAllianceY, 2));
    }
    
    /**
     * Get position relative to Blue Alliance starting area (DECODE configuration)
     * @param x Field X coordinate
     * @param y Field Y coordinate
     * @return Distance from Blue Alliance area
     */
    public static double getDistanceFromBlueAlliance(double x, double y) {
        // Blue Alliance is at positive Y values (right side from audience in DECODE)
        double blueAllianceX = 0.0; // Center of field in X direction
        double blueAllianceY = BLUE_ALLIANCE_Y; // Near Blue Wall (right side)
        return Math.sqrt(Math.pow(x - blueAllianceX, 2) + Math.pow(y - blueAllianceY, 2));
    }
    
    /**
     * Check if position is within DECODE field boundaries
     * @param x Field X coordinate
     * @param y Field Y coordinate
     * @return true if position is within field
     */
    public static boolean isPositionInField(double x, double y) {
        return Math.abs(x) <= FIELD_CENTER_TO_WALL && Math.abs(y) <= FIELD_CENTER_TO_WALL;
    }
    
    /**
     * Clamp position to DECODE field boundaries
     * @param x Field X coordinate
     * @param y Field Y coordinate
     * @return Array containing clamped [x, y] coordinates
     */
    public static double[] clampToField(double x, double y) {
        double clampedX = Math.max(-FIELD_CENTER_TO_WALL, Math.min(FIELD_CENTER_TO_WALL, x));
        double clampedY = Math.max(-FIELD_CENTER_TO_WALL, Math.min(FIELD_CENTER_TO_WALL, y));
        return new double[]{clampedX, clampedY};
    }
    
    /**
     * Get AprilTag example coordinate for DECODE field
     * Example: Red Goal AprilTag center position
     * @return Array containing [x, y, z] coordinates in inches
     */
    public static double[] getRedGoalAprilTagPosition() {
        // Convert from the given example: (-58.3727, 55.6425, 29.5) 
        // This is the center of Red Goal AprilTag on DECODE field
        return new double[]{-58.3727, 55.6425, 29.5};
    }
    
    // ========================================
    // UNIFIED ODOMETRY SYSTEM METHODS (v2.0)
    // ========================================
    
    /**
     * Get the type of odometry system being used
     */
    public DeadWheelOdometry.OdometryType getOdometryType() {
        return odometryType;
    }
    
    /**
     * Check if using Pinpoint system
     */
    public boolean isPinpointSystem() {
        return useDeadWheels && deadWheelOdometry != null && deadWheelOdometry.isPinpointSystem();
    }
    
    /**
     * Check if using traditional 3-wheel system
     */
    public boolean isTraditionalSystem() {
        return useDeadWheels && deadWheelOdometry != null && deadWheelOdometry.isTraditionalSystem();
    }
    
    /**
     * Get robot velocity (available with Pinpoint system)
     * @return Array containing [vx, vy, omega] in inches/sec and radians/sec
     */
    public double[] getVelocity() {
        if (isPinpointSystem()) {
            return deadWheelOdometry.getVelocity();
        }
        return new double[]{0.0, 0.0, 0.0};
    }
    
    /**
     * Check if velocity data is available
     */
    public boolean hasVelocityData() {
        return isPinpointSystem();
    }
    
    /**
     * Get update frequency of the positioning system
     */
    public double getPositioningUpdateFrequency() {
        if (useDeadWheels && deadWheelOdometry != null) {
            return deadWheelOdometry.getUpdateFrequency();
        }
        return 0.0;
    }
    
    /**
     * Get odometry system status
     */
    public String getOdometrySystemStatus() {
        if (useDeadWheels && deadWheelOdometry != null) {
            return deadWheelOdometry.getSystemStatus();
        }
        return "Motor Encoders Only";
    }
    
    /**
     * Recalibrate odometry system (Pinpoint IMU if available)
     */
    public void recalibrateOdometry() {
        if (isPinpointSystem()) {
            deadWheelOdometry.recalibrateIMU();
            telemetry.addData("Calibration", "Pinpoint IMU recalibrated");
        } else {
            telemetry.addData("Calibration", "Not available for current system");
        }
    }
    
    /**
     * Get comprehensive system diagnostics
     */
    public String getSystemDiagnostics() {
        StringBuilder diagnostics = new StringBuilder();
        
        diagnostics.append("=== ADVANCED POSITIONING DIAGNOSTICS ===\n");
        diagnostics.append("System Version: 2.0 (Unified Odometry)\n");
        diagnostics.append("Odometry Type: ").append(odometryType).append("\n");
        diagnostics.append("Using Dead Wheels: ").append(useDeadWheels).append("\n");
        diagnostics.append("AprilTag Available: ").append(aprilTagUtil != null).append("\n");
        diagnostics.append("IMU Calibrated: ").append(imuCalibrated).append("\n");
        diagnostics.append("\n");
        
        diagnostics.append("Current Position: ").append(getFormattedPosition()).append("\n");
        diagnostics.append("Update Frequency: ").append(String.format("%.1f Hz", getPositioningUpdateFrequency())).append("\n");
        
        if (hasVelocityData()) {
            double[] velocity = getVelocity();
            diagnostics.append("Velocity: X=").append(String.format("%.2f", velocity[0]))
                      .append(" Y=").append(String.format("%.2f", velocity[1]))
                      .append(" ω=").append(String.format("%.2f°/s", Math.toDegrees(velocity[2]))).append("\n");
        }
        
        diagnostics.append("\n");
        
        // Add detailed odometry diagnostics if available
        if (useDeadWheels && deadWheelOdometry != null) {
            diagnostics.append("ODOMETRY DETAILS:\n");
            diagnostics.append(deadWheelOdometry.getDiagnostics()).append("\n");
        }
        
        // Field analysis
        diagnostics.append("FIELD ANALYSIS:\n");
        diagnostics.append("Distance to Center: ").append(String.format("%.1f\"", getDistanceToPosition(0, 0))).append("\n");
        diagnostics.append("In Red Alliance: ").append(getCurrentY() < -26.25).append("\n");
        diagnostics.append("In Blue Alliance: ").append(getCurrentY() > 26.25).append("\n");
        diagnostics.append("Near Field Edge: ").append(isPositionNearEdge(getCurrentX(), getCurrentY(), 12.0)).append("\n");
        
        return diagnostics.toString();
    }
    
    /**
     * Add enhanced telemetry with unified odometry info
     */
    public void addEnhancedTelemetry() {
        telemetry.addData("=== POSITIONING SYSTEM v2.0 ===", "");
        telemetry.addData("Odometry Type", odometryType);
        telemetry.addData("System Status", getOdometrySystemStatus());
        telemetry.addData("Update Rate", String.format("%.1f Hz", getPositioningUpdateFrequency()));
        
        // Position info
        telemetry.addData("", "");
        telemetry.addData("=== ROBOT POSITION ===", "");
        telemetry.addData("Position", getFormattedPosition());
        telemetry.addData("Field Zone", getCurrentY() < -26.25 ? "Red Alliance" : 
                                     getCurrentY() > 26.25 ? "Blue Alliance" : "Center");
        
        // Velocity info (if available)
        if (hasVelocityData()) {
            double[] velocity = getVelocity();
            telemetry.addData("", "");
            telemetry.addData("=== VELOCITY ===", "");
            telemetry.addData("Linear", String.format("X:%.1f Y:%.1f in/s", velocity[0], velocity[1]));
            telemetry.addData("Angular", String.format("%.1f°/s", Math.toDegrees(velocity[2])));
        }
        
        // System capabilities
        telemetry.addData("", "");
        telemetry.addData("=== CAPABILITIES ===", "");
        if (isPinpointSystem()) {
            telemetry.addData("Pinpoint Features", "✓ High accuracy, velocity, built-in IMU");
        } else if (isTraditionalSystem()) {
            telemetry.addData("Traditional Features", "✓ Reliable, proven, customizable");
        } else {
            telemetry.addData("Motor Encoders", "✓ Basic positioning, fallback mode");
        }
        
        if (aprilTagUtil != null) {
            telemetry.addData("AprilTag Corrections", "✓ Available");
        }
    }
    
    /**
     * Check if position is near field edge
     */
    private boolean isPositionNearEdge(double x, double y, double tolerance) {
        return Math.abs(x) > (FIELD_CENTER_TO_WALL - tolerance) ||
               Math.abs(y) > (FIELD_CENTER_TO_WALL - tolerance);
    }
    
    // ========================================
    // DUAL CAMERA MANAGEMENT METHODS (v2.1)
    // ========================================
    
    /**
     * Switch to front camera and update camera pose
     */
    public void switchToFrontCamera() {
        if (aprilTagUtil != null && useDualCameras && frontCameraName != null) {
            aprilTagUtil.switchToWebcam1(); // Webcam1 is front camera
            currentActiveCamera = "front";
            
            // Update camera pose for front camera
            Position frontCameraPosition = new Position(DistanceUnit.INCH, frontCameraX, frontCameraY, frontCameraZ, 0);
            YawPitchRollAngles frontCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, frontCameraYaw, frontCameraPitch, frontCameraRoll, 0);
            aprilTagUtil.setCameraPose(frontCameraPosition, frontCameraOrientation);
        }
    }
    
    /**
     * Switch to back camera and update camera pose
     */
    public void switchToBackCamera() {
        if (aprilTagUtil != null && useDualCameras && backCameraName != null) {
            aprilTagUtil.switchToWebcam2(); // Webcam2 is back camera
            currentActiveCamera = "back";
            
            // Update camera pose for back camera
            Position backCameraPosition = new Position(DistanceUnit.INCH, backCameraX, backCameraY, backCameraZ, 0);
            YawPitchRollAngles backCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, backCameraYaw, backCameraPitch, backCameraRoll, 0);
            aprilTagUtil.setCameraPose(backCameraPosition, backCameraOrientation);
        }
    }
    
    /**
     * Automatically switch to the camera with better AprilTag detections
     * @return true if a switch was made to a camera with better detections
     */
    public boolean switchToBestCamera() {
        if (!useDualCameras || aprilTagUtil == null) return false;
        
        // Test front camera
        switchToFrontCamera();
        int frontDetections = 0;
        double frontBestConfidence = 0.0;
        List<AprilTagDetection> frontTags = aprilTagUtil.getDetections();
        
        for (AprilTagDetection detection : frontTags) {
            if (detection.metadata != null) {
                frontDetections++;
                double confidence = calculateAprilTagConfidence(detection);
                frontBestConfidence = Math.max(frontBestConfidence, confidence);
            }
        }
        
        // Test back camera
        switchToBackCamera();
        int backDetections = 0;
        double backBestConfidence = 0.0;
        List<AprilTagDetection> backTags = aprilTagUtil.getDetections();
        
        for (AprilTagDetection detection : backTags) {
            if (detection.metadata != null) {
                backDetections++;
                double confidence = calculateAprilTagConfidence(detection);
                backBestConfidence = Math.max(backBestConfidence, confidence);
            }
        }
        
        // Choose the better camera based on number of detections and confidence
        boolean useFront = (frontDetections > backDetections) || 
                          (frontDetections == backDetections && frontBestConfidence > backBestConfidence);
        
        if (useFront) {
            switchToFrontCamera();
            return frontDetections > 0;
        } else {
            switchToBackCamera();
            return backDetections > 0;
        }
    }
    
    /**
     * Get information about current camera setup
     */
    public String getCameraInfo() {
        if (!useDualCameras) {
            return "Single camera: " + frontCameraName;
        } else {
            return String.format("Dual cameras - Front: %s (%.1f,%.1f,%.1f)@%.0f°, Back: %s (%.1f,%.1f,%.1f)@%.0f°",
                    frontCameraName, frontCameraX, frontCameraY, frontCameraZ, frontCameraYaw,
                    backCameraName, backCameraX, backCameraY, backCameraZ, backCameraYaw);
        }
    }
    
    /**
     * Check if using dual cameras
     */
    public boolean isUsingDualCameras() {
        return useDualCameras;
    }
    
    /**
     * Get current AprilTag detection status from both cameras (if dual camera setup)
     * Note: This method temporarily switches cameras for status checking
     */
    public String getAprilTagStatus() {
        if (aprilTagUtil == null) return "No cameras available";
        
        if (useDualCameras) {
            StringBuilder status = new StringBuilder();
            String originalCamera = currentActiveCamera;
            
            // Check front camera
            switchToFrontCamera();
            List<AprilTagDetection> frontTags = aprilTagUtil.getDetections();
            status.append("Front: ").append(frontTags.size()).append(" tags");
            
            // Check back camera  
            switchToBackCamera();
            List<AprilTagDetection> backTags = aprilTagUtil.getDetections();
            status.append(", Back: ").append(backTags.size()).append(" tags");
            
            // Restore original camera
            if (originalCamera.equals("front")) {
                switchToFrontCamera();
            } else {
                switchToBackCamera();
            }
            
            status.append(" (Active: ").append(currentActiveCamera).append(")");
            return status.toString();
        } else {
            List<AprilTagDetection> tags = aprilTagUtil.getDetections();
            return "Single: " + tags.size() + " tags";
        }
    }
    
    /**
     * Configure camera switching behavior
     * @param minSwitchIntervalSeconds Minimum time between camera switches (default: 0.5 seconds)
     */
    public void setCameraSwitchingStrategy(double minSwitchIntervalSeconds) {
        this.minSwitchInterval = minSwitchIntervalSeconds;
    }
    
    /**
     * Get current active camera name
     */
    public String getCurrentActiveCamera() {
        if (!useDualCameras) return frontCameraName;
        return currentActiveCamera.equals("front") ? frontCameraName : backCameraName;
    }
    
    /**
     * Force switch to specific camera (bypasses switching interval)
     */
    public void forceSwitchToCamera(String camera) {
        if (useDualCameras && aprilTagUtil != null) {
            if (camera.equalsIgnoreCase("front") && frontCameraName != null) {
                switchToFrontCamera();
                lastCameraSwitchTime.reset();
            } else if (camera.equalsIgnoreCase("back") && backCameraName != null) {
                switchToBackCamera();
                lastCameraSwitchTime.reset();
            }
        }
    }
    
    /**
     * Optionally integrate with existing MechanumDrive class
     * Call this if you want APH to use your existing MechanumDrive class for motor control
     * @param mechanumDrive Your existing MechanumDrive instance (pass null to use built-in control)
     */
    public void setMechanumDriveClass(MechanumDrive mechanumDrive) {
        this.mechanumDrive = mechanumDrive;
    }
    
    /**
     * Check if using existing MechanumDrive class
     */
    public boolean isUsingMechanumDriveClass() {
        return mechanumDrive != null;
    }
    
    /**
     * Get formatted position string for display
     */
    public String getFormattedPosition() {
        return String.format("(%.1f, %.1f, %.0f°)", getCurrentX(), getCurrentY(), getCurrentHeading());
    }
    
    /**
     * Get distance to a target position
     */
    public double getDistanceToPosition(double targetX, double targetY) {
        double deltaX = targetX - getCurrentX();
        double deltaY = targetY - getCurrentY();
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
}