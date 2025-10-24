package org.firstinspires.ftc.teamcode.util.auroraone.config;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.auroraone.subsystems.localization.VisionLocalizer;
import java.util.HashMap;
import java.util.Map;

/**
 * AURORA ONE - Advanced Unified Robot Operating & Response Architecture One
 * Robot Hardware Mapping System - Centralized hardware component management
 *
 * This class provides a unified interface for accessing all robot hardware components
 * used by the Aurora One State Machine system. It handles hardware initialization,
 * error management, and provides type-safe access to all robot components.
 *
 * Based on the existing Aurora package hardware mappings and enhanced for the
 * Aurora One State Machine architecture.
 */
public class RobotMap {

    // Hardware map reference
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // === DRIVE SYSTEM ===
    // Mecanum drive motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // === SHOOTER SYSTEM ===
    public DcMotor shooter;
    public CRServo feedServo1;  // servo1
    public CRServo feedServo2;  // servo2

    // === ADDITIONAL ACTUATORS ===
    public DcMotor arm;
    public DcMotor intake;
    public DcMotor lift;

    // === SENSORS ===
    // Distance sensors
    public DistanceSensor frontDistance;
    public DistanceSensor backDistance;
    public DistanceSensor leftDistance;
    public DistanceSensor rightDistance;

    // Color sensors
    public ColorSensor colorSensor;
    public ColorSensor leftColor;
    public ColorSensor rightColor;

    // Touch sensors
    public TouchSensor touchSensor;
    public TouchSensor limitSwitch;
    public TouchSensor bumper;

    // IMU
    public IMU imu;

    // === VISION SYSTEM ===
    public WebcamName primaryWebcam;
    public WebcamName secondaryWebcam;
    public VisionLocalizer visionLocalizer;

    // === POWER MANAGEMENT ===
    public VoltageSensor voltageSensor;

    // === SERVO COLLECTIONS ===
    private Map<String, Servo> servos = new HashMap<>();
    private Map<String, CRServo> crServos = new HashMap<>();

    // === MOTOR COLLECTIONS ===
    private Map<String, DcMotor> motors = new HashMap<>();
    private Map<String, DcMotorEx> motorExs = new HashMap<>();

    // === SENSOR COLLECTIONS ===
    private Map<String, DistanceSensor> distanceSensors = new HashMap<>();
    private Map<String, ColorSensor> colorSensors = new HashMap<>();
    private Map<String, TouchSensor> touchSensors = new HashMap<>();

    // === INITIALIZATION STATUS ===
    private boolean driveSystemInitialized = false;
    private boolean shooterSystemInitialized = false;
    private boolean sensorSystemInitialized = false;
    private boolean visionSystemInitialized = false;

    // === ERROR TRACKING ===
    private Map<String, String> initializationErrors = new HashMap<>();

    /**
     * Initialize the robot hardware map
     * @param hardwareMap The FTC hardware map
     * @param telemetry Telemetry for status reporting
     */
    public RobotMap(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initializeAllSystems();
    }

    /**
     * Initialize all robot hardware systems
     */
    private void initializeAllSystems() {
        telemetry.addLine("🤖 Aurora One Hardware Initialization Starting...");
        telemetry.update();

        initializeDriveSystem();
        initializeShooterSystem();
        initializeActuators();
        initializeSensors();
        initializeVisionSystem();
        initializePowerManagement();

        telemetry.addLine("✅ Aurora One Hardware Initialization Complete");
        telemetry.update();
    }

    /**
     * Initialize mecanum drive system
     */
    private void initializeDriveSystem() {
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            // Configure motor directions (adjust based on your robot)
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Add to collections
            motorExs.put("frontLeft", frontLeft);
            motorExs.put("frontRight", frontRight);
            motorExs.put("backLeft", backLeft);
            motorExs.put("backRight", backRight);

            driveSystemInitialized = true;
            telemetry.addLine("✅ Drive system initialized successfully");

        } catch (Exception e) {
            driveSystemInitialized = false;
            String error = "Drive system initialization failed: " + e.getMessage();
            initializationErrors.put("DriveSystem", error);
            telemetry.addLine("⚠️ " + error);
            telemetry.addLine("   Expected motor names: frontLeft, frontRight, backLeft, backRight");
        }
    }

    /**
     * Initialize shooter system
     */
    private void initializeShooterSystem() {
        try {
            shooter = hardwareMap.get(DcMotor.class, "shooter");
            feedServo1 = hardwareMap.get(CRServo.class, "servo1");
            feedServo2 = hardwareMap.get(CRServo.class, "servo2");

            // Configure shooter motor
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Add to collections
            motors.put("shooter", shooter);
            crServos.put("servo1", feedServo1);
            crServos.put("servo2", feedServo2);

            shooterSystemInitialized = true;
            telemetry.addLine("✅ Shooter system initialized successfully");

        } catch (Exception e) {
            shooterSystemInitialized = false;
            String error = "Shooter system initialization failed: " + e.getMessage();
            initializationErrors.put("ShooterSystem", error);
            telemetry.addLine("⚠️ " + error);
            telemetry.addLine("   Expected components: shooter, servo1, servo2");
        }
    }

    /**
     * Initialize additional actuators
     */
    private void initializeActuators() {
        // Try to initialize common motors
        String[] motorNames = {"arm", "intake", "lift"};
        for (String name : motorNames) {
            try {
                DcMotor motor = hardwareMap.get(DcMotor.class, name);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors.put(name, motor);

                // Set specific motor references
                switch (name) {
                    case "arm":
                        arm = motor;
                        break;
                    case "intake":
                        intake = motor;
                        break;
                    case "lift":
                        lift = motor;
                        break;
                }

                telemetry.addLine("✅ Initialized motor: " + name);
            } catch (Exception e) {
                telemetry.addLine("⚠️ Motor '" + name + "' not found (optional)");
            }
        }

        // Try to initialize common servos
        String[] servoNames = {"clawServo", "armServo", "liftServo"};
        for (String name : servoNames) {
            try {
                Servo servo = hardwareMap.get(Servo.class, name);
                servos.put(name, servo);
                telemetry.addLine("✅ Initialized servo: " + name);
            } catch (Exception e) {
                telemetry.addLine("⚠️ Servo '" + name + "' not found (optional)");
            }
        }
    }

    /**
     * Initialize sensor systems
     */
    private void initializeSensors() {
        try {
            // Distance sensors
            String[] distanceNames = {"frontDistance", "backDistance", "leftDistance", "rightDistance"};
            for (String name : distanceNames) {
                try {
                    DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, name);
                    distanceSensors.put(name, sensor);

                    // Set specific sensor references
                    switch (name) {
                        case "frontDistance":
                            frontDistance = sensor;
                            break;
                        case "backDistance":
                            backDistance = sensor;
                            break;
                        case "leftDistance":
                            leftDistance = sensor;
                            break;
                        case "rightDistance":
                            rightDistance = sensor;
                            break;
                    }

                    telemetry.addLine("✅ Initialized distance sensor: " + name);
                } catch (Exception e) {
                    telemetry.addLine("⚠️ Distance sensor '" + name + "' not found (optional)");
                }
            }

            // Color sensors
            String[] colorNames = {"colorSensor", "leftColor", "rightColor"};
            for (String name : colorNames) {
                try {
                    ColorSensor sensor = hardwareMap.get(ColorSensor.class, name);
                    colorSensors.put(name, sensor);

                    // Set specific sensor references
                    switch (name) {
                        case "colorSensor":
                            colorSensor = sensor;
                            break;
                        case "leftColor":
                            leftColor = sensor;
                            break;
                        case "rightColor":
                            rightColor = sensor;
                            break;
                    }

                    telemetry.addLine("✅ Initialized color sensor: " + name);
                } catch (Exception e) {
                    telemetry.addLine("⚠️ Color sensor '" + name + "' not found (optional)");
                }
            }

            // Touch sensors
            String[] touchNames = {"touchSensor", "limitSwitch", "bumper"};
            for (String name : touchNames) {
                try {
                    TouchSensor sensor = hardwareMap.get(TouchSensor.class, name);
                    touchSensors.put(name, sensor);

                    // Set specific sensor references
                    switch (name) {
                        case "touchSensor":
                            touchSensor = sensor;
                            break;
                        case "limitSwitch":
                            limitSwitch = sensor;
                            break;
                        case "bumper":
                            bumper = sensor;
                            break;
                    }

                    telemetry.addLine("✅ Initialized touch sensor: " + name);
                } catch (Exception e) {
                    telemetry.addLine("⚠️ Touch sensor '" + name + "' not found (optional)");
                }
            }

            // IMU
            try {
                imu = hardwareMap.get(IMU.class, "imu");
                telemetry.addLine("✅ IMU initialized successfully");
            } catch (Exception e) {
                telemetry.addLine("⚠️ IMU not found (optional)");
            }

            sensorSystemInitialized = true;

        } catch (Exception e) {
            sensorSystemInitialized = false;
            String error = "Sensor system initialization error: " + e.getMessage();
            initializationErrors.put("SensorSystem", error);
            telemetry.addLine("⚠️ " + error);
        }
    }

    /**
     * Initialize vision system including VisionLocalizer
     */
    private void initializeVisionSystem() {
        try {
            // Primary webcam
            try {
                primaryWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
                telemetry.addLine("✅ Primary webcam initialized");
            } catch (Exception e) {
                telemetry.addLine("⚠️ Primary webcam 'Webcam 1' not found (optional)");
            }

            // Secondary webcam
            try {
                secondaryWebcam = hardwareMap.get(WebcamName.class, "Webcam 2");
                telemetry.addLine("✅ Secondary webcam initialized");
            } catch (Exception e) {
                telemetry.addLine("⚠️ Secondary webcam 'Webcam 2' not found (optional)");
            }

            // Initialize VisionLocalizer if cameras are available
            if (primaryWebcam != null || secondaryWebcam != null) {
                try {
                    visionLocalizer = new VisionLocalizer(this, telemetry);
                    telemetry.addLine("✅ VisionLocalizer initialized");
                } catch (Exception e) {
                    telemetry.addLine("⚠️ VisionLocalizer initialization failed: " + e.getMessage());
                }
            } else {
                telemetry.addLine("⚠️ No cameras available for VisionLocalizer");
            }

            visionSystemInitialized = true;

        } catch (Exception e) {
            visionSystemInitialized = false;
            String error = "Vision system initialization error: " + e.getMessage();
            initializationErrors.put("VisionSystem", error);
            telemetry.addLine("⚠️ " + error);
        }
    }

    /**
     * Initialize power management
     */
    private void initializePowerManagement() {
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            telemetry.addLine("✅ Voltage sensor initialized");
        } catch (Exception e) {
            telemetry.addLine("⚠️ Voltage sensor not found (optional)");
        }
    }

    // === GETTER METHODS ===

    /**
     * Get the hardware map for direct access when needed
     */
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public boolean isDriveSystemReady() {
        return driveSystemInitialized;
    }

    public boolean isShooterSystemReady() {
        return shooterSystemInitialized;
    }

    public boolean isSensorSystemReady() {
        return sensorSystemInitialized;
    }

    public boolean isVisionSystemReady() {
        return visionSystemInitialized;
    }

    /**
     * Get the VisionLocalizer instance
     */
    public VisionLocalizer getVisionLocalizer() {
        return visionLocalizer;
    }

    /**
     * Get a motor by name from the motor collection
     */
    public DcMotor getMotor(String name) {
        return motors.get(name);
    }

    /**
     * Get a motor ex by name from the motor ex collection
     */
    public DcMotorEx getMotorEx(String name) {
        return motorExs.get(name);
    }

    /**
     * Get a servo by name from the servo collection
     */
    public Servo getServo(String name) {
        return servos.get(name);
    }

    /**
     * Get a CR servo by name from the CR servo collection
     */
    public CRServo getCRServo(String name) {
        return crServos.get(name);
    }

    /**
     * Get a distance sensor by name
     */
    public DistanceSensor getDistanceSensor(String name) {
        return distanceSensors.get(name);
    }

    /**
     * Get a color sensor by name
     */
    public ColorSensor getColorSensor(String name) {
        return colorSensors.get(name);
    }

    /**
     * Get a touch sensor by name
     */
    public TouchSensor getTouchSensor(String name) {
        return touchSensors.get(name);
    }

    /**
     * Get all initialization errors
     */
    public Map<String, String> getInitializationErrors() {
        return new HashMap<>(initializationErrors);
    }

    /**
     * Check if any critical systems failed to initialize
     */
    public boolean hasCriticalErrors() {
        return !driveSystemInitialized && !shooterSystemInitialized;
    }

    /**
     * Get system status summary
     */
    public String getSystemStatus() {
        StringBuilder status = new StringBuilder();
        status.append("Aurora One Hardware Status:\n");
        status.append("Drive System: ").append(driveSystemInitialized ? "✅ Ready" : "❌ Failed").append("\n");
        status.append("Shooter System: ").append(shooterSystemInitialized ? "✅ Ready" : "❌ Failed").append("\n");
        status.append("Sensor System: ").append(sensorSystemInitialized ? "✅ Ready" : "⚠️ Partial").append("\n");
        status.append("Vision System: ").append(visionSystemInitialized ? "✅ Ready" : "⚠️ Partial").append("\n");

        if (!initializationErrors.isEmpty()) {
            status.append("\nErrors:\n");
            for (Map.Entry<String, String> error : initializationErrors.entrySet()) {
                status.append("- ").append(error.getKey()).append(": ").append(error.getValue()).append("\n");
            }
        }

        return status.toString();
    }
}
