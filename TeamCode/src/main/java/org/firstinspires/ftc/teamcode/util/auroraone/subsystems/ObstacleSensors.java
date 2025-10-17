package org.firstinspires.ftc.teamcode.util.auroraone.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.auroraone.config.RobotMap;
import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;
import org.firstinspires.ftc.teamcode.util.auroraone.core.Blackboard;

/**
 * AURORA ONE - Obstacle Sensors
 * Advanced obstacle detection and avoidance system
 *
 * This class is responsible for managing the robot's obstacle sensors.
 * It provides methods to read data from various obstacle detection sensors such as ultrasonic sensors,
 * infrared sensors, or bump sensors. It processes the sensor data to help the robot avoid collisions
 * and navigate around obstacles.
 *
 * Features:
 * - Multi-directional distance sensing
 * - Obstacle detection zones
 * - Emergency stop triggers
 * - Blackboard integration for state machine communication
 * - Performance monitoring and validation
 */
public class ObstacleSensors {

    // Hardware references
    private RobotMap robotMap;
    private Telemetry telemetry;
    private Blackboard blackboard;

    // Distance sensors
    private DistanceSensor frontDistance;
    private DistanceSensor backDistance;
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;

    // Sensor readings (in inches)
    private double frontDistanceValue = Double.MAX_VALUE;
    private double backDistanceValue = Double.MAX_VALUE;
    private double leftDistanceValue = Double.MAX_VALUE;
    private double rightDistanceValue = Double.MAX_VALUE;

    // Obstacle detection flags
    private boolean frontObstacle = false;
    private boolean backObstacle = false;
    private boolean leftObstacle = false;
    private boolean rightObstacle = false;
    private boolean emergencyStopTriggered = false;

    // Timing
    private ElapsedTime updateTimer = new ElapsedTime();
    private boolean isInitialized = false;

    /**
     * Constructor
     */
    public ObstacleSensors(RobotMap robotMap, Telemetry telemetry) {
        this.robotMap = robotMap;
        this.telemetry = telemetry;
        this.blackboard = Blackboard.getInstance();

        initialize();
    }

    /**
     * Initialize obstacle sensors
     */
    private void initialize() {
        try {
            // Get distance sensors from RobotMap
            frontDistance = robotMap.frontDistance;
            backDistance = robotMap.backDistance;
            leftDistance = robotMap.leftDistance;
            rightDistance = robotMap.rightDistance;

            isInitialized = (frontDistance != null || backDistance != null ||
                           leftDistance != null || rightDistance != null);

            if (isInitialized) {
                telemetry.addLine("✅ ObstacleSensors: Initialized successfully");
                updateTimer.reset();
            } else {
                telemetry.addLine("⚠️ ObstacleSensors: No distance sensors available");
            }

        } catch (Exception e) {
            isInitialized = false;
            telemetry.addLine("❌ ObstacleSensors: Initialization failed - " + e.getMessage());
        }
    }

    /**
     * Main update method - call this every loop
     */
    public void update() {
        if (!isInitialized) return;

        // Read sensor values
        updateSensorReadings();

        // Detect obstacles
        detectObstacles();

        // Check for emergency stop conditions
        checkEmergencyStop();

        // Update Blackboard
        updateBlackboard();
    }

    /**
     * Read values from all available distance sensors
     */
    private void updateSensorReadings() {
        // Read front distance sensor
        if (frontDistance != null) {
            try {
                frontDistanceValue = frontDistance.getDistance(DistanceUnit.INCH);
                if (frontDistanceValue > 120) frontDistanceValue = Double.MAX_VALUE; // Max reasonable range
            } catch (Exception e) {
                frontDistanceValue = Double.MAX_VALUE;
            }
        }

        // Read back distance sensor
        if (backDistance != null) {
            try {
                backDistanceValue = backDistance.getDistance(DistanceUnit.INCH);
                if (backDistanceValue > 120) backDistanceValue = Double.MAX_VALUE;
            } catch (Exception e) {
                backDistanceValue = Double.MAX_VALUE;
            }
        }

        // Read left distance sensor
        if (leftDistance != null) {
            try {
                leftDistanceValue = leftDistance.getDistance(DistanceUnit.INCH);
                if (leftDistanceValue > 120) leftDistanceValue = Double.MAX_VALUE;
            } catch (Exception e) {
                leftDistanceValue = Double.MAX_VALUE;
            }
        }

        // Read right distance sensor
        if (rightDistance != null) {
            try {
                rightDistanceValue = rightDistance.getDistance(DistanceUnit.INCH);
                if (rightDistanceValue > 120) rightDistanceValue = Double.MAX_VALUE;
            } catch (Exception e) {
                rightDistanceValue = Double.MAX_VALUE;
            }
        }
    }

    /**
     * Detect obstacles in each direction
     */
    private void detectObstacles() {
        double obstacleThreshold = 12.0; // 12 inches default threshold

        frontObstacle = (frontDistanceValue < obstacleThreshold);
        backObstacle = (backDistanceValue < obstacleThreshold);
        leftObstacle = (leftDistanceValue < obstacleThreshold);
        rightObstacle = (rightDistanceValue < obstacleThreshold);
    }

    /**
     * Check for emergency stop conditions
     */
    private void checkEmergencyStop() {
        double emergencyThreshold = 4.0; // 4 inches emergency threshold

        emergencyStopTriggered = (frontDistanceValue < emergencyThreshold) ||
                                (backDistanceValue < emergencyThreshold) ||
                                (leftDistanceValue < emergencyThreshold) ||
                                (rightDistanceValue < emergencyThreshold);
    }

    /**
     * Update Blackboard with sensor data
     */
    private void updateBlackboard() {
        // Distance sensor readings
        blackboard.put("sensors.distance.front", frontDistanceValue);
        blackboard.put("sensors.distance.back", backDistanceValue);
        blackboard.put("sensors.distance.left", leftDistanceValue);
        blackboard.put("sensors.distance.right", rightDistanceValue);

        // Obstacle detection flags
        blackboard.put("obstacles.front", frontObstacle);
        blackboard.put("obstacles.back", backObstacle);
        blackboard.put("obstacles.left", leftObstacle);
        blackboard.put("obstacles.right", rightObstacle);
        blackboard.put("obstacles.emergency_stop", emergencyStopTriggered);

        // Any obstacle detected
        blackboard.put("obstacles.any_detected", frontObstacle || backObstacle || leftObstacle || rightObstacle);

        // Sensor system status
        blackboard.put("hardware.sensors.ready", isInitialized);
        blackboard.put("sensors.obstacle.initialized", isInitialized);
    }

    // === Public Interface Methods ===

    public boolean isInitialized() { return isInitialized; }
    public boolean hasObstacle() { return frontObstacle || backObstacle || leftObstacle || rightObstacle; }
    public boolean isEmergencyStopTriggered() { return emergencyStopTriggered; }

    public double getFrontDistance() { return frontDistanceValue; }
    public double getBackDistance() { return backDistanceValue; }
    public double getLeftDistance() { return leftDistanceValue; }
    public double getRightDistance() { return rightDistanceValue; }

    public boolean isFrontObstacle() { return frontObstacle; }
    public boolean isBackObstacle() { return backObstacle; }
    public boolean isLeftObstacle() { return leftObstacle; }
    public boolean isRightObstacle() { return rightObstacle; }

    /**
     * Get status summary for debugging
     */
    public String getStatusSummary() {
        if (!isInitialized) {
            return "ObstacleSensors: Not initialized";
        }

        return String.format("ObstacleSensors: F:%.1f B:%.1f L:%.1f R:%.1f %s",
            frontDistanceValue == Double.MAX_VALUE ? 0.0 : frontDistanceValue,
            backDistanceValue == Double.MAX_VALUE ? 0.0 : backDistanceValue,
            leftDistanceValue == Double.MAX_VALUE ? 0.0 : leftDistanceValue,
            rightDistanceValue == Double.MAX_VALUE ? 0.0 : rightDistanceValue,
            emergencyStopTriggered ? "[EMERGENCY]" : (hasObstacle() ? "[OBSTACLE]" : "[CLEAR]"));
    }
}
