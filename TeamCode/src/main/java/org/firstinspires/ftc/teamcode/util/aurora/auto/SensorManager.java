package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;
import java.util.Map;

// Used to manage various sensors(like distance, color, touch, etc) on the robot for autonomous mode
public class SensorManager {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Sensor collections
    private Map<String, DistanceSensor> distanceSensors;
    private Map<String, ColorSensor> colorSensors;
    private Map<String, TouchSensor> touchSensors;
    private IMU imu;

    // Sensor data cache
    private Map<String, Double> lastDistanceReadings;
    private Map<String, Integer> lastColorReadings;
    private Map<String, Boolean> lastTouchReadings;

    /**
     * Initialize sensor manager with hardware map
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     */
    public SensorManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Initialize collections
        distanceSensors = new HashMap<>();
        colorSensors = new HashMap<>();
        touchSensors = new HashMap<>();
        lastDistanceReadings = new HashMap<>();
        lastColorReadings = new HashMap<>();
        lastTouchReadings = new HashMap<>();

        initializeSensors();
    }

    /**
     * Initialize all available sensors from hardware map
     */
    private void initializeSensors() {
        // Try to initialize common distance sensors
        String[] distanceSensorNames = {"frontDistance", "backDistance", "leftDistance", "rightDistance"};
        for (String name : distanceSensorNames) {
            try {
                DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, name);
                distanceSensors.put(name, sensor);
                lastDistanceReadings.put(name, 0.0);
                telemetry.addData("SensorManager", "Initialized distance sensor: " + name);
            } catch (Exception e) {
                // Sensor not found, continue
            }
        }

        // Try to initialize color sensors
        String[] colorSensorNames = {"colorSensor", "leftColor", "rightColor"};
        for (String name : colorSensorNames) {
            try {
                ColorSensor sensor = hardwareMap.get(ColorSensor.class, name);
                colorSensors.put(name, sensor);
                lastColorReadings.put(name, 0);
                telemetry.addData("SensorManager", "Initialized color sensor: " + name);
            } catch (Exception e) {
                // Sensor not found, continue
            }
        }

        // Try to initialize touch sensors
        String[] touchSensorNames = {"touchSensor", "limitSwitch", "bumper"};
        for (String name : touchSensorNames) {
            try {
                TouchSensor sensor = hardwareMap.get(TouchSensor.class, name);
                touchSensors.put(name, sensor);
                lastTouchReadings.put(name, false);
                telemetry.addData("SensorManager", "Initialized touch sensor: " + name);
            } catch (Exception e) {
                // Sensor not found, continue
            }
        }

        // Try to initialize IMU
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            telemetry.addData("SensorManager", "Initialized IMU");
        } catch (Exception e) {
            telemetry.addData("SensorManager", "IMU not found");
        }
    }

    /**
     * Update all sensor readings
     */
    public void updateSensors() {
        // Update distance sensors
        for (Map.Entry<String, DistanceSensor> entry : distanceSensors.entrySet()) {
            double distance = entry.getValue().getDistance(DistanceUnit.INCH);
            lastDistanceReadings.put(entry.getKey(), distance);
        }

        // Update color sensors
        for (Map.Entry<String, ColorSensor> entry : colorSensors.entrySet()) {
            int red = entry.getValue().red();
            int green = entry.getValue().green();
            int blue = entry.getValue().blue();
            // Store combined RGB value for simplicity
            int combined = (red << 16) | (green << 8) | blue;
            lastColorReadings.put(entry.getKey(), combined);
        }

        // Update touch sensors
        for (Map.Entry<String, TouchSensor> entry : touchSensors.entrySet()) {
            boolean pressed = entry.getValue().isPressed();
            lastTouchReadings.put(entry.getKey(), pressed);
        }
    }

    /**
     * Get distance reading from specified sensor
     * @param sensorName Name of distance sensor
     * @return Distance in inches, or -1 if sensor not found
     */
    public double getDistance(String sensorName) {
        return lastDistanceReadings.getOrDefault(sensorName, -1.0);
    }

    /**
     * Get color reading from specified sensor
     * @param sensorName Name of color sensor
     * @param colorComponent "red", "green", "blue", or "alpha"
     * @return Color component value (0-255)
     */
    public int getColor(String sensorName, String colorComponent) {
        if (!colorSensors.containsKey(sensorName)) return 0;

        ColorSensor sensor = colorSensors.get(sensorName);
        switch (colorComponent.toLowerCase()) {
            case "red": return sensor.red();
            case "green": return sensor.green();
            case "blue": return sensor.blue();
            case "alpha": return sensor.alpha();
            default: return 0;
        }
    }

    /**
     * Check if touch sensor is pressed
     * @param sensorName Name of touch sensor
     * @return True if pressed, false otherwise
     */
    public boolean isTouchPressed(String sensorName) {
        return lastTouchReadings.getOrDefault(sensorName, false);
    }

    /**
     * Get IMU heading
     * @return Heading in degrees, or 0 if IMU not available
     */
    public double getIMUHeading() {
        if (imu == null) return 0.0;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Check if obstacle is detected within specified distance
     * @param sensorName Name of distance sensor
     * @param threshold Distance threshold in inches
     * @return True if obstacle detected within threshold
     */
    public boolean isObstacleDetected(String sensorName, double threshold) {
        double distance = getDistance(sensorName);
        return distance > 0 && distance < threshold;
    }

    /**
     * Detect if robot is near a wall
     * @param threshold Distance threshold in inches
     * @return True if any distance sensor detects a wall within threshold
     */
    public boolean isNearWall(double threshold) {
        for (String sensorName : distanceSensors.keySet()) {
            if (isObstacleDetected(sensorName, threshold)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Get the closest obstacle distance and direction
     * @return Array with [distance, direction] where direction is sensor index
     */
    public double[] getClosestObstacle() {
        double minDistance = Double.MAX_VALUE;
        String closestSensor = "";

        for (Map.Entry<String, Double> entry : lastDistanceReadings.entrySet()) {
            if (entry.getValue() > 0 && entry.getValue() < minDistance) {
                minDistance = entry.getValue();
                closestSensor = entry.getKey();
            }
        }

        return new double[]{minDistance, getSensorDirection(closestSensor)};
    }

    /**
     * Get direction angle for sensor name
     * @param sensorName Name of sensor
     * @return Angle in degrees (0=front, 90=right, 180=back, 270=left)
     */
    private double getSensorDirection(String sensorName) {
        switch (sensorName.toLowerCase()) {
            case "frontdistance": return 0;
            case "rightdistance": return 90;
            case "backdistance": return 180;
            case "leftdistance": return 270;
            default: return 0;
        }
    }

    /**
     * Add sensor telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== SENSOR DATA ===", "");

        // Distance sensors
        for (Map.Entry<String, Double> entry : lastDistanceReadings.entrySet()) {
            telemetry.addData(entry.getKey() + " Distance", "%.2f in", entry.getValue());
        }

        // Color sensors
        for (String sensorName : colorSensors.keySet()) {
            ColorSensor sensor = colorSensors.get(sensorName);
            telemetry.addData(sensorName + " Color", "R:%d G:%d B:%d",
                sensor.red(), sensor.green(), sensor.blue());
        }

        // Touch sensors
        for (Map.Entry<String, Boolean> entry : lastTouchReadings.entrySet()) {
            telemetry.addData(entry.getKey() + " Touch", entry.getValue() ? "PRESSED" : "Released");
        }

        // IMU
        if (imu != null) {
            telemetry.addData("IMU Heading", "%.2f deg", getIMUHeading());
        }
    }

    /**
     * Check if all critical sensors are working
     * @return True if sensors are responding
     */
    public boolean areSensorsReady() {
        // Check if at least one sensor type is available
        return !distanceSensors.isEmpty() || !colorSensors.isEmpty() || !touchSensors.isEmpty();
    }
}
