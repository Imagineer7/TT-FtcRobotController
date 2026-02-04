package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;

import java.util.Locale;

/**
 * SensorMappingDiagnostic - Diagnostic OpMode for sensor identification
 *
 * This OpMode displays raw sensor readings from ALL color sensors and distance sensors
 * to help identify which physical sensor corresponds to which hardware mapping.
 *
 * Use this to debug sensor configuration issues:
 * 1. Run the OpMode
 * 2. Place an artifact in front of each sensor one at a time
 * 3. Watch which telemetry lines change
 * 4. Match physical location to hardware name
 * 5. Update AuroraHardwareConfig mappings if needed
 *
 * Controls:
 * - A: Cycle through telemetry pages
 * - X: Toggle detailed color RGB display
 *
 * Displays:
 * Page 1: Front Intake Sensors (Left/Right)
 * Page 2: Back Intake Sensors (Left/Right)
 * Page 3: Center Slot Sensors (Left/Right)
 * Page 4: Color Sensor Summary
 */
@TeleOp(name="Diagnostic: Sensor Mapping", group="Testing")
public class SensorMappingDiagnostic extends LinearOpMode {

    private AuroraHardwareConfig hardware;

    // Sensor references
    private AnalogInput frontDistanceSensor;
    private AnalogInput backDistanceSensor;
    private NormalizedColorSensor frontLeftColorSensor;
    private NormalizedColorSensor frontRightColorSensor;
    private NormalizedColorSensor backLeftColorSensor;
    private NormalizedColorSensor backRightColorSensor;
    private NormalizedColorSensor centerLeftColorSensor;
    private NormalizedColorSensor centerRightColorSensor;

    // UI state
    private int currentPage = 0;
    private boolean showDetailedColors = false;
    private boolean lastAButton = false;
    private boolean lastXButton = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing sensors...");
        telemetry.update();

        // Initialize hardware config
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        // Get sensor references (use deprecated methods to access by name)
        initializeSensors();

        telemetry.addData("Status", "Ready!");
        telemetry.addData("Controls", "A=Next page, X=Toggle details");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            handleInput();
            displayTelemetry();
            telemetry.update();
        }
    }

    private void initializeSensors() {
        try {
            HardwareMap hardwareMap = hardware.getHardwareMap();

            // Distance sensors - Try common device names
            frontDistanceSensor = tryGetAnalogInput(hardwareMap,
                "LaserSensorFront", "FrontDistanceSensor", "DistanceSensorFront");
            backDistanceSensor = tryGetAnalogInput(hardwareMap,
                "LaserSensorBack", "BackDistanceSensor", "DistanceSensorBack");

            // Front color sensors - Try common device names
            frontLeftColorSensor = tryGetColorSensor(hardwareMap,
                "ColorSensorLeftFront", "ColorSensorFrontLeft", "FrontLeftColor");
            frontRightColorSensor = tryGetColorSensor(hardwareMap,
                "ColorSensorRightFront", "ColorSensorFrontRight", "FrontRightColor");

            // Back color sensors - Try common device names
            backLeftColorSensor = tryGetColorSensor(hardwareMap,
                "ColorSensorLeftBack", "ColorSensorBackLeft", "BackLeftColor");
            backRightColorSensor = tryGetColorSensor(hardwareMap,
                "ColorSensorRightBack", "ColorSensorBackRight", "BackRightColor");

            // Center slot color sensors - Try common device names
            centerLeftColorSensor = tryGetColorSensor(hardwareMap,
                "ColorSensorCL", "ColorSensorCenterLeft", "CenterLeftColor");
            centerRightColorSensor = tryGetColorSensor(hardwareMap,
                "ColorSensorCR", "ColorSensorCenterRight", "CenterRightColor");

            telemetry.addData("✓", "Sensor discovery complete");
        } catch (Exception e) {
            telemetry.addData("❌", "Sensor init error: " + e.getMessage());
        }
    }

    /**
     * Try to get an analog input sensor with multiple possible names
     */
    private AnalogInput tryGetAnalogInput(HardwareMap hardwareMap, String... possibleNames) {
        for (String name : possibleNames) {
            try {
                return hardwareMap.get(AnalogInput.class, name);
            } catch (Exception e) {
                // Try next name
            }
        }
        return null;  // Not found
    }

    /**
     * Try to get a color sensor with multiple possible names
     */
    private NormalizedColorSensor tryGetColorSensor(HardwareMap hardwareMap, String... possibleNames) {
        for (String name : possibleNames) {
            try {
                return hardwareMap.get(NormalizedColorSensor.class, name);
            } catch (Exception e) {
                // Try next name
            }
        }
        return null;  // Not found
    }

    private void handleInput() {
        // A button: next page
        boolean currentA = gamepad1.a;
        if (currentA && !lastAButton) {
            currentPage = (currentPage + 1) % 4;
        }
        lastAButton = currentA;

        // X button: toggle detailed colors
        boolean currentX = gamepad1.x;
        if (currentX && !lastXButton) {
            showDetailedColors = !showDetailedColors;
        }
        lastXButton = currentX;
    }

    private void displayTelemetry() {
        telemetry.addData("════════════════════════════════════", "");
        telemetry.addData("Page", (currentPage + 1) + " of 4");
        telemetry.addData("Mode", showDetailedColors ? "DETAILED" : "SUMMARY");
        telemetry.addData("════════════════════════════════════", "");

        switch (currentPage) {
            case 0:
                displayFrontIntakePage();
                break;
            case 1:
                displayBackIntakePage();
                break;
            case 2:
                displayCenterSlotPage();
                break;
            case 3:
                displayColorSensorSummaryPage();
                break;
        }

        // Footer
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("  A", "Next page");
        telemetry.addData("  X", "Toggle details");
    }

    private void displayFrontIntakePage() {
        telemetry.addData("FRONT INTAKE SENSORS", "");
        telemetry.addData("", "");

        // Front distance sensor
        telemetry.addData("Front Distance Sensor (Analog Laser)", "");
        if (frontDistanceSensor != null) {
            try {
                double voltage = frontDistanceSensor.getVoltage();
                double distanceMm = (voltage / 3.3) * 1000.0;
                double distanceCm = distanceMm / 10.0;
                telemetry.addData("  Voltage", String.format(Locale.US, "%.2f V", voltage));
                telemetry.addData("  Distance", String.format(Locale.US, "%.1f cm", distanceCm));
                telemetry.addData("  Status", distanceCm < 10.0 ? "✓ ARTIFACT NEAR" : "✗ CLEAR");
            } catch (Exception e) {
                telemetry.addData("  ERROR", e.getMessage());
            }
        } else {
            telemetry.addData("  ERROR", "Sensor not found");
        }

        telemetry.addData("", "");

        // Front left color sensor
        telemetry.addData("Front Left Color Sensor", "");
        displayColorSensorData(frontLeftColorSensor, showDetailedColors);


        telemetry.addData("", "");

        // Front right color sensor
        telemetry.addData("Front Right Color Sensor", "");
        displayColorSensorData(frontRightColorSensor, showDetailedColors);
    }

    private void displayBackIntakePage() {
        telemetry.addData("BACK INTAKE SENSORS", "");
        telemetry.addData("", "");

        // Back distance sensor
        telemetry.addData("Back Distance Sensor (Analog Laser)", "");
        if (backDistanceSensor != null) {
            try {
                double voltage = backDistanceSensor.getVoltage();
                double distanceMm = (voltage / 3.3) * 1000.0;
                double distanceCm = distanceMm / 10.0;
                telemetry.addData("  Voltage", String.format(Locale.US, "%.2f V", voltage));
                telemetry.addData("  Distance", String.format(Locale.US, "%.1f cm", distanceCm));
                telemetry.addData("  Status", distanceCm < 10.0 ? "✓ ARTIFACT NEAR" : "✗ CLEAR");
            } catch (Exception e) {
                telemetry.addData("  ERROR", e.getMessage());
            }
        } else {
            telemetry.addData("  ERROR", "Sensor not found");
        }

        telemetry.addData("", "");

        // Back left color sensor
        telemetry.addData("Back Left Color Sensor", "");
        displayColorSensorData(backLeftColorSensor, showDetailedColors);


        telemetry.addData("", "");

        // Back right color sensor
        telemetry.addData("Back Right Color Sensor", "");
        displayColorSensorData(backRightColorSensor, showDetailedColors);
    }

    private void displayCenterSlotPage() {
        telemetry.addData("CENTER SLOT SENSORS", "");
        telemetry.addData("", "");

        // Center left color sensor
        telemetry.addData("Center Left Color Sensor", "");
        displayColorSensorData(centerLeftColorSensor, showDetailedColors);

        telemetry.addData("", "");

        // Center right color sensor
        telemetry.addData("Center Right Color Sensor", "");
        displayColorSensorData(centerRightColorSensor, showDetailedColors);
    }

    private void displayColorSensorSummaryPage() {
        telemetry.addData("SENSOR MAPPING SUMMARY", "");
        telemetry.addData("", "");
        telemetry.addData("Instructions:", "");
        telemetry.addData("1.", "Point a colored artifact at each sensor");
        telemetry.addData("2.", "Watch which line shows color change");
        telemetry.addData("3.", "Record the sensor location");
        telemetry.addData("4.", "Update AuroraHardwareConfig if needed");
        telemetry.addData("", "");

        telemetry.addData("Front Intake Sensors", "");
        telemetry.addData("  Left", getSensorStatus(frontLeftColorSensor));
        telemetry.addData("  Right", getSensorStatus(frontRightColorSensor));

        telemetry.addData("", "");

        telemetry.addData("Back Intake Sensors", "");
        telemetry.addData("  Left", getSensorStatus(backLeftColorSensor));
        telemetry.addData("  Right", getSensorStatus(backRightColorSensor));

        telemetry.addData("", "");

        telemetry.addData("Center Slot Sensors", "");
        telemetry.addData("  Left", getSensorStatus(centerLeftColorSensor));
        telemetry.addData("  Right", getSensorStatus(centerRightColorSensor));

        telemetry.addData("", "");
        telemetry.addData("Troubleshooting:", "");
        telemetry.addData("Q: No sensors detected?", "Check hardware map config");
        telemetry.addData("Q: Wrong locations?", "Update hardware getter methods");
        telemetry.addData("Q: No proximity values?", "REV sensors may not support DistanceSensor");
    }

    /**
     * Display color sensor data (RGB and proximity)
     */
    private void displayColorSensorData(NormalizedColorSensor sensor, boolean detailed) {
        if (sensor == null) {
            telemetry.addData("  ERROR", "Sensor not found");
            return;
        }

        try {
            // Get RGB values
            double red = sensor.getNormalizedColors().red;
            double green = sensor.getNormalizedColors().green;
            double blue = sensor.getNormalizedColors().blue;

            if (detailed) {
                // Detailed mode: show all values
                telemetry.addData("  RGB (Normalized)", String.format(Locale.US, "R:%.3f G:%.3f B:%.3f", red, green, blue));
            } else {
                // Summary mode: show RGB compactly
                telemetry.addData("  RGB", String.format(Locale.US, "R:%.2f G:%.2f B:%.2f", red, green, blue));
            }

            // Brightness indicator
            double brightness = Math.max(Math.max(red, green), blue);
            String brightnessIndicator = brightness < 0.1 ? "DARK" :
                                        brightness < 0.3 ? "LOW" :
                                        brightness < 0.6 ? "MEDIUM" :
                                        "BRIGHT";
            telemetry.addData("  Brightness", brightnessIndicator + String.format(Locale.US, " (%.2f)", brightness));

            // Color detection hint
            String colorHint = detectColorHint(red, green, blue);
            telemetry.addData("  Color", colorHint);

            // Proximity (if REV Color V3)
            if (sensor instanceof DistanceSensor) {
                try {
                    double distanceCm = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
                    String status = (distanceCm <= 7.0 && distanceCm > 0.1) ? " ✓ DETECTED" : " ✗ CLEAR";
                    telemetry.addData("  Proximity", String.format(Locale.US, "%.1f cm", distanceCm) + status);
                } catch (Exception e) {
                    telemetry.addData("  Proximity", "ERROR: " + e.getMessage());
                }
            } else {
                telemetry.addData("  Proximity", "N/A (not DistanceSensor)");
            }
        } catch (Exception e) {
            telemetry.addData("  ERROR", e.getMessage());
        }
    }

    /**
     * Get simple color hint from RGB values
     * Based on actual observations:
     * - Purple: R < G ≤ B (Red lowest, Blue highest)
     * - Green: R < B < G (Red lowest, Green highest, Blue between)
     */
    private String detectColorHint(double red, double green, double blue) {
        double max = Math.max(Math.max(red, green), blue);

        // Too dark to detect
        if (max < 0.05) return "TOO DARK";

        // Check ordering patterns
        // Purple: Blue is highest or tied with green, Red is lowest
        if (blue >= green && green > red && (blue - red) >= 0.08) {
            return "PURPLE-ish";
        }

        // Green: Green is highest, Blue is between, Red is lowest
        if (green > blue && blue > red) {
            return "GREEN-ish";
        }

        // Alternative green: Green highest but ordering may vary
        if (green > red && green > blue && (green - red) >= 0.1) {
            return "GREEN-ish";
        }

        // White/reflection: all values similar and high
        if (Math.abs(red - green) < 0.1 && Math.abs(green - blue) < 0.1 && max > 0.5) {
            return "WHITE/REFLECT";
        }

        return "UNKNOWN";
    }

    /**
     * Get sensor status string for summary view (including proximity)
     */
    private String getSensorStatus(NormalizedColorSensor sensor) {
        if (sensor == null) return "NOT FOUND";

        try {
            double red = sensor.getNormalizedColors().red;
            double green = sensor.getNormalizedColors().green;
            double blue = sensor.getNormalizedColors().blue;
            double max = Math.max(Math.max(red, green), blue);

            String colorStatus = (max < 0.05) ? "DARK" :
                String.format(Locale.US, "R:%.2f G:%.2f B:%.2f", red, green, blue);

            // Add proximity if available
            String proximityStatus = "";
            if (sensor instanceof DistanceSensor) {
                try {
                    double distanceCm = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
                    proximityStatus = String.format(Locale.US, " Prox:%.1f cm", distanceCm);
                } catch (Exception e) {
                    proximityStatus = " Prox:ERR";
                }
            }

            return colorStatus + proximityStatus;
        } catch (Exception e) {
            return "ERROR";
        }
    }
}
