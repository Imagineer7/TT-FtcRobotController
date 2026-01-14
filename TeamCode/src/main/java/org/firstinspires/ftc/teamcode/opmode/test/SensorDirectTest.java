package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * BARE BONES SENSOR TEST
 *
 * This OpMode directly tests sensors WITHOUT using any custom classes.
 * It will help identify if sensors are properly configured in the Driver Station.
 *
 * IMPORTANT - Color Sensor Gain:
 * REV Color Sensors have a default gain that is too low for practical use.
 * This OpMode sets gain=50 which improves detection range significantly.
 * Without proper gain, colors are only detected when objects are 1-2mm away!
 *
 * NOTE: Front Left and Back Right color sensors are temporarily replaced
 * with REV 2m distance sensors. These will be swapped back to color sensors later.
 *
 * Instructions:
 * 1. Select this OpMode in Driver Station
 * 2. Press INIT
 * 3. Check telemetry to see which sensors were found
 * 4. Press PLAY to see live sensor readings
 * 5. Press STOP when done
 */
@TeleOp(name = "🔍 Sensor Direct Test", group = "Test")
public class SensorDirectTest extends LinearOpMode {

    // Distance sensors (goBILDA Laser Distance Sensors - Analog Mode)
    private AnalogInput frontDistanceSensor = null;
    private AnalogInput backDistanceSensor = null;

    // Color sensors (REV Color Sensor V3)
    private NormalizedColorSensor frontLeftColorSensor = null;
    private NormalizedColorSensor frontRightColorSensor = null;
    private NormalizedColorSensor backRightColorSensor = null;
    private NormalizedColorSensor leftRightColorSensor = null;
    private NormalizedColorSensor frontCenterColorSensor = null;
    private NormalizedColorSensor backCenterColorSensor = null;

    // Distance sensor calibration (goBILDA laser sensors)
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    @Override
    public void runOpMode() {
        telemetry.addLine("🔍 SENSOR DIRECT TEST");
        telemetry.addLine("====================");
        telemetry.addLine("");
        telemetry.addLine("Attempting to find all sensors...");
        telemetry.update();

        // List all available hardware devices
        telemetry.addLine("\n📋 ALL HARDWARE DEVICES:");
        for (HardwareDevice device : hardwareMap) {
            telemetry.addLine("  - " + hardwareMap.getNamesOf(device).toArray()[0]);
        }
        telemetry.update();
        sleep(2000);

        // Try to initialize distance sensors
        telemetry.addLine("\n🔍 DISTANCE SENSORS:");

        try {
            frontDistanceSensor = hardwareMap.get(AnalogInput.class, "LaserSensorFront");
            telemetry.addLine("  ✅ Front Distance (LaserSensorFront)");
        } catch (Exception e) {
            telemetry.addLine("  ❌ Front Distance: " + e.getMessage());
        }

        try {
            backDistanceSensor = hardwareMap.get(AnalogInput.class, "LaserSensorBack");
            telemetry.addLine("  ✅ Back Distance (LaserSensorBack)");
        } catch (Exception e) {
            telemetry.addLine("  ❌ Back Distance: " + e.getMessage());
        }

        // Try to initialize color sensors
        telemetry.addLine("\n🎨 COLOR SENSORS (Gain=50):");
        telemetry.addLine("  NOTE: Front Left & Back Right are");
        telemetry.addLine("  temporarily REV 2m distance sensors");
        telemetry.addLine("");

        try {
            frontLeftColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorLeftFront");
            if (frontLeftColorSensor != null) {
                frontLeftColorSensor.setGain(50);  // Increased gain for better detection range
            }
            telemetry.addLine("  ✅ Front Left (ColorSensorLeftFront) - Gain=50");
        } catch (Exception e) {
            telemetry.addLine("  ❌ Front Left: " + e.getMessage());
            telemetry.addLine("     (Expected - temp REV 2m distance sensor)");
        }

        try {
            frontRightColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorRightFront");
            if (frontRightColorSensor != null) {
                frontRightColorSensor.setGain(50);  // Increased gain for better detection range
            }
            telemetry.addLine("  ✅ Front Right (ColorSensorRightFront) - Gain=50");
        } catch (Exception e) {
            telemetry.addLine("  ❌ Front Right: " + e.getMessage());
        }

        try {
            backRightColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorRightBack");
            if (backRightColorSensor != null) {
                backRightColorSensor.setGain(50);  // Increased gain for better detection range
            }
            telemetry.addLine("  ✅ Back Right (ColorSensorRightBack) - Gain=50");
        } catch (Exception e) {
            telemetry.addLine("  ❌ Back Right: " + e.getMessage());
            telemetry.addLine("     (Expected - temp REV 2m distance sensor)");
        }

        try {
            leftRightColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorLeftBack");
            if (leftRightColorSensor != null) {
                leftRightColorSensor.setGain(50);  // Increased gain for better detection range
            }
            telemetry.addLine("  ✅ Left Back (ColorSensorLeftBack) - Gain=50");
        } catch (Exception e) {
            telemetry.addLine("  ❌ Left Back: " + e.getMessage());
        }

        try {
            frontCenterColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorFront");
            if (frontCenterColorSensor != null) {
                frontCenterColorSensor.setGain(50);  // Increased gain for better detection range
            }
            telemetry.addLine("  ✅ Front Center (ColorSensorFront) - Gain=50");
        } catch (Exception e) {
            telemetry.addLine("  ❌ Front Center: " + e.getMessage());
        }

        try {
            backCenterColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorBack");
            if (backCenterColorSensor != null) {
                backCenterColorSensor.setGain(50);  // Increased gain for better detection range
            }
            telemetry.addLine("  ✅ Back Center (ColorSensorBack) - Gain=50");
        } catch (Exception e) {
            telemetry.addLine("  ❌ Back Center: " + e.getMessage());
        }

        telemetry.addLine("\n✅ Initialization Complete");
        telemetry.addLine("Press PLAY to see live sensor readings");
        telemetry.update();

        waitForStart();

        // Main loop - show live sensor readings
        while (opModeIsActive()) {
            telemetry.clear();
            telemetry.addLine("🔍 LIVE SENSOR READINGS");
            telemetry.addLine("======================");

            // Distance sensor readings
            telemetry.addLine("\n📏 DISTANCE SENSORS:");
            if (frontDistanceSensor != null) {
                double volts = frontDistanceSensor.getVoltage();
                double mm = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
                telemetry.addLine(String.format("  Front: %.2fV (%.1fmm)", volts, mm));
            } else {
                telemetry.addLine("  Front: NOT FOUND");
            }

            if (backDistanceSensor != null) {
                double volts = backDistanceSensor.getVoltage();
                double mm = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
                telemetry.addLine(String.format("  Back: %.2fV (%.1fmm)", volts, mm));
            } else {
                telemetry.addLine("  Back: NOT FOUND");
            }

            // Color sensor readings
            telemetry.addLine("\n🎨 COLOR SENSORS (RGB):");

            if (frontLeftColorSensor != null) {
                NormalizedRGBA colors = frontLeftColorSensor.getNormalizedColors();
                telemetry.addLine(String.format("  Front Left: R=%.2f G=%.2f B=%.2f",
                    colors.red, colors.green, colors.blue));
            } else {
                telemetry.addLine("  Front Left: NOT FOUND");
            }

            if (frontRightColorSensor != null) {
                NormalizedRGBA colors = frontRightColorSensor.getNormalizedColors();
                telemetry.addLine(String.format("  Front Right: R=%.2f G=%.2f B=%.2f",
                    colors.red, colors.green, colors.blue));
            } else {
                telemetry.addLine("  Front Right: NOT FOUND");
            }

            if (backRightColorSensor != null) {
                NormalizedRGBA colors = backRightColorSensor.getNormalizedColors();
                telemetry.addLine(String.format("  Back Right: R=%.2f G=%.2f B=%.2f",
                    colors.red, colors.green, colors.blue));
            } else {
                telemetry.addLine("  Back Right: NOT FOUND");
            }

            if (leftRightColorSensor != null) {
                NormalizedRGBA colors = leftRightColorSensor.getNormalizedColors();
                telemetry.addLine(String.format("  Left Back: R=%.2f G=%.2f B=%.2f",
                    colors.red, colors.green, colors.blue));
            } else {
                telemetry.addLine("  Left Back: NOT FOUND");
            }

            if (frontCenterColorSensor != null) {
                NormalizedRGBA colors = frontCenterColorSensor.getNormalizedColors();
                telemetry.addLine(String.format("  Front Center: R=%.2f G=%.2f B=%.2f",
                    colors.red, colors.green, colors.blue));
            } else {
                telemetry.addLine("  Front Center: NOT FOUND");
            }

            if (backCenterColorSensor != null) {
                NormalizedRGBA colors = backCenterColorSensor.getNormalizedColors();
                telemetry.addLine(String.format("  Back Center: R=%.2f G=%.2f B=%.2f",
                    colors.red, colors.green, colors.blue));
            } else {
                telemetry.addLine("  Back Center: NOT FOUND");
            }

            telemetry.addLine("\n⏹️ Press STOP to end test");
            telemetry.update();

            sleep(100);  // Update at 10Hz
        }
    }
}

