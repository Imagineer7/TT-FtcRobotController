package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Artifact;

/**
 * SensorTest - Independent testing for color and distance sensors
 *
 * Tests the artifact detection logic used by the indexing system
 * without running the full indexing operations.
 *
 * SENSOR DETECTION LOGIC:
 * ────────────────────────────────────────────────────────────────────────────
 * 1. Distance Sensor: Detects when artifact is close (< 10cm)
 * 2. Color Sensors: Determine if artifact is Purple or Green
 * 3. Confidence Check: Color reading must be strong enough (> 60%)
 *
 * CONTROLS:
 * ────────────────────────────────────────────────────────────────────────────
 * Gamepad 1:
 *   [A] - Toggle front intake sensor monitoring
 *   [B] - Toggle back intake sensor monitoring
 *   [X] - Toggle center sensors monitoring
 *   [Y] - Reset all detection flags
 *
 *   [DPAD_UP]    - Increase detection distance threshold
 *   [DPAD_DOWN]  - Decrease detection distance threshold
 *   [DPAD_LEFT]  - Decrease color confidence threshold
 *   [DPAD_RIGHT] - Increase color confidence threshold
 *
 *   [START] - Toggle continuous monitoring mode
 *   [BACK]  - Toggle raw sensor data display
 */
@TeleOp(name = "🔍 Sensor Test", group = "Testing")
@Disabled
public class SensorTest extends LinearOpMode {

    // Hardware
    private AuroraHardwareConfig hardware;

    // Distance sensor calibration (for goBILDA laser sensors in analog mode)
    // Official spec: 0.0V = 0mm, 3.3V = 1000mm (linear mapping)
    // These values match the official goBILDA example code
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    // Detection thresholds (adjustable)
    private double distanceThresholdCm = 10.0;  // Artifact detected when < 10cm
    private double colorConfidenceThreshold = 0.6;  // 60% confidence needed

    // Monitoring flags
    private boolean monitorFront = true;
    private boolean monitorBack = true;
    private boolean monitorCenter = true;
    private boolean continuousMode = true;
    private boolean showRawData = false;

    // Detection state
    private boolean frontArtifactDetected = false;
    private boolean backArtifactDetected = false;
    private Artifact.Color frontDetectedColor = Artifact.Color.UNKNOWN;
    private Artifact.Color backDetectedColor = Artifact.Color.UNKNOWN;

    // Button state tracking
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastStart = false;
    private boolean lastBack = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() {
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🔍 SENSOR TEST");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("Initializing sensors...");
        telemetry.update();

        // Initialize hardware
        try {
            hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
            hardware.initialize();

            // Give time to read initialization messages
            sleep(1000);

            if (!hardware.isIndexingSystemInitialized()) {
                telemetry.addLine("⚠️ WARNING: Indexing system failed to initialize");
            }
        } catch (Exception e) {
            telemetry.addLine("❌ Hardware initialization failed:");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            throw e;
        }

        // DON'T clear telemetry - keep the initialization messages visible
        telemetry.addLine("");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("📊 DETAILED SENSOR STATUS:");
        telemetry.addLine("════════════════════════════════════");

        // Distance sensors
        telemetry.addLine("\n📏 Distance Sensors:");
        if (hardware.getFrontDistanceSensor() != null) {
            telemetry.addLine("  ✅ Front Distance - OK");
        } else {
            telemetry.addLine("  ❌ Front Distance - MISSING");
        }

        if (hardware.getBackDistanceSensor() != null) {
            telemetry.addLine("  ✅ Back Distance - OK");
        } else {
            telemetry.addLine("  ❌ Back Distance - MISSING");
        }

        // Color sensors
        telemetry.addLine("\n🎨 Color Sensors (Gain=50):");
        telemetry.addLine("  ⚠️ NOTE: Front Left & Back Right temporarily");
        telemetry.addLine("     replaced with REV 2m distance sensors");
        telemetry.addLine("");
        if (hardware.getFrontLeftColorSensor() != null) {
            telemetry.addLine("  ✅ Front Left - OK");
        } else {
            telemetry.addLine("  ⚠️ Front Left - TEMP: REV 2m Distance Sensor");
        }

        if (hardware.getFrontRightColorSensor() != null) {
            telemetry.addLine("  ✅ Front Right - OK");
        } else {
            telemetry.addLine("  ❌ Front Right - MISSING");
        }

        if (hardware.getBackRightColorSensor() != null) {
            telemetry.addLine("  ✅ Back Right - OK");
        } else {
            telemetry.addLine("  ⚠️ Back Right - TEMP: REV 2m Distance Sensor");
        }

        if (hardware.getLeftRightColorSensor() != null) {
            telemetry.addLine("  ✅ Left Back - OK");
        } else {
            telemetry.addLine("  ❌ Left Back - MISSING");
        }

        if (hardware.getFrontCenterColorSensor() != null) {
            telemetry.addLine("  ✅ Front Center - OK");
        } else {
            telemetry.addLine("  ❌ Front Center - MISSING");
        }

        if (hardware.getBackCenterColorSensor() != null) {
            telemetry.addLine("  ✅ Back Center - OK");
        } else {
            telemetry.addLine("  ❌ Back Center - MISSING");
        }

        telemetry.addLine("");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("Press [START] to begin sensor testing");
        telemetry.addLine("════════════════════════════════════");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Main loop
        while (opModeIsActive()) {
            // Handle button inputs
            handleGamepadInputs();

            // Check sensors if monitoring enabled
            if (continuousMode) {
                if (monitorFront) {
                    checkFrontIntakeSensors();
                }
                if (monitorBack) {
                    checkBackIntakeSensors();
                }
            }

            // Update telemetry
            updateTelemetry();

            sleep(50);  // Update at 20Hz
        }
    }

    /**
     * Handle gamepad inputs
     */
    private void handleGamepadInputs() {
        // [A] - Toggle front monitoring
        if (gamepad1.a && !lastA) {
            monitorFront = !monitorFront;
            if (!monitorFront) {
                frontArtifactDetected = false;
                frontDetectedColor = Artifact.Color.UNKNOWN;
            }
        }
        lastA = gamepad1.a;

        // [B] - Toggle back monitoring
        if (gamepad1.b && !lastB) {
            monitorBack = !monitorBack;
            if (!monitorBack) {
                backArtifactDetected = false;
                backDetectedColor = Artifact.Color.UNKNOWN;
            }
        }
        lastB = gamepad1.b;

        // [X] - Toggle center monitoring
        if (gamepad1.x && !lastX) {
            monitorCenter = !monitorCenter;
        }
        lastX = gamepad1.x;

        // [Y] - Reset detections
        if (gamepad1.y && !lastY) {
            frontArtifactDetected = false;
            backArtifactDetected = false;
            frontDetectedColor = Artifact.Color.UNKNOWN;
            backDetectedColor = Artifact.Color.UNKNOWN;
        }
        lastY = gamepad1.y;

        // [START] - Toggle continuous mode
        if (gamepad1.start && !lastStart) {
            continuousMode = !continuousMode;
        }
        lastStart = gamepad1.start;

        // [BACK] - Toggle raw data display
        if (gamepad1.back && !lastBack) {
            showRawData = !showRawData;
        }
        lastBack = gamepad1.back;

        // [DPAD_UP] - Increase distance threshold
        if (gamepad1.dpad_up && !lastDpadUp) {
            distanceThresholdCm += 1.0;
            if (distanceThresholdCm > 50.0) distanceThresholdCm = 50.0;
        }
        lastDpadUp = gamepad1.dpad_up;

        // [DPAD_DOWN] - Decrease distance threshold
        if (gamepad1.dpad_down && !lastDpadDown) {
            distanceThresholdCm -= 1.0;
            if (distanceThresholdCm < 1.0) distanceThresholdCm = 1.0;
        }
        lastDpadDown = gamepad1.dpad_down;

        // [DPAD_LEFT] - Decrease color confidence
        if (gamepad1.dpad_left && !lastDpadLeft) {
            colorConfidenceThreshold -= 0.05;
            if (colorConfidenceThreshold < 0.1) colorConfidenceThreshold = 0.1;
        }
        lastDpadLeft = gamepad1.dpad_left;

        // [DPAD_RIGHT] - Increase color confidence
        if (gamepad1.dpad_right && !lastDpadRight) {
            colorConfidenceThreshold += 0.05;
            if (colorConfidenceThreshold > 1.0) colorConfidenceThreshold = 1.0;
        }
        lastDpadRight = gamepad1.dpad_right;
    }

    /**
     * Check front intake sensors (same logic as IndexingSystem)
     */
    private void checkFrontIntakeSensors() {
        // Get distance sensor
        AnalogInput distanceSensor = hardware.getFrontDistanceSensor();
        if (distanceSensor == null) return;

        // Check distance
        double distanceCm = getDistanceFromAnalogSensor(distanceSensor);

        if (distanceCm < distanceThresholdCm) {
            // Artifact is close enough - check color
            Artifact.Color detectedColor = detectColorFront();

            if (detectedColor != Artifact.Color.UNKNOWN) {
                frontArtifactDetected = true;
                frontDetectedColor = detectedColor;
            }
        } else {
            // No artifact close
            frontArtifactDetected = false;
            frontDetectedColor = Artifact.Color.UNKNOWN;
        }
    }

    /**
     * Check back intake sensors (same logic as IndexingSystem)
     */
    private void checkBackIntakeSensors() {
        // Get distance sensor
        AnalogInput distanceSensor = hardware.getBackDistanceSensor();
        if (distanceSensor == null) return;

        // Check distance
        double distanceCm = getDistanceFromAnalogSensor(distanceSensor);

        if (distanceCm < distanceThresholdCm) {
            // Artifact is close enough - check color
            Artifact.Color detectedColor = detectColorBack();

            if (detectedColor != Artifact.Color.UNKNOWN) {
                backArtifactDetected = true;
                backDetectedColor = detectedColor;
            }
        } else {
            // No artifact close
            backArtifactDetected = false;
            backDetectedColor = Artifact.Color.UNKNOWN;
        }
    }

    /**
     * Convert analog sensor voltage to distance in cm
     *
     * For goBILDA Dual-Mode Laser Distance Sensor (Analog Mode):
     * - Linear mapping: 0.0V → 0mm, 3.3V → 1000mm
     * - This matches the official goBILDA example code
     * - Sensor must be configured in analog mode (not I2C)
     *
     * Hardware Requirements:
     * - Sensor connected to Hub Analog port (0-1 or 2-3)
     * - Configured as AnalogInput in Robot Configuration
     * - Device names: "Laser Sensor Front" and "Laser Sensor Back"
     *
     * @param sensor The AnalogInput sensor to read
     * @return Distance in centimeters (0-100cm), or 999.0 if sensor unavailable
     */
    private double getDistanceFromAnalogSensor(AnalogInput sensor) {
        if (sensor == null) return 999.0;

        // Read sensor voltage (0.0–3.3V) - direct from official example
        double voltage = sensor.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMm = (voltage / MAX_VOLTS) * MAX_DISTANCE_MM;

        // Convert mm to cm for easier reading
        return distanceMm / 10.0;
    }

    /**
     * Detect color from front intake color sensors
     * Uses left and right sensors to determine color
     */
    private Artifact.Color detectColorFront() {
        NormalizedColorSensor leftSensor = hardware.getFrontLeftColorSensor();
        NormalizedColorSensor rightSensor = hardware.getFrontRightColorSensor();

        if (leftSensor == null || rightSensor == null) {
            return Artifact.Color.UNKNOWN;
        }

        // Get normalized color readings
        NormalizedRGBA leftColor = leftSensor.getNormalizedColors();
        NormalizedRGBA rightColor = rightSensor.getNormalizedColors();

        // Average the readings
        float avgRed = (leftColor.red + rightColor.red) / 2.0f;
        float avgGreen = (leftColor.green + rightColor.green) / 2.0f;
        float avgBlue = (leftColor.blue + rightColor.blue) / 2.0f;

        // Determine color (same logic as IndexingSystem)
        return determineArtifactColor(avgRed, avgGreen, avgBlue);
    }

    /**
     * Detect color from back intake color sensors
     */
    private Artifact.Color detectColorBack() {
        NormalizedColorSensor leftSensor = hardware.getLeftRightColorSensor();
        NormalizedColorSensor rightSensor = hardware.getBackRightColorSensor();

        if (leftSensor == null || rightSensor == null) {
            return Artifact.Color.UNKNOWN;
        }

        // Get normalized color readings
        NormalizedRGBA leftColor = leftSensor.getNormalizedColors();
        NormalizedRGBA rightColor = rightSensor.getNormalizedColors();

        // Average the readings
        float avgRed = (leftColor.red + rightColor.red) / 2.0f;
        float avgGreen = (leftColor.green + rightColor.green) / 2.0f;
        float avgBlue = (leftColor.blue + rightColor.blue) / 2.0f;

        // Determine color
        return determineArtifactColor(avgRed, avgGreen, avgBlue);
    }

    /**
     * Determine artifact color from RGB values
     * Uses IndexingConfig RGB threshold method with measured values
     */
    private Artifact.Color determineArtifactColor(float red, float green, float blue) {
        // Use IndexingConfig for color detection with measured RGB thresholds
        IndexingConfig config = new IndexingConfig();
        String detectedColor = config.detectArtifactColor(red, green, blue);

        if ("PURPLE".equals(detectedColor)) {
            // Verify confidence meets minimum threshold
            double confidence = config.calculateColorConfidence(red, green, blue, "PURPLE");
            if (confidence >= config.getColorDetectionMinScore()) {
                return Artifact.Color.PURPLE;
            }
        } else if ("GREEN".equals(detectedColor)) {
            // Verify confidence meets minimum threshold
            double confidence = config.calculateColorConfidence(red, green, blue, "GREEN");
            if (confidence >= config.getColorDetectionMinScore()) {
                return Artifact.Color.GREEN;
            }
        }


        return Artifact.Color.UNKNOWN;
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.clear();

        // Header
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🔍 SENSOR TEST");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("");

        // Configuration
        telemetry.addLine("⚙️ DETECTION SETTINGS:");
        telemetry.addData("  Distance Threshold", String.format("%.1f cm", distanceThresholdCm));
        telemetry.addData("  Color Confidence", String.format("%.0f%%", colorConfidenceThreshold * 100));
        telemetry.addData("  Continuous Mode", continuousMode ? "ON" : "OFF");
        telemetry.addLine("");

        // Front Intake Status
        telemetry.addLine("🟦 FRONT INTAKE:");
        telemetry.addData("  Monitoring", monitorFront ? "ON" : "OFF");
        if (monitorFront) {
            AnalogInput frontDist = hardware.getFrontDistanceSensor();
            if (frontDist != null) {
                double voltage = frontDist.getVoltage();
                double dist = getDistanceFromAnalogSensor(frontDist);
                telemetry.addData("  Voltage", String.format("%.3f V", voltage));
                telemetry.addData("  Distance", String.format("%.1f cm (%.0f mm)", dist, dist * 10));
                telemetry.addData("  Artifact Detected", frontArtifactDetected ? "YES ✓" : "NO");
                if (frontArtifactDetected) {
                    telemetry.addData("  Color", frontDetectedColor);
                }
            } else {
                telemetry.addLine("  ⚠️ Distance sensor not available");
                telemetry.addLine("     Check Robot Config for:");
                telemetry.addLine("     'Laser Sensor Front' (AnalogInput)");
            }

            if (showRawData) {
                if (hardware.getFrontLeftColorSensor() != null) {
                    displayRawColorData("Front Left", hardware.getFrontLeftColorSensor());
                } else {
                    telemetry.addLine("  Front Left: ✗ Check 'Color Sensor Left Front'");
                }
                if (hardware.getFrontRightColorSensor() != null) {
                    displayRawColorData("Front Right", hardware.getFrontRightColorSensor());
                } else {
                    telemetry.addLine("  Front Right: ✗ Check 'Color Sensor Right Front'");
                }
            }
        }
        telemetry.addLine("");

        // Back Intake Status
        telemetry.addLine("🟨 BACK INTAKE:");
        telemetry.addData("  Monitoring", monitorBack ? "ON" : "OFF");
        if (monitorBack) {
            AnalogInput backDist = hardware.getBackDistanceSensor();
            if (backDist != null) {
                double voltage = backDist.getVoltage();
                double dist = getDistanceFromAnalogSensor(backDist);
                telemetry.addData("  Voltage", String.format("%.3f V", voltage));
                telemetry.addData("  Distance", String.format("%.1f cm (%.0f mm)", dist, dist * 10));
                telemetry.addData("  Artifact Detected", backArtifactDetected ? "YES ✓" : "NO");
                if (backArtifactDetected) {
                    telemetry.addData("  Color", backDetectedColor);
                }
            } else {
                telemetry.addLine("  ⚠️ Distance sensor not available");
                telemetry.addLine("     Check Robot Config for:");
                telemetry.addLine("     'Laser Sensor Back' (AnalogInput)");
            }

            if (showRawData) {
                if (hardware.getLeftRightColorSensor() != null) {
                    displayRawColorData("Back Left", hardware.getLeftRightColorSensor());
                } else {
                    telemetry.addLine("  Back Left: ✗ Check 'Color Sensor Left Back'");
                }
                if (hardware.getBackRightColorSensor() != null) {
                    displayRawColorData("Back Right", hardware.getBackRightColorSensor());
                } else {
                    telemetry.addLine("  Back Right: ✗ Check 'Color Sensor Right Back'");
                }
            }
        }
        telemetry.addLine("");

        // Center Sensors (if monitoring)
        if (monitorCenter && showRawData) {
            telemetry.addLine("🟩 CENTER SENSORS:");
            displayRawColorData("Center Front", hardware.getFrontCenterColorSensor());
            displayRawColorData("Center Back", hardware.getBackCenterColorSensor());
            telemetry.addLine("");
        }

        // Controls
        telemetry.addLine("🎮 CONTROLS:");
        telemetry.addLine("  [A] Front On/Off  [B] Back On/Off  [Y] Reset");
        telemetry.addLine("  [DPAD ↑↓] Distance  [DPAD ←→] Confidence");
        telemetry.addLine("  [START] Continuous  [BACK] Raw Data");

        // Detection summary
        if (frontArtifactDetected || backArtifactDetected) {
            telemetry.addLine("");
            telemetry.addLine("✨ DETECTIONS:");
            if (frontArtifactDetected) {
                telemetry.addLine("  🔵 FRONT: " + frontDetectedColor + " artifact!");
            }
            if (backArtifactDetected) {
                telemetry.addLine("  🟡 BACK: " + backDetectedColor + " artifact!");
            }
        }

        telemetry.update();
    }

    /**
     * Display raw color sensor data
     */
    private void displayRawColorData(String name, NormalizedColorSensor sensor) {
        if (sensor == null) {
            telemetry.addLine("  " + name + ": Not available");
            return;
        }

        NormalizedRGBA colors = sensor.getNormalizedColors();
        telemetry.addLine(String.format("  %s: R:%.2f G:%.2f B:%.2f",
            name, colors.red, colors.green, colors.blue));
    }
}

