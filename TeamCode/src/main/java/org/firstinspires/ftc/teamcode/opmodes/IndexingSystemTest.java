package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingSystemOld;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Artifact;

/**
 * IndexingSystemTest - Testing OpMode for Indexing and Intake Systems
 *
 * This OpMode provides comprehensive testing capabilities for the artifact indexing
 * and intake systems. It includes manual control, automated testing sequences, and
 * real-time telemetry for debugging.
 *
 * INTAKE SYSTEM BEHAVIOR:
 * ────────────────────────────────────────────────────────────────────────────
 * - Both intake rollers run CONTINUOUSLY at full power (from config)
 * - When artifact detected, collection starts automatically
 * - During collection, intake continues at full power
 * - When intake stores an artifact, power reduces to 50% to retain it
 * - System automatically manages intake speeds - no manual control needed
 *
 * CONTROLS:
 * ────────────────────────────────────────────────────────────────────────────
 * Gamepad 1:
 *   [A] - Simulate artifact detection at FRONT intake
 *   [B] - Simulate artifact detection at BACK intake
 *   [X] - Fire artifact (if ready)
 *   [Y] - Emergency stop all motors/servos
 *
 *   [LEFT_BUMPER]  - Toggle manual servo mode (for troubleshooting)
 *   [RIGHT_BUMPER] - Run injector servos (manual mode only)
 *   [LEFT_TRIGGER] - Run uptake servos (manual mode only)
 *   [RIGHT_TRIGGER] - Run transfer servos (manual mode only)
 *
 *   [START] - Reset indexing system
 *   [BACK]  - Toggle debug telemetry
 *
 * Gamepad 2:
 *   [A] - Spin up shooter
 *   [B] - Stop shooter
 *
 * TESTING MODES:
 * ────────────────────────────────────────────────────────────────────────────
 * 1. Manual Mode (Default)
 *    - Full manual control of all systems
 *    - Real-time telemetry display
 *    - Intakes run automatically based on system state
 *
 * 2. Manual Servo Mode (LEFT_BUMPER)
 *    - For troubleshooting individual servos only
 *    - Does NOT affect intake roller operation
 */
@TeleOp(name = "🔧 Indexing System Test", group = "Testing")
@Disabled
public class IndexingSystemTest extends LinearOpMode {

    // Hardware and Systems
    private AuroraHardwareConfig hardware;
    private IndexingSystemOld indexingSystem;
    private IndexingConfig indexingConfig;
    private Shooter shooter;
    private ShooterConfig shooterConfig;

    // Test Mode State
    private enum TestMode {
        AUTO_COLLECTION,     // Default: Auto-detect and collect artifacts
        MANUAL_CONTROL,      // Manual button control
        SERVO_DEBUG,         // Individual servo testing
        SENSOR_DEBUG,        // Raw sensor data
        RUNNING_TEST
    }
    private TestMode currentMode = TestMode.AUTO_COLLECTION;
    private long testStartTime = 0;

    // Telemetry Page System
    private enum TelemetryPage {
        OVERVIEW,           // Main system status
        DEBUG,              // IndexingSystem debug output
        SUBSYSTEMS,         // Detailed subsystem status
        RAW_DATA,           // Raw sensor and motor data
        CONFIGURATION,      // Config parameters and timings
        DIAGNOSTICS,        // Error logs and performance
        CONTROLS            // Control help and status
    }
    private TelemetryPage currentPage = TelemetryPage.OVERVIEW;

    // Manual Control State
    private boolean manualMode = false;
    private boolean debugTelemetry = true;

    // Button state tracking (for edge detection)
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastStart = false;
    private boolean lastBack = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftStick = false;
    private boolean lastRightStick = false;

    // Gamepad 2 state
    private boolean lastG2A = false;
    private boolean lastG2B = false;
    private boolean lastG2X = false;
    private boolean lastG2Y = false;

    // Performance tracking
    private long loopStartTime = 0;
    private long maxLoopTime = 0;
    private long totalLoops = 0;
    private long avgLoopTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🔧 INDEXING SYSTEM TEST");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        // Initialize hardware
        try {
            hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
            hardware.initialize();

            if (!hardware.isIndexingSystemInitialized()) {
                telemetry.addLine("⚠️ WARNING: Indexing system not fully initialized");
                telemetry.addLine(hardware.getIndexingInitError());
            }

            if (!hardware.isShooterSystemInitialized()) {
                telemetry.addLine("⚠️ WARNING: Shooter system not fully initialized");
                telemetry.addLine(hardware.getShooterInitError());
            }
        } catch (Exception e) {
            telemetry.addLine("❌ Hardware initialization failed:");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            throw e;
        }

        // Initialize configurations
        indexingConfig = new IndexingConfig();
        indexingConfig.setDebugTelemetry(true);

        shooterConfig = new ShooterConfig();

        // Initialize shooter first (needed by indexing system)
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        shooter.enable();

        // Initialize indexing system (requires shooter)
        indexingSystem = new IndexingSystemOld(hardware, indexingConfig, shooter, telemetry);

        telemetry.addLine("✅ Initialization complete!");
        telemetry.addLine("");
        telemetry.addLine("🤖 Default Mode: AUTO_COLLECTION");
        telemetry.addLine("📊 Default Page: OVERVIEW");
        telemetry.addLine("");
        telemetry.addLine("Press [START] to begin");
        telemetry.addLine("See CONTROLS page for all commands");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Initialize timing
        testStartTime = System.currentTimeMillis();

        // Enable systems after start
        indexingSystem.enable();

        telemetry.clear();
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🤖 INDEXING SYSTEM TEST - ACTIVE");
        telemetry.addLine("════════════════════════════════════");
        telemetry.update();

        // Main control loop - only runs after START is pressed
        while (opModeIsActive()) {
            loopStartTime = System.currentTimeMillis();

            // Handle button inputs first
            handleGamepadInputs();

            // For DEBUG page, we need special handling to show IndexingSystem debug output
            if (currentPage == TelemetryPage.DEBUG) {
                // Set up DEBUG page header
                telemetry.clear();
                telemetry.addLine("════════════════════════════════════");
                telemetry.addLine("🔧 INDEXING SYSTEM TEST");
                telemetry.addData("Page", currentPage.toString() + " (" + (currentPage.ordinal() + 1) + "/7)");
                telemetry.addData("Mode", currentMode.toString());
                telemetry.addLine("════════════════════════════════════");
                telemetry.addLine("");
                telemetry.addLine("🔍 INDEXING SYSTEM DEBUG OUTPUT:");
                telemetry.addLine("────────────────────────────────────");

                // Update systems - IndexingSystem will add debug info to telemetry
                indexingSystem.update();
                shooter.update();

                // Add navigation footer
                telemetry.addLine("────────────────────────────────────");
                telemetry.addLine("Navigation: DPAD ↑↓ | DPAD ← Overview | DPAD → Raw Data");
                telemetry.update();
            } else {
                // Normal processing for other pages
                // Update all systems - IndexingSystem handles auto-detection internally
                indexingSystem.update();
                shooter.update();

                // Update telemetry based on current page
                updateTelemetryPages();
            }

            // Performance tracking
            updatePerformanceMetrics();

            // Small delay to prevent loop overrun
            sleep(20);
        }

        // Cleanup
        indexingSystem.disable();
        hardware.stopAllMotors();
    }

    /**
     * Convert sensor voltage to distance in centimeters
     * Based on goBILDA laser sensor characteristics
     */
    private double convertVoltageToDistance(double voltage) {
        if (voltage <= 0.5) return 0.0; // Invalid reading

        // goBILDA laser sensor: ~0.5V at 5cm, ~3.3V at 100cm
        // Linear approximation: distance = (voltage - 0.5) * 34.3 + 5
        return (voltage - 0.5) * 34.3 + 5.0;
    }

    /**
     * Create artifact with color detection from sensors
     */
    private Artifact createDetectedArtifact(IndexingSystemOld.IntakeSource source) {
        Artifact.Color detectedColor = detectArtifactColor(source);

        return new Artifact(
            detectedColor,
            source == IndexingSystemOld.IntakeSource.FRONT ?
                Artifact.Location.FRONT_INTAKE : Artifact.Location.BACK_INTAKE,
            0  // Collection order will be set by indexing system
        );
    }

    /**
     * Detect artifact color using color sensors at specified intake
     */
    private Artifact.Color detectArtifactColor(IndexingSystemOld.IntakeSource source) {
        if (hardware == null) {
            telemetry.addLine("⚠️ Color detection: hardware is null");
            return Artifact.Color.UNKNOWN;
        }

        try {
            double totalRed = 0, totalGreen = 0, totalBlue = 0;
            int sensorCount = 0;

            // Get color readings from available sensors
            if (source == IndexingSystemOld.IntakeSource.FRONT) {
                if (hardware.getFrontLeftColorSensor() != null) {
                    com.qualcomm.robotcore.hardware.NormalizedRGBA colors = hardware.getFrontLeftColorSensor().getNormalizedColors();
                    totalRed += colors.red;
                    totalGreen += colors.green;
                    totalBlue += colors.blue;
                    sensorCount++;
                }
                if (hardware.getFrontRightColorSensor() != null) {
                    com.qualcomm.robotcore.hardware.NormalizedRGBA colors = hardware.getFrontRightColorSensor().getNormalizedColors();
                    totalRed += colors.red;
                    totalGreen += colors.green;
                    totalBlue += colors.blue;
                    sensorCount++;
                }
                if (hardware.getFrontCenterColorSensor() != null) {
                    com.qualcomm.robotcore.hardware.NormalizedRGBA colors = hardware.getFrontCenterColorSensor().getNormalizedColors();
                    totalRed += colors.red;
                    totalGreen += colors.green;
                    totalBlue += colors.blue;
                    sensorCount++;
                }
            } else if (source == IndexingSystemOld.IntakeSource.BACK) {
                if (hardware.getBackRightColorSensor() != null) {
                    com.qualcomm.robotcore.hardware.NormalizedRGBA colors = hardware.getBackRightColorSensor().getNormalizedColors();
                    totalRed += colors.red;
                    totalGreen += colors.green;
                    totalBlue += colors.blue;
                    sensorCount++;
                }
                if (hardware.getLeftRightColorSensor() != null) {
                    com.qualcomm.robotcore.hardware.NormalizedRGBA colors = hardware.getLeftRightColorSensor().getNormalizedColors();
                    totalRed += colors.red;
                    totalGreen += colors.green;
                    totalBlue += colors.blue;
                    sensorCount++;
                }
                if (hardware.getBackCenterColorSensor() != null) {
                    com.qualcomm.robotcore.hardware.NormalizedRGBA colors = hardware.getBackCenterColorSensor().getNormalizedColors();
                    totalRed += colors.red;
                    totalGreen += colors.green;
                    totalBlue += colors.blue;
                    sensorCount++;
                }
            }

            if (sensorCount == 0) {
                telemetry.addLine("⚠️ Color detection: No color sensors available for " + source);
                return Artifact.Color.UNKNOWN;
            }

            // Average the readings and detect color
            double avgRed = totalRed / sensorCount;
            double avgGreen = totalGreen / sensorCount;
            double avgBlue = totalBlue / sensorCount;

            // Debug output for color detection
            telemetry.addLine(String.format("🎨 %s Color: R:%.3f G:%.3f B:%.3f (%d sensors)",
                source, avgRed, avgGreen, avgBlue, sensorCount));

            // Calculate ratio scores for both colors
            double purpleScore = indexingConfig.calculateColorConfidence(avgRed, avgGreen, avgBlue, "PURPLE");
            double greenScore = indexingConfig.calculateColorConfidence(avgRed, avgGreen, avgBlue, "GREEN");

            telemetry.addLine(String.format("🟣 Purple Score: %.3f | 🟢 Green Score: %.3f",
                purpleScore, greenScore));

            // Use the main indexing config for detection
            String detectedColor = indexingConfig.detectArtifactColor(avgRed, avgGreen, avgBlue);

            if ("PURPLE".equals(detectedColor)) {
                telemetry.addLine(String.format("🟣 PURPLE detected! Score: %.3f (need %.2f)",
                    purpleScore, indexingConfig.getColorDetectionMinScore()));
                return Artifact.Color.PURPLE;
            } else if ("GREEN".equals(detectedColor)) {
                telemetry.addLine(String.format("🟢 GREEN detected! Score: %.3f (need %.2f)",
                    greenScore, indexingConfig.getColorDetectionMinScore()));
                return Artifact.Color.GREEN;
            } else {
                telemetry.addLine(String.format("❓ No detection - scores too low (need %.2f)",
                    indexingConfig.getColorDetectionMinScore()));
            }

        } catch (Exception e) {
            telemetry.addLine("❌ Color detection error: " + e.getMessage());
        }

        return Artifact.Color.UNKNOWN;
    }

    /**
     * Handle all gamepad inputs
     */
    private void handleGamepadInputs() {
        // ═══════════════════════════════════════════════════════════════
        // GAMEPAD 1 - Indexing System Controls
        // ═══════════════════════════════════════════════════════════════

        // [A] - Toggle auto-detection (when in auto mode) or manual collect from front
        if (gamepad1.a && !lastA) {
            if (currentMode == TestMode.AUTO_COLLECTION) {
                boolean currentlyEnabled = indexingSystem.isAutoDetectionEnabled();
                indexingSystem.setAutoDetectionEnabled(!currentlyEnabled);
                telemetry.addLine(currentlyEnabled ? "🔄 Auto-detection DISABLED" : "🔄 Auto-detection ENABLED");
            } else if (currentMode == TestMode.MANUAL_CONTROL) {
                Artifact artifact = new Artifact(
                    Artifact.Color.UNKNOWN,
                    Artifact.Location.UNKNOWN,
                    0  // Collection order will be set by system
                );
                indexingSystem.onArtifactDetected(artifact, IndexingSystemOld.IntakeSource.FRONT);
                telemetry.addLine("▶ Manual: Collecting from FRONT intake");
            }
        }
        lastA = gamepad1.a;

        // [B] - Collect from BACK intake (manual mode) or force manual collection
        if (gamepad1.b && !lastB) {
            if (currentMode == TestMode.MANUAL_CONTROL) {
                Artifact artifact = new Artifact(
                    Artifact.Color.UNKNOWN,
                    Artifact.Location.UNKNOWN,
                    0  // Collection order will be set by system
                );
                indexingSystem.onArtifactDetected(artifact, IndexingSystemOld.IntakeSource.BACK);
                telemetry.addLine("▶ Manual: Collecting from BACK intake");
            } else {
                // Force collect from back in any mode
                Artifact artifact = createDetectedArtifact(IndexingSystemOld.IntakeSource.BACK);
                indexingSystem.onArtifactDetected(artifact, IndexingSystemOld.IntakeSource.BACK);
                telemetry.addLine("▶ Forced: Collecting from BACK intake");
            }
        }
        lastB = gamepad1.b;

        // [X] - Fire artifact
        if (gamepad1.x && !lastX) {
            boolean fired = indexingSystem.onFireSignal();
            if (fired) {
                telemetry.addLine("🔥 Firing artifact!");
            } else {
                telemetry.addLine("⚠️ Cannot fire - no artifact ready");
            }
        }
        lastX = gamepad1.x;

        // [Y] - Emergency stop / Cycle test modes
        if (gamepad1.y && !lastY) {
            if (gamepad1.right_stick_button) {
                // Emergency stop when right stick pressed
                hardware.stopAllMotors();
                indexingSystem.reset();
                telemetry.addLine("🛑 EMERGENCY STOP");
            } else {
                // Cycle through test modes
                currentMode = getNextTestMode(currentMode);
                telemetry.addLine("🔄 Mode: " + currentMode.toString());

                // Enable/disable auto-detection based on mode
                if (currentMode == TestMode.AUTO_COLLECTION) {
                    indexingSystem.setAutoDetectionEnabled(true);
                } else {
                    indexingSystem.setAutoDetectionEnabled(false);
                }
            }
        }
        lastY = gamepad1.y;

        // [START] - Reset system
        if (gamepad1.start && !lastStart) {
            indexingSystem.reset();
            telemetry.addLine("🔄 System reset");
        }
        lastStart = gamepad1.start;

        // [BACK] - Toggle debug telemetry
        if (gamepad1.back && !lastBack) {
            debugTelemetry = !debugTelemetry;
            indexingConfig.setDebugTelemetry(debugTelemetry);
            telemetry.addLine(debugTelemetry ? "📊 Debug ON" : "📊 Debug OFF");
        }
        lastBack = gamepad1.back;

        // D-PAD - Telemetry page navigation
        if (gamepad1.dpad_up && !lastDpadUp) {
            currentPage = getPreviousPage(currentPage);
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown) {
            currentPage = getNextPage(currentPage);
        }
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_left && !lastDpadLeft) {
            currentPage = TelemetryPage.OVERVIEW; // Quick return to overview
        }
        lastDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRight) {
            currentPage = TelemetryPage.RAW_DATA; // Quick jump to raw data
        }
        lastDpadRight = gamepad1.dpad_right;

        // Bumpers - Manual servo control
        if (gamepad1.left_bumper && !lastLeftBumper) {
            manualMode = !manualMode;
            telemetry.addLine(manualMode ? "🔧 Manual Servo Mode ON" : "🔧 Manual Servo Mode OFF");
        }
        lastLeftBumper = gamepad1.left_bumper;

        // Manual servo control when in manual mode
        if (manualMode) {
            // Check if IndexingSystem is controlling servos
            boolean indexingSystemControlling = indexingSystem.isUptakeServoPrePositioned() ||
                                              indexingSystem.isOperationInProgress() ||
                                              indexingSystem.getCurrentState() == IndexingSystemOld.SystemState.FIRING ||
                                              indexingSystem.getCurrentState() == IndexingSystemOld.SystemState.PUSHING;

            // Right bumper - Injector servos
            if (gamepad1.right_bumper) {
                if (hardware.getInjectorServoLeft() != null) {
                    hardware.getInjectorServoLeft().setPower(1.0);
                }
                if (hardware.getInjectorServoRight() != null) {
                    hardware.getInjectorServoRight().setPower(1.0);
                }
            } else {
                if (hardware.getInjectorServoLeft() != null) {
                    hardware.getInjectorServoLeft().setPower(0.0);
                }
                if (hardware.getInjectorServoRight() != null) {
                    hardware.getInjectorServoRight().setPower(0.0);
                }
            }

            // Left trigger - Uptake servos (only if IndexingSystem is not controlling them)
            if (!indexingSystemControlling) {
                if (gamepad1.left_trigger > 0.1) {
                    if (hardware.getUptakeServoL() != null) {
                        hardware.getUptakeServoL().setPower(gamepad1.left_trigger);
                    }
                    if (hardware.getUptakeServoR() != null) {
                        hardware.getUptakeServoR().setPower(gamepad1.left_trigger);
                    }
                } else {
                    if (hardware.getUptakeServoL() != null) {
                        hardware.getUptakeServoL().setPower(0.0);
                    }
                    if (hardware.getUptakeServoR() != null) {
                        hardware.getUptakeServoR().setPower(0.0);
                    }
                }
            }

            // Right trigger - Transfer servos
            if (gamepad1.right_trigger > 0.1) {
                if (hardware.getFrontTransferServo() != null) {
                    hardware.getFrontTransferServo().setPower(gamepad1.right_trigger);
                }
                if (hardware.getBackTransferServo() != null) {
                    hardware.getBackTransferServo().setPower(gamepad1.right_trigger);
                }
            } else {
                if (hardware.getFrontTransferServo() != null) {
                    hardware.getFrontTransferServo().setPower(0.0);
                }
                if (hardware.getBackTransferServo() != null) {
                    hardware.getBackTransferServo().setPower(0.0);
                }
            }
        }

        // MANUAL PUSH MODE CONTROLS (available in all modes)
        // Left stick button - Toggle manual push mode
        if (gamepad1.left_stick_button && !lastLeftStick) {
            boolean currentMode = indexingSystem.isManualPushMode();
            indexingSystem.setManualPushMode(!currentMode);
            telemetry.addLine(currentMode ? "🔄 Manual Push Mode DISABLED" : "🔄 Manual Push Mode ENABLED");
        }
        lastLeftStick = gamepad1.left_stick_button;

        // Right stick button - Execute manual push (only if 2 artifacts)
        if (gamepad1.right_stick_button && !lastRightStick) {
            boolean pushed = indexingSystem.onManualPush();
            if (pushed) {
                telemetry.addLine("🔄 Manual push executed!");
            } else {
                telemetry.addLine("⚠️ Cannot manual push - check conditions");
            }
        }
        lastRightStick = gamepad1.right_stick_button;

        // ═══════════════════════════════════════════════════════════════
        // GAMEPAD 2 - Shooter Controls & Advanced Functions
        // ═══════════════════════════════════════════════════════════════

        // [A] - Spin up shooter
        if (gamepad2.a && !lastG2A) {
            shooter.spinUp();
            telemetry.addLine("🔄 Shooter spinning up");
        }
        lastG2A = gamepad2.a;

        // [B] - Stop shooter
        if (gamepad2.b && !lastG2B) {
            shooter.stop();
            telemetry.addLine("⏹️ Shooter stopped");
        }
        lastG2B = gamepad2.b;

        // [X] - Toggle shooter power setting / Manual color test
        if (gamepad2.x && !lastG2X) {
            // Manual color detection test
            telemetry.addLine("🎨 MANUAL COLOR TEST:");
            Artifact.Color frontColor = detectArtifactColor(IndexingSystemOld.IntakeSource.FRONT);
            Artifact.Color backColor = detectArtifactColor(IndexingSystemOld.IntakeSource.BACK);
            telemetry.addLine("Front: " + frontColor + ", Back: " + backColor);
        }
        lastG2X = gamepad2.x;

        // [Y] - Advanced diagnostics
        if (gamepad2.y && !lastG2Y) {
            currentPage = TelemetryPage.DIAGNOSTICS;
            telemetry.addLine("🔍 Diagnostics mode");
        }
        lastG2Y = gamepad2.y;
    }

    /**
     * Get next test mode in sequence
     */
    private TestMode getNextTestMode(TestMode current) {
        switch (current) {
            case AUTO_COLLECTION: return TestMode.MANUAL_CONTROL;
            case MANUAL_CONTROL: return TestMode.SERVO_DEBUG;
            case SERVO_DEBUG: return TestMode.SENSOR_DEBUG;
            case SENSOR_DEBUG: return TestMode.AUTO_COLLECTION;
            default: return TestMode.AUTO_COLLECTION;
        }
    }

    /**
     * Get next telemetry page
     */
    private TelemetryPage getNextPage(TelemetryPage current) {
        switch (current) {
            case OVERVIEW: return TelemetryPage.SUBSYSTEMS;
            case SUBSYSTEMS: return TelemetryPage.RAW_DATA;
            case RAW_DATA: return TelemetryPage.CONFIGURATION;
            case CONFIGURATION: return TelemetryPage.DIAGNOSTICS;
            case DIAGNOSTICS: return TelemetryPage.CONTROLS;
            case CONTROLS: return TelemetryPage.OVERVIEW;
            default: return TelemetryPage.OVERVIEW;
        }
    }

    /**
     * Get previous telemetry page
     */
    private TelemetryPage getPreviousPage(TelemetryPage current) {
        switch (current) {
            case OVERVIEW: return TelemetryPage.CONTROLS;
            case SUBSYSTEMS: return TelemetryPage.OVERVIEW;
            case RAW_DATA: return TelemetryPage.SUBSYSTEMS;
            case CONFIGURATION: return TelemetryPage.RAW_DATA;
            case DIAGNOSTICS: return TelemetryPage.CONFIGURATION;
            case CONTROLS: return TelemetryPage.DIAGNOSTICS;
            default: return TelemetryPage.OVERVIEW;
        }
    }

    /**
     * Update telemetry based on current page
     */
    private void updateTelemetryPages() {
        telemetry.clear();

        // Common header for all pages
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🔧 INDEXING SYSTEM TEST");
        telemetry.addData("Page", currentPage.toString() + " (" + (currentPage.ordinal() + 1) + "/7)");
        telemetry.addData("Mode", currentMode.toString());
        telemetry.addLine("════════════════════════════════════");

        switch (currentPage) {
            case OVERVIEW:
                displayOverviewPage();
                break;
            case DEBUG:
                // DEBUG page is handled in main loop to preserve IndexingSystem debug output
                telemetry.addLine("DEBUG page handled in main loop");
                break;
            case SUBSYSTEMS:
                displaySubsystemsPage();
                break;
            case RAW_DATA:
                displayRawDataPage();
                break;
            case CONFIGURATION:
                displayConfigurationPage();
                break;
            case DIAGNOSTICS:
                displayDiagnosticsPage();
                break;
            case CONTROLS:
                displayControlsPage();
                break;
        }

        // Common navigation footer
        telemetry.addLine("────────────────────────────────────");
        telemetry.addLine("Navigation: DPAD ↑↓ | DPAD ← Overview | DPAD → Raw Data");

        telemetry.update();
    }

    /**
     * Display overview page - main system status
     */
    private void displayOverviewPage() {
        telemetry.addLine("");

        // System Status
        telemetry.addLine("🤖 INDEXING SYSTEM:");
        telemetry.addData("  State", indexingSystem.getCurrentState());
        telemetry.addData("  Artifacts", indexingSystem.getArtifactCount() + "/3");
        telemetry.addData("  Ready to Fire", indexingSystem.isReadyToFire() ? "YES ✓" : "NO");

        if (currentMode == TestMode.AUTO_COLLECTION) {
            telemetry.addData("  Auto-Detection", indexingSystem.isAutoDetectionEnabled() ? "ACTIVE ✓" : "PAUSED");
        }

        // Manual Push Mode Status
        telemetry.addData("  Manual Push Mode", indexingSystem.isManualPushMode() ? "ENABLED ✓" : "DISABLED");

        // Intake Status with explanations
        telemetry.addLine("");
        telemetry.addLine("🔄 INTAKE STATUS:");
        telemetry.addData("  Front", getDetailedIntakeStatus(indexingSystem.getArtifactInFrontIntake()));
        telemetry.addData("  Back", getDetailedIntakeStatus(indexingSystem.getArtifactInBackIntake()));

        // Artifact Locations
        telemetry.addLine("");
        telemetry.addLine("📍 ARTIFACT LOCATIONS:");
        Artifact centerArtifact = indexingSystem.getArtifactInCenter();
        Artifact frontArtifact = indexingSystem.getArtifactInFrontIntake();
        Artifact backArtifact = indexingSystem.getArtifactInBackIntake();

        telemetry.addData("  Center", centerArtifact != null ?
            String.format("%s #%d", centerArtifact.getColor(), centerArtifact.getCollectionOrder()) : "Empty");
        telemetry.addData("  Front Storage", frontArtifact != null ?
            String.format("%s #%d", frontArtifact.getColor(), frontArtifact.getCollectionOrder()) : "Empty");
        telemetry.addData("  Back Storage", backArtifact != null ?
            String.format("%s #%d", backArtifact.getColor(), backArtifact.getCollectionOrder()) : "Empty");

        // Shooter Status
        telemetry.addLine("");
        telemetry.addLine("🎯 SHOOTER:");
        telemetry.addData("  Status", shooter.isEnabled() ? "Enabled" : "Disabled");
        telemetry.addData("  Ready", shooter.isReadyToFire() ? "YES ✓" : "NO");

        // Distance sensors with enhanced display
        if (currentMode == TestMode.AUTO_COLLECTION) {
            telemetry.addLine("");
            telemetry.addLine("📏 DISTANCE SENSORS:");
            try {
                // Original laser sensors
                if (hardware.getFrontDistanceSensor() != null) {
                    double voltage = hardware.getFrontDistanceSensor().getVoltage();
                    double distance = convertVoltageToDistance(voltage);
                    boolean detected = distance > 0 && distance < indexingConfig.getArtifactDetectionDistance();
                    telemetry.addData("  Laser Front", String.format("%.1fcm%s", distance, detected ? " 🔍" : ""));

                    // Show color detection for front if artifact detected
                    if (detected) {
                        Artifact.Color color = detectArtifactColor(IndexingSystemOld.IntakeSource.FRONT);
                        telemetry.addData("    Color", color.toString());
                    }
                }
                if (hardware.getBackDistanceSensor() != null) {
                    double voltage = hardware.getBackDistanceSensor().getVoltage();
                    double distance = convertVoltageToDistance(voltage);
                    boolean detected = distance > 0 && distance < indexingConfig.getArtifactDetectionDistance();
                    telemetry.addData("  Laser Back", String.format("%.1fcm%s", distance, detected ? " 🔍" : ""));

                    // Show color detection for back if artifact detected
                    if (detected) {
                        Artifact.Color color = detectArtifactColor(IndexingSystemOld.IntakeSource.BACK);
                        telemetry.addData("    Color", color.toString());
                    }
                }

                // NEW: REV 2m Distance Sensors
                if (indexingConfig.getUseRevDistanceSensors()) {
                    double frontRevCM = hardware.getFrontLeftDistanceCM();
                    double backRevCM = hardware.getBackRightDistanceCM();

                    if (frontRevCM >= 0) {
                        boolean revDetected = frontRevCM < indexingConfig.getRevSensorDetectionThreshold();
                        telemetry.addData("  REV Front", String.format("%.1fcm%s", frontRevCM, revDetected ? " 🔍" : ""));
                    } else {
                        telemetry.addData("  REV Front", "N/A");
                    }

                    if (backRevCM >= 0) {
                        boolean revDetected = backRevCM < indexingConfig.getRevSensorDetectionThreshold();
                        telemetry.addData("  REV Back", String.format("%.1fcm%s", backRevCM, revDetected ? " 🔍" : ""));
                    } else {
                        telemetry.addData("  REV Back", "N/A");
                    }

                    telemetry.addData("  Threshold", String.format("%.1fcm", indexingConfig.getRevSensorDetectionThreshold()));
                }
            } catch (Exception e) {
                telemetry.addLine("  Sensors: Error reading - " + e.getMessage());
            }
        }

        // Errors
        String lastError = indexingSystem.getLastError();
        if (lastError != null && !lastError.isEmpty()) {
            telemetry.addLine("");
            telemetry.addLine("⚠️ LAST ERROR:");
            telemetry.addLine("  " + lastError);
        }
    }

    /**
     * Display detailed subsystems page
     */
    private void displaySubsystemsPage() {
        telemetry.addLine("");

        // Indexing System State Machine
        telemetry.addLine("🤖 INDEXING STATE MACHINE:");
        telemetry.addData("  Current State", indexingSystem.getCurrentState());
        telemetry.addData("  Operation Active", indexingSystem.isOperationInProgress() ? "YES" : "NO");

        // Explain what the system is doing
        String stateExplanation = getStateExplanation(indexingSystem.getCurrentState());
        if (!stateExplanation.isEmpty()) {
            telemetry.addLine("  Why: " + stateExplanation);
        }

        // Motor Subsystems
        telemetry.addLine("");
        telemetry.addLine("⚙️ MOTOR SUBSYSTEMS:");

        // Front Intake Motor
        if (hardware.getFrontRollerMotor() != null) {
            double power = hardware.getFrontRollerMotor().getPower();
            String status = power == 0 ? "STOPPED" : (power > 0.7 ? "COLLECTING" : "HOLDING");
            telemetry.addData("  Front Roller", String.format("%s (%.2f)", status, power));
        }

        // Back Intake Motor
        if (hardware.getBackRollerMotor() != null) {
            double power = hardware.getBackRollerMotor().getPower();
            String status = power == 0 ? "STOPPED" : (power > 0.7 ? "COLLECTING" : "HOLDING");
            telemetry.addData("  Back Roller", String.format("%s (%.2f)", status, power));
        }

        // Servo Subsystems
        telemetry.addLine("");
        telemetry.addLine("🔧 SERVO SUBSYSTEMS:");

        // Injector Servos
        if (hardware.getInjectorServoLeft() != null) {
            double power = hardware.getInjectorServoLeft().getPower();
            telemetry.addData("  Injector Left", power == 0 ? "IDLE" : String.format("ACTIVE (%.2f)", power));
        }
        if (hardware.getInjectorServoRight() != null) {
            double power = hardware.getInjectorServoRight().getPower();
            telemetry.addData("  Injector Right", power == 0 ? "IDLE" : String.format("ACTIVE (%.2f)", power));
        }

        // Transfer Servos
        if (hardware.getFrontTransferServo() != null) {
            double power = hardware.getFrontTransferServo().getPower();
            telemetry.addData("  Transfer Front", power == 0 ? "IDLE" : String.format("ACTIVE (%.2f)", power));
        }
        if (hardware.getBackTransferServo() != null) {
            double power = hardware.getBackTransferServo().getPower();
            telemetry.addData("  Transfer Back", power == 0 ? "IDLE" : String.format("ACTIVE (%.2f)", power));
        }

        // Shooter Subsystem
        telemetry.addLine("");
        telemetry.addLine("🎯 SHOOTER SUBSYSTEM:");
        telemetry.addData("  Enabled", shooter.isEnabled() ? "YES" : "NO");
        telemetry.addData("  Ready to Fire", shooter.isReadyToFire() ? "YES" : "NO");
        telemetry.addData("  At Target RPM", shooter.isAtTargetRPM() ? "YES" : "NO");

        // Hardware Status
        telemetry.addLine("");
        telemetry.addLine("🔌 HARDWARE STATUS:");

        // Count connected components
        int connectedMotors = 0;
        int totalMotors = 2;
        if (hardware.getFrontRollerMotor() != null) connectedMotors++;
        if (hardware.getBackRollerMotor() != null) connectedMotors++;

        int connectedServos = 0;
        int totalServos = 6; // IndexingSystem uses 6 servos (excludes bottom intake servos)
        if (hardware.getInjectorServoLeft() != null) connectedServos++;
        if (hardware.getInjectorServoRight() != null) connectedServos++;
        if (hardware.getUptakeServoL() != null) connectedServos++;
        if (hardware.getUptakeServoR() != null) connectedServos++;
        if (hardware.getFrontTransferServo() != null) connectedServos++;
        if (hardware.getBackTransferServo() != null) connectedServos++;

        telemetry.addData("  Motors Connected", String.format("%d/%d", connectedMotors, totalMotors));
        telemetry.addData("  Servos Connected", String.format("%d/%d", connectedServos, totalServos));

        // Show details about missing servos
        if (connectedServos < totalServos) {
            telemetry.addLine("  Missing servos handled gracefully");
            if (hardware.getInjectorServoLeft() == null) telemetry.addLine("    - Injector Left servo not found");
            if (hardware.getInjectorServoRight() == null) telemetry.addLine("    - Injector Right servo not found");
            if (hardware.getUptakeServoL() == null) telemetry.addLine("    - Uptake Left servo not found");
            if (hardware.getUptakeServoR() == null) telemetry.addLine("    - Uptake Right servo not found");
            if (hardware.getFrontTransferServo() == null) telemetry.addLine("    - Front Transfer servo not found");
            if (hardware.getBackTransferServo() == null) telemetry.addLine("    - Back Transfer servo not found");
        }
    }

    /**
     * Display raw sensor and motor data
     */
    private void displayRawDataPage() {
        telemetry.addLine("");

        // Distance Sensors
        telemetry.addLine("📏 DISTANCE SENSORS:");
        try {
            // Original goBILDA Laser Sensors (Analog)
            telemetry.addLine("  LASER SENSORS (Analog):");
            if (hardware.getFrontDistanceSensor() != null) {
                double voltage = hardware.getFrontDistanceSensor().getVoltage();
                double distance = convertVoltageToDistance(voltage);
                boolean detected = distance < indexingConfig.getArtifactDetectionDistance();
                telemetry.addData("    Front", String.format("%.3fV → %.1fcm %s",
                    voltage, distance, detected ? "DETECTED" : ""));
            } else {
                telemetry.addLine("    Front: NOT CONNECTED");
            }

            if (hardware.getBackDistanceSensor() != null) {
                double voltage = hardware.getBackDistanceSensor().getVoltage();
                double distance = convertVoltageToDistance(voltage);
                boolean detected = distance < indexingConfig.getArtifactDetectionDistance();
                telemetry.addData("    Back", String.format("%.3fV → %.1fcm %s",
                    voltage, distance, detected ? "DETECTED" : ""));
            } else {
                telemetry.addLine("    Back: NOT CONNECTED");
            }

            // NEW: REV 2m Distance Sensors (I2C)
            telemetry.addLine("  REV 2M SENSORS (I2C):");
            if (indexingConfig.getUseRevDistanceSensors()) {
                double frontRevCM = hardware.getFrontLeftDistanceCM();
                double backRevCM = hardware.getBackRightDistanceCM();
                double threshold = indexingConfig.getRevSensorDetectionThreshold();

                if (frontRevCM >= 0) {
                    boolean detected = frontRevCM < threshold;
                    telemetry.addData("    Front Left", String.format("%.1fcm %s (threshold: %.1fcm)",
                        frontRevCM, detected ? "DETECTED" : "", threshold));
                } else {
                    telemetry.addLine("    Front Left: NOT CONNECTED");
                }

                if (backRevCM >= 0) {
                    boolean detected = backRevCM < threshold;
                    telemetry.addData("    Back Right", String.format("%.1fcm %s (threshold: %.1fcm)",
                        backRevCM, detected ? "DETECTED" : "", threshold));
                } else {
                    telemetry.addLine("    Back Right: NOT CONNECTED");
                }

                telemetry.addData("    Sensor Weight", String.format("%.0f%% REV, %.0f%% Laser",
                    indexingConfig.getRevSensorWeight() * 100,
                    (1.0 - indexingConfig.getRevSensorWeight()) * 100));
            } else {
                telemetry.addLine("    REV sensors DISABLED in config");
            }
        } catch (Exception e) {
            telemetry.addLine("  Error: " + e.getMessage());
        }

        // Color Sensors - Front Intake
        telemetry.addLine("");
        telemetry.addLine("🎨 COLOR SENSORS - FRONT:");
        telemetry.addLine("    Note: Left sensor replaced with REV distance sensor");
        displayColorSensorData("  Left", hardware.getFrontLeftColorSensor(), "(REPLACED w/ REV distance)");
        displayColorSensorData("  Right", hardware.getFrontRightColorSensor());
        displayColorSensorData("  Center", hardware.getFrontCenterColorSensor());

        // Color Sensors - Back Intake
        telemetry.addLine("");
        telemetry.addLine("🎨 COLOR SENSORS - BACK:");
        telemetry.addLine("    Note: Right sensor replaced with REV distance sensor");
        displayColorSensorData("  Right", hardware.getBackRightColorSensor(), "(REPLACED w/ REV distance)");
        displayColorSensorData("  Left", hardware.getLeftRightColorSensor());
        displayColorSensorData("  Center", hardware.getBackCenterColorSensor());

        // Motor Powers
        telemetry.addLine("");
        telemetry.addLine("⚙️ MOTOR POWERS:");
        if (hardware.getFrontRollerMotor() != null) {
            telemetry.addData("  Front Roller", String.format("%.3f", hardware.getFrontRollerMotor().getPower()));
        }
        if (hardware.getBackRollerMotor() != null) {
            telemetry.addData("  Back Roller", String.format("%.3f", hardware.getBackRollerMotor().getPower()));
        }

        // Servo Powers
        telemetry.addLine("");
        telemetry.addLine("🔧 SERVO POWERS:");
        displayServoPower("  Injector L", hardware.getInjectorServoLeft());
        displayServoPower("  Injector R", hardware.getInjectorServoRight());
        displayServoPower("  Uptake L", hardware.getUptakeServoL());
        displayServoPower("  Uptake R", hardware.getUptakeServoR());
        displayServoPower("  Transfer F", hardware.getFrontTransferServo());
        displayServoPower("  Transfer B", hardware.getBackTransferServo());
    }

    /**
     * Display configuration parameters
     */
    private void displayConfigurationPage() {
        telemetry.addLine("");

        // Timing Configuration
        telemetry.addLine("⏱️ TIMING CONFIG:");
        telemetry.addData("  Intake Time", String.format("%.2fs", indexingConfig.getIntakeRollerTime()));
        telemetry.addData("  Transfer Time", String.format("%.2fs", indexingConfig.getTransferServoTime()));
        telemetry.addData("  Fire Feed Time", String.format("%.2fs", indexingConfig.getFireFeedTime()));
        telemetry.addData("  Settle Time", String.format("%.2fs", indexingConfig.getFirstArtifactSettleTime()));
        telemetry.addData("  Push Time", String.format("%.2fs", indexingConfig.getSecondArtifactPushTime()));

        // Power Configuration
        telemetry.addLine("");
        telemetry.addLine("⚡ POWER CONFIG:");
        telemetry.addData("  Intake Power", String.format("%.2f", indexingConfig.getIntakeRollerPower()));
        telemetry.addData("  Transfer Power", String.format("%.2f", indexingConfig.getTransferServoPower()));
        telemetry.addData("  Fire Feed Power", String.format("%.2f", indexingConfig.getFireFeedPower()));
        telemetry.addData("  Center Power", String.format("%.2f", indexingConfig.getCenterRollerPower()));

        // Color Detection Configuration
        telemetry.addLine("");
        telemetry.addLine("🎨 COLOR DETECTION CONFIG:");
        telemetry.addData("  Distance Threshold", String.format("%.1fcm", indexingConfig.getArtifactDetectionDistance()));
        telemetry.addData("  Color Confidence", String.format("%.2f", indexingConfig.getColorConfidenceThreshold()));
        telemetry.addData("  Sensor Debounce", String.format("%.3fs", indexingConfig.getSensorDebounceTime()));

        // Color Detection Patterns
        telemetry.addLine("");
        telemetry.addLine("🎨 COLOR PATTERNS (From Measurements):");
        telemetry.addLine("  Purple: R≈G, Blue>R/G (R:0.1-0.15, G:0.1-0.15, B:0.15-0.3)");
        telemetry.addLine("  Green: G>R/B (R:0.05-0.1, G:0.2-0.35, B:0.1-0.2)");
        telemetry.addData("  Min Score Required", String.format("%.2f", indexingConfig.getColorDetectionMinScore()));
        telemetry.addLine("");
        telemetry.addLine("  Algorithm: Pattern-based scoring");
        telemetry.addLine("  • Purple: Red≈Green similar, Blue dominant");
        telemetry.addLine("  • Green: Green dominant, lower overall values");

        // System Configuration
        telemetry.addLine("");
        telemetry.addLine("⚙️ SYSTEM CONFIG:");
        telemetry.addData("  Debug Telemetry", debugTelemetry ? "ON" : "OFF");
        telemetry.addData("  Safety Checks", indexingConfig.isEnableSafetyChecks() ? "ON" : "OFF");
        telemetry.addData("  Auto Recovery", indexingConfig.isEnableAutoRecovery() ? "ON" : "OFF");
        telemetry.addData("  Max Artifacts", String.valueOf(IndexingConfig.MAX_ARTIFACTS));
    }

    /**
     * Display diagnostics and performance data
     */
    private void displayDiagnosticsPage() {
        telemetry.addLine("");

        // Performance Metrics
        telemetry.addLine("📊 PERFORMANCE:");
        telemetry.addData("  Loop Time", String.format("%.1fms (Max: %.1fms)", (double)avgLoopTime, (double)maxLoopTime));
        telemetry.addData("  Total Loops", String.valueOf(totalLoops));
        telemetry.addData("  Loop Rate", String.format("%.1f Hz", 1000.0 / Math.max(avgLoopTime, 1)));

        // System Health
        telemetry.addLine("");
        telemetry.addLine("💚 SYSTEM HEALTH:");

        // Check sensor connectivity
        int connectedSensors = 0;
        int totalSensors = 0;

        if (hardware.getFrontDistanceSensor() != null) connectedSensors++;
        totalSensors++;
        if (hardware.getBackDistanceSensor() != null) connectedSensors++;
        totalSensors++;

        if (hardware.getFrontLeftColorSensor() != null) connectedSensors++;
        totalSensors++;
        if (hardware.getFrontRightColorSensor() != null) connectedSensors++;
        totalSensors++;
        if (hardware.getFrontCenterColorSensor() != null) connectedSensors++;
        totalSensors++;
        if (hardware.getBackRightColorSensor() != null) connectedSensors++;
        totalSensors++;
        if (hardware.getLeftRightColorSensor() != null) connectedSensors++;
        totalSensors++;
        if (hardware.getBackCenterColorSensor() != null) connectedSensors++;
        totalSensors++;

        telemetry.addData("  Sensors", String.format("%d/%d connected", connectedSensors, totalSensors));

        // Check motor connectivity
        int connectedMotors = 0;
        int totalMotors = 2; // Front and back roller motors

        if (hardware.getFrontRollerMotor() != null) connectedMotors++;
        if (hardware.getBackRollerMotor() != null) connectedMotors++;

        telemetry.addData("  Motors", String.format("%d/%d connected", connectedMotors, totalMotors));

        // Check servo connectivity
        int connectedServos = 0;
        int totalServos = 6; // IndexingSystem uses 6 servos

        if (hardware.getInjectorServoLeft() != null) connectedServos++;
        if (hardware.getInjectorServoRight() != null) connectedServos++;
        if (hardware.getUptakeServoL() != null) connectedServos++;
        if (hardware.getUptakeServoR() != null) connectedServos++;
        if (hardware.getFrontTransferServo() != null) connectedServos++;
        if (hardware.getBackTransferServo() != null) connectedServos++;

        telemetry.addData("  Servos", String.format("%d/%d connected", connectedServos, totalServos));

        // Error History
        telemetry.addLine("");
        telemetry.addLine("⚠️ ERROR STATUS:");
        String lastError = indexingSystem.getLastError();
        if (lastError != null && !lastError.isEmpty()) {
            telemetry.addLine("  Last Error: " + lastError);
        } else {
            telemetry.addLine("  No errors reported");
        }

        // Auto-Detection Statistics (if enabled)
        if (currentMode == TestMode.AUTO_COLLECTION) {
            telemetry.addLine("");
            telemetry.addLine("🔍 AUTO-DETECTION:");
            telemetry.addData("  Status", indexingSystem.isAutoDetectionEnabled() ? "ACTIVE" : "PAUSED");
            telemetry.addLine("  Managed by IndexingSystem internally");
        }

        // Memory and System Info
        telemetry.addLine("");
        telemetry.addLine("💾 SYSTEM INFO:");
        Runtime runtime = Runtime.getRuntime();
        long totalMemory = runtime.totalMemory() / 1024 / 1024; // MB
        long freeMemory = runtime.freeMemory() / 1024 / 1024; // MB
        long usedMemory = totalMemory - freeMemory;

        telemetry.addData("  Memory", String.format("%dMB used / %dMB total", usedMemory, totalMemory));
        telemetry.addData("  Uptime", String.format("%.1fs", (System.currentTimeMillis() - testStartTime) / 1000.0));
    }

    /**
     * Display controls and help
     */
    private void displayControlsPage() {
        telemetry.addLine("");

        telemetry.addLine("🎮 GAMEPAD 1 - INDEXING CONTROLS:");
        telemetry.addLine("  [A] Toggle auto-detection / Manual front");
        telemetry.addLine("  [B] Force back collect / Manual back");
        telemetry.addLine("  [X] Fire artifact");
        telemetry.addLine("  [Y] Cycle modes (+ R-stick = ESTOP)");
        telemetry.addLine("  [START] Reset system");
        telemetry.addLine("  [BACK] Toggle debug telemetry");
        telemetry.addLine("");
        telemetry.addLine("  [DPAD ↑↓] Navigate pages");
        telemetry.addLine("  [DPAD ←] Quick overview");
        telemetry.addLine("  [DPAD →] Quick raw data");
        telemetry.addLine("");
        telemetry.addLine("  [L-BUMPER] Manual servo mode");
        telemetry.addLine("  [R-BUMPER] Injector servos (manual)");
        telemetry.addLine("  [L-TRIGGER] Uptake servos (manual)");
        telemetry.addLine("  [R-TRIGGER] Transfer servos (manual)");
        telemetry.addLine("");
        telemetry.addLine("  [L-STICK] Toggle manual push mode");
        telemetry.addLine("  [R-STICK] Execute manual push (2 artifacts)");

        telemetry.addLine("");
        telemetry.addLine("🎮 GAMEPAD 2 - SHOOTER & ADVANCED:");
        telemetry.addLine("  [A] Spin up shooter");
        telemetry.addLine("  [B] Stop shooter");
        telemetry.addLine("  [X] Manual color detection test");
        telemetry.addLine("  [Y] Quick diagnostics");

        telemetry.addLine("");
        telemetry.addLine("📋 TEST MODES:");
        telemetry.addLine("  AUTO_COLLECTION - IndexingSystem auto-detects");
        telemetry.addLine("  MANUAL_CONTROL - Button-triggered only");
        telemetry.addLine("  SERVO_DEBUG - Individual servo testing");
        telemetry.addLine("  SENSOR_DEBUG - Raw sensor monitoring");

        telemetry.addLine("");
        telemetry.addLine("📊 TELEMETRY PAGES:");
        telemetry.addLine("  OVERVIEW - Main system status");
        telemetry.addLine("  DEBUG - IndexingSystem debug output");
        telemetry.addLine("  SUBSYSTEMS - Detailed component status");
        telemetry.addLine("  RAW_DATA - Sensor readings & motor powers");
        telemetry.addLine("  CONFIGURATION - All config parameters");
        telemetry.addLine("  DIAGNOSTICS - Performance & health data");
        telemetry.addLine("  CONTROLS - This help page");
    }

    /**
     * Display debug page - IndexingSystem debug output
     * This method is kept for reference but the DEBUG page is now handled directly in the main loop
     */
    private void displayDebugPage() {
        // This method is no longer used - DEBUG page telemetry is handled in main loop
        // to preserve IndexingSystem debug output
        telemetry.addLine("🔍 Debug output is shown above this message");
        telemetry.addLine("   Look for messages with emojis like:");
        telemetry.addLine("   🔍 🔄 ✅ ⚠️ 🔢");
    }

    /**
     * Update performance metrics
     */
    private void updatePerformanceMetrics() {
        long currentTime = System.currentTimeMillis();
        long loopTime = currentTime - loopStartTime;

        totalLoops++;
        maxLoopTime = Math.max(maxLoopTime, loopTime);

        // Calculate rolling average (last 100 loops)
        if (totalLoops == 1) {
            avgLoopTime = loopTime;
        } else {
            avgLoopTime = (long)(avgLoopTime * 0.99 + loopTime * 0.01);
        }
    }

    /**
     * Get detailed intake status with explanation
     */
    private String getDetailedIntakeStatus(Artifact artifact) {
        if (artifact != null) {
            return String.format("STORING %s #%d (50%% power)",
                artifact.getColor(), artifact.getCollectionOrder());
        } else {
            return "Ready to collect (100% power)";
        }
    }

    /**
     * Get explanation of what the indexing system is doing in current state
     */
    private String getStateExplanation(IndexingSystemOld.SystemState state) {
        switch (state) {
            case IDLE:
                return "Waiting for artifact detection or fire command";
            case COLLECTING:
                return "Running intake rollers to collect detected artifact";
            case TRANSFERRING:
                return "Moving artifact from intake to center using transfer servos";
            case PUSHING:
                return "Second artifact pushing first artifact to storage intake";
            case FIRING:
                return "Feeding artifact from center to shooter";
            case READY_TO_FIRE:
                return "Artifact in center, waiting for fire signal";
            case ERROR:
                return "System error - check diagnostics page";
            default:
                return "";
        }
    }

    /**
     * Display color sensor data for telemetry with optional note
     */
    private void displayColorSensorData(String name, com.qualcomm.robotcore.hardware.NormalizedColorSensor sensor, String note) {
        if (sensor != null) {
            try {
                com.qualcomm.robotcore.hardware.NormalizedRGBA colors = sensor.getNormalizedColors();

                // Calculate scores for both colors
                double purpleScore = indexingConfig.calculateColorConfidence(colors.red, colors.green, colors.blue, "PURPLE");
                double greenScore = indexingConfig.calculateColorConfidence(colors.red, colors.green, colors.blue, "GREEN");

                String detectedColor = indexingConfig.detectArtifactColor(colors.red, colors.green, colors.blue);

                telemetry.addData(name, String.format("R:%.3f G:%.3f B:%.3f", colors.red, colors.green, colors.blue));
                telemetry.addData(name + " Scores", String.format("P:%.2f G:%.2f → %s", purpleScore, greenScore, detectedColor));
            } catch (Exception e) {
                telemetry.addData(name, "ERROR: " + e.getMessage());
            }
        } else {
            telemetry.addData(name, note != null ? note : "NOT CONNECTED");
        }
    }

    /**
     * Display color sensor data for telemetry
     */
    private void displayColorSensorData(String name, com.qualcomm.robotcore.hardware.NormalizedColorSensor sensor) {
        displayColorSensorData(name, sensor, null);
    }

    /**
     * Display servo power for telemetry
     */
    private void displayServoPower(String name, com.qualcomm.robotcore.hardware.CRServo servo) {
        if (servo != null) {
            try {
                double power = servo.getPower();
                telemetry.addData(name, String.format("%.3f", power));
            } catch (Exception e) {
                telemetry.addData(name, "ERROR");
            }
        } else {
            telemetry.addData(name, "N/C");
        }
    }

    /**
     * Get human-readable intake status
     */
    private String getIntakeStatus(Artifact artifact) {
        if (artifact != null) {
            int storagePowerPercent = (int)(indexingConfig.getIntakeStoragePower() * 100);
            return "STORING " + artifact.getColor() + " #" + artifact.getCollectionOrder() + " (" + storagePowerPercent + "% power)";
        } else {
            return "Empty (100% power - ready to collect)";
        }
    }
}

