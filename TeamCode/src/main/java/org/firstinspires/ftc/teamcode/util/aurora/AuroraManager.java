/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Comprehensive robot system manager for coordinated subsystem control
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * AURORA - Advanced Unified Robot Operating & Response Architecture
 * Robot System Manager - Central coordination hub for all robot subsystems
 *
 * Features:
 * - Unified subsystem management
 * - Cross-system optimization
 * - Predictive maintenance
 * - Emergency protocols
 * - Performance analytics dashboard
 * - Dual-driver mode support
 */
public class AuroraManager {

    // Driver mode configuration
    public enum DriverMode {
        SINGLE_DRIVER,    // Traditional single driver controls everything
        DUAL_DRIVER       // Driver 1: Movement, Driver 2: Utilities/Tools
    }

    // Subsystems
    private SmartMechanumDrive driveSystem;
    private EnhancedDecodeHelper shooterSystem;
    private PerformanceMonitor globalMonitor;
    private MovementRecorder movementRecorder;

    // Hardware references
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private VoltageSensor voltageSensor;

    // System state
    private ElapsedTime systemUptime = new ElapsedTime();
    private boolean systemsHealthy = true;
    private String lastError = "";

    // Initialization error tracking
    private String driveInitError = "";
    private String shooterInitError = "";
    private boolean driveInitialized = false;
    private boolean shooterInitialized = false;

    // Coordination flags
    private boolean shootingMode = false;
    private boolean precisionDriving = false;
    private SmartMechanumDrive.DriveMode savedDriveMode = SmartMechanumDrive.DriveMode.SPORT; // Track original mode

    // Driver mode configuration
    private DriverMode currentDriverMode = DriverMode.DUAL_DRIVER;
    private boolean prevModeToggleButton = false;

    // Recording mode support
    private boolean recordingMode = false;

    /**
     * Initialize all robot systems
     */
    public AuroraManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        globalMonitor = new PerformanceMonitor();
        initializeSystems();
    }

    /**
     * Initialize all subsystems with error handling
     */
    private void initializeSystems() {
        // Initialize drive system with graceful error handling
        try {
            DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
            DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
            DcMotor leftBack = hardwareMap.get(DcMotor.class, "backLeft");
            DcMotor rightBack = hardwareMap.get(DcMotor.class, "backRight");

            driveSystem = new SmartMechanumDrive(leftFront, rightFront, leftBack, rightBack, null, voltageSensor);
            driveInitialized = true;
            driveInitError = ""; // Clear any previous error
            telemetry.addLine("✅ Drive system initialized successfully");

        } catch (Exception e) {
            driveSystem = null;
            driveInitialized = false;
            driveInitError = e.getMessage();
            telemetry.addLine("⚠️ Drive system failed: " + e.getMessage());
            telemetry.addLine("   Check motor names in hardware configuration:");
            telemetry.addLine("   Expected: leftFront, rightFront, leftBack, rightBack");
        }

        // Initialize shooter system with graceful error handling
        try {
            shooterSystem = new EnhancedDecodeHelper(hardwareMap);
            shooterInitialized = true;
            shooterInitError = ""; // Clear any previous error
            telemetry.addLine("✅ Shooter system initialized successfully");

        } catch (Exception e) {
            shooterSystem = null;
            shooterInitialized = false;
            shooterInitError = e.getMessage();
            telemetry.addLine("⚠️ Shooter system failed: " + e.getMessage());
        }

        // Initialize movement recorder (this shouldn't fail)
        try {
            movementRecorder = new MovementRecorder();
            telemetry.addLine("✅ Movement recorder initialized successfully");
        } catch (Exception e) {
            movementRecorder = null;
            telemetry.addLine("⚠️ Movement recorder failed: " + e.getMessage());
        }

        // System is healthy if at least one major subsystem works, or if we want to allow basic operation
        systemsHealthy = true; // Always allow basic operation for debugging

        if (!driveInitialized && !shooterInitialized) {
            telemetry.addLine("⚠️ WARNING: No major subsystems initialized");
            telemetry.addLine("   Gamepad controls will still work for testing");
        }

        telemetry.addLine("🎮 Gamepad controls enabled");
        lastError = ""; // Clear any previous errors
    }

    /**
     * Main update loop - call this every cycle
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        globalMonitor.updateLoopTiming();

        if (!systemsHealthy) {
            handleSystemFailure();
            return;
        }

        // Handle system-level controls (mode switching, etc.)
        handleSystemControls(gamepad1, gamepad2);

        // Handle semi-auto actions
        handleSemiAutoActions(gamepad1);

        // Update subsystems with coordination
        updateDriveSystem(gamepad1, gamepad2);
        updateShooterSystem(gamepad1, gamepad2);
        updateSystemCoordination();

        // Monitor system health
        checkSystemHealth();
    }

    /**
     * Update drive system with context awareness and dual-driver support
     */
    private void updateDriveSystem(Gamepad gamepad1, Gamepad gamepad2) {
        if (driveSystem == null) return;

        // Determine which gamepad controls driving based on current mode
        Gamepad driveGamepad = (currentDriverMode == DriverMode.DUAL_DRIVER) ? gamepad1 : gamepad1;

        // In dual driver mode, only gamepad1 controls movement
        // In single driver mode, gamepad1 controls everything
        if (currentDriverMode == DriverMode.DUAL_DRIVER) {
            // Driver 1 (gamepad1) controls movement only
            handleDriveControls(gamepad1);
        } else {
            // Single driver mode - gamepad1 controls everything
            handleDriveControls(gamepad1);
        }

        // Adjust drive behavior based on robot state with proper mode restoration
        if (shootingMode || precisionDriving) {
            // Save current mode before switching to precision (if not already saved)
            if (!shootingMode && !precisionDriving) {
                savedDriveMode = driveSystem.getCurrentMode();
            }
            driveSystem.setCurrentMode(SmartMechanumDrive.DriveMode.PRECISION);
        } else {
            // Restore saved mode when neither shooting nor precision driving
            driveSystem.setCurrentMode(savedDriveMode);
        }

        driveSystem.update();
    }

    /**
     * Handle drive-specific controls
     */
    private void handleDriveControls(Gamepad gamepad) {
        // Set main joystick inputs for basic movement
        double axial = -gamepad.right_stick_y;   // Forward/backward (inverted)
        double lateral = gamepad.right_stick_x;  // Strafe left/right
        double yaw = gamepad.left_stick_x;     // Rotation

        // Pass the main joystick inputs to the drive system
        driveSystem.setDriveInputs(axial, lateral, yaw);

        // Drive mode switching
        if (gamepad.dpad_up) {
            // Fine forward movement
            driveSystem.setFineMovement(0.0, 0.3, 0.0); // Forward strafe
        } else if (gamepad.dpad_down) {
            // Fine backward movement
            driveSystem.setFineMovement(0.0, -0.3, 0.0); // Backward strafe
        } else if (gamepad.dpad_left) {
            // Fine left strafe
            driveSystem.setFineMovement(-0.3, 0.0, 0.0); // Left strafe
        } else if (gamepad.dpad_right) {
            // Fine right strafe
            driveSystem.setFineMovement(0.3, 0.0, 0.0); // Right strafe
        } else {
            // No fine movement
            driveSystem.setFineMovement(0.0, 0.0, 0.0);
        }

        // Fine rotation control with bumpers
        if (gamepad.right_bumper) {
            // Fine clockwise rotation
            driveSystem.setFineRotation(0.3); // Positive rotation
        } else if (gamepad.left_bumper) {
            // Fine counter-clockwise rotation
            driveSystem.setFineRotation(-0.3); // Negative rotation
        } else {
            // No fine rotation
            driveSystem.setFineRotation(0.0);
        }

        // Slow drive toggle with left stick button - use precision mode instead
        if (gamepad.left_stick_button) {
            driveSystem.setCurrentMode(SmartMechanumDrive.DriveMode.PRECISION);
        }

        // Field relative toggle with left stick button + right stick button
        if (gamepad.left_stick_button && gamepad.right_stick_button) {
            driveSystem.toggleFieldRelative();
        }
    }

    /**
     * Update shooter system with dual-driver support
     */
    private void updateShooterSystem(Gamepad gamepad1, Gamepad gamepad2) {
        if (shooterSystem == null) return;

        // Determine which gamepad controls utilities based on current mode
        if (currentDriverMode == DriverMode.DUAL_DRIVER) {
            // Dual driver mode - gamepad2 controls shooting
            handleShooterControls(gamepad2);
        } else {
            // Single driver mode - gamepad1 controls everything, gamepad2 as backup
            handleShooterControls(gamepad1);
            // Also allow gamepad2 as backup in single driver mode
            if (gamepad2.a || gamepad2.y || gamepad2.x || gamepad2.b || gamepad2.right_trigger > 0.05) {
                handleShooterControls(gamepad2);
            }
        }
    }

    /**
     * Handle shooter controls
     */
    private void handleShooterControls(Gamepad gamepad) {
        // Determine shooting preset based on bumper inputs
        ShooterConfig.ShooterPreset currentPreset = ShooterConfig.ShooterPreset.LONG_RANGE; // default

        if (gamepad.right_bumper) {
            currentPreset = ShooterConfig.ShooterPreset.LONG_RANGE;
        } else if (gamepad.left_bumper) {
            currentPreset = ShooterConfig.ShooterPreset.SHORT_RANGE;
        }

        // Smart shooting with gamepad.a (single shot) and gamepad.y (continuous)
        boolean singleShot = gamepad.a;
        boolean continuousShot = gamepad.y;

        if (singleShot) {
            shooterSystem.handleShootButton(true, currentPreset);
            shootingMode = true;
        } else if (continuousShot) {
            // For continuous shooting, use rapid fire preset
            shooterSystem.handleShootButton(true, ShooterConfig.ShooterPreset.RAPID_FIRE);
            shootingMode = true;
        } else {
            shooterSystem.handleShootButton(false, currentPreset);
            shootingMode = false;
        }

        // Manual shooter control with right trigger - use startShooter instead
        if (gamepad.right_trigger > 0.05) {
            shooterSystem.startShooter();
            shootingMode = true;
        }

        // Manual feed servo control with gamepad.b - use fireSingleShot for feed control
        if (gamepad.b) {
            shooterSystem.fireSingleShot();
        }

        // Emergency stop all shooting with gamepad.x
        if (gamepad.x) {
            shooterSystem.reset();
            shootingMode = false;
        }
    }

    /**
     * Handle semi-auto actions (gamepad1 only)
     */
    private void handleSemiAutoActions(Gamepad gamepad1) {
        // Only gamepad1 can trigger semi-auto actions in both modes
        // Note: Bumpers and D-pad are now used for fine movement controls

        // Semi-auto: Move to basket and shoot (Y button)
        if (gamepad1.y) {
            // Future implementation - coordinate with autonomous system
            telemetry.addData("Semi-Auto", "Move to basket & shoot requested");
        }

        // Semi-auto: Precision align to scoring position (B button)
        if (gamepad1.b) {
            // Set precision mode temporarily
            if (driveSystem != null) {
                driveSystem.setCurrentMode(SmartMechanumDrive.DriveMode.PRECISION);
            }
            precisionDriving = true;
        } else {
            precisionDriving = false;
        }

        // Semi-auto: Quick retreat (A button)
        if (gamepad1.a) {
            // Future implementation - quick movement sequence
            telemetry.addData("Semi-Auto", "Quick retreat requested");
        }

        // Semi-auto actions moved to other buttons since bumpers/D-pad are now used for fine control:
        // Semi-auto: Rotate to heading 0 (Left trigger)
        if (gamepad1.left_trigger > 0.5) {
            // Future implementation - auto-rotate to field heading 0
            telemetry.addData("Semi-Auto", "Rotate to zero requested");
        }

        // Semi-auto: Strafe to wall align (Right trigger + Y)
        if (gamepad1.right_trigger > 0.5 && gamepad1.y) {
            // Future implementation - wall alignment
            telemetry.addData("Semi-Auto", "Wall align requested");
        }

        // Cancel any semi-auto with back button
        if (gamepad1.back && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            // Cancel semi-auto (but not if D-pad is being used for fine movement or mode toggle)
            precisionDriving = false;
            if (driveSystem != null) {
                driveSystem.setCurrentMode(SmartMechanumDrive.DriveMode.SPORT);
            }
        }
    }

    /**
     * Handle driver mode switching and system controls
     */
    public void handleSystemControls(Gamepad gamepad1, Gamepad gamepad2) {
        // Toggle driver mode with select + dpad_left on gamepad1
        boolean modeTogglePressed = gamepad1.back && gamepad1.dpad_left;
        if (modeTogglePressed && !prevModeToggleButton) {
            toggleDriverMode();
        }
        prevModeToggleButton = modeTogglePressed;
    }

    /**
     * Toggle between single and dual driver modes
     */
    public void toggleDriverMode() {
        currentDriverMode = (currentDriverMode == DriverMode.SINGLE_DRIVER) ?
            DriverMode.DUAL_DRIVER : DriverMode.SINGLE_DRIVER;

        telemetry.addLine("🔄 Driver mode: " + getDriverModeString());
    }

    /**
     * Set driver mode explicitly
     */
    public void setDriverMode(DriverMode mode) {
        currentDriverMode = mode;
    }

    /**
     * Get current driver mode
     */
    public DriverMode getDriverMode() {
        return currentDriverMode;
    }

    /**
     * Get driver mode as user-friendly string
     */
    public String getDriverModeString() {
        return currentDriverMode == DriverMode.SINGLE_DRIVER ? "Single Driver" : "Dual Driver";
    }

    /**
     * Coordinate between subsystems for optimal performance
     */
    private void updateSystemCoordination() {
        // Battery-based coordination
        double voltage = voltageSensor != null ? voltageSensor.getVoltage() : 12.0;
        globalMonitor.recordVoltage(voltage);

        // Let the SmartMechanumDrive handle its own voltage-based mode switching
        // with sustained voltage monitoring to avoid false triggers during motor startup
        // Only handle extremely critical voltage levels here for system-wide coordination
        if (voltage < 9.5) {
            // Only intervene at critically low voltage (below what SmartMechanumDrive handles)
            if (driveSystem != null) {
                driveSystem.setCurrentMode(SmartMechanumDrive.DriveMode.EFFICIENCY);
            }
            if (shooterSystem != null) {
                shooterSystem.getConfig().setPreset(ShooterConfig.ShooterPreset.BATTERY_SAVER);
            }
        }

        // Temperature-based coordination (future expansion)
        // IMU-based stability coordination (future expansion)
    }

    /**
     * Monitor overall system health
     */
    private void checkSystemHealth() {
        // Check individual subsystem health but be more tolerant of missing hardware
        boolean driveHealthy = driveSystem == null || driveSystem.getCurrentVoltage() > 10.0;
        boolean shooterHealthy = shooterSystem == null || !shooterSystem.isEmergencyStop();
        boolean performanceHealthy = globalMonitor == null || !globalMonitor.isPerformanceDegraded();

        // System is healthy if no critical failures exist
        // Allow operation even if some subsystems are missing (null)
        systemsHealthy = driveHealthy && shooterHealthy && performanceHealthy;

        // Only set error if there's an actual problem, not just missing hardware
        if (!systemsHealthy) {
            if (driveSystem != null && driveSystem.getCurrentVoltage() <= 10.0) {
                lastError = "Critical battery voltage: " + driveSystem.getCurrentVoltage() + "V";
            } else if (shooterSystem != null && shooterSystem.isEmergencyStop()) {
                lastError = "Shooter emergency stop active";
            } else if (globalMonitor != null && globalMonitor.isPerformanceDegraded()) {
                lastError = "Performance degraded";
            } else {
                lastError = "Unknown system health issue";
            }
        } else {
            lastError = ""; // Clear error when healthy
        }
    }

    /**
     * Handle system failures gracefully
     */
    private void handleSystemFailure() {
        telemetry.addLine("⚠️ SYSTEM FAILURE DETECTED");
        telemetry.addData("Error", lastError);
        telemetry.addLine("Attempting recovery...");

        // Try to recover
        try {
            if (shooterSystem != null && shooterSystem.isEmergencyStop()) {
                shooterSystem.resetEmergencyStop();
            }

            systemsHealthy = true;
            lastError = "";

        } catch (Exception e) {
            telemetry.addLine("Recovery failed: " + e.getMessage());
        }
    }

    /**
     * Get comprehensive system status
     * Note: This method is deprecated in favor of SmartTelemetryManager
     * Only kept for backward compatibility or debugging purposes
     */
    @Deprecated
    public void updateTelemetry() {
        telemetry.addLine("=== ROBOT SYSTEM MANAGER ===");
        telemetry.addData("Uptime", "%.1fs", systemUptime.seconds());
        telemetry.addData("System Health", systemsHealthy ? "✅ HEALTHY" : "⚠️ DEGRADED");
        telemetry.addData("Shooting Mode", shootingMode ? "🔥 ACTIVE" : "⏸ Standby");

        if (voltageSensor != null) {
            double voltage = voltageSensor.getVoltage();
            String batteryStatus = voltage > 11.5 ? "🔋 Good" : voltage > 10.5 ? "⚠️ Low" : "🪫 Critical";
            telemetry.addData("Battery", "%.1fV %s", voltage, batteryStatus);
        }

        // Note: Individual subsystem telemetry is now handled by SmartTelemetryManager
        // using structured data access rather than direct telemetry calls

        if (!lastError.isEmpty()) {
            telemetry.addLine("⚠️ " + lastError);
        }
    }

    /**
     * Autonomous shooting sequence with system coordination
     */
    public void autonomousShootSequence(int shots, ShooterConfig.ShooterPreset preset) {
        if (shooterSystem == null) return;

        // Set precision driving mode during autonomous shooting
        precisionDriving = true;
        if (driveSystem != null) {
            driveSystem.setCurrentMode(SmartMechanumDrive.DriveMode.PRECISION);
        }

        // Execute shooting sequence
        shooterSystem.autoShootSmart(shots, false, preset);

        // Reset to normal driving
        precisionDriving = false;
        if (driveSystem != null) {
            driveSystem.setCurrentMode(SmartMechanumDrive.DriveMode.NORMAL);
        }
    }

    /**
     * Emergency stop all systems
     */
    public void emergencyStopAll() {
        if (shooterSystem != null) {
            shooterSystem.emergencyStop();
        }

        // Stop drive motors (would need access to drive motors)
        systemsHealthy = false;
        lastError = "Emergency stop activated";
    }

    // Getters for subsystem access
    public SmartMechanumDrive getDriveSystem() { return driveSystem; }
    public EnhancedDecodeHelper getShooterSystem() { return shooterSystem; }
    public PerformanceMonitor getGlobalMonitor() { return globalMonitor; }
    public MovementRecorder getMovementRecorder() { return movementRecorder; }
    public boolean isSystemsHealthy() { return systemsHealthy; }
    public boolean isRecordingMode() { return recordingMode; }

    /**
     * Get initialization status and error information
     */
    public boolean isDriveSystemInitialized() { return driveInitialized; }
    public boolean isShooterSystemInitialized() { return shooterInitialized; }
    public String getDriveInitError() { return driveInitError; }
    public String getShooterInitError() { return shooterInitError; }

    /**
     * Enable/disable recording mode
     */
    public void setRecordingMode(boolean enabled) {
        recordingMode = enabled;
    }

    /**
     * Update recording system with current robot state
     */
    public void updateRecording(Gamepad gamepad1, Gamepad gamepad2) {
        if (!recordingMode || movementRecorder == null) return;

        // Get current joystick inputs
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Collect current subsystem states
        java.util.Map<String, Object> actions = collectCurrentActions(gamepad1, gamepad2);

        // Record the waypoint
        movementRecorder.recordWaypoint(axial, lateral, yaw, actions);
    }

    /**
     * Collect current robot state for recording
     */
    private java.util.Map<String, Object> collectCurrentActions(Gamepad gamepad1, Gamepad gamepad2) {
        java.util.Map<String, Object> actions = new java.util.HashMap<>();

        // Record drive system state
        if (driveSystem != null) {
            SmartMechanumDrive.DriveSystemData driveData = driveSystem.getTelemetryData();
            actions.put("drive_mode", driveData.driveMode);
            actions.put("field_relative", driveData.fieldRelative);
            actions.put("fine_movement", driveSystem.getFineMovement());
        }

        // Record shooter system state
        if (shooterSystem != null) {
            actions.put("shooter_active", shootingMode);
            actions.put("precision_driving", precisionDriving);
        }

        // Record gamepad states
        actions.put("gp1_a", gamepad1.a);
        actions.put("gp1_b", gamepad1.b);
        actions.put("gp1_x", gamepad1.x);
        actions.put("gp1_y", gamepad1.y);
        actions.put("gp1_left_bumper", gamepad1.left_bumper);
        actions.put("gp1_right_bumper", gamepad1.right_bumper);
        actions.put("gp1_left_trigger", gamepad1.left_trigger);
        actions.put("gp1_right_trigger", gamepad1.right_trigger);
        actions.put("gp1_dpad_up", gamepad1.dpad_up);
        actions.put("gp1_dpad_down", gamepad1.dpad_down);
        actions.put("gp1_dpad_left", gamepad1.dpad_left);
        actions.put("gp1_dpad_right", gamepad1.dpad_right);

        return actions;
    }
}
