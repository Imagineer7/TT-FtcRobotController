package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * GamepadConfig - Centralized gamepad control configuration for AURORA System
 *
 * This class provides:
 * - Button mapping constants for consistent control schemes
 * - Support for SINGLE_GAMEPAD and DUAL_GAMEPAD modes
 * - Input sensitivity configuration
 * - Dead zone settings for analog inputs
 * - Easy-to-change control schemes in one place
 *
 * Control Schemes:
 *
 * DUAL_GAMEPAD MODE (Recommended for competition):
 * ──────────────────────────────────────────────
 * Gamepad 1 (Driver):
 *   - Left Stick X: Rotation (Yaw)
 *   - Right Stick Y: Forward/Backward (Axial)
 *   - Right Stick X: Strafe Left/Right (Lateral)
 *   - Left Bumper: Toggle slow mode
 *   - Right Bumper: Toggle field-centric mode
 *   - Y Button: Reset IMU heading
 *   - D-Pad: Fine control movement
 *
 * Gamepad 2 (Operator):
 *   - Y Button: Fire shots (Long Range preset)
 *   - A Button: Fire shots (Short Range preset)
 *   - B Button: Fire shots (Auto/Mid Range preset)
 *   - X Button: Intake toggle (on/off)
 *   - D-Pad Up: Manual intake in
 *   - D-Pad Down: Manual intake out
 *   - D-Pad Left: Manual turret left
 *   - D-Pad Right: Manual turret right
 *   - Right Trigger: Manual shooter power control
 *   - Left Trigger: Warmup mode (hold to enable)
 *
 * SINGLE_GAMEPAD MODE (For testing/practice):
 * ──────────────────────────────────────────────
 * Gamepad 1 (All Controls):
 *   - Left Stick X: Rotation (Yaw)
 *   - Right Stick Y: Forward/Backward (Axial)
 *   - Right Stick X: Strafe Left/Right (Lateral)
 *   - Bumpers: Slow/Field-centric toggle
 *   - Face Buttons: Shooter presets
 *   - Triggers: Shooter control
 *   - D-Pad: Manual mechanism control
 */
public class GamepadConfig {

    // ═══════════════════════════════════════════════════════════════════════
    // CONTROL MODE
    // ═══════════════════════════════════════════════════════════════════════

    public enum ControlMode {
        SINGLE_GAMEPAD,  // One gamepad controls everything
        DUAL_GAMEPAD     // Two gamepads: driver + operator
    }

    private ControlMode controlMode = ControlMode.DUAL_GAMEPAD;

    // ═══════════════════════════════════════════════════════════════════════
    // SENSITIVITY SETTINGS
    // ═══════════════════════════════════════════════════════════════════════

    /** Dead zone for analog stick inputs (0.0 to 1.0) */
    public static final double STICK_DEAD_ZONE = 0.05;

    /** Dead zone for trigger inputs (0.0 to 1.0) */
    public static final double TRIGGER_DEAD_ZONE = 0.1;

    /** Drive sensitivity multiplier (0.0 to 1.0) */
    private double driveSensitivity = 1.0;

    /** Rotation sensitivity multiplier (0.0 to 1.0) */
    private double rotationSensitivity = 0.8;

    /** Slow mode multiplier */
    public static final double SLOW_MODE_MULTIPLIER = 0.33;

    // ═══════════════════════════════════════════════════════════════════════
    // GAMEPAD REFERENCES
    // ═══════════════════════════════════════════════════════════════════════

    private Gamepad driverGamepad;
    private Gamepad operatorGamepad;

    // ═══════════════════════════════════════════════════════════════════════
    // BUTTON DEBOUNCE TRACKING
    // ═══════════════════════════════════════════════════════════════════════

    // Driver button states (for edge detection)
    private boolean prevDriverSlowMode = false;
    private boolean prevDriverFieldCentric = false;
    private boolean prevDriverResetHeading = false;

    // Operator button states (for edge detection)
    private boolean prevOperatorLongRange = false;
    private boolean prevOperatorShortRange = false;
    private boolean prevOperatorAutoRange = false;
    private boolean prevOperatorIntakeToggle = false;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTORS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Create GamepadConfig for DUAL_GAMEPAD mode
     *
     * @param driverGamepad Gamepad for driving (gamepad1)
     * @param operatorGamepad Gamepad for mechanisms (gamepad2)
     */
    public GamepadConfig(Gamepad driverGamepad, Gamepad operatorGamepad) {
        this.driverGamepad = driverGamepad;
        this.operatorGamepad = operatorGamepad;
        this.controlMode = ControlMode.DUAL_GAMEPAD;
    }

    /**
     * Create GamepadConfig for SINGLE_GAMEPAD mode
     *
     * @param gamepad Single gamepad for all controls
     */
    public GamepadConfig(Gamepad gamepad) {
        this.driverGamepad = gamepad;
        this.operatorGamepad = gamepad;  // Use same gamepad for both
        this.controlMode = ControlMode.SINGLE_GAMEPAD;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // DRIVER CONTROLS (GAMEPAD 1)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Get forward/backward input (axial movement)
     * @return -1.0 to 1.0 (negative = forward, positive = backward)
     */
    public double getAxial() {
        return applyDeadZone(-driverGamepad.right_stick_y, STICK_DEAD_ZONE) * driveSensitivity;
    }

    /**
     * Get strafe left/right input (lateral movement)
     * @return -1.0 to 1.0 (negative = left, positive = right)
     */
    public double getLateral() {
        return applyDeadZone(driverGamepad.right_stick_x, STICK_DEAD_ZONE) * driveSensitivity;
    }

    /**
     * Get rotation input (yaw)
     * @return -1.0 to 1.0 (negative = counterclockwise, positive = clockwise)
     */
    public double getYaw() {
        return applyDeadZone(driverGamepad.left_stick_x, STICK_DEAD_ZONE) * rotationSensitivity;
    }

    /**
     * Check if slow mode button was pressed (rising edge)
     * @return true if button was just pressed
     */
    public boolean isSlowModePressed() {
        boolean current = driverGamepad.left_bumper;
        boolean pressed = current && !prevDriverSlowMode;
        prevDriverSlowMode = current;
        return pressed;
    }

    /**
     * Check if field-centric toggle button was pressed (rising edge)
     * @return true if button was just pressed
     */
    public boolean isFieldCentricPressed() {
        boolean current = driverGamepad.right_bumper;
        boolean pressed = current && !prevDriverFieldCentric;
        prevDriverFieldCentric = current;
        return pressed;
    }

    /**
     * Check if reset heading button was pressed (rising edge)
     * @return true if button was just pressed
     */
    public boolean isResetHeadingPressed() {
        boolean current = driverGamepad.y;
        boolean pressed = current && !prevDriverResetHeading;
        prevDriverResetHeading = current;
        return pressed;
    }

    /**
     * Check if D-pad up is pressed
     */
    public boolean isDpadUp() {
        return driverGamepad.dpad_up;
    }

    /**
     * Check if D-pad down is pressed
     */
    public boolean isDpadDown() {
        return driverGamepad.dpad_down;
    }

    /**
     * Check if D-pad left is pressed
     */
    public boolean isDpadLeft() {
        return driverGamepad.dpad_left;
    }

    /**
     * Check if D-pad right is pressed
     */
    public boolean isDpadRight() {
        return driverGamepad.dpad_right;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OPERATOR CONTROLS (GAMEPAD 2)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Check if long range shoot button was pressed (rising edge)
     * @return true if Y button was just pressed
     */
    public boolean isLongRangeShootPressed() {
        boolean current = operatorGamepad.y;
        boolean pressed = current && !prevOperatorLongRange;
        prevOperatorLongRange = current;
        return pressed;
    }

    /**
     * Check if long range shoot button is held
     * @return true if Y button is currently held
     */
    public boolean isLongRangeShootHeld() {
        return operatorGamepad.y;
    }

    /**
     * Check if short range shoot button was pressed (rising edge)
     * @return true if A button was just pressed
     */
    public boolean isShortRangeShootPressed() {
        boolean current = operatorGamepad.a;
        boolean pressed = current && !prevOperatorShortRange;
        prevOperatorShortRange = current;
        return pressed;
    }

    /**
     * Check if short range shoot button is held
     * @return true if A button is currently held
     */
    public boolean isShortRangeShootHeld() {
        return operatorGamepad.a;
    }

    /**
     * Check if auto/mid range shoot button was pressed (rising edge)
     * @return true if B button was just pressed
     */
    public boolean isAutoRangeShootPressed() {
        boolean current = operatorGamepad.b;
        boolean pressed = current && !prevOperatorAutoRange;
        prevOperatorAutoRange = current;
        return pressed;
    }

    /**
     * Check if auto/mid range shoot button is held
     * @return true if B button is currently held
     */
    public boolean isAutoRangeShootHeld() {
        return operatorGamepad.b;
    }

    /**
     * Check if intake toggle button was pressed (rising edge)
     * @return true if X button was just pressed
     */
    public boolean isIntakeTogglePressed() {
        boolean current = operatorGamepad.x;
        boolean pressed = current && !prevOperatorIntakeToggle;
        prevOperatorIntakeToggle = current;
        return pressed;
    }

    /**
     * Check if manual intake in is pressed (D-pad up)
     */
    public boolean isManualIntakeIn() {
        return operatorGamepad.dpad_up;
    }

    /**
     * Check if manual intake out is pressed (D-pad down)
     */
    public boolean isManualIntakeOut() {
        return operatorGamepad.dpad_down;
    }

    /**
     * Check if manual turret left is pressed (D-pad left)
     */
    public boolean isManualTurretLeft() {
        return operatorGamepad.dpad_left;
    }

    /**
     * Check if manual turret right is pressed (D-pad right)
     */
    public boolean isManualTurretRight() {
        return operatorGamepad.dpad_right;
    }

    /**
     * Get manual shooter power control (right trigger)
     * @return 0.0 to 1.0
     */
    public double getManualShooterPower() {
        return applyDeadZone(operatorGamepad.right_trigger, TRIGGER_DEAD_ZONE);
    }

    /**
     * Check if warmup mode is active (left trigger held)
     * @return true if left trigger is pressed beyond threshold
     */
    public boolean isWarmupActive() {
        return operatorGamepad.left_trigger > TRIGGER_DEAD_ZONE;
    }

    /**
     * Get warmup trigger value
     * @return 0.0 to 1.0
     */
    public double getWarmupTrigger() {
        return applyDeadZone(operatorGamepad.left_trigger, TRIGGER_DEAD_ZONE);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // CONFIGURATION METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Set the control mode
     */
    public void setControlMode(ControlMode mode) {
        this.controlMode = mode;
        if (mode == ControlMode.SINGLE_GAMEPAD && operatorGamepad != driverGamepad) {
            // If switching to single gamepad, use driver gamepad for both
            operatorGamepad = driverGamepad;
        }
    }

    /**
     * Get current control mode
     */
    public ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Set drive sensitivity
     * @param sensitivity 0.0 to 1.0
     */
    public void setDriveSensitivity(double sensitivity) {
        this.driveSensitivity = Math.max(0.0, Math.min(1.0, sensitivity));
    }

    /**
     * Set rotation sensitivity
     * @param sensitivity 0.0 to 1.0
     */
    public void setRotationSensitivity(double sensitivity) {
        this.rotationSensitivity = Math.max(0.0, Math.min(1.0, sensitivity));
    }

    /**
     * Get drive sensitivity
     */
    public double getDriveSensitivity() {
        return driveSensitivity;
    }

    /**
     * Get rotation sensitivity
     */
    public double getRotationSensitivity() {
        return rotationSensitivity;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Apply dead zone to analog input
     * @param value Input value
     * @param deadZone Dead zone threshold
     * @return Adjusted value with dead zone applied
     */
    private double applyDeadZone(double value, double deadZone) {
        if (Math.abs(value) < deadZone) {
            return 0.0;
        }
        // Scale the remaining range to maintain smooth control
        return (value - Math.signum(value) * deadZone) / (1.0 - deadZone);
    }

    /**
     * Reset all button state tracking (useful when switching modes)
     */
    public void resetButtonStates() {
        prevDriverSlowMode = false;
        prevDriverFieldCentric = false;
        prevDriverResetHeading = false;
        prevOperatorLongRange = false;
        prevOperatorShortRange = false;
        prevOperatorAutoRange = false;
        prevOperatorIntakeToggle = false;
    }

    /**
     * Update gamepad references (if gamepads change during runtime)
     */
    public void updateGamepads(Gamepad driver, Gamepad operator) {
        this.driverGamepad = driver;
        this.operatorGamepad = operator;
    }

    /**
     * Get driver gamepad reference
     */
    public Gamepad getDriverGamepad() {
        return driverGamepad;
    }

    /**
     * Get operator gamepad reference
     */
    public Gamepad getOperatorGamepad() {
        return operatorGamepad;
    }
}
