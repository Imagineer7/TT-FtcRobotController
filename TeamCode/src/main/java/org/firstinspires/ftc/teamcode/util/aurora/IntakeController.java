package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * IntakeController - Synchronized control of intake rollers and assist servos
 *
 * This class manages the front and back intake systems, ensuring that the bottom
 * intake assist servos run in the opposite direction of their corresponding main
 * roller motors for optimal artifact collection.
 *
 * Hardware Configuration:
 * - Front System: Top Intake Front motor + Bottom intake Front servo
 * - Back System: Top Intake Back motor + Bottom intake Back servo
 *
 * Usage:
 *   IntakeController intake = new IntakeController(hardware);
 *   intake.setFrontPower(0.8);  // Front roller at 80%, servo at -80%
 *   intake.setBackPower(0.8);   // Back roller at 80%, servo at -80%
 *   intake.stopAll();           // Stop all intake systems
 */
@Deprecated
public class IntakeController {

    // Hardware references
    private final DcMotor frontRollerMotor;
    private final DcMotor backRollerMotor;
    private final CRServo frontBottomServo;
    private final CRServo backBottomServo;

    // Current power settings
    private double frontPower = 0.0;
    private double backPower = 0.0;

    // Configuration
    private boolean invertBottomServos = true;  // Bottom servos run opposite of motors

    /**
     * Create a new IntakeController
     * @param hardware AuroraHardwareConfig instance with initialized hardware
     */
    public IntakeController(AuroraHardwareConfig hardware) {
        this.frontRollerMotor = hardware.getFrontRollerMotor();
        this.backRollerMotor = hardware.getBackRollerMotor();
        this.frontBottomServo = hardware.getFrontBottomIntakeServo();
        this.backBottomServo = hardware.getBackBottomIntakeServo();
    }

    /**
     * Set front intake power (motor + servo synchronized)
     * @param power Power level -1.0 to 1.0 (positive = intake, negative = eject)
     */
    public void setFrontPower(double power) {
        frontPower = power;
        frontRollerMotor.setPower(power);

        // Bottom servo runs opposite direction for proper artifact grip
        if (invertBottomServos) {
            frontBottomServo.setPower(-power);
        } else {
            frontBottomServo.setPower(power);
        }
    }

    /**
     * Set back intake power (motor + servo synchronized)
     * @param power Power level -1.0 to 1.0 (positive = intake, negative = eject)
     */
    public void setBackPower(double power) {
        backPower = power;
        backRollerMotor.setPower(power);

        // Bottom servo runs opposite direction for proper artifact grip
        if (invertBottomServos) {
            backBottomServo.setPower(-power);
        } else {
            backBottomServo.setPower(power);
        }
    }

    /**
     * Set both intake systems to the same power
     * @param power Power level -1.0 to 1.0
     */
    public void setBothPower(double power) {
        setFrontPower(power);
        setBackPower(power);
    }

    /**
     * Stop all intake systems (motors and servos)
     */
    public void stopAll() {
        setFrontPower(0.0);
        setBackPower(0.0);
    }

    /**
     * Stop front intake system
     */
    public void stopFront() {
        setFrontPower(0.0);
    }

    /**
     * Stop back intake system
     */
    public void stopBack() {
        setBackPower(0.0);
    }

    /**
     * Get current front intake power
     */
    public double getFrontPower() {
        return frontPower;
    }

    /**
     * Get current back intake power
     */
    public double getBackPower() {
        return backPower;
    }

    /**
     * Configure whether bottom servos run opposite to motors
     * @param invert true = servos run opposite (default), false = servos run same direction
     */
    public void setInvertBottomServos(boolean invert) {
        this.invertBottomServos = invert;
        // Re-apply current power to update servo directions
        setFrontPower(frontPower);
        setBackPower(backPower);
    }

    /**
     * Check if bottom servos are inverted
     */
    public boolean isBottomServosInverted() {
        return invertBottomServos;
    }

    /**
     * Intake artifacts (positive power on both systems)
     * @param power Power level 0.0 to 1.0
     */
    public void intake(double power) {
        setBothPower(Math.abs(power));
    }

    /**
     * Eject artifacts (negative power on both systems)
     * @param power Power level 0.0 to 1.0 (will be negated)
     */
    public void eject(double power) {
        setBothPower(-Math.abs(power));
    }
}

