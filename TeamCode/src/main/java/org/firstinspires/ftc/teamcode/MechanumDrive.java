package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * THREE OPTIONS TO FIX FORWARD/BACKWARD DIRECTION IF REVERSED:
 * 
 * OPTION 1 (RECOMMENDED): In AdvancedPositioningHelper.java, swap angles in moveCardinalRobot():
 *   - Change FORWARD from 0° to 180°
 *   - Change BACKWARD from 180° to 0°
 * 
 * OPTION 2: In this file (MechanumDrive.java), flip ALL motor directions:
 *   - Change all REVERSE to FORWARD and all FORWARD to REVERSE
 * 
 * OPTION 3: In AdvancedPositioningHelper.java goToPosition(), negate forward:
 *   - Change setMechanumPowers(forward, strafe, turnPower) to setMechanumPowers(-forward, strafe, turnPower)
 * 
 * See comments marked "TO FIX:" throughout the code for exact line changes.
 */
public class MechanumDrive {
    private static final double SLOW_DRIVE_SCALE = 1.0 / 3.0;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private Gamepad gamepad;
    private int slowDrive = 0;

    public double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
    private double axial, lateral, yaw;
    private double max;

    public MechanumDrive(DcMotor leftFrontDrive, DcMotor rightFrontDrive, DcMotor leftBackDrive, DcMotor rightBackDrive, Gamepad gamepad) {
        this.leftFrontDrive = leftFrontDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightBackDrive = rightBackDrive;
        this.gamepad = gamepad;

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // DIRECTION FIX OPTION 2: If forward/backward are reversed, flip ALL motor directions:
        // Change REVERSE to FORWARD and FORWARD to REVERSE on all motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);   // TO FIX: Change to FORWARD
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);    // TO FIX: Change to FORWARD
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);  // TO FIX: Change to REVERSE
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);   // TO FIX: Change to REVERSE
    }

    public void toggleSlowDrive() {
        slowDrive = (slowDrive == 0) ? 1 : 0;
    }

    public int getSlowDrive() {
        return slowDrive;
    }

    public void mechanum() {
        if ((gamepad.left_stick_y != 0) || (gamepad.right_stick_x != 0) || (gamepad.right_trigger != 0) || (gamepad.left_trigger != 0)) {
            axial = gamepad.left_stick_y;
            lateral = -(gamepad.right_trigger - gamepad.left_trigger);
            yaw = -gamepad.right_stick_x;
            if (slowDrive == 1) {
                axial *= SLOW_DRIVE_SCALE;
                lateral *= SLOW_DRIVE_SCALE;
                yaw *= SLOW_DRIVE_SCALE;
            }
        } else {
            dpadMove();
            yaw = 0;
        }
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
    }

    public void dpadMove() {
        if (gamepad.dpad_up) {
            axial = -0.2;
        } else if (gamepad.dpad_down) {
            axial = 0.2;
        } else {
            axial = 0;
        }
        if (gamepad.dpad_right) {
            lateral = -0.2;
        } else if (gamepad.dpad_left) {
            lateral = 0.2;
        } else {
            lateral = 0;
        }
    }

    public void setMotorPowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
