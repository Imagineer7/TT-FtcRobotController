package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumFieldRelative {
    private IMU imu;
    private static final double SLOW_DRIVE_SCALE = 1.0 / 3.0;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private Gamepad gamepad;
    private int slowDrive = 0;
    public double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
    private double axial, lateral, yaw;
    private double max;

    public MechanumFieldRelative(DcMotor leftFrontDrive, DcMotor rightFrontDrive, DcMotor leftBackDrive, DcMotor rightBackDrive, Gamepad gamepad, HardwareMap hardwareMap) {
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

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(revOrientation));
    }

    public void toggleSlowDrive() {
        slowDrive = (slowDrive == 0) ? 1 : 0;
    }

    public int getSlowDrive() {
        return slowDrive;
    }

    public void drive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double maxPower = Math.max(1.0,
            Math.max(Math.abs(leftFrontPower),
            Math.max(Math.abs(rightFrontPower),
            Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        double maxSpeed = 1.0; // Default to full speed; you can set this dynamically

        leftFrontDrive.setPower(maxSpeed * (leftFrontPower / maxPower));
        rightFrontDrive.setPower(maxSpeed * (rightFrontPower / maxPower));
        leftBackDrive.setPower(maxSpeed * (leftBackPower / maxPower));
        rightBackDrive.setPower(maxSpeed * (rightBackPower / maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        theta = AngleUnit.normalizeRadians(theta - robotYaw);
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);
        drive(newForward, newStrafe, rotate);
    }

    public void dpadMove() {
        if (gamepad.dpad_up) {
            axial = 0.2;
        } else if (gamepad.dpad_down) {
            axial = -0.2;
        } else {
            axial = 0;
        }
        if (gamepad.dpad_right) {
            lateral = 0.2;
        } else if (gamepad.dpad_left) {
            lateral = -0.2;
        } else {
            lateral = 0;
        }
        drive(axial, lateral, 0);
    }
}
