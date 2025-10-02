/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Verification test for motor direction fixes
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Quick test to verify motor direction fixes are working
 */
@Autonomous(name="Motor Fix Verification", group="Debug")
public class MotorFixVerification extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Apply the FIXED motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);    // FIXED
        frontRight.setDirection(DcMotor.Direction.REVERSE);   // FIXED
        backLeft.setDirection(DcMotor.Direction.FORWARD);     // FIXED
        backRight.setDirection(DcMotor.Direction.REVERSE);    // Was correct

        // Set modes and brake behavior
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("=== MOTOR FIX VERIFICATION ===", "");
        telemetry.addData("Applied Fixes", "Changed 3 motor directions");
        telemetry.addData("Test", "Forward movement should now be straight");
        telemetry.addData("Watch", "Robot should move forward without wiggling");
        telemetry.update();

        waitForStart();

        // Test forward movement with corrected motors
        telemetry.addData("Status", "Testing forward movement...");
        telemetry.addData("Expected", "Straight line, no wiggling");
        telemetry.update();

        // Forward movement - all motors same power
        frontLeft.setPower(0.3);
        frontRight.setPower(0.3);
        backLeft.setPower(0.3);
        backRight.setPower(0.3);

        sleep(2000);  // 4 seconds forward

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        sleep(1000);

        // Test strafe right
        telemetry.addData("Status", "Testing strafe right...");
        telemetry.addData("Expected", "Move sideways to the right");
        telemetry.update();

        // Strafe right: front-left and back-right positive, others negative
        frontLeft.setPower(0.3);
        frontRight.setPower(-0.3);
        backLeft.setPower(-0.3);
        backRight.setPower(0.3);

        sleep(2000);  // 3 seconds strafe

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addData("=== TEST COMPLETE ===", "");
        telemetry.addData("Results", "Did the robot move correctly?");
        telemetry.addData("Forward", "Should have been straight without wiggling");
        telemetry.addData("Strafe", "Should have moved sideways to the right");
        telemetry.addData("", "");
        telemetry.addData("If successful", "Your wiggling problem is FIXED!");
        telemetry.addData("If still wrong", "Run individual motor test again");
        telemetry.update();

        sleep(10000);
    }
}
