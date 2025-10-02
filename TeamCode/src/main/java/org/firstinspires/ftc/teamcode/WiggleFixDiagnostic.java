/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Focused diagnostic for forward movement wiggling issue
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tests individual motor directions and basic movement to isolate wiggling
 */
@Autonomous(name="Wiggle Fix Diagnostic", group="Debug")
public class WiggleFixDiagnostic extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor modes
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set brake behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Current motor directions (as set in MechanumDrive.java)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("=== WIGGLE DIAGNOSTIC ===", "");
        telemetry.addData("Problem", "Robot starts straight then wiggles");
        telemetry.addData("Test Plan", "1. Individual motors → 2. Forward movement → 3. Analysis");
        telemetry.addData("Watch", "Which direction each motor actually moves robot");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // =================================================================
        // TEST 1: INDIVIDUAL MOTOR DIRECTIONS
        // =================================================================

        telemetry.addData("=== TEST 1: INDIVIDUAL MOTORS ===", "");
        telemetry.addData("Testing", "Each motor individually at 30% power");
        telemetry.addData("Watch", "Robot should move in consistent directions");
        telemetry.update();

        // Test Front Left motor
        telemetry.addData("Testing", "FRONT LEFT motor only");
        telemetry.addData("Expected", "Robot should move forward-left diagonal");
        telemetry.update();
        frontLeft.setPower(0.3);
        sleep(2000);
        frontLeft.setPower(0);
        sleep(1000);

        // Test Front Right motor
        telemetry.addData("Testing", "FRONT RIGHT motor only");
        telemetry.addData("Expected", "Robot should move forward-right diagonal");
        telemetry.update();
        frontRight.setPower(0.3);
        sleep(2000);
        frontRight.setPower(0);
        sleep(1000);

        // Test Back Left motor
        telemetry.addData("Testing", "BACK LEFT motor only");
        telemetry.addData("Expected", "Robot should move backward-right diagonal");
        telemetry.update();
        backLeft.setPower(0.3);
        sleep(2000);
        backLeft.setPower(0);
        sleep(1000);

        // Test Back Right motor
        telemetry.addData("Testing", "BACK RIGHT motor only");
        telemetry.addData("Expected", "Robot should move backward-left diagonal");
        telemetry.update();
        backRight.setPower(0.3);
        sleep(2000);
        backRight.setPower(0);
        sleep(1000);

        // =================================================================
        // TEST 2: MECANUM FORWARD MOVEMENT (LOW POWER)
        // =================================================================

        telemetry.addData("=== TEST 2: FORWARD MOVEMENT ===", "");
        telemetry.addData("Testing", "All motors for forward movement");
        telemetry.addData("Power", "Low 20% to reduce wiggling");
        telemetry.addData("Watch", "Does robot go straight or wiggle?");
        telemetry.update();

        // Forward movement: all motors same direction at low power
        frontLeft.setPower(0.2);
        frontRight.setPower(0.2);
        backLeft.setPower(0.2);
        backRight.setPower(0.2);

        sleep(3000);  // 3 seconds of forward movement

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        sleep(2000);

        // =================================================================
        // TEST 3: FORWARD MOVEMENT (HIGHER POWER)
        // =================================================================

        telemetry.addData("=== TEST 3: FORWARD (HIGHER POWER) ===", "");
        telemetry.addData("Testing", "Same movement at 40% power");
        telemetry.addData("Watch", "Does wiggling get worse with more power?");
        telemetry.update();

        // Forward movement at higher power
        frontLeft.setPower(0.4);
        frontRight.setPower(0.4);
        backLeft.setPower(0.4);
        backRight.setPower(0.4);

        sleep(3000);

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // =================================================================
        // ANALYSIS AND RECOMMENDATIONS
        // =================================================================

        telemetry.addData("=== DIAGNOSTIC COMPLETE ===", "");
        telemetry.addData("Analysis", "Check what you observed:");
        telemetry.addData("", "");
        telemetry.addData("If individual motors", "moved in wrong directions:");
        telemetry.addData("→ Solution", "Fix motor directions in MechanumDrive.java");
        telemetry.addData("", "");
        telemetry.addData("If forward was straight", "at low power but wiggled at high:");
        telemetry.addData("→ Solution", "Reduce power or improve positioning system");
        telemetry.addData("", "");
        telemetry.addData("If forward wiggled", "at both powers:");
        telemetry.addData("→ Solution", "Motor direction conflict or hardware issue");
        telemetry.addData("", "");
        telemetry.addData("Next Steps", "Use solutions below based on your observation");
        telemetry.update();

        sleep(15000);  // Keep results visible for 15 seconds
    }
}
