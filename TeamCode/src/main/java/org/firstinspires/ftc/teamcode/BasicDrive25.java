/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MechanumFieldRelative;
import org.firstinspires.ftc.teamcode.util.DecodeHelper;

@TeleOp(name="StarterBotV2.0", group="Linear OpMode")
public class BasicDrive25 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private boolean prevSlowToggle = false;
    private boolean prevModeToggle = false;
    private boolean fieldRelativeMode = false;
    private MechanumFieldRelative drive;
    
    // DECODE Helper for artifact shooting
    private DecodeHelper decodeHelper;
    
    // Runaway robot prevention
    private ElapsedTime lastInputTime = new ElapsedTime();
    private ElapsedTime connectionWatchdog = new ElapsedTime();
    private static final double INPUT_TIMEOUT = 3.0; // Stop robot if no input for 3 seconds
    private static final double CONNECTION_TIMEOUT = 1.0; // Check connection every 1 second
    private Gamepad prevGamepad1 = new Gamepad();
    private Gamepad prevGamepad2 = new Gamepad();
    private boolean emergencyStop = false;
    
    // Semi-auto actions
    private boolean semiAutoActive = false;
    private ElapsedTime semiAutoTimer = new ElapsedTime();
    private String currentSemiAuto = "";
    
    // Button state tracking for semi-auto
    private boolean prevY1 = false;
    private boolean prevB1 = false;
    private boolean prevA1 = false;
    private boolean prevRightBumper1 = false;
    private boolean prevLeftBumper1 = false;


    @Override
    public void runOpMode() {
        // Map motors with custom names if desired
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize DecodeHelper for shooting functionality
        decodeHelper = new DecodeHelper(hardwareMap, telemetry);

        // Pass mapped motors to MechanumFieldRelative
        drive = new MechanumFieldRelative(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, gamepad1, hardwareMap);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        lastInputTime.reset();
        connectionWatchdog.reset();

        while (opModeIsActive()) {
            // ========== RUNAWAY ROBOT PREVENTION ==========
            if (!checkGamepadSafety()) {
                continue; // Skip this loop iteration if safety check fails
            }
            // Toggle slow drive
            if (gamepad1.left_stick_button && !prevSlowToggle) {
                drive.toggleSlowDrive();
            }
            prevSlowToggle = gamepad1.left_stick_button;

            // Toggle field-relative/normal mode with X button
            if (gamepad1.x && !prevModeToggle) {
                fieldRelativeMode = !fieldRelativeMode;
            }
            prevModeToggle = gamepad1.x;

            // ========== SEMI-AUTO ACTIONS ==========
            handleSemiAutoActions();
            
            // ========== DRIVE CONTROLS ==========
            if (!semiAutoActive && !emergencyStop) {
                if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                    // Use dpad for movement
                    drive.dpadMove();
                } else {
                    // Get joystick values only when needed
                    double forward = -gamepad1.right_stick_y;
                    double strafe = gamepad1.right_stick_x;
                    double rotate = gamepad1.left_stick_x;
                    
                    // Apply deadzone to prevent drift (efficiency improvement)
                    if (Math.abs(forward) < 0.05) forward = 0;
                    if (Math.abs(strafe) < 0.05) strafe = 0;
                    if (Math.abs(rotate) < 0.05) rotate = 0;
                    
                    if (fieldRelativeMode) {
                        drive.driveFieldRelative(forward, strafe, rotate);
                    } else {
                        drive.drive(forward, strafe, rotate);
                    }
                }
            } else {
                // Stop all movement during emergency stop or semi-auto (simplified condition)
                drive.drive(0, 0, 0);
            }

            // ========== DECODE HELPER CONTROLS ==========
            if (!emergencyStop) {
                // Smart shooting with gamepad2.a (single shot on press, continuous when held)
                decodeHelper.handleShootButton(gamepad2.a, false);
                decodeHelper.handleShootButton(gamepad2.y, true);
                
                // Manual shooter control with right trigger (overrides smart shooting)
                if (gamepad2.right_trigger > 0.05) {
                    decodeHelper.setShooterPower(gamepad2.right_trigger * 0.9);
                }
                
                // Manual feed servo control with gamepad2.b (for testing/emergency)
                if (gamepad2.b) {
                    decodeHelper.setFeedPower(-1.0);
                } else if (!decodeHelper.isShooting()) {
                    // Only stop feed if DecodeHelper isn't currently shooting
                    decodeHelper.setFeedPower(0.0);
                }
                
                // Emergency stop all shooting with gamepad2.x
                if (gamepad2.x) {
                    decodeHelper.reset();
                }
            }
            // Note: DecodeHelper.reset() is called in checkGamepadSafety() when emergencyStop = true

            // ========== TELEMETRY ==========
            updateTelemetry();
            
            // Update previous gamepad states for next loop
            prevGamepad1.copy(gamepad1);
            prevGamepad2.copy(gamepad2);
        }
    }
    
    // ========== RUNAWAY ROBOT PREVENTION ==========
    private boolean checkGamepadSafety() {
        // Check if gamepads have changed (indicating active connection)
        // Use more reliable input detection instead of equals()
        boolean gamepad1Changed = hasSignificantInput(gamepad1) || hasButtonChanged(gamepad1, prevGamepad1);
        boolean gamepad2Changed = hasSignificantInput(gamepad2) || hasButtonChanged(gamepad2, prevGamepad2);
        
        if (gamepad1Changed || gamepad2Changed) {
            lastInputTime.reset();
            connectionWatchdog.reset();
            if (emergencyStop) {
                emergencyStop = false;
            }
        }
        
        // Check for input timeout (runaway prevention)
        if (lastInputTime.seconds() > INPUT_TIMEOUT) {
            emergencyStop = true;
            drive.drive(0, 0, 0); // Stop all movement
            decodeHelper.reset(); // Stop all shooting
            telemetry.addData("SAFETY", "EMERGENCY STOP - No input for %.1f seconds", lastInputTime.seconds());
            telemetry.addData("SAFETY", "Move any stick or press any button to resume");
            telemetry.update();
            return false;
        }
        
        // Check connection health
        if (connectionWatchdog.seconds() > CONNECTION_TIMEOUT) {
            connectionWatchdog.reset();
            // Additional connection health check could go here
        }
        
        // Emergency stop override - both start buttons pressed
        if (gamepad1.start && gamepad2.start) {
            emergencyStop = true;
            drive.drive(0, 0, 0);
            decodeHelper.reset();
            telemetry.addData("SAFETY", "MANUAL EMERGENCY STOP ACTIVATED");
            telemetry.update();
            return false;
        }
        
        return true;
    }
    
    // ========== SEMI-AUTO ACTIONS ==========
    private void handleSemiAutoActions() {
        // Update button states first to prevent conflicts
        boolean currentY1 = gamepad1.y;
        boolean currentB1 = gamepad1.b;
        boolean currentA1 = gamepad1.a;
        boolean currentRightBumper1 = gamepad1.right_bumper;
        boolean currentLeftBumper1 = gamepad1.left_bumper;
        
        // Cancel any semi-auto with back button
        if (gamepad1.back) {
            cancelSemiAuto();
        }
        
        // Update current semi-auto if one is active
        if (semiAutoActive) {
            updateCurrentSemiAuto();
        }
        
        // Don't start new semi-auto if one is active or in emergency stop
        if (!semiAutoActive && !emergencyStop) {
            // Semi-auto: Move to basket and shoot (Y button)
            if (currentY1 && !prevY1) {
                startSemiAuto("MOVE_TO_BASKET_AND_SHOOT");
            }
            
            // Semi-auto: Precision align to scoring position (B button)
            if (currentB1 && !prevB1) {
                startSemiAuto("PRECISION_ALIGN");
            }
            
            // Semi-auto: Quick retreat (A button)
            if (currentA1 && !prevA1) {
                startSemiAuto("QUICK_RETREAT");
            }
            
            // Semi-auto: Rotate to heading 0 (Right bumper)
            if (currentRightBumper1 && !prevRightBumper1) {
                startSemiAuto("ROTATE_TO_ZERO");
            }
            
            // Semi-auto: Strafe to wall align (Left bumper)
            if (currentLeftBumper1 && !prevLeftBumper1) {
                startSemiAuto("WALL_ALIGN");
            }
        }
        
        // Update button states for next loop
        prevY1 = currentY1;
        prevB1 = currentB1;
        prevA1 = currentA1;
        prevRightBumper1 = currentRightBumper1;
        prevLeftBumper1 = currentLeftBumper1;
    }
    
    private void startSemiAuto(String action) {
        semiAutoActive = true;
        currentSemiAuto = action;
        semiAutoTimer.reset();
        telemetry.addData("Semi-Auto", "Starting: " + action);
        telemetry.update();
    }
    
    private void cancelSemiAuto() {
        if (semiAutoActive) {
            semiAutoActive = false;
            currentSemiAuto = "";
            drive.drive(0, 0, 0); // Stop movement
            telemetry.addData("Semi-Auto", "CANCELLED");
            telemetry.update();
        }
    }
    
    private void updateCurrentSemiAuto() {
        if (!semiAutoActive) return;
        
        double elapsed = semiAutoTimer.seconds();
        boolean completed = false;
        
        switch (currentSemiAuto) {
            case "MOVE_TO_BASKET_AND_SHOOT":
                
                
            case "PRECISION_ALIGN":
                
                
            case "QUICK_RETREAT":
                
                
            case "ROTATE_TO_ZERO":
                
                
            case "WALL_ALIGN":
               
                
            default:
                completed = true;
                break;
        }
        
        if (completed) {
            semiAutoActive = false;
            currentSemiAuto = "";
            drive.drive(0, 0, 0);
            telemetry.addData("Semi-Auto", "COMPLETED");
            telemetry.update();
        }
    }
    
    // ========== TELEMETRY ==========
    private void updateTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        
        // Safety status
        if (emergencyStop) {
            telemetry.addData("⚠️ EMERGENCY STOP", "ACTIVE - Move controller to resume");
        } else {
            telemetry.addData("✅ Safety", "Normal - %.1fs since last input", lastInputTime.seconds());
        }
        
        // Semi-auto status
        if (semiAutoActive) {
            telemetry.addData("Quick-Auto", "%s (%.1fs)", currentSemiAuto, semiAutoTimer.seconds());
        }
        
        telemetry.addData("Drive Mode", fieldRelativeMode ? "Field Relative" : "Robot Centric");
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", drive.leftFrontPower, drive.rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", drive.leftBackPower, drive.rightBackPower);
        telemetry.addData("Slow Drive", drive.getSlowDrive() == 1 ? "Enabled" : "Disabled");

        
        telemetry.addData("", "--- DECODE SHOOTING ---");
        decodeHelper.updateTelemetry();
        /*
        telemetry.addData("", "--- DRIVE CONTROLS ---");
        telemetry.addData("Left Stick Button", "Toggle slow drive");
        telemetry.addData("X", "Toggle field relative");
        telemetry.addData("Start + Start", "Emergency stop");
        
        telemetry.addData("", "--- SEMI-AUTO CONTROLS ---");
        telemetry.addData("Y", "Move to basket & shoot");
        telemetry.addData("B", "Precision align mode");
        telemetry.addData("A", "Quick retreat");
        telemetry.addData("Right Bumper", "Rotate to 0°");
        telemetry.addData("Left Bumper", "Wall align");
        telemetry.addData("Back", "Cancel semi-auto");
        
        telemetry.addData("", "--- SHOOTING CONTROLS ---");
        telemetry.addData("Gamepad2 A", "Smart shooting");
        telemetry.addData("Gamepad2 RT", "Manual shooter");
        telemetry.addData("Gamepad2 B", "Manual feed");
        telemetry.addData("Gamepad2 X", "Stop shooting");*/
        
        telemetry.update();
    }
    
    // ========== HELPER METHODS FOR SAFETY ==========
    private boolean hasSignificantInput(Gamepad gamepad) {
        final double STICK_THRESHOLD = 0.1;
        final double TRIGGER_THRESHOLD = 0.1;
        
        return Math.abs(gamepad.left_stick_x) > STICK_THRESHOLD ||
               Math.abs(gamepad.left_stick_y) > STICK_THRESHOLD ||
               Math.abs(gamepad.right_stick_x) > STICK_THRESHOLD ||
               Math.abs(gamepad.right_stick_y) > STICK_THRESHOLD ||
               gamepad.left_trigger > TRIGGER_THRESHOLD ||
               gamepad.right_trigger > TRIGGER_THRESHOLD;
    }
    
    private boolean isNewPress(boolean current, boolean previous) {
        return current && !previous; // true only when the button was just pressed
    }

    private boolean hasButtonChanged(Gamepad current, Gamepad previous) {
        return isNewPress(current.left_bumper, previous.left_bumper) ||
            isNewPress(current.right_bumper, previous.right_bumper) ||
            isNewPress(current.left_stick_button, previous.left_stick_button) ||
            isNewPress(current.right_stick_button, previous.right_stick_button) ||
            isNewPress(current.dpad_up, previous.dpad_up) ||
            isNewPress(current.dpad_down, previous.dpad_down) ||
            isNewPress(current.dpad_left, previous.dpad_left) ||
            isNewPress(current.dpad_right, previous.dpad_right) ||
            isNewPress(current.start, previous.start) ||
            isNewPress(current.back, previous.back) ||
            isNewPress(current.x, previous.x) ||
            isNewPress(current.y, previous.y) ||
            isNewPress(current.a, previous.a) ||
            isNewPress(current.b, previous.b);
    }
}

