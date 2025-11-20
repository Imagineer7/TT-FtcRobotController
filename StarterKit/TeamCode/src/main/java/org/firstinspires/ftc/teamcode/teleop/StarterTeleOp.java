/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * StarterTeleOp - Basic driver-controlled OpMode for DECODE robots
 * 
 * This OpMode provides simple gamepad controls for driving and shooting.
 * It's designed to be easy to understand and modify for beginners.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.StarterRobotManager;
import org.firstinspires.ftc.teamcode.core.StarterDrive;
import org.firstinspires.ftc.teamcode.core.StarterShooter;

/**
 * StarterTeleOp - Driver-controlled OpMode
 * 
 * This OpMode runs during the DRIVER-CONTROLLED period (2 minutes).
 * Drivers use gamepads to control the robot and score ARTIFACTS.
 * 
 * Controls (Gamepad 1 - Single Driver Mode):
 * ───────────────────────────────────────────
 * Left Stick Y:      Forward/Backward
 * Left Stick X:      Strafe Left/Right (mecanum only)
 * Right Stick X:     Rotate
 * 
 * Y Button:          Spin up/stop shooter
 * A Button:          Fire artifact (when shooter ready)
 * X Button:          Change shooter preset
 * 
 * How It Works:
 * ─────────────
 * 1. Initialize robot subsystems (drive + shooter)
 * 2. Wait for driver to press START
 * 3. Main loop:
 *    - Read gamepad inputs
 *    - Update robot subsystems
 *    - Display telemetry
 * 4. Stop all systems when match ends
 * 
 * Tips for Beginners:
 * ───────────────────
 * - Test drive controls first before enabling shooter
 * - Start with lower shooter preset (SAFE_TEST) for practice
 * - Use telemetry to monitor robot state
 * - If something doesn't work, check hardware config names
 */
@TeleOp(name="Starter TeleOp", group="Competition")
public class StarterTeleOp extends LinearOpMode {
    
    // Robot manager - coordinates all subsystems
    private StarterRobotManager robot;
    
    // Timer for match time display
    private ElapsedTime runtime = new ElapsedTime();
    
    // Button state tracking
    private boolean prevPresetButton = false;
    
    @Override
    public void runOpMode() {
        // ═══════════════════════════════════════
        // INITIALIZATION PHASE
        // ═══════════════════════════════════════
        
        telemetry.addLine("╔════════════════════════════════════╗");
        telemetry.addLine("║   FTC DECODE STARTER KIT - TELEOP   ║");
        telemetry.addLine("╚════════════════════════════════════╝");
        telemetry.addLine();
        
        // Initialize robot systems
        telemetry.addLine("Initializing robot systems...");
        telemetry.update();
        
        robot = new StarterRobotManager(hardwareMap, telemetry);
        
        // Display control scheme
        telemetry.addLine();
        telemetry.addLine("━━━━━━━━ CONTROLS ━━━━━━━━");
        telemetry.addLine("Left Stick:  Drive");
        telemetry.addLine("Right Stick: Rotate");
        telemetry.addLine("Y Button:    Spin Up/Stop Shooter");
        telemetry.addLine("A Button:    Fire Artifact");
        telemetry.addLine("X Button:    Change Shooter Mode");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine();
        telemetry.addLine("✅ Ready! Press START to begin.");
        telemetry.update();
        
        // ═══════════════════════════════════════
        // WAIT FOR START
        // ═══════════════════════════════════════
        waitForStart();
        runtime.reset();
        
        // ═══════════════════════════════════════
        // MAIN CONTROL LOOP
        // ═══════════════════════════════════════
        while (opModeIsActive()) {
            
            // Update robot subsystems (drive + shooter)
            robot.update(gamepad1, gamepad2);
            
            // Handle preset switching (X button)
            if (gamepad1.x && !prevPresetButton && robot.isShooterInitialized()) {
                StarterShooter shooter = robot.getShooter();
                if (shooter != null) {
                    // Toggle between presets
                    if (shooter.getCurrentPreset() == StarterShooter.ShootingPreset.SHORT_RANGE) {
                        shooter.setPreset(StarterShooter.ShootingPreset.SAFE_TEST);
                    } else {
                        shooter.setPreset(StarterShooter.ShootingPreset.SHORT_RANGE);
                    }
                }
            }
            prevPresetButton = gamepad1.x;
            
            // ═══════════════════════════════════════
            // TELEMETRY DISPLAY
            // ═══════════════════════════════════════
            
            telemetry.addLine("╔════════════════════════════════════╗");
            telemetry.addLine("║   FTC DECODE - DRIVER CONTROL      ║");
            telemetry.addLine("╚════════════════════════════════════╝");
            telemetry.addLine();
            
            // Match time
            telemetry.addData("⏱️ Match Time", "%.1f sec", runtime.seconds());
            telemetry.addLine();
            
            // Drive system status
            if (robot.isDriveInitialized()) {
                StarterDrive drive = robot.getDrive();
                telemetry.addLine("━━━━ DRIVE SYSTEM ━━━━");
                telemetry.addData("Drive Type", drive.getDriveType());
                telemetry.addData("Max Power", "%.0f%%", drive.getMaxPower() * 100);
                telemetry.addLine();
            } else {
                telemetry.addLine("❌ Drive System: OFFLINE");
                telemetry.addLine();
            }
            
            // Shooter system status
            if (robot.isShooterInitialized()) {
                StarterShooter shooter = robot.getShooter();
                telemetry.addLine("━━━━ SHOOTER SYSTEM ━━━━");
                telemetry.addData("Status", shooter.getStateString());
                telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
                telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
                telemetry.addData("Preset", shooter.getCurrentPreset());
                
                // Ready indicator
                if (shooter.isRunning()) {
                    if (shooter.isAtSpeed()) {
                        telemetry.addLine("✅ READY TO FIRE!");
                    } else {
                        telemetry.addLine("⏳ Spinning up...");
                    }
                }
                telemetry.addLine();
            } else {
                telemetry.addLine("❌ Shooter System: OFFLINE");
                telemetry.addLine();
            }
            
            // Control reminders
            telemetry.addLine("━━━━ QUICK CONTROLS ━━━━");
            telemetry.addLine("Y = Spin Up    A = Fire");
            telemetry.addLine("X = Change Mode");
            
            // System health
            telemetry.addLine();
            if (robot.isHealthy()) {
                telemetry.addLine("✅ All systems operational");
            } else {
                telemetry.addLine("⚠️  Some systems offline - check init screen");
            }
            
            telemetry.update();
        }
        
        // ═══════════════════════════════════════
        // CLEANUP PHASE
        // ═══════════════════════════════════════
        robot.stop();
        telemetry.addLine("Match ended. Robot stopped.");
        telemetry.update();
    }
}
