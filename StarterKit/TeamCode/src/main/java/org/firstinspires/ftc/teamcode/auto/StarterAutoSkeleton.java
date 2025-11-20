/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * StarterAutoSkeleton - Template for autonomous OpModes
 * 
 * This is a basic autonomous template that demonstrates the minimum scoring
 * strategy for DECODE: leave LAUNCH LINE and shoot preloaded ARTIFACTS.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.StarterDrive;
import org.firstinspires.ftc.teamcode.core.StarterShooter;

/**
 * StarterAutoSkeleton - Basic autonomous template
 * 
 * DECODE Autonomous Phase Goals:
 * ─────────────────────────────────
 * 1. LEAVE LAUNCH LINE (3 points) - Robot must move completely off starting line
 * 2. SCORE ARTIFACTS (6 points each) - Launch preloaded artifacts into GOAL
 * 3. PARK IN BASE (3 points) - Optional endgame positioning
 * 
 * This Template Does:
 * ───────────────────
 * 1. Drive forward to leave LAUNCH LINE
 * 2. Stop and spin up shooter
 * 3. Fire preloaded ARTIFACTS (you start with 2)
 * 4. Drive back toward BASE (optional)
 * 
 * Movement Strategy:
 * ──────────────────
 * This example uses TIME-BASED movement (simple but less accurate).
 * 
 * Advanced teams can upgrade to:
 * - ENCODER-BASED: Use motor encoders for precise distance
 * - ODOMETRY: Use external encoders for accurate positioning
 * - PEDRO PATHING: Use path-following library for smooth curves
 * 
 * How to Customize:
 * ─────────────────
 * 1. Adjust drive times/powers for your field position
 * 2. Add more movements for complex strategies
 * 3. Change shooter preset based on distance to GOAL
 * 4. Add sensor-based decision making (color, distance)
 * 
 * Safety Notes:
 * ─────────────
 * - Always test on a practice field first!
 * - Start with slow speeds and short movements
 * - Use telemetry to monitor progress
 * - Have emergency stop ready (press STOP on Driver Station)
 */
@Autonomous(name="Starter Auto Skeleton", group="Competition")
public class StarterAutoSkeleton extends LinearOpMode {
    
    // Subsystems
    private StarterDrive drive;
    private StarterShooter shooter;
    
    // Timer for delays
    private ElapsedTime timer = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        // ═══════════════════════════════════════
        // INITIALIZATION
        // ═══════════════════════════════════════
        
        telemetry.addLine("╔════════════════════════════════════╗");
        telemetry.addLine("║  FTC DECODE - AUTONOMOUS SKELETON  ║");
        telemetry.addLine("╚════════════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("Initializing subsystems...");
        telemetry.update();
        
        // Initialize drive system
        try {
            drive = new StarterDrive(hardwareMap, StarterDrive.DriveType.MECANUM);
            telemetry.addLine("✅ Drive initialized");
        } catch (Exception e) {
            drive = null;
            telemetry.addLine("❌ Drive failed: " + e.getMessage());
        }
        
        // Initialize shooter system
        try {
            shooter = new StarterShooter(hardwareMap);
            telemetry.addLine("✅ Shooter initialized");
        } catch (Exception e) {
            shooter = null;
            telemetry.addLine("❌ Shooter failed: " + e.getMessage());
        }
        
        telemetry.addLine();
        telemetry.addLine("Strategy: Leave LAUNCH LINE → Shoot → Park");
        telemetry.addLine();
        telemetry.addLine("✅ Ready! Press START when positioned.");
        telemetry.update();
        
        // ═══════════════════════════════════════
        // WAIT FOR START
        // ═══════════════════════════════════════
        waitForStart();
        
        if (!opModeIsActive()) {
            return;  // OpMode was stopped before start
        }
        
        // ═══════════════════════════════════════
        // AUTONOMOUS SEQUENCE
        // ═══════════════════════════════════════
        
        // STEP 1: Leave LAUNCH LINE (required for 3 points)
        telemetry.addLine("STEP 1: Leaving LAUNCH LINE...");
        telemetry.update();
        leaveStartLine();
        
        // STEP 2: Position for shooting
        telemetry.addLine("STEP 2: Positioning for shot...");
        telemetry.update();
        positionForShot();
        
        // STEP 3: Spin up shooter and fire preloaded artifacts
        telemetry.addLine("STEP 3: Shooting preloaded ARTIFACTS...");
        telemetry.update();
        shootPreloadedArtifacts();
        
        // STEP 4: Optional - drive toward BASE for parking
        telemetry.addLine("STEP 4: Driving toward BASE...");
        telemetry.update();
        driveTowardBase();
        
        // ═══════════════════════════════════════
        // SEQUENCE COMPLETE
        // ═══════════════════════════════════════
        
        telemetry.addLine();
        telemetry.addLine("✅ Autonomous sequence complete!");
        telemetry.addLine("Estimated Points: 15");
        telemetry.addLine("  - Leave LAUNCH LINE: 3 pts");
        telemetry.addLine("  - 2 ARTIFACTS: 12 pts");
        telemetry.addLine();
        telemetry.update();
        
        // Stop all systems
        if (drive != null) drive.stop();
        if (shooter != null) shooter.stop();
    }
    
    /**
     * STEP 1: Leave the LAUNCH LINE
     * 
     * Drive forward for ~1 second to completely clear the starting line.
     * Adjust time/power based on your robot's speed and field position.
     */
    private void leaveStartLine() {
        if (drive == null || !opModeIsActive()) return;
        
        // Drive forward at 50% power for 1 second
        drive.drive(0.5, 0, 0);  // axial, lateral, yaw
        sleep(1000);  // 1 second
        drive.stop();
        
        telemetry.addLine("✅ Cleared LAUNCH LINE");
        telemetry.update();
    }
    
    /**
     * STEP 2: Position for shooting
     * 
     * Make small adjustments to line up with GOAL.
     * This example just stops, but you can add:
     * - Strafing to align
     * - Rotating to face GOAL
     * - Using vision to find AprilTag
     */
    private void positionForShot() {
        if (drive == null || !opModeIsActive()) return;
        
        // For this basic example, we just pause briefly
        // Advanced teams: Add vision-based alignment here
        sleep(500);
        
        telemetry.addLine("✅ Positioned for shot");
        telemetry.update();
    }
    
    /**
     * STEP 3: Shoot preloaded ARTIFACTS
     * 
     * Spin up the flywheel and fire both preloaded artifacts.
     * You start each match with 2 artifacts already loaded.
     */
    private void shootPreloadedArtifacts() {
        if (shooter == null || !opModeIsActive()) return;
        
        // Set shooter to SHORT_RANGE preset
        shooter.setPreset(StarterShooter.ShootingPreset.SHORT_RANGE);
        
        // Spin up flywheel
        shooter.spinUp();
        telemetry.addLine("⏳ Spinning up flywheel...");
        telemetry.update();
        
        // Wait for shooter to reach target speed
        timer.reset();
        while (opModeIsActive() && !shooter.isAtSpeed() && timer.seconds() < 3.0) {
            // Update shooter (calculates RPM)
            shooter.update(gamepad1);  // gamepad not used in auto, but needed for update
            
            telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
            telemetry.update();
            
            sleep(50);  // Short delay
        }
        
        if (!shooter.isAtSpeed()) {
            telemetry.addLine("⚠️  Shooter not ready, firing anyway...");
            telemetry.update();
        }
        
        // Fire first artifact
        telemetry.addLine("🎯 Firing artifact 1...");
        telemetry.update();
        shooter.fireOnce();
        sleep(500);  // Wait for feed to complete
        
        // Fire second artifact
        telemetry.addLine("🎯 Firing artifact 2...");
        telemetry.update();
        shooter.fireOnce();
        sleep(500);  // Wait for feed to complete
        
        // Stop shooter
        shooter.stop();
        
        telemetry.addLine("✅ Fired 2 ARTIFACTS");
        telemetry.update();
    }
    
    /**
     * STEP 4: Drive toward BASE (optional)
     * 
     * Drive back toward BASE area for endgame positioning.
     * This is optional but can be useful for DRIVER-CONTROLLED phase.
     */
    private void driveTowardBase() {
        if (drive == null || !opModeIsActive()) return;
        
        // Drive backward at 40% power for 0.8 seconds
        drive.drive(-0.4, 0, 0);  // negative = backward
        sleep(800);
        drive.stop();
        
        telemetry.addLine("✅ Moved toward BASE");
        telemetry.update();
    }
    
    /**
     * Helper: Sleep with opModeIsActive check
     * 
     * This is safer than regular sleep() because it stops immediately
     * if the OpMode is stopped by the driver.
     */
    private void sleep(long milliseconds) {
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            idle();  // Yield to other threads
        }
    }
}
