/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * PedroAutoBlue - Advanced autonomous using Pedro Pathing (Blue Alliance)
 * 
 * This OpMode demonstrates Pedro Pathing for smooth, reliable autonomous movement.
 * It's more advanced than StarterAutoSkeleton but provides better path following.
 */

package org.firstinspires.ftc.teamcode.auto.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Point;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.StarterShooter;
import org.firstinspires.ftc.teamcode.dashboard.PanelsDashboardHelper;

/**
 * PedroAutoBlue - Blue Alliance autonomous with Pedro Pathing
 * 
 * Pedro Pathing provides:
 * - Smooth bezier curve paths (no jerky movements)
 * - Automatic path following (no manual PID tuning)
 * - Real-time pose tracking with odometry
 * - Field visualization on Panels dashboard
 * 
 * Strategy:
 * 1. Start at blue alliance LAUNCH LINE
 * 2. Follow smooth path away from line
 * 3. Turn to face GOAL
 * 4. Shoot preloaded artifacts
 * 5. Move toward BASE
 * 
 * Prerequisites:
 * - GoBilda Pinpoint odometry computer installed and configured
 * - Motor names match PedroConstants
 * - Panels dashboard running (optional but recommended)
 * 
 * How to Customize:
 * 1. Adjust starting pose (based on your field position)
 * 2. Modify path waypoints (see buildPaths() method)
 * 3. Change shooter preset (SHORT_RANGE vs LONG_RANGE)
 * 4. Add more paths for cycling or complex strategies
 */
@Autonomous(name="Pedro Auto Blue (Advanced)", group="Pedro Autonomous")
@Configurable  // Allows configuration via Panels dashboard
public class PedroAutoBlue extends OpMode {
    
    // ═══════════════════════════════════════════════════════════════
    // SUBSYSTEMS
    // ═══════════════════════════════════════════════════════════════
    
    private Follower follower;               // Pedro path follower
    private StarterShooter shooter;          // Shooter subsystem
    private PanelsDashboardHelper dashboard; // Dashboard for telemetry and field view
    
    // ═══════════════════════════════════════════════════════════════
    // PATHS
    // ═══════════════════════════════════════════════════════════════
    
    private PathChain leaveLaunchLine;  // Path 1: Leave starting line
    private PathChain moveToBase;       // Path 2: Move toward base
    
    // ═══════════════════════════════════════════════════════════════
    // STARTING POSITION (Blue Alliance)
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Starting pose for blue alliance
     * 
     * Coordinate system:
     * - Origin (0, 0) at field center
     * - +X toward red wall, +Y toward blue audience
     * 
     * This pose assumes:
     * - Robot starts on blue LAUNCH LINE
     * - Facing toward field center (90° = toward audience side)
     * 
     * Adjust based on your actual starting position!
     */
    private static final Pose START_POSE = new Pose(56.5, 8.5, Math.toRadians(90));
    
    // ═══════════════════════════════════════════════════════════════
    // STATE MACHINE
    // ═══════════════════════════════════════════════════════════════
    
    private enum State {
        LEAVE_LAUNCH_LINE,   // Following path away from start
        TURN_TO_GOAL,        // Rotating to face goal
        SHOOT_PRELOADS,      // Spinning up and firing
        MOVE_TO_BASE,        // Following path toward base
        IDLE                 // Autonomous complete
    }
    
    private State currentState = State.LEAVE_LAUNCH_LINE;
    private double stateStartTime = 0;
    
    // ═══════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ═══════════════════════════════════════════════════════════════
    
    @Override
    public void init() {
        // Initialize dashboard
        dashboard = new PanelsDashboardHelper(telemetry);
        dashboard.logLine("╔════════════════════════════════════╗");
        dashboard.logLine("║  PEDRO AUTO BLUE (ADVANCED)        ║");
        dashboard.logLine("╚════════════════════════════════════╝");
        dashboard.logLine("");
        
        // Initialize Pedro Pathing
        try {
            follower = PedroConstants.createFollower(hardwareMap);
            follower.setStartingPose(START_POSE);
            dashboard.logLine("✅ Pedro Pathing initialized");
        } catch (Exception e) {
            follower = null;
            dashboard.logLine("❌ Pedro Pathing FAILED: " + e.getMessage());
            dashboard.logLine("   Check: odometry hardware 'odo'");
        }
        
        // Initialize shooter
        try {
            shooter = new StarterShooter(hardwareMap);
            dashboard.logLine("✅ Shooter initialized");
        } catch (Exception e) {
            shooter = null;
            dashboard.logLine("❌ Shooter FAILED: " + e.getMessage());
        }
        
        // Build paths
        if (follower != null) {
            buildPaths();
            dashboard.logLine("✅ Paths built");
        }
        
        dashboard.logLine("");
        dashboard.logLine("Strategy: Leave line → Turn → Shoot → Base");
        dashboard.logLine("");
        dashboard.logLine("✅ Ready! Press START");
        
        dashboard.update();
    }
    
    /**
     * Build all paths for autonomous
     * 
     * Path 1: Leave LAUNCH LINE
     * - Smooth forward movement
     * - Clears the starting line for bonus points
     * 
     * Path 2: Move to BASE
     * - Positions robot near BASE for endgame
     * - Optional: Can add more complex paths here
     */
    private void buildPaths() {
        // Path 1: Leave LAUNCH LINE (straight forward 30 inches)
        leaveLaunchLine = follower.pathBuilder()
                .addBezierLine(
                    new Point(START_POSE.getX(), START_POSE.getY(), Point.CARTESIAN),
                    new Point(START_POSE.getX(), START_POSE.getY() + 30, Point.CARTESIAN)
                )
                .setLinearHeadingInterpolation(START_POSE.getHeading(), START_POSE.getHeading())
                .build();
        
        // Path 2: Move toward BASE (example - adjust for your field position)
        Pose afterLeaving = new Pose(START_POSE.getX(), START_POSE.getY() + 30, START_POSE.getHeading());
        moveToBase = follower.pathBuilder()
                .addBezierLine(
                    new Point(afterLeaving.getX(), afterLeaving.getY(), Point.CARTESIAN),
                    new Point(afterLeaving.getX() - 20, afterLeaving.getY() + 20, Point.CARTESIAN)
                )
                .setLinearHeadingInterpolation(afterLeaving.getHeading(), Math.toRadians(45))
                .build();
    }
    
    // ═══════════════════════════════════════════════════════════════
    // START
    // ═══════════════════════════════════════════════════════════════
    
    @Override
    public void start() {
        if (follower != null) {
            // Start following first path
            follower.followPath(leaveLaunchLine);
            stateStartTime = getRuntime();
            dashboard.logLine("▶️  Autonomous started!");
            dashboard.update();
        }
    }
    
    // ═══════════════════════════════════════════════════════════════
    // MAIN LOOP
    // ═══════════════════════════════════════════════════════════════
    
    @Override
    public void loop() {
        if (follower == null) {
            dashboard.logLine("❌ Cannot run - initialization failed");
            dashboard.update();
            return;
        }
        
        // Update follower (REQUIRED - must be called before anything else!)
        follower.update();
        
        // Update shooter if available
        if (shooter != null) {
            shooter.update(gamepad1);  // gamepad not used in auto, but needed for RPM calc
        }
        
        // State machine
        switch (currentState) {
            case LEAVE_LAUNCH_LINE:
                if (!follower.isBusy()) {
                    // Path complete - transition to turn
                    currentState = State.TURN_TO_GOAL;
                    stateStartTime = getRuntime();
                }
                break;
                
            case TURN_TO_GOAL:
                // Turn to face GOAL (adjust angle for your field position)
                double targetHeading = Math.toRadians(114);  // Example: face toward goal
                follower.setMaxPower(0.5);  // Slower for precise turning
                follower.update();  // Update again for turn
                
                // Check if turn complete (within 5 degrees)
                double currentHeading = follower.getPose().getHeading();
                double headingError = Math.abs(targetHeading - currentHeading);
                
                if (headingError < Math.toRadians(5) || (getRuntime() - stateStartTime) > 2.0) {
                    // Turn complete or timeout - start shooting
                    currentState = State.SHOOT_PRELOADS;
                    stateStartTime = getRuntime();
                    
                    if (shooter != null) {
                        shooter.setPreset(StarterShooter.ShootingPreset.SHORT_RANGE);
                        shooter.spinUp();
                    }
                }
                break;
                
            case SHOOT_PRELOADS:
                double shootTime = getRuntime() - stateStartTime;
                
                // Wait for shooter to spin up
                if (shootTime > 1.5 && shooter != null && shooter.isAtSpeed()) {
                    // Fire first shot
                    if (shootTime < 2.0) {
                        shooter.fireOnce();
                    }
                    // Fire second shot
                    else if (shootTime > 2.5 && shootTime < 3.0) {
                        shooter.fireOnce();
                    }
                    // Shooting complete
                    else if (shootTime > 3.5) {
                        if (shooter != null) {
                            shooter.stop();
                        }
                        currentState = State.MOVE_TO_BASE;
                        stateStartTime = getRuntime();
                        follower.followPath(moveToBase);
                    }
                }
                // Timeout - move on anyway
                else if (shootTime > 5.0) {
                    if (shooter != null) {
                        shooter.stop();
                    }
                    currentState = State.MOVE_TO_BASE;
                    stateStartTime = getRuntime();
                    follower.followPath(moveToBase);
                }
                break;
                
            case MOVE_TO_BASE:
                if (!follower.isBusy()) {
                    // Path complete - autonomous done
                    currentState = State.IDLE;
                }
                break;
                
            case IDLE:
                // Do nothing - autonomous complete
                break;
        }
        
        // Update telemetry and field view
        updateTelemetry();
    }
    
    /**
     * Update telemetry and field visualization
     */
    private void updateTelemetry() {
        // Get current pose
        Pose currentPose = follower.getPose();
        
        // Clear previous drawings
        dashboard.clearFieldOverlays();
        
        // Draw robot on field
        dashboard.setRobotPose(currentPose);
        
        // Telemetry
        dashboard.log("State", currentState.toString());
        dashboard.log("X Position", "%.1f in", currentPose.getX());
        dashboard.log("Y Position", "%.1f in", currentPose.getY());
        dashboard.log("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        dashboard.logLine("");
        
        if (shooter != null) {
            dashboard.log("Shooter", shooter.getStateString());
            dashboard.log("RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        }
        
        dashboard.logLine("");
        dashboard.log("Runtime", "%.1f sec", getRuntime());
        
        dashboard.update();
    }
}
