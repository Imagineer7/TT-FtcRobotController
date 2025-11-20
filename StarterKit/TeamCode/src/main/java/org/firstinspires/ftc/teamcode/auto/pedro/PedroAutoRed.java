/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * PedroAutoRed - Advanced autonomous using Pedro Pathing (Red Alliance)
 * 
 * This OpMode is the red alliance version of PedroAutoBlue.
 * See PedroAutoBlue.java for detailed comments and explanation.
 */

package org.firstinspires.ftc.teamcode.auto.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Point;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.StarterShooter;
import org.firstinspires.ftc.teamcode.dashboard.PanelsDashboardHelper;

@Autonomous(name="Pedro Auto Red (Advanced)", group="Pedro Autonomous")
@Configurable
public class PedroAutoRed extends OpMode {
    
    private Follower follower;
    private StarterShooter shooter;
    private PanelsDashboardHelper dashboard;
    
    private PathChain leaveLaunchLine;
    private PathChain moveToBase;
    
    /**
     * Starting pose for red alliance
     * Mirror of blue alliance pose across X-axis
     */
    private static final Pose START_POSE = new Pose(-56.5, 8.5, Math.toRadians(90));
    
    private enum State {
        LEAVE_LAUNCH_LINE,
        TURN_TO_GOAL,
        SHOOT_PRELOADS,
        MOVE_TO_BASE,
        IDLE
    }
    
    private State currentState = State.LEAVE_LAUNCH_LINE;
    private double stateStartTime = 0;
    
    @Override
    public void init() {
        dashboard = new PanelsDashboardHelper(telemetry);
        dashboard.logLine("╔════════════════════════════════════╗");
        dashboard.logLine("║  PEDRO AUTO RED (ADVANCED)         ║");
        dashboard.logLine("╚════════════════════════════════════╝");
        dashboard.logLine("");
        
        try {
            follower = PedroConstants.createFollower(hardwareMap);
            follower.setStartingPose(START_POSE);
            dashboard.logLine("✅ Pedro Pathing initialized");
        } catch (Exception e) {
            follower = null;
            dashboard.logLine("❌ Pedro Pathing FAILED: " + e.getMessage());
        }
        
        try {
            shooter = new StarterShooter(hardwareMap);
            dashboard.logLine("✅ Shooter initialized");
        } catch (Exception e) {
            shooter = null;
            dashboard.logLine("❌ Shooter FAILED: " + e.getMessage());
        }
        
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
    
    private void buildPaths() {
        // Path 1: Leave LAUNCH LINE
        leaveLaunchLine = follower.pathBuilder()
                .addBezierLine(
                    new Point(START_POSE.getX(), START_POSE.getY(), Point.CARTESIAN),
                    new Point(START_POSE.getX(), START_POSE.getY() + 30, Point.CARTESIAN)
                )
                .setLinearHeadingInterpolation(START_POSE.getHeading(), START_POSE.getHeading())
                .build();
        
        // Path 2: Move toward BASE
        Pose afterLeaving = new Pose(START_POSE.getX(), START_POSE.getY() + 30, START_POSE.getHeading());
        moveToBase = follower.pathBuilder()
                .addBezierLine(
                    new Point(afterLeaving.getX(), afterLeaving.getY(), Point.CARTESIAN),
                    new Point(afterLeaving.getX() + 20, afterLeaving.getY() + 20, Point.CARTESIAN)
                )
                .setLinearHeadingInterpolation(afterLeaving.getHeading(), Math.toRadians(135))
                .build();
    }
    
    @Override
    public void start() {
        if (follower != null) {
            follower.followPath(leaveLaunchLine);
            stateStartTime = getRuntime();
            dashboard.logLine("▶️  Autonomous started!");
            dashboard.update();
        }
    }
    
    @Override
    public void loop() {
        if (follower == null) {
            dashboard.logLine("❌ Cannot run - initialization failed");
            dashboard.update();
            return;
        }
        
        follower.update();
        
        if (shooter != null) {
            shooter.update(gamepad1);
        }
        
        switch (currentState) {
            case LEAVE_LAUNCH_LINE:
                if (!follower.isBusy()) {
                    currentState = State.TURN_TO_GOAL;
                    stateStartTime = getRuntime();
                }
                break;
                
            case TURN_TO_GOAL:
                double targetHeading = Math.toRadians(66);  // Mirror of blue angle
                follower.setMaxPower(0.5);
                follower.update();
                
                double currentHeading = follower.getPose().getHeading();
                double headingError = Math.abs(targetHeading - currentHeading);
                
                if (headingError < Math.toRadians(5) || (getRuntime() - stateStartTime) > 2.0) {
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
                
                if (shootTime > 1.5 && shooter != null && shooter.isAtSpeed()) {
                    if (shootTime < 2.0) {
                        shooter.fireOnce();
                    }
                    else if (shootTime > 2.5 && shootTime < 3.0) {
                        shooter.fireOnce();
                    }
                    else if (shootTime > 3.5) {
                        if (shooter != null) {
                            shooter.stop();
                        }
                        currentState = State.MOVE_TO_BASE;
                        stateStartTime = getRuntime();
                        follower.followPath(moveToBase);
                    }
                }
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
                    currentState = State.IDLE;
                }
                break;
                
            case IDLE:
                break;
        }
        
        updateTelemetry();
    }
    
    private void updateTelemetry() {
        Pose currentPose = follower.getPose();
        
        dashboard.clearFieldOverlays();
        dashboard.setRobotPose(currentPose);
        
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
