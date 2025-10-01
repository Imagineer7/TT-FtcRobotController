/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Enhanced autonomous system with field map integration
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * MappedAutoHelper - Advanced autonomous system with field mapping
 *
 * Integrates with your existing AutoHelper and adds:
 * - Named location navigation
 * - Strategic path planning
 * - Dynamic obstacle avoidance
 * - Alliance-aware autonomous routines
 */
public class MappedAutoHelper {

    private AutoHelper autoHelper;
    private FieldMap fieldMap;
    private PathPlanningSystem pathPlanner;
    private LinearOpMode opMode;
    private Telemetry telemetry;

    // Navigation state
    private String currentTarget = "";
    private boolean isNavigating = false;
    private double positionTolerance = 3.0; // inches

    /**
     * Initialize mapped autonomous system
     */
    public MappedAutoHelper(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry, FieldMap.Alliance alliance) {
        this.opMode = opMode;
        this.telemetry = telemetry;

        // Initialize field map for current alliance
        fieldMap = new FieldMap(alliance);

        // Initialize path planning system
        pathPlanner = new PathPlanningSystem(fieldMap);

        // Initialize existing AutoHelper
        autoHelper = new AutoHelper(opMode, hardwareMap, telemetry);
    }

    /**
     * Navigate to a named location on the field
     */
    public MappedAutoHelper goToLocation(String locationName) {
        return goToLocation(locationName, "Navigate to " + locationName);
    }

    /**
     * Navigate to a named location with custom description
     */
    public MappedAutoHelper goToLocation(String locationName, String description) {
        currentTarget = locationName;
        isNavigating = true;

        telemetry.addData("Navigation", "Planning path to %s", locationName);
        telemetry.update();

        try {
            // Get current robot position (you'll need to implement this with your odometry)
            PathPlanningSystem.Point currentPos = getCurrentRobotPosition();
            pathPlanner.updateRobotPosition(currentPos.x, currentPos.y);

            // Plan path to location
            java.util.List<PathPlanningSystem.Point> path = pathPlanner.planPathToLocation(locationName, currentPos);

            if (path.isEmpty()) {
                telemetry.addLine("⚠️ No path found to " + locationName);
                telemetry.update();
                return this;
            }

            // Execute path using AutoHelper
            executePathWithAutoHelper(path, description);

        } catch (Exception e) {
            telemetry.addLine("❌ Navigation failed: " + e.getMessage());
            telemetry.update();
        }

        isNavigating = false;
        return this;
    }

    /**
     * Navigate to the closest goal
     */
    public MappedAutoHelper goToClosestGoal() {
        PathPlanningSystem.Point currentPos = getCurrentRobotPosition();
        FieldMap.FieldPosition closestGoal = fieldMap.getClosestGoal(currentPos.x, currentPos.y);

        if (closestGoal != null) {
            return goToLocation(closestGoal.name, "Navigate to closest goal");
        } else {
            telemetry.addLine("⚠️ No goals found for current alliance");
            telemetry.update();
            return this;
        }
    }

    /**
     * Navigate to loading zone
     */
    public MappedAutoHelper goToLoadingZone() {
        FieldMap.FieldPosition loadingZone = fieldMap.getLoadingZone();
        if (loadingZone != null) {
            return goToLocation(loadingZone.name, "Navigate to loading zone");
        } else {
            telemetry.addLine("⚠️ Loading zone not found");
            telemetry.update();
            return this;
        }
    }

    /**
     * Navigate to parking zone
     */
    public MappedAutoHelper goToParkingZone() {
        PathPlanningSystem.Point currentPos = getCurrentRobotPosition();
        FieldMap.FieldPosition parkingZone = fieldMap.getClosestParkingZone(currentPos.x, currentPos.y);

        if (parkingZone != null) {
            return goToLocation(parkingZone.name, "Navigate to parking zone");
        } else {
            telemetry.addLine("⚠️ Parking zone not found");
            telemetry.update();
            return this;
        }
    }

    /**
     * Execute emergency retreat to safe zone
     */
    public MappedAutoHelper emergencyRetreat() {
        telemetry.addLine("🚨 Emergency retreat initiated");
        telemetry.update();

        try {
            PathPlanningSystem.Point currentPos = getCurrentRobotPosition();
            pathPlanner.updateRobotPosition(currentPos.x, currentPos.y);

            java.util.List<PathPlanningSystem.Point> emergencyPath = pathPlanner.planEmergencyPath();
            executePathWithAutoHelper(emergencyPath, "Emergency retreat");

        } catch (Exception e) {
            telemetry.addLine("❌ Emergency retreat failed: " + e.getMessage());
            telemetry.update();
        }

        return this;
    }

    /**
     * Add dynamic obstacle (like another robot)
     */
    public MappedAutoHelper addObstacle(double x, double y, double radius) {
        pathPlanner.addDynamicObstacle(x, y, radius);
        telemetry.addData("Obstacle Added", "%.1f, %.1f (r=%.1f)", x, y, radius);
        return this;
    }

    /**
     * Clear dynamic obstacles
     */
    public MappedAutoHelper clearObstacles() {
        pathPlanner.clearDynamicObstacles();
        telemetry.addLine("Dynamic obstacles cleared");
        return this;
    }

    /**
     * Wait at current location
     */
    public MappedAutoHelper waitHere(double seconds, String reason) {
        autoHelper.waitFor((int)(seconds * 1000), reason);
        return this;
    }

    /**
     * Execute shooting sequence at current location
     */
    public MappedAutoHelper shootSequence(int shots, String description) {
        // This would integrate with your EnhancedDecodeHelper
        telemetry.addData("Shooting", "%s (%d shots)", description, shots);
        telemetry.update();

        // Placeholder for actual shooting implementation
        autoHelper.waitFor(shots * 1000, "Shooting " + shots + " shots");

        return this;
    }

    /**
     * Execute all planned movements
     */
    public void executeAll() {
        autoHelper.executeAll();
    }

    /**
     * Get current robot position from odometry system
     * You'll need to implement this based on your positioning system
     */
    private PathPlanningSystem.Point getCurrentRobotPosition() {
        // This is a placeholder - integrate with your actual positioning system
        // Example using AutoHelper's positioning (if available):

        try {
            // Try to get position from your positioning helper
            if (autoHelper != null) {
                // You'll need to add position getters to your AutoHelper
                // For now, return a default position
                return new PathPlanningSystem.Point(36, 60); // Default start position
            }
        } catch (Exception e) {
            telemetry.addLine("⚠️ Position unavailable, using default");
        }

        return new PathPlanningSystem.Point(36, 60); // Default fallback
    }

    /**
     * Execute a planned path using AutoHelper
     */
    private void executePathWithAutoHelper(java.util.List<PathPlanningSystem.Point> path, String description) {
        if (path.isEmpty()) return;

        telemetry.addData("Path Execution", "%s (%d waypoints)", description, path.size());
        telemetry.update();

        // Execute each waypoint in the path
        for (int i = 1; i < path.size(); i++) { // Skip first point (current position)
            PathPlanningSystem.Point waypoint = path.get(i);

            // Use AutoHelper to move to each waypoint
            autoHelper.moveTo(waypoint.x, waypoint.y, 0,
                String.format("Waypoint %d/%d", i, path.size() - 1));

            // Check if we need to stop (for safety)
            if (!opMode.opModeIsActive()) {
                break;
            }
        }

        // Final position adjustment at target
        if (!path.isEmpty()) {
            PathPlanningSystem.Point finalTarget = path.get(path.size() - 1);
            double targetHeading = pathPlanner.getTargetHeading();

            // Turn to correct heading at destination
            if (targetHeading != 0) {
                autoHelper.turnTo(targetHeading, "Orient at destination");
            }
        }
    }

    /**
     * Show field map telemetry
     */
    public void showFieldMapTelemetry() {
        telemetry.addLine("=== FIELD MAP STATUS ===");
        telemetry.addData("Alliance", fieldMap.getCurrentAlliance());
        telemetry.addData("Current Target", isNavigating ? currentTarget : "None");

        if (isNavigating) {
            double distanceToTarget = pathPlanner.getDistanceToTarget();
            telemetry.addData("Distance to Target", "%.1f inches", distanceToTarget);
            telemetry.addData("At Target", pathPlanner.isAtTarget(positionTolerance) ? "✅ YES" : "❌ NO");
        }

        // Show available locations by type
        telemetry.addLine("=== AVAILABLE LOCATIONS ===");
        java.util.List<FieldMap.FieldPosition> goals = fieldMap.getLocationsByType(FieldMap.ElementType.GOAL);
        telemetry.addData("Goals", "%d available", goals.size());

        java.util.List<FieldMap.FieldPosition> parkingZones = fieldMap.getLocationsByType(FieldMap.ElementType.PARKING_ZONE);
        telemetry.addData("Parking Zones", "%d available", parkingZones.size());

        java.util.List<FieldMap.FieldPosition> loadingZones = fieldMap.getLocationsByType(FieldMap.ElementType.LOADING_ZONE);
        telemetry.addData("Loading Zones", "%d available", loadingZones.size());
    }

    // Getters for advanced usage
    public FieldMap getFieldMap() { return fieldMap; }
    public PathPlanningSystem getPathPlanner() { return pathPlanner; }
    public AutoHelper getAutoHelper() { return autoHelper; }

    /**
     * Set position tolerance for "at target" checks
     */
    public MappedAutoHelper setPositionTolerance(double toleranceInches) {
        this.positionTolerance = toleranceInches;
        return this;
    }
}
