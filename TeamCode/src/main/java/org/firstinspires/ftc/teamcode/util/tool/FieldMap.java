/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Field map system for DECODE season autonomous navigation
 */

package org.firstinspires.ftc.teamcode.util.tool;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * FieldMap - Comprehensive field mapping system for autonomous navigation
 *
 * Features:
 * - Predefined field element locations
 * - Multiple coordinate systems (field-centric, robot-centric)
 * - Alliance-specific locations
 * - Dynamic obstacle tracking
 * - Named waypoints and zones
 */
public class FieldMap {

    // Field dimensions (DECODE 2024-2025 season - adjust as needed)
    public static final double FIELD_WIDTH = 144.0;  // 12 feet in inches
    public static final double FIELD_HEIGHT = 144.0; // 12 feet in inches

    // Alliance colors
    public enum Alliance {
        RED, BLUE
    }

    // Field element types
    public enum ElementType {
        GOAL, PARKING_ZONE, LOADING_ZONE, OBSTACLE, WALL, WAYPOINT, SAFE_ZONE
    }

    // Field coordinate system origin (bottom-left corner of field)
    public static class FieldPosition {
        public double x, y, heading; // x, y in inches, heading in degrees
        public String name;
        public ElementType type;
        public Alliance alliance;

        public FieldPosition(double x, double y, double heading, String name, ElementType type, Alliance alliance) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.name = name;
            this.type = type;
            this.alliance = alliance;
        }

        public FieldPosition(double x, double y, String name, ElementType type) {
            this(x, y, 0, name, type, null);
        }

        public PathPlanningSystem.Point toPoint() {
            return new PathPlanningSystem.Point(x, y);
        }
    }

    // Predefined field elements
    private Map<String, FieldPosition> namedLocations = new HashMap<>();
    private List<PathPlanningSystem.Obstacle> staticObstacles = new ArrayList<>();
    private Alliance currentAlliance;

    /**
     * Initialize field map with DECODE season elements
     */
    public FieldMap(Alliance alliance) {
        this.currentAlliance = alliance;
        initializeFieldElements();
    }

    /**
     * Initialize all field elements for DECODE season
     */
    private void initializeFieldElements() {
        // Clear existing data
        namedLocations.clear();
        staticObstacles.clear();
         /* - Goal AprilTags: Both on back wall (opposite audience, -X side), 45° angles toward center
                *   • Red Goal (ID 24): (-58.3727, 55.6425, 29.5) heading 315° (back-right corner)
                *   • Blue Goal (ID 20): (-58.3727, -55.6425, 29.5) heading 45° (back-left corner)
          */

        // === GOALS AND SCORING AREAS ===
        addLocation("RED_GOAL", -58, 55, 315, ElementType.GOAL, Alliance.RED);
        addLocation("BLUE_GOAL", -58, -55, 45, ElementType.GOAL, Alliance.BLUE);

        // === PARKING ZONES ===
        addLocation("RED_PARKING_ZONE", 38, -32, 0, ElementType.PARKING_ZONE, Alliance.RED);
        addLocation("BLUE_PARKING_ZONE", 38, 32, 0, ElementType.PARKING_ZONE, Alliance.BLUE);

        // === LOADING ZONES (Human Player Stations) ===
        addLocation("RED_LOADING_ZONE", 61, -58, 0, ElementType.LOADING_ZONE, Alliance.RED);
        addLocation("BLUE_LOADING_ZONE", 61, 58, 0, ElementType.LOADING_ZONE, Alliance.BLUE);

        // === ARTIFACT LOCATIONS (adjust based on actual field layout) ===
        addLocation("SPIKE_MARK_1", -11, -47, 0, ElementType.WAYPOINT, null);
        addLocation("SPIKE_MARK_2", 11, -47, 0, ElementType.WAYPOINT, null);
        addLocation("SPIKE_MARK_3", 35, -47, 0, ElementType.WAYPOINT, null);
        addLocation("SPIKE_MARK_4", -11, 47, 0, ElementType.WAYPOINT, null);
        addLocation("SPIKE_MARK_5", 11, 47, 0, ElementType.WAYPOINT, null);
        addLocation("SPIKE_MARK_6", 35, 47, 0, ElementType.WAYPOINT, null);

        // === STARTING POSITIONS ===
        addLocation("RED_START_GOAL", -55, 47, 315, ElementType.WAYPOINT, Alliance.RED);
        addLocation("RED_START_FAR", 58, -11, 180, ElementType.WAYPOINT, Alliance.RED);
        addLocation("BLUE_START_GOAL", -55, -47, 45, ElementType.WAYPOINT, Alliance.BLUE);
        addLocation("BLUE_START_FAR", 58, 11, 180, ElementType.WAYPOINT, Alliance.BLUE);

        // === WAYPOINTS FOR NAVIGATION ===
        addLocation("CENTER_FIELD", 0, 0, 0, ElementType.WAYPOINT, null);

        // === SAFE ZONES (areas to retreat to if needed) ===

        // === STATIC OBSTACLES (walls, permanent field elements) ===
    }

    /**
     * Add a named location to the field map
     */
    public void addLocation(String name, double x, double y, double heading, ElementType type, Alliance alliance) {
        namedLocations.put(name, new FieldPosition(x, y, heading, name, type, alliance));
    }

    /**
     * Add a rectangular obstacle to the field map
     */
    public void addObstacle(String name, double centerX, double centerY, double width, double height, boolean isMoving) {
        // Convert rectangle to circle for simplified collision detection
        double radius = Math.sqrt(width * width + height * height) / 2.0;
        PathPlanningSystem.Obstacle obs = new PathPlanningSystem.Obstacle(
            new PathPlanningSystem.Point(centerX, centerY), radius, isMoving);
        staticObstacles.add(obs);
    }

    /**
     * Get a named location
     */
    public FieldPosition getLocation(String name) {
        return namedLocations.get(name);
    }

    /**
     * Get all locations of a specific type
     */
    public List<FieldPosition> getLocationsByType(ElementType type) {
        List<FieldPosition> result = new ArrayList<>();
        for (FieldPosition pos : namedLocations.values()) {
            if (pos.type == type) {
                result.add(pos);
            }
        }
        return result;
    }

    /**
     * Get all locations for a specific alliance
     */
    public List<FieldPosition> getLocationsByAlliance(Alliance alliance) {
        List<FieldPosition> result = new ArrayList<>();
        for (FieldPosition pos : namedLocations.values()) {
            if (pos.alliance == alliance || pos.alliance == null) {
                result.add(pos);
            }
        }
        return result;
    }

    /**
     * Get the closest goal to a position
     */
    public FieldPosition getClosestGoal(double x, double y) {
        List<FieldPosition> goals = getLocationsByType(ElementType.GOAL);
        if (currentAlliance != null) {
            goals = filterByAlliance(goals, currentAlliance);
        }

        return findClosestLocation(x, y, goals);
    }

    /**
     * Get the closest parking zone
     */
    public FieldPosition getClosestParkingZone(double x, double y) {
        List<FieldPosition> parkingZones = getLocationsByType(ElementType.PARKING_ZONE);
        if (currentAlliance != null) {
            parkingZones = filterByAlliance(parkingZones, currentAlliance);
        }

        return findClosestLocation(x, y, parkingZones);
    }

    /**
     * Get loading zone for current alliance
     */
    public FieldPosition getLoadingZone() {
        String loadingZoneName = currentAlliance == Alliance.RED ? "RED_LOADING_ZONE" : "BLUE_LOADING_ZONE";
        return getLocation(loadingZoneName);
    }

    /**
     * Get starting position for current alliance and position
     */
    public FieldPosition getStartPosition(boolean leftSide) {
        String startName;
        if (currentAlliance == Alliance.RED) {
            startName = leftSide ? "RED_START_LEFT" : "RED_START_RIGHT";
        } else {
            startName = leftSide ? "BLUE_START_LEFT" : "BLUE_START_RIGHT";
        }
        return getLocation(startName);
    }

    /**
     * Get all static obstacles for path planning
     */
    public List<PathPlanningSystem.Obstacle> getStaticObstacles() {
        return new ArrayList<>(staticObstacles);
    }

    /**
     * Check if a position is in bounds
     */
    public boolean isInBounds(double x, double y) {
        return x >= 0 && x <= FIELD_WIDTH && y >= 0 && y <= FIELD_HEIGHT;
    }

    /**
     * Convert field coordinates to robot coordinates (if needed)
     */
    public PathPlanningSystem.Point fieldToRobot(double fieldX, double fieldY, double robotX, double robotY, double robotHeading) {
        // Transform field coordinates to robot-relative coordinates
        double deltaX = fieldX - robotX;
        double deltaY = fieldY - robotY;

        double cos = Math.cos(-Math.toRadians(robotHeading));
        double sin = Math.sin(-Math.toRadians(robotHeading));

        double robotRelativeX = deltaX * cos - deltaY * sin;
        double robotRelativeY = deltaX * sin + deltaY * cos;

        return new PathPlanningSystem.Point(robotRelativeX, robotRelativeY);
    }

    /**
     * Get strategic waypoints for navigating to a target
     */
    public List<FieldPosition> getStrategicWaypoints(String targetName) {
        FieldPosition target = getLocation(targetName);
        if (target == null) return new ArrayList<>();

        List<FieldPosition> waypoints = new ArrayList<>();

        // Add alliance-specific strategic waypoints based on target
        if (target.type == ElementType.GOAL) {
            // Add approach waypoints for goals
            if (currentAlliance == Alliance.RED) {
                waypoints.add(getLocation("RED_SIDE_MID"));
            } else {
                waypoints.add(getLocation("BLUE_SIDE_MID"));
            }
        } else if (target.type == ElementType.LOADING_ZONE) {
            // Add center field waypoint for loading zone approach
            waypoints.add(getLocation("CENTER_FIELD"));
        }

        return waypoints;
    }

    /**
     * Get safe retreat position
     */
    public FieldPosition getSafeZone() {
        String safeZoneName = currentAlliance == Alliance.RED ? "RED_SAFE_ZONE" : "BLUE_SAFE_ZONE";
        return getLocation(safeZoneName);
    }

    // Helper methods
    private List<FieldPosition> filterByAlliance(List<FieldPosition> positions, Alliance alliance) {
        List<FieldPosition> filtered = new ArrayList<>();
        for (FieldPosition pos : positions) {
            if (pos.alliance == alliance || pos.alliance == null) {
                filtered.add(pos);
            }
        }
        return filtered;
    }

    private FieldPosition findClosestLocation(double x, double y, List<FieldPosition> locations) {
        if (locations.isEmpty()) return null;

        FieldPosition closest = locations.get(0);
        double minDistance = distance(x, y, closest.x, closest.y);

        for (FieldPosition pos : locations) {
            double dist = distance(x, y, pos.x, pos.y);
            if (dist < minDistance) {
                minDistance = dist;
                closest = pos;
            }
        }

        return closest;
    }

    private double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    /**
     * Get all named locations (for debugging/telemetry)
     */
    public Map<String, FieldPosition> getAllLocations() {
        return new HashMap<>(namedLocations);
    }

    /**
     * Set current alliance (updates which locations are prioritized)
     */
    public void setAlliance(Alliance alliance) {
        this.currentAlliance = alliance;
    }

    public Alliance getCurrentAlliance() {
        return currentAlliance;
    }
}
