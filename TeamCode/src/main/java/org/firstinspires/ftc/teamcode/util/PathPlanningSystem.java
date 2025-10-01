/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Advanced autonomous path planning with obstacle avoidance
 */

package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.List;

/**
 * PathPlanningSystem - Intelligent navigation with obstacle avoidance and field map integration
 *
 * Features:
 * - Dynamic path recalculation
 * - Obstacle detection and avoidance
 * - Optimal route planning
 * - Field map integration for named locations
 * - Traffic management for multiple robots
 */
public class PathPlanningSystem {

    public static class Point {
        public double x, y;
        public Point(double x, double y) { this.x = x; this.y = y; }
    }

    public static class Obstacle {
        public Point center;
        public double radius;
        public boolean isMoving;

        public Obstacle(Point center, double radius, boolean isMoving) {
            this.center = center;
            this.radius = radius;
            this.isMoving = isMoving;
        }
    }

    private List<Obstacle> knownObstacles = new ArrayList<>();
    private List<Point> currentPath = new ArrayList<>();
    private Point robotPosition = new Point(0, 0);
    private Point targetPosition = new Point(0, 0);
    private FieldMap fieldMap;
    private List<Obstacle> dynamicObstacles = new ArrayList<>();

    /**
     * Constructor with field map integration
     */
    public PathPlanningSystem(FieldMap fieldMap) {
        this.fieldMap = fieldMap;
        if (fieldMap != null) {
            // Add static obstacles from field map
            for (Obstacle staticObs : fieldMap.getStaticObstacles()) {
                knownObstacles.add(staticObs);
            }
        }
    }

    /**
     * Default constructor for backwards compatibility
     */
    public PathPlanningSystem() {
        this(null);
    }

    /**
     * Plan optimal path from current position to target
     */
    public List<Point> planPath(Point start, Point end, List<Obstacle> obstacles) {
        List<Point> path = new ArrayList<>();

        // Simple A* implementation for demonstration
        // In practice, you'd use a more sophisticated algorithm

        if (isPathClear(start, end, obstacles)) {
            // Direct path is clear
            path.add(start);
            path.add(end);
        } else {
            // Need to navigate around obstacles
            path = findAlternatePath(start, end, obstacles);
        }

        return optimizePath(path);
    }

    /**
     * Plan path to a named location on the field
     */
    public List<Point> planPathToLocation(String locationName) {
        if (fieldMap == null) {
            throw new IllegalStateException("Field map not initialized");
        }

        FieldMap.FieldPosition target = fieldMap.getLocation(locationName);
        if (target == null) {
            throw new IllegalArgumentException("Location not found: " + locationName);
        }

        return planPathToLocation(locationName, robotPosition);
    }

    /**
     * Plan path from current position to named location
     */
    public List<Point> planPathToLocation(String locationName, Point startPos) {
        if (fieldMap == null) {
            throw new IllegalStateException("Field map not initialized");
        }

        FieldMap.FieldPosition target = fieldMap.getLocation(locationName);
        if (target == null) {
            throw new IllegalArgumentException("Location not found: " + locationName);
        }

        // Get strategic waypoints for this target
        List<FieldMap.FieldPosition> waypoints = fieldMap.getStrategicWaypoints(locationName);

        List<Point> fullPath = new ArrayList<>();
        Point currentStart = startPos;

        // Plan path through strategic waypoints
        for (FieldMap.FieldPosition waypoint : waypoints) {
            List<Point> segment = planPath(currentStart, waypoint.toPoint(), getAllObstacles());

            // Add segment (skip first point to avoid duplicates)
            for (int i = (fullPath.isEmpty() ? 0 : 1); i < segment.size(); i++) {
                fullPath.add(segment.get(i));
            }

            currentStart = waypoint.toPoint();
        }

        // Plan final segment to target
        List<Point> finalSegment = planPath(currentStart, target.toPoint(), getAllObstacles());
        for (int i = (fullPath.isEmpty() ? 0 : 1); i < finalSegment.size(); i++) {
            fullPath.add(finalSegment.get(i));
        }

        currentPath = fullPath;
        targetPosition = target.toPoint();

        return fullPath;
    }

    /**
     * Check if direct path is clear of obstacles
     */
    private boolean isPathClear(Point start, Point end, List<Obstacle> obstacles) {
        for (Obstacle obs : obstacles) {
            if (lineIntersectsCircle(start, end, obs.center, obs.radius + 6)) { // 6" safety margin
                return false;
            }
        }
        return true;
    }

    /**
     * Find alternate path around obstacles
     */
    private List<Point> findAlternatePath(Point start, Point end, List<Obstacle> obstacles) {
        List<Point> path = new ArrayList<>();
        path.add(start);

        // Generate waypoints around obstacles
        for (Obstacle obs : obstacles) {
            if (lineIntersectsCircle(start, end, obs.center, obs.radius + 6)) {
                // Generate bypass points
                Point[] bypasses = generateBypassPoints(obs, start, end);

                // Choose best bypass point
                Point bestBypass = chooseBestBypass(bypasses, start, end, obstacles);
                if (bestBypass != null) {
                    path.add(bestBypass);
                }
            }
        }

        path.add(end);
        return path;
    }

    /**
     * Generate bypass points around an obstacle
     */
    private Point[] generateBypassPoints(Obstacle obs, Point start, Point end) {
        double safeRadius = obs.radius + 8; // 8" safety margin
        double angle1 = Math.atan2(end.y - obs.center.y, end.x - obs.center.x) + Math.PI/2;
        double angle2 = angle1 + Math.PI;

        Point bypass1 = new Point(
            obs.center.x + safeRadius * Math.cos(angle1),
            obs.center.y + safeRadius * Math.sin(angle1)
        );

        Point bypass2 = new Point(
            obs.center.x + safeRadius * Math.cos(angle2),
            obs.center.y + safeRadius * Math.sin(angle2)
        );

        return new Point[]{bypass1, bypass2};
    }

    /**
     * Choose the best bypass point based on total distance
     */
    private Point chooseBestBypass(Point[] bypasses, Point start, Point end, List<Obstacle> obstacles) {
        double bestDistance = Double.MAX_VALUE;
        Point bestBypass = null;

        for (Point bypass : bypasses) {
            double totalDistance = distance(start, bypass) + distance(bypass, end);

            // Check if this bypass creates new collisions
            if (isPathClear(start, bypass, obstacles) && isPathClear(bypass, end, obstacles)) {
                if (totalDistance < bestDistance) {
                    bestDistance = totalDistance;
                    bestBypass = bypass;
                }
            }
        }

        return bestBypass;
    }

    /**
     * Optimize path by removing unnecessary waypoints
     */
    private List<Point> optimizePath(List<Point> path) {
        if (path.size() <= 2) return path;

        List<Point> optimized = new ArrayList<>();
        optimized.add(path.get(0));

        for (int i = 1; i < path.size() - 1; i++) {
            Point prev = optimized.get(optimized.size() - 1);
            Point next = path.get(i + 1);

            // If we can go directly from prev to next, skip current point
            if (!isPathClear(prev, next, knownObstacles)) {
                optimized.add(path.get(i));
            }
        }

        optimized.add(path.get(path.size() - 1));
        return optimized;
    }

    // Utility methods
    private double distance(Point a, Point b) {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    private boolean lineIntersectsCircle(Point start, Point end, Point center, double radius) {
        // Distance from point to line segment algorithm
        double A = end.x - start.x;
        double B = end.y - start.y;
        double C = center.x - start.x;
        double D = center.y - start.y;

        double dot = C * A + D * B;
        double lenSq = A * A + B * B;

        if (lenSq == 0) return distance(start, center) <= radius;

        double param = dot / lenSq;

        Point closest;
        if (param < 0) {
            closest = start;
        } else if (param > 1) {
            closest = end;
        } else {
            closest = new Point(start.x + param * A, start.y + param * B);
        }

        return distance(center, closest) <= radius;
    }

    /**
     * Add obstacle to known obstacles list
     */
    public void addObstacle(double x, double y, double radius, boolean isMoving) {
        knownObstacles.add(new Obstacle(new Point(x, y), radius, isMoving));
    }

    /**
     * Update robot position for dynamic replanning
     */
    public void updateRobotPosition(double x, double y) {
        robotPosition = new Point(x, y);
    }

    /**
     * Get next waypoint in current path
     */
    public Point getNextWaypoint() {
        if (currentPath.isEmpty()) return null;

        // Find closest point on path ahead of robot
        double minDistance = Double.MAX_VALUE;
        int closestIndex = 0;

        for (int i = 0; i < currentPath.size(); i++) {
            double dist = distance(robotPosition, currentPath.get(i));
            if (dist < minDistance) {
                minDistance = dist;
                closestIndex = i;
            }
        }

        // Return next point after closest
        if (closestIndex < currentPath.size() - 1) {
            return currentPath.get(closestIndex + 1);
        }

        return currentPath.get(currentPath.size() - 1); // Return final target
    }

    /**
     * Get combined list of all obstacles (static + dynamic)
     */
    private List<Obstacle> getAllObstacles() {
        List<Obstacle> allObstacles = new ArrayList<>(knownObstacles);
        allObstacles.addAll(dynamicObstacles);
        return allObstacles;
    }

    /**
     * Add temporary dynamic obstacle (like another robot)
     */
    public void addDynamicObstacle(double x, double y, double radius) {
        dynamicObstacles.add(new Obstacle(new Point(x, y), radius, true));
    }

    /**
     * Clear all dynamic obstacles
     */
    public void clearDynamicObstacles() {
        dynamicObstacles.clear();
    }

    /**
     * Plan emergency path to safe zone
     */
    public List<Point> planEmergencyPath() {
        if (fieldMap == null) {
            // Fallback: move away from center field
            Point safePoint = new Point(
                robotPosition.x < FieldMap.FIELD_WIDTH / 2 ? 24 : FieldMap.FIELD_WIDTH - 24,
                robotPosition.y < FieldMap.FIELD_HEIGHT / 2 ? 24 : FieldMap.FIELD_HEIGHT - 24
            );
            return planPath(robotPosition, safePoint, getAllObstacles());
        }

        FieldMap.FieldPosition safeZone = fieldMap.getSafeZone();
        return planPath(robotPosition, safeZone.toPoint(), getAllObstacles());
    }

    /**
     * Check if robot is at target location
     */
    public boolean isAtTarget(double toleranceInches) {
        return distance(robotPosition, targetPosition) <= toleranceInches;
    }

    /**
     * Get distance to target
     */
    public double getDistanceToTarget() {
        return distance(robotPosition, targetPosition);
    }

    /**
     * Get recommended approach heading for current target
     */
    public double getTargetHeading() {
        if (fieldMap == null) return 0;

        // Find the field position that matches our target
        for (FieldMap.FieldPosition pos : fieldMap.getAllLocations().values()) {
            if (Math.abs(pos.x - targetPosition.x) < 1 && Math.abs(pos.y - targetPosition.y) < 1) {
                return pos.heading;
            }
        }

        return 0; // Default heading
    }

    /**
     * Update path if obstacles have changed or robot has deviated
     */
    public boolean updatePath() {
        if (currentPath.isEmpty() || targetPosition == null) return false;

        // Check if current path is still valid
        Point nextWaypoint = getNextWaypoint();
        if (nextWaypoint != null && !isPathClear(robotPosition, nextWaypoint, getAllObstacles())) {
            // Replan path from current position
            currentPath = planPath(robotPosition, targetPosition, getAllObstacles());
            return true;
        }

        return false;
    }
}
