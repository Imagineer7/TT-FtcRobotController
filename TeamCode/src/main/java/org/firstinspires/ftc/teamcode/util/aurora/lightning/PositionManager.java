package org.firstinspires.ftc.teamcode.util.aurora.lightning;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.teamcode.util.aurora.lightning.OdoHelper;

import java.util.HashMap;
import java.util.Map;

public class PositionManager {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Position data
    private double currentX, currentY, currentHeading;
    private boolean isPositionValid;

    // Start position reference for navigation and distance calculations
    private double startX, startY, startHeading;
    private boolean hasStartPosition = false;

    // Field map for known positions
    private FieldMap fieldMap;
    private FieldMap.Alliance alliance;

    // Odometry system for tracking position
    private OdoHelper odoHelper;
    private boolean useOdometry = false;

    // Known AprilTag positions for vision positioning
    private Map<Integer, double[]> knownTagPositions;

    // Position staleness tracking
    private long lastUpdateTimeNanos = 0;

    /**
     * Constructor - Initialize the PositionManager
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry for logging
     * @param alliance Alliance color (RED or BLUE)
     */
    public PositionManager(HardwareMap hardwareMap, Telemetry telemetry, FieldMap.Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;

        // Initialize subsystems
        fieldMap = new FieldMap(alliance);

        // Initialize odometry helper
        try {
            odoHelper = new OdoHelper();
            odoHelper.DeadWheelOdometry(hardwareMap, telemetry);
            useOdometry = true;
            telemetry.addData("PositionManager", "Odometry initialized successfully");
        } catch (Exception e) {
            telemetry.addData("PositionManager", "Odometry init failed: " + e.getMessage());
            useOdometry = false;
        }

        // Initialize position
        currentX = 0;
        currentY = 0;
        currentHeading = 0;
        isPositionValid = false;

        // Load known tag positions
        loadKnownTagPositions();

        telemetry.addData("AuroraPositioning", "Initialized for " + alliance + " alliance");
    }

    private void loadKnownTagPositions() {
        knownTagPositions = new HashMap<>();

        // Null safety check
        if (fieldMap == null) {
            if (telemetry != null) {
                telemetry.addData("PositionManager", "FieldMap not initialized, skipping tag loading");
            }
            return;
        }

        // Load goal AprilTag positions from field map
        FieldMap.FieldPosition redGoal = fieldMap.getLocation("RED_GOAL");
        FieldMap.FieldPosition blueGoal = fieldMap.getLocation("BLUE_GOAL");

        if (redGoal != null) {
            knownTagPositions.put(24, new double[]{redGoal.x, redGoal.y}); // Red goal tag
        }

        if (blueGoal != null) {
            knownTagPositions.put(20, new double[]{blueGoal.x, blueGoal.y}); // Blue goal tag
        }

        // Add additional known tags based on your field setup
        telemetry.addData("Tag Positions", "Loaded %d AprilTag positions", knownTagPositions.size());
    }

    // ========== PUBLIC POSITION UPDATE METHODS ==========

    /**
     * Update the robot's current position and heading
     * Call this periodically from your main control loop
     *
     * @param x       Current X position in inches
     * @param y       Current Y position in inches
     * @param heading Current heading in degrees (will be normalized to 0-360)
     */
    public void updatePosition(double x, double y, double heading) {
        this.currentX = x;
        this.currentY = y;
        this.currentHeading = normalizeHeading(heading);
        this.isPositionValid = true;
        this.lastUpdateTimeNanos = System.nanoTime();
    }

    /**
     * Update position from odometry or vision system
     * Marks position as valid once updated
     *
     * @param x Current X position
     * @param y Current Y position
     */
    public void updatePosition(double x, double y) {
        updatePosition(x, y, this.currentHeading);
    }

    /**
     * Update position from OdoHelper (dead wheel odometry)
     * Call this in your main loop to pull odometry data
     *
     * @return true if odometry data was successfully read, false if not available
     */
    public boolean updatePositionFromOdometry() {
        if (!useOdometry || odoHelper == null) {
            return false;
        }

        try {
            odoHelper.updatePosition();
            this.currentX = odoHelper.getX();
            this.currentY = odoHelper.getY();
            this.currentHeading = normalizeHeading(odoHelper.getHeadingDegrees());
            this.isPositionValid = true;
            this.lastUpdateTimeNanos = System.nanoTime();
            return true;
        } catch (Exception e) {
            telemetry.addData("PositionManager ERROR", "Odometry update failed: " + e.getMessage());
            this.isPositionValid = false;  // Mark position as invalid on failure
            return false;
        }
    }

    /**
     * Get velocity information from odometry
     *
     * @return Array [vx, vy, omega] where vx/vy are in/s and omega is deg/s, or null if not available
     */
    public double[] getVelocityFromOdometry() {
        if (!useOdometry || odoHelper == null) {
            return null;
        }

        try {
            return odoHelper.getVelocity();
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Check if odometry system is available and ready
     *
     * @return true if odometry is initialized and ready
     */
    public boolean isOdometryReady() {
        if (!useOdometry || odoHelper == null) {
            return false;
        }

        try {
            return odoHelper.isReady();
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Get the status of the odometry device
     *
     * @return Status string (e.g., "READY", "NOT_INITIALIZED")
     */
    public String getOdometryStatus() {
        if (!useOdometry || odoHelper == null) {
            return "ODOMETRY_DISABLED";
        }

        try {
            return odoHelper.getDeviceStatus();
        } catch (Exception e) {
            return "ERROR";
        }
    }

    // ========== START POSITION MANAGEMENT METHODS ==========

    /**
     * Set the robot's starting position for reference tracking
     * Useful for calculating total distance traveled or returning to start
     *
     * @param x       Starting X position in inches
     * @param y       Starting Y position in inches
     * @param heading Starting heading in degrees (will be normalized to 0-360)
     */
    public void setStartPosition(double x, double y, double heading) {
        this.startX = x;
        this.startY = y;
        this.startHeading = normalizeHeading(heading);
        this.hasStartPosition = true;
        telemetry.addData("Start Position Set", "(%.1f, %.1f) @ %.1f°", x, y, this.startHeading);
    }

    /**
     * Set starting position using current position
     * Call this at the beginning of autonomous to mark the starting point
     */
    public void setStartPositionToCurrent() {
        setStartPosition(currentX, currentY, currentHeading);
    }

    /**
     * Get the robot's starting X position
     *
     * @return Starting X position, or 0 if not set
     */
    public double getStartX() {
        return startX;
    }

    /**
     * Get the robot's starting Y position
     *
     * @return Starting Y position, or 0 if not set
     */
    public double getStartY() {
        return startY;
    }

    /**
     * Get the robot's starting heading
     *
     * @return Starting heading in degrees, or 0 if not set
     */
    public double getStartHeading() {
        return startHeading;
    }

    /**
     * Get the starting position as an array
     *
     * @return [startX, startY, startHeading]
     */
    public double[] getStartPosition() {
        return new double[]{startX, startY, startHeading};
    }

    /**
     * Check if starting position has been set
     *
     * @return true if start position is defined
     */
    public boolean hasStartPosition() {
        return hasStartPosition;
    }

    /**
     * Reset starting position
     */
    public void clearStartPosition() {
        startX = 0;
        startY = 0;
        startHeading = 0;
        hasStartPosition = false;
    }

    // ========== POSITION COMPARISON AND DISTANCE METHODS ==========

    /**
     * Calculate distance traveled from starting position
     *
     * @return Total distance from start to current position, or -1 if start not set
     */
    public double getDistanceTraveledFromStart() {
        if (!hasStartPosition) return -1;
        return calculateDistance(startX, startY, currentX, currentY);
    }

    /**
     * Calculate heading change from starting position
     *
     * @return Difference in heading (current - start) in -180 to 180 range, or 0 if start not set
     */
    public double getHeadingChangeFromStart() {
        if (!hasStartPosition) return 0;
        return normalizeAngleDelta(currentHeading - startHeading);
    }

    /**
     * Calculate bearing from current position to starting position
     *
     * @return Bearing in degrees (0-360), or -1 if start not set
     */
    public double getBearingToStart() {
        if (!hasStartPosition) return -1;
        return calculateBearing(currentX, currentY, startX, startY);
    }

    /**
     * Calculate distance from current position to a target position
     *
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     * @return Distance in inches
     */
    public double calculateDistanceTo(double targetX, double targetY) {
        return calculateDistance(currentX, currentY, targetX, targetY);
    }

    /**
     * Calculate bearing (compass heading) from current position to target
     * 0° = North (positive Y), 90° = East (positive X), etc.
     *
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     * @return Bearing in degrees (0-360)
     */
    public double calculateBearingTo(double targetX, double targetY) {
        return calculateBearing(currentX, currentY, targetX, targetY);
    }

    /**
     * Calculate turn angle needed to face a target
     * Positive = turn left (counterclockwise), negative = turn right (clockwise)
     *
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     * @return Relative heading in degrees (-180 to 180)
     */
    public double calculateTurnAngleToTarget(double targetX, double targetY) {
        double bearingToTarget = calculateBearing(currentX, currentY, targetX, targetY);
        return normalizeAngleDelta(bearingToTarget - currentHeading);
    }

    // ========== PUBLIC POSITION GETTER METHODS ==========

    /**
     * Get the robot's current X position
     *
     * @return X position in inches
     */
    public double getCurrentX() {
        return currentX;
    }

    /**
     * Get the robot's current Y position
     *
     * @return Y position in inches
     */
    public double getCurrentY() {
        return currentY;
    }

    /**
     * Get the robot's current heading
     *
     * @return Heading in degrees (0-360)
     */
    public double getCurrentHeading() {
        return currentHeading;
    }

    /**
     * Get the robot's current position as an array
     *
     * @return [x, y, heading]
     */
    public double[] getCurrentPosition() {
        return new double[]{currentX, currentY, currentHeading};
    }

    /**
     * Check if the current position has been set/updated
     *
     * @return true if position data is valid
     */
    public boolean isPositionValid() {
        return isPositionValid;
    }

    // ========== PUBLIC ZONE AND LOCATION QUERY METHODS ==========

    /**
     * Get a descriptive string of which zones the robot is currently in
     *
     * @return Zone information string (e.g., "Long Range Launch Zone", "Red Parking Zone")
     */
    public String getCurrentZone() {
        if (fieldMap == null) return "FIELD_MAP_NOT_INITIALIZED";
        return fieldMap.getZoneInfo(currentX, currentY);
    }

    /**
     * Check if robot is in launch zone
     *
     * @return true if in any launch zone
     */
    public boolean isInLaunchZone() {
        if (fieldMap == null) return false;
        return fieldMap.isInLaunchZone(currentX, currentY);
    }

    /**
     * Check if robot is in parking zone
     *
     * @return true if in any parking zone
     */
    public boolean isInParkingZone() {
        if (fieldMap == null) return false;
        return fieldMap.isInParkingZone(currentX, currentY);
    }

    /**
     * Check if robot is in loading zone
     *
     * @return true if in any loading zone
     */
    public boolean isInLoadingZone() {
        if (fieldMap == null) return false;
        return fieldMap.isInLoadingZone(currentX, currentY);
    }

    /**
     * Check if robot is in alliance-specific parking zone
     *
     * @return true if in parking zone for current alliance
     */
    public boolean isInAllianceParkingZone() {
        if (fieldMap == null) return false;
        return fieldMap.isInAllianceParkingZone(currentX, currentY, alliance);
    }

    /**
     * Check if robot is in alliance-specific loading zone
     *
     * @return true if in loading zone for current alliance
     */
    public boolean isInAllianceLoadingZone() {
        if (fieldMap == null) return false;
        return fieldMap.isInAllianceLoadingZone(currentX, currentY, alliance);
    }

    /**
     * Check if robot is in alliance area
     *
     * @return true if in area for current alliance
     */
    public boolean isInAllianceArea() {
        if (fieldMap == null) return false;
        return fieldMap.isInAllianceArea(currentX, currentY, alliance);
    }

    // ========== PUBLIC CLOSEST POINT/GOAL METHODS ==========

    /**
     * Get the closest goal to the robot's current position
     *
     * @return FieldPosition of closest goal, or null if none found
     */
    public FieldMap.FieldPosition getClosestGoal() {
        if (fieldMap == null) return null;
        return fieldMap.getClosestGoal(currentX, currentY);
    }

    /**
     * Get the closest parking zone to the robot's current position
     *
     * @return FieldPosition of closest parking zone, or null if none found
     */
    public FieldMap.FieldPosition getClosestParkingZone() {
        if (fieldMap == null) return null;
        return fieldMap.getClosestParkingZone(currentX, currentY);
    }

    /**
     * Get the loading zone for the current alliance
     *
     * @return FieldPosition of alliance loading zone, or null if not defined
     */
    public FieldMap.FieldPosition getAllianceLoadingZone() {
        if (fieldMap == null) return null;
        return fieldMap.getLoadingZone();
    }

    /**
     * Get a named location from the field map
     *
     * @param name Location name (e.g., "RED_GOAL", "CENTER_FIELD")
     * @return FieldPosition with location data, or null if not found
     */
    public FieldMap.FieldPosition getLocation(String name) {
        if (fieldMap == null) return null;
        return fieldMap.getLocation(name);
    }

    /**
     * Get all locations of a specific type
     *
     * @param type ElementType to query (GOAL, PARKING_ZONE, etc.)
     * @return List of matching FieldPositions, or empty list if fieldMap not initialized
     */
    public java.util.List<FieldMap.FieldPosition> getLocationsByType(FieldMap.ElementType type) {
        if (fieldMap == null) return new java.util.ArrayList<>();
        return fieldMap.getLocationsByType(type);
    }

    /**
     * Get all locations for current alliance
     *
     * @return List of FieldPositions for current alliance, or empty list if fieldMap not initialized
     */
    public java.util.List<FieldMap.FieldPosition> getAllianceLocations() {
        if (fieldMap == null) return new java.util.ArrayList<>();
        return fieldMap.getLocationsByAlliance(alliance);
    }

    // ========== PUBLIC DISTANCE CALCULATION METHODS ==========

    /**
     * Calculate distance from robot's current position to a named location
     *
     * @param locationName Name of the location (e.g., "RED_GOAL")
     * @return Distance in inches, or -1 if location not found
     */
    public double getDistanceToLocation(String locationName) {
        FieldMap.FieldPosition target = fieldMap.getLocation(locationName);
        if (target == null) return -1;
        return calculateDistance(currentX, currentY, target.x, target.y);
    }

    /**
     * Calculate distance from robot's current position to a specific point
     *
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     * @return Distance in inches
     */
    public double getDistanceToPoint(double targetX, double targetY) {
        return calculateDistance(currentX, currentY, targetX, targetY);
    }

    /**
     * Calculate distance to the closest goal
     *
     * @return Distance in inches, or -1 if no goal found
     */
    public double getDistanceToClosestGoal() {
        FieldMap.FieldPosition goal = getClosestGoal();
        if (goal == null) return -1;
        return calculateDistance(currentX, currentY, goal.x, goal.y);
    }

    /**
     * Calculate distance to the closest parking zone
     *
     * @return Distance in inches, or -1 if no parking zone found
     */
    public double getDistanceToClosestParkingZone() {
        FieldMap.FieldPosition parkingZone = getClosestParkingZone();
        if (parkingZone == null) return -1;
        return calculateDistance(currentX, currentY, parkingZone.x, parkingZone.y);
    }

    /**
     * Calculate distance to alliance loading zone
     *
     * @return Distance in inches, or -1 if loading zone not found
     */
    public double getDistanceToLoadingZone() {
        FieldMap.FieldPosition loadingZone = getAllianceLoadingZone();
        if (loadingZone == null) return -1;
        return calculateDistance(currentX, currentY, loadingZone.x, loadingZone.y);
    }

    // ========== ODOMETRY CONTROL METHODS ==========

    /**
     * Reset the odometry system to origin (0, 0, 0)
     */
    public void resetOdometry() {
        if (useOdometry && odoHelper != null) {
            try {
                odoHelper.reset();
                currentX = 0;
                currentY = 0;
                currentHeading = 0;
                isPositionValid = true;
                lastUpdateTimeNanos = System.nanoTime();
                telemetry.addData("PositionManager", "Odometry reset to origin");
            } catch (Exception e) {
                telemetry.addData("PositionManager", "Failed to reset odometry: " + e.getMessage());
                isPositionValid = false;
            }
        }
    }

    /**
     * Set odometry position directly
     *
     * @param x       X position in inches
     * @param y       Y position in inches
     * @param heading Heading in degrees (will be normalized to 0-360)
     */
    public void setOdometryPosition(double x, double y, double heading) {
        if (useOdometry && odoHelper != null) {
            try {
                odoHelper.setPosition(x, y, heading);
                currentX = x;
                currentY = y;
                currentHeading = normalizeHeading(heading);
                isPositionValid = true;
                lastUpdateTimeNanos = System.nanoTime();
                telemetry.addData("PositionManager", "Odometry position set to (%.1f, %.1f) @ %.1f°", x, y, currentHeading);
            } catch (Exception e) {
                telemetry.addData("PositionManager", "Failed to set odometry position: " + e.getMessage());
                isPositionValid = false;
            }
        }
    }

    /**
     * Recalibrate the IMU (robot must be stationary)
     */
    public void recalibrateIMU() {
        if (useOdometry && odoHelper != null) {
            try {
                odoHelper.recalibrateIMU();
                telemetry.addData("PositionManager", "IMU recalibrated");
            } catch (Exception e) {
                telemetry.addData("PositionManager", "Failed to recalibrate IMU: " + e.getMessage());
            }
        }
    }

    /**
     * Get raw encoder counts from odometry
     *
     * @return Array [xEncoder, yEncoder] or null if not available
     */
    public int[] getEncoderCounts() {
        if (useOdometry && odoHelper != null) {
            try {
                return odoHelper.getEncoderCounts();
            } catch (Exception e) {
                return null;
            }
        }
        return null;
    }

    // ========== PRIVATE HELPER METHODS ==========

    /**
     * Calculate Euclidean distance between two points
     */
    private double calculateDistance(double x1, double y1, double x2, double y2) {
        double deltaX = x2 - x1;
        double deltaY = y2 - y1;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Calculate bearing (compass heading) from one point to another
     * 0° = North (positive Y), 90° = East (positive X)
     *
     * @param x1 Starting X coordinate
     * @param y1 Starting Y coordinate
     * @param x2 Target X coordinate
     * @param y2 Target Y coordinate
     * @return Bearing in degrees (0-360)
     */
    private double calculateBearing(double x1, double y1, double x2, double y2) {
        double deltaX = x2 - x1;
        double deltaY = y2 - y1;

        // Calculate bearing using atan2
        double bearing = Math.toDegrees(Math.atan2(deltaX, deltaY));

        // Normalize to 0-360
        if (bearing < 0) {
            bearing += 360;
        }

        return bearing;
    }

    /**
     * Add odometry telemetry data for debugging
     */
    public void addOdometryTelemetry() {
        if (useOdometry && odoHelper != null) {
            odoHelper.addTelemetry();
        }
    }

    /**
     * Normalize heading to 0-360 degree range
     * @param heading Heading in degrees (can be any value)
     * @return Normalized heading in 0-360 range
     */
    private double normalizeHeading(double heading) {
        heading = heading % 360;
        if (heading < 0) {
            heading += 360;
        }
        return heading;
    }

    /**
     * Normalize angle delta to -180 to 180 degree range
     * Used for calculating turn angles and heading changes
     * @param angle Angle difference in degrees
     * @return Normalized angle in -180 to 180 range
     */
    private double normalizeAngleDelta(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        }
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Check if position data is fresh (recently updated)
     * @param maxAgeSeconds Maximum acceptable age in seconds
     * @return true if position is valid and fresh, false if stale or invalid
     */
    public boolean isPositionFresh(double maxAgeSeconds) {
        if (!isPositionValid) {
            return false;
        }
        if (lastUpdateTimeNanos == 0) {
            return false;  // Never updated
        }
        double ageSeconds = (System.nanoTime() - lastUpdateTimeNanos) / 1e9;
        return ageSeconds < maxAgeSeconds;
    }

    /**
     * Get the age of the current position data in seconds
     * @return Age in seconds, or -1 if never updated
     */
    public double getPositionAgeSeconds() {
        if (lastUpdateTimeNanos == 0) {
            return -1;
        }
        return (System.nanoTime() - lastUpdateTimeNanos) / 1e9;
    }

    /**
     * Get the current alliance
     * @return Alliance color (RED or BLUE)
     */
    public FieldMap.Alliance getAlliance() {
        return alliance;
    }

    /**
     * Add position manager telemetry data
     */
    public void addPositionTelemetry() {
        telemetry.addData("=== POSITION MANAGER ===", "");
        telemetry.addData("Current Position", "(%.2f, %.2f) @ %.1f°", currentX, currentY, currentHeading);
        telemetry.addData("Position Valid", isPositionValid);

        if (hasStartPosition) {
            telemetry.addData("Start Position", "(%.2f, %.2f) @ %.1f°", startX, startY, startHeading);
            telemetry.addData("Distance from Start", "%.2f in", getDistanceTraveledFromStart());
            telemetry.addData("Heading Change", "%.1f°", getHeadingChangeFromStart());
        }

        telemetry.addData("Odometry Enabled", useOdometry);
        if (useOdometry) {
            telemetry.addData("Odometry Status", getOdometryStatus());
        }
    }
}
