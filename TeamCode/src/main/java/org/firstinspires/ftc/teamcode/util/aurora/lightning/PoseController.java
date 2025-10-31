package org.firstinspires.ftc.teamcode.util.aurora.lightning;

import org.firstinspires.ftc.teamcode.util.aurora.SmartMechanumDrive;
import org.firstinspires.ftc.teamcode.util.tool.Pose;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class PoseController {

    // ======== Hardware interfaces ========
    private SmartMechanumDrive drive;
    private PositionManager positionManager;

    // ======== Tunables (start simple, adjust on the field) ========
    // Position P-gains (separate per axis for better tuning)
    public double kP_x = 0.06;          // position gain for X axis (forward/back in field frame)
    public double kP_y = 0.06;          // position gain for Y axis (left/right in field frame) - may need different value for strafe
    public double kP_theta = 0.03;      // heading gain (yaw)

    // Feed-forward gains (velocity-based prediction, separate per axis)
    public double kV_x = 0.0;           // velocity feed-forward for X axis (start at 0, tune up if needed)
    public double kV_y = 0.0;           // velocity feed-forward for Y axis (start at 0, tune up if needed)
    public double kV_theta = 0.0;       // rotational velocity feed-forward (start at 0, tune up if needed)

    // Output limits
    public double maxAxial = 0.9;       // clamp for axial power
    public double maxLateral = 0.9;     // clamp for lateral power
    public double maxYaw = 0.7;         // clamp for yaw power

    // Tolerances
    public double posTolerance = 2;  // inches
    public double angTolerance = Math.toRadians(5); // radians

    // ======== Coordinate & Axis Config ========
    // Define how your odometry axes map to "field" directions.
    // By convention here: +x = field forward (away from your driver wall), +y = field left (driver-left)
    // If your odometry differs, fix it by flipping signs here (no other math changes needed).
    public int signX = +1;  // flip to -1 if your reported +x is actually "backwards"
    public int signY = +1;  // flip to -1 if your reported +y is actually "right"
    public int signHeading = +1; // flip to -1 if your heading increases clockwise (should be CCW)

    // If your drive expects a different robot-centric direction for "axial/lateral", adjust here:
    // axial = forward/back (robot +X), lateral = left/right (robot +Y)
    public int signAxialOut = +1;   // flip if your robot forward power is inverted
    public int signLateralOut = +1; // flip if your strafe direction is inverted
    public int signYawOut = +1;     // flip if yaw is inverted

    // ======== Heading Mode ========
    public enum HeadingMode {
        Hold,       // hold a specified targetHeading
        FaceTarget  // automatically yaw toward the XY target
    }

    // ======== Target Pose ========
    private Double targetX = null;
    private Double targetY = null;
    private Double targetHeading = null;
    private HeadingMode headingMode = HeadingMode.Hold;
    private boolean hasTarget = false;

    // ======== Feed-forward state ========
    private Double prevTargetX = null;
    private Double prevTargetY = null;
    private Double prevTargetHeading = null;
    private long lastUpdateTimeNanos = 0;
    private double targetVelocityX = 0;  // field frame (inches/sec)
    private double targetVelocityY = 0;  // field frame (inches/sec)
    private double targetVelocityHeading = 0;  // radians/sec

    // Robot velocity from odometry (for better feed-forward)
    private double robotVelocityX = 0;  // field frame (inches/sec)
    private double robotVelocityY = 0;  // field frame (inches/sec)
    private double robotVelocityHeading = 0;  // radians/sec

    // Feed-forward mode
    public boolean useRobotVelocityFeedForward = true;  // Use actual velocity instead of target velocity

    // ======== Waypoint Queue & Path Following ========
    private Queue<Pose> waypointQueue = new LinkedList<>();
    private List<Pose> fullPath = new ArrayList<>();  // Store complete path for reversing/partial execution
    private boolean isFollowingPath = false;
    private int currentWaypointIndex = 0;
    private int totalWaypoints = 0;

    // Path execution control
    private int startWaypointIndex = 0;   // Where to start in the path (for partial execution)
    private int endWaypointIndex = 0;     // Where to end in the path (for partial execution)
    private boolean autoAdvance = true;   // Automatically advance to next waypoint when at target

    /**
     * Constructor
     */
    public PoseController(SmartMechanumDrive drive, PositionManager positionManager) {
        this.drive = drive;
        this.positionManager = positionManager;

        // Disable acceleration limiting for direct PID control
        drive.setAccelerationLimiting(false);
    }

    /**
     * Set target position with a specific heading
     */
    public void setTarget(double x, double y, double heading) {
        // Calculate velocity if target changed
        long currentTime = System.nanoTime();
        if (lastUpdateTimeNanos > 0 && prevTargetX != null && prevTargetY != null && prevTargetHeading != null) {
            double dt = (currentTime - lastUpdateTimeNanos) / 1e9; // convert to seconds
            if (dt > 0) {
                targetVelocityX = (x - prevTargetX) / dt;
                targetVelocityY = (y - prevTargetY) / dt;
                targetVelocityHeading = angleWrap(heading - prevTargetHeading) / dt;
            }
        }

        this.prevTargetX = this.targetX;
        this.prevTargetY = this.targetY;
        this.prevTargetHeading = this.targetHeading;

        this.targetX = x;
        this.targetY = y;
        this.targetHeading = heading;
        this.headingMode = HeadingMode.Hold;
        this.hasTarget = true;
        this.lastUpdateTimeNanos = currentTime;
    }

    /**
     * Set target position and automatically face that point
     */
    public void setTargetFacePoint(double x, double y) {
        // Calculate velocity if target changed
        long currentTime = System.nanoTime();
        if (lastUpdateTimeNanos > 0 && prevTargetX != null && prevTargetY != null) {
            double dt = (currentTime - lastUpdateTimeNanos) / 1e9; // convert to seconds
            if (dt > 0) {
                targetVelocityX = (x - prevTargetX) / dt;
                targetVelocityY = (y - prevTargetY) / dt;
            }
        }

        this.prevTargetX = this.targetX;
        this.prevTargetY = this.targetY;

        this.targetX = x;
        this.targetY = y;
        this.headingMode = HeadingMode.FaceTarget;
        this.hasTarget = true;
        this.lastUpdateTimeNanos = currentTime;
    }

    /**
     * Clear the target - robot will stop moving
     */
    public void clearTarget() {
        this.targetX = null;
        this.targetY = null;
        this.targetHeading = null;
        this.hasTarget = false;

        // Reset feed-forward state
        this.prevTargetX = null;
        this.prevTargetY = null;
        this.prevTargetHeading = null;
        this.targetVelocityX = 0;
        this.targetVelocityY = 0;
        this.targetVelocityHeading = 0;
        this.robotVelocityX = 0;
        this.robotVelocityY = 0;
        this.robotVelocityHeading = 0;
        this.lastUpdateTimeNanos = 0;
    }

    /**
     * Check if a target is currently set
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    // ======== Velocity and Feed-Forward Getters ========

    /**
     * Get the current robot velocity from odometry (field frame)
     * @return [vx, vy, omega] where vx/vy are inches/sec and omega is rad/sec
     */
    public double[] getRobotVelocity() {
        return new double[]{robotVelocityX, robotVelocityY, robotVelocityHeading};
    }

    /**
     * Get the target velocity (from target changes)
     * @return [vx, vy, omega] where vx/vy are inches/sec and omega is rad/sec
     */
    public double[] getTargetVelocity() {
        return new double[]{targetVelocityX, targetVelocityY, targetVelocityHeading};
    }

    /**
     * Get the robot's speed (magnitude of velocity vector)
     * @return Speed in inches/sec
     */
    public double getRobotSpeed() {
        return Math.hypot(robotVelocityX, robotVelocityY);
    }

    /**
     * Set whether to use robot velocity or target velocity for feed-forward
     * @param useRobotVelocity true = use actual robot velocity (recommended), false = use target velocity
     */
    public void setUseRobotVelocityFeedForward(boolean useRobotVelocity) {
        this.useRobotVelocityFeedForward = useRobotVelocity;
    }

    /**
     * Set the heading control mode
     */
    public void setHeadingMode(HeadingMode mode) {
        this.headingMode = mode;
    }

    // ======== Gain Configuration Methods ========

    /**
     * Set all position P-gains to the same value (simple tuning)
     * @param kP Position gain for both X and Y axes
     */
    public void setPositionGains(double kP) {
        this.kP_x = kP;
        this.kP_y = kP;
    }

    /**
     * Set position P-gains separately per axis (advanced tuning)
     * @param kP_x Position gain for X axis (forward/back in field frame)
     * @param kP_y Position gain for Y axis (left/right in field frame)
     */
    public void setPositionGains(double kP_x, double kP_y) {
        this.kP_x = kP_x;
        this.kP_y = kP_y;
    }

    /**
     * Set all feed-forward gains to the same value (simple tuning)
     * @param kV Velocity feed-forward gain for both X and Y axes
     */
    public void setVelocityGains(double kV) {
        this.kV_x = kV;
        this.kV_y = kV;
    }

    /**
     * Set velocity feed-forward gains separately per axis (advanced tuning)
     * @param kV_x Velocity gain for X axis
     * @param kV_y Velocity gain for Y axis
     */
    public void setVelocityGains(double kV_x, double kV_y) {
        this.kV_x = kV_x;
        this.kV_y = kV_y;
    }

    /**
     * Set all gains at once (complete control setup)
     * @param kP_x Position gain for X axis
     * @param kP_y Position gain for Y axis
     * @param kP_theta Heading gain
     * @param kV_x Velocity feed-forward for X axis
     * @param kV_y Velocity feed-forward for Y axis
     * @param kV_theta Velocity feed-forward for heading
     */
    public void setAllGains(double kP_x, double kP_y, double kP_theta, double kV_x, double kV_y, double kV_theta) {
        this.kP_x = kP_x;
        this.kP_y = kP_y;
        this.kP_theta = kP_theta;
        this.kV_x = kV_x;
        this.kV_y = kV_y;
        this.kV_theta = kV_theta;
    }

    // ======== Helper Methods for Relative Movement ========

    /**
     * Move forward/backward relative to robot's current orientation
     * @param inches positive = forward, negative = backward
     */
    public void moveAxial(double inches) {
        double currentX = positionManager.getCurrentX();
        double currentY = positionManager.getCurrentY();
        double currentHeading = Math.toRadians(positionManager.getCurrentHeading());

        // Calculate target in field frame
        double targetX = currentX + inches * Math.cos(currentHeading);
        double targetY = currentY + inches * Math.sin(currentHeading);

        setTarget(targetX, targetY, Math.toDegrees(currentHeading));
    }

    /**
     * Strafe left/right relative to robot's current orientation
     * @param inches positive = left, negative = right
     */
    public void moveLateral(double inches) {
        double currentX = positionManager.getCurrentX();
        double currentY = positionManager.getCurrentY();
        double currentHeading = Math.toRadians(positionManager.getCurrentHeading());

        // Lateral is perpendicular to heading (90 degrees offset)
        double targetX = currentX + inches * Math.cos(currentHeading + Math.PI/2);
        double targetY = currentY + inches * Math.sin(currentHeading + Math.PI/2);

        setTarget(targetX, targetY, Math.toDegrees(currentHeading));
    }

    /**
     * Rotate relative to current heading
     * @param degrees positive = CCW, negative = CW
     */
    public void rotate(double degrees) {
        double currentX = positionManager.getCurrentX();
        double currentY = positionManager.getCurrentY();
        double currentHeading = positionManager.getCurrentHeading();

        setTarget(currentX, currentY, currentHeading + degrees);
    }

    /**
     * Move to a position relative to robot's current pose (robot-frame offset)
     * @param axialInches forward/back offset (positive = forward)
     * @param lateralInches left/right offset (positive = left)
     * @param headingDegrees rotation offset (positive = CCW)
     */
    public void moveRelative(double axialInches, double lateralInches, double headingDegrees) {
        double currentX = positionManager.getCurrentX();
        double currentY = positionManager.getCurrentY();
        double currentHeading = Math.toRadians(positionManager.getCurrentHeading());

        // Transform robot-frame offset to field frame
        double cosH = Math.cos(currentHeading);
        double sinH = Math.sin(currentHeading);
        double targetX = currentX + (axialInches * cosH - lateralInches * sinH);
        double targetY = currentY + (axialInches * sinH + lateralInches * cosH);

        setTarget(targetX, targetY, Math.toDegrees(currentHeading) + headingDegrees);
    }

    /**
     * Move to a position relative to robot's current pose and face that point
     * @param axialInches forward/back offset (positive = forward)
     * @param lateralInches left/right offset (positive = left)
     */
    public void moveRelativeFacePoint(double axialInches, double lateralInches) {
        double currentX = positionManager.getCurrentX();
        double currentY = positionManager.getCurrentY();
        double currentHeading = Math.toRadians(positionManager.getCurrentHeading());

        // Transform robot-frame offset to field frame
        double cosH = Math.cos(currentHeading);
        double sinH = Math.sin(currentHeading);
        double targetX = currentX + (axialInches * cosH - lateralInches * sinH);
        double targetY = currentY + (axialInches * sinH + lateralInches * cosH);

        setTargetFacePoint(targetX, targetY);
    }

    /**
     * Stop and hold the current position
     */
    public void holdPosition() {
        double currentX = positionManager.getCurrentX();
        double currentY = positionManager.getCurrentY();
        double currentHeading = positionManager.getCurrentHeading();

        setTarget(currentX, currentY, currentHeading);
    }

    // ======== Waypoint Queue Management ========

    /**
     * Load a path from PathPlanner into the waypoint queue
     * @param path List of Pose objects from PathPlanner.generatePath()
     */
    public void setPath(List<Pose> path) {
        clearWaypoints();
        if (path == null || path.isEmpty()) {
            return;
        }

        fullPath = new ArrayList<>(path);
        totalWaypoints = path.size();
        startWaypointIndex = 0;
        endWaypointIndex = totalWaypoints - 1;

        // Don't start following yet - user must call startPath()
        isFollowingPath = false;
    }

    /**
     * Add a single waypoint to the queue
     * @param x X position (inches)
     * @param y Y position (inches)
     * @param heading Heading (degrees)
     */
    public void addWaypoint(double x, double y, double heading) {
        Pose pose = new Pose(x, y, heading);
        fullPath.add(pose);
        totalWaypoints = fullPath.size();
        endWaypointIndex = totalWaypoints - 1;
    }

    /**
     * Add a waypoint to the queue
     * @param pose Pose object
     */
    public void addWaypoint(Pose pose) {
        fullPath.add(pose);
        totalWaypoints = fullPath.size();
        endWaypointIndex = totalWaypoints - 1;
    }

    /**
     * Clear all waypoints and stop path following
     */
    public void clearWaypoints() {
        waypointQueue.clear();
        fullPath.clear();
        isFollowingPath = false;
        currentWaypointIndex = 0;
        totalWaypoints = 0;
        startWaypointIndex = 0;
        endWaypointIndex = 0;
    }

    /**
     * Start following the loaded path (forward direction)
     * Non-blocking - call update() in your loop to execute
     */
    public void startPath() {
        if (fullPath.isEmpty()) {
            return;
        }

        // Build queue from start to end
        waypointQueue.clear();
        for (int i = startWaypointIndex; i <= endWaypointIndex; i++) {
            waypointQueue.add(fullPath.get(i));
        }

        isFollowingPath = true;
        currentWaypointIndex = startWaypointIndex;

        // Set first waypoint as target
        if (!waypointQueue.isEmpty()) {
            Pose first = waypointQueue.peek();
            setTarget(first.x, first.y, first.heading);
        }
    }

    /**
     * Start following the path in reverse (backward direction)
     * Non-blocking - call update() in your loop to execute
     */
    public void startPathReverse() {
        if (fullPath.isEmpty()) {
            return;
        }

        // Build queue from end to start with reversed headings
        waypointQueue.clear();
        for (int i = endWaypointIndex; i >= startWaypointIndex; i--) {
            Pose p = fullPath.get(i);
            // Reverse heading by 180 degrees for backing up
            waypointQueue.add(new Pose(p.x, p.y, p.heading + 180));
        }

        isFollowingPath = true;
        currentWaypointIndex = endWaypointIndex;

        // Set first waypoint as target
        if (!waypointQueue.isEmpty()) {
            Pose first = waypointQueue.peek();
            setTarget(first.x, first.y, first.heading);
        }
    }

    /**
     * Set which portion of the path to execute
     * @param startPercent Start percentage (0.0 to 1.0)
     * @param endPercent End percentage (0.0 to 1.0)
     */
    public void setPathRange(double startPercent, double endPercent) {
        if (fullPath.isEmpty()) {
            return;
        }

        // Clamp percentages
        startPercent = Math.max(0.0, Math.min(1.0, startPercent));
        endPercent = Math.max(0.0, Math.min(1.0, endPercent));

        // Ensure start < end
        if (startPercent > endPercent) {
            double temp = startPercent;
            startPercent = endPercent;
            endPercent = temp;
        }

        // Calculate indices
        startWaypointIndex = (int) (startPercent * (totalWaypoints - 1));
        endWaypointIndex = (int) (endPercent * (totalWaypoints - 1));

        // Ensure valid range
        startWaypointIndex = Math.max(0, Math.min(totalWaypoints - 1, startWaypointIndex));
        endWaypointIndex = Math.max(0, Math.min(totalWaypoints - 1, endWaypointIndex));
    }

    /**
     * Execute only a percentage of the path from the start
     * @param percent Percentage of path to execute (0.0 to 1.0)
     */
    public void setPathPercent(double percent) {
        setPathRange(0.0, percent);
    }

    /**
     * Stop path following immediately
     * Robot will hold current position
     */
    public void stopPath() {
        isFollowingPath = false;
        waypointQueue.clear();
        clearTarget();
    }

    /**
     * Pause path following at current waypoint
     * Call resumePath() to continue
     */
    public void pausePath() {
        autoAdvance = false;
    }

    /**
     * Resume path following after pause
     */
    public void resumePath() {
        autoAdvance = true;
    }

    /**
     * Check if currently following a path
     * @return true if path is active
     */
    public boolean isFollowingPath() {
        return isFollowingPath;
    }

    /**
     * Check if path is complete
     * @return true if all waypoints have been visited
     */
    public boolean isPathComplete() {
        return isFollowingPath && waypointQueue.isEmpty();
    }

    /**
     * Get current progress through the path
     * @return Percentage complete (0.0 to 1.0)
     */
    public double getPathProgress() {
        if (totalWaypoints == 0) {
            return 0.0;
        }
        return (double) currentWaypointIndex / (double) totalWaypoints;
    }

    /**
     * Get number of waypoints remaining
     * @return Number of waypoints left to visit
     */
    public int getRemainingWaypoints() {
        return waypointQueue.size();
    }

    /**
     * Get total number of waypoints in current path
     * @return Total waypoint count
     */
    public int getTotalWaypoints() {
        return totalWaypoints;
    }

    /**
     * Get the full path currently loaded
     * @return List of all path waypoints
     */
    public List<Pose> getFullPath() {
        return new ArrayList<>(fullPath);
    }

    // ======== Main update ========
    /**
     * Main update loop - MUST be called every iteration
     * This is non-blocking and updates PID controllers
     * Also handles automatic waypoint advancement if following a path
     */
    public void update() {
        // Update position from odometry
        positionManager.updatePositionFromOdometry();

        // Get robot velocity from odometry for feed-forward
        double[] velocity = positionManager.getVelocityFromOdometry();
        if (velocity != null && velocity.length >= 3) {
            robotVelocityX = velocity[0];  // inches/sec in field frame
            robotVelocityY = velocity[1];  // inches/sec in field frame
            robotVelocityHeading = Math.toRadians(velocity[2]);  // convert deg/s to rad/s
        } else {
            robotVelocityX = 0;
            robotVelocityY = 0;
            robotVelocityHeading = 0;
        }

        // Handle automatic waypoint advancement
        if (isFollowingPath && autoAdvance && atTarget() && !waypointQueue.isEmpty()) {
            // Remove current waypoint
            waypointQueue.poll();
            currentWaypointIndex++;

            // Set next waypoint as target
            if (!waypointQueue.isEmpty()) {
                Pose next = waypointQueue.peek();
                setTarget(next.x, next.y, next.heading);
            } else {
                // Path complete
                isFollowingPath = false;
            }
        }

        // If no target is set, stop the robot
        if (!hasTarget || targetX == null || targetY == null) {
            drive.setDriveInputs(0, 0, 0);
            drive.update();
            return;
        }

        // Get current position
        double x = positionManager.getCurrentX();
        double y = positionManager.getCurrentY();
        double heading = Math.toRadians(positionManager.getCurrentHeading());

        // Normalize inputs to our sign convention
        double fx = signX * x;
        double fy = signY * y;
        double h  = signHeading * heading;

        double ftx = signX * targetX;
        double fty = signY * targetY;

        // 1) Field-frame error
        double dx = ftx - fx;
        double dy = fty - fy;

        // 2) Rotate error into robot frame (so "forward" is axial, "left" is lateral)
        // Robot frame rotation by -heading
        double cosH = Math.cos(-h);
        double sinH = Math.sin(-h);
        double ex_robot = dx * cosH - dy * sinH; // forward/back
        double ey_robot = dx * sinH + dy * cosH; // left/right

        // 3) Position P-control + Feed-forward -> desired translational powers
        // Choose velocity source: actual robot velocity or target velocity
        double feedForwardVX = useRobotVelocityFeedForward ? robotVelocityX : targetVelocityX;
        double feedForwardVY = useRobotVelocityFeedForward ? robotVelocityY : targetVelocityY;
        double feedForwardVHeading = useRobotVelocityFeedForward ? robotVelocityHeading : targetVelocityHeading;

        // Rotate velocity into robot frame for feed-forward
        double vx_robot = feedForwardVX * cosH - feedForwardVY * sinH;
        double vy_robot = feedForwardVX * sinH + feedForwardVY * cosH;

        // Apply separate gains per axis
        // Note: We use field-frame gains (kP_x, kP_y, kV_x, kV_y) applied to field-frame errors/velocities,
        // then rotate the resulting control vector into robot frame
        double controlX_field = kP_x * dx + kV_x * feedForwardVX;
        double controlY_field = kP_y * dy + kV_y * feedForwardVY;

        // Rotate control output to robot frame
        double axial = controlX_field * cosH - controlY_field * sinH;
        double lateral = controlX_field * sinH + controlY_field * cosH;

        // 4) Heading control
        double yaw;
        double desiredHeading = targetHeading != null ? targetHeading : heading;

        if (headingMode == HeadingMode.FaceTarget) {
            desiredHeading = Math.atan2(dy, dx); // face the target
        }

        // Note: desiredHeading is in field frame; wrap and compare to current heading
        double headingErr = angleWrap(desiredHeading - h);
        yaw = kP_theta * headingErr + kV_theta * feedForwardVHeading;

        // 5) Output sign & clamp
        axial   = clamp(signAxialOut   * axial,   -maxAxial,   maxAxial);
        lateral = clamp(signLateralOut * lateral, -maxLateral, maxLateral);
        yaw     = clamp(signYawOut     * yaw,     -maxYaw,     maxYaw);

        // 6) Send to drive system
        drive.setDriveInputs(axial, lateral, yaw);
        drive.update();
    }

    /**
     * Check if robot is at target position
     */
    public boolean atTarget() {
        // No target set means we're always "at target" (idle state)
        if (!hasTarget || targetX == null || targetY == null) {
            return true;
        }

        double x = positionManager.getCurrentX();
        double y = positionManager.getCurrentY();
        double heading = Math.toRadians(positionManager.getCurrentHeading());

        double fx = signX * x, fy = signY * y, h = signHeading * heading;
        double ftx = signX * targetX, fty = signY * targetY;
        double dist = Math.hypot(ftx - fx, fty - fy);

        double desiredHeading = (headingMode == HeadingMode.FaceTarget)
                ? Math.atan2((fty - fy), (ftx - fx))
                : (targetHeading != null ? targetHeading : heading);

        double dTheta = Math.abs(angleWrap(desiredHeading - h));
        return dist <= posTolerance && dTheta <= angTolerance;
    }

    // ======== Utils ========
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }

}

