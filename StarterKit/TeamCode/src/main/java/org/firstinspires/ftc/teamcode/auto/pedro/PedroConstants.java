/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * PedroConstants - Configuration for Pedro Pathing autonomous system
 * 
 * This file contains all tuning parameters for the Pedro Pathing library.
 * Advanced teams should tune these values during practice.
 */

package org.firstinspires.ftc.teamcode.auto.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PedroConstants - Configuration for Pedro Pathing
 * 
 * Purpose:
 * - Define all tuning parameters for Pedro path following
 * - Configure drivetrain (motor names, directions)
 * - Configure localization (odometry computer or encoders)
 * - Define path constraints (speed, acceleration limits)
 * 
 * Important Notes:
 * - These values are starting points - tune them for your robot!
 * - Motor names and directions must match your robot configuration
 * - Odometry offsets are critical for accurate positioning
 * 
 * Coordinate System:
 * - Origin (0, 0) is at field center
 * - +X points toward red alliance wall
 * - +Y points toward blue audience side
 * - +Heading is counterclockwise (0° = facing +X direction)
 */
public class PedroConstants {
    
    // ═══════════════════════════════════════════════════════════════
    // FOLLOWER CONSTANTS (Tuning Parameters)
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Robot mass in kilograms (for centripetal force compensation)
     * 
     * How to measure:
     * 1. Stand on a scale with your robot
     * 2. Subtract your weight
     * 3. Convert pounds to kg (divide by 2.205)
     * 
     * Why it matters:
     * - Heavier robots need more force to turn
     * - Pedro uses this to predict how much power is needed for curves
     */
    public static final double ROBOT_MASS_KG = 9.0;  // Update with your robot's mass
    
    /**
     * Zero-power acceleration (deceleration when no power applied)
     * 
     * These values describe how quickly your robot slows down:
     * - Forward: When driving straight and power is cut
     * - Lateral: When strafing and power is cut
     * 
     * Negative values = deceleration (robot slows down)
     * 
     * How to tune:
     * 1. Drive robot at full speed
     * 2. Cut power completely
     * 3. Time how long it takes to stop
     * 4. Calculate deceleration
     * 
     * Starting values (adjust based on your wheels and weight):
     */
    public static final double FORWARD_ZERO_POWER_ACCEL = -35.66;
    public static final double LATERAL_ZERO_POWER_ACCEL = -55.63;
    
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(FORWARD_ZERO_POWER_ACCEL)
            .lateralZeroPowerAcceleration(LATERAL_ZERO_POWER_ACCEL)
            .mass(ROBOT_MASS_KG);
    
    // ═══════════════════════════════════════════════════════════════
    // PATH CONSTRAINTS (Speed and Acceleration Limits)
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Path constraints define how aggressively Pedro moves
     * 
     * Parameters: (maxPower, maxAccel, maxDecel, maxAngularVelocity)
     * 
     * maxPower: Maximum motor power (0.0 to 1.0)
     *   - Start conservative (0.8) and increase as you gain confidence
     *   - Too high = overshoots and unstable paths
     *   - Too low = slow autonomous
     * 
     * maxAccel/maxDecel: How quickly robot speeds up/slows down
     *   - Higher values = faster movements but less smooth
     *   - Lower values = smoother but slower
     *   - Units: power units per second
     * 
     * maxAngularVelocity: How fast robot can rotate
     *   - Higher values = faster turns
     *   - Units: radians per second
     */
    public static final double MAX_POWER = 0.85;           // Start at 85% for safety
    public static final double MAX_ACCELERATION = 80.0;    // Moderate acceleration
    public static final double MAX_DECELERATION = 100.0;   // Can brake harder than accelerate
    public static final double MAX_ANGULAR_VELOCITY = 2.0; // Radians per second
    
    public static PathConstraints pathConstraints = new PathConstraints(
        MAX_POWER, 
        MAX_ACCELERATION, 
        MAX_DECELERATION, 
        MAX_ANGULAR_VELOCITY
    );
    
    // ═══════════════════════════════════════════════════════════════
    // MECANUM DRIVETRAIN CONFIGURATION
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Motor names and directions
     * 
     * IMPORTANT: These MUST match your robot configuration file!
     * 
     * Standard configuration (same as StarterDrive):
     * - Left motors: REVERSE
     * - Right motors: FORWARD
     * 
     * If your robot moves wrong direction when testing:
     * 1. Try swapping left/right motor directions
     * 2. OR swap FORWARD/REVERSE on all motors
     */
    public static final String FRONT_LEFT_MOTOR = "frontLeft";
    public static final String FRONT_RIGHT_MOTOR = "frontRight";
    public static final String BACK_LEFT_MOTOR = "backLeft";
    public static final String BACK_RIGHT_MOTOR = "backRight";
    
    /**
     * Velocity calibration (inches per second)
     * 
     * These values help Pedro predict how fast your robot moves:
     * - xVelocity: Forward/backward speed at full power
     * - yVelocity: Strafe left/right speed at full power
     * 
     * How to measure:
     * 1. Drive straight forward at full power for 3 seconds
     * 2. Measure distance traveled
     * 3. Calculate speed: distance (inches) / time (seconds)
     * 4. Repeat for strafing
     * 
     * Typical values for mecanum:
     * - Forward: 50-70 in/s
     * - Strafe: 40-60 in/s (usually slower than forward)
     */
    public static final double X_VELOCITY = 60.0;  // Forward/backward inches per second
    public static final double Y_VELOCITY = 50.0;  // Strafe inches per second
    
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(MAX_POWER)
            .rightFrontMotorName(FRONT_RIGHT_MOTOR)
            .rightRearMotorName(BACK_RIGHT_MOTOR)
            .leftRearMotorName(BACK_LEFT_MOTOR)
            .leftFrontMotorName(FRONT_LEFT_MOTOR)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(X_VELOCITY)
            .yVelocity(Y_VELOCITY);
    
    // ═══════════════════════════════════════════════════════════════
    // PINPOINT ODOMETRY CONFIGURATION
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * GoBilda Pinpoint Odometry Computer settings
     * 
     * The Pinpoint uses two encoder "pods" to track robot position:
     * - Forward pod: Tracks forward/backward movement
     * - Strafe pod: Tracks left/right movement
     * - Built-in IMU: Tracks heading (rotation)
     * 
     * CRITICAL: Pod offsets MUST be measured accurately!
     * 
     * How to measure offsets:
     * 1. Find the center of rotation of your robot (usually geometric center)
     * 2. Measure distance from center to each pod:
     *    - forwardPodY: Y distance from center to forward pod
     *      (positive = pod is toward blue side, negative = toward red side)
     *    - strafePodX: X distance from center to strafe pod
     *      (positive = pod is toward red wall, negative = toward blue wall)
     * 
     * Encoder directions:
     * - Test by driving robot and checking if X/Y increase correctly
     * - Forward = X should increase
     * - Left = Y should increase
     * - If wrong, flip encoder direction
     * 
     * Hardware config name:
     * - Must match the name in your robot configuration
     * - Default: "odo"
     */
    public static final double FORWARD_POD_Y_OFFSET = -6.62;  // Inches from center (Y direction)
    public static final double STRAFE_POD_X_OFFSET = 4.71;    // Inches from center (X direction)
    public static final String ODOMETRY_HARDWARE_NAME = "odo";
    
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(FORWARD_POD_Y_OFFSET)
            .strafePodX(STRAFE_POD_X_OFFSET)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(ODOMETRY_HARDWARE_NAME)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    
    // ═══════════════════════════════════════════════════════════════
    // FOLLOWER FACTORY METHOD
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Creates a Follower instance with all configured constants
     * 
     * This is the main object used to follow paths in autonomous.
     * 
     * Usage in OpMode:
     *   Follower follower = PedroConstants.createFollower(hardwareMap);
     *   follower.setStartingPose(startPose);
     *   follower.followPath(myPath);
     * 
     * @param hardwareMap Hardware map from OpMode
     * @return Configured Follower ready for path following
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    
    /**
     * Creates a Follower with custom path constraints
     * 
     * Useful when you need different constraints for different paths:
     * - Slow, precise movements for scoring
     * - Fast movements for cycling
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param customConstraints Custom path constraints
     * @return Configured Follower with custom constraints
     */
    public static Follower createFollower(HardwareMap hardwareMap, PathConstraints customConstraints) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(customConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
