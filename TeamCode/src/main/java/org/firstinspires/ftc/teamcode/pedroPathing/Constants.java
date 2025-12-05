package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class Constants {
    /**
     * Robot mass in kilograms (for centripetal force compensation)
     * Tip: Stand on a scale with your robot, then subtract your weight
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-35.66)
            .lateralZeroPowerAcceleration(-55.63)
            .mass(9); // Update this with your robot's actual mass in kg

    /**
     * Path constraints: (maxPower, maxAccel, maxDecel, maxAngularVelocity)
     * TODO: Tune these values during the tuning process
     */
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * Mecanum drivetrain configuration
     * Motor names match your existing Aurora Lightning setup:
     * - frontLeft, frontRight, backLeft, backRight
     * TODO: Test motor directions and adjust if needed during tuning
     */
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(60.92)
            .yVelocity(50.99);

    /**
     * Pinpoint Odometry Computer Configuration
     * Settings match your existing OdoHelper.java configuration:
     * - X_OFFSET: -154mm (-6.06 inches) - Forward pod is 154mm BEHIND center
     * - Y_OFFSET: 0mm (0 inches) - Strafe pod is at center
     * - Pod Type: goBILDA_4_BAR_POD
     * - Hardware config name: "odo"
     * - Both encoders: FORWARD direction
     *
     * NOTE: These are the verified settings from your Aurora Lightning system.
     * Pedro Pathing uses different offset naming:
     * - forwardPodY = Y offset of forward encoder (your X_OFFSET converted)
     * - strafePodX = X offset of strafe encoder (your Y_OFFSET converted)
     */
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.83)
            .strafePodX(4.71)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")  // Matches your hardware config
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)  // Strafe pod reversed so Y increases when moving left
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD); // Forward pod forward so X increases when moving forward
    // Forward encoder direction is FORWARD by default
    // Strafe encoder is REVERSED to correct localization direction
    // Run "Localization Test" to verify: Forward = X increases, Left = Y increases

    /**
     * Creates a Follower instance with the configured constants
     * This is the main object used to follow paths
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
