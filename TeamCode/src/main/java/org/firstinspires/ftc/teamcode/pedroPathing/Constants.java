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

public class Constants {
    /**
     * Robot mass in kilograms (for centripetal force compensation)
     * TODO: Weigh your robot and update this value
     * Tip: Stand on a scale with your robot, then subtract your weight
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5); // Update this with your robot's actual mass in kg

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
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

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
            .forwardPodY(-6.06)  // -154mm converted to inches (forward pod BEHIND center)
            .strafePodX(0)       // 0mm = at centerline
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo");  // Matches your hardware config
    // Encoder directions are set to FORWARD by default in Pedro Pathing
    // This matches your OdoHelper configuration: both encoders FORWARD
    // Run "Localization Test" to verify movement directions are correct

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
