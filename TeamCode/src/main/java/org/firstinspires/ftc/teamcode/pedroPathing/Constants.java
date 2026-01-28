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
     * Aurora Lightning robot mass: ~25 lbs = 11.34 kg
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-35.0719)
            .lateralZeroPowerAcceleration(-83.6295)
            .mass(11.34); // 25 lbs robot mass

    /**
     * Path constraints: (maxPower, maxAccel, maxDecel, maxAngularVelocity)
     * TODO: Tune these values during the tuning process
     */
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * Mecanum drivetrain configuration
     * Motor names EXACTLY match AuroraHardwareConfig:
     * - "Left Front" (REVERSED, BRAKE)
     * - "Right Front" (FORWARD, BRAKE)
     * - "Left Back" (REVERSED, BRAKE)
     * - "Right Back" (FORWARD, BRAKE)
     * All motors: RUN_WITHOUT_ENCODER (for odometry)
     */
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("Right Front")
            .rightRearMotorName("Right Back")
            .leftRearMotorName("Left Back")
            .leftFrontMotorName("Left Front")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(53.5116)
            .yVelocity(33.7778);

    /**
     * Pinpoint Odometry Computer Configuration
     * Values from verified Localization.java Aurora Lightning setup:
     * - Forward Pod Y Offset: -6.06 inches (154mm BEHIND robot center)
     * - Strafe Pod X Offset: 0 inches (at robot center)
     * - Pod Type: goBILDA_4_BAR_POD
     * - Hardware name: "odo" (matches AuroraHardwareConfig.ODOMETRY_COMPUTER)
     * - Forward encoder: FORWARD direction (X increases when moving forward)
     * - Strafe encoder: REVERSED direction (Y increases when moving left)
     *
     * NOTE: Pedro Pathing coordinate system matches FTC field coordinates
     * Run "Localization Test" to verify: Forward = X+, Left = Y+, CCW = heading+
     */
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.947)  // Forward pod is 6.06" behind center (verified from Localization.java)
            .strafePodX(0.0)     // Strafe pod at center (verified from Localization.java)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")  // EXACT match to AuroraHardwareConfig.ODOMETRY_COMPUTER
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

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
