package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.DeadWheelOdometry;

// Used to manage encoders and dead wheel data for precise movement and positioning in autonomous mode
public class EncoderManager {
    private DeadWheelOdometry odometry;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Motor encoder references for direct access
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor horizontalEncoder;

    /**
     * Initialize the encoder manager with hardware map
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     */
    public EncoderManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Initialize the unified odometry system
        this.odometry = new DeadWheelOdometry(hardwareMap, telemetry);

        // Try to get direct encoder references for additional functionality
        try {
            leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
            rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
            horizontalEncoder = hardwareMap.get(DcMotor.class, "horizontalEncoder");
        } catch (Exception e) {
            telemetry.addData("EncoderManager", "Direct encoder access not available");
        }
    }

    /**
     * Initialize odometry system with starting position
     * @param startX Starting X position
     * @param startY Starting Y position
     * @param startHeading Starting heading in degrees
     */
    public void initialize(double startX, double startY, double startHeading) {
        odometry.setPosition(startX, startY, startHeading);
    }

    /**
     * Update encoder readings and calculate position
     */
    public void update() {
        odometry.updatePosition();
    }

    /**
     * Get current X position
     * @return X position in inches
     */
    public double getX() {
        return odometry.getX();
    }

    /**
     * Get current Y position
     * @return Y position in inches
     */
    public double getY() {
        return odometry.getY();
    }

    /**
     * Get current heading
     * @return Heading in degrees
     */
    public double getHeading() {
        return odometry.getHeadingDegrees();
    }

    /**
     * Get current velocity in X direction
     * @return X velocity in inches per second
     */
    public double getVelocityX() {
        double[] velocity = odometry.getVelocity();
        return velocity[0];
    }

    /**
     * Get current velocity in Y direction
     * @return Y velocity in inches per second
     */
    public double getVelocityY() {
        double[] velocity = odometry.getVelocity();
        return velocity[1];
    }

    /**
     * Get current angular velocity
     * @return Angular velocity in degrees per second
     */
    public double getAngularVelocity() {
        double[] velocity = odometry.getVelocity();
        return Math.toDegrees(velocity[2]); // Convert from radians/sec to degrees/sec
    }

    /**
     * Reset all encoder positions to zero
     */
    public void resetEncoders() {
        if (leftEncoder != null) leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightEncoder != null) rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (horizontalEncoder != null) horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (leftEncoder != null) leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (rightEncoder != null) rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (horizontalEncoder != null) horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Get raw encoder position
     * @param encoderName Name of encoder ("left", "right", "horizontal")
     * @return Raw encoder ticks
     */
    public int getRawEncoderPosition(String encoderName) {
        switch (encoderName.toLowerCase()) {
            case "left":
                return leftEncoder != null ? leftEncoder.getCurrentPosition() : 0;
            case "right":
                return rightEncoder != null ? rightEncoder.getCurrentPosition() : 0;
            case "horizontal":
                return horizontalEncoder != null ? horizontalEncoder.getCurrentPosition() : 0;
            default:
                return 0;
        }
    }

    /**
     * Add encoder telemetry data
     */
    public void addTelemetry() {
        odometry.addTelemetry();

        if (leftEncoder != null) {
            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
        }
        if (rightEncoder != null) {
            telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        }
        if (horizontalEncoder != null) {
            telemetry.addData("Horizontal Encoder", horizontalEncoder.getCurrentPosition());
        }
    }

    /**
     * Check if odometry system is ready
     * @return True if odometry is initialized and working
     */
    public boolean isReady() {
        return odometry != null;
    }
}
