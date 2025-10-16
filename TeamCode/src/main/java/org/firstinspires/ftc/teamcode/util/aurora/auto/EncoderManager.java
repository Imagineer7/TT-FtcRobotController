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
            telemetry.addData("EncoderManager", "Direct encoder access initialized successfully");
        } catch (Exception e) {
            // Try alternative encoder names that might be used
            try {
                leftEncoder = hardwareMap.get(DcMotor.class, "frontLeft");
                rightEncoder = hardwareMap.get(DcMotor.class, "frontRight");
                horizontalEncoder = hardwareMap.get(DcMotor.class, "backLeft");
                telemetry.addData("EncoderManager", "Using drive motors for encoder access");
            } catch (Exception e2) {
                telemetry.addData("EncoderManager", "Direct encoder access not available - using odometry system only");
                // This is not an error - the odometry system will still work
            }
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
     * Correct position using external data (e.g., vision system)
     * @param correctedX Corrected X position
     * @param correctedY Corrected Y position
     * @param correctedHeading Corrected heading in degrees
     */
    public void correctPosition(double correctedX, double correctedY, double correctedHeading) {
        odometry.setPosition(correctedX, correctedY, correctedHeading);
        telemetry.addData("EncoderManager", "Position corrected to: (%.1f, %.1f) @ %.1f°",
                         correctedX, correctedY, correctedHeading);
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
        return velocity[2];
    }

    /**
     * Reset encoder positions (typically called at start of autonomous)
     */
    public void resetEncoders() {
        if (leftEncoder != null) {
            leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightEncoder != null) {
            rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (horizontalEncoder != null) {
            horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Also reset the odometry system
        odometry.setPosition(0, 0, 0);

        telemetry.addData("EncoderManager", "Encoders reset");
    }

    /**
     * Get encoder counts for debugging
     * @return Array of [left, right, horizontal] encoder counts
     */
    public int[] getEncoderCounts() {
        int[] counts = new int[3];
        counts[0] = leftEncoder != null ? leftEncoder.getCurrentPosition() : 0;
        counts[1] = rightEncoder != null ? rightEncoder.getCurrentPosition() : 0;
        counts[2] = horizontalEncoder != null ? horizontalEncoder.getCurrentPosition() : 0;
        return counts;
    }

    /**
     * Add encoder telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== ENCODER DATA ===", "");

        // Current position
        telemetry.addData("Encoder Position", "(%.1f, %.1f) @ %.1f°", getX(), getY(), getHeading());

        // Current velocity
        telemetry.addData("Velocity", "X:%.1f Y:%.1f Ω:%.1f",
                         getVelocityX(), getVelocityY(), getAngularVelocity());

        // Raw encoder counts (if available)
        if (leftEncoder != null) {
            int[] counts = getEncoderCounts();
            telemetry.addData("Raw Counts", "L:%d R:%d H:%d", counts[0], counts[1], counts[2]);
        }

        // System status
        telemetry.addData("Odometry Ready", isReady() ? "YES" : "NO");
    }

    /**
     * Check if odometry system is ready
     * @return True if odometry is initialized and working
     */
    public boolean isReady() {
        return odometry != null;
    }
}
