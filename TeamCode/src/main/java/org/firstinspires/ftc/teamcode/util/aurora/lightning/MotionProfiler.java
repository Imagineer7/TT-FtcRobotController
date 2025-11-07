package org.firstinspires.ftc.teamcode.util.aurora.lightning;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * MotionProfiler - Intelligent real-time robot motion analysis
 *
 * Tracks and analyzes:
 * - Actual vs. planned trajectories
 * - Distance to target (convergence analysis)
 * - Wheel slip detection
 * - Velocity profiles and smoothness
 * - Acceleration patterns
 * - Turning efficiency
 * - Motion anomalies (stuck, oscillating, drifting)
 * - Path deviation
 *
 * Provides:
 * - Real-time convergence status
 * - Motion quality scores
 * - Anomaly alerts
 * - Performance suggestions
 */
public class MotionProfiler {

    // Telemetry for warnings
    private Telemetry telemetry;

    // Position tracking
    private CircularDataBuffer<Double> xPositions = new CircularDataBuffer<>(100);
    private CircularDataBuffer<Double> yPositions = new CircularDataBuffer<>(100);
    private CircularDataBuffer<Double> headings = new CircularDataBuffer<>(100);

    // Target tracking
    private Double targetX = null;
    private Double targetY = null;
    private Double targetHeading = null;

    // Distance to target history
    private CircularDataBuffer<Double> distanceToTarget = new CircularDataBuffer<>(100);
    private CircularDataBuffer<Double> headingError = new CircularDataBuffer<>(100);

    // Velocity tracking
    private CircularDataBuffer<Double> velocityX = new CircularDataBuffer<>(100);
    private CircularDataBuffer<Double> velocityY = new CircularDataBuffer<>(100);
    private CircularDataBuffer<Double> velocityMagnitude = new CircularDataBuffer<>(100);
    private CircularDataBuffer<Double> angularVelocity = new CircularDataBuffer<>(100);

    // Acceleration tracking
    private CircularDataBuffer<Double> accelerationMagnitude = new CircularDataBuffer<>(50);
    private CircularDataBuffer<Double> angularAcceleration = new CircularDataBuffer<>(50);

    // Jerk tracking (smoothness)
    private CircularDataBuffer<Double> jerkMagnitude = new CircularDataBuffer<>(50);

    // Convergence analysis
    private boolean isConverging = false;
    private boolean isDiverging = false;
    private boolean isOscillating = false;
    private boolean isStuck = false;
    private boolean isSlipping = false;
    private int convergenceStreak = 0;
    private int divergenceStreak = 0;
    private int oscillationCount = 0;

    // Motion state
    private double lastDistance = Double.MAX_VALUE;
    private double previousVelocityMag = 0;
    private double previousAngularVel = 0;
    private ElapsedTime motionTimer = new ElapsedTime();
    private ElapsedTime stuckTimer = new ElapsedTime();

    // Thresholds and parameters
    public double convergenceThreshold = 0.5;  // inches - distance must decrease by this much
    public double stuckVelocityThreshold = 0.5;  // in/s - below this is considered stuck
    public double stuckTimeThreshold = 1.0;  // seconds - how long before warning
    public double oscillationThreshold = 3.0;  // distance changes sign this many times = oscillating
    public double slipDetectionThreshold = 2.0;  // velocity vs. commanded discrepancy
    public double smoothnessThreshold = 50.0;  // jerk magnitude threshold for rough motion

    // Statistics
    private double totalDistance = 0;
    private double peakVelocity = 0;
    private double peakAcceleration = 0;
    private int samplesCollected = 0;

    /**
     * Constructor
     */
    public MotionProfiler(Telemetry telemetry) {
        this.telemetry = telemetry;
        motionTimer.reset();
        stuckTimer.reset();
    }

    /**
     * Update motion profiler with current state
     * Call this every loop iteration
     *
     * @param currentX Current X position (inches)
     * @param currentY Current Y position (inches)
     * @param currentHeading Current heading (degrees)
     * @param velX Current X velocity (in/s)
     * @param velY Current Y velocity (in/s)
     * @param angVel Current angular velocity (deg/s)
     */
    public void update(double currentX, double currentY, double currentHeading,
                      double velX, double velY, double angVel) {

        // Store position history
        xPositions.add(currentX);
        yPositions.add(currentY);
        headings.add(currentHeading);

        // Store velocity
        velocityX.add(velX);
        velocityY.add(velY);
        double velMag = Math.hypot(velX, velY);
        velocityMagnitude.add(velMag);
        angularVelocity.add(angVel);

        // Calculate acceleration (change in velocity)
        if (samplesCollected > 0) {
            double accelMag = Math.abs(velMag - previousVelocityMag) / motionTimer.seconds();
            accelerationMagnitude.add(accelMag);

            double angAccel = Math.abs(angVel - previousAngularVel) / motionTimer.seconds();
            angularAcceleration.add(angAccel);

            // Calculate jerk (change in acceleration) for smoothness
            if (accelerationMagnitude.size() > 1) {
                double prevAccel = accelerationMagnitude.get(accelerationMagnitude.size() - 2);
                double jerk = Math.abs(accelMag - prevAccel) / motionTimer.seconds();
                jerkMagnitude.add(jerk);
            }

            // Track peak values
            if (velMag > peakVelocity) peakVelocity = velMag;
            if (accelMag > peakAcceleration) peakAcceleration = accelMag;
        }

        previousVelocityMag = velMag;
        previousAngularVel = angVel;

        // Calculate distance traveled (integrate velocity)
        if (samplesCollected > 0) {
            totalDistance += velMag * motionTimer.seconds();
        }

        // Analyze convergence if target is set
        if (targetX != null && targetY != null) {
            analyzeConvergence(currentX, currentY, currentHeading);
        }

        // Detect motion anomalies
        detectStuck(velMag);
        detectSlippage(velX, velY);

        samplesCollected++;
        motionTimer.reset();
    }

    /**
     * Set the current target for convergence analysis
     */
    public void setTarget(double targetX, double targetY, double targetHeading) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = targetHeading;

        // Reset convergence tracking for new target
        convergenceStreak = 0;
        divergenceStreak = 0;
        oscillationCount = 0;
        lastDistance = Double.MAX_VALUE;
    }

    /**
     * Clear the target
     */
    public void clearTarget() {
        targetX = null;
        targetY = null;
        targetHeading = null;
        isConverging = false;
        isDiverging = false;
        isOscillating = false;
    }

    /**
     * Analyze convergence toward target
     */
    private void analyzeConvergence(double currentX, double currentY, double currentHeading) {
        // Calculate current distance to target
        double distance = Math.hypot(targetX - currentX, targetY - currentY);
        distanceToTarget.add(distance);

        // Calculate heading error
        if (targetHeading != null) {
            double headErr = normalizeAngle(targetHeading - currentHeading);
            headingError.add(Math.abs(headErr));
        }

        // Analyze convergence trend
        if (lastDistance != Double.MAX_VALUE) {
            double deltaDistance = distance - lastDistance;

            // Converging: getting closer
            if (deltaDistance < -convergenceThreshold) {
                convergenceStreak++;
                divergenceStreak = 0;

                if (convergenceStreak >= 3) {
                    isConverging = true;
                    isDiverging = false;
                }
            }
            // Diverging: getting farther
            else if (deltaDistance > convergenceThreshold) {
                divergenceStreak++;
                convergenceStreak = 0;

                if (divergenceStreak >= 3) {
                    isDiverging = true;
                    isConverging = false;

                    // Log warning
                    telemetry.addData("⚠️ WARNING", "Diverging from target!");
                }
            }
            // Oscillating: distance changing direction frequently
            else if (Math.abs(deltaDistance) > 0.1) {
                // Check if distance is oscillating
                if (distanceToTarget.size() >= 10) {
                    int directionChanges = 0;
                    double prevDelta = 0;

                    for (int i = distanceToTarget.size() - 10; i < distanceToTarget.size() - 1; i++) {
                        double delta = distanceToTarget.get(i + 1) - distanceToTarget.get(i);
                        if (prevDelta != 0 && Math.signum(delta) != Math.signum(prevDelta)) {
                            directionChanges++;
                        }
                        prevDelta = delta;
                    }

                    if (directionChanges >= oscillationThreshold) {
                        isOscillating = true;
                        telemetry.addData("⚠️ WARNING", "Oscillating around target!");
                    } else {
                        isOscillating = false;
                    }
                }
            }
        }

        lastDistance = distance;
    }

    /**
     * Detect if robot is stuck (not moving despite control input)
     */
    private void detectStuck(double velocityMagnitude) {
        if (velocityMagnitude < stuckVelocityThreshold) {
            if (stuckTimer.seconds() > stuckTimeThreshold) {
                if (!isStuck) {
                    isStuck = true;
                    telemetry.addData("⚠️ WARNING", "Robot appears stuck!");
                }
            }
        } else {
            isStuck = false;
            stuckTimer.reset();
        }
    }

    /**
     * Detect wheel slippage (velocity doesn't match commanded motion)
     * Note: Requires commanded velocity to be set separately
     */
    private void detectSlippage(double actualVelX, double actualVelY) {
        // This is a placeholder - would need commanded velocity for comparison
        // For now, detect sudden velocity drops
        if (velocityMagnitude.size() >= 5) {
            double recentAvg = velocityMagnitude.getMovingAverage(5);
            double current = velocityMagnitude.getLast();

            if (recentAvg > 2.0 && current < recentAvg * 0.5) {
                isSlipping = true;
                telemetry.addData("⚠️ WARNING", "Possible wheel slip detected!");
            } else {
                isSlipping = false;
            }
        }
    }

    /**
     * Calculate motion smoothness score (0-100, higher is smoother)
     */
    public double getSmoothnessScore() {
        if (jerkMagnitude.size() < 10) return 100;

        double avgJerk = jerkMagnitude.getMean();
        double maxJerk = jerkMagnitude.getMax();

        // Lower jerk = smoother motion
        // Score: 100 - (jerk as percentage of threshold)
        double score = 100 - Math.min(100, (avgJerk / smoothnessThreshold) * 100);
        return Math.max(0, score);
    }

    /**
     * Calculate path efficiency (0-100, higher is better)
     * Ratio of straight-line distance to actual distance traveled
     */
    public double getPathEfficiency() {
        if (xPositions.size() < 2) return 100;

        double startX = xPositions.getFirst();
        double startY = yPositions.getFirst();
        double endX = xPositions.getLast();
        double endY = yPositions.getLast();

        double straightLineDistance = Math.hypot(endX - startX, endY - startY);

        if (totalDistance < 0.1) return 100;

        double efficiency = (straightLineDistance / totalDistance) * 100;
        return Math.min(100, Math.max(0, efficiency));
    }

    /**
     * Get current distance to target
     */
    public double getCurrentDistanceToTarget() {
        if (distanceToTarget.isEmpty()) return -1;
        return distanceToTarget.getLast();
    }

    /**
     * Get convergence rate (inches per second toward target)
     */
    public double getConvergenceRate() {
        if (distanceToTarget.size() < 10) return 0;

        double oldDist = distanceToTarget.get(distanceToTarget.size() - 10);
        double newDist = distanceToTarget.getLast();

        double deltaDistance = oldDist - newDist;  // Positive = converging
        double deltaTime = 10 * 0.02;  // Assuming ~50Hz update rate

        return deltaDistance / deltaTime;
    }

    /**
     * Check if robot is converging toward target
     */
    public boolean isConverging() {
        return isConverging;
    }

    /**
     * Check if robot is diverging from target
     */
    public boolean isDiverging() {
        return isDiverging;
    }

    /**
     * Check if robot is oscillating around target
     */
    public boolean isOscillating() {
        return isOscillating;
    }

    /**
     * Check if robot appears stuck
     */
    public boolean isStuck() {
        return isStuck;
    }

    /**
     * Check if wheel slip detected
     */
    public boolean isSlipping() {
        return isSlipping;
    }

    /**
     * Get current velocity magnitude
     */
    public double getCurrentVelocity() {
        if (velocityMagnitude.isEmpty()) return 0;
        return velocityMagnitude.getLast();
    }

    /**
     * Get average velocity over recent history
     */
    public double getAverageVelocity() {
        if (velocityMagnitude.isEmpty()) return 0;
        return velocityMagnitude.getMovingAverage(20);
    }

    /**
     * Get peak velocity achieved
     */
    public double getPeakVelocity() {
        return peakVelocity;
    }

    /**
     * Get current acceleration
     */
    public double getCurrentAcceleration() {
        if (accelerationMagnitude.isEmpty()) return 0;
        return accelerationMagnitude.getLast();
    }

    /**
     * Get average acceleration
     */
    public double getAverageAcceleration() {
        if (accelerationMagnitude.isEmpty()) return 0;
        return accelerationMagnitude.getMean();
    }

    /**
     * Get peak acceleration
     */
    public double getPeakAcceleration() {
        return peakAcceleration;
    }

    /**
     * Get total distance traveled
     */
    public double getTotalDistanceTraveled() {
        return totalDistance;
    }

    /**
     * Get heading stability (lower is more stable)
     * Standard deviation of recent heading changes
     */
    public double getHeadingStability() {
        if (headingError.size() < 10) return 0;
        return headingError.getStdDev();
    }

    /**
     * Get motion quality score (0-100, higher is better)
     * Composite score based on smoothness, efficiency, and stability
     */
    public double getMotionQualityScore() {
        double smoothness = getSmoothnessScore();
        double efficiency = getPathEfficiency();

        // Penalize for anomalies
        double anomalyPenalty = 0;
        if (isOscillating) anomalyPenalty += 20;
        if (isDiverging) anomalyPenalty += 30;
        if (isStuck) anomalyPenalty += 40;
        if (isSlipping) anomalyPenalty += 25;

        double score = (smoothness * 0.4 + efficiency * 0.6) - anomalyPenalty;
        return Math.max(0, Math.min(100, score));
    }

    /**
     * Get detailed motion analysis summary
     */
    public MotionAnalysisSummary getAnalysisSummary() {
        MotionAnalysisSummary summary = new MotionAnalysisSummary();

        summary.totalDistance = totalDistance;
        summary.avgVelocity = getAverageVelocity();
        summary.peakVelocity = peakVelocity;
        summary.avgAcceleration = getAverageAcceleration();
        summary.peakAcceleration = peakAcceleration;

        summary.smoothnessScore = getSmoothnessScore();
        summary.pathEfficiency = getPathEfficiency();
        summary.motionQuality = getMotionQualityScore();

        summary.isConverging = isConverging;
        summary.isDiverging = isDiverging;
        summary.isOscillating = isOscillating;
        summary.isStuck = isStuck;
        summary.isSlipping = isSlipping;

        if (targetX != null && targetY != null) {
            summary.distanceToTarget = getCurrentDistanceToTarget();
            summary.convergenceRate = getConvergenceRate();
        }

        summary.headingStability = getHeadingStability();

        return summary;
    }

    /**
     * Reset all tracking data
     */
    public void reset() {
        xPositions.clear();
        yPositions.clear();
        headings.clear();
        distanceToTarget.clear();
        headingError.clear();
        velocityX.clear();
        velocityY.clear();
        velocityMagnitude.clear();
        angularVelocity.clear();
        accelerationMagnitude.clear();
        angularAcceleration.clear();
        jerkMagnitude.clear();

        isConverging = false;
        isDiverging = false;
        isOscillating = false;
        isStuck = false;
        isSlipping = false;

        convergenceStreak = 0;
        divergenceStreak = 0;
        oscillationCount = 0;
        lastDistance = Double.MAX_VALUE;

        totalDistance = 0;
        peakVelocity = 0;
        peakAcceleration = 0;
        samplesCollected = 0;
    }

    /**
     * Add motion profiling telemetry
     */
    public void addTelemetry() {
        telemetry.addData("=== MOTION PROFILER ===", "");

        // Current state
        telemetry.addData("Velocity", "%.1f in/s (peak: %.1f)",
            getCurrentVelocity(), peakVelocity);
        telemetry.addData("Acceleration", "%.1f in/s² (peak: %.1f)",
            getCurrentAcceleration(), peakAcceleration);

        // Target convergence
        if (targetX != null && targetY != null) {
            String convergenceStatus = "STEADY";
            if (isConverging) convergenceStatus = "✓ CONVERGING";
            else if (isDiverging) convergenceStatus = "❌ DIVERGING";
            else if (isOscillating) convergenceStatus = "⚠️ OSCILLATING";

            telemetry.addData("Convergence", "%s (%.1f\"/s)",
                convergenceStatus, getConvergenceRate());
            telemetry.addData("Distance to Target", "%.1f in",
                getCurrentDistanceToTarget());
        }

        // Quality metrics
        telemetry.addData("Motion Quality", "%.0f/100", getMotionQualityScore());
        telemetry.addData("Smoothness", "%.0f/100", getSmoothnessScore());
        telemetry.addData("Path Efficiency", "%.0f%%", getPathEfficiency());

        // Anomalies
        if (isStuck) {
            telemetry.addData("STATUS", "⚠️ STUCK");
        }
        if (isSlipping) {
            telemetry.addData("STATUS", "⚠️ WHEEL SLIP");
        }

        // Statistics
        telemetry.addData("Distance Traveled", "%.1f in", totalDistance);
        telemetry.addData("Samples", "%d (%.1fs)",
            samplesCollected, samplesCollected * 0.02);
    }

    /**
     * Normalize angle to -180 to 180 range
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Motion analysis summary data class
     */
    public static class MotionAnalysisSummary {
        public double totalDistance;
        public double avgVelocity;
        public double peakVelocity;
        public double avgAcceleration;
        public double peakAcceleration;

        public double smoothnessScore;
        public double pathEfficiency;
        public double motionQuality;

        public boolean isConverging;
        public boolean isDiverging;
        public boolean isOscillating;
        public boolean isStuck;
        public boolean isSlipping;

        public double distanceToTarget;
        public double convergenceRate;
        public double headingStability;

        @Override
        public String toString() {
            return String.format(
                "Motion: Quality=%.0f/100, Smoothness=%.0f/100, Efficiency=%.0f%%, " +
                "Vel=%.1f in/s (peak %.1f), Dist=%.1f in",
                motionQuality, smoothnessScore, pathEfficiency,
                avgVelocity, peakVelocity, totalDistance
            );
        }
    }
}

