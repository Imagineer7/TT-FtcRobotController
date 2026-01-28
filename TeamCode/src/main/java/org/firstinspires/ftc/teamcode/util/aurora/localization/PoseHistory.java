package org.firstinspires.ftc.teamcode.util.aurora.localization;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Ring buffer for storing historical robot poses.
 * Used for latency compensation in vision fusion.
 * 
 * Features:
 * - Fixed-size circular buffer
 * - Automatic removal of old poses
 * - Timestamp-based retrieval
 * - Linear interpolation for exact timestamps
 */
public class PoseHistory {
    
    private final Deque<RobotPose2D> buffer;
    private final long maxAge;  // milliseconds
    private final int maxSize;
    
    /**
     * Create pose history buffer
     * @param maxAge Maximum age of poses to keep (milliseconds)
     */
    public PoseHistory(long maxAge) {
        this(maxAge, 100);  // Default max 100 poses
    }
    
    /**
     * Create pose history buffer with size limit
     * @param maxAge Maximum age of poses to keep (milliseconds)
     * @param maxSize Maximum number of poses to store
     */
    public PoseHistory(long maxAge, int maxSize) {
        this.buffer = new ArrayDeque<>(maxSize);
        this.maxAge = maxAge;
        this.maxSize = maxSize;
    }
    
    /**
     * Add a pose to the history
     * Automatically removes poses that are too old or exceed size limit
     */
    public void add(RobotPose2D pose) {
        // Add new pose
        buffer.addLast(pose);
        
        // Remove old poses
        long currentTime = System.currentTimeMillis();
        while (!buffer.isEmpty() && 
               (currentTime - buffer.peekFirst().timestamp > maxAge)) {
            buffer.removeFirst();
        }
        
        // Enforce size limit
        while (buffer.size() > maxSize) {
            buffer.removeFirst();
        }
    }
    
    /**
     * Get pose at specific timestamp
     * If exact timestamp not found, interpolates between nearest poses
     * 
     * @param timestamp Desired timestamp (milliseconds)
     * @return Interpolated pose, or null if timestamp out of range
     */
    public RobotPose2D get(long timestamp) {
        if (buffer.isEmpty()) {
            return null;
        }
        
        // Check if timestamp is in range
        long oldestTime = buffer.peekFirst().timestamp;
        long newestTime = buffer.peekLast().timestamp;
        
        if (timestamp < oldestTime || timestamp > newestTime) {
            return null;  // Out of range
        }
        
        // Find bracketing poses
        RobotPose2D before = null;
        RobotPose2D after = null;
        
        for (RobotPose2D pose : buffer) {
            if (pose.timestamp <= timestamp) {
                before = pose;
            }
            if (pose.timestamp >= timestamp) {
                after = pose;
                break;
            }
        }
        
        // Exact match
        if (before != null && before.timestamp == timestamp) {
            return before.copy();
        }
        if (after != null && after.timestamp == timestamp) {
            return after.copy();
        }
        
        // Interpolate
        if (before != null && after != null) {
            long dt = after.timestamp - before.timestamp;
            if (dt == 0) {
                return before.copy();
            }
            
            double alpha = (double)(timestamp - before.timestamp) / dt;
            return before.interpolate(after, alpha);
        }
        
        // Fallback to nearest
        if (before != null) {
            return before.copy();
        }
        if (after != null) {
            return after.copy();
        }
        
        return null;
    }
    
    /**
     * Get the most recent pose
     */
    public RobotPose2D getLatest() {
        if (buffer.isEmpty()) {
            return null;
        }
        return buffer.peekLast().copy();
    }
    
    /**
     * Get the oldest pose
     */
    public RobotPose2D getOldest() {
        if (buffer.isEmpty()) {
            return null;
        }
        return buffer.peekFirst().copy();
    }
    
    /**
     * Get number of poses in buffer
     */
    public int size() {
        return buffer.size();
    }
    
    /**
     * Check if buffer is empty
     */
    public boolean isEmpty() {
        return buffer.isEmpty();
    }
    
    /**
     * Clear all poses from buffer
     */
    public void clear() {
        buffer.clear();
    }
    
    /**
     * Get age span of buffer (newest - oldest timestamp)
     */
    public long getAgeSpan() {
        if (buffer.size() < 2) {
            return 0;
        }
        return buffer.peekLast().timestamp - buffer.peekFirst().timestamp;
    }
}
