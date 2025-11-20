/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * PanelsDashboardHelper - Unified dashboard interface for Panels telemetry and field visualization
 * 
 * This helper class provides a clean API for sending telemetry and drawing on the field view
 * without needing to understand Panels internals.
 */

package org.firstinspires.ftc.teamcode.dashboard;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * PanelsDashboardHelper - Centralized dashboard interface
 * 
 * Purpose:
 * - Wrap Panels telemetry and field visualization APIs
 * - Provide simple methods for common tasks (log data, draw robot, draw paths)
 * - Handle coordinate system conversions
 * - Fall back to standard SDK telemetry if Panels unavailable
 * 
 * Why use this?
 * - One API for both Driver Station and Panels dashboard
 * - Easy to add telemetry without learning Panels internals
 * - Automatic fallback if Panels isn't running
 * - Consistent coordinate system handling
 * 
 * How to use:
 * 1. Create instance in OpMode init:
 *    PanelsDashboardHelper dashboard = new PanelsDashboardHelper(telemetry);
 * 
 * 2. Log telemetry in loop:
 *    dashboard.log("X Position", pose.getX());
 *    dashboard.log("Y Position", pose.getY());
 * 
 * 3. Draw robot on field:
 *    dashboard.setRobotPose(x, y, heading);
 * 
 * 4. Update at end of loop:
 *    dashboard.update();
 */
public class PanelsDashboardHelper {
    
    // ═══════════════════════════════════════════════════════════════
    // TELEMETRY MANAGERS
    // ═══════════════════════════════════════════════════════════════
    
    private final Telemetry sdkTelemetry;      // Standard Driver Station telemetry
    private TelemetryManager panelsTelemetry;  // Panels telemetry (may be null)
    private FieldManager panelsField;          // Panels field view (may be null)
    
    private boolean panelsAvailable = false;   // Track if Panels is working
    
    // ═══════════════════════════════════════════════════════════════
    // FIELD VISUALIZATION STYLES
    // ═══════════════════════════════════════════════════════════════
    
    // Robot marker style (green circle)
    private static final Style ROBOT_STYLE = new Style("Robot", "#00FF00", 0.75);
    private static final double ROBOT_RADIUS = 9.0;  // Inches
    
    // Path style (blue line)
    private static final Style PATH_STYLE = new Style("Path", "#3F51B5", 0.75);
    
    // Target/waypoint style (red circle)
    private static final Style TARGET_STYLE = new Style("Target", "#FF0000", 0.75);
    
    // ═══════════════════════════════════════════════════════════════
    // COORDINATE SYSTEM
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * FTC/Pedro Coordinate System (Right-Hand System):
     * 
     * Origin (0, 0):
     * - Located at field center
     * 
     * Axes:
     * - +X points toward red alliance wall
     * - +Y points toward blue audience side
     * - +Heading (rotation) is counterclockwise
     *   - 0° = facing +X direction (toward red wall)
     *   - 90° = facing +Y direction (toward blue audience)
     *   - 180° = facing -X direction (toward blue wall)
     *   - 270° = facing -Y direction (toward red audience)
     * 
     * Field Dimensions:
     * - Width: 144 inches (12 feet)
     * - Height: 144 inches (12 feet)
     * - X range: -72 to +72 inches
     * - Y range: -72 to +72 inches
     * 
     * Panels Field View:
     * - Uses same coordinate system
     * - Automatically scales for field size
     * - Rotation handled correctly by Panels
     */
    
    // ═══════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Initialize dashboard helper
     * 
     * Attempts to connect to Panels dashboard. If Panels is not running or
     * fails to initialize, gracefully falls back to SDK telemetry only.
     * 
     * @param sdkTelemetry Telemetry object from OpMode
     */
    public PanelsDashboardHelper(Telemetry sdkTelemetry) {
        this.sdkTelemetry = sdkTelemetry;
        
        try {
            // Attempt to get Panels telemetry instance
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
            
            // Attempt to get Panels field manager
            panelsField = PanelsField.INSTANCE.getField();
            
            // Set field offsets for Pedro Pathing coordinate system
            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            
            panelsAvailable = true;
            sdkTelemetry.addLine("✅ Panels dashboard connected");
            sdkTelemetry.addLine("   Access at: http://192.168.43.1:8080");
            
        } catch (Exception e) {
            // Panels not available - fall back to SDK telemetry only
            panelsAvailable = false;
            panelsTelemetry = null;
            panelsField = null;
            
            sdkTelemetry.addLine("⚠️  Panels dashboard not available");
            sdkTelemetry.addLine("   Using Driver Station telemetry only");
        }
    }
    
    // ═══════════════════════════════════════════════════════════════
    // TELEMETRY METHODS
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Log a key-value pair to both SDK and Panels telemetry
     * 
     * @param caption Label for the data
     * @param value Value to display
     */
    public void log(String caption, Object value) {
        // Always log to SDK telemetry
        sdkTelemetry.addData(caption, value);
        
        // Log to Panels if available
        if (panelsAvailable && panelsTelemetry != null) {
            try {
                panelsTelemetry.debug(caption, value);
            } catch (Exception e) {
                // Panels failed - disable it
                panelsAvailable = false;
            }
        }
    }
    
    /**
     * Log a formatted key-value pair
     * 
     * @param caption Label for the data
     * @param format Format string (e.g., "%.2f")
     * @param args Arguments for format string
     */
    public void log(String caption, String format, Object... args) {
        String formatted = String.format(format, args);
        log(caption, formatted);
    }
    
    /**
     * Log a line of text (no key-value pair)
     * 
     * @param text Text to display
     */
    public void logLine(String text) {
        sdkTelemetry.addLine(text);
        
        if (panelsAvailable && panelsTelemetry != null) {
            try {
                panelsTelemetry.debug("", text);
            } catch (Exception e) {
                panelsAvailable = false;
            }
        }
    }
    
    /**
     * Get the raw SDK telemetry object
     * 
     * Use this for direct telemetry calls that don't need Panels mirroring
     * 
     * @return SDK Telemetry object
     */
    public Telemetry getSdkTelemetry() {
        return sdkTelemetry;
    }
    
    // ═══════════════════════════════════════════════════════════════
    // FIELD VISUALIZATION METHODS
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Draw robot position on field view
     * 
     * This displays the robot as a circle at its current position.
     * The circle is oriented to show heading direction.
     * 
     * @param xInches X position in inches (field coordinate system)
     * @param yInches Y position in inches (field coordinate system)
     * @param headingDeg Heading in degrees (0° = facing +X)
     */
    public void setRobotPose(double xInches, double yInches, double headingDeg) {
        if (!panelsAvailable || panelsField == null) {
            return;  // Panels not available
        }
        
        try {
            // Draw robot as a circle
            panelsField.drawCircle(xInches, yInches, ROBOT_RADIUS, ROBOT_STYLE);
            
            // Draw heading indicator (line from center to edge)
            double headingRad = Math.toRadians(headingDeg);
            double endX = xInches + ROBOT_RADIUS * Math.cos(headingRad);
            double endY = yInches + ROBOT_RADIUS * Math.sin(headingRad);
            panelsField.drawLine(xInches, yInches, endX, endY, ROBOT_STYLE);
            
        } catch (Exception e) {
            panelsAvailable = false;
        }
    }
    
    /**
     * Draw robot position from Pedro Pose object
     * 
     * Convenience method that extracts position and heading from Pose.
     * 
     * @param pose Pedro Pose object (x, y, heading in radians)
     */
    public void setRobotPose(Pose pose) {
        double headingDeg = Math.toDegrees(pose.getHeading());
        setRobotPose(pose.getX(), pose.getY(), headingDeg);
    }
    
    /**
     * Draw a path on the field view
     * 
     * This displays a planned or active path as a series of line segments.
     * 
     * @param waypoints List of (x, y) positions along the path
     */
    public void drawPath(List<double[]> waypoints) {
        if (!panelsAvailable || panelsField == null || waypoints == null || waypoints.size() < 2) {
            return;
        }
        
        try {
            // Draw line segments between consecutive waypoints
            for (int i = 0; i < waypoints.size() - 1; i++) {
                double[] start = waypoints.get(i);
                double[] end = waypoints.get(i + 1);
                
                panelsField.drawLine(start[0], start[1], end[0], end[1], PATH_STYLE);
            }
        } catch (Exception e) {
            panelsAvailable = false;
        }
    }
    
    /**
     * Draw a target position on the field
     * 
     * Useful for showing goal positions or intermediate waypoints.
     * 
     * @param xInches X position in inches
     * @param yInches Y position in inches
     * @param radius Radius of target marker
     */
    public void drawTarget(double xInches, double yInches, double radius) {
        if (!panelsAvailable || panelsField == null) {
            return;
        }
        
        try {
            panelsField.drawCircle(xInches, yInches, radius, TARGET_STYLE);
        } catch (Exception e) {
            panelsAvailable = false;
        }
    }
    
    /**
     * Clear all field overlays
     * 
     * Removes all robot markers, paths, and targets from field view.
     * Call this before drawing updated positions.
     */
    public void clearFieldOverlays() {
        if (!panelsAvailable || panelsField == null) {
            return;
        }
        
        try {
            panelsField.clearDrawings();
        } catch (Exception e) {
            panelsAvailable = false;
        }
    }
    
    // ═══════════════════════════════════════════════════════════════
    // UPDATE METHOD
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Update telemetry and field view
     * 
     * IMPORTANT: Call this at the end of every loop iteration!
     * 
     * This flushes all queued telemetry to both Driver Station and Panels.
     */
    public void update() {
        // Update SDK telemetry (always)
        sdkTelemetry.update();
        
        // Update Panels telemetry if available
        if (panelsAvailable && panelsTelemetry != null) {
            try {
                panelsTelemetry.update(sdkTelemetry);
            } catch (Exception e) {
                panelsAvailable = false;
            }
        }
    }
    
    // ═══════════════════════════════════════════════════════════════
    // STATUS METHODS
    // ═══════════════════════════════════════════════════════════════
    
    /**
     * Check if Panels dashboard is available
     * 
     * @return true if Panels is connected and working
     */
    public boolean isPanelsAvailable() {
        return panelsAvailable;
    }
    
    /**
     * Get Panels dashboard URL
     * 
     * @return URL string or null if Panels not available
     */
    public String getPanelsUrl() {
        if (panelsAvailable) {
            return "http://192.168.43.1:8080";
        }
        return null;
    }
}
