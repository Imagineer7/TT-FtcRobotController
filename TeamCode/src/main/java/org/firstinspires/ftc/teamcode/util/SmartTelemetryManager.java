/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Smart Telemetry Manager - Efficient telemetry display system with focused pages
 */

package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SmartMechanumDrive;

/**
 * SmartTelemetryManager - Manages efficient telemetry display with multiple focused pages
 *
 * Features:
 * - Multiple telemetry pages focused on specific systems
 * - Always-visible critical information
 * - Efficient display to reduce lag
 * - Smart cycling between pages
 */
public class SmartTelemetryManager {

    public enum TelemetryPage {
        OVERVIEW("📊 Overview", "System status & critical info"),
        DRIVE("🚗 Drive", "Movement & navigation systems"),
        SHOOTER("🎯 Shooter", "Shooting system & performance"),
        RECORDING("🎬 Recording", "Movement recording & playback"),
        PERFORMANCE("⚡ Performance", "System metrics & diagnostics"),
        CONTROLS("🎮 Controls", "Control mapping & help");

        private final String displayName;
        private final String description;

        TelemetryPage(String displayName, String description) {
            this.displayName = displayName;
            this.description = description;
        }

        public String getDisplayName() { return displayName; }
        public String getDescription() { return description; }
    }

    private final Telemetry telemetry;
    private final AuroraManager robotManager;

    private TelemetryPage currentPage = TelemetryPage.OVERVIEW;
    private boolean prevPageToggle = false;
    private long lastUpdateTime = 0;
    private static final long UPDATE_INTERVAL_MS = 100; // 10Hz telemetry update rate

    // Critical status cache (always displayed)
    private String batteryStatus = "";
    private String systemHealth = "";
    private String emergencyStatus = "";

    public SmartTelemetryManager(Telemetry telemetry, AuroraManager robotManager) {
        this.telemetry = telemetry;
        this.robotManager = robotManager;
    }

    /**
     * Handle page cycling input
     */
    public void handlePageCycling(boolean button) {
        if (button && !prevPageToggle) {
            cyclePage();
        }
        prevPageToggle = button;
    }

    /**
     * Cycle to the next telemetry page
     */
    private void cyclePage() {
        TelemetryPage[] pages = TelemetryPage.values();
        int currentIndex = currentPage.ordinal();
        currentPage = pages[(currentIndex + 1) % pages.length];
    }

    /**
     * Update telemetry display (call this every loop)
     */
    public void updateDisplay(boolean emergencyStop) {
        long currentTime = System.currentTimeMillis();

        // Rate limit telemetry updates to improve performance
        if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) {
            return;
        }
        lastUpdateTime = currentTime;

        // Update critical status cache
        updateCriticalStatus(emergencyStop);

        telemetry.clear();

        // Always show critical header
        showCriticalHeader();

        // Show current page content
        switch (currentPage) {
            case OVERVIEW:
                showOverviewPage();
                break;
            case DRIVE:
                showDrivePage();
                break;
            case SHOOTER:
                showShooterPage();
                break;
            case PERFORMANCE:
                showPerformancePage();
                break;
            case CONTROLS:
                showControlsPage();
                break;
            case RECORDING:
                showRecordingPage();
                break;
        }

        // Show page navigation footer
        showNavigationFooter();

        telemetry.update();
    }

    /**
     * Update critical status information
     */
    private void updateCriticalStatus(boolean emergencyStop) {
        if (emergencyStop) {
            emergencyStatus = "⛔ EMERGENCY STOP";
            return;
        } else {
            emergencyStatus = "";
        }

        // Battery status
        double voltage = robotManager.getDriveSystem() != null ?
            robotManager.getDriveSystem().getCurrentVoltage() : 12.0;
        String batteryIcon = voltage > 11.5 ? "🔋" : voltage > 10.5 ? "⚠️" : "🪫";
        batteryStatus = String.format("%s %.1fV", batteryIcon, voltage);

        // System health
        systemHealth = robotManager.isSystemsHealthy() ? "✅" : "⚠️";
    }

    /**
     * Show critical information header (always visible)
     */
    private void showCriticalHeader() {
        if (!emergencyStatus.isEmpty()) {
            telemetry.addLine(emergencyStatus);
            telemetry.addLine("Press Back to cancel");
            return;
        }

        telemetry.addLine("=== AURORA SMART TELEMETRY ===");
        telemetry.addData("Page", currentPage.getDisplayName());
        telemetry.addData("Battery", batteryStatus);
        telemetry.addData("Health", systemHealth);
        telemetry.addData("Mode", "🎮 " + robotManager.getDriverModeString());
        telemetry.addLine("");
    }

    /**
     * Overview page - System status summary
     */
    private void showOverviewPage() {
        telemetry.addLine("🔧 SYSTEM STATUS");

        // Drive system summary
        if (robotManager.getDriveSystem() != null) {
            telemetry.addData("Drive Mode", robotManager.getDriveSystem().getCurrentMode().getName());
            telemetry.addData("Field Relative", robotManager.getDriveSystem().isFieldRelative() ? "🧭 ON" : "🚗 OFF");
        }

        // Shooter system summary
        if (robotManager.getShooterSystem() != null) {
            String shooterStatus = robotManager.getShooterSystem().isShooterRunning() ? "🔥 ACTIVE" : "⏸ IDLE";
            if (robotManager.getShooterSystem().isShooting()) {
                shooterStatus = "💥 FIRING";
            }
            telemetry.addData("Shooter", shooterStatus);

            if (robotManager.getShooterSystem().isShooterRunning()) {
                telemetry.addData("RPM", "%.0f", robotManager.getShooterSystem().getCurrentRPM());
            }
        }

        // Quick performance metrics
        if (robotManager.getGlobalMonitor() != null) {
            telemetry.addData("Loop Time", "%.1f ms", robotManager.getGlobalMonitor().getAverageLoopTime());
            telemetry.addData("Shot Accuracy", "🎯 %.1f%%", robotManager.getGlobalMonitor().getAccuracy());
        }

        telemetry.addLine("");
        telemetry.addLine("📋 Quick status overview");
        telemetry.addLine("Use X button to cycle pages");
    }

    /**
     * Drive page - Movement and navigation details
     */
    private void showDrivePage() {
        telemetry.addLine("🚗 DRIVE SYSTEM");

        if (robotManager.getDriveSystem() != null) {
            SmartMechanumDrive.DriveSystemData driveData = robotManager.getDriveSystem().getTelemetryData();

            telemetry.addData("Current Mode", driveData.driveMode);
            telemetry.addData("Field Relative", driveData.fieldRelative ? "🧭 ENABLED" : "🚗 DISABLED");
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotManager.getDriveSystem().getCurrentHeading()));

            telemetry.addLine("");
            telemetry.addLine("🔧 MOTOR STATUS");
            telemetry.addData("Voltage", "%.1fV", driveData.voltage);

            if (driveData.lowBattery) {
                telemetry.addLine("⚠️ LOW BATTERY - Consider replacement");
            }

            telemetry.addLine("");
            telemetry.addLine("⚡ PERFORMANCE");
            telemetry.addData("Distance", "%.1fm", driveData.totalDistance);
            telemetry.addData("Efficiency", "%.2f m/energy", driveData.efficiency);
            telemetry.addData("Sharp Turns", "%d", driveData.sharpTurns);
            telemetry.addData("Avg Speed", "%.1f", driveData.averageSpeed);

            telemetry.addLine("");
            telemetry.addLine("🎮 MOTOR POWERS");
            telemetry.addData("LF/RF", "%.2f / %.2f", driveData.motorPowers[0], driveData.motorPowers[1]);
            telemetry.addData("LB/RB", "%.2f / %.2f", driveData.motorPowers[2], driveData.motorPowers[3]);

        } else {
            telemetry.addLine("❌ Drive system not available");
        }

        telemetry.addLine("");
        telemetry.addLine("Movement & navigation data");
    }

    /**
     * Shooter page - Shooting system details
     */
    private void showShooterPage() {
        telemetry.addLine("🎯 SHOOTER SYSTEM");

        if (robotManager.getShooterSystem() != null) {
            EnhancedDecodeHelper shooter = robotManager.getShooterSystem();

            // Current state
            String state = "⏸ IDLE";
            if (shooter.isShooting()) {
                state = "💥 FIRING";
            } else if (shooter.isShooterRunning()) {
                state = "🔥 SPINNING";
            }
            telemetry.addData("Status", state);

            // RPM information
            if (shooter.isShooterRunning()) {
                telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
                telemetry.addData("Target RPM", "%.0f", shooter.getConfig().getTargetRPM());

                // RPM accuracy
                double accuracy = (shooter.getCurrentRPM() / shooter.getConfig().getTargetRPM()) * 100;
                String accuracyIcon = accuracy > 95 ? "✅" : accuracy > 85 ? "⚠️" : "❌";
                telemetry.addData("RPM Accuracy", "%s %.1f%%", accuracyIcon, accuracy);
            }

            telemetry.addLine("");
            telemetry.addLine("🎯 PERFORMANCE");

            // Shooting statistics if available
            if (shooter.getMonitor() != null) {
                PerformanceMonitor.PerformanceData perfData = shooter.getMonitor().getTelemetryData();
                telemetry.addData("Total Shots", "%d", perfData.totalShots);
                telemetry.addData("Shot Accuracy", "%.1f%%", perfData.accuracy);
                telemetry.addData("Avg Spinup", "%.1fs", perfData.averageSpinupTime);
            }

        } else {
            telemetry.addLine("❌ Shooter system not available");
        }

        telemetry.addLine("");
        telemetry.addLine("Shooting system metrics");
    }

    /**
     * Performance page - System diagnostics and metrics
     */
    private void showPerformancePage() {
        telemetry.addLine("⚡ SYSTEM PERFORMANCE");

        if (robotManager.getGlobalMonitor() != null) {
            PerformanceMonitor.PerformanceData perfData = robotManager.getGlobalMonitor().getTelemetryData();

            // Loop performance
            telemetry.addData("Loop Time", "%.1f ms (avg)", perfData.averageLoopTime);
            telemetry.addData("Max Loop Time", "%.1f ms", perfData.maxLoopTime);

            // Performance grade
            String grade = robotManager.getGlobalMonitor().getPerformanceGrade();
            telemetry.addData("Performance", grade);

            telemetry.addLine("");
            telemetry.addLine("🎯 ACCURACY METRICS");
            telemetry.addData("Shot Accuracy", "%.1f%%", perfData.accuracy);
            telemetry.addData("Successful Shots", "%d/%d", perfData.successfulShots, perfData.totalShots);

            telemetry.addLine("");
            telemetry.addLine("🔋 POWER METRICS");
            telemetry.addData("Current Voltage", "%.1fV", perfData.currentVoltage);
            telemetry.addData("Voltage Drop", "%.1f%%", perfData.voltageDropPercentage);
            telemetry.addData("Peak RPM", "%.0f", perfData.peakRPM);
            telemetry.addData("Average RPM", "%.0f", perfData.averageRPM);

            if (perfData.performanceDegraded) {
                telemetry.addLine("");
                telemetry.addLine("⚠️ PERFORMANCE DEGRADED");
                if (!perfData.recommendations.isEmpty()) {
                    telemetry.addLine("Recommendations:");
                    String[] lines = perfData.recommendations.split("\n");
                    for (String line : lines) {
                        if (!line.trim().isEmpty()) {
                            telemetry.addLine(line);
                        }
                    }
                }
            }

        } else {
            telemetry.addLine("❌ Performance monitor not available");
        }

        telemetry.addLine("");
        telemetry.addLine("Detailed performance metrics");
        telemetry.addLine("Press B (GP2) to reset stats");
    }

    /**
     * Controls page - Control mapping and help
     */
    private void showControlsPage() {
        telemetry.addLine("🎮 CONTROL MAPPING");

        if (robotManager.getDriverMode() == AuroraManager.DriverMode.DUAL_DRIVER) {
            telemetry.addLine("");
            telemetry.addLine("👥 DUAL DRIVER MODE");
            telemetry.addLine("");
            telemetry.addLine("🎮 GAMEPAD 1 (Driver):");
            telemetry.addLine("• Left stick: Forward/Strafe");
            telemetry.addLine("• Right stick: Rotate");
            telemetry.addLine("• D-pad: Fine movement");
            telemetry.addLine("• Bumpers: Fine rotation");
            telemetry.addLine("• LSB: Precision mode");
            telemetry.addLine("• X: Field relative toggle");
            telemetry.addLine("");
            telemetry.addLine("🎮 GAMEPAD 2 (Operator):");
            telemetry.addLine("• A: Single shot");
            telemetry.addLine("• Y: Continuous fire");
            telemetry.addLine("• B: Feed control");
            telemetry.addLine("• X: Emergency stop");
            telemetry.addLine("• Bumpers: Range select");
        } else {
            telemetry.addLine("");
            telemetry.addLine("👤 SINGLE DRIVER MODE");
            telemetry.addLine("");
            telemetry.addLine("🎮 GAMEPAD 1 (All Controls):");
            telemetry.addLine("• Movement + Shooting");
            telemetry.addLine("• Semi-auto functions");
            telemetry.addLine("");
            telemetry.addLine("🎮 GAMEPAD 2 (Backup):");
            telemetry.addLine("• Diagnostics & stats");
        }

        telemetry.addLine("");
        telemetry.addLine("🔄 SHARED CONTROLS");
        telemetry.addLine("• X (GP1): Cycle telemetry pages");
        telemetry.addLine("• Back + D-pad Left: Toggle mode");
        telemetry.addLine("• LSB + RSB: Field relative toggle");
        telemetry.addLine("• Start + Start: Emergency stop");
    }

    /**
     * Recording page - Movement recording and playback
     */
    private void showRecordingPage() {
        telemetry.addLine("🎬 MOVEMENT RECORDING");

        if (robotManager.getMovementRecorder() != null) {
            MovementRecorder.RecordingTelemetryData recordingData = robotManager.getMovementRecorder().getTelemetryData();

            // Recording status
            if (recordingData.isRecording) {
                String status = recordingData.isPaused ? "⏸ PAUSED" : "🔴 RECORDING";
                telemetry.addData("Status", status);
                telemetry.addData("Recording", recordingData.currentRecording);
                telemetry.addData("Duration", String.format("%.1fs", recordingData.duration));
                telemetry.addData("Waypoints", recordingData.waypointCount);
                telemetry.addData("Rate", String.format("%.0f Hz", recordingData.waypointCount / Math.max(recordingData.duration, 0.1)));
            } else {
                telemetry.addData("Status", "⏹ STOPPED");
                telemetry.addData("Available", String.format("%d recordings", recordingData.availableRecordings));
            }

            telemetry.addLine("");
            telemetry.addLine("🎮 RECORDING CONTROLS");
            telemetry.addLine("• Y: Start/Stop Recording");
            telemetry.addLine("• X: Pause/Resume");
            telemetry.addLine("• B: Start Over");
            telemetry.addLine("• A: Quick Save");

        } else {
            telemetry.addLine("❌ Recording system not available");
        }

        telemetry.addLine("");
        telemetry.addLine("Movement recording & playback");
    }

    /**
     * Show navigation footer
     */
    private void showNavigationFooter() {
        telemetry.addLine("");
        telemetry.addLine("═══════════════════════════");

        // Show available pages with indicators
        StringBuilder pageIndicator = new StringBuilder("Pages: ");
        for (TelemetryPage page : TelemetryPage.values()) {
            if (page == currentPage) {
                pageIndicator.append("[").append(page.getDisplayName()).append("] ");
            } else {
                pageIndicator.append(page.getDisplayName()).append(" ");
            }
        }

        telemetry.addLine(pageIndicator.toString());
        telemetry.addLine("X (GP1) to cycle →");
    }

    /**
     * Get current page
     */
    public TelemetryPage getCurrentPage() {
        return currentPage;
    }

    /**
     * Set current page (for external control)
     */
    public void setCurrentPage(TelemetryPage page) {
        this.currentPage = page;
    }

    /**
     * Quick access to specific pages
     */
    public void showOverview() { currentPage = TelemetryPage.OVERVIEW; }
    public void showDrive() { currentPage = TelemetryPage.DRIVE; }
    public void showShooter() { currentPage = TelemetryPage.SHOOTER; }
    public void showPerformance() { currentPage = TelemetryPage.PERFORMANCE; }
    public void showControls() { currentPage = TelemetryPage.CONTROLS; }
    public void showRecording() { currentPage = TelemetryPage.RECORDING; }

    /**
     * Simple update method for compatibility
     */
    public void update() {
        updateDisplay(false);
    }
}
