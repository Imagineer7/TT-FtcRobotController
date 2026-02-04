package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.v3.IndexingSystemV3;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;
import org.firstinspires.ftc.teamcode.util.debug.LogLevel;
import org.firstinspires.ftc.teamcode.util.debug.PerformanceMonitor;

/**
 * PerformanceTestOpMode - Measure loop timing and identify bottlenecks.
 * 
 * This OpMode runs IndexingSystemV3 with performance monitoring enabled
 * to measure loop times, subsystem update times, and identify latency sources.
 * 
 * Controls:
 * - dpad_up: Toggle performance monitoring on/off
 * - dpad_left/right: Switch telemetry pages
 * - start: Run for 2 minutes and log final stats
 * 
 * Test Protocol:
 * 1. Start OpMode with robot ready (all hardware connected)
 * 2. Enable performance monitoring (dpad_up)
 * 3. Let it run for 2 minutes (or press start)
 * 4. Check page 3 for performance stats (avg/max loop times)
 * 5. Note any spikes or sustained high latency
 * 
 * Expected Results (Before Optimization):
 * - Avg loop time: 30-50ms
 * - Max loop time: 80-150ms (with I2C saturation)
 * - Perception: 5-15ms (sensor reads)
 * - Operations: 1-5ms (state machine)
 * - FiringHelper: 3-10ms (shooter PID)
 * 
 * @author FTC 26581 Tundra Tech
 * @version 1.0
 */
@TeleOp(name="Performance Test", group="Testing")
public class PerformanceTestOpMode extends LinearOpMode {
    
    private AuroraHardwareConfig hardware;
    private Shooter shooter;
    private IndexingSystemV3 indexing;
    
    private boolean perfMonLastPressed = false;
    private boolean pageLastPressed = false;
    private boolean testRunning = false;
    private long testStartTime = 0;
    private static final long TEST_DURATION_MS = 120000;  // 2 minutes
    
    @Override
    public void runOpMode() {
        // Configure logging
        Dbg.setGlobalLevel(LogLevel.INFO);
        Dbg.setContext("PerfTest", "INIT");
        
        telemetry.addLine("=== PERFORMANCE TEST ===");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();
        
        // Initialize shooter
        shooter = new Shooter(hardware, new ShooterConfig(), telemetry);
        shooter.enable();
        
        // Initialize indexing system V3
        IndexingConfig config = new IndexingConfig();
        indexing = new IndexingSystemV3(hardware, config, shooter, telemetry);
        indexing.enable();
        
        // Enable performance monitoring by default
        indexing.setPerformanceMonitoringEnabled(true);
        
        // Configure hunt mode (test with auto-collection)
        indexing.setHuntEnabled(true);
        indexing.setSkipColorDetection(true);  // Fast mode
        
        telemetry.clear();
        telemetry.addLine("=== READY ===");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  dpad_up: Toggle perf monitoring");
        telemetry.addLine("  dpad_left/right: Switch pages");
        telemetry.addLine("  start: Run 2-minute test");
        telemetry.addLine();
        telemetry.addData("Perf Monitoring", "ENABLED");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();
        
        waitForStart();
        Dbg.setContext("PerfTest", "RUN");
        
        // Main loop
        while (opModeIsActive()) {
            // Handle controls
            handleControls();
            
            // Update indexing system (this is the critical loop)
            indexing.update();
            
            // Manual mode detection (none for this test - pure auto)
            indexing.setManualModeActive(false);
            
            // Watchdog trigger (no firing in this test)
            indexing.setFiringButtonHeld(false);
            
            // Check test duration
            if (testRunning) {
                long elapsed = System.currentTimeMillis() - testStartTime;
                if (elapsed >= TEST_DURATION_MS) {
                    // Test complete - log final stats
                    logFinalStats();
                    testRunning = false;
                }
            }
            
            // Display telemetry
            addCustomTelemetry();
            indexing.addTelemetry();
            telemetry.update();
        }
    }
    
    private void handleControls() {
        // Toggle performance monitoring
        boolean perfMonPressed = gamepad1.dpad_up;
        if (perfMonPressed && !perfMonLastPressed) {
            boolean currentState = indexing.isPerformanceMonitoringEnabled();
            indexing.setPerformanceMonitoringEnabled(!currentState);
            telemetry.addLine("Perf monitoring: " + (!currentState ? "ON" : "OFF"));
        }
        perfMonLastPressed = perfMonPressed;
        
        // Switch telemetry pages
        boolean pageLeftPressed = gamepad1.dpad_left;
        boolean pageRightPressed = gamepad1.dpad_right;
        if ((pageLeftPressed || pageRightPressed) && !pageLastPressed) {
            indexing.nextTelemetryPage();
        }
        pageLastPressed = pageLeftPressed || pageRightPressed;
        
        // Start/stop 2-minute test
        if (gamepad1.start && !testRunning) {
            testRunning = true;
            testStartTime = System.currentTimeMillis();
            indexing.getPerformanceMonitor().reset();
            Dbg.i(LogGroup.SYSTEM, "Performance test started (2 minutes)");
        }
    }
    
    private void addCustomTelemetry() {
        telemetry.addLine("=== PERFORMANCE TEST ===");
        telemetry.addData("Perf Mon", indexing.isPerformanceMonitoringEnabled() ? "✓" : "✗");
        
        if (testRunning) {
            long elapsed = System.currentTimeMillis() - testStartTime;
            long remaining = (TEST_DURATION_MS - elapsed) / 1000;
            telemetry.addData("Test Timer", remaining + "s remaining");
        }
        
        telemetry.addLine();
    }
    
    private void logFinalStats() {
        Dbg.i(LogGroup.SYSTEM, "=== PERFORMANCE TEST COMPLETE ===");
        Dbg.i(LogGroup.SYSTEM, "Duration: 2 minutes");
        
        PerformanceMonitor perf = indexing.getPerformanceMonitor();
        Dbg.i(LogGroup.SYSTEM, "Loop Time: avg=%.1fms max=%.1fms", 
            perf.getLoopAvgMs(), perf.getLoopMaxMs());
        Dbg.i(LogGroup.SYSTEM, "Loop Period: avg=%.1fms", 
            perf.getLoopPeriodAvgMs());
        
        Dbg.i(LogGroup.SYSTEM, "Sections:");
        Dbg.i(LogGroup.SYSTEM, "  perception: avg=%.1fms", 
            perf.getSectionAvgMs("perception"));
        Dbg.i(LogGroup.SYSTEM, "  indexingHelper: avg=%.1fms", 
            perf.getSectionAvgMs("indexingHelper"));
        Dbg.i(LogGroup.SYSTEM, "  firingHelper: avg=%.1fms", 
            perf.getSectionAvgMs("firingHelper"));
        Dbg.i(LogGroup.SYSTEM, "  operations: avg=%.1fms", 
            perf.getSectionAvgMs("operations"));
        Dbg.i(LogGroup.SYSTEM, "  shotPlanner: avg=%.1fms", 
            perf.getSectionAvgMs("shotPlanner"));
        
        telemetry.addLine();
        telemetry.addLine("=== TEST COMPLETE ===");
        telemetry.addLine("Check logs for detailed stats");
    }
}
