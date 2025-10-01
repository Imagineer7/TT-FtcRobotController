/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Template for DECODE season autonomous with shooting
 * Copy this file and modify for your specific autonomous routine
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.AutoHelper;
import org.firstinspires.ftc.teamcode.util.DecodeHelper;

/**
 * Simple Auto for DECODE autonomous with integrated shooting
 */
@Autonomous(name="Simple Auto Decode", group="Templates")
public class SimpleAutoDecode extends LinearOpMode {
    
    private AutoHelper autoHelper;
    private DecodeHelper decodeHelper;
    
    @Override
    public void runOpMode() {
        // Initialize both helpers
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        decodeHelper = new DecodeHelper(hardwareMap, telemetry);
        
        // Configure AutoHelper for your robot
        autoHelper.initialize(null, null);  // Change camera names as needed
        autoHelper.getPositioningHelper().resetPosition(-12, -52.5, 90);
        //autoHelper.setRelocalizationEnabled(true);      // Use AprilTag correction
        autoHelper.setStuckDetectionEnabled(true);      // Handle getting stuck
        autoHelper.setDefaultParameters(0.4, 5000);     // 40% power, 5s timeout
        
        telemetry.addData("Status", "Ready to start autonomous");
        telemetry.update();
        
        waitForStart();
        
        // =================================================================
        // AUTONOMOUS SEQUENCE - Modify this section for your strategy
        // =================================================================
        
        // Auto Helper Sequence Setup
        autoHelper
            // Drive off the start: forward 18", hold heading
            .forward(24, "Push off start")
            // Strafe right 12" (robot frame)
            .strafeRight(1, "Align")
            // Move 24" toward the audience wall (field +X = 90°) but keep current heading
            //.moveInchesField(24, /*fieldDirDeg=*/90, "Translate toward audience")
            // Face basket (45°), then nudge forward 6" in robot frame
            .turnTo(180, "Face goal")
            .addStep("Start shooter", decodeHelper.createStartShooterAction())
            .waitFor(1200, "Wait for shooter to spin up")
            .addStep("Fire 3 artifacts", decodeHelper.createShootAction(3, false))
            //.moveInchesRobot(6, /*robotDirDeg=*/0, /*power=*/0.5, /*timeoutMs=*/2000, /*holdHeading=*/true, "Nudge to score")
            .executeAll();
        
        // =================================================================
        // END OF AUTONOMOUS SEQUENCE
        // =================================================================
        
        // Final status and cleanup
        telemetry.addData("Autonomous", "Complete");
        telemetry.addData("Status", autoHelper.isExecutionComplete() ? "SUCCESS" : "PARTIAL");
        decodeHelper.updateTelemetry();
        telemetry.update();
        
        // Keep robot stopped and telemetry visible
        sleep(1000);
    }
}

/*
 * USAGE NOTES FOR DECODE HELPER WITH AAF:
 * 
 * 1. SIMPLE SHOOTING:
 *    decodeHelper.addShootingSequence(autoHelper, numShots, "description");
 * 
 * 2. SHOOTER CONTROL:
 *    decodeHelper.addStartShooterStep(autoHelper, "description");
 *    decodeHelper.addStopShooterStep(autoHelper, "description");
 *    
 * 3. CUSTOM ACTIONS:
 *    autoHelper.addStep("description", decodeHelper.createShootAction(shots, keepRunning));
 *    
 * 4. NON-BLOCKING SHOOTING (advanced):
 *    java.util.function.Supplier<Boolean> shootAction = 
 *        decodeHelper.createNonBlockingShootAction(shots, keepRunning);
 *    autoHelper.addStep("description", shootAction);
 * 
 * 5. MIXED WITH MOVEMENT:
 *    autoHelper
 *        .moveTo(x, y, heading, "description")
 *        .addStep("shoot", decodeHelper.createShootAction(shots, keepRunning))
 *        .moveTo(x2, y2, heading2, "next position");
 * 
 * FIELD COORDINATES (DECODE 2025):
 * - Origin (0,0) is field center
 * - +X is toward audience (front of field)  
 * - +Y is from Red Wall toward Blue Alliance
 * - Heading 0° faces toward Blue Alliance (+Y)
 * 
 * COMMON POSITIONS:
 *   For DECODE SEASON:
 *   • Red Goal (ID 24): (-58.3727, 55.6425, 29.5) heading 315° (back-right corner)
 *   • Blue Goal (ID 20): (-58.3727, -55.6425, 29.5) heading 45° (back-left corner)
 * - Red Alliance Goal: (-58, 55)
 * - Blue Alliance Goal: (-58, -55)
 * - Starting positions: Check your alliance and starting tile
 *   For INTO THE DEEP SEASON:
 * - Basket area: (24, 36) to (36, 48)
 * - Specimen pickup: (12, 60) area
 * - Starting positions: Check your alliance and starting tile
 */