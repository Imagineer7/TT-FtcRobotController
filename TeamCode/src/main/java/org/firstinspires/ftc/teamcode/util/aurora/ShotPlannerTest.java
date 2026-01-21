package org.firstinspires.ftc.teamcode.util.aurora;

import java.util.ArrayList;
import java.util.List;

/**
 * ShotPlannerTest - Test suite for ShotPlanner functionality
 * 
 * Tests pattern scoring algorithm and shot planning logic
 */
@Deprecated
public class ShotPlannerTest {

    public static void main(String[] args) {
        ShotPlannerTest test = new ShotPlannerTest();
        
        System.out.println("========================================");
        System.out.println("ShotPlanner Test Suite");
        System.out.println("========================================\n");
        
        int passed = 0;
        int failed = 0;
        
        // Test 1: Pattern scoring with PPG pattern
        System.out.println("Test 1: Pattern scoring with PPG pattern");
        if (test.testPatternScoringPPG()) {
            System.out.println("✓ PASSED\n");
            passed++;
        } else {
            System.out.println("✗ FAILED\n");
            failed++;
        }
        
        // Test 2: Pattern scoring with PGP pattern
        System.out.println("Test 2: Pattern scoring with PGP pattern");
        if (test.testPatternScoringPGP()) {
            System.out.println("✓ PASSED\n");
            passed++;
        } else {
            System.out.println("✗ FAILED\n");
            failed++;
        }
        
        // Test 3: Two artifact rearrangement request
        System.out.println("Test 3: Two artifact rearrangement request");
        if (test.testTwoArtifactRearrangement()) {
            System.out.println("✓ PASSED\n");
            passed++;
        } else {
            System.out.println("✗ FAILED\n");
            failed++;
        }
        
        // Test 4: Skip planning with one artifact
        System.out.println("Test 4: Skip planning with one artifact");
        if (test.testSkipPlanningOneArtifact()) {
            System.out.println("✓ PASSED\n");
            passed++;
        } else {
            System.out.println("✗ FAILED\n");
            failed++;
        }
        
        // Test 5: Skip planning with three artifacts
        System.out.println("Test 5: Skip planning with three artifacts");
        if (test.testSkipPlanningThreeArtifacts()) {
            System.out.println("✓ PASSED\n");
            passed++;
        } else {
            System.out.println("✗ FAILED\n");
            failed++;
        }
        
        // Test 6: Skip planning with all green artifacts
        System.out.println("Test 6: Skip planning with all green artifacts");
        if (test.testSkipPlanningAllGreen()) {
            System.out.println("✓ PASSED\n");
            passed++;
        } else {
            System.out.println("✗ FAILED\n");
            failed++;
        }
        
        // Test 7: Canonical scenario - PPG pattern
        System.out.println("Test 7: Canonical scenario - PPG pattern");
        if (test.testCanonicalScenarioPPG()) {
            System.out.println("✓ PASSED\n");
            passed++;
        } else {
            System.out.println("✗ FAILED\n");
            failed++;
        }
        
        System.out.println("========================================");
        System.out.println(String.format("Test Results: %d passed, %d failed", passed, failed));
        System.out.println("========================================");
        
        if (failed > 0) {
            System.exit(1);
        }
    }
    
    /**
     * Test pattern scoring with PPG pattern
     * Scenario: Green in center, Purple in storage
     * Expected: Planner should request rearrangement to get Purple first
     */
    private boolean testPatternScoringPPG() {
        ShotPlanner planner = new ShotPlanner();
        planner.setMotifPattern("PPG");
        planner.setManualPushMode(false);
        
        // Setup: Green in center (#1), Purple in front intake (#2)
        List<Artifact> artifacts = new ArrayList<>();
        Artifact green = new Artifact(Artifact.Color.GREEN, Artifact.Location.CENTER_STORAGE, 1);
        Artifact purple = new Artifact(Artifact.Color.PURPLE, Artifact.Location.FRONT_INTAKE, 2);
        artifacts.add(green);
        artifacts.add(purple);
        
        // Update planner
        planner.updateShotPlan(artifacts, green, purple, null);
        
        // Check: Should request rearrangement to get Purple first
        Artifact desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter == null) {
            System.out.println("  ERROR: Expected rearrangement request, got null");
            return false;
        }
        
        if (desiredCenter.getColor() != Artifact.Color.PURPLE) {
            System.out.println("  ERROR: Expected Purple in center, got " + desiredCenter.getColor());
            return false;
        }
        
        // Check shot plan order: should be Purple, Green
        List<Artifact> shotPlan = planner.getShotPlan();
        if (shotPlan.size() != 2) {
            System.out.println("  ERROR: Expected 2 artifacts in shot plan, got " + shotPlan.size());
            return false;
        }
        
        if (shotPlan.get(0).getColor() != Artifact.Color.PURPLE) {
            System.out.println("  ERROR: Expected first shot to be Purple, got " + shotPlan.get(0).getColor());
            return false;
        }
        
        if (shotPlan.get(1).getColor() != Artifact.Color.GREEN) {
            System.out.println("  ERROR: Expected second shot to be Green, got " + shotPlan.get(1).getColor());
            return false;
        }
        
        System.out.println("  Shot plan: Purple → Green");
        System.out.println("  Desired center: Purple #2");
        return true;
    }
    
    /**
     * Test pattern scoring with PGP pattern
     * Scenario: Green in center, Purple in storage
     * Expected: No rearrangement needed (Purple matches second position)
     */
    private boolean testPatternScoringPGP() {
        ShotPlanner planner = new ShotPlanner();
        planner.setMotifPattern("PGP");
        planner.setManualPushMode(false);
        
        // Setup: Purple in center (#1), Green in front intake (#2)
        List<Artifact> artifacts = new ArrayList<>();
        Artifact purple = new Artifact(Artifact.Color.PURPLE, Artifact.Location.CENTER_STORAGE, 1);
        Artifact green = new Artifact(Artifact.Color.GREEN, Artifact.Location.FRONT_INTAKE, 2);
        artifacts.add(purple);
        artifacts.add(green);
        
        // Update planner
        planner.updateShotPlan(artifacts, purple, green, null);
        
        // Check: Should NOT request rearrangement (Purple already in center matches first position)
        Artifact desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter != null) {
            System.out.println("  ERROR: Expected no rearrangement request, got " + desiredCenter.getColor());
            return false;
        }
        
        // Check shot plan order: should be Purple, Green
        List<Artifact> shotPlan = planner.getShotPlan();
        if (shotPlan.size() != 2) {
            System.out.println("  ERROR: Expected 2 artifacts in shot plan, got " + shotPlan.size());
            return false;
        }
        
        if (shotPlan.get(0).getColor() != Artifact.Color.PURPLE) {
            System.out.println("  ERROR: Expected first shot to be Purple, got " + shotPlan.get(0).getColor());
            return false;
        }
        
        System.out.println("  Shot plan: Purple → Green");
        System.out.println("  No rearrangement needed");
        return true;
    }
    
    /**
     * Test two artifact rearrangement request
     */
    private boolean testTwoArtifactRearrangement() {
        ShotPlanner planner = new ShotPlanner();
        planner.setMotifPattern("GPP");
        planner.setManualPushMode(false);
        
        // Setup: Purple in center (#1), Green in back intake (#2)
        List<Artifact> artifacts = new ArrayList<>();
        Artifact purple = new Artifact(Artifact.Color.PURPLE, Artifact.Location.CENTER_STORAGE, 1);
        Artifact green = new Artifact(Artifact.Color.GREEN, Artifact.Location.BACK_INTAKE, 2);
        artifacts.add(purple);
        artifacts.add(green);
        
        // Update planner
        planner.updateShotPlan(artifacts, purple, null, green);
        
        // Check: Should request rearrangement to get Green first (GPP pattern)
        Artifact desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter == null) {
            System.out.println("  ERROR: Expected rearrangement request, got null");
            return false;
        }
        
        if (desiredCenter.getColor() != Artifact.Color.GREEN) {
            System.out.println("  ERROR: Expected Green in center, got " + desiredCenter.getColor());
            return false;
        }
        
        System.out.println("  Shot plan: Green → Purple");
        System.out.println("  Rearrangement requested: Green to center");
        return true;
    }
    
    /**
     * Test skip planning with one artifact
     */
    private boolean testSkipPlanningOneArtifact() {
        ShotPlanner planner = new ShotPlanner();
        planner.setMotifPattern("PPG");
        planner.setManualPushMode(false);
        
        // Setup: Only one artifact (Purple in center)
        List<Artifact> artifacts = new ArrayList<>();
        Artifact purple = new Artifact(Artifact.Color.PURPLE, Artifact.Location.CENTER_STORAGE, 1);
        artifacts.add(purple);
        
        // Update planner
        planner.updateShotPlan(artifacts, purple, null, null);
        
        // Check: Should not request rearrangement (can't rearrange with 1 artifact)
        Artifact desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter != null) {
            System.out.println("  ERROR: Expected no rearrangement with 1 artifact, got " + desiredCenter.getColor());
            return false;
        }
        
        System.out.println("  Correctly skipped planning (1 artifact)");
        return true;
    }
    
    /**
     * Test skip planning with three artifacts
     */
    private boolean testSkipPlanningThreeArtifacts() {
        ShotPlanner planner = new ShotPlanner();
        planner.setMotifPattern("PPG");
        planner.setManualPushMode(false);
        
        // Setup: Three artifacts (Green in center, Purple in front, Purple in back)
        List<Artifact> artifacts = new ArrayList<>();
        Artifact green = new Artifact(Artifact.Color.GREEN, Artifact.Location.CENTER_STORAGE, 2);
        Artifact purple1 = new Artifact(Artifact.Color.PURPLE, Artifact.Location.FRONT_INTAKE, 1);
        Artifact purple2 = new Artifact(Artifact.Color.PURPLE, Artifact.Location.BACK_INTAKE, 3);
        artifacts.add(purple1);
        artifacts.add(green);
        artifacts.add(purple2);
        
        // Update planner
        planner.updateShotPlan(artifacts, green, purple1, purple2);
        
        // Check: Should not request rearrangement (can't rearrange with 3 artifacts)
        Artifact desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter != null) {
            System.out.println("  ERROR: Expected no rearrangement with 3 artifacts, got " + desiredCenter.getColor());
            return false;
        }
        
        System.out.println("  Correctly skipped planning (3 artifacts)");
        return true;
    }
    
    /**
     * Test skip planning with all green artifacts
     */
    private boolean testSkipPlanningAllGreen() {
        ShotPlanner planner = new ShotPlanner();
        planner.setMotifPattern("PPG");
        planner.setManualPushMode(false);
        
        // Setup: Two green artifacts
        List<Artifact> artifacts = new ArrayList<>();
        Artifact green1 = new Artifact(Artifact.Color.GREEN, Artifact.Location.CENTER_STORAGE, 1);
        Artifact green2 = new Artifact(Artifact.Color.GREEN, Artifact.Location.FRONT_INTAKE, 2);
        artifacts.add(green1);
        artifacts.add(green2);
        
        // Update planner
        planner.updateShotPlan(artifacts, green1, green2, null);
        
        // Check: Should not request rearrangement (all green, order doesn't matter)
        Artifact desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter != null) {
            System.out.println("  ERROR: Expected no rearrangement with all green, got " + desiredCenter.getColor());
            return false;
        }
        
        System.out.println("  Correctly skipped planning (all green)");
        return true;
    }
    
    /**
     * Test canonical scenario: PPG pattern
     * Step 1: Green collected → Goes to center
     * Step 2: Purple collected → Purple desired first → request push
     * Step 3: Purple collected → Stored, no rearrangement
     * Result: Shot Plan P → P → G
     */
    private boolean testCanonicalScenarioPPG() {
        ShotPlanner planner = new ShotPlanner();
        planner.setMotifPattern("PPG");
        planner.setManualPushMode(false);
        
        System.out.println("  Step 1: Green collected, goes to center");
        List<Artifact> artifacts = new ArrayList<>();
        Artifact green = new Artifact(Artifact.Color.GREEN, Artifact.Location.CENTER_STORAGE, 1);
        artifacts.add(green);
        
        planner.updateShotPlan(artifacts, green, null, null);
        Artifact desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter != null) {
            System.out.println("  ERROR: Step 1 - Expected no rearrangement, got " + desiredCenter.getColor());
            return false;
        }
        System.out.println("  → No rearrangement needed");
        
        System.out.println("  Step 2: Purple collected");
        Artifact purple1 = new Artifact(Artifact.Color.PURPLE, Artifact.Location.FRONT_INTAKE, 2);
        artifacts.add(purple1);
        
        planner.updateShotPlan(artifacts, green, purple1, null);
        desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter == null) {
            System.out.println("  ERROR: Step 2 - Expected rearrangement request");
            return false;
        }
        if (desiredCenter.getColor() != Artifact.Color.PURPLE) {
            System.out.println("  ERROR: Step 2 - Expected Purple, got " + desiredCenter.getColor());
            return false;
        }
        System.out.println("  → Rearrangement requested: Purple to center");
        
        // Simulate rearrangement: Purple now in center, Green in back intake
        green = green.withLocation(Artifact.Location.BACK_INTAKE);
        purple1 = purple1.withLocation(Artifact.Location.CENTER_STORAGE);
        
        System.out.println("  Step 3: Second Purple collected");
        Artifact purple2 = new Artifact(Artifact.Color.PURPLE, Artifact.Location.FRONT_INTAKE, 3);
        artifacts.clear();
        artifacts.add(green);
        artifacts.add(purple1);
        artifacts.add(purple2);
        
        planner.updateShotPlan(artifacts, purple1, purple2, green);
        desiredCenter = planner.getDesiredCenterArtifact();
        
        if (desiredCenter != null) {
            System.out.println("  ERROR: Step 3 - Expected no rearrangement with 3 artifacts");
            return false;
        }
        System.out.println("  → No rearrangement (3 artifacts)");
        
        List<Artifact> shotPlan = planner.getShotPlan();
        if (shotPlan.size() != 3) {
            System.out.println("  ERROR: Expected 3 artifacts in shot plan, got " + shotPlan.size());
            return false;
        }
        
        // Check final shot plan: P → P → G
        if (shotPlan.get(0).getColor() != Artifact.Color.PURPLE ||
            shotPlan.get(1).getColor() != Artifact.Color.PURPLE ||
            shotPlan.get(2).getColor() != Artifact.Color.GREEN) {
            System.out.println("  ERROR: Expected P → P → G, got " +
                shotPlan.get(0).getColor() + " → " +
                shotPlan.get(1).getColor() + " → " +
                shotPlan.get(2).getColor());
            return false;
        }
        
        System.out.println("  Final shot plan: P → P → G ✓");
        return true;
    }
}
