package org.firstinspires.ftc.teamcode.util.aurora;

/**
 * IndexingSystemTest - Simple test cases for IndexingSystem logic
 *
 * This class provides basic validation of the indexing system state machine
 * without requiring actual hardware. Useful for development and debugging.
 *
 * NOTE: This is NOT a JUnit test class - it's designed to be run from an OpMode
 * or called directly for validation. For proper unit testing, integrate with
 * your test framework.
 */
public class IndexingSystemTestClass {

    /**
     * Test result container
     */
    public static class TestResult {
        public final String testName;
        public final boolean passed;
        public final String message;

        public TestResult(String testName, boolean passed, String message) {
            this.testName = testName;
            this.passed = passed;
            this.message = message;
        }

        @Override
        public String toString() {
            return String.format("%s: %s - %s", 
                passed ? "✅ PASS" : "❌ FAIL", 
                testName, 
                message);
        }
    }

    /**
     * Test: Collecting first artifact should place it in center
     */
    public static TestResult testFirstArtifactCollection() {
        try {
            // Create mock system (without hardware)
            IndexingSystem system = createMockSystem();

            // Simulate first artifact detection
            Artifact artifact = new Artifact(Artifact.Color.PURPLE, Artifact.Location.UNKNOWN, 0);
            boolean collected = system.onArtifactDetected(artifact, IndexingSystem.IntakeSource.FRONT);

            if (!collected) {
                return new TestResult("First Artifact Collection", false, 
                    "Failed to start collection");
            }

            // Simulate time passing (collection completes)
            simulateTime(system, 1000); // 1 second

            // Verify artifact is in center
            Artifact inCenter = system.getArtifactInCenter();
            if (inCenter == null) {
                return new TestResult("First Artifact Collection", false, 
                    "Artifact not in center after collection");
            }

            if (inCenter.getCollectionOrder() != 1) {
                return new TestResult("First Artifact Collection", false, 
                    "Wrong collection order: " + inCenter.getCollectionOrder());
            }

            // Verify count is 1
            if (system.getArtifactCount() != 1) {
                return new TestResult("First Artifact Collection", false, 
                    "Wrong artifact count: " + system.getArtifactCount());
            }

            return new TestResult("First Artifact Collection", true, 
                "First artifact correctly placed in center");

        } catch (Exception e) {
            return new TestResult("First Artifact Collection", false, 
                "Exception: " + e.getMessage());
        }
    }

    /**
     * Test: Second artifact should push first to opposite intake
     */
    public static TestResult testSecondArtifactPush() {
        try {
            IndexingSystem system = createMockSystem();

            // Collect first artifact from front
            Artifact first = new Artifact(Artifact.Color.PURPLE, Artifact.Location.UNKNOWN, 0);
            system.onArtifactDetected(first, IndexingSystem.IntakeSource.FRONT);
            simulateTime(system, 1000);

            // Verify first is in center
            if (system.getArtifactInCenter() == null || 
                system.getArtifactInCenter().getCollectionOrder() != 1) {
                return new TestResult("Second Artifact Push", false, 
                    "First artifact not properly collected");
            }

            // Collect second artifact from back
            Artifact second = new Artifact(Artifact.Color.GREEN, Artifact.Location.UNKNOWN, 0);
            system.onArtifactDetected(second, IndexingSystem.IntakeSource.BACK);
            simulateTime(system, 2000); // Allow time for push operation

            // Verify second is now in center
            Artifact inCenter = system.getArtifactInCenter();
            if (inCenter == null || inCenter.getCollectionOrder() != 2) {
                return new TestResult("Second Artifact Push", false, 
                    "Second artifact not in center: " + 
                    (inCenter != null ? inCenter.getCollectionOrder() : "null"));
            }

            // Verify first was pushed to front intake (opposite of back)
            Artifact inFront = system.getArtifactInFrontIntake();
            if (inFront == null || inFront.getCollectionOrder() != 1) {
                return new TestResult("Second Artifact Push", false, 
                    "First artifact not in front intake: " + 
                    (inFront != null ? inFront.getCollectionOrder() : "null"));
            }

            // Verify count is 2
            if (system.getArtifactCount() != 2) {
                return new TestResult("Second Artifact Push", false, 
                    "Wrong artifact count: " + system.getArtifactCount());
            }

            return new TestResult("Second Artifact Push", true, 
                "Second artifact pushed first to opposite intake correctly");

        } catch (Exception e) {
            return new TestResult("Second Artifact Push", false, 
                "Exception: " + e.getMessage());
        }
    }

    /**
     * Test: Third artifact stays in same intake
     */
    public static TestResult testThirdArtifactStorage() {
        try {
            IndexingSystem system = createMockSystem();

            // Collect first from front
            system.onArtifactDetected(
                new Artifact(Artifact.Color.PURPLE, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.FRONT
            );
            simulateTime(system, 1000);

            // Collect second from back (pushes first to front)
            system.onArtifactDetected(
                new Artifact(Artifact.Color.GREEN, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.BACK
            );
            simulateTime(system, 2000);

            // Collect third from back
            system.onArtifactDetected(
                new Artifact(Artifact.Color.PURPLE, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.BACK
            );
            simulateTime(system, 1000);

            // Verify third is in back intake (same as collection source)
            Artifact inBack = system.getArtifactInBackIntake();
            if (inBack == null || inBack.getCollectionOrder() != 3) {
                return new TestResult("Third Artifact Storage", false, 
                    "Third artifact not in back intake: " + 
                    (inBack != null ? inBack.getCollectionOrder() : "null"));
            }

            // Verify count is 3
            if (system.getArtifactCount() != 3) {
                return new TestResult("Third Artifact Storage", false, 
                    "Wrong artifact count: " + system.getArtifactCount());
            }

            // Verify all locations are occupied
            if (system.getArtifactInCenter() == null ||
                system.getArtifactInFrontIntake() == null ||
                system.getArtifactInBackIntake() == null) {
                return new TestResult("Third Artifact Storage", false, 
                    "Not all locations occupied");
            }

            return new TestResult("Third Artifact Storage", true, 
                "Third artifact stored in same intake correctly");

        } catch (Exception e) {
            return new TestResult("Third Artifact Storage", false, 
                "Exception: " + e.getMessage());
        }
    }

    /**
     * Test: Cannot collect when system is full
     */
    public static TestResult testSystemFull() {
        try {
            IndexingSystem system = createMockSystem();

            // Fill system with 3 artifacts
            system.onArtifactDetected(
                new Artifact(Artifact.Color.PURPLE, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.FRONT
            );
            simulateTime(system, 1000);

            system.onArtifactDetected(
                new Artifact(Artifact.Color.GREEN, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.BACK
            );
            simulateTime(system, 2000);

            system.onArtifactDetected(
                new Artifact(Artifact.Color.PURPLE, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.BACK
            );
            simulateTime(system, 1000);

            // Try to collect fourth artifact (should fail)
            boolean collected = system.onArtifactDetected(
                new Artifact(Artifact.Color.GREEN, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.FRONT
            );

            if (collected) {
                return new TestResult("System Full Check", false, 
                    "System allowed 4th artifact collection");
            }

            // Verify count is still 3
            if (system.getArtifactCount() != 3) {
                return new TestResult("System Full Check", false, 
                    "Wrong artifact count: " + system.getArtifactCount());
            }

            return new TestResult("System Full Check", true, 
                "System correctly rejected 4th artifact");

        } catch (Exception e) {
            return new TestResult("System Full Check", false, 
                "Exception: " + e.getMessage());
        }
    }

    /**
     * Test: Early fire with one artifact
     */
    public static TestResult testEarlyFireOneArtifact() {
        try {
            IndexingSystem system = createMockSystem();

            // Collect one artifact
            system.onArtifactDetected(
                new Artifact(Artifact.Color.PURPLE, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.FRONT
            );
            simulateTime(system, 1000);

            // Fire
            boolean fired = system.onFireSignal();

            if (!fired) {
                return new TestResult("Early Fire - One Artifact", false, 
                    "Failed to start firing");
            }

            return new TestResult("Early Fire - One Artifact", true, 
                "Early fire with one artifact successful");

        } catch (Exception e) {
            return new TestResult("Early Fire - One Artifact", false, 
                "Exception: " + e.getMessage());
        }
    }

    /**
     * Test: Early fire with two artifacts
     */
    public static TestResult testEarlyFireTwoArtifacts() {
        try {
            IndexingSystem system = createMockSystem();

            // Collect two artifacts
            system.onArtifactDetected(
                new Artifact(Artifact.Color.PURPLE, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.FRONT
            );
            simulateTime(system, 1000);

            system.onArtifactDetected(
                new Artifact(Artifact.Color.GREEN, Artifact.Location.UNKNOWN, 0),
                IndexingSystem.IntakeSource.BACK
            );
            simulateTime(system, 2000);

            // Verify second is in center, first in storage
            if (system.getArtifactInCenter() == null || 
                system.getArtifactInCenter().getCollectionOrder() != 2) {
                return new TestResult("Early Fire - Two Artifacts", false, 
                    "Wrong artifact in center before fire");
            }

            // Fire (should fire second artifact)
            boolean fired = system.onFireSignal();

            if (!fired) {
                return new TestResult("Early Fire - Two Artifacts", false, 
                    "Failed to start firing");
            }

            return new TestResult("Early Fire - Two Artifacts", true, 
                "Early fire with two artifacts successful");

        } catch (Exception e) {
            return new TestResult("Early Fire - Two Artifacts", false, 
                "Exception: " + e.getMessage());
        }
    }

    /**
     * Run all tests and return results
     */
    public static TestResult[] runAllTests() {
        return new TestResult[] {
            testFirstArtifactCollection(),
            testSecondArtifactPush(),
            testThirdArtifactStorage(),
            testSystemFull(),
            testEarlyFireOneArtifact(),
            testEarlyFireTwoArtifacts()
        };
    }

    /**
     * Create a mock IndexingSystem for testing (without hardware)
     */
    private static IndexingSystem createMockSystem() {
        // Create minimal config
        IndexingConfig config = new IndexingConfig();
        config.setDebugTelemetry(false); // Disable telemetry for tests

        // Create system with null hardware and shooter (testing logic only)
        return new IndexingSystem(null, config, null, null);
    }

    /**
     * Simulate time passing by calling update repeatedly
     */
    private static void simulateTime(IndexingSystem system, long milliseconds) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < milliseconds) {
            system.update();
            try {
                Thread.sleep(20); // Update every 20ms
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    /**
     * Print test results to console
     */
    public static void printTestResults(TestResult[] results) {
        int passed = 0;
        int failed = 0;

        System.out.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        System.out.println("  INDEXING SYSTEM TEST RESULTS");
        System.out.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        System.out.println();

        for (TestResult result : results) {
            System.out.println(result);
            if (result.passed) {
                passed++;
            } else {
                failed++;
            }
        }

        System.out.println();
        System.out.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        System.out.println(String.format("  SUMMARY: %d passed, %d failed", passed, failed));
        System.out.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    }
}
