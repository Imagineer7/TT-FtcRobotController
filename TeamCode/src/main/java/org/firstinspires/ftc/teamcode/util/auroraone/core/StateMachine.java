package org.firstinspires.ftc.teamcode.util.auroraone.core;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.auroraone.config.RobotMap;
import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;
import org.firstinspires.ftc.teamcode.util.auroraone.subsystems.*;
import org.firstinspires.ftc.teamcode.util.auroraone.subsystems.localization.LocalizationUnifier;
import org.firstinspires.ftc.teamcode.util.auroraone.subsystems.localization.VisionLocalizer;
import org.firstinspires.ftc.teamcode.util.auroraone.utility.Logger;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * AURORA ONE - Advanced State Machine Framework
 *
 * This class represents a state machine used for managing different states and transitions
 * within the robot's operation. It provides a structured way to handle complex behaviors
 * by defining states, events, and transitions, allowing for organized and maintainable code.
 * This class is essential for implementing autonomous routines, handling user inputs,
 * and managing various operational modes of the robot.
 *
 * ARCHITECTURE OVERVIEW:
 * =====================
 * The StateMachine follows a hierarchical finite state machine (HFSM) pattern with the following components:
 *
 * 1. STATES: Represent different operational modes (IDLE, COLLECTING, SHOOTING, DRIVING, etc.)
 * 2. EVENTS: Triggers that cause state transitions (BUTTON_PRESS, SENSOR_TRIGGER, TIMER_EXPIRED, etc.)
 * 3. TRANSITIONS: Rules that define how states change based on events and conditions
 * 4. ACTIONS: Code that executes during state entry, exit, and while active
 * 5. CONDITIONS: Boolean checks that must be met for transitions to occur
 *
 * SUBSYSTEM INTEGRATION:
 * =====================
 * - DriveHandler: Controls robot movement and navigation
 * - ShooterHandler: Manages shooting mechanisms and RPM control
 * - Collector: Handles intake and outtake of game elements
 * - ObstacleSensors: Provides collision detection and avoidance
 * - LocalizationUnifier: Tracks robot position on the field
 * - VisionLocalizer: Provides vision-based localization
 * - Blackboard: Centralized data sharing between subsystems
 * - Logger: Advanced logging and debugging capabilities
 *
 * USAGE EXAMPLES:
 * ==============
 *
 * Basic Setup:
 * ```java
 * StateMachine stateMachine = StateMachine.getInstance();
 * stateMachine.initialize(robotMap, telemetry);
 * stateMachine.start();
 * ```
 *
 * Adding Custom States:
 * ```java
 * stateMachine.addState(new CustomState("CUSTOM_COLLECT") {
 *     @Override
 *     public void onEntry() {
 *         collector.startIntake();
 *         logger.info("Started custom collection sequence");
 *     }
 *
 *     @Override
 *     public void onUpdate() {
 *         if (collector.hasGameElement()) {
 *             triggerEvent(Event.ELEMENT_COLLECTED);
 *         }
 *     }
 * });
 * ```
 *
 * Triggering Events:
 * ```java
 * stateMachine.triggerEvent(Event.START_COLLECTION);
 * stateMachine.triggerEvent(Event.EMERGENCY_STOP);
 * ```
 */
public class StateMachine {

    // =========================================================================================
    // SINGLETON PATTERN - Ensures only one state machine instance exists
    // =========================================================================================
    private static volatile StateMachine instance;
    private static final Object lock = new Object();

    /**
     * Gets the singleton instance of the StateMachine
     * Thread-safe lazy initialization with double-checked locking
     */
    public static StateMachine getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new StateMachine();
                }
            }
        }
        return instance;
    }

    // =========================================================================================
    // STATE AND EVENT DEFINITIONS
    // =========================================================================================

    /**
     * Enumeration of all possible robot states
     * Each state represents a distinct operational mode with specific behaviors
     */
    public enum State {
        // Core operational states
        IDLE("IDLE", "Robot is idle and waiting for commands"),
        INITIALIZING("INIT", "System initialization and hardware setup"),
        EMERGENCY_STOP("E_STOP", "Emergency stop - all systems halt"),
        FAULT("FAULT", "System fault detected - requires intervention"),

        // Movement states
        DRIVING("DRIVE", "Robot is actively moving"),
        POSITIONING("POS", "Robot is positioning for an action"),
        ALIGNING("ALIGN", "Robot is aligning with target"),

        // Collection states
        COLLECTING("COLLECT", "Actively collecting game elements"),
        COLLECTION_READY("COL_RDY", "Ready to collect - sensors active"),
        COLLECTION_COMPLETE("COL_DONE", "Collection sequence completed"),

        // Shooting states
        SHOOTING("SHOOT", "Actively shooting game elements"),
        SHOOTER_SPINUP("SHOOT_UP", "Shooter motors spinning up to target RPM"),
        SHOOTER_READY("SHOOT_RDY", "Shooter ready - at target RPM"),
        SHOOTING_SEQUENCE("SHOOT_SEQ", "Executing full shooting sequence"),

        // Autonomous states
        AUTO_INIT("AUTO_INIT", "Autonomous initialization"),
        AUTO_RUNNING("AUTO_RUN", "Autonomous sequence executing"),
        AUTO_COMPLETE("AUTO_DONE", "Autonomous sequence completed"),

        // Teleop states
        TELEOP_INIT("TELE_INIT", "Teleoperated initialization"),
        TELEOP_RUNNING("TELE_RUN", "Teleoperated mode active"),

        // Maintenance states
        CALIBRATING("CALIB", "System calibration in progress"),
        TESTING("TEST", "Test mode - diagnostic operations");

        private final String shortName;
        private final String description;

        State(String shortName, String description) {
            this.shortName = shortName;
            this.description = description;
        }

        public String getShortName() { return shortName; }
        public String getDescription() { return description; }
    }

    /**
     * Enumeration of all possible events that can trigger state transitions
     * Events represent external stimuli or internal conditions that drive the state machine
     */
    public enum Event {
        // System events
        SYSTEM_INIT("System initialization requested"),
        SYSTEM_READY("System initialization completed"),
        EMERGENCY_STOP("Emergency stop triggered"),
        FAULT_DETECTED("System fault detected"),
        FAULT_CLEARED("System fault cleared"),

        // User input events
        GAMEPAD_A("Gamepad A button pressed"),
        GAMEPAD_B("Gamepad B button pressed"),
        GAMEPAD_X("Gamepad X button pressed"),
        GAMEPAD_Y("Gamepad Y button pressed"),
        GAMEPAD_LEFT_BUMPER("Gamepad left bumper pressed"),
        GAMEPAD_RIGHT_BUMPER("Gamepad right bumper pressed"),
        GAMEPAD_LEFT_TRIGGER("Gamepad left trigger pressed"),
        GAMEPAD_RIGHT_TRIGGER("Gamepad right trigger pressed"),

        // Collection events
        START_COLLECTION("Start collection sequence"),
        STOP_COLLECTION("Stop collection sequence"),
        ELEMENT_DETECTED("Game element detected by sensors"),
        ELEMENT_COLLECTED("Game element successfully collected"),
        COLLECTION_TIMEOUT("Collection sequence timed out"),

        // Shooting events
        START_SHOOTING("Start shooting sequence"),
        STOP_SHOOTING("Stop shooting sequence"),
        SHOOTER_AT_SPEED("Shooter reached target RPM"),
        SHOT_COMPLETE("Single shot completed"),
        SHOOTING_TIMEOUT("Shooting sequence timed out"),

        // Movement events
        START_DRIVING("Start driving"),
        STOP_DRIVING("Stop driving"),
        TARGET_REACHED("Movement target reached"),
        ALIGNMENT_COMPLETE("Robot alignment completed"),
        OBSTACLE_DETECTED("Obstacle detected by sensors"),

        // Autonomous events
        AUTO_START("Autonomous period started"),
        AUTO_COMPLETE("Autonomous sequence completed"),

        // Teleop events
        TELEOP_START("Teleoperated period started"),
        TELEOP_COMPLETE("Teleoperated period completed"),

        // Timer events
        TIMEOUT_SHORT("Short timeout expired (1-5 seconds)"),
        TIMEOUT_MEDIUM("Medium timeout expired (5-15 seconds)"),
        TIMEOUT_LONG("Long timeout expired (15+ seconds)"),

        // Sensor events
        DISTANCE_THRESHOLD("Distance sensor threshold triggered"),
        VISION_TARGET_ACQUIRED("Vision target acquired"),
        VISION_TARGET_LOST("Vision target lost"),

        // Test events
        START_TEST("Start test sequence"),
        TEST_COMPLETE("Test sequence completed");

        private final String description;

        Event(String description) {
            this.description = description;
        }

        public String getDescription() { return description; }
    }

    // =========================================================================================
    // STATE MACHINE CORE COMPONENTS
    // =========================================================================================

    /**
     * Abstract base class for all states
     * Provides template methods for state lifecycle management
     */
    public abstract static class BaseState {
        protected final String name;
        protected final State stateEnum;
        protected ElapsedTime stateTimer;
        protected Logger logger;
        protected Blackboard blackboard;

        public BaseState(State stateEnum) {
            this.stateEnum = stateEnum;
            this.name = stateEnum.name();
            this.stateTimer = new ElapsedTime();
            this.logger = Logger.getInstance();
            this.blackboard = Blackboard.getInstance();
        }

        /**
         * Called once when the state is entered
         * Override to implement state entry logic
         */
        public void onEntry() {
            stateTimer.reset();
            logger.info("StateMachine", "Entering state: " + name);
            blackboard.put("state_machine.current_state", name);
            blackboard.put("state_machine.state_entry_time", System.currentTimeMillis());
        }

        /**
         * Called repeatedly while the state is active
         * Override to implement state update logic
         */
        public void onUpdate() {
            // Default implementation - override in subclasses
            blackboard.put("state_machine.state_duration", stateTimer.seconds());
        }

        /**
         * Called once when the state is exited
         * Override to implement state cleanup logic
         */
        public void onExit() {
            logger.info("StateMachine", "Exiting state: " + name + " (duration: " +
                       String.format(java.util.Locale.US, "%.2f", stateTimer.seconds()) + "s)");
            blackboard.put("state_machine.last_state", name);
            blackboard.put("state_machine.last_state_duration", stateTimer.seconds());
        }

        /**
         * Checks if this state can transition to another state given an event
         * Override to implement custom transition conditions
         */
        public boolean canTransition(Event event, State targetState) {
            return true; // Default: allow all transitions
        }

        public String getName() { return name; }
        public State getStateEnum() { return stateEnum; }
        public double getStateDuration() { return stateTimer.seconds(); }
    }

    /**
     * Represents a state transition with source, target, event, and optional condition
     */
    public static class Transition {
        private final State fromState;
        private final State toState;
        private final Event event;
        private final TransitionCondition condition;
        private final String description;

        public interface TransitionCondition {
            boolean evaluate();
        }

        public Transition(State fromState, State toState, Event event) {
            this(fromState, toState, event, () -> true, "");
        }

        public Transition(State fromState, State toState, Event event, TransitionCondition condition) {
            this(fromState, toState, event, condition, "");
        }

        public Transition(State fromState, State toState, Event event, String description) {
            this(fromState, toState, event, () -> true, description);
        }

        public Transition(State fromState, State toState, Event event,
                         TransitionCondition condition, String description) {
            this.fromState = fromState;
            this.toState = toState;
            this.event = event;
            this.condition = condition;
            this.description = description;
        }

        public boolean canTransition(State currentState, Event triggeredEvent) {
            return fromState == currentState &&
                   event == triggeredEvent &&
                   condition.evaluate();
        }

        // Getters
        public State getFromState() { return fromState; }
        public State getToState() { return toState; }
        public Event getEvent() { return event; }
        public String getDescription() { return description; }
    }

    // =========================================================================================
    // INSTANCE VARIABLES - All subsystems and core components
    // =========================================================================================

    // Core components
    private Logger logger;
    private Blackboard blackboard;
    private RobotMap robotMap;
    private Telemetry telemetry;

    // Subsystems
    private DriveHandler driveHandler;
    private ShooterHandler shooterHandler;
    private Collector collector;
    private ObstacleSensors obstacleSensors;
    private LocalizationUnifier localizationUnifier;
    private VisionLocalizer visionLocalizer;

    // State machine state
    private State currentState;
    private BaseState currentStateObject;
    private final Map<State, BaseState> stateObjects;
    private final List<Transition> transitions;
    private final Queue<Event> eventQueue;

    // Timing and performance
    private ElapsedTime machineTimer;
    private ElapsedTime updateTimer;
    private double lastUpdateTime;
    private long totalUpdates;
    private double averageUpdateTime;

    // Configuration
    private boolean isInitialized;
    private boolean isRunning;
    private boolean debugMode;

    // =========================================================================================
    // CONSTRUCTOR AND INITIALIZATION
    // =========================================================================================

    private StateMachine() {
        // Initialize core collections
        this.stateObjects = new ConcurrentHashMap<>();
        this.transitions = new CopyOnWriteArrayList<>();
        this.eventQueue = new ArrayDeque<>();

        // Initialize timing
        this.machineTimer = new ElapsedTime();
        this.updateTimer = new ElapsedTime();

        // Initialize state
        this.currentState = State.IDLE;
        this.isInitialized = false;
        this.isRunning = false;
        this.debugMode = false;
        this.totalUpdates = 0;
        this.averageUpdateTime = 0.0;

        // Get core instances
        this.logger = Logger.getInstance();
        this.blackboard = Blackboard.getInstance();
    }

    /**
     * Initializes the state machine with hardware map and telemetry
     * Must be called before using the state machine
     *
     * @param robotMap Hardware map containing all robot components
     * @param telemetry FTC telemetry instance for debugging
     */
    public void initialize(RobotMap robotMap, Telemetry telemetry) {
        if (isInitialized) {
            logger.warn("StateMachine", "StateMachine already initialized");
            return;
        }

        this.robotMap = robotMap;
        this.telemetry = telemetry;

        logger.info("StateMachine", "Initializing Aurora One State Machine...");

        try {
            // Initialize subsystems
            initializeSubsystems();

            // Setup default states
            setupDefaultStates();

            // Setup default transitions
            setupDefaultTransitions();

            // Initialize blackboard values
            initializeBlackboard();

            this.isInitialized = true;
            logger.info("StateMachine", "State Machine initialization completed successfully");

        } catch (Exception e) {
            logger.error("StateMachine", "Failed to initialize State Machine: " + e.getMessage());
            this.currentState = State.FAULT;
            throw new RuntimeException("StateMachine initialization failed", e);
        }
    }

    /**
     * Initializes all subsystems
     * Each subsystem is responsible for its own hardware initialization
     */
    private void initializeSubsystems() {
        logger.info("StateMachine", "Initializing subsystems...");

        // Initialize drive system
        this.driveHandler = new DriveHandler(robotMap);
        // driveHandler.initialize(robotMap, telemetry); // Uncomment when DriveHandler has this method

        // Initialize shooter system
        this.shooterHandler = new ShooterHandler(robotMap);
        // shooterHandler.initialize(robotMap, telemetry); // Uncomment when ShooterHandler has this method

        // Initialize collector system
        this.collector = new Collector();
        // collector.initialize(robotMap, telemetry); // Uncomment when Collector has this method

        // Initialize obstacle sensors
        this.obstacleSensors = new ObstacleSensors(robotMap, telemetry);
        // obstacleSensors.initialize(robotMap, telemetry); // Uncomment when ObstacleSensors has this method

        // Initialize localization systems
        this.localizationUnifier = new LocalizationUnifier(robotMap, telemetry, null);
        // localizationUnifier.initialize(robotMap, telemetry); // Uncomment when LocalizationUnifier has this method

        this.visionLocalizer = new VisionLocalizer(robotMap, telemetry);
        // visionLocalizer.initialize(robotMap, telemetry); // Uncomment when VisionLocalizer has this method

        logger.info("StateMachine", "All subsystems initialized");
    }

    /**
     * Sets up default state objects for common robot operations
     * These can be extended or replaced with custom implementations
     */
    private void setupDefaultStates() {
        logger.info("StateMachine", "Setting up default states...");

        // IDLE State - Default state when no operations are active
        addState(new BaseState(State.IDLE) {
            @Override
            public void onEntry() {
                super.onEntry();
                // Stop all subsystems
                // driveHandler.stop();
                // shooterHandler.stop();
                // collector.stop();
                blackboard.put("robot.mode", "idle");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();
                // Monitor for system faults or emergency conditions
                checkEmergencyConditions();
            }
        });

        // INITIALIZING State - System startup and calibration
        addState(new BaseState(State.INITIALIZING) {
            @Override
            public void onEntry() {
                super.onEntry();
                blackboard.put("robot.mode", "initializing");
                blackboard.put("init.progress", 0.0);
            }

            @Override
            public void onUpdate() {
                super.onUpdate();

                // Simulate initialization progress
                double progress = Math.min(1.0, getStateDuration() / 3.0); // 3 second init
                blackboard.put("init.progress", progress);

                if (progress >= 1.0) {
                    triggerEvent(Event.SYSTEM_READY);
                }
            }
        });

        // DRIVING State - Active movement operations
        addState(new BaseState(State.DRIVING) {
            @Override
            public void onEntry() {
                super.onEntry();
                blackboard.put("robot.mode", "driving");
                // driveHandler.enable();
            }

            @Override
            public void onUpdate() {
                super.onUpdate();

                // Check for obstacles
                // if (obstacleSensors.isObstacleDetected()) {
                //     triggerEvent(Event.OBSTACLE_DETECTED);
                // }

                // Update drive system
                // driveHandler.update();

                // Check if target reached
                // if (driveHandler.isTargetReached()) {
                //     triggerEvent(Event.TARGET_REACHED);
                // }
            }

            @Override
            public void onExit() {
                super.onExit();
                // driveHandler.stop();
            }
        });

        // COLLECTING State - Active collection operations
        addState(new BaseState(State.COLLECTING) {
            @Override
            public void onEntry() {
                super.onEntry();
                blackboard.put("robot.mode", "collecting");
                // collector.startIntake();
            }

            @Override
            public void onUpdate() {
                super.onUpdate();

                // Check for collected elements
                // if (collector.hasGameElement()) {
                //     triggerEvent(Event.ELEMENT_COLLECTED);
                // }

                // Check for timeout
                if (getStateDuration() > Tunables.COLLECTION_TIMEOUT) {
                    triggerEvent(Event.COLLECTION_TIMEOUT);
                }
            }

            @Override
            public void onExit() {
                super.onExit();
                // collector.stop();
            }
        });

        // SHOOTING State - Active shooting operations
        addState(new BaseState(State.SHOOTING) {
            @Override
            public void onEntry() {
                super.onEntry();
                blackboard.put("robot.mode", "shooting");
                // shooterHandler.spinUp();
            }

            @Override
            public void onUpdate() {
                super.onUpdate();

                // Check if shooter is ready
                // if (shooterHandler.isAtTargetRPM()) {
                //     triggerEvent(Event.SHOOTER_AT_SPEED);
                // }

                // Check for timeout
                if (getStateDuration() > Tunables.SHOOTING_TIMEOUT) {
                    triggerEvent(Event.SHOOTING_TIMEOUT);
                }
            }

            @Override
            public void onExit() {
                super.onExit();
                // shooterHandler.stop();
            }
        });

        // EMERGENCY_STOP State - Safety override state
        addState(new BaseState(State.EMERGENCY_STOP) {
            @Override
            public void onEntry() {
                super.onEntry();
                blackboard.put("robot.mode", "emergency_stop");

                // Stop all subsystems immediately
                // driveHandler.emergencyStop();
                // shooterHandler.emergencyStop();
                // collector.emergencyStop();

                logger.error("StateMachine", "EMERGENCY STOP ACTIVATED");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();
                // Remain in emergency stop until manually cleared
            }
        });

        // FAULT State - System error state
        addState(new BaseState(State.FAULT) {
            @Override
            public void onEntry() {
                super.onEntry();
                blackboard.put("robot.mode", "fault");

                // Stop all subsystems safely
                // driveHandler.stop();
                // shooterHandler.stop();
                // collector.stop();

                logger.error("StateMachine", "SYSTEM FAULT DETECTED");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();
                // Monitor for fault clearance
                if (isFaultCleared()) {
                    triggerEvent(Event.FAULT_CLEARED);
                }
            }
        });

        logger.info("StateMachine", "Default states setup completed");
    }

    /**
     * Sets up default transitions between states
     * These define the valid state changes and their triggering events
     */
    private void setupDefaultTransitions() {
        logger.info("StateMachine", "Setting up default transitions...");

        // System initialization transitions
        addTransition(State.IDLE, State.INITIALIZING, Event.SYSTEM_INIT);
        addTransition(State.INITIALIZING, State.IDLE, Event.SYSTEM_READY);

        // Emergency stop transitions (from any state)
        for (State state : State.values()) {
            if (state != State.EMERGENCY_STOP) {
                addTransition(state, State.EMERGENCY_STOP, Event.EMERGENCY_STOP);
            }
        }

        // Fault transitions (from any state except emergency stop)
        for (State state : State.values()) {
            if (state != State.FAULT && state != State.EMERGENCY_STOP) {
                addTransition(state, State.FAULT, Event.FAULT_DETECTED);
            }
        }

        // Recovery transitions
        addTransition(State.FAULT, State.IDLE, Event.FAULT_CLEARED);

        // Driving transitions
        addTransition(State.IDLE, State.DRIVING, Event.START_DRIVING);
        addTransition(State.DRIVING, State.IDLE, Event.STOP_DRIVING);
        addTransition(State.DRIVING, State.IDLE, Event.TARGET_REACHED);
        addTransition(State.DRIVING, State.IDLE, Event.OBSTACLE_DETECTED);

        // Collection transitions
        addTransition(State.IDLE, State.COLLECTING, Event.START_COLLECTION);
        addTransition(State.COLLECTING, State.IDLE, Event.ELEMENT_COLLECTED);
        addTransition(State.COLLECTING, State.IDLE, Event.STOP_COLLECTION);
        addTransition(State.COLLECTING, State.IDLE, Event.COLLECTION_TIMEOUT);

        // Shooting transitions
        addTransition(State.IDLE, State.SHOOTING, Event.START_SHOOTING);
        addTransition(State.SHOOTING, State.IDLE, Event.STOP_SHOOTING);
        addTransition(State.SHOOTING, State.IDLE, Event.SHOOTING_TIMEOUT);

        // Gamepad input transitions (examples)
        addTransition(State.IDLE, State.COLLECTING, Event.GAMEPAD_A);
        addTransition(State.IDLE, State.SHOOTING, Event.GAMEPAD_B);
        addTransition(State.IDLE, State.DRIVING, Event.GAMEPAD_X);

        logger.info("StateMachine", "Default transitions setup completed");
    }

    /**
     * Initializes blackboard with default values
     */
    private void initializeBlackboard() {
        blackboard.put("state_machine.initialized", true);
        blackboard.put("state_machine.start_time", System.currentTimeMillis());
        blackboard.put("state_machine.current_state", currentState.name());
        blackboard.put("state_machine.version", "Aurora One v1.0");
        blackboard.put("robot.mode", "idle");
        blackboard.put("robot.emergency_stop", false);
        blackboard.put("robot.fault_status", false);
    }

    // =========================================================================================
    // CORE STATE MACHINE OPERATIONS
    // =========================================================================================

    /**
     * Starts the state machine
     * Must be called after initialization
     */
    public void start() {
        if (!isInitialized) {
            throw new IllegalStateException("StateMachine must be initialized before starting");
        }

        if (isRunning) {
            logger.warn("StateMachine", "StateMachine is already running");
            return;
        }

        this.isRunning = true;
        this.machineTimer.reset();

        // Enter initial state
        if (currentStateObject == null) {
            currentStateObject = stateObjects.get(currentState);
        }

        if (currentStateObject != null) {
            currentStateObject.onEntry();
        }

        logger.info("StateMachine", "State Machine started in state: " + currentState.name());
        blackboard.put("state_machine.running", true);
    }

    /**
     * Stops the state machine
     */
    public void stop() {
        if (!isRunning) {
            return;
        }

        this.isRunning = false;

        // Exit current state
        if (currentStateObject != null) {
            currentStateObject.onExit();
        }

        logger.info("StateMachine", "State Machine stopped");
        blackboard.put("state_machine.running", false);
    }

    /**
     * Main update loop - call this repeatedly in your OpMode
     * Processes events, handles state transitions, and updates current state
     */
    public void update() {
        if (!isRunning) {
            return;
        }

        updateTimer.reset();

        try {
            // Process all queued events
            processEvents();

            // Update current state
            if (currentStateObject != null) {
                currentStateObject.onUpdate();
            }

            // Update subsystems
            updateSubsystems();

            // Update telemetry
            updateTelemetry();

            // Update performance metrics
            updatePerformanceMetrics();

        } catch (Exception e) {
            logger.error("StateMachine", "Error in state machine update: " + e.getMessage());
            triggerEvent(Event.FAULT_DETECTED);
        }
    }

    /**
     * Processes all events in the queue and handles state transitions
     */
    private void processEvents() {
        while (!eventQueue.isEmpty()) {
            Event event = eventQueue.poll();
            if (event != null) {
                handleEvent(event);
            }
        }
    }

    /**
     * Handles a single event and performs state transition if applicable
     */
    private void handleEvent(Event event) {
        logger.debug("StateMachine", "Processing event: " + event.name() + " in state: " + currentState.name());

        // Find matching transition
        for (Transition transition : transitions) {
            if (transition.canTransition(currentState, event)) {
                performTransition(transition.getToState(), event);
                return;
            }
        }

        // No transition found - log for debugging
        logger.debug("StateMachine", "No transition found for event: " + event.name() + " in state: " + currentState.name());
    }

    /**
     * Performs a state transition
     */
    private void performTransition(State newState, Event triggeringEvent) {
        State oldState = currentState;

        logger.info("StateMachine", "State transition: " + oldState.name() + " -> " + newState.name() +
                   " (event: " + triggeringEvent.name() + ")");

        // Exit current state
        if (currentStateObject != null) {
            currentStateObject.onExit();
        }

        // Update state
        this.currentState = newState;
        this.currentStateObject = stateObjects.get(newState);

        // Enter new state
        if (currentStateObject != null) {
            currentStateObject.onEntry();
        } else {
            logger.warn("StateMachine", "No state object found for state: " + newState.name());
        }

        // Update blackboard
        blackboard.put("state_machine.transition_count",
                      blackboard.get("state_machine.transition_count", 0) + 1);
    }

    /**
     * Updates all subsystems
     */
    private void updateSubsystems() {
        // Update each subsystem - uncomment when methods are available
        // driveHandler.update();
        // shooterHandler.update();
        // collector.update();
        // obstacleSensors.update();
        // localizationUnifier.update();
        // visionLocalizer.update();
    }

    /**
     * Updates telemetry with state machine information
     */
    private void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("=== STATE MACHINE ===", "");
        telemetry.addData("Current State", currentState.getShortName() + " (" + currentState.getDescription() + ")");
        telemetry.addData("State Duration", String.format(java.util.Locale.US, "%.2f s", getCurrentStateDuration()));
        telemetry.addData("Total Runtime", String.format(java.util.Locale.US, "%.2f s", machineTimer.seconds()));
        telemetry.addData("Events in Queue", eventQueue.size());
        telemetry.addData("Average Update Time", String.format(java.util.Locale.US, "%.2f ms", averageUpdateTime * 1000));

        if (debugMode) {
            telemetry.addData("=== DEBUG INFO ===", "");
            telemetry.addData("Total Updates", totalUpdates);
            telemetry.addData("Transitions", blackboard.get("state_machine.transition_count", 0));
            telemetry.addData("Last Update Time", String.format(java.util.Locale.US, "%.2f ms", lastUpdateTime * 1000));
        }
    }

    /**
     * Updates performance metrics
     */
    private void updatePerformanceMetrics() {
        lastUpdateTime = updateTimer.seconds();
        totalUpdates++;

        // Calculate rolling average update time
        double alpha = 0.1; // Smoothing factor
        averageUpdateTime = (alpha * lastUpdateTime) + ((1.0 - alpha) * averageUpdateTime);

        // Update blackboard metrics
        blackboard.put("state_machine.total_updates", totalUpdates);
        blackboard.put("state_machine.average_update_time", averageUpdateTime);
        blackboard.put("state_machine.last_update_time", lastUpdateTime);
    }

    // =========================================================================================
    // PUBLIC API METHODS
    // =========================================================================================

    /**
     * Triggers an event to be processed on the next update cycle
     * Thread-safe method for external components to communicate with the state machine
     *
     * @param event The event to trigger
     */
    public void triggerEvent(Event event) {
        synchronized (eventQueue) {
            eventQueue.offer(event);
        }
        logger.debug("StateMachine", "Event queued: " + event.name());
    }

    /**
     * Adds a custom state to the state machine
     *
     * @param state The state object to add
     */
    public void addState(BaseState state) {
        stateObjects.put(state.getStateEnum(), state);
        logger.debug("StateMachine", "Added state: " + state.getName());
    }

    /**
     * Adds a transition to the state machine
     *
     * @param transition The transition to add
     */
    public void addTransition(Transition transition) {
        transitions.add(transition);
        logger.debug("StateMachine", "Added transition: " + transition.getFromState().name() +
                    " -> " + transition.getToState().name() +
                    " on " + transition.getEvent().name());
    }

    /**
     * Convenience method to add a simple transition
     */
    public void addTransition(State fromState, State toState, Event event) {
        addTransition(new Transition(fromState, toState, event));
    }

    /**
     * Convenience method to add a conditional transition
     */
    public void addTransition(State fromState, State toState, Event event,
                             Transition.TransitionCondition condition) {
        addTransition(new Transition(fromState, toState, event, condition));
    }

    // =========================================================================================
    // UTILITY AND HELPER METHODS
    // =========================================================================================

    /**
     * Checks for emergency conditions that should trigger emergency stop
     */
    private void checkEmergencyConditions() {
        // Example emergency conditions - customize as needed

        // Check for hardware faults
        // if (driveHandler.hasFault() || shooterHandler.hasFault() || collector.hasFault()) {
        //     triggerEvent(Event.FAULT_DETECTED);
        // }

        // Check for sensor failures
        // if (obstacleSensors.hasFault() || localizationUnifier.hasFault()) {
        //     triggerEvent(Event.FAULT_DETECTED);
        // }

        // Check blackboard emergency flag
        if (blackboard.get("robot.emergency_stop", false)) {
            triggerEvent(Event.EMERGENCY_STOP);
        }
    }

    /**
     * Checks if system faults have been cleared
     */
    private boolean isFaultCleared() {
        // Add your fault checking logic here
        // Return true when it's safe to resume operation

        boolean hardwareFaultsCleared = true; // Check hardware subsystems
        boolean emergencyCleared = !blackboard.get("robot.emergency_stop", false);
        boolean manualClearance = blackboard.get("robot.fault_manual_clear", false);

        return hardwareFaultsCleared && emergencyCleared && manualClearance;
    }

    // =========================================================================================
    // GETTER METHODS
    // =========================================================================================

    public State getCurrentState() { return currentState; }
    public boolean isRunning() { return isRunning; }
    public boolean isInitialized() { return isInitialized; }
    public double getMachineRuntime() { return machineTimer.seconds(); }
    public double getCurrentStateDuration() {
        return currentStateObject != null ? currentStateObject.getStateDuration() : 0.0;
    }
    public int getQueuedEventCount() { return eventQueue.size(); }
    public long getTotalUpdates() { return totalUpdates; }
    public double getAverageUpdateTime() { return averageUpdateTime; }

    // Subsystem getters
    public DriveHandler getDriveHandler() { return driveHandler; }
    public ShooterHandler getShooterHandler() { return shooterHandler; }
    public Collector getCollector() { return collector; }
    public ObstacleSensors getObstacleSensors() { return obstacleSensors; }
    public LocalizationUnifier getLocalizationUnifier() { return localizationUnifier; }
    public VisionLocalizer getVisionLocalizer() { return visionLocalizer; }
    public Logger getLogger() { return logger; }
    public Blackboard getBlackboard() { return blackboard; }

    // =========================================================================================
    // DEBUG AND TESTING METHODS
    // =========================================================================================

    /**
     * Enables debug mode for additional telemetry and logging
     */
    public void setDebugMode(boolean enabled) {
        this.debugMode = enabled;
        logger.info("StateMachine", "Debug mode " + (enabled ? "enabled" : "disabled"));
    }

    /**
     * Forces a state change (for testing purposes only)
     * WARNING: This bypasses normal transition logic - use with caution!
     */
    public void forceState(State state) {
        logger.warn("StateMachine", "FORCE STATE: Forcing transition to " + state.name());
        performTransition(state, Event.START_TEST);
    }

    /**
     * Gets current state machine statistics for debugging
     */
    public String getStatistics() {
        return String.format(java.util.Locale.US,
            "StateMachine Stats:\n" +
            "  Current State: %s (%.2fs)\n" +
            "  Total Runtime: %.2fs\n" +
            "  Total Updates: %d\n" +
            "  Avg Update Time: %.2fms\n" +
            "  Queued Events: %d\n" +
            "  Total Transitions: %d",
            currentState.name(),
            getCurrentStateDuration(),
            getMachineRuntime(),
            totalUpdates,
            averageUpdateTime * 1000,
            eventQueue.size(),
            blackboard.get("state_machine.transition_count", 0)
        );
    }
}

/*
 * =========================================================================================
 * USAGE EXAMPLES AND IMPLEMENTATION GUIDE
 * =========================================================================================
 *
 * BASIC OPMODE INTEGRATION:
 * ========================
 *
 * ```java
 * @TeleOp(name="Aurora One TeleOp", group="Aurora")
 * public class AuroraOneTeleOp extends LinearOpMode {
 *     private StateMachine stateMachine;
 *     private RobotMap robotMap;
 *
 *     @Override
 *     public void runOpMode() {
 *         // Initialize hardware map
 *         robotMap = new RobotMap();
 *         robotMap.init(hardwareMap);
 *
 *         // Initialize state machine
 *         stateMachine = StateMachine.getInstance();
 *         stateMachine.initialize(robotMap, telemetry);
 *
 *         waitForStart();
 *
 *         // Start state machine
 *         stateMachine.start();
 *         stateMachine.triggerEvent(StateMachine.Event.TELEOP_START);
 *
 *         while (opModeIsActive()) {
 *             // Update state machine
 *             stateMachine.update();
 *
 *             // Handle gamepad inputs
 *             handleGamepadInputs();
 *
 *             // Update telemetry
 *             telemetry.update();
 *         }
 *
 *         stateMachine.stop();
 *     }
 *
 *     private void handleGamepadInputs() {
 *         if (gamepad1.a && !lastGamepad1.a) {
 *             stateMachine.triggerEvent(StateMachine.Event.GAMEPAD_A);
 *         }
 *         if (gamepad1.b && !lastGamepad1.b) {
 *             stateMachine.triggerEvent(StateMachine.Event.GAMEPAD_B);
 *         }
 *         // ... handle other inputs
 *     }
 * }
 * ```
 *
 * ADDING CUSTOM STATES:
 * ====================
 *
 * ```java
 * // Create a custom state for a complex autonomous sequence
 * stateMachine.addState(new StateMachine.BaseState(StateMachine.State.AUTO_RUNNING) {
 *     private int sequenceStep = 0;
 *
 *     @Override
 *     public void onEntry() {
 *         super.onEntry();
 *         sequenceStep = 0;
 *         logger.info("Starting autonomous sequence");
 *     }
 *
 *     @Override
 *     public void onUpdate() {
 *         super.onUpdate();
 *
 *         switch (sequenceStep) {
 *             case 0: // Drive to position
 *                 driveHandler.driveToPosition(24, 24, 0);
 *                 if (driveHandler.isAtTarget()) {
 *                     sequenceStep++;
 *                 }
 *                 break;
 *
 *             case 1: // Collect element
 *                 collector.startIntake();
 *                 if (collector.hasGameElement()) {
 *                     sequenceStep++;
 *                 }
 *                 break;
 *
 *             case 2: // Drive to shooting position
 *                 driveHandler.driveToPosition(0, 0, 90);
 *                 if (driveHandler.isAtTarget()) {
 *                     sequenceStep++;
 *                 }
 *                 break;
 *
 *             case 3: // Shoot
 *                 shooterHandler.shoot();
 *                 if (shooterHandler.isComplete()) {
 *                     triggerEvent(StateMachine.Event.AUTO_COMPLETE);
 *                 }
 *                 break;
 *         }
 *
 *         // Timeout safety
 *         if (getStateDuration() > 30.0) { // 30 second timeout
 *             triggerEvent(StateMachine.Event.AUTO_COMPLETE);
 *         }
 *     }
 * });
 * ```
 *
 * CONDITIONAL TRANSITIONS:
 * =======================
 *
 * ```java
 * // Only allow shooting if we have ammo
 * stateMachine.addTransition(
 *     StateMachine.State.IDLE,
 *     StateMachine.State.SHOOTING,
 *     StateMachine.Event.START_SHOOTING,
 *     () -> collector.hasGameElement() && shooterHandler.isReady()
 * );
 *
 * // Emergency stop only if not already in emergency state
 * stateMachine.addTransition(
 *     StateMachine.State.DRIVING,
 *     StateMachine.State.EMERGENCY_STOP,
 *     StateMachine.Event.EMERGENCY_STOP,
 *     () -> obstacleSensors.isImmediateDanger()
 * );
 * ```
 *
 * BLACKBOARD COMMUNICATION:
 * =========================
 *
 * ```java
 * // Subsystems can communicate through the blackboard
 * // In DriveHandler:
 * blackboard.put("drive.target_reached", true);
 * blackboard.put("drive.current_position", new Pose2D(x, y, heading));
 *
 * // In StateMachine:
 * if (blackboard.get("drive.target_reached", false)) {
 *     triggerEvent(StateMachine.Event.TARGET_REACHED);
 * }
 *
 * // In ShooterHandler:
 * double batteryVoltage = blackboard.get("robot.battery_voltage", 12.0);
 * adjustPowerForVoltage(batteryVoltage);
 * ```
 *
 * PERFORMANCE MONITORING:
 * ======================
 *
 * ```java
 * // Enable debug mode for detailed telemetry
 * stateMachine.setDebugMode(true);
 *
 * // Monitor state machine performance
 * telemetry.addData("SM Statistics", stateMachine.getStatistics());
 *
 * // Check for performance issues
 * if (stateMachine.getAverageUpdateTime() > 0.020) { // > 20ms
 *     logger.warn("State machine update time is high: " +
 *                 (stateMachine.getAverageUpdateTime() * 1000) + "ms");
 * }
 * ```
 *
 * TESTING AND DEBUGGING:
 * ======================
 *
 * ```java
 * // Force state changes for testing (use carefully!)
 * if (gamepad2.dpad_up) {
 *     stateMachine.forceState(StateMachine.State.COLLECTING);
 * }
 *
 * // Monitor specific events
 * Logger.getInstance().addLogListener((level, message) -> {
 *     if (message.contains("State transition")) {
 *         telemetry.addLine("TRANSITION: " + message);
 *     }
 * });
 * ```
 *
 * Remember to:
 * - Always call initialize() before start()
 * - Call update() in your main loop
 * - Handle exceptions appropriately
 * - Test state transitions thoroughly
 * - Monitor performance in competition
 * - Use the Logger for debugging
 * - Leverage the Blackboard for inter-subsystem communication
 */
