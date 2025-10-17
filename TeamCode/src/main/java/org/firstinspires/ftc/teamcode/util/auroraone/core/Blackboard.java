package org.firstinspires.ftc.teamcode.util.auroraone.core;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.Map;
import java.util.List;
import java.util.Set;
import java.util.HashSet;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * AURORA ONE - Blackboard Pattern Implementation
 *
 * This class serves as a centralized data repository for sharing information between different subsystems of the robot.
 * It allows various components to read from and write to a common set of variables, facilitating communication
 * and coordination. The blackboard pattern helps in decoupling subsystems, making the overall architecture more modular and easier to manage.
 *
 * Features:
 * - Thread-safe concurrent access to shared data
 * - Type-safe data storage and retrieval
 * - Event notification system for data changes
 * - Data validation and constraints
 * - Performance monitoring and metrics
 * - Namespace support for organizing data
 * - Data persistence and history tracking
 * - Debugging and telemetry support
 *
 * Usage Example:
 * ```java
 * // Writing data
 * Blackboard.getInstance().put("robot.position.x", 24.5);
 * Blackboard.getInstance().put("sensors.distance.front", 12.0);
 *
 * // Reading data
 * double robotX = Blackboard.getInstance().get("robot.position.x", 0.0);
 * double frontDistance = Blackboard.getInstance().get("sensors.distance.front", Double.MAX_VALUE);
 *
 * // Subscribing to changes
 * Blackboard.getInstance().subscribe("robot.position.*", (key, oldValue, newValue) -> {
 *     System.out.println("Position updated: " + key + " = " + newValue);
 * });
 * ```
 */
public class Blackboard {

    // Singleton instance
    private static volatile Blackboard instance;
    private static final Object lock = new Object();

    // Core data storage - thread-safe concurrent map
    private final ConcurrentHashMap<String, BlackboardEntry> dataStore;

    // Event notification system
    private final CopyOnWriteArrayList<DataChangeListener> globalListeners;
    private final ConcurrentHashMap<String, CopyOnWriteArrayList<DataChangeListener>> keyListeners;
    private final ConcurrentHashMap<String, CopyOnWriteArrayList<DataChangeListener>> patternListeners;

    // Performance and monitoring
    private final ConcurrentHashMap<String, Long> accessCounts;
    private final ConcurrentHashMap<String, Long> lastAccessTimes;
    private final ReadWriteLock metricsLock;

    // Data validation
    private final ConcurrentHashMap<String, DataValidator> validators;

    // Configuration
    private boolean enableHistory = true;
    private int maxHistorySize = 100;
    private boolean enableMetrics = true;
    private boolean enableValidation = true;

    /**
     * Private constructor for singleton pattern
     */
    private Blackboard() {
        this.dataStore = new ConcurrentHashMap<>();
        this.globalListeners = new CopyOnWriteArrayList<>();
        this.keyListeners = new ConcurrentHashMap<>();
        this.patternListeners = new ConcurrentHashMap<>();
        this.accessCounts = new ConcurrentHashMap<>();
        this.lastAccessTimes = new ConcurrentHashMap<>();
        this.metricsLock = new ReentrantReadWriteLock();
        this.validators = new ConcurrentHashMap<>();

        // Initialize with common robot data structure
        initializeCommonKeys();
    }

    /**
     * Get singleton instance of Blackboard
     */
    public static Blackboard getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new Blackboard();
                }
            }
        }
        return instance;
    }

    /**
     * Initialize common keys used by Aurora One subsystems
     */
    private void initializeCommonKeys() {
        // Robot state
        put("robot.state.current", "INITIALIZING");
        put("robot.state.previous", "UNKNOWN");
        put("robot.state.transition_time", System.currentTimeMillis());

        // Robot position and orientation
        put("robot.position.x", 0.0);
        put("robot.position.y", 0.0);
        put("robot.position.heading", 0.0);
        put("robot.position.confidence", 0.0);
        put("robot.position.last_update", 0L);

        // Robot velocity
        put("robot.velocity.x", 0.0);
        put("robot.velocity.y", 0.0);
        put("robot.velocity.angular", 0.0);

        // Hardware status
        put("hardware.initialized", false);
        put("hardware.drive.ready", false);
        put("hardware.sensors.ready", false);
        put("hardware.vision.ready", false);

        // Sensor data
        put("sensors.imu.heading", 0.0);
        put("sensors.imu.pitch", 0.0);
        put("sensors.imu.roll", 0.0);
        put("sensors.distance.front", Double.MAX_VALUE);
        put("sensors.distance.back", Double.MAX_VALUE);
        put("sensors.distance.left", Double.MAX_VALUE);
        put("sensors.distance.right", Double.MAX_VALUE);

        // Vision system
        put("vision.target.detected", false);
        put("vision.target.x", 0.0);
        put("vision.target.y", 0.0);
        put("vision.target.confidence", 0.0);

        // Drive system
        put("drive.mode", "MANUAL");
        put("drive.target.x", 0.0);
        put("drive.target.y", 0.0);
        put("drive.target.heading", 0.0);
        put("drive.auto.active", false);

        // Game-specific data
        put("game.alliance", "UNKNOWN");
        put("game.match_time", 0.0);
        put("game.autonomous", false);
        put("game.endgame", false);

        // Performance metrics
        put("performance.loop_time", 0.0);
        put("performance.cpu_usage", 0.0);
        put("performance.memory_usage", 0.0);
    }

    // === CORE DATA OPERATIONS ===

    /**
     * Store a value in the blackboard
     */
    public <T> void put(String key, T value) {
        if (key == null) {
            throw new IllegalArgumentException("Key cannot be null");
        }

        // Validate data if validation is enabled
        if (enableValidation && validators.containsKey(key)) {
            DataValidator validator = validators.get(key);
            if (!validator.isValid(value)) {
                throw new IllegalArgumentException("Value validation failed for key: " + key);
            }
        }

        // Get old value for change notification
        BlackboardEntry oldEntry = dataStore.get(key);
        Object oldValue = oldEntry != null ? oldEntry.getValue() : null;

        // Create new entry
        BlackboardEntry newEntry = new BlackboardEntry(value, System.currentTimeMillis());

        // Add to history if enabled
        if (enableHistory && oldEntry != null) {
            newEntry.addToHistory(oldEntry.getHistory());
        }

        // Store the new entry
        dataStore.put(key, newEntry);

        // Update metrics
        if (enableMetrics) {
            updateMetrics(key);
        }

        // Notify listeners
        notifyListeners(key, oldValue, value);
    }

    /**
     * Retrieve a value from the blackboard with default fallback
     */
    @SuppressWarnings("unchecked")
    public <T> T get(String key, T defaultValue) {
        if (key == null) {
            return defaultValue;
        }

        BlackboardEntry entry = dataStore.get(key);
        if (entry == null) {
            return defaultValue;
        }

        // Update metrics
        if (enableMetrics) {
            updateMetrics(key);
        }

        try {
            return (T) entry.getValue();
        } catch (ClassCastException e) {
            // Return default if type casting fails
            return defaultValue;
        }
    }

    /**
     * Retrieve a value from the blackboard
     */
    public Object get(String key) {
        return get(key, null);
    }

    /**
     * Check if a key exists in the blackboard
     */
    public boolean containsKey(String key) {
        return dataStore.containsKey(key);
    }

    /**
     * Remove a key from the blackboard
     */
    public Object remove(String key) {
        BlackboardEntry entry = dataStore.remove(key);

        if (entry != null) {
            // Notify listeners about removal
            notifyListeners(key, entry.getValue(), null);
            return entry.getValue();
        }

        return null;
    }

    /**
     * Get all keys in the blackboard
     */
    public Set<String> getKeys() {
        return new HashSet<>(dataStore.keySet());
    }

    /**
     * Get keys matching a pattern (simple wildcard support)
     */
    public Set<String> getKeys(String pattern) {
        Set<String> matchingKeys = new HashSet<>();
        String regex = pattern.replace("*", ".*").replace("?", ".");

        for (String key : dataStore.keySet()) {
            if (key.matches(regex)) {
                matchingKeys.add(key);
            }
        }

        return matchingKeys;
    }

    /**
     * Clear all data from the blackboard
     */
    public void clear() {
        Set<String> keys = new HashSet<>(dataStore.keySet());
        dataStore.clear();

        // Notify listeners about clearing
        for (String key : keys) {
            notifyListeners(key, dataStore.get(key), null);
        }

        // Clear metrics
        if (enableMetrics) {
            accessCounts.clear();
            lastAccessTimes.clear();
        }
    }

    // === EVENT NOTIFICATION SYSTEM ===

    /**
     * Interface for data change listeners
     */
    public interface DataChangeListener {
        void onDataChanged(String key, Object oldValue, Object newValue);
    }

    /**
     * Subscribe to all data changes
     */
    public void subscribe(DataChangeListener listener) {
        globalListeners.add(listener);
    }

    /**
     * Subscribe to changes for a specific key
     */
    public void subscribe(String key, DataChangeListener listener) {
        keyListeners.computeIfAbsent(key, k -> new CopyOnWriteArrayList<>()).add(listener);
    }

    /**
     * Subscribe to changes matching a pattern
     */
    public void subscribePattern(String pattern, DataChangeListener listener) {
        patternListeners.computeIfAbsent(pattern, k -> new CopyOnWriteArrayList<>()).add(listener);
    }

    /**
     * Unsubscribe from all changes
     */
    public void unsubscribe(DataChangeListener listener) {
        globalListeners.remove(listener);

        // Remove from key-specific listeners
        for (CopyOnWriteArrayList<DataChangeListener> listeners : keyListeners.values()) {
            listeners.remove(listener);
        }

        // Remove from pattern listeners
        for (CopyOnWriteArrayList<DataChangeListener> listeners : patternListeners.values()) {
            listeners.remove(listener);
        }
    }

    /**
     * Unsubscribe from specific key
     */
    public void unsubscribe(String key, DataChangeListener listener) {
        CopyOnWriteArrayList<DataChangeListener> listeners = keyListeners.get(key);
        if (listeners != null) {
            listeners.remove(listener);
        }
    }

    /**
     * Notify all relevant listeners about data changes
     */
    private void notifyListeners(String key, Object oldValue, Object newValue) {
        // Notify global listeners
        for (DataChangeListener listener : globalListeners) {
            try {
                listener.onDataChanged(key, oldValue, newValue);
            } catch (Exception e) {
                // Log error but continue notifying other listeners
                System.err.println("Error in global listener: " + e.getMessage());
            }
        }

        // Notify key-specific listeners
        CopyOnWriteArrayList<DataChangeListener> keySpecificListeners = keyListeners.get(key);
        if (keySpecificListeners != null) {
            for (DataChangeListener listener : keySpecificListeners) {
                try {
                    listener.onDataChanged(key, oldValue, newValue);
                } catch (Exception e) {
                    System.err.println("Error in key-specific listener: " + e.getMessage());
                }
            }
        }

        // Notify pattern listeners
        for (Map.Entry<String, CopyOnWriteArrayList<DataChangeListener>> entry : patternListeners.entrySet()) {
            String pattern = entry.getKey();
            String regex = pattern.replace("*", ".*").replace("?", ".");

            if (key.matches(regex)) {
                for (DataChangeListener listener : entry.getValue()) {
                    try {
                        listener.onDataChanged(key, oldValue, newValue);
                    } catch (Exception e) {
                        System.err.println("Error in pattern listener: " + e.getMessage());
                    }
                }
            }
        }
    }

    // === DATA VALIDATION ===

    /**
     * Interface for data validators
     */
    public interface DataValidator {
        boolean isValid(Object value);
        String getErrorMessage();
    }

    /**
     * Add a validator for a specific key
     */
    public void addValidator(String key, DataValidator validator) {
        validators.put(key, validator);
    }

    /**
     * Remove validator for a key
     */
    public void removeValidator(String key) {
        validators.remove(key);
    }

    /**
     * Common validators
     */
    public static class Validators {
        public static DataValidator range(double min, double max) {
            return new DataValidator() {
                @Override
                public boolean isValid(Object value) {
                    if (!(value instanceof Number)) return false;
                    double d = ((Number) value).doubleValue();
                    return d >= min && d <= max;
                }

                @Override
                public String getErrorMessage() {
                    return "Value must be between " + min + " and " + max;
                }
            };
        }

        public static DataValidator notNull() {
            return new DataValidator() {
                @Override
                public boolean isValid(Object value) {
                    return value != null;
                }

                @Override
                public String getErrorMessage() {
                    return "Value cannot be null";
                }
            };
        }

        public static DataValidator instanceOf(Class<?> clazz) {
            return new DataValidator() {
                @Override
                public boolean isValid(Object value) {
                    return value == null || clazz.isInstance(value);
                }

                @Override
                public String getErrorMessage() {
                    return "Value must be instance of " + clazz.getSimpleName();
                }
            };
        }
    }

    // === PERFORMANCE METRICS ===

    /**
     * Update access metrics for a key
     */
    private void updateMetrics(String key) {
        metricsLock.writeLock().lock();
        try {
            accessCounts.put(key, accessCounts.getOrDefault(key, 0L) + 1);
            lastAccessTimes.put(key, System.currentTimeMillis());
        } finally {
            metricsLock.writeLock().unlock();
        }
    }

    /**
     * Get access count for a key
     */
    public long getAccessCount(String key) {
        metricsLock.readLock().lock();
        try {
            return accessCounts.getOrDefault(key, 0L);
        } finally {
            metricsLock.readLock().unlock();
        }
    }

    /**
     * Get last access time for a key
     */
    public long getLastAccessTime(String key) {
        metricsLock.readLock().lock();
        try {
            return lastAccessTimes.getOrDefault(key, 0L);
        } finally {
            metricsLock.readLock().unlock();
        }
    }

    /**
     * Get total number of stored entries
     */
    public int size() {
        return dataStore.size();
    }

    /**
     * Get memory usage statistics
     */
    public BlackboardStats getStats() {
        metricsLock.readLock().lock();
        try {
            return new BlackboardStats(
                dataStore.size(),
                globalListeners.size(),
                keyListeners.size() + patternListeners.size(),
                accessCounts.values().stream().mapToLong(Long::longValue).sum(),
                validators.size()
            );
        } finally {
            metricsLock.readLock().unlock();
        }
    }

    // === CONFIGURATION ===

    /**
     * Enable or disable history tracking
     */
    public void setHistoryEnabled(boolean enabled) {
        this.enableHistory = enabled;
    }

    /**
     * Set maximum history size per key
     */
    public void setMaxHistorySize(int size) {
        this.maxHistorySize = Math.max(1, size);
    }

    /**
     * Enable or disable metrics collection
     */
    public void setMetricsEnabled(boolean enabled) {
        this.enableMetrics = enabled;
        if (!enabled) {
            accessCounts.clear();
            lastAccessTimes.clear();
        }
    }

    /**
     * Enable or disable data validation
     */
    public void setValidationEnabled(boolean enabled) {
        this.enableValidation = enabled;
    }

    // === UTILITY METHODS ===

    /**
     * Get data entry with metadata
     */
    public BlackboardEntry getEntry(String key) {
        return dataStore.get(key);
    }

    /**
     * Get value history for a key
     */
    public List<Object> getHistory(String key) {
        BlackboardEntry entry = dataStore.get(key);
        return entry != null ? entry.getHistory() : new CopyOnWriteArrayList<>();
    }

    /**
     * Export all data as a map (snapshot)
     */
    public Map<String, Object> exportData() {
        Map<String, Object> export = new ConcurrentHashMap<>();
        for (Map.Entry<String, BlackboardEntry> entry : dataStore.entrySet()) {
            export.put(entry.getKey(), entry.getValue().getValue());
        }
        return export;
    }

    /**
     * Import data from a map
     */
    public void importData(Map<String, Object> data) {
        for (Map.Entry<String, Object> entry : data.entrySet()) {
            put(entry.getKey(), entry.getValue());
        }
    }

    /**
     * Data entry wrapper with metadata
     */
    public static class BlackboardEntry {
        private final Object value;
        private final long timestamp;
        private final CopyOnWriteArrayList<Object> history;

        public BlackboardEntry(Object value, long timestamp) {
            this.value = value;
            this.timestamp = timestamp;
            this.history = new CopyOnWriteArrayList<>();
        }

        public Object getValue() { return value; }
        public long getTimestamp() { return timestamp; }
        public List<Object> getHistory() { return new CopyOnWriteArrayList<>(history); }

        public void addToHistory(List<Object> previousHistory) {
            history.addAll(previousHistory);
            history.add(value);

            // Limit history size
            while (history.size() > 100) {
                history.remove(0);
            }
        }
    }

    /**
     * Statistics container
     */
    public static class BlackboardStats {
        public final int totalEntries;
        public final int globalListeners;
        public final int totalListeners;
        public final long totalAccesses;
        public final int validators;

        public BlackboardStats(int totalEntries, int globalListeners, int totalListeners,
                              long totalAccesses, int validators) {
            this.totalEntries = totalEntries;
            this.globalListeners = globalListeners;
            this.totalListeners = totalListeners;
            this.totalAccesses = totalAccesses;
            this.validators = validators;
        }

        @Override
        public String toString() {
            return String.format("BlackboardStats{entries=%d, listeners=%d, accesses=%d, validators=%d}",
                               totalEntries, totalListeners, totalAccesses, validators);
        }
    }
}
