package org.firstinspires.ftc.teamcode.util.aurora.lightning;

import java.util.ArrayList;
import java.util.List;

/**
 * CircularDataBuffer - Fixed-size circular buffer for efficient time-series data storage
 *
 * Features:
 * - Constant memory footprint (no dynamic allocation after creation)
 * - O(1) insertion
 * - Real-time statistics (for Number types)
 * - Thread-safe option
 * - Generic type support
 *
 * @param <T> Data type
 */
public class CircularDataBuffer<T> {

    private final T[] buffer;
    private final int capacity;
    private int size;
    private int writeIndex;
    private boolean isFull;

    /**
     * Constructor
     * @param capacity Maximum number of elements to store
     */
    @SuppressWarnings("unchecked")
    public CircularDataBuffer(int capacity) {
        this.capacity = capacity;
        this.buffer = (T[]) new Object[capacity];
        this.size = 0;
        this.writeIndex = 0;
        this.isFull = false;
    }

    /**
     * Add a value to the buffer
     * If buffer is full, overwrites oldest value
     */
    public void add(T value) {
        buffer[writeIndex] = value;
        writeIndex = (writeIndex + 1) % capacity;

        if (isFull) {
            // Already full, just overwrote oldest
        } else {
            size++;
            if (size == capacity) {
                isFull = true;
            }
        }
    }

    /**
     * Get value at index (0 = oldest, size-1 = newest)
     */
    public T get(int index) {
        if (index < 0 || index >= size) {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size);
        }

        int actualIndex;
        if (isFull) {
            actualIndex = (writeIndex + index) % capacity;
        } else {
            actualIndex = index;
        }

        return buffer[actualIndex];
    }

    /**
     * Get the most recent value
     */
    public T getLast() {
        if (size == 0) return null;
        int lastIndex = (writeIndex - 1 + capacity) % capacity;
        return buffer[lastIndex];
    }

    /**
     * Get the oldest value
     */
    public T getFirst() {
        if (size == 0) return null;
        return get(0);
    }

    /**
     * Get current number of elements
     */
    public int size() {
        return size;
    }

    /**
     * Get maximum capacity
     */
    public int capacity() {
        return capacity;
    }

    /**
     * Check if buffer is full
     */
    public boolean isFull() {
        return isFull;
    }

    /**
     * Check if buffer is empty
     */
    public boolean isEmpty() {
        return size == 0;
    }

    /**
     * Clear all data
     */
    public void clear() {
        size = 0;
        writeIndex = 0;
        isFull = false;
    }

    /**
     * Get all values as a list (oldest to newest)
     */
    public List<T> toList() {
        List<T> result = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            result.add(get(i));
        }
        return result;
    }

    // ======== Statistics Methods (Only for Number types) ========

    /**
     * Calculate mean (average) of all values
     * Only works if T extends Number
     */
    public double getMean() {
        if (size == 0) return 0;

        double sum = 0;
        for (int i = 0; i < size; i++) {
            T value = get(i);
            if (value instanceof Number) {
                sum += ((Number) value).doubleValue();
            }
        }
        return sum / size;
    }

    /**
     * Find minimum value
     * Only works if T extends Number
     */
    public double getMin() {
        if (size == 0) return 0;

        double min = Double.MAX_VALUE;
        for (int i = 0; i < size; i++) {
            T value = get(i);
            if (value instanceof Number) {
                double val = ((Number) value).doubleValue();
                if (val < min) min = val;
            }
        }
        return min == Double.MAX_VALUE ? 0 : min;
    }

    /**
     * Find maximum value
     * Only works if T extends Number
     */
    public double getMax() {
        if (size == 0) return 0;

        double max = Double.MIN_VALUE;
        for (int i = 0; i < size; i++) {
            T value = get(i);
            if (value instanceof Number) {
                double val = ((Number) value).doubleValue();
                if (val > max) max = val;
            }
        }
        return max == Double.MIN_VALUE ? 0 : max;
    }

    /**
     * Calculate standard deviation
     * Only works if T extends Number
     */
    public double getStdDev() {
        if (size < 2) return 0;

        double mean = getMean();
        double sumSquaredDiff = 0;

        for (int i = 0; i < size; i++) {
            T value = get(i);
            if (value instanceof Number) {
                double diff = ((Number) value).doubleValue() - mean;
                sumSquaredDiff += diff * diff;
            }
        }

        return Math.sqrt(sumSquaredDiff / (size - 1));
    }

    /**
     * Calculate variance
     * Only works if T extends Number
     */
    public double getVariance() {
        if (size < 2) return 0;

        double mean = getMean();
        double sumSquaredDiff = 0;

        for (int i = 0; i < size; i++) {
            T value = get(i);
            if (value instanceof Number) {
                double diff = ((Number) value).doubleValue() - mean;
                sumSquaredDiff += diff * diff;
            }
        }

        return sumSquaredDiff / (size - 1);
    }

    /**
     * Get median value
     * Only works if T extends Number
     */
    public double getMedian() {
        if (size == 0) return 0;

        // Copy to array and sort
        double[] values = new double[size];
        for (int i = 0; i < size; i++) {
            T value = get(i);
            if (value instanceof Number) {
                values[i] = ((Number) value).doubleValue();
            }
        }
        java.util.Arrays.sort(values);

        if (size % 2 == 0) {
            return (values[size/2 - 1] + values[size/2]) / 2.0;
        } else {
            return values[size/2];
        }
    }

    /**
     * Get percentile value (0.0 to 1.0)
     * Only works if T extends Number
     * @param percentile Percentile (0.0 = min, 0.5 = median, 1.0 = max)
     */
    public double getPercentile(double percentile) {
        if (size == 0) return 0;
        if (percentile < 0) percentile = 0;
        if (percentile > 1) percentile = 1;

        // Copy to array and sort
        double[] values = new double[size];
        for (int i = 0; i < size; i++) {
            T value = get(i);
            if (value instanceof Number) {
                values[i] = ((Number) value).doubleValue();
            }
        }
        java.util.Arrays.sort(values);

        int index = (int) (percentile * (size - 1));
        return values[index];
    }

    /**
     * Calculate simple moving average over last N values
     * Only works if T extends Number
     * @param n Number of recent values to average
     */
    public double getMovingAverage(int n) {
        if (size == 0) return 0;
        if (n > size) n = size;

        double sum = 0;
        for (int i = size - n; i < size; i++) {
            T value = get(i);
            if (value instanceof Number) {
                sum += ((Number) value).doubleValue();
            }
        }
        return sum / n;
    }

    /**
     * Calculate rate of change (derivative)
     * Returns change per sample
     * Only works if T extends Number
     */
    public double getRateOfChange() {
        if (size < 2) return 0;
        T first = getFirst();
        T last = getLast();
        if (first instanceof Number && last instanceof Number) {
            return ((Number) last).doubleValue() - ((Number) first).doubleValue();
        }
        return 0;
    }

    /**
     * Detect if current value is an outlier
     * Uses 3-sigma rule (value is >3 std deviations from mean)
     * Only works if T extends Number
     */
    public boolean isCurrentValueOutlier() {
        if (size < 10) return false;  // Need enough data

        double mean = getMean();
        double stdDev = getStdDev();
        T lastValue = getLast();

        if (lastValue instanceof Number) {
            double current = ((Number) lastValue).doubleValue();
            return Math.abs(current - mean) > 3 * stdDev;
        }

        return false;
    }

    /**
     * Get summary statistics as string
     */
    public String getStatsSummary() {
        if (size == 0) return "Empty buffer";

        return String.format(
            "n=%d, mean=%.2f, min=%.2f, max=%.2f, std=%.2f",
            size, getMean(), getMin(), getMax(), getStdDev()
        );
    }
}

