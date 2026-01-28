package org.firstinspires.ftc.teamcode.util.aurora.localization;

/**
 * Lightweight 3x3 matrix implementation for covariance operations.
 * Optimized for symmetric matrices (covariance matrices are always symmetric).
 */
public class Matrix3x3 {
    
    // Matrix elements stored in row-major order
    // [0][1][2]
    // [3][4][5]
    // [6][7][8]
    private final double[] data = new double[9];
    
    /**
     * Create zero matrix
     */
    public Matrix3x3() {
        // All zeros by default
    }
    
    /**
     * Create matrix from array (row-major order)
     */
    public Matrix3x3(double[] values) {
        if (values.length != 9) {
            throw new IllegalArgumentException("Matrix3x3 requires 9 values");
        }
        System.arraycopy(values, 0, data, 0, 9);
    }
    
    /**
     * Create diagonal matrix
     */
    public static Matrix3x3 diagonal(double a, double b, double c) {
        Matrix3x3 m = new Matrix3x3();
        m.set(0, 0, a);
        m.set(1, 1, b);
        m.set(2, 2, c);
        return m;
    }
    
    /**
     * Create identity matrix
     */
    public static Matrix3x3 identity() {
        return diagonal(1.0, 1.0, 1.0);
    }
    
    /**
     * Get element at (row, col)
     */
    public double get(int row, int col) {
        return data[row * 3 + col];
    }
    
    /**
     * Set element at (row, col)
     */
    public void set(int row, int col, double value) {
        data[row * 3 + col] = value;
    }
    
    /**
     * Matrix addition: this + other
     */
    public Matrix3x3 add(Matrix3x3 other) {
        Matrix3x3 result = new Matrix3x3();
        for (int i = 0; i < 9; i++) {
            result.data[i] = this.data[i] + other.data[i];
        }
        return result;
    }
    
    /**
     * Matrix subtraction: this - other
     */
    public Matrix3x3 subtract(Matrix3x3 other) {
        Matrix3x3 result = new Matrix3x3();
        for (int i = 0; i < 9; i++) {
            result.data[i] = this.data[i] - other.data[i];
        }
        return result;
    }
    
    /**
     * Matrix multiplication: this * other
     */
    public Matrix3x3 multiply(Matrix3x3 other) {
        Matrix3x3 result = new Matrix3x3();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double sum = 0.0;
                for (int k = 0; k < 3; k++) {
                    sum += this.get(i, k) * other.get(k, j);
                }
                result.set(i, j, sum);
            }
        }
        return result;
    }
    
    /**
     * Scalar multiplication: this * scalar
     */
    public Matrix3x3 multiply(double scalar) {
        Matrix3x3 result = new Matrix3x3();
        for (int i = 0; i < 9; i++) {
            result.data[i] = this.data[i] * scalar;
        }
        return result;
    }
    
    /**
     * Matrix-vector multiplication: this * [x, y, z]^T
     * @return result as [x, y, z]
     */
    public double[] multiply(double x, double y, double z) {
        double[] result = new double[3];
        result[0] = get(0,0)*x + get(0,1)*y + get(0,2)*z;
        result[1] = get(1,0)*x + get(1,1)*y + get(1,2)*z;
        result[2] = get(2,0)*x + get(2,1)*y + get(2,2)*z;
        return result;
    }
    
    /**
     * Matrix transpose
     */
    public Matrix3x3 transpose() {
        Matrix3x3 result = new Matrix3x3();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.set(j, i, this.get(i, j));
            }
        }
        return result;
    }
    
    /**
     * Matrix inversion using cofactor method
     * For 3x3 matrices, this is more efficient than Gaussian elimination
     */
    public Matrix3x3 inverse() {
        double det = determinant();
        if (Math.abs(det) < 1e-10) {
            throw new ArithmeticException("Matrix is singular (determinant near zero)");
        }
        
        Matrix3x3 result = new Matrix3x3();
        
        // Compute cofactor matrix
        result.set(0, 0, get(1,1)*get(2,2) - get(1,2)*get(2,1));
        result.set(0, 1, -(get(1,0)*get(2,2) - get(1,2)*get(2,0)));
        result.set(0, 2, get(1,0)*get(2,1) - get(1,1)*get(2,0));
        
        result.set(1, 0, -(get(0,1)*get(2,2) - get(0,2)*get(2,1)));
        result.set(1, 1, get(0,0)*get(2,2) - get(0,2)*get(2,0));
        result.set(1, 2, -(get(0,0)*get(2,1) - get(0,1)*get(2,0)));
        
        result.set(2, 0, get(0,1)*get(1,2) - get(0,2)*get(1,1));
        result.set(2, 1, -(get(0,0)*get(1,2) - get(0,2)*get(1,0)));
        result.set(2, 2, get(0,0)*get(1,1) - get(0,1)*get(1,0));
        
        // Transpose and divide by determinant
        result = result.transpose();
        result = result.multiply(1.0 / det);
        
        return result;
    }
    
    /**
     * Compute determinant
     */
    public double determinant() {
        return get(0,0) * (get(1,1)*get(2,2) - get(1,2)*get(2,1))
             - get(0,1) * (get(1,0)*get(2,2) - get(1,2)*get(2,0))
             + get(0,2) * (get(1,0)*get(2,1) - get(1,1)*get(2,0));
    }
    
    /**
     * Compute trace (sum of diagonal elements)
     */
    public double trace() {
        return get(0,0) + get(1,1) + get(2,2);
    }
    
    /**
     * Enforce symmetry (for covariance matrices)
     * Sets M = (M + M^T) / 2
     */
    public void enforceSymmetry() {
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                double avg = (get(i, j) + get(j, i)) / 2.0;
                set(i, j, avg);
                set(j, i, avg);
            }
        }
    }
    
    /**
     * Copy this matrix
     */
    public Matrix3x3 copy() {
        Matrix3x3 result = new Matrix3x3();
        System.arraycopy(this.data, 0, result.data, 0, 9);
        return result;
    }
    
    /**
     * Check if matrix is positive definite (required for covariance)
     * Simple check: all diagonal elements > 0 and determinant > 0
     */
    public boolean isPositiveDefinite() {
        return get(0,0) > 0 && get(1,1) > 0 && get(2,2) > 0 && determinant() > 0;
    }
    
    @Override
    public String toString() {
        return String.format("[%.2f %.2f %.2f]\n[%.2f %.2f %.2f]\n[%.2f %.2f %.2f]",
            get(0,0), get(0,1), get(0,2),
            get(1,0), get(1,1), get(1,2),
            get(2,0), get(2,1), get(2,2));
    }
}
