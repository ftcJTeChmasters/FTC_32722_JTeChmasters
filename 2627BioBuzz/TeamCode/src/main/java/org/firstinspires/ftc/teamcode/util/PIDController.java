package org.firstinspires.ftc.teamcode.util;

/**
 * PIDF Controller for motor velocity and position control.
 * <p>
 * Supports both velocity control (feedforward = kF * targetVelocity) and
 * position control (feedforward = kF * targetVelocity where targetVelocity
 * is typically 0 for holding position, or can be set via setVelocityFeedforward).
 * <p>
 * Features:
 * <ul>
 *   <li>Anti-windup integral clamping</li>
 *   <li>Configurable output bounds</li>
 *   <li>Output saturation detection</li>
 *   <li>Derivative kick prevention (derivative on measurement, not error)</li>
 * </ul>
 */
public class PIDController {
    private double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastMeasurement = 0;
    private long lastTimeNanoseconds = 0;
    private double integralLimit = Double.MAX_VALUE; // anti-windup clamp
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    private boolean continuousInput = false;
    private double continuousInputMinimum = 0;
    private double continuousInputMaximum = 360;

    // Feedforward for velocity control mode
    private double velocityFeedforward = 0;
    private boolean useVelocityFeedforward = false;

    // Status flags
    private boolean saturated = false;

    /**
     * Creates a new PIDF controller with the given gains.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kF feedforward gain (for velocity control: kF * targetVelocity)
     */
    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Sets all PIDF gains at once.
     */
    public void setPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Sets the integral limit for anti-windup.
     *
     * @param limit maximum absolute value of the integral term
     */
    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }

    /**
     * Sets the output bounds.
     *
     * @param min minimum output value
     * @param max maximum output value
     */
    public void setOutputBounds(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    /**
     * Enables continuous input mode (e.g., for angle wrapping).
     * <p>
     * When enabled, the error is computed using the shortest path around
     * the circle defined by [min, max).
     *
     * @param min minimum input value (inclusive)
     * @param max maximum input value (exclusive)
     */
    public void setContinuousInput(double min, double max) {
        this.continuousInput = true;
        this.continuousInputMinimum = min;
        this.continuousInputMaximum = max;
    }

    /**
     * Disables continuous input mode.
     */
    public void disableContinuousInput() {
        this.continuousInput = false;
    }

    /**
     * Sets a constant velocity feedforward value.
     * <p>
     * When using velocity feedforward, the feedforward term is
     * {@code kF * velocityFeedforward} instead of {@code kF * targetVelocity}.
     * This is useful for position control where you want to feed forward
     * a known velocity profile.
     *
     * @param velocityFeedforward the velocity feedforward value
     */
    public void setVelocityFeedforward(double velocityFeedforward) {
        this.velocityFeedforward = velocityFeedforward;
        this.useVelocityFeedforward = true;
    }

    /**
     * Disables custom velocity feedforward, reverting to kF * targetVelocity.
     */
    public void disableVelocityFeedforward() {
        this.useVelocityFeedforward = false;
    }

    /**
     * Calculates the PID output.
     * <p>
     * For velocity control: call with target = targetVelocity, current = currentVelocity
     * For position control: call with target = targetPosition, current = currentPosition
     *
     * @param target  target value (position or velocity)
     * @param current current measured value
     * @return bounded PID output
     */
    public double calculate(double target, double current) {
        double error = calculateError(target, current);
        long now = System.nanoTime();

        if (lastTimeNanoseconds == 0) {
            lastTimeNanoseconds = now;
            lastError = error;
            lastMeasurement = current;
            double output = (kP * error) + calculateFeedforward(target);
            saturated = false;
            return clamp(output);
        }

        double dt = (now - lastTimeNanoseconds) / 1e9;
        if (dt <= 0) {
            double output = (kP * error) + (kI * integralSum) + calculateFeedforward(target);
            saturated = false;
            return clamp(output);
        }

        lastTimeNanoseconds = now;

        // Integral with anti-windup
        integralSum += error * dt;
        integralSum = clampIntegral(integralSum);

        // Derivative on measurement (prevents derivative kick on setpoint changes)
        double derivative = (lastMeasurement - current) / dt;
        lastMeasurement = current;
        lastError = error;

        double feedforward = calculateFeedforward(target);
        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + feedforward;

        // Check for saturation
        double clampedOutput = clamp(output);
        saturated = (output != clampedOutput);

        return clampedOutput;
    }

    /**
     * Calculates error with continuous input handling if enabled.
     */
    private double calculateError(double target, double current) {
        double error = target - current;
        if (continuousInput) {
            double range = continuousInputMaximum - continuousInputMinimum;
            error = ((error + range / 2) % range + range) % range - range / 2;
        }
        return error;
    }

    /**
     * Calculates the feedforward term.
     * <p>
     * If velocity feedforward is enabled: kF * velocityFeedforward
     * Otherwise (velocity control mode): kF * target
     */
    private double calculateFeedforward(double target) {
        if (useVelocityFeedforward) {
            return kF * velocityFeedforward;
        }
        // For velocity control, feedforward is kF * targetVelocity
        // For position control with no velocity FF, this should be 0
        // The caller should use setVelocityFeedforward for position control
        return kF * target;
    }

    private double clampIntegral(double value) {
        return Math.max(-integralLimit, Math.min(integralLimit, value));
    }

    private double clamp(double value) {
        return Math.max(minOutput, Math.min(maxOutput, value));
    }

    /**
     * Resets the controller state (integral, derivative, time).
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastMeasurement = 0;
        lastTimeNanoseconds = 0;
        saturated = false;
    }

    /**
     * Returns whether the output is currently saturated (hitting bounds).
     *
     * @return true if the last calculate() call hit the output bounds
     */
    public boolean isSaturated() {
        return saturated;
    }

    /**
     * Returns the current integral term value.
     *
     * @return the integral sum
     */
    public double getIntegral() {
        return integralSum;
    }

    /**
     * Returns the proportional gain.
     */
    public double getKP() {
        return kP;
    }

    /**
     * Returns the integral gain.
     */
    public double getKI() {
        return kI;
    }

    /**
     * Returns the derivative gain.
     */
    public double getKD() {
        return kD;
    }

    /**
     * Returns the feedforward gain.
     */
    public double getKF() {
        return kF;
    }

    /**
     * Sets the proportional gain.
     */
    public void setKP(double kP) {
        this.kP = kP;
    }

    /**
     * Sets the integral gain.
     */
    public void setKI(double kI) {
        this.kI = kI;
    }

    /**
     * Sets the derivative gain.
     */
    public void setKD(double kD) {
        this.kD = kD;
    }

    /**
     * Sets the feedforward gain.
     */
    public void setKF(double kF) {
        this.kF = kF;
    }
}