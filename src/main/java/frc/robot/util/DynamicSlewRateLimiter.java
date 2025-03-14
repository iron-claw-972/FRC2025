// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 * Edited by 972 to be "dynamic", that is, the slew rate can be modified on the fly.
 * Additionally, it can be set to be continuous on a range, useful for angles.
 */
public class DynamicSlewRateLimiter {
    private double positiveRateLimit;
    private double negativeRateLimit;
    private double prevVal;
    private double prevTime;

    private boolean continuous = false;
    private double lowerContinuousLimit = -1;
    private double upperContinuousLimit = 1;

    /**
     * Creates a new DynamicSlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *                          second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *                          second. This is expected to be negative.
     * @param initialValue      The initial value of the input.
     */
    public DynamicSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        prevVal = initialValue;
        prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * Creates a new DynamicSlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit and initial value.
     *
     * @param rateLimit    The rate-of-change limit, in units per second.
     * @param initialValue The initial value of the input.
     */
    @Deprecated(since = "2023", forRemoval = true)
    public DynamicSlewRateLimiter(double rateLimit, double initialValue) {
        this(rateLimit, -rateLimit, initialValue);
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public DynamicSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - prevTime;
        prevTime = currentTime;

        double change = MathUtil.clamp(
                input - prevVal,
                negativeRateLimit * elapsedTime,
                positiveRateLimit * elapsedTime);

        if (continuous) {
            change = MathUtil.clamp(
                MathUtil.inputModulus(input - prevVal, lowerContinuousLimit, upperContinuousLimit),
                negativeRateLimit * elapsedTime,
                positiveRateLimit * elapsedTime);

            prevVal += change;

            //Extra check to make sure it is within the limits, probably unnecessary
            prevVal = MathUtil.inputModulus(prevVal, lowerContinuousLimit, upperContinuousLimit);
        } else {
            prevVal += change;
        }

        return prevVal;
    }

    /**
     * Sets a new slewrate and filters the input to limit its slew rate.
     *
     * @param input     The input value whose slew rate is to be limited.
     * @param rateLimit The new rate-of-change limit, in units per second.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input, double rateLimit) {
        setRateLimit(rateLimit);
        return calculate(input);
    }

    /**
     * Sets new slew rates and filters the input to limit its slew rate.
     *
     * @param input             The input value whose slew rate is to be limited.
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *                          second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *                          second. This is expected to be negative.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input, double positiveRateLimit, double negativeRateLimit) {
        setRateLimit(positiveRateLimit, negativeRateLimit);
        return calculate(input);
    }


    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        prevVal = value;
        prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * set positive rate limit
     *
     * @param positiveRateLimit new positive rate limit
     */
    public void setPositiveRateLimit(double positiveRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
    }

    /**
     * set negative rate limit
     *
     * @param negativeRateLimit new negative rate limit
     */
    public void setNegativeRateLimit(double negativeRateLimit) {
        this.negativeRateLimit = negativeRateLimit;
    }

    /**
     * Sets positive and negative rate limits
     *
     * @param rateLimit new rate limits
     */
    public void setRateLimit(double rateLimit) {
        positiveRateLimit = rateLimit;
        negativeRateLimit = -rateLimit;
    }

    /**
     * Sets positive and negative rate limits
     *
     * @param positiveRateLimit new positive rate limit
     * @param negativeRateLimit new negative rate limit
     */
    public void setRateLimit(double positiveRateLimit, double negativeRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
    }

    /**
     * Sets Continuous Limits
     *
     * @param lowerContinuousLimit Lower Continuous Limit
     * @param upperContinuousLimit Upper Continuous Limit
     */
    public void setContinuousLimits(double lowerContinuousLimit, double upperContinuousLimit) {
        this.lowerContinuousLimit = lowerContinuousLimit;
        this.upperContinuousLimit = upperContinuousLimit;
    }

    /**
     * Enables or disables continuous
     * WARNING: Continuous doesn't work properly with non-symmetrical rate limits
     *
     * @param continuous is continuous enabled
     */
    public void enableContinuous(boolean continuous) {
        this.continuous = continuous;
    }
}