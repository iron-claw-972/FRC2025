package frc.robot.util;

import lib.PolynomialRegression;

import java.util.LinkedList;
import java.util.List;

/**
 * A class for storing and processing feedforward characterization data. Used in automatic feedforward characterization.
 */
public class FeedForwardCharacterizationData {
    private PolynomialRegression regression;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    /**
     * Adds a data point to the data set.
     *
     * @param velocity the velocity of the motor
     * @param voltage  the voltage applied to the motor
     */
    public void add(double velocity, double voltage) {
        if (Math.abs(velocity) > 1E-4) {
            velocityData.add(Math.abs(velocity));
            voltageData.add(Math.abs(voltage));
        }
    }

    /**
     * Processes the data set using {@link PolynomialRegression}
     *
     * @see PolynomialRegression
     */
    public void process() {
        // creates a new process polynomial regression to get calculated values
        regression = new PolynomialRegression(
                velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
                voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
                1
        );
    }

    /**
     * Gets the static voltage of the motor.
     *
     * @return the static voltage of the motor
     */
    public double getStatic() {
        // gets y-intercept
        return regression.beta(0);
    }

    /**
     * Gets the velocity of the motor.
     *
     * @return the velocity of the motor
     */
    public double getVelocity() {
        // gets a slope of regression line
        return regression.beta(1);
    }

    /**
     * Gets the variance of the data set.
     *
     * @return the variance of the data set
     */
    public double getVariance() {
        // gets variance of data set
        return regression.R2();
    }
}

