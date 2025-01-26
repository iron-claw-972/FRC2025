package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

/**
 * Utility class for useful functions.
 */
public class MathUtils {

    /**
     * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be linear from
     * (deadband, 0) to (1,1)
     *
     * @param input    The input value to rescale
     * @param deadband The deadband
     * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
     */
    public static double deadband(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

    /**
     * Deadbands an input to [-1, -OIConstants.DEADBAND], [OIConstants.DEADBAND, 1], rescaling inputs to be linear from
     * (OIConstants.DEADBAND, 0) to (1,1)
     *
     * @param input The input value to rescale
     * @return the input rescaled and to fit [-1, -DEADBAND], [DEADBAND, 1]
     */
    public static double deadband(double input) {
        return deadband(input, Constants.DEFAULT_DEADBAND);
    }

    /**
     * An exponential function that maintains positive or negative sign.
     *
     * @param exponent the power to raise the base to
     * @param base     the base which will be raised to the power
     * @return base to the power of exponent, maintaining sign of base
     */
    public static double expoMS(double base, double exponent) {
        // weird stuff will happen if you don't put a number > 0 for controller inputs
        double finVal = Math.pow(Math.abs(base), exponent);
        if (base < 0) {
            finVal *= -1;
        }
        return finVal;
    }

    /**
     * Calculates Midpoint of two numbers on modulus number line
     *
     * @param num1       first number
     * @param num2       second number
     * @param lowerBound lower bound of modulus number line
     * @param upperBound upper bound of modulus number line
     * @return midpoint of 2 numbers on modulus number line
     */
    public static double modulusMidpoint(double num1, double num2, double lowerBound, double upperBound) {
        num1 = MathUtil.inputModulus(num1, lowerBound, upperBound);
        num2 = MathUtil.inputModulus(num2, lowerBound, upperBound);
        if (Math.abs(num1 - num2) > (upperBound - lowerBound) / 2) {
            return MathUtil.inputModulus((num1 + num2) / 2 + (upperBound - lowerBound) / 2, lowerBound, upperBound);
        }
        return (num1 + num2) / 2;
    }

    /**
     * Interpolates between two numbers on modulus number line
     *
     * @param num1       first number
     * @param num2       second number
     * @param amount     the amount to interpolate, 0 = first number, 1 = second number
     * @param lowerBound lower bound of modulus number line
     * @param upperBound upper bound of modulus number line
     * @return interpolated value between 2 numbers on modulus number line
     */
    public static double modulusInterpolate(double num1, double num2, double amount, double lowerBound, double upperBound) {
        num1 = MathUtil.inputModulus(num1, lowerBound, upperBound);
        num2 = MathUtil.inputModulus(num2, lowerBound, upperBound);
        if (Math.abs(num1 - num2) > (upperBound - lowerBound) / 2) {
            if(num1 < num2){
                num1 += upperBound-lowerBound;
            }else{
                num2 += upperBound-lowerBound;
            }
        }
        return MathUtil.inputModulus((1-amount)*num1 + amount*num2, lowerBound, upperBound);
    }

    /**
     * Calls {@link #mean(double...)}.
     *
     * @param data the list of data to find the mean of
     * @return the mean of the data
     */
    public static double mean(List<Double> data) {
        return mean(doubleListToArray(data));
    }

    /**
     * Finds the mean of the provided array of doubles
     *
     * @param data an array of doubles
     * @return the mean of the data
     */
    public static double mean(double... data) {
        double mean = 0;
        for (double datum : data) {
            mean += datum;
        }
        mean /= data.length;
        return mean;
    }

    /**
     * Calls {@link #stdDev(double...)}.
     *
     * @param data the list of data to find the standard deviation of
     * @return the standard deviation of the data
     */
    public static double stdDev(List<Double> data) {
        return stdDev(doubleListToArray(data));
    }

    /**
     * Finds the standard deviation of the provided array of doubles
     *
     * @param data an array of doubles
     * @return the standard deviation of the data
     */
    public static double stdDev(double... data) {
        if (data.length == 0 || data.length == 1) return 0;

        double mean = mean(data);

        double total = 0;
        for (double datum : data) {
            total += Math.pow(datum - mean, 2);
        }
        return Math.sqrt(total / (data.length - 1));
    }

    private static double[] doubleListToArray(List<Double> arrayList) {
        return arrayList.stream().mapToDouble(Double::doubleValue).toArray();
    }

}
