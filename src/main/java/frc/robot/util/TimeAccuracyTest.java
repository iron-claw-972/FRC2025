package frc.robot.util;

import edu.wpi.first.util.WPIUtilJNI;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Class for testing if a value is within a certain margin of error for a certain amount of time.
 */
public class TimeAccuracyTest {

    private final BooleanSupplier accuracyTest;
    private final double setpointUpdateTime;
    private final double errorMargin;
    private final double timeMargin;
    private boolean lastUseableResult = false;

    /**
     * @param actual      DoubleSupplier that returns the actual value
     * @param setpoint    DoubleSupplier that returns the setpoint
     * @param errorMargin margin of error for the test to be accurate
     * @param timeMargin  time in seconds that the setpoint must be held for the test to be accurate
     */
    public TimeAccuracyTest(DoubleSupplier actual, DoubleSupplier setpoint, double errorMargin, double timeMargin) {
        this.errorMargin = errorMargin;
        this.timeMargin = timeMargin;
        setpointUpdateTime = WPIUtilJNI.now() * 1e-6;
        accuracyTest = () -> getDoubleAccuracyTest(actual, setpoint);
    }

    /**
     * Determines if the test is successful.
     *
     * @return true if the test is successful, false if not
     */
    public boolean calculate() {
        if (setpointUpdateTime + timeMargin <= WPIUtilJNI.now() * 1e-6)
            lastUseableResult = accuracyTest.getAsBoolean();
        return lastUseableResult;
    }

    /**
     * Determines if the actual value is within the error margin of the setpoint.
     *
     * @return true if the actual value is within the error margin of the setpoint, false if not
     */
    private boolean getDoubleAccuracyTest(DoubleSupplier actual, DoubleSupplier setpoint) {
        return Math.abs(actual.getAsDouble() - setpoint.getAsDouble()) <= errorMargin;
    }
}
