package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Container class for test constants.
 */
public class TestConstants {

    public static final double TRANSLATION_ERROR = 0.6;

    //time error for odometry is dependent on test
    public static final double HEADING_ERROR = Units.degreesToRadians(1);
    public static final double HEADING_TIME_ERROR = 0.1;
    public static final double STEER_ANGLE_ERROR = Units.degreesToRadians(1);

    public static final double STEER_ANGLE_TIME_ERROR = 0.1;
    public static final double DRIVE_VELOCITY_ERROR = 0.1;
    public static final double DRIVE_VELOCITY_TIME_ERROR = 0.1;
    public static final double DRIVE_FEED_FORWARD_VOLTAGE_STEP = 0.2;

    public static final double DRIVE_FEED_FORWARD_MAX_VOLTAGE = 11;
    public static final double DRIVE_FEED_FORWARD_ACCELERATION_TIME_BUFFER = 0.5;
    public static final double DRIVE_FEED_FORWARD_RECORDING_TIME = 1.5;
    public static final double STEER_FEED_FORWARD_VOLTAGE_STEP = 0.2;

    public static final double STEER_FEED_FORWARD_MAX_VOLTAGE = 6;
    public static final double STEER_FEED_FORWARD_ACCELERATION_TIME_BUFFER = 0.5;
    public static final double STEER_FEED_FORWARD_RECORDING_TIME = 2;

    private TestConstants() {
    }

}