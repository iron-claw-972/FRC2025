package frc.robot.constants;

public class IndexerConstants {
	public static final double SPEED = 0.5;

	public static final double MOMENT_OF_INERTIA = 0.000326;
	public static final double GEAR_RATIO = 1.0; // TODO: actual value

	// this stuff is for sim, all in meters
	public static final double WHEEL_CIRCUMFERENCE = Math.PI * 0.1;
	public static final double START_SIM_POS_AT = -1.0; // start coral here
	public static final double END_SIM_POS_AT = 1.0; // wrap around here
	public static final double START_SIM_SENSOR_POS_AT = -0.5; // activate the sensor here
	public static final double END_SIM_SENSOR_POS_AT = 0.5; // deactivte the sensor here
}

