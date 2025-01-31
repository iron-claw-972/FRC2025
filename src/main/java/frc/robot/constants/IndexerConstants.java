package frc.robot.constants;

public class IndexerConstants {
	public static final double speed = 0.5;

	public static final double runForExtra = 1.0; // time to run after sensor finishes

	public static final double momentOfInertia = 1.0; // TODO: actual value
	public static final double gearRatio = 1.0; // TODO: actual value

	// this stuff is for sim, all in meters
	public static final double wheelCircumference = Math.PI * 0.1;
	public static final double startSimPosAt = -1.0; // start coral here
	public static final double endSimPosAt = 1.0; // wrap around here
	public static final double startSimSensorPosAt = -0.5; // activate the sensor here
	public static final double endSimSensorPosAt = 0.5; // deactivte the sensor here
}

