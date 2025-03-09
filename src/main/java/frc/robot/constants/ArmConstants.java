package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
    // TODO: acutal values
    // Degrees
    public static final double START_ANGLE = -90;
    public static final double MIN_ANGLE = -90;
    public static final double MAX_ANGLE = 90;
    public static final double OFFSET = 0 - START_ANGLE;

    public static final int GEAR_RATIO = 1;
    //public static final double GEAR_RATIO = 50/24;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

    public static final double MASS = 5; // kilograms
    public static final double LENGTH = 0.127; // meters
    public static final double MOI = MASS*LENGTH*LENGTH/3; // kg*m^2
    public static final double CENTER_OF_MASS_LENGTH = LENGTH/2; // meters

    public static final double MAX_VELOCITY = 5; // rad/s
    public static final double MAX_ACCELERATION = 8; // rad/s^2
}
