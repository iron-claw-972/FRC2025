package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
    // Degrees
    public static final double START_ANGLE = -86.5;
    public static final double MIN_ANGLE = -86.5;
    public static final double MAX_ANGLE = 180;
    public static final double OFFSET = 0 - START_ANGLE;

    public static final double GEAR_RATIO = 29.36;
    public static final double ENCODER_GEAR_RATIO = 50.0/24.0;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

    public static final double MASS = 2.46; // kilograms
    public static final double LENGTH = 0.138*2; // meters
    public static final double MOI = 0.0261057394; // kg*m^2
    public static final double CENTER_OF_MASS_LENGTH = 0.138; // meters

    public static final double MAX_VELOCITY = 21; // rad/s
    public static final double MAX_ACCELERATION = 120; // rad/s^2

    public static final double INTAKE_SETPOINT = START_ANGLE;
    public static final double STATION_INTAKE_SETPOINT = 75.5;

    public static final double TOLERANCE = 3.0;

    //Dunk L4 = 6.4
    public static final double L1_SETPOINT = 50;

    //4 in offset
    // public static final double L4_SETPOINT = 11;
    // Original L4: 7.5 degrees
    public static final double L4_SETPOINT_RIGHT = 7.5;
    public static final double L4_SETPOINT_LEFT = 6;
    public static final double L2_L3_SETPOINT = 21.25;

    //touching reef
    public static final double L4_SETPOINT_ALT = 4.5;
    public static final double L2_L3_SETPOINT_ALT = 12.23;

    public static final double ALGAE_SETPOINT = -19.37;
    public static final double ALGAE_NET_SETPOINT_1 = 85.0;
    public static final double ALGAE_NET_SETPOINT_2 = 25;
    public static final double ALGAE_STOW_SETPOINT  = 50;
    public static final double ALGAE_LOLI_SETPOINT  = -19;
    
    public static final double PROCESSOR_SETPOINT = -65.0;

    public static final double STOW_SETPOINT = -14.0;
}
