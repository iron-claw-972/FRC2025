package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
    // TODO: get angles and offset
    // Degrees
    public static final double START_ANGLE = -90;
    public static final double MIN_ANGLE = -90;
    public static final double MAX_ANGLE = 180;
    public static final double OFFSET = 0 - START_ANGLE;

    public static final double GEAR_RATIO = 29.36;
    public static final double ENCODER_GEAR_RATIO = 50.0/24.0;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

    public static final double MASS = 2.46; // kilograms
    public static final double LENGTH = 0.138*2; // meters
    public static final double MOI = 0.0261057394; // kg*m^2
    public static final double CENTER_OF_MASS_LENGTH = 0.138; // meters

    public static final double MAX_VELOCITY = 20; // rad/s
    public static final double MAX_ACCELERATION = 30; // rad/s^2

    //TODO: get setpoint angles
    public static final double INTAKE_SETPOINT = START_ANGLE;
    public static final double STATION_INTAKE_SETPOINT = 30;

    public static final double TOLERANCE = 3.0;

    public static final double L4_SETPOINT = 6.89;
    //Dunk L4 = 6.4
    public static final double L2_L3_SETPOINT = 12.23;
    //TODO: find L1 setpoint angle
    public static final double L1_SETPOINT = 40;

    public static final double ALGAE_SETPOINT = -16.37;
    public static final double ALGAE_NET_SETPOINT_1 = 90.0;
    public static final double ALGAE_NET_SETPOINT_2 = 20;
    public static final double ALGAE_STOW_SETPOINT  = 50;
    public static final double ALGAE_LOLI_SETPOINT  = -19;
    

    // TODO
    public static final double PROCESSOR_SETPOINT = -65.0;

    public static final double STOW_SETPOINT = -14.0;

}
