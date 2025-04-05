// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Stores constants for the elevator. */
public class ElevatorConstants {
    public static final double ANGLE = Units.degreesToRadians(3.5); // radians, from vertical
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    public static final int NUMBER_OF_STAGES = 3;
    public static final double GEARING = 8.333/NUMBER_OF_STAGES;
    public static final double MIN_HEIGHT = 0.0; // meters
    public static final double MAX_HEIGHT = Units.inchesToMeters(66);//Units.inchesToMeters(48); // meters
    public static final double START_HEIGHT = MIN_HEIGHT; // meters
    public static final double CARRIAGE_MASS = 3; // kilograms 2.49475803
    public static final double DRUM_RADIUS = Units.inchesToMeters(1.281/2); // meters
    public static final double SPRING_FORCE = 0; //Newtons

    public static final double BOTTOM_LIMIT_SWITCH_HEIGHT = 0;//0.015; // meters
    public static final double TOP_LIMIT_SWITCH_HEIGHT = MAX_HEIGHT; // meters
    public static final double SIM_LIMIT_SWITCH_TRIGGER_DISTANCE = 0.01; // meters

    public static final double STOW_SETPOINT = 0;
    public static final double INTAKE_SETPOINT = 0.036;
    public static final double SAFE_SETPOINT = 0.225;
    public static final double INTAKE_STOW_SETPOINT = 0.58;
    
    //4 inch offset
    public static final double L1_SETPOINT = 0.0;
    public static final double L2_SETPOINT = 0.523;
    public static final double L3_SETPOINT = 0.9192;
    // public static final double L4_SETPOINT = 1.675;
    public static final double L4_SETPOINT = 1.675;

    //touching reef
    public static final double L1_SETPOINT_ALT = 0.27;
    public static final double L2_SETPOINT_ALT = 0.588;
    public static final double L3_SETPOINT_ALT = 0.98-0.0254-0.01;
    public static final double L4_SETPOINT_ALT = 1.675;
    //Dunk L4 = 1.5

    public static final double BOTTOM_ALGAE_SETPOINT = 0.405;
    public static final double TOP_ALGAE_SETPOINT = 0.799;

    public static final double NET_SETPOINT = MAX_HEIGHT;
    public static final double STATION_INTAKE_SETPOINT = 0.233;


    public static final double CENTER_OF_MASS_HEIGHT_STOWED = Units.inchesToMeters(9.44);
    public static final double CENTER_OF_MASS_HEIGHT_EXTENDED = Units.inchesToMeters(10+14.767);


    // The x distance from the center of the robot to the outtake.
    public static final double OUTTAKE_X = Units.inchesToMeters(-7.25);
}
