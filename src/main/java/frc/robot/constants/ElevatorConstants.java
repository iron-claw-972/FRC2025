// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Stores constants for the elevator. */
public class ElevatorConstants {
    // TODO: update for comp robot
    public static final double ANGLE = 0; // radians, from vertical
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(2);
    public static final int NUMBER_OF_STAGES = 3;
    public static final double GEARING = 6/NUMBER_OF_STAGES;
    public static final double MIN_HEIGHT = 0.0; // meters
    public static final double MAX_HEIGHT = 1.257;//Units.inchesToMeters(48); // meters
    public static final double START_HEIGHT = MIN_HEIGHT; // meters
    public static final double CARRIAGE_MASS = 6.85105916; // kilograms
    public static final double DRUM_RADIUS = 0.0133; // meters
    public static final double SPRING_FORCE = 24.910067711605; //Newtons

    public static final double BOTTOM_LIMIT_SWITCH_HEIGHT = 0;//0.015; // meters
    public static final double TOP_LIMIT_SWITCH_HEIGHT = MAX_HEIGHT; // meters
    public static final double SIM_LIMIT_SWITCH_TRIGGER_DISTANCE = 0.01; // meters

    public static final double STOW_SETPOINT = 0;
    public static final double INTAKE_SETPOINT = 0;
    public static final double L1_SETPOINT = 0;
    public static final double L2_SETPOINT = 0.08;
    public static final double L3_SETPOINT = 0.5;
    public static final double L4_SETPOINT = 1.18;

    public static final double CENTER_OF_MASS_HEIGHT_STOWED = Units.inchesToMeters(8.933);
    public static final double CENTER_OF_MASS_HEIGHT_EXTENDED = Units.inchesToMeters(16.931);

    // The x distance from the center of the robot to the outtake.
    public static final double OUTTAKE_X = Units.inchesToMeters(-7.25);
}
