// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotId;

/** Stores constants for the elevator. */
public class ElevatorConstants {
    public static final double ANGLE = 0; // radians, from vertical
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(2);
    public static final int NUMBER_OF_STAGES = 2;
    public static final double GEARING = 12.5/NUMBER_OF_STAGES;
    public static final double MIN_HEIGHT = 0.0; // meters
    public static double MAX_HEIGHT = 1.257; // meters
    public static final double START_HEIGHT = MIN_HEIGHT; // meters
    public static double CARRIAGE_MASS = 6.85105916; // kilograms
    public static double DRUM_RADIUS = 0.0133; // meters
    public static final double SPRING_FORCE = 0; //Newtons

    public static double BOTTOM_LIMIT_SWITCH_HEIGHT = 0; // meters
    public static final double SIM_LIMIT_SWITCH_TRIGGER_DISTANCE = 0.01; // meters

    public static final double STOW_SETPOINT = 0;
    public static final double INTAKE_SETPOINT = 0;
    public static double L1_SETPOINT = 0;
    public static double L2_SETPOINT = 0.08;
    public static double L3_SETPOINT = 0.5;
    public static double L4_SETPOINT = 1.18;

    public static double CENTER_OF_MASS_HEIGHT_STOWED = Units.inchesToMeters(8.933);
    public static double CENTER_OF_MASS_HEIGHT_EXTENDED = Units.inchesToMeters(16.931);

    // This is not the best way to do this, but it is the easiest for the Alpha Bot
    public static double CENTER_OF_MASS_HEIGHT = CENTER_OF_MASS_HEIGHT_STOWED;

    // The x distance from the center of the robot to the outtake.
    public static double OUTTAKE_X = Units.inchesToMeters(-7.25);

    // Updates elevator constants for the alpha bot
    public static void update(RobotId robotId){
        if(robotId == RobotId.Phil){
            NUMBER_OF_STAGES = 2;
            GEARING = 12.5/NUMBER_OF_STAGES;
            MAX_HEIGHT = 1.257;
            CARRIAGE_MASS = 6.85105916; // kilograms
            DRUM_RADIUS = 0.0133; // meters
            BOTTOM_LIMIT_SWITCH_HEIGHT = 0; // meters
            L1_SETPOINT = 0;
            L2_SETPOINT = 0.08;
            L3_SETPOINT = 0.5;
            L4_SETPOINT = 1.18;
            CENTER_OF_MASS_HEIGHT_STOWED = Units.inchesToMeters(8.933);
            CENTER_OF_MASS_HEIGHT_EXTENDED = Units.inchesToMeters(16.931);
            OUTTAKE_X = Units.inchesToMeters(-7.25);
        }
    }
}
