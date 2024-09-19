// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class ElevatorConstants {
    public static final int MOTOR_ID = -1;

    public static final double ANGLE = 0; // radians, from vertical
    public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
    public static final double GEARING = 18 * NUMBER_OF_STAGES;
    public static final double MIN_HEIGHT = 0; // meters
    public static final double MAX_HEIGHT = 4; // meters
    public static final double START_HEIGHT = MIN_HEIGHT; // meters
    public static final double CARRIAGE_MASS = 10; // kilograms
    public static final double DRUM_RADIUS = 0.0325374/2; // meters
    
    public static final int NUMBER_OF_STAGES = 3
}
