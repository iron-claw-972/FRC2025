// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
    public static final int MOTOR_ID = 0;

    public static final double ANGLE = 0; // radians, from vertical
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(2);
    public static final int NUMBER_OF_STAGES = 2;
    public static final double GEARING = 18/NUMBER_OF_STAGES;
    public static final double MIN_HEIGHT = 0; // meters
    public static final double MAX_HEIGHT = Units.inchesToMeters(49); // meters
    public static final double START_HEIGHT = MIN_HEIGHT; // meters
    public static final double CARRIAGE_MASS = 3.0268219; // kilograms
    public static final double DRUM_RADIUS = 0.0162687; // meters
    public static final double SPRING_FORCE = 24.910067711605; //Newtons

    
}
