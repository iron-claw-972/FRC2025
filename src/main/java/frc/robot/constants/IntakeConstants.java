// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Constants for the intake
*/
public class IntakeConstants {

    public static final DCMotor MOTOR = DCMotor.getNEO(1);
    public static final double GEARING = 20;
    public static final double MOI = 0.3;
    public static final double ARM_LENGTH = 0.3;
    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = Math.PI/2;
    public static final double STARTING_ANGLE = Math.PI/2;
    
}
