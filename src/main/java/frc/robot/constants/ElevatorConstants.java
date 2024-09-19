// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ElevatorConstants {
    
    public static final int leftMotorID = 0;
    public static final int rightMotorID = 0;

    public static final int numberOfStages = 3;


    public static final int maxHeight = 4;
    
    //gear ratio for state 3
    public static final int gearRatio = 18 * numberOfStages;

    public static final double spoolDiameter = 0.0325374;

    public static final double spoolcircumfrince = spoolDiameter * Math.PI;


}
