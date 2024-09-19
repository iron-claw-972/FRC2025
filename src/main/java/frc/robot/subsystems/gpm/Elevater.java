// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevater extends SubsystemBase {
  /** Creates a new Elevater. */

  TalonFX leftMotor;
  TalonFX rightMotor;
  
  //height off the ground
  int baseheight = 0;


  double currentHeight = 0;
  

  PositionDutyCycle dutyCycle;

  public Elevater() {

    leftMotor = new TalonFX(ElevatorConstants.leftMotorID);
    rightMotor = new TalonFX(ElevatorConstants.rightMotorID);
    leftMotor.setControl(new StrictFollower(ElevatorConstants.rightMotorID));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    double desiredRotations = (ElevatorConstants.gearRatio/ElevatorConstants.spoolcircumfrince) * currentHeight;
    
    leftMotor.setControl(new PositionDutyCycle(desiredRotations));
  }
  
  /**
     * Sets the height of the elevater in meters where the foor is at a height 0 meters.
     * @param height (in meters)
     */
  public void setHeight(int height){
    currentHeight = MathUtil.clamp(height-baseheight, 0, ElevatorConstants.maxHeight);
  }

  public double getCurrentHeight(){return currentHeight;}

  public void motorConfiguration(){

  }
}
