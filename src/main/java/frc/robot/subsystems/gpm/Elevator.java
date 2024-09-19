// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private TalonFX motor = new TalonFX(ElevatorConstants.MOTOR_ID);
  private AngledElevatorSim sim;
  private double setpoint = -1;
  private Mechanism2d mechanism;
  private MechanismLigament2d ligament;

  /** Creates a new Elevator. */
  public Elevator() {
    // TODO: This assumes elevator always starts at starting position, use different sensor instead
    resetEncoder(ElevatorConstants.START_HEIGHT);
    if(RobotBase.isSimulation()){
      sim = new AngledElevatorSim(ElevatorConstants.MOTOR, ElevatorConstants.GEARING, ElevatorConstants.CARRIAGE_MASS, ElevatorConstants.DRUM_RADIUS, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT, true, ElevatorConstants.START_HEIGHT, ElevatorConstants.ANGLE);
      double width = ElevatorConstants.MAX_HEIGHT * Math.sin(ElevatorConstants.ANGLE);
      double height = ElevatorConstants.MAX_HEIGHT * Math.cos(ElevatorConstants.ANGLE);
      double size = Math.max(width, height);
      mechanism = new Mechanism2d(size, size);
      ligament = mechanism.getRoot("base", size/2-width/2, size/2-height/2).append(new MechanismLigament2d("elevator", ElevatorConstants.START_HEIGHT, 90-Units.radiansToDegrees(Math.abs(ElevatorConstants.ANGLE))));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Temporary bang bang, TODO: @chem7397, replace with your algorithm
    double speed = 0.5;
    if(getPosition() > setpoint){
      setpoint = ElevatorConstants.MIN_HEIGHT+0.01;
      speed *= -1;
    }else{
      setpoint = ElevatorConstants.MAX_HEIGHT-0.01;
    }
    if(RobotBase.isReal()){
      motor.set(speed);
    }else{
      sim.setInputVoltage(speed*Constants.ROBOT_VOLTAGE);
    }
  }

  @Override
  public void simulationPeriodic(){
    sim.update(Constants.LOOP_TIME);
    ligament.setLength(getPosition());
  }

  public void resetEncoder(double height){
    motor.setPosition(height/(2*Math.PI*ElevatorConstants.DRUM_RADIUS)*ElevatorConstants.GEARING);
  }

  public double getPosition(){
    if(RobotBase.isReal()){
      return motor.getPosition().getValueAsDouble()/ElevatorConstants.GEARING*(2*Math.PI*ElevatorConstants.DRUM_RADIUS);
    }else{
      return sim.getPositionMeters();
    }
  }

  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
  }

  public Mechanism2d getMechanism2d(){
    return mechanism;
  }
}
