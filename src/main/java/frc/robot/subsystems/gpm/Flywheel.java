// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gpm;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;

/**
 * This is a sample program to demonstrate how to use a state-space controller to control a
 * flywheel.
 * <p> It extends Shooter to work with 2024's code
 */
public class Flywheel extends Shooter {
  private double leftSetpoint;
  private double rightSetpoint;

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  // TODO: Can multiple controllers use teh same plant?
  private final LinearSystem<N1, N1, N1> leftFlywheelPlant =
    LinearSystemId.createFlywheelSystem(
      gearbox, MOI_SHAFT, gearRatio);
  private final LinearSystem<N1, N1, N1> rightFlywheelPlant =
    LinearSystemId.createFlywheelSystem(
      gearbox, MOI_SHAFT, gearRatio);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  // TODO: Can both sides us the same filter?
  private final KalmanFilter<N1, N1, N1> leftObserver =
    new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      leftFlywheelPlant,
      VecBuilder.fill(3.0), // How accurate we think our model is
      VecBuilder.fill(0.01), // How accurate we think our encoder
      // data is
      Constants.LOOP_TIME);
  private final KalmanFilter<N1, N1, N1> rightObserver =
    new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      rightFlywheelPlant,
      VecBuilder.fill(3.0), // How accurate we think our model is
      VecBuilder.fill(0.01), // How accurate we think our encoder
      // data is
      Constants.LOOP_TIME);
    
  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> leftController =
    new LinearQuadraticRegulator<>(
      leftFlywheelPlant,
      VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
      // this to more heavily penalize state excursion, or make the controller behave more
      // aggressively.
      VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12 is a good
      // starting point because that is the (approximate) maximum voltage of a battery.
      Constants.LOOP_TIME); // Nominal time between loops. Constants.LOOP_TIME for TimedRobot, but can be
  // lower if using notifiers.
  private final LinearQuadraticRegulator<N1, N1, N1> rightController =
    new LinearQuadraticRegulator<>(
      rightFlywheelPlant,
      VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
      // this to more heavily penalize state excursion, or make the controller behave more
      // aggressively.
      VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12 is a good
      // starting point because that is the (approximate) maximum voltage of a battery.
      Constants.LOOP_TIME); // Nominal time between loops. Constants.LOOP_TIME for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> leftLoop =
    new LinearSystemLoop<>(leftFlywheelPlant, leftController, leftObserver, 12.0, Constants.LOOP_TIME);
  private final LinearSystemLoop<N1, N1, N1> rightLoop =
    new LinearSystemLoop<>(rightFlywheelPlant, rightController, rightObserver, 12.0, Constants.LOOP_TIME);
    
  private double leftVoltage;
  private double rightVoltage;

  private FlywheelSim leftSim;
  private FlywheelSim rightSim;

  private ShuffleboardTab tab = Shuffleboard.getTab("Flywheel");
  private GenericEntry setpointEntry = tab.add("Setpoint", 0).getEntry();
  private double prevSetpointEntry = 0;

  private static final double tolerance = 20; // RPM

  public Flywheel() {
    leftLoop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(leftMotorEncoder.getVelocity())));
    rightLoop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(leftMotorEncoder.getVelocity())));
    leftSetpoint = 0;
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(false);
	  leftMotor.setInverted(true);

    if(RobotBase.isSimulation()){
      leftSim = new FlywheelSim(leftFlywheelPlant, gearbox, gearRatio);
      rightSim = new FlywheelSim(rightFlywheelPlant, gearbox, gearRatio);
    }

    tab.addDouble("Left speed", ()->getLeftSpeed());
    tab.addDouble("Right speed", ()->getRightSpeed());
  }


  @Override
  public void periodic() {
    double currentSetpointEntry = setpointEntry.getDouble(0);
    if(currentSetpointEntry != prevSetpointEntry){
      // If the value on Shuffleboard changed, update the setpoint
      setSpeed(currentSetpointEntry);
      prevSetpointEntry = currentSetpointEntry;
    }else{
      // If it didn't change, update it to the current setpoint. This allows commands to change the setpoint
      prevSetpointEntry = leftSetpoint/2 + rightSetpoint/2;
      setpointEntry.setDouble(prevSetpointEntry);
    }

    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
    // PID controller.
    leftLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(leftSetpoint)));
    rightLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(rightSetpoint)));
    
    // Correct our Kalman filter's state vector estimate with encoder data.
    leftLoop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(getLeftSpeed())));
    rightLoop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(getRightSpeed())));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    leftLoop.predict(Constants.LOOP_TIME);
    rightLoop.predict(Constants.LOOP_TIME);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    leftVoltage = leftLoop.getU(0);
    leftMotor.setVoltage(leftVoltage);
    rightVoltage = rightLoop.getU(0);
    rightMotor.setVoltage(rightVoltage);
  }

  @Override
  public void simulationPeriodic(){
    leftSim.setInputVoltage(leftVoltage);
    rightSim.setInputVoltage(rightVoltage);
    leftSim.update(Constants.LOOP_TIME);
    rightSim.update(Constants.LOOP_TIME);
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    leftSetpoint = leftSpeed;
    rightSetpoint = rightSpeed;
  }
  public void setSpeed(double speed){
    // Use the "slip" code in Shooter
    // This is actually more likely caused by conservation of momentum in the collision between the notes and shooter than slip, but the coefficient still works
    setTargetRPM(speed);
  }
  public double getLeftSpeed(){
    // I couldn't find a class that supports simulated encoder velocities for a RelativeEncoder. TODO: Find one if it exists
    if(RobotBase.isSimulation()){
      return leftSim.getAngularVelocityRPM();
    }
    return leftMotorEncoder.getVelocity() / gearRatio;
  }
  public double getRightSpeed(){
    // I couldn't find a class that supports simulated encoder velocities for a RelativeEncoder. TODO: Find one if it exists
    if(RobotBase.isSimulation()){
      return rightSim.getAngularVelocityRPM();
    }
    return rightMotorEncoder.getVelocity() / gearRatio;
  }

  // Methods to make previously existing commands work

  @Override
  public void setTargetRPM(double left, double right){
    setSpeed(left, right);
  }
  @Override
  public boolean atSetpoint(){
    return Math.abs(leftSetpoint - getLeftSpeed()) < tolerance
      && Math.abs(rightSetpoint - getRightSpeed()) < tolerance;
  }
  @Override
  public double getLeftMotorRPM(){
    return getLeftSpeed();
  }
  @Override
  public double getRightMotorRPM(){
    return getRightSpeed();
  }

  // All other methods in Shooter call these
}
