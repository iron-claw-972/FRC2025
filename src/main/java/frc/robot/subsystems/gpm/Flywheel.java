// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gpm;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;

/**
 * This is a sample program to demonstrate how to use a state-space controller to control a
 * flywheel.
 */
public class Flywheel extends SubsystemBase {

  private double leftSetpoint;
  private double rightSetpoint;

  // 1 motor per side
  private static final DCMotor gearbox = DCMotor.getNeoVortex(1);

  // TODO: Check if this is for 1 side or both sides
  private static final double momentOfInertia = 0.00032; // kg * m^2

  // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
  // the motors, this number should be greater than one.
  private static final double gearing = 1.0;

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in rotations per minute.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in rotations per minute.
  // TODO: Can multiple controllers use teh same plant?
  private final LinearSystem<N1, N1, N1> leftFlywheelPlant =
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1), momentOfInertia, gearing);
  private final LinearSystem<N1, N1, N1> rightFlywheelPlant =
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1), momentOfInertia, gearing);

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
  
  private final CANSparkFlex leftMotor = new CANSparkFlex(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkFlex rightMotor = new CANSparkFlex(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  
  private final RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  private FlywheelSim leftSim;
  private FlywheelSim rightSim;

  public Flywheel() {
    leftLoop.reset(VecBuilder.fill(leftMotorEncoder.getVelocity()));
    rightLoop.reset(VecBuilder.fill(leftMotorEncoder.getVelocity()));
    leftSetpoint = 0;
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(false);
	  leftMotor.setInverted(true);

    if(RobotBase.isSimulation()){
      leftSim = new FlywheelSim(leftFlywheelPlant, gearbox, gearing);
      rightSim = new FlywheelSim(rightFlywheelPlant, gearbox, gearing);
    }
  }


  @Override
  public void periodic() {
    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
    // PID controller.
    leftLoop.setNextR(VecBuilder.fill(leftSetpoint));
    rightLoop.setNextR(VecBuilder.fill(rightSetpoint));
    
    // Correct our Kalman filter's state vector estimate with encoder data.
    leftLoop.correct(VecBuilder.fill(leftMotorEncoder.getVelocity()));
    rightLoop.correct(VecBuilder.fill(rightMotorEncoder.getVelocity()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    leftLoop.predict(Constants.LOOP_TIME);
    rightLoop.predict(Constants.LOOP_TIME);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = leftLoop.getU(0);
    leftMotor.setVoltage(nextVoltage);
    nextVoltage = rightLoop.getU(0);
    rightMotor.setVoltage(nextVoltage);
  }

  @Override
  public void simulationPeriodic(){
    leftSim.setInputVoltage(leftMotor.getBusVoltage());
    rightSim.setInputVoltage(leftMotor.getBusVoltage());
    leftSim.update(Constants.LOOP_TIME);
    rightSim.update(Constants.LOOP_TIME);
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    leftSetpoint = leftSpeed;
    rightSetpoint = rightSpeed;
  }
  public void setSpeed(double speed){
    setSpeed(speed, speed);
  }
  public double getLeftSpeed(){
    // I couldn't find a class that supports simulated encoder velocities for a RelativeEncoder. TODO: Find one if it exists
    if(RobotBase.isSimulation()){
      return leftSim.getAngularVelocityRPM();
    }
    return leftMotorEncoder.getVelocity();
  }
  public double getRightSpeed(){
    // I couldn't find a class that supports simulated encoder velocities for a RelativeEncoder. TODO: Find one if it exists
    if(RobotBase.isSimulation()){
      return rightSim.getAngularVelocityRPM();
    }
    return rightMotorEncoder.getVelocity();
  }
}
