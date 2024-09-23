// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private double setpoint = ElevatorConstants.START_HEIGHT;
  private Mechanism2d mechanism;
  private MechanismLigament2d ligament;

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Units.feetToMeters(3.0),
              Units.feetToMeters(6.0))); // Max elevator speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  private final LinearSystem<N2, N1, N1> m_elevatorPlant =
      LinearSystemId.createElevatorSystem(
          ElevatorConstants.MOTOR, ElevatorConstants.CARRIAGE_MASS, ElevatorConstants.DRUM_RADIUS, ElevatorConstants.GEARING);

  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          (LinearSystem<N2, N1, N1>) m_elevatorPlant,
          VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(40)), // How accurate we
          // think our model is, in meters and meters/second.
          VecBuilder.fill(0.001), // How accurate we think our encoder position
          // data is. In this case we very highly trust our encoder position reading.
          Constants.LOOP_TIME);

  private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          (LinearSystem<N2, N1, N1>) m_elevatorPlant,
          VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)), // qelms. Position
          // and velocity error tolerances, in meters and meters per second. Decrease this to more
          // heavily penalize state excursion, or make the controller behave more aggressively. In
          // this example we weight position much more highly than velocity, but this can be
          // tuned to balance the two.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          Constants.LOOP_TIME); // Nominal time between loops. 0.020 for TimedRobot, but can be
  
  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop =
      new LinearSystemLoop<>(
          (LinearSystem<N2, N1, N1>) m_elevatorPlant,
          m_controller,
          m_observer,
          12.0,
          Constants.LOOP_TIME);

          

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
    m_loop.reset(VecBuilder.fill(getPosition(), 0));
    m_lastProfiledReference =
        new TrapezoidProfile.State(getPosition(), 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    TrapezoidProfile.State goal;

    goal = new TrapezoidProfile.State(setpoint, 0.0);

    m_lastProfiledReference = m_profile.calculate(Constants.LOOP_TIME, m_lastProfiledReference, goal);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(getPosition()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(Constants.LOOP_TIME);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    
    motor.setVoltage(nextVoltage);

    if(RobotBase.isSimulation()){
      sim.setInputVoltage(nextVoltage);
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
    this.setpoint = MathUtil.clamp(setpoint, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
  }

  public Mechanism2d getMechanism2d(){
    return mechanism;
  }
}
