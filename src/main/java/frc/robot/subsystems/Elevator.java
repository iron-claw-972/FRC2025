// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MatBuilder;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IdConstants;
import frc.robot.util.AngledElevatorSim;
import frc.robot.util.LogManager;
import frc.robot.util.LogManager.LogLevel;

public class Elevator extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(IdConstants.ELEVATOR_RIGHT_MOTOR, Constants.CANIVORE_CAN);

  private double setpoint = ElevatorConstants.START_HEIGHT;
  private double maxVoltage = 10

  ;
  // Sim variables
  private AngledElevatorSim sim;
  private Mechanism2d mechanism;
  private MechanismLigament2d ligament;
  private double voltage;

  
  private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          3.0,
          Units.feetToMeters(9.0))); // Max elevator speed and acceleration.
 
  private State m_lastProfiledReference = new State();

  private final LinearSystem<N2, N1, N2> m_elevatorPlant = LinearSystemId.createElevatorSystem(
      ElevatorConstants.MOTOR, ElevatorConstants.CARRIAGE_MASS, ElevatorConstants.DRUM_RADIUS,
      ElevatorConstants.GEARING);

  private final KalmanFilter<N2, N1, N2> m_observer = new KalmanFilter<>(
      Nat.N2(),
      Nat.N2(),
      (LinearSystem<N2, N1, N2>) m_elevatorPlant,
      VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(20)), // How accurate we
      // think our model is, in meters and meters/second.
      VecBuilder.fill(0.001,0.001), // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position reading.
      Constants.LOOP_TIME);
  
  private final LinearQuadraticRegulator<N2, N1, N2> m_controller = new LinearQuadraticRegulator<>(
      (LinearSystem<N2, N1, N2>) m_elevatorPlant,
      VecBuilder.fill(Units.inchesToMeters(1), Units.inchesToMeters(20.0)), // qelms. Position
      // and velocity error tolerances, in meters and meters per second. Decrease this
      // to more
      // heavily penalize state excursion, or make the controller behave more
      // aggressively. In
      // this example we weight position much more highly than velocity, but this can
      // be
      // tuned to balance the two.
      VecBuilder.fill(maxVoltage), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12
      // is a good
      // starting point because that is the (approximate) maximum voltage of a
      // battery.
      Constants.LOOP_TIME); // Nominal time between loops. 0.020 for TimedRobot, but can be changed

  // The state-space loop combines a controller, observer, feedforward and plant
  // for easy control.
  private final LinearSystemLoop<N2, N1, N2> m_loop = new LinearSystemLoop<>(
      (LinearSystem<N2, N1, N2>) m_elevatorPlant,
      m_controller,
      m_observer,
      maxVoltage,
      Constants.LOOP_TIME);

 // ExponentialProfile profile = new ExponentialProfile(Constraints.fromStateSpace(maxVoltage, m_elevatorPlant.getA(1, 1), m_elevatorPlant.getB().get(1,0)));
  //ExponentialProfile.State m_lastProfiledReference;
  /** Creates a new Elevator. */
  public Elevator() {

    // This increases both the time and memory efficiency of the code when running
    // on a real robot; do not remove this if statement
    if (RobotBase.isSimulation()) {
      sim = new AngledElevatorSim(ElevatorConstants.MOTOR, ElevatorConstants.GEARING, ElevatorConstants.CARRIAGE_MASS,
        ElevatorConstants.DRUM_RADIUS, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT, true,
        ElevatorConstants.START_HEIGHT, ElevatorConstants.ANGLE, ElevatorConstants.SPRING_FORCE);
      double width = ElevatorConstants.MAX_HEIGHT * Math.sin(ElevatorConstants.ANGLE);
      double height = ElevatorConstants.MAX_HEIGHT * Math.cos(ElevatorConstants.ANGLE);
      double size = Math.max(width, height);
      mechanism = new Mechanism2d(size, size);
      ligament = mechanism.getRoot("base", size / 2 - width / 2, size / 2 - height / 2).append(new MechanismLigament2d(
        "elevator", ElevatorConstants.START_HEIGHT, 90 - Units.radiansToDegrees(Math.abs(ElevatorConstants.ANGLE))));
      SmartDashboard.putData("elevator", mechanism);
    }
    Timer.delay(1.0);
    m_loop.reset(VecBuilder.fill(getPosition(), 0));
    m_lastProfiledReference = new State(getPosition(), 0);
    //m_lastProfiledReference = new ExponentialProfile.State(getPosition(),0);
    resetEncoder(ElevatorConstants.START_HEIGHT);

    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    //Logging
    LogManager.logSupplier("Elevator/Voltage", () -> getVoltage(), 100, LogLevel.INFO);
    LogManager.logSupplier("Elevator/Velocity", () -> getVelocity(), 100, LogLevel.INFO);
    LogManager.logSupplier("Elevator/position", () -> getPosition(), 100, LogLevel.INFO);
    SmartDashboard.putNumber("setpoint elevator", 0);

  }

  @Override
  public void periodic() {
    setSetpoint(SmartDashboard.getNumber("setpoint elevator", 0));


    // The final state that the elevator is trying to get to
    State goal = new State(setpoint, 0.0);

    double currentPosition = getPosition();
    
    m_lastProfiledReference = m_profile.calculate(Constants.LOOP_TIME, m_lastProfiledReference, goal);
    //m_lastProfiledReference = m_profile.calculate(Constants.LOOP_TIME, m_lastProfiledReference, goal);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(MatBuilder.fill(Nat.N2(), Nat.N1(), currentPosition, getVelocity()));

    // Update our LQR to generate new voltage commands and use the voltages to
    // predict the next
    // state with out Kalman filter.
    m_loop.predict(Constants.LOOP_TIME);
    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    double uff = ElevatorConstants.MOTOR.rOhms*ElevatorConstants.DRUM_RADIUS*ElevatorConstants.CARRIAGE_MASS*Constants.GRAVITY_ACCELERATION/ElevatorConstants.GEARING/ElevatorConstants.MOTOR.KtNMPerAmp;
    
    // SmartDashboard.putNumber("position", getPosition());
    // SmartDashboard.putNumber("rightmotor", rightMotor.getPosition().getValueAsDouble());
    if(nextVoltage<0){
      nextVoltage+=uff;
    }
    //SmartDashboard.putNumber("voltage", nextVoltage);
    set(nextVoltage);
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(voltage);
    sim.update(Constants.LOOP_TIME);
    ligament.setLength(sim.getPositionMeters());
    rightMotor.getSimState().setRawRotorPosition(
        sim.getPositionMeters() / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS) * ElevatorConstants.GEARING);
  }

  private void set(double volts) {
    rightMotor.setVoltage(volts);
    voltage = volts;
  }

  public void resetEncoder(double height) {
    // Without the if statement, this causes loop overruns in simulation, and this
    // code does nothing anyway on sim (it sets the position to itself)
    if (RobotBase.isReal()) {
      rightMotor.setPosition(height / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS) * ElevatorConstants.GEARING);
    }
  }
  
  /**
   * Get the position of the elevator in  meters. 
  */
  public double getPosition() {
    return rightMotor.getPosition().getValueAsDouble() / ElevatorConstants.GEARING
        * (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
  }
  
  /**
   * Get the velocity of the elevator in m/s. 
  */
  public double getVelocity(){
    return rightMotor.getVelocity().getValueAsDouble()/ ElevatorConstants.GEARING
    * (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
  }

  public double getVoltage(){
    return voltage;
  }

  /**
   * Method to set the setpoint of the elevator. Clamped between min and max height.
   * @param setpoint The setpoint in meters.
  */
  public void setSetpoint(double setpoint) {
    this.setpoint = MathUtil.clamp(setpoint, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
  }

  /**
   * Get the velocity of the elevator in meters. 
  */
  public double getSetpoint() {
    return setpoint;
  }

  public Mechanism2d getMechanism2d() {
    return mechanism;
  }

  public boolean atSetpoint(){
    return Math.abs(getPosition() - setpoint) < 0.025;
  }

  /**
   * Get the COM at the current elevater height. 
  */
  public double getCenterOfMassHeight(){
    return (getPosition()-ElevatorConstants.MIN_HEIGHT)/(ElevatorConstants.MAX_HEIGHT-ElevatorConstants.MIN_HEIGHT)*(ElevatorConstants.CENTER_OF_MASS_HEIGHT_EXTENDED-ElevatorConstants.CENTER_OF_MASS_HEIGHT_STOWED)+ElevatorConstants.CENTER_OF_MASS_HEIGHT_STOWED;
  }
}
