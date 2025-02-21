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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IdConstants;
import frc.robot.util.AngledElevatorSim;

public class Elevator extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(IdConstants.ELEVATOR_RIGHT_MOTOR);
  private TalonFX leftMotor = new TalonFX(IdConstants.ELEVATOR_LEFT_MOTOR);

  private DigitalInput bottomLimitSwitch = new DigitalInput(IdConstants.ELEVATOR_BOTTOM_LIMIT_SWITCH);
  protected DIOSim bottomLimitSwitchSim;
  // TODO: Use or delete
  private boolean limitSwitchPressed = false;

  // Calibration variables
  private boolean calibrated;
  // TODO: Use or delete
  private boolean movingUp;
  private double start;

  private double setpoint = ElevatorConstants.START_HEIGHT;

  // Sim variables
  protected AngledElevatorSim sim;
  private Mechanism2d mechanism;
  private MechanismLigament2d ligament;
  protected double voltage;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          Units.feetToMeters(9.0),
          Units.feetToMeters(11.0))); // Max elevator speed and acceleration.
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
      VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(20.0)), // qelms. Position
      // and velocity error tolerances, in meters and meters per second. Decrease this
      // to more
      // heavily penalize state excursion, or make the controller behave more
      // aggressively. In
      // this example we weight position much more highly than velocity, but this can
      // be
      // tuned to balance the two.
      VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12
      // is a good
      // starting point because that is the (approximate) maximum voltage of a
      // battery.
      Constants.LOOP_TIME); // Nominal time between loops. 0.020 for TimedRobot, but can be

  // The state-space loop combines a controller, observer, feedforward and plant
  // for easy control.
  private final LinearSystemLoop<N2, N1, N2> m_loop = new LinearSystemLoop<>(
      (LinearSystem<N2, N1, N2>) m_elevatorPlant,
      m_controller,
      m_observer,
      12.0,
      Constants.LOOP_TIME);

  /** Creates a new Elevator. */
  public Elevator() {
    // Left motor follows right motor in the opposite direction
    if (!isSimulation()){
      leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
      rightMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    }
    

    // This increases both the time and memory efficiency of the code when running
    // on a real robot; do not remove this if statement
    if (isSimulation()) {
      sim = new AngledElevatorSim(ElevatorConstants.MOTOR, ElevatorConstants.GEARING, ElevatorConstants.CARRIAGE_MASS,
        ElevatorConstants.DRUM_RADIUS, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT, true,
        ElevatorConstants.START_HEIGHT, ElevatorConstants.ANGLE, ElevatorConstants.SPRING_FORCE);
      double width = ElevatorConstants.MAX_HEIGHT * Math.sin(ElevatorConstants.ANGLE);
      double height = ElevatorConstants.MAX_HEIGHT * Math.cos(ElevatorConstants.ANGLE);
      double size = Math.max(width, height);
      mechanism = new Mechanism2d(size, size);
      ligament = mechanism.getRoot("base", size / 2 - width / 2, size / 2 - height / 2).append(new MechanismLigament2d(
        "elevator", ElevatorConstants.START_HEIGHT, 90 - Units.radiansToDegrees(Math.abs(ElevatorConstants.ANGLE))));

      bottomLimitSwitchSim = new DIOSim(bottomLimitSwitch);
      bottomLimitSwitchSim.setValue(Math.abs(ElevatorConstants.START_HEIGHT
        - ElevatorConstants.BOTTOM_LIMIT_SWITCH_HEIGHT) > ElevatorConstants.SIM_LIMIT_SWITCH_TRIGGER_DISTANCE);
    }
    Timer.delay(1.0);
    m_loop.reset(VecBuilder.fill(getPosition(), 0));
    m_lastProfiledReference = new State(getPosition(), 0);

    resetEncoder(ElevatorConstants.START_HEIGHT);
    calibrate();
    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);

  }

  @Override
  public void periodic() {
    // The final state that the elevator is trying to get to
    State goal = new State(setpoint, 0.0);

    double currentPosition = getPosition();

    m_lastProfiledReference = m_profile.calculate(Constants.LOOP_TIME, m_lastProfiledReference, goal);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(MatBuilder.fill(Nat.N2(), Nat.N1(),currentPosition,getVelocity()));

    // Update our LQR to generate new voltage commands and use the voltages to
    // predict the next
    // state with out Kalman filter.
    m_loop.predict(Constants.LOOP_TIME);
    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    SmartDashboard.putBoolean("bottom Sensor", getBottomLimitSwitch());
    SmartDashboard.putNumber("next voltage", nextVoltage);
    SmartDashboard.putNumber("left motor encoder", getLeftEncoder());
    SmartDashboard.putNumber("right motor encoder", getRightEncoder());
    SmartDashboard.putNumber("height", getPosition());
    set(nextVoltage);
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(voltage);
    sim.update(Constants.LOOP_TIME);
    ligament.setLength(getPosition());
    bottomLimitSwitchSim.setValue(Math.abs(getPosition()
        - ElevatorConstants.BOTTOM_LIMIT_SWITCH_HEIGHT) > ElevatorConstants.SIM_LIMIT_SWITCH_TRIGGER_DISTANCE);
    setSimMotorPosition(
        sim.getPositionMeters() / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS) * ElevatorConstants.GEARING);
  }

  protected void setSimMotorPosition(double rotations){
    rightMotor.getSimState().setRawRotorPosition(rotations);
  }

  protected void set(double volts) {
    rightMotor.setVoltage(volts);
    voltage = volts;
  }

  protected double getLeftEncoder(){
    return leftMotor.getPosition().getValueAsDouble();
  }
  protected double getRightEncoder(){
    return rightMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoder(double height) {
    // Without the if statement, this causes loop overruns in simulation, and this
    // code does nothing anyway on sim (it sets the position to itself)
    if (!isSimulation()) {
      rightMotor.setPosition(height / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS) * ElevatorConstants.GEARING);
    }
  }

  public double getPosition() {
    return getRightEncoder() / ElevatorConstants.GEARING
        * (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
  }

  public double getVelocity(){
    return rightMotor.getVelocity().getValueAsDouble()/ ElevatorConstants.GEARING
    * (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
  }

  public double getVoltage(){
    return voltage;
  }



  public boolean getBottomLimitSwitch() {
    return !bottomLimitSwitch.get();
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = MathUtil.clamp(setpoint, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
  }

  public double getSetpoint() {
    return setpoint;
  }

  public Mechanism2d getMechanism2d() {
    return mechanism;
  }

  /**
   * Starts the elevator calibration.
   * @deprecated Currently not supported
   */
  @Deprecated
  public void calibrate() {
    calibrated = false;
    start = getPosition();
    // This prevents it from breaking on a second calibration
    movingUp = true;
    // If it is already at the limit switch, it can reset the encoder
    limitSwitchPressed = false;
  }

  public boolean isCalibrated() {
    return calibrated;
  }

  public double getCenterOfMassHeight(){
    return (getPosition()-ElevatorConstants.MIN_HEIGHT)/(ElevatorConstants.MAX_HEIGHT-ElevatorConstants.MIN_HEIGHT)*(ElevatorConstants.CENTER_OF_MASS_HEIGHT_EXTENDED-ElevatorConstants.CENTER_OF_MASS_HEIGHT_STOWED)+ElevatorConstants.CENTER_OF_MASS_HEIGHT_STOWED;
  }

  public boolean isSimulation(){
    return RobotBase.isSimulation();
  }

  /**
   * Closes the motors and sets them to null
   */
  public void deleteMotors(){
    leftMotor.close();
    rightMotor.close();
    leftMotor = null;
    rightMotor = null;
  }
}
