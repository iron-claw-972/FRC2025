// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
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
import frc.robot.util.LogManager;
import frc.robot.util.LogManager.LogLevel;

public class Elevator extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(IdConstants.ELEVATOR_RIGHT_MOTOR, Constants.CANIVORE_CAN);
  private TalonFX leftMotor = new TalonFX(IdConstants.ELEVATOR_LEFT_MOTOR, Constants.CANIVORE_CAN);

  private DigitalInput bottomLimitSwitch = new DigitalInput(IdConstants.ELEVATOR_BOTTOM_LIMIT_SWITCH);
  private DIOSim bottomLimitSwitchSim;
  private boolean limitSwitchPressed = false;
  private TalonFX leftMotor = new TalonFX(IdConstants.ELEVATOR_LEFT_MOTOR, Constants.CANIVORE_CAN);

  // private ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
  // private GenericEntry Voltage = tab.add("Voltage", 0).getEntry();
  // private GenericEntry height = tab.add("Height", 0).getEntry();
  // private GenericEntry leftMotorEncoder = tab.add("leftEncoder", 0).getEntry();
  // private GenericEntry rightMotorEncoder = tab.add("rightEncoder", 0).getEntry();
  // private GenericEntry Setpoint = tab.add("setpoint", 0).getEntry();
  // private GenericEntry bottomSensor = tab.add("bottom sensor", 0).getEntry();
  // private GenericEntry topSensor = tab.add("top sensor", 0).getEntry();

  // Calibration variables
  private boolean calibrated;
  private boolean movingUp;
  private double start;

  private double setpoint = ElevatorConstants.START_HEIGHT;

  // Sim variables
  private AngledElevatorSim sim;
  private Mechanism2d mechanism;
  private MechanismLigament2d ligament;
  private double voltage;

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
      VecBuilder.fill(0.001, 0.001), // How accurate we think our encoder position
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
      VecBuilder.fill(2.0), // relms. Control effort (voltage) tolerance. Decrease this to more
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
      2.0,
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
      //tab.add("Elevator", getMechanism2d());
    }
    Timer.delay(1.0);
    m_loop.reset(VecBuilder.fill(getPosition(), 0));
    m_lastProfiledReference = new State(getPosition(), 0);

    resetEncoder(ElevatorConstants.START_HEIGHT);
    calibrate();
    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);
    ParentDevice.optimizeBusUtilizationForAll(leftMotor);
    
    LogManager.logSupplier("Elevator/voltage", () -> getVoltage(), 100, LogLevel.COMP);
    LogManager.logSupplier("Elevator/position", () -> getPosition(), 100, LogLevel.COMP);
    LogManager.logSupplier("Elevator/velocity", () -> getVelocity(), 100, LogLevel.COMP);
  }
  // 14 inches verically
  // 2.901 icnehs toward battery
  // 16.901 inches 
  @Override
  public void periodic() {
    //System.out.println(leftMotor.getDeviceID());
    // If it hits the limit switch, reset the encoder 
    // if (getBottomLimitSwitch() && (calibrated || !movingUp)) {
    //   if (false) {
    //     resetEncoder(ElevatorConstants.BOTTOM_LIMIT_SWITCH_HEIGHT);
    //   }
    //   calibrated = true;
    //   limitSwitchPressed = true;
    // } else if (getTopLimitSwitch()) {
    //   if (false) {
    //     resetEncoder(ElevatorConstants.TOP_LIMIT_SWITCH_HEIGHT);
    //   }
    //   calibrated = true;
    //   limitSwitchPressed = true;
    // } else {
    //   limitSwitchPressed = false;
    // }
    // SmartDashboard.putNumber("position", getPosition());
    // SmartDashboard.putNumber("velocity", getVelocity());
    // // The final state that the elevator is trying to get to
     State goal = new State(setpoint, 0.0);

     double currentPosition = getPosition();

    // // If it isn't calibrated yet, try to find the limit switch
    // if (!calibrated) {
    //   double v = -0.25;
    //   if (movingUp) {
    //     // This is only to get above the limit switch, so it can move faster
    //     v = 0.5;
    //     if (currentPosition - start > ElevatorConstants.BOTTOM_LIMIT_SWITCH_HEIGHT) {
    //       movingUp = false;
    //     }
    //   }
    //   goal = new State(currentPosition + v * Constants.LOOP_TIME, v);
    // }

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
    SmartDashboard.putNumber("voltage", voltage);
    SmartDashboard.putNumber("position",currentPosition);
    SmartDashboard.putNumber("left motor encoder", leftMotor.getPosition().getValueAsDouble());
    // rightMotorEncoder.setDouble(rightMotor.getPosition().getValue());
    // bottomSensor.setBoolean(getBottomLimitSwitch());
    // topSensor.setBoolean(getTopLimitSwitch());
  
    //Voltage.setDouble(nextVoltage);
    set(nextVoltage);
    ElevatorConstants.CENTER_OF_MASS_HEIGHT = (getPosition()-ElevatorConstants.MIN_HEIGHT)/(ElevatorConstants.MAX_HEIGHT-ElevatorConstants.MIN_HEIGHT)*(ElevatorConstants.CENTER_OF_MASS_HEIGHT_EXTENDED-ElevatorConstants.CENTER_OF_MASS_HEIGHT_STOWED)+ElevatorConstants.CENTER_OF_MASS_HEIGHT_STOWED;
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(voltage);
    sim.update(Constants.LOOP_TIME);
    ligament.setLength(getPosition());
    bottomLimitSwitchSim.setValue(Math.abs(getPosition()
        - ElevatorConstants.BOTTOM_LIMIT_SWITCH_HEIGHT) > ElevatorConstants.SIM_LIMIT_SWITCH_TRIGGER_DISTANCE);
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
    if (!isSimulation()) {
      rightMotor.setPosition(height / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS) * ElevatorConstants.GEARING);
    }
  }

  public double getPosition() {
    return rightMotor.getPosition().getValueAsDouble() / ElevatorConstants.GEARING
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
   * <p>
   * Note: The elevator will probably break if the code is deployed and enabled
   * when the elvator is above the top limit switch
   */
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
}
