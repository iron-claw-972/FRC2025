// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
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
// import frc.robot.util.LogManager;
// import frc.robot.util.LogManager.LogLevel;

public class Elevator extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(IdConstants.ELEVATOR_RIGHT_MOTOR, Constants.CANIVORE_CAN);

  private double setpoint = ElevatorConstants.START_HEIGHT;
  
  MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

  private double maxVelocity = 3; // m/s
  private double maxAcceleration = 8
  ; // m/s
        
  // Sim variables
  private AngledElevatorSim sim;
  private Mechanism2d mechanism;
  private MechanismLigament2d ligament;
  private double voltage;
  // gravity feedforward
  double uff = ElevatorConstants.MOTOR.rOhms*ElevatorConstants.DRUM_RADIUS*ElevatorConstants.
  CARRIAGE_MASS*Constants.GRAVITY_ACCELERATION/ElevatorConstants.GEARING/ElevatorConstants.MOTOR.KtNMPerAmp;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();


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

    //m_lastProfiledReference = new ExponentialProfile.State(getPosition(),0);
    resetEncoder(ElevatorConstants.START_HEIGHT);

    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.5; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.GEARING * maxVelocity/ElevatorConstants.DRUM_RADIUS/Math.PI/2; // Target cruise velocity 
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.GEARING * maxAcceleration/ElevatorConstants.DRUM_RADIUS/Math.PI/2; // Target acceleration 
    rightMotor.getConfigurator().apply(talonFXConfigs);
    rightMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));


    //Logging
    // LogManager.logSupplier("Elevator/Voltage", () -> getVoltage(), 100, LogLevel.INFO);
    // LogManager.logSupplier("Elevator/Velocity", () -> getVelocity(), 100, LogLevel.INFO);
    // LogManager.logSupplier("Elevator/position", () -> getPosition(), 100, LogLevel.INFO);

  }

  @Override
  public void periodic() {
    double setpoint2 = ElevatorConstants.GEARING * setpoint / ElevatorConstants.DRUM_RADIUS/Math.PI/2;
    rightMotor.setControl(voltageRequest.withPosition(setpoint2).withFeedForward(0.15));

    inputs.measuredPosition = getPosition();
    inputs.velocity = getVelocity();
    inputs.currentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Setpoint", getSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(0);
    sim.update(Constants.LOOP_TIME);
    ligament.setLength(sim.getPositionMeters());
    rightMotor.getSimState().setRawRotorPosition(
        sim.getPositionMeters() / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS) * ElevatorConstants.GEARING);
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

  @AutoLogOutput
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
