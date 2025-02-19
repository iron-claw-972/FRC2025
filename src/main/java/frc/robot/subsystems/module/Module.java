package frc.robot.subsystems.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.ConversionUtils;
import frc.robot.util.LogManager;
import frc.robot.util.LogManager.LogLevel;
import lib.CTREModuleState;


public class Module extends SubsystemBase {
    private final ModuleType type;
    
    // Degrees
    private final double angleOffset;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder CANcoder;
    private SwerveModuleState desiredState;

    protected boolean stateDeadband = true;

    final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0);
    
    private boolean optimizeStates = true;

    private StatusSignal<Angle> drivePosition;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<Angle> steerAngle;
    private StatusSignal<Angle> CANangle;


    private ModuleConstants moduleConstants;

    private final LinearSystem<N1, N1, N1> m_driveMotor = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), DriveConstants.WHEEL_MOI, DriveConstants.DRIVE_GEAR_RATIO );
    
  private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      (LinearSystem<N1, N1, N1>) m_driveMotor,
      VecBuilder.fill(20), // How accurate we
     
      VecBuilder.fill(0.001), // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position reading.
      Constants.LOOP_TIME);
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller = new LinearQuadraticRegulator<>(
      (LinearSystem<N1, N1, N1>) m_driveMotor,
      VecBuilder.fill(3), // qelms. Position
       
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
  private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
      (LinearSystem<N1, N1, N1>) m_driveMotor,
      m_controller,
      m_observer,
      12,
      Constants.LOOP_TIME);


    public Module(ModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;
        
        type = moduleConstants.getType();
        angleOffset = moduleConstants.getSteerOffset();

        /* Angle Encoder Config */
        CANcoder = new CANcoder(moduleConstants.getEncoderPort(), DriveConstants.STEER_ENCODER_CAN);
        configCANcoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.getSteerPort(), DriveConstants.STEER_ENCODER_CAN);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.getDrivePort(), DriveConstants.DRIVE_MOTOR_CAN);
        configDriveMotor();

        ParentDevice.optimizeBusUtilizationForAll(angleMotor, driveMotor, CANcoder);

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerAngle = angleMotor.getPosition();
        CANangle = CANcoder.getAbsolutePosition();

        StatusSignal.setUpdateFrequencyForAll(100, drivePosition, driveVelocity, steerAngle);
        StatusSignal.setUpdateFrequencyForAll(5, CANangle);


        m_loop.reset(VecBuilder.fill(driveMotor.getVelocity().getValueAsDouble()));

        setDesiredState(new SwerveModuleState(0, getAngle()), false);

        String directory_name = "Drivetrain/Module" + type.name();
        LogManager.logSupplier(directory_name +"/AngleDesired/", () -> getDesiredAngle().getRadians(), 1000, LogLevel.DEBUG);
        LogManager.logSupplier(directory_name +"/AngleActual/", () -> getAngle().getRadians(), 1000, LogLevel.DEBUG);
        LogManager.logSupplier(directory_name +"/VelocityDesired/", () -> getDesiredVelocity(), 100, LogLevel.INFO);
        LogManager.logSupplier(directory_name +"/VelocityActual/", () -> getState().speedMetersPerSecond, 100, LogLevel.INFO);
        LogManager.logSupplier(directory_name +"/DriveVoltage/", () -> driveMotor.getMotorVoltage().getValueAsDouble(), 1000, LogLevel.DEBUG);
        LogManager.logSupplier(directory_name +"/DriveCurrent/", () -> driveMotor.getStatorCurrent().getValueAsDouble(), 1000, LogLevel.DEBUG);
    }

    public void close() {
        angleMotor.close();
        driveMotor.close();
        CANcoder.close();
    }
    
    public void periodic() {
        refreshStatusSignals();
    }

    public void setDesiredState(SwerveModuleState wantedState, boolean isOpenLoop) {
        // Separate if here and in setAngle() to avoid warning
        if(!DriveConstants.DISABLE_DEADBAND_AND_OPTIMIZATION){
            /*
            * This is a custom optimize function, since default WPILib optimize assumes
            * continuous controller which CTRE and Rev onboard is not
            */
            desiredState = optimizeStates ? CTREModuleState.optimize(wantedState, getState().angle) : wantedState;
        }else{
            desiredState = wantedState;
        }
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(desiredState == null){
            return;
        }
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond/DriveConstants.WHEEL_RADIUS;
            m_loop.setNextR(velocity);
            // Correct our Kalman filter's state vector estimate with encoder data.
            m_loop.correct(MatBuilder.fill(Nat.N1(), Nat.N1(), driveMotor.getVelocity().getValueAsDouble()*2*Math.PI/DriveConstants.DRIVE_GEAR_RATIO));
            // Update our LQR to generate new voltage commands and use the voltages to
            // predict the next
            // state with out Kalman filter.
            m_loop.predict(Constants.LOOP_TIME);
            // Send the new calculated voltage to the motors.
            // voltage = duty cycle * battery voltage, so
            // duty cycle = voltage / battery voltage
            double nextVoltage = m_loop.getU(0);
            driveMotor.setControl(new VoltageOut(nextVoltage));
        }
        
    }

    private void setAngle(SwerveModuleState desiredState) {
        if(!DriveConstants.DISABLE_DEADBAND_AND_OPTIMIZATION){
            // Prevent rotating module if desired speed < 1%. Prevents jittering and unnecessary movement.
            if (stateDeadband && (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.MAX_SPEED * 0.01))) {
                stop();
                return;
            }
        }
        if(desiredState == null){
            return;
        }
        angleMotor.setControl(new PositionDutyCycle(desiredState.angle.getRotations()*DriveConstants.MODULE_CONSTANTS.angleGearRatio));
    }

    public void setDriveVoltage(Voltage voltage){
        driveMotor.setVoltage(voltage.baseUnitMagnitude());
    }
    public void setAngle(Rotation2d angle){
        angleMotor.setControl(new PositionDutyCycle(angle.getRotations()*DriveConstants.MODULE_CONSTANTS.angleGearRatio));
    }

    public void setOptimize(boolean enable) {
        optimizeStates = enable;
    }

    public byte getModuleIndex() {
        return type.id;
    }

    public Rotation2d getAngle() {
        
        return Rotation2d.fromRotations(
            steerAngle.getValueAsDouble()/DriveConstants.MODULE_CONSTANTS.angleGearRatio);
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition().getValueAsDouble()*360);
    }

    public void resetToAbsolute() {
        // Sensor ticks
        double absolutePosition = getCANcoder().getRotations() - Units.degreesToRotations(angleOffset);
        angleMotor.setPosition(absolutePosition*DriveConstants.MODULE_CONSTANTS.angleGearRatio);
    }

    public void refreshStatusSignals(){
        StatusSignal.refreshAll(drivePosition, driveVelocity, steerAngle);
    }

    private void configCANcoder() {
        CANcoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(1).
        withSensorDirection(DriveConstants.MODULE_CONSTANTS.canCoderInvert?SensorDirectionValue.Clockwise_Positive:SensorDirectionValue.CounterClockwise_Positive));
    }

    private void configAngleMotor() {
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = DriveConstants.STEER_ENABLE_CURRENT_LIMIT;
        config.SupplyCurrentLimit = DriveConstants.STEER_CONTINUOUS_CURRENT_LIMIT;
        config.SupplyCurrentLowerLimit = DriveConstants.STEER_PEAK_CURRENT_LIMIT;
        config.SupplyCurrentLowerTime = DriveConstants.STEER_PEAK_CURRENT_DURATION;
        angleMotor.getConfigurator().apply(config);
        angleMotor.getConfigurator().apply(new Slot0Configs()
            .withKP(DriveConstants.MODULE_CONSTANTS.angleKP)
            .withKI(DriveConstants.MODULE_CONSTANTS.angleKI)
            .withKD(DriveConstants.MODULE_CONSTANTS.angleKD));
        angleMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(DriveConstants.INVERT_STEER_MOTOR));
        angleMotor.setNeutralMode(DriveConstants.STEER_NEUTRAL_MODE);
        angleMotor.setPosition(0);
        m_VelocityVoltage.Slot = 0;
        
        resetToAbsolute();
    }

    /**
     * @return Speed in RPM
     */
    public double getDriveVelocity() {
        return driveVelocity.getValueAsDouble()*60/DriveConstants.MODULE_CONSTANTS.driveGearRatio;
    }

    public double getDriveVoltage(){
        return driveMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getDriveStatorCurrent(){
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }

    private void configDriveMotor() {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = DriveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        config.SupplyCurrentLimit = DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        config.SupplyCurrentLowerLimit = DriveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        config.SupplyCurrentLowerTime = DriveConstants.DRIVE_PEAK_CURRENT_DURATION;
        driveMotor.getConfigurator().apply(config);
        driveMotor.getConfigurator().apply(new Slot0Configs()
            .withKP(moduleConstants.getDriveP())
            .withKI(moduleConstants.getDriveI())
            .withKD(moduleConstants.getDriveD()));
        driveMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(DriveConstants.INVERT_DRIVE_MOTOR));
        driveMotor.getConfigurator().apply(new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(DriveConstants.OPEN_LOOP_RAMP));
        driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(DriveConstants.OPEN_LOOP_RAMP));
        driveMotor.setNeutralMode(DriveConstants.DRIVE_NEUTRAL_MODE);
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                ConversionUtils.falconToMPS(ConversionUtils.RPMToFalcon(driveMotor.getVelocity().getValueAsDouble()*60, 1), DriveConstants.WHEEL_CIRCUMFERENCE,
                                            DriveConstants.DRIVE_GEAR_RATIO),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                ConversionUtils.falconToMeters(ConversionUtils.degreesToFalcon(drivePosition.getValueAsDouble()*360, 1), DriveConstants.WHEEL_CIRCUMFERENCE,
                                               DriveConstants.DRIVE_GEAR_RATIO),
                getAngle());
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }


    public double getDriveVelocityError() {
        return getDesiredState().speedMetersPerSecond - getState().speedMetersPerSecond;
    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public TalonFX getDriveMotor(){
        return driveMotor;
    }

    public TalonFX getAngleMotor(){
        return angleMotor;
    }

    public ModuleType getModuleType(){
        return type;
    }

    public void setStateDeadband(boolean enabled) {
        stateDeadband = enabled;
    }

    public double getDesiredVelocity() {
        return getDesiredState().speedMetersPerSecond;
      }
    
      public Rotation2d getDesiredAngle() {
        return getDesiredState().angle;
      }

      /**
       * Get an array of this module's status signals
       * @return The array of BaseStatusSignals
       */
      public BaseStatusSignal[] getStatusSignals(){
        return new BaseStatusSignal[]{
            drivePosition,
            driveVelocity,
            steerAngle,
        };
      }

      /**
       * Closes the motors and CANcoder and sets them to null
       * Also deletes all status signals
       */
      protected void deleteMotors(){
          drivePosition = null;
          driveVelocity = null;
          steerAngle = null;
          CANangle = null;
        driveMotor.close();
        angleMotor.close();
        CANcoder.close();
        driveMotor = null;
        angleMotor = null;
        CANcoder = null;
      }
}