package frc.robot.subsystems.drivetrain;

import java.util.Queue;

import org.littletonrobotics.junction.Logger;

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
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.PhoenixOdometryThread;
import lib.CTREModuleState;


public class Module implements ModuleIO{
    private final ModuleType type;
    
    // Degrees
    private final double angleOffset;

    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANcoder CANcoder;
    private SwerveModuleState desiredState;

    protected boolean stateDeadband = true;
    
    private boolean optimizeStates = true;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    // Timestamp inputs from Phoenix thread
    protected final Queue<Double> timestampQueue;
    protected final Queue<Double> drivePositionQueue;
    protected final Queue<Double> turnPositionQueue;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;

    protected final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private ModuleConstants moduleConstants;
      private final MotionMagicVelocityVoltage velocityRequest =
      new MotionMagicVelocityVoltage(0.0).withUpdateFreqHz(0);


    public Module(ModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;
        
        type = moduleConstants.getType();
        angleOffset = moduleConstants.getSteerOffset();

        /* Angle Encoder Config */
        CANcoder = new CANcoder(moduleConstants.getEncoderPort(), DriveConstants.STEER_ENCODER_CAN);
        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.getSteerPort(), DriveConstants.STEER_ENCODER_CAN);
        driveMotor = new TalonFX(moduleConstants.getDrivePort(), DriveConstants.DRIVE_MOTOR_CAN);
        // Create drive status signals
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getStatorCurrent();
      
        // Create turn status signals
        turnAbsolutePosition = CANcoder.getAbsolutePosition();
        turnPosition = angleMotor.getPosition();
        turnVelocity = angleMotor.getVelocity();
        turnAppliedVolts = angleMotor.getMotorVoltage();
        turnCurrent = angleMotor.getStatorCurrent();
      
        // Create timestamp queue
        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition());
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(angleMotor.getPosition());
        updateInputs();
        
        configCANcoder();
        configAngleMotor();
        configDriveMotor();

        driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(moduleConstants.ordinal()) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(moduleConstants.ordinal()) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(moduleConstants.ordinal()) + ".",
            AlertType.kError);        
   

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            250, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, angleMotor);
        
        setDesiredState(new SwerveModuleState(0, getAngle()), false);
    }

    public void close() {
        angleMotor.close();
        driveMotor.close();
        CANcoder.close();
    }

    @Override
    public void updateInputs() {
      // Refresh all signals
      var driveStatus =
          BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
      var turnStatus =
          BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
      var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);
  
      // Update drive inputs
      inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
      inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()/DriveConstants.DRIVE_GEAR_RATIO);
      inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()/DriveConstants.DRIVE_GEAR_RATIO);
      inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
      inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
  
      // Update turn inputs
      inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
      inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
      inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
      inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble()/DriveConstants.MODULE_CONSTANTS.angleGearRatio);
      inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble()/DriveConstants.MODULE_CONSTANTS.angleGearRatio);
      inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
      inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    }
    
    public void periodic() {
        updateInputs();
        Logger.processInputs("Drive/Module" + Integer.toString(moduleConstants.ordinal()), inputs);

         // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
        double positionMeters = inputs.odometryDrivePositionsRad[i]/DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_RADIUS;
        Rotation2d angle = inputs.odometryTurnPositions[i].div(DriveConstants.MODULE_CONSTANTS.angleGearRatio);
        odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
        Logger.recordOutput("Angle "+ moduleConstants.ordinal(), MathUtil.inputModulus(getAngle().getDegrees(), 0, 360));
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
        setAngle();
        setSpeed(isOpenLoop);
    }

    public void setSpeed(boolean isOpenLoop) {
        if(desiredState == null){
            return;
        }
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond/DriveConstants.WHEEL_RADIUS/2/Math.PI*DriveConstants.DRIVE_GEAR_RATIO;
            Logger.recordOutput("desired vel" + moduleConstants.ordinal(), velocity);
            
            driveMotor.setControl(
                velocityRequest
                    .withVelocity(velocity));
                    ///.withFeedForward(feedforward));
        }     
    }

    private void setAngle() {
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
        return inputs.turnPosition;
    }

    public Rotation2d getCANcoder() {
        return inputs.turnAbsolutePosition;
    }

    public void resetToAbsolute() {
        // Sensor ticks
        double absolutePosition = getCANcoder().getRotations() - Units.degreesToRotations(angleOffset);
        angleMotor.setPosition(absolutePosition*DriveConstants.MODULE_CONSTANTS.angleGearRatio);
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
        
        resetToAbsolute();
    }

    /**
     * @return Speed in RPM
     */
    public double getDriveVelocity() {
        return inputs.driveVelocityRadPerSec*60/DriveConstants.MODULE_CONSTANTS.driveGearRatio/2/Math.PI;
    }

    public double getDriveVoltage(){
        return inputs.driveAppliedVolts;
    }

    public double getDriveStatorCurrent(){
        return inputs.driveCurrentAmps;
    }
    private void configDriveMotor() {
        var talonFXConfigs = new TalonFXConfiguration();
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = DriveConstants.MAX_SPEED/DriveConstants.WHEEL_CIRCUMFERENCE * DriveConstants.DRIVE_GEAR_RATIO;
        motionMagicConfigs.MotionMagicAcceleration = DriveConstants.MAX_DRIVE_ACCEL/DriveConstants.WHEEL_CIRCUMFERENCE * DriveConstants.DRIVE_GEAR_RATIO;
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.11; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.006; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = moduleConstants.getDriveP(); // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = moduleConstants.getDriveI(); // no output for integrated error
        slot0Configs.kD = moduleConstants.getDriveD(); // A velocity error of 1 rps results in 0.1 V output
        driveMotor.getConfigurator().apply(talonFXConfigs);
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = DriveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        config.SupplyCurrentLimit = DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        config.SupplyCurrentLowerLimit = DriveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        config.SupplyCurrentLowerTime = DriveConstants.DRIVE_PEAK_CURRENT_DURATION;
        config.StatorCurrentLimit = DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        config.StatorCurrentLimitEnable = DriveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        driveMotor.getConfigurator().apply(config);
        driveMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(DriveConstants.INVERT_DRIVE_MOTOR));
        driveMotor.getConfigurator().apply(new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(DriveConstants.OPEN_LOOP_RAMP));
        driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(DriveConstants.OPEN_LOOP_RAMP));
        driveMotor.setNeutralMode(DriveConstants.DRIVE_NEUTRAL_MODE);
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                inputs.driveVelocityRadPerSec*DriveConstants.WHEEL_RADIUS,
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                inputs.drivePositionRad*DriveConstants.WHEEL_RADIUS,
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

        /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

}