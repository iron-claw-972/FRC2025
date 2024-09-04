package frc.robot.subsystems.module;

import java.time.Duration;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.ConversionUtils;
import frc.robot.util.LogManager;
import lib.CTREModuleState;


public class Module extends SubsystemBase {
    private final ModuleType type;
    
    // Motor ticks
    private final double angleOffset;

    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANcoder CANcoder;
    private SwerveModuleState desiredState;

    protected boolean stateDeadband = true;

    private SimpleMotorFeedforward feedforward;
    
    final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0);
    
    private boolean optimizeStates = true;

    private ModuleConstants moduleConstants;


    public Module(ModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;

        type = moduleConstants.getType();
        feedforward = new SimpleMotorFeedforward(moduleConstants.getKs(), moduleConstants.getKv(), moduleConstants.getKa());
        //angleOffset = new Rotation2d(constants.getSteerOffset());
        angleOffset = moduleConstants.getSteerOffset();

        /* Angle Encoder Config */
        CANcoder = new CANcoder(moduleConstants.getEncoderPort(), DriveConstants.kSteerEncoderCAN);
        configCANcoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.getSteerPort(), DriveConstants.kSteerEncoderCAN);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.getDrivePort(), DriveConstants.kDriveMotorCAN);
        configDriveMotor();

        setDesiredState(new SwerveModuleState(0, getAngle()), false);
    }

    public void close() {
        angleMotor.close();
        driveMotor.close();
        CANcoder.close();
    }

    public void periodic() {
        
    }

    public void setDesiredState(SwerveModuleState wantedState, boolean isOpenLoop) {

        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = optimizeStates ? CTREModuleState.optimize(wantedState, getState().angle) : wantedState;
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeed;
            driveMotor.set(percentOutput);
        } else {
            double velocity = ConversionUtils.falconToRPM(ConversionUtils.MPSToFalcon(desiredState.speedMetersPerSecond, DriveConstants.kWheelCircumference,
                DriveConstants.kDriveGearRatio), 1)/60;
            // TODO: This curently doesn't use the feedforward.
            driveMotor.setControl(m_VelocityVoltage.withVelocity(velocity).withEnableFOC(true).withFeedForward(feedforward.calculate(velocity)));
        }
        if (Constants.DO_LOGGING) {
            String directory_name = "Drivetrain/Module" + type.name();
            LogManager.add(directory_name +"/DriveSpeedActual/" , () -> ConversionUtils.falconToMPS(ConversionUtils.RPMToFalcon(driveMotor.getVelocity().getValue()/60, 1), DriveConstants.kWheelCircumference,
                DriveConstants.kDriveGearRatio), Duration.ofSeconds(1));
            LogManager.add(directory_name +"/DriveSpeedDesired/", () -> desiredState.speedMetersPerSecond, Duration.ofSeconds(1));
            LogManager.add(directory_name +"/AngleDesired/", () -> getDesiredAngle().getRadians(), Duration.ofSeconds(1));
            LogManager.add(directory_name +"/AngleActual/", () -> getAngle().getRadians(), Duration.ofSeconds(1));
            LogManager.add(directory_name +"/VelocityDesired/", () -> getDesiredVelocity(), Duration.ofSeconds(1));
            LogManager.add(directory_name +"/VelocityActual/", () -> getState().speedMetersPerSecond, Duration.ofSeconds(1));
            LogManager.add(directory_name +"/DriveVoltage/", () -> driveMotor.getMotorVoltage().getValue(), Duration.ofSeconds(1));
            LogManager.add(directory_name +"/DriveCurrent/", () -> driveMotor.getStatorCurrent().getValue(), Duration.ofSeconds(1));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if desired speed < 1%. Prevents Jittering.
        if (stateDeadband && (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01))) {
            stop();
            return;
        }
        // angleMotor.setControl(new PositionDutyCycle(3));
        angleMotor.setControl(new PositionDutyCycle(desiredState.angle.getRotations()*DriveConstants.kModuleConstants.angleGearRatio));
    }

    public void setDriveVoltage(Measure<Voltage> voltage){
        driveMotor.setVoltage(voltage.baseUnitMagnitude());
    }
    public void setAngle(Rotation2d angle){
        angleMotor.setControl(new PositionDutyCycle(angle.getRotations()*DriveConstants.kModuleConstants.angleGearRatio));
    }

    public void setOptimize(boolean enable) {
        optimizeStates = enable;
    }

    public byte getModuleIndex() {
        return type.id;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(
                angleMotor.getPosition().getValue()/DriveConstants.kModuleConstants.angleGearRatio);
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition().getValue()*360);
    }

    public void resetToAbsolute() {
        // Sensor ticks
        double absolutePosition = getCANcoder().getRotations() - Units.degreesToRotations(angleOffset);
        angleMotor.setPosition(absolutePosition*DriveConstants.kModuleConstants.angleGearRatio);
    }

    private void configCANcoder() {
        CANcoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(DriveConstants.kModuleConstants.canCoderInvert?SensorDirectionValue.Clockwise_Positive:SensorDirectionValue.CounterClockwise_Positive));
    }

    private void configAngleMotor() {
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = DriveConstants.kAngleEnableCurrentLimit;
        config.SupplyCurrentLimit = DriveConstants.kAngleContinuousCurrentLimit;
        config.SupplyCurrentThreshold = DriveConstants.kAnglePeakCurrentLimit;
        config.SupplyTimeThreshold = DriveConstants.kAnglePeakCurrentDuration;
        angleMotor.getConfigurator().apply(config);
        angleMotor.getConfigurator().apply(new Slot0Configs()
            .withKP(DriveConstants.kModuleConstants.angleKP)
            .withKI(DriveConstants.kModuleConstants.angleKI)
            .withKD(DriveConstants.kModuleConstants.angleKD));
        angleMotor.setInverted(DriveConstants.kAngleMotorInvert);
        angleMotor.setNeutralMode(DriveConstants.kAngleNeutralMode);
        angleMotor.setPosition(0);
        m_VelocityVoltage.Slot = 0;
        
        resetToAbsolute();
    }

    /**
     * @return Speed in RPM
     */
    public double getSteerVelocity() {
        return angleMotor.getVelocity().getValue()/DriveConstants.kModuleConstants.angleGearRatio*60;
    }
    /**
     * @return Speed in RPM
     */
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue()*60/DriveConstants.kModuleConstants.driveGearRatio;
    }

    public double getDriveVoltage(){
        return driveMotor.getMotorVoltage().getValue();
    }

    public double getDriveStatorCurrent(){
        return driveMotor.getStatorCurrent().getValue();
    }

    private void configDriveMotor() {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = DriveConstants.kDriveEnableCurrentLimit;
        config.SupplyCurrentLimit = DriveConstants.kDriveContinuousCurrentLimit;
        config.SupplyCurrentThreshold = DriveConstants.kDrivePeakCurrentLimit;
        config.SupplyTimeThreshold = DriveConstants.kDrivePeakCurrentDuration;
        driveMotor.getConfigurator().apply(config);
        driveMotor.getConfigurator().apply(new Slot0Configs()
            .withKP(moduleConstants.getDriveP())
            .withKI(moduleConstants.getDriveI())
            .withKD(moduleConstants.getDriveD()));
        driveMotor.getConfigurator().apply(new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(DriveConstants.kOpenLoopRamp));
        driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(DriveConstants.kOpenLoopRamp));
        driveMotor.setInverted(DriveConstants.kDriveMotorInvert);
        driveMotor.setNeutralMode(DriveConstants.kDriveNeutralMode);
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                ConversionUtils.falconToMPS(ConversionUtils.RPMToFalcon(driveMotor.getVelocity().getValue()*60, 1), DriveConstants.kWheelCircumference,
                                            DriveConstants.kDriveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                ConversionUtils.falconToMeters(ConversionUtils.degreesToFalcon(driveMotor.getPosition().getValue()*360, 1), DriveConstants.kWheelCircumference,
                                               DriveConstants.kDriveGearRatio),
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
}