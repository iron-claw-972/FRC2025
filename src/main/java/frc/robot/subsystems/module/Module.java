package frc.robot.subsystems.module;


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
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        feedforward = new SimpleMotorFeedforward(moduleConstants.getDriveS(), moduleConstants.getDriveV(), moduleConstants.getDriveA());
        //angleOffset = new Rotation2d(constants.getSteerOffset());
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

        setDesiredState(new SwerveModuleState(0, getAngle()), false);
    

        String directory_name = "Drivetrain/Module" + type.name();
        LogManager.logSupplier(directory_name +"/DriveSpeedActual/" , () -> ConversionUtils.falconToMPS(ConversionUtils.RPMToFalcon(driveMotor.getVelocity().getValueAsDouble()/60, 1), DriveConstants.WHEEL_CIRCUMFERENCE,
        DriveConstants.DRIVE_GEAR_RATIO), 1000);
        LogManager.logSupplier(directory_name +"/DriveSpeedDesired/", () -> desiredState.speedMetersPerSecond, 1000);
        LogManager.logSupplier(directory_name +"/AngleDesired/", () -> getDesiredAngle().getRadians(), 1000);
        LogManager.logSupplier(directory_name +"/AngleActual/", () -> getAngle().getRadians(), 1000);
        LogManager.logSupplier(directory_name +"/VelocityDesired/", () -> getDesiredVelocity(), 1000);
        LogManager.logSupplier(directory_name +"/VelocityActual/", () -> getState().speedMetersPerSecond, 1000);
        LogManager.logSupplier(directory_name +"/DriveVoltage/", () -> driveMotor.getMotorVoltage().getValue(), 1000);
        LogManager.logSupplier(directory_name +"/DriveCurrent/", () -> driveMotor.getStatorCurrent().getValue(), 1000);

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
            double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = ConversionUtils.falconToRPM(ConversionUtils.MPSToFalcon(desiredState.speedMetersPerSecond, DriveConstants.WHEEL_CIRCUMFERENCE,
                DriveConstants.DRIVE_GEAR_RATIO), 1)/60;
            // TODO: This curently doesn't use the feedforward.
            // TODO: Maybe use current and next velocity instead of only 1 parameter
            driveMotor.setControl(m_VelocityVoltage.withVelocity(velocity).withEnableFOC(true).withFeedForward(feedforward.calculate(velocity)));
        }
        
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if desired speed < 1%. Prevents Jittering.
        if (stateDeadband && (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.MAX_SPEED * 0.01))) {
            stop();
            return;
        }
        // angleMotor.setControl(new PositionDutyCycle(3));
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
                angleMotor.getPosition().getValueAsDouble()/DriveConstants.MODULE_CONSTANTS.angleGearRatio);
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(CANcoder.getAbsolutePosition().getValueAsDouble()*360);
    }

    public void resetToAbsolute() {
        // Sensor ticks
        double absolutePosition = getCANcoder().getRotations() - Units.degreesToRotations(angleOffset);
        angleMotor.setPosition(absolutePosition*DriveConstants.MODULE_CONSTANTS.angleGearRatio);
    }

    private void configCANcoder() {
        CANcoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(DriveConstants.MODULE_CONSTANTS.canCoderInvert?SensorDirectionValue.Clockwise_Positive:SensorDirectionValue.CounterClockwise_Positive));
    }

    private void configAngleMotor() {
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = DriveConstants.STEER_ENABLE_CURRENT_LIMIT;
        config.SupplyCurrentLimit = DriveConstants.STEER_CONTINUOUS_CURRENT_LIMIT;
        config.SupplyCurrentThreshold = DriveConstants.STEER_PEAK_CURRENT_LIMIT;
        config.SupplyTimeThreshold = DriveConstants.STEER_PEAK_CURRENT_DURATION;
        angleMotor.getConfigurator().apply(config);
        angleMotor.getConfigurator().apply(new Slot0Configs()
            .withKP(DriveConstants.MODULE_CONSTANTS.angleKP)
            .withKI(DriveConstants.MODULE_CONSTANTS.angleKI)
            .withKD(DriveConstants.MODULE_CONSTANTS.angleKD));
        angleMotor.setInverted(DriveConstants.INVERT_STEER_MOTOR);
        angleMotor.setNeutralMode(DriveConstants.STEER_NEUTRAL_MODE);
        angleMotor.setPosition(0);
        m_VelocityVoltage.Slot = 0;
        
        resetToAbsolute();
    }

    /**
     * @return Speed in RPM
     */
    public double getSteerVelocity() {
        return angleMotor.getVelocity().getValueAsDouble()/DriveConstants.MODULE_CONSTANTS.angleGearRatio*60;
    }
    /**
     * @return Speed in RPM
     */
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble()*60/DriveConstants.MODULE_CONSTANTS.driveGearRatio;
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
        config.SupplyCurrentThreshold = DriveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        config.SupplyTimeThreshold = DriveConstants.DRIVE_PEAK_CURRENT_DURATION;
        driveMotor.getConfigurator().apply(config);
        driveMotor.getConfigurator().apply(new Slot0Configs()
            .withKP(moduleConstants.getDriveP())
            .withKI(moduleConstants.getDriveI())
            .withKD(moduleConstants.getDriveD()));
        driveMotor.getConfigurator().apply(new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(DriveConstants.OPEN_LOOP_RAMP));
        driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(DriveConstants.OPEN_LOOP_RAMP));
        driveMotor.setInverted(DriveConstants.INVERT_DRIVE_MOTOR);
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
                ConversionUtils.falconToMeters(ConversionUtils.degreesToFalcon(driveMotor.getPosition().getValueAsDouble()*360, 1), DriveConstants.WHEEL_CIRCUMFERENCE,
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
}