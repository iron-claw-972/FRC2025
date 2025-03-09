package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.swerve.DriveConstants;

public class Arm extends SubsystemBase{
    //motor
    private TalonFX motor = new TalonFX(IdConstants.ARM_MOTOR);
    private TalonFXSimState encoderSim;
    double offset = 0;

    //Mechism2d display
    private Mechanism2d simulationMechanism;
    private MechanismLigament2d simLigament;
    private SingleJointedArmSim armSim;

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(5);

    private double setpoint = Units.rotationsToDegrees(absoluteEncoder.get());

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    // rad/s and rad/s^2
    private double maxVelocity = 5;
    private double maxAcceleration = 8;

    private final ArmFeedforward feedforward = new ArmFeedforward(
        0, 
        ArmConstants.MASS*ArmConstants.CENTER_OF_MASS_LENGTH/ArmConstants.GEAR_RATIO/ArmConstants.MOTOR.KtNMPerAmp*ArmConstants.MOTOR.rOhms, 
        0);

    public Arm() {
        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();
            encoderSim.setRawRotorPosition(Units.degreesToRotations(ArmConstants.START_ANGLE)*ArmConstants.GEAR_RATIO);
            armSim = new SingleJointedArmSim(
                ArmConstants.MOTOR, 
                ArmConstants.GEAR_RATIO,
                ArmConstants.MOI, 
                ArmConstants.LENGTH, 
                Units.degreesToRadians(ArmConstants.MIN_ANGLE), //min angle
                Units.degreesToRadians(ArmConstants.MAX_ANGLE), //max angle
                true, 
                Units.degreesToRadians(ArmConstants.START_ANGLE));
            simulationMechanism = new Mechanism2d(3, 3);
            MechanismRoot2d root = simulationMechanism.getRoot("Arm", 1.5, 1.5);
            simLigament = root.append(
                new MechanismLigament2d("angle", 1, ArmConstants.START_ANGLE, 4, new Color8Bit(Color.kAliceBlue))
            );
            SmartDashboard.putData("Arm Display", simulationMechanism);
        }

        motor.setPosition(Units.degreesToRotations(ArmConstants.START_ANGLE)*ArmConstants.GEAR_RATIO);
        motor.setNeutralMode(NeutralModeValue.Brake);

        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        // TODO: tune later
        slot0Configs.kS = 0;  // Static friction compensation (should be >0 if friction exists)
        slot0Configs.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        slot0Configs.kA = 0;  // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        slot0Configs.kP = 0.5; // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        slot0Configs.kI = 0;   // Integral term (usually left at 0 for MotionMagic)
        slot0Configs.kD = 0;   // Derivative term (used to dampen oscillations)

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity * ArmConstants.GEAR_RATIO/Math.PI/2;
        motionMagicConfigs.MotionMagicAcceleration = maxAcceleration * ArmConstants.GEAR_RATIO/Math.PI/2;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(talonFXConfigs);

        resetAbsolute();
    }

    @Override
    public void periodic() {
        double setpointRadians = Units.degreesToRadians(setpoint) * ArmConstants.GEAR_RATIO;
        motor.setControl(voltageRequest.withPosition(setpointRadians).withFeedForward(feedforward.calculate(Units.degreesToRadians(getAngle()), 0)));
        SmartDashboard.putNumber("Angle", getAngle());
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInputVoltage(getAppliedVoltage());
        armSim.update(Constants.LOOP_TIME);

        double armRotations = Units.radiansToRotations(armSim.getAngleRads());
        encoderSim.setRawRotorPosition(armRotations * ArmConstants.GEAR_RATIO);
        
        simLigament.setAngle(getAngle());
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = MathUtil.clamp(setpoint, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Gets the angle of the arm
     * @return The angle in degrees
     */

    public double getAngle() {
        return Units.rotationsToDegrees(absoluteEncoder.get()/ArmConstants.GEAR_RATIO);
    }

    public void resetAbsolute(){
        double absolutePosition = absoluteEncoder.get() - Units.degreesToRotations(offset);
        motor.setPosition(absolutePosition * DriveConstants.MODULE_CONSTANTS.angleGearRatio);
    }    

}
