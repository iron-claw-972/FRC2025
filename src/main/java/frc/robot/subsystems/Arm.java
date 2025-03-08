package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
    private TalonFX motor = new TalonFX(IdConstants.ARM_MOTOR, Constants.CANIVORE_CAN);
    private final DCMotor armGearBox = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;
    double offset = 0;

    //Mechism2d display
    private final Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
    private final MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Arm", 1.5, 1.5);
    private final MechanismLigament2d simLigament = mechanismRoot.append(
        new MechanismLigament2d("angle", 1, ArmConstants.START_ANGLE, 4, new Color8Bit(Color.kAliceBlue))
    );
    private SingleJointedArmSim armSim;

    private double setpoint = ArmConstants.START_ANGLE;

    MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    private double maxVelocity = 5;
    private double maxAcceleration = 8;

    public Arm() {
        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();
            encoderSim.setRawRotorPosition(Units.degreesToRotations(ArmConstants.START_ANGLE)*ArmConstants.GEAR_RATIO);
            armSim = new SingleJointedArmSim(
                armGearBox, 
                ArmConstants.GEAR_RATIO,
                0.1, 
                0.127, 
                Units.degreesToRadians(0), //min angle
                Units.degreesToRadians(360), //max angle
                true, 
                ArmConstants.GEAR_RATIO);
            SmartDashboard.putData("Arm Display", simulationMechanism);
        }

        motor.setPosition(Units.degreesToRotations(ArmConstants.START_ANGLE)*ArmConstants.GEAR_RATIO);
        motor.setNeutralMode(NeutralModeValue.Brake);

        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        //tune later
        slot0Configs.kS = 0.3;  // Static friction compensation (should be >0 if friction exists)
        slot0Configs.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        slot0Configs.kA = 0;  // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        slot0Configs.kP = 0.5; // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        slot0Configs.kI = 0;   // Integral term (usually left at 0 for MotionMagic)
        slot0Configs.kD = 0;   // Derivative term (used to dampen oscillations)

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity * ArmConstants.GEAR_RATIO/Math.PI/2;
        motionMagicConfigs.MotionMagicAcceleration = maxAcceleration * ArmConstants.GEAR_RATIO/Math.PI/2;
        motor.getConfigurator().apply(talonFXConfigs);
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        resetAbsolute();
    }

    @Override
    public void periodic() {
        double setpointRotations = Units.degreesToRotations(setpoint) * ArmConstants.GEAR_RATIO;
        motor.setControl(voltageRequest.withPosition(setpointRotations).withFeedForward(0.15));
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInputVoltage(motor.getSimState().getMotorVoltage());
        armSim.update(Constants.LOOP_TIME);

        double armRotations = Units.radiansToRotations(armSim.getAngleRads());
        encoderSim.setRawRotorPosition(armRotations * ArmConstants.GEAR_RATIO);
        
        simLigament.setAngle(getAngle());
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = MathUtil.clamp(setpoint, -360, 360);
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getAngle() {
        return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()/ArmConstants.GEAR_RATIO);
    }

    public void resetAbsolute(){
        double absolutePosition = motor.getPosition().getValueAsDouble() - Units.degreesToRotations(offset);
        motor.setPosition(absolutePosition * DriveConstants.MODULE_CONSTANTS.angleGearRatio);
    }    

}
