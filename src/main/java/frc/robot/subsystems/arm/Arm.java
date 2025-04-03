package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

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
import frc.robot.util.PhoenixUtil;

public class Arm extends SubsystemBase implements ArmIO {
    //motor
    private TalonFX motor = new TalonFX(IdConstants.ARM_MOTOR);
    private TalonFXSimState encoderSim;

    // Mechism2d display
    private Mechanism2d simulationMechanism;
    private MechanismLigament2d simLigament;
    private SingleJointedArmSim armSim;

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(IdConstants.ARM_ABSOLUTE_ENCODER);

    private double setpoint = ArmConstants.START_ANGLE;

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    private final ArmFeedforward feedforward = new ArmFeedforward(0, ArmConstants.MASS*ArmConstants.CENTER_OF_MASS_LENGTH/ArmConstants.GEAR_RATIO/ArmConstants.MOTOR.KtNMPerAmp*ArmConstants.MOTOR.rOhms, 0);

    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private BooleanSupplier elevatorStowed;

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

        // resetAbsolute();
        motor.setPosition(Units.degreesToRotations(ArmConstants.START_ANGLE)*ArmConstants.GEAR_RATIO);

        var talonFXConfigs = new TalonFXConfiguration();
        
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.1;  // Static friction compensation (should be >0 if friction exists)
        slot0Configs.kG = ArmConstants.MASS * 9.81 * ArmConstants.CENTER_OF_MASS_LENGTH / ArmConstants.GEAR_RATIO; // Gravity compensation
        slot0Configs.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        slot0Configs.kA = 0;  // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        slot0Configs.kP = 0.85; // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        slot0Configs.kI = 0.;   // Integral term (usually left at 0 for MotionMagic)
        slot0Configs.kD = 0;   // Derivative term (used to dampen oscillations)

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY * ArmConstants.GEAR_RATIO/Math.PI/2;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION * ArmConstants.GEAR_RATIO/Math.PI/2;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(talonFXConfigs);
        updateInputs();
        PhoenixUtil.tryUntilOk(100, ()->motor.setNeutralMode(NeutralModeValue.Brake));

    }

    public void setElevatorStowed(BooleanSupplier elevatorStowed){
        this.elevatorStowed = elevatorStowed;
    }

    @Override
    public void periodic() {
        double setpoint2 = setpoint;
        if(elevatorStowed == null || elevatorStowed.getAsBoolean() && Math.abs(setpoint2-ArmConstants.L1_SETPOINT) > 0.0001){
            setpoint2 = ArmConstants.START_ANGLE;
        }
        double setpointRotations = Units.degreesToRotations(setpoint2) * ArmConstants.GEAR_RATIO;
        motor.setControl(voltageRequest.withPosition(setpointRotations).withFeedForward(feedforward.calculate(Units.degreesToRadians(getAngle()), 0)));
        updateInputs();
        Logger.recordOutput("Arm/Atsetpoint",atSetpoint());
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
        return inputs.measuredAngle;
    }

    public void resetAbsolute(){
        if(RobotBase.isSimulation()){
            motor.setPosition(Units.degreesToRotations(ArmConstants.START_ANGLE)*ArmConstants.GEAR_RATIO);
        }else{
            double absolutePosition = absoluteEncoder.get() / ArmConstants.ENCODER_GEAR_RATIO;
            motor.setPosition(MathUtil.inputModulus(absolutePosition - Units.degreesToRotations(ArmConstants.OFFSET), -0.5, 0.5)*ArmConstants.GEAR_RATIO);
        }
    }

    public boolean atSetpoint() {
        return Math.abs(getAngle() - setpoint) < ArmConstants.TOLERANCE;
    }

    public boolean canMoveElevator() {
        return Math.abs(getAngle() - ArmConstants.START_ANGLE) < 5 || Math.abs(getAngle() - ArmConstants.L1_SETPOINT) < 5;
    }

    @Override
    public void updateInputs(){
        inputs.measuredAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / ArmConstants.GEAR_RATIO;
        inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();

        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/setpointDeg", setpoint);
    }
}
