package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.util.ClimbArmSim;

public class Climb extends SubsystemBase {
    
    private static final double startingPosition = 0;
    private static final double extendPosition = 1;
    private static final double climbPosition = -1;

    //Motors
    // TODO: tune better on real robot
    private final PIDController pid = new PIDController(0.1, 0, 0.04);

    private TalonFX motor = new TalonFX(IdConstants.CLIMB_MOTOR);
    private final DCMotor climbGearBox = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    //Mechism2d display
    private final Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
    private final MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Climb", 1.5, 1.5);
    private final MechanismLigament2d simLigament = mechanismRoot.append(
        new MechanismLigament2d("angle", 1, startingPosition, 4, new Color8Bit(Color.kAntiqueWhite))
    );

    private final double versaPlanetaryGearRatio = 1.0;
    private final double climbGearRatio = 75.0/1.0;
    private final double totalGearRatio = versaPlanetaryGearRatio * climbGearRatio;

    private ClimbArmSim climbSim;

    private double power;

    private boolean resetting = false;

    public Climb() {
        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();
            encoderSim.setRawRotorPosition(Units.degreesToRotations(startingPosition)*totalGearRatio);

            climbSim = new ClimbArmSim(
                climbGearBox, 
                totalGearRatio, 
                0.1, 
                0.127, 
                0, //min angle 
                Units.degreesToRadians(90), //max angle
                true, 
                Units.degreesToRadians(startingPosition),
                60
            );

            climbSim.setIsClimbing(true);
        }

        pid.setIZone(1);
        pid.setSetpoint(Units.degreesToRadians(startingPosition));

        motor.setPosition(Units.degreesToRotations(startingPosition)*totalGearRatio);

        SmartDashboard.putData("PID", pid);
        SmartDashboard.putData("Climb Display", simulationMechanism);
    }

    @Override
    public void periodic() { 
        double motorPosition = motor.getPosition().getValueAsDouble();
        double currentPosition = Units.rotationsToRadians(motorPosition/totalGearRatio);
        SmartDashboard.putNumber("climb current", motor.getTorqueCurrent().getValueAsDouble());
        power = pid.calculate(currentPosition);

        if(resetting){
            power = -0.1;
        }

        motor.set(MathUtil.clamp(power, -0.1, 0.1));

        simLigament.setAngle(Units.radiansToDegrees(currentPosition));

        SmartDashboard.putNumber("Climb Position", getAngle());

        SmartDashboard.putNumber("Encoder Position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Velocity", motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        climbSim.setInput(power * Constants.ROBOT_VOLTAGE);
        climbSim.update(Constants.LOOP_TIME);

        double climbRotations = Units.radiansToRotations(climbSim.getAngleRads());
        encoderSim.setRawRotorPosition(climbRotations * totalGearRatio);
    }

    /**
     * Sets the motor to an angle from 0-90 deg
     * @param angle in degrees
     */
    public void setAngle(double angle) {
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(angle));
    }

    /**
     * Gets the current position of the motor in degrees
     * @return The angle in degrees
     */
    public double getAngle() {
        return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble() / totalGearRatio);
    }

    /**
     * Turns the motor to 90 degrees (extended positiion)
     */
    public void extend(){
        double extendAngle = Units.rotationsToDegrees(extendPosition);
        setAngle(extendAngle);
    }

    /**
     * Turns the motor to 0 degrees (climb position)
     */
    public void climb(){
        setAngle(Units.rotationsToDegrees(climbPosition));
    }

    /**
     * Turns the motor to 0 degrees (climb position)
     */
    public void stow(){
        setAngle(startingPosition);
    }

    public void reset(boolean resetting){
        this.resetting = resetting;
        if(!resetting){
            motor.setPosition(climbPosition*totalGearRatio);
            pid.setSetpoint(Units.degreesToRadians(startingPosition));
            pid.reset();
        }
    }

    public double getCurrent(){
        return motor.getStatorCurrent().getValueAsDouble();
    }
}