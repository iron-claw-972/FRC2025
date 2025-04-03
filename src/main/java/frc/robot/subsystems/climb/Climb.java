package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private static final double extendPosition = 2;
    private static final double climbPosition = -0.83;

    //Motors
    private final PIDController pid = new PIDController(2.5, 0, 0.0);

    private TalonFX motor = new TalonFX(IdConstants.CLIMB_MOTOR, Constants.CANIVORE_CAN);
    private final DCMotor climbGearBox = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    //Mechism2d display
    private final Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
    private final MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Climb", 1.5, 1.5);
    private final MechanismLigament2d simLigament = mechanismRoot.append(
        new MechanismLigament2d("angle", 1, startingPosition, 4, new Color8Bit(Color.kAntiqueWhite))
    );

    private final double versaPlanetaryGearRatio = 1.0;
    private final double climbGearRatio = 60.0/1.0;
    private final double totalGearRatio = versaPlanetaryGearRatio * climbGearRatio;

    private ClimbArmSim climbSim;

    private double power;

    private boolean resetting = false;

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

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
            SmartDashboard.putData("Climb Display", simulationMechanism);
        }

        pid.setIZone(1);

        pid.setSetpoint(Units.degreesToRadians(startingPosition));

        motor.setPosition(Units.degreesToRotations(startingPosition)*totalGearRatio);
        motor.setNeutralMode(NeutralModeValue.Brake);
        //SmartDashboard.putData("Climb PID", pid);
    }

    @Override
    public void periodic() { 

        inputs.measuredPositionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble() / totalGearRatio);
        inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
        Logger.processInputs("Climb", inputs);

        double motorPosition = motor.getPosition().getValueAsDouble();
        double currentPosition = Units.rotationsToRadians(motorPosition/totalGearRatio);
        power = pid.calculate(currentPosition);

        if(resetting){
            power = -0.1;
        }

        Logger.recordOutput("Climb/Motor Power", power);
        Logger.recordOutput("Climb/setpointDeg", motorPosition/totalGearRatio);

        motor.set(MathUtil.clamp(power, -1, 1));
    }


    @Override
    public void simulationPeriodic() {
        climbSim.setInput(power * Constants.ROBOT_VOLTAGE);
        climbSim.update(Constants.LOOP_TIME);

        double climbRotations = Units.radiansToRotations(climbSim.getAngleRads());
        encoderSim.setRawRotorPosition(climbRotations * totalGearRatio);

        simLigament.setAngle(Units.radiansToDegrees(getAngle()));
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
        return inputs.measuredPositionDeg;
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

    //not working
    public Mechanism2d getMech2d() {
        return simulationMechanism;
    }

    /**
     * Gets the estimated angle of the climb
     * This is slightly inaccurate since it assumes the climb rotates exactly 90 degrees and the motor position is proportional to the climb position
     * Used only for 3D robot display
     */
    public double getEstimatedClimbAngle(){
        return Units.degreesToRotations(getAngle())/(extendPosition-climbPosition)*Math.PI/2;
    }
}