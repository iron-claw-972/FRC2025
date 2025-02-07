package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ClimbArmSim;

public class Climb extends SubsystemBase {
    //Motors
    private final PIDController pid = new PIDController(0.1, 0.0, 0.0);

    private TalonFX motor = new TalonFX(20);
    private final DCMotor climbGearBox = DCMotor.getFalcon500(1);
    private TalonFXSimState encoderSim;

    //Mechism2d display
    private final Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
    private final MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Climb", 1.5, 1.5);
    private final MechanismLigament2d simLigament = mechanismRoot.append(
        new MechanismLigament2d("angle", 1, 0, 4, new Color8Bit(Color.kAntiqueWhite))
    );

    private final double versaPlanetaryGearRatio = 5;
    private final double climbGearRatio = 75.0/1.0;
    private final double totalGearRatio = versaPlanetaryGearRatio * climbGearRatio;

    private ClimbArmSim climbSim;

    public Climb() {
        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();

            climbSim = new ClimbArmSim(
                climbGearBox, 
                totalGearRatio, 
                1.014, 
                0.127, 
                Double.NEGATIVE_INFINITY, //min angle 
                Double.POSITIVE_INFINITY, //max angle
                false, 
                0.0,
                60
                );
        }

        pid.setIZone(Units.degreesToRadians(5));

        SmartDashboard.putData("PID", pid);
        SmartDashboard.putData("Climb Display", simulationMechanism);       

        motor.setPosition(0);
    }

    @Override
    public void periodic() {
        double motorPosition = motor.getPosition().getValueAsDouble();
        double currentPosition = Units.rotationsToRadians(motorPosition/totalGearRatio);

        double power = pid.calculate(currentPosition);
        motor.set(MathUtil.clamp(power, -1, 1));

        simLigament.setAngle(Units.radiansToDegrees(currentPosition));

        SmartDashboard.putNumber("Climb VIN Voltage", RoboRioSim.getVInVoltage());
        SmartDashboard.putNumber("Climb Position", getAngle());

        SmartDashboard.putNumber("Encoder Position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Velocity", motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        climbSim.setInput(motor.get() * Constants.ROBOT_VOLTAGE);
        climbSim.update(Constants.LOOP_TIME);

        double climbRotations = Units.radiansToDegrees(climbSim.getAngleRads());
        encoderSim.setRawRotorPosition(climbRotations * totalGearRatio);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(climbSim.getCurrentDrawAmps())
        );
    }

    public void setAngle(double angle) {
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(angle));
        System.out.println("hamburger");
    }

    public double getAngle() {
        return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble() / totalGearRatio);
    }

    public void extend(){
        double extendAngle = 90;
        setAngle(extendAngle);
    }
    public void climb(){
        double climbAngle = 0;
        setAngle(climbAngle);
    }
}
