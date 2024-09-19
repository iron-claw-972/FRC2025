package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.LogManager;

public class Turret extends SubsystemBase {
    
    public DigitalInput hall; 
    public Boolean hallTriggered;
    private DutyCycleEncoder encoder;
    private PIDController pid = new PIDController (.2, 0, 0);
    private CANSparkMax sparkMotor;
    private DCMotor turretGearBox = DCMotor.getNEO(1);
    private DutyCycleEncoderSim encoderSim;
    MechanismLigament2d simLigament;
    private Mechanism2d simulationMechanism;
    MechanismRoot2d mechanismRoot;  

    //Physics Turret Sim
    private final SingleJointedArmSim turretSim =
            new SingleJointedArmSim(
                turretGearBox,
                12.0,
                .01403,
                .127,
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(360.0),
                false,
                0.0
          );
       
    public Turret() {
        hall = new DigitalInput(1); //TODO: Change port later
        //Display
        simulationMechanism = new Mechanism2d(3, 3);
        mechanismRoot = simulationMechanism.getRoot("Turret", 1.5, 1.5);
        simLigament = mechanismRoot.append(
            new MechanismLigament2d("angle", 1, 0, 4, new Color8Bit(Color.kYellow)));

        encoder = new DutyCycleEncoder(8); // TODO: Change to actual id
        sparkMotor = new CANSparkMax(18, MotorType.kBrushless); // TODO: Change to actual id 
        encoderSim = new DutyCycleEncoderSim(encoder); 
        SmartDashboard.putData("PID", pid); 
        SmartDashboard.putData("Turret Sim", simulationMechanism);
    } 

    @Override
    public void periodic() {
        hallTriggered = true;
        double currentPosition = encoder.getDistance(); 
        double power = pid.calculate(currentPosition); 
        sparkMotor.set(MathUtil.clamp(power, -.25, .25)); 
        //Put Data to SmartDashboard
        SmartDashboard.putNumber("Turet VIN Voltage", RoboRioSim.getVInVoltage());
        SmartDashboard.putNumber("Turret Current Draw", turretSim.getCurrentDrawAmps());
        SmartDashboard.putBoolean("Hall is triggered", hallTriggered);
        //Log Data 
        LogManager.add("Turret Current", () -> turretSim.getCurrentDrawAmps());
        LogManager.add("Voltage With Turret", () -> RoboRioSim.getVInVoltage());
    }

    @Override
    public void simulationPeriodic() {
        turretSim.setInput(sparkMotor.get() * Constants.ROBOT_VOLTAGE); 
        turretSim.update(0.020); 
        encoderSim.setDistance(turretSim.getAngleRads());
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(turretSim.getCurrentDrawAmps()));
        simLigament.setAngle(Units.radiansToDegrees(turretSim.getAngleRads()));
    }
    
    /*
     * Sets angle of turret
     */
    public void setAngle(double angle) {
        double numRotations = angle / 360;
        pid.reset();
        pid.setSetpoint(Units.rotationsToRadians(numRotations));
    }
}