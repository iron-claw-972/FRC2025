package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
import frc.robot.util.LogManager;


public class Turret extends SubsystemBase {
    
    public DigitalInput hall = new DigitalInput(1); //TODO: Change port later; 
    private boolean hallTriggered = false;
    private PIDController pid = new PIDController (.001, 0, 0);
    private TalonFX motor = new TalonFX(1); // TODO: Change to actual id ;
    private DCMotor turretGearBox = DCMotor.getFalcon500(1);
    // private DutyCycleEncoderSim encoderSim;
    // private DutyCycleEncoder encoder; 
    TalonFXSimState encoderSim; 
    MechanismLigament2d simLigament;
    private Mechanism2d simulationMechanism;
    MechanismRoot2d mechanismRoot;  
    private final double versaPlanetaryGearRatio = 5; 
    private final double turretGearRatio = 12;  
    private final double totalGearRatio = versaPlanetaryGearRatio * turretGearRatio; 

    //Physics Turret Sim
    private SingleJointedArmSim turretSim;
       
    public Turret() {
        motor.setInverted(true);
         //Display
        simulationMechanism = new Mechanism2d(3, 3);
        mechanismRoot = simulationMechanism.getRoot("Turret", 1.5, 1.5);
        simLigament = mechanismRoot.append(
            new MechanismLigament2d("angle", 1, 0, 4, new Color8Bit(Color.kYellow)));

        if (RobotBase.isSimulation()) {

            // resources needed for simuulation
            // encoder = new DutyCycleEncoder(18);
            // encoderSim = new DutyCycleEncoderSim(encoder); 
            encoderSim = motor.getSimState(); 

            turretSim =  new SingleJointedArmSim(
                turretGearBox,
                totalGearRatio,
                1.01403,
                .127,
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(360.0),
                false,
                0.0
          );
        }

        SmartDashboard.putData("PID", pid); 

    }

    @Override
    public void periodic() {
        hallTriggered = true;

        // motor position in rotations
        double motorPosition = motor.getRotorPosition().getValueAsDouble();

        // turret position in radians
        double currentPosition = Units.rotationsToRadians(motorPosition/totalGearRatio);

        // calculate motor power to turn turret
        double power = pid.calculate(currentPosition);

        motor.set(MathUtil.clamp(power, -.25, .25));

        //Put Data to SmartDashboard
        SmartDashboard.putNumber("Turret VIN Voltage", RoboRioSim.getVInVoltage());
        // we may not be simulating...
        // SmartDashboard.putNumber("Turret Current Draw", turretSim.getCurrentDrawAmps());
        SmartDashboard.putBoolean("Hall is triggered", hallTriggered);
        // SmartDashboard.putBoolean("LED state is on", ledState);
        //Log Data 
        // LogManager.add("Turret Current", () -> turretSim.getCurrentDrawAmps());
        // LogManager.add("Voltage With Turret", () -> RoboRioSim.getVInVoltage());
    }

    @Override
    public void simulationPeriodic() {
        // find input voltage to the motor
        turretSim.setInput(motor.get() * Constants.ROBOT_VOLTAGE); 

        turretSim.update(0.020);

        // set the encoder
        double turretRotations = Units.radiansToRotations(turretSim.getAngleRads());
        encoderSim.setRawRotorPosition(turretRotations * totalGearRatio);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(turretSim.getCurrentDrawAmps()));

        simLigament.setAngle(Units.radiansToDegrees(turretSim.getAngleRads()));
    }
    
    /**
     * Set the turret angle
     * @param angle (degrees)
     */
    public void setAngle(double angle) {
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(angle));
    }
}   