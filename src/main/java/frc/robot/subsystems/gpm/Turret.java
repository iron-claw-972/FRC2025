package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
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

/**
 * Class for the GreyT Turret (previously the Iron Claw turret).
 * 
 */
public class Turret extends SubsystemBase {
    // TODO: change Hall-effect port later
    /** A Hall-effect sensor determines a known position of the turret. */
    private final DigitalInput hall = new DigitalInput(1);
    /** Simulation resource for the Hall-effect sensor */
    private DIOSim hallSim;
    /** whether the Hall effect sensor has been seen */
    private boolean hallTriggered = false;

    /** PID controller for the turret. */
    private final PIDController pid = new PIDController (0.1, 0.0, 0.0);

    // TODO: change to actual motor id
    // Motor IDs should be specified in one place. Right now, I must check several files to see which motors are in use.
    // ... or run the simulator to see which ones are created.
    /** Motor that controls the turret */
    private final TalonFX motor = new TalonFX(20);
    private final DCMotor turretGearBox = DCMotor.getFalcon500(1);
    /** object to set the motor's encoder during simulation */
    private TalonFXSimState encoderSim;

    /** Mechanism2d display (always displayed) */
    private final Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
    private final MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Turret", 1.5, 1.5);
    /** pointer that shows the turret direction */
    private final MechanismLigament2d simLigament = mechanismRoot.append(
            new MechanismLigament2d("angle", 1, 0, 4, new Color8Bit(Color.kYellow)));

    /** Gear ratio for the planetary gearbox. The motor is attached to a VersaPlanetary gearbox. */
    private final double versaPlanetaryGearRatio = 5.0;
    /** 
     * Gear ratio for the turret. 
     * The VersaPlanetary drives a (10-tooth 10DP) pinion gear that engages the 140 teeth on the turret.
     */
    private final double turretGearRatio = 140.0 / 10.0;  
    /** combination of the VersaPlanetary and turret gear ratios. */
    private final double totalGearRatio = versaPlanetaryGearRatio * turretGearRatio; 

    /** Physics Turret Sim (only used during simulations) */
    private SingleJointedArmSim turretSim;
     
    /**
     * Constructor for the Turret class.
     * The GreyT turret (https://docs.wcproducts.com/greyt-turret-11in-id) has a
     * Talon motor driving a VersaPlanetary gearbox.
     * The VersaPlanetary gearbox turns the (10-tooth 10DP) pinion
     * that meshes with the (140-tooth) turret gear.
     */
    public Turret() {

        if (RobotBase.isSimulation()) {
            // get the simulation object from the motor
            encoderSim = motor.getSimState(); 

            // set up the physics simulation
            turretSim = new SingleJointedArmSim(
                turretGearBox,
                totalGearRatio,
                1.01403,
                .127,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                false,
                0.0);

            // get the simulator for the Hall-effect
            hallSim = new DIOSim(hall);
            // make sure the Hall-effect sensor state is set
            hallSim.setValue(true);
        }

        SmartDashboard.putData("PID", pid); 
        // put the Mechanism2D display on the SmartDashboard
        SmartDashboard.putData("turret display", simulationMechanism);

        // Enable continuous pid input because there are no hard stops 
        pid.enableContinuousInput(-Math.PI, Math.PI);

        //Invert direction of simulation encoder to get correct position value in simultion
        encoderSim.Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void periodic() {
        if (!hallTriggered) {
            // the Hall-effect sensor has not been triggered yet.
            // check to see if it is active
            if (!hall.get()) {
                // Hall-effect sensor sees a magnet
                hallTriggered = true;
                // TODO: the turrent angle is now known, so set the encoder offset...
                motor.setPosition(0); 
                
            }
        }

        // get the motor position in rotations
        double motorPosition = motor.getPosition().getValueAsDouble();

        // convert the motor position to a turret position in radians
        double currentPosition = Units.rotationsToRadians(motorPosition/totalGearRatio);

        // calculate motor power to turn turret
        double power = pid.calculate(currentPosition);

        motor.set(MathUtil.clamp(power, -1, 1));

        // update the Mechanism2d display based on measured position
        simLigament.setAngle(Units.radiansToDegrees(currentPosition));
        
        // Put Data to SmartDashboard
        SmartDashboard.putNumber("Turret VIN Voltage", RoboRioSim.getVInVoltage());
        // we may not be simulating...
        // SmartDashboard.putNumber("Turret Current Draw", turretSim.getCurrentDrawAmps());
        SmartDashboard.putBoolean("Hall is triggered", hallTriggered);
        // SmartDashboard.putBoolean("LED state is on", ledState);
        //Log Data, TODO: move to constructor if using suppliers
        // LogManager.add("Turret Current", () -> turretSim.getCurrentDrawAmps());
        // LogManager.add("Voltage With Turret", () -> RoboRioSim.getVInVoltage());
    }

    @Override
    public void simulationPeriodic() {
        // find input voltage to the motor
        turretSim.setInput(motor.get() * Constants.ROBOT_VOLTAGE); 

        turretSim.update(Constants.LOOP_TIME);

        // set the encoder
        // get the turret position in rotations
        double turretRotations = Units.radiansToRotations(turretSim.getAngleRads());
        // set the motor rotation by considering the gear ratio
        encoderSim.setRawRotorPosition(turretRotations * totalGearRatio);

        // simulate the Hall-effect sensor
        // sensor goes false near 1/8 of a rotation
        hallSim.setValue(Math.abs(turretRotations - 0.125) > 0.01);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(turretSim.getCurrentDrawAmps()));
    }
    
    /**
     * Set the turret angle.
     * @param angle (degrees)
     */
    public void setAngle(double angle) {
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(angle));
    }
}   