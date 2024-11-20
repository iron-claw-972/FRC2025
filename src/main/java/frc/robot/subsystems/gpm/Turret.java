package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
    private final DigitalInput hall = new DigitalInput(0);
    /** Simulation resource for the Hall-effect sensor */
    private DIOSim hallSim;
    /** whether the Hall effect sensor has been seen */
    private boolean hallTriggered = false;

    private boolean calibrated = false;

    /** PID controller for the turret. */
    private final PIDController pid = new PIDController (0.8, 0.4, 0.0);

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
    private final double versaPlanetaryGearRatio = 5;
    /** 
     * Gear ratio for the turret. 
     * The VersaPlanetary drives a (10-tooth 10DP) pinion gear that engages the 140 teeth on the turret.
     */
    private final double turretGearRatio = 140.0 / 10.0;  
    /** combination of the VersaPlanetary and turret gear ratios. */
    private final double totalGearRatio = versaPlanetaryGearRatio * turretGearRatio; 

    /** Physics Turret Sim (only used during simulations) */
    private SingleJointedArmSim turretSim;

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState; // Target state (desired angle)
    private TrapezoidProfile.State currentState; // Current state (current angle and velocity)
    private double maxVelocity;
    private double maxAcceleration;

    private Timer timer = new Timer();
    private boolean useTimeConstraint = false; 

    /**
     * Constructor for the Turret class.
     * The GreyT turret (https://docs.wcproducts.com/greyt-turret-11in-id) has a
     * Talon motor driving a VersaPlanetary gearbox.
     * The VersaPlanetary gearbox turns the (10-tooth 10DP) pinion
     * that meshes with the (140-tooth) turret gear.
     */
    public Turret() {

        currentState = new TrapezoidProfile.State(0, 0); // Start at angle 0 with zero velocity

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
            // encoderSim.Orientation = ChassisReference.Clockwise_Positive;
        }
        // Set I zone for PID
        pid.setIZone(Units.degreesToRadians(5)); 
        
        // put PID on Smart Dashboard 
        SmartDashboard.putData("PID", pid); 

        // put the Mechanism2D display on the SmartDashboard
        SmartDashboard.putData("turret display", simulationMechanism);

        // Enable continuous pid input because there are no hard stops 
        pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        // Hall-effect sensor sees a magnet
        if (!hall.get()) {
            if(!hallTriggered) {
                if(motor.getVelocity().getValueAsDouble() > 0) {
                    // Reset encoder position 
                    motor.setPosition(0);
                }
                else{   
                    // Reset encoder position with offset to account for sensor's wide detection gap 
                    motor.setPosition(0.14);
                }
            }
            hallTriggered = true;
        }
        else {
            hallTriggered = false; 
        }
        // Calibrate turret on start up 
        if (!calibrated) {
            calibrate();
        }
        else {
        // get the motor position in rotations
        double motorPosition = motor.getPosition().getValueAsDouble();

        // convert the motor position to a turret position in radians
        double currentPosition = Units.rotationsToRadians(motorPosition/totalGearRatio);
        
        // Only calculate and set power to PID when time constraint is not being used 
        if (!useTimeConstraint || !calibrated) {
            // calculate motor power to turn turret
            double power = pid.calculate(currentPosition);

            motor.set(MathUtil.clamp(power, -1, 1));
        }

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
        
        //Position in degrees
        SmartDashboard.putNumber("Turret Position", getAngle()); 

        //Encoder Position
        SmartDashboard.putNumber("Encoder Position", motor.getPosition().getValueAsDouble());

        //Motor velocity
        SmartDashboard.putNumber("Motor Velocity", motor.getVelocity().getValueAsDouble());

        //Change between using time constrait and pid 
        SmartDashboard.putBoolean("Use Time Constraint", useTimeConstraint); 
        // TODO: FIX TIMER (CURRENTLY CAUSES ERROR; ROBOT CODE GETS MILK)
        // if (atSetpoint()) {
        //     timer.stop(); 
        //     SmartDashboard.putNumber("Time to reach setpoint", timer.get());
        // }
    }

        // Only runs once angle has been set with setAngleWithTime AND using time constraint 
        if (goalState != null && useTimeConstraint) {
            // Initialize profile with constraints 
            TrapezoidProfile profile = new TrapezoidProfile(constraints);
            
            // Determines setpoint and sets PID to that position 
            TrapezoidProfile.State setpoint = profile.calculate(Constants.LOOP_TIME, currentState, goalState);
            pid.setSetpoint(setpoint.position);
    
            double motorPositionNow = getAngle();
            double currentPositionNow = Units.degreesToRadians(motorPositionNow);
            
            // Calculates power based off current position and sets it to motors 
            double power = pid.calculate(currentPositionNow);
            motor.set(MathUtil.clamp(power, -1, 1));
            
            // Current state of trapezoid profile 
            currentState = new TrapezoidProfile.State(setpoint.position, setpoint.velocity);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Find input voltage to the motor
        turretSim.setInput(motor.get() * Constants.ROBOT_VOLTAGE); 

        // Up dates every 0.020 seconds 
        turretSim.update(Constants.LOOP_TIME);

        // Set the encoder
        // Get the turret position in rotations
        double turretRotations = Units.radiansToRotations(turretSim.getAngleRads());
        // Set the motor rotation by considering the gear ratio
        encoderSim.setRawRotorPosition(turretRotations * totalGearRatio);

        // Simulate the Hall-effect sensor
        // Sensor goes false near 1/8 of a rotation
        hallSim.setValue(Math.abs(turretRotations - 0.125) > 0.01);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(turretSim.getCurrentDrawAmps())
        );

    }
    
    /**
     * Set the turret angle.
     * @param angle (degrees)
     */
    public void setAngle(double angle) {
        pid.enableContinuousInput(-Math.PI, Math.PI);
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(angle));
    }

    /**
     * Returns the current angle of turret in degrees
     * @return
     */
    public double getAngle() {
        return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble() / totalGearRatio);
    }

    /**
     * Returns if turret is at setpoint to tolerance of 1.5 degrees
     * @return
     */
    public boolean atSetpoint() {
        if (useTimeConstraint) {
            // Use setpoint based off Trapezoidal Profile when using time constraint 
            if(Math.abs(getAngle() - Units.radiansToDegrees(goalState.position)) < 1.5) {
                return true; 
            }
        }
        else {
            // Use setpoint based of PID when only using PID 
            if(Math.abs(getAngle() - Units.radiansToDegrees(pid.getSetpoint())) < 1.5) {
                return true; 
            }
        }
        return false; 
    }


    /**
     * Spins turret to zero position and resets encoder on start up 
     */
    public void calibrate() {
        if (!hallTriggered) {
            motor.set(0.1); 
        }
        else {
            motor.stopMotor(); 
            calibrated = true; 
        }
    }

    /**
     * Sets the angle of turret in degrees with a time constraint in seconds  
     *  
     */
    public void setAngleWithTime(double targetAngle, double timeToReach) {
        // Calculate target setpoint, current angle, and distance 
        double targetRadians = Units.degreesToRadians(targetAngle);
        double currentRadians = currentState.position;
        double distance = Math.abs(targetRadians - currentRadians);

        // Start timer  
        timer.reset();
        timer.start();
        
        // Set max velocity to a high value so that it never runs at constant velocity 
        // maxVelocity = 500; 
        // Acceleratin derived from equation distance = (1/2)(a)(t)^2
        // maxAcceleration = 4 * distance / Math.pow(timeToReach, 2);
        

        // Calculations for more linear motion and constant acceleration always 
        maxAcceleration = 20; // TODO: find good max acceleration near motor's rpm 
        // Solved for v in the equation from x = -v^2 / a + vt
        maxVelocity = ((maxAcceleration * timeToReach) + Math.sqrt((Math.pow(maxAcceleration, 2))*(Math.pow(timeToReach, 2)) - (4 * maxAcceleration * distance))) / 2;

        // Create constraints based of calculated acceleration 
        constraints = new TrapezoidProfile.Constraints(
            maxVelocity, 
            maxAcceleration
        );
        
        // Set goal state to target angle 
        goalState = new TrapezoidProfile.State(targetRadians, 0);

    }

    /**
     * Run method to switch between Trapezoidal Motion Profile and only PID 
     */
    public void switchTimeConstraint() {
        useTimeConstraint = !useTimeConstraint; 
    }
}   