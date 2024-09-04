package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
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
import frc.robot.subsystems.PowerPanel;
import frc.robot.util.LogManager;

import java.time.Duration;
import java.util.ArrayList;

/**
 * Subsystem that controls the arm.
 * <p>
 * The indexer and shooter are at the end of the arm.
 */
public class Arm extends SubsystemBase {

    /**
     * The motors are stored in an array.
     * <p>
     * The first entry is the master motor.
     * <p>
     * The first set of entries are the left side motors, and the second set are the right side.
     */
    private final TalonFX[] motors = new TalonFX[ArmConstants.MOTOR_IDS.length];
    // DCMotor model is 4 Kraken X60
    protected static final DCMotor motorModel = DCMotor.getKrakenX60(ArmConstants.MOTOR_IDS.length);

    /**
     * Gearbox ratio
     * <p>
     * The gearing starting from the motor is:
     * <p>
     * 8T : 72T
     * <p>
     * 18T : 84T
     * <p>
     * 16T : 64T
     */
    protected static final double GEARING = (72.0 / 8.0) * (84.0 / 18.0) * (64.0 / 16.0);

    // Encoder in the TalonFX.... currently not used.
    //   check docs. Normal update rate is 4 Hz.
    StatusSignal<Double> rotorPositionSignal;

    // DutyCycle control
    // TODO: change to voltage control
    private final DutyCycleOut m_request = new DutyCycleOut(0);
    public double dutyCycle = 0;

    /**
     * REV absolute encoder.
     * <p>
     *  It is mounted on the right hand side. CCW rotation is arm raises.
     */
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);
    /** this instance sets the REV absolute encoder value during simulations */
    private DutyCycleEncoderSim encoderSim;
    /* New value for OFFSET
     * stow is 0.599
     * high is 0.357
     * */
    /** 
     * REV encoder offset in radians.
     * <p>
     * WARNING: This value will change if the belt driving the REV encoder slips!
     * New value for OFFSET
     * stow is 0.599
     * high is 0.357
     */
    protected static final double OFFSET =  0.1483 + Units.radiansToRotations(ArmConstants.MIN_ANGLE_RADS);
    /** REV encoder scale factor. This is fixed. */
    protected static final double DISTANCE_PER_ROTATION = -2 * Math.PI;

    // Motor PID control
    private static final double TOLERANCE = Units.degreesToRadians(1);
    // P = 5 worked during simulation
    private static final double P = 0.85;
    private static final double I = 0;
    private static final double D = 0;
    private final PIDController pid = new PIDController(P, I, D);

    // Motor feedforward control
    public static final double S = 0;
    public static final double G = 0.02;
    public static final double V = 0;
    private final ArmFeedforward feedforward = new ArmFeedforward(S, G, V);

    /** arm simulator */
    private SingleJointedArmSim simulation;

    // Mechanism display.
    // The display is always built, but it only changes during simulation.
    private Mechanism2d wristDisplay = new Mechanism2d(90, 90);
    private MechanismRoot2d pivot = wristDisplay.getRoot("ArmPivot", 45, 45);
    private MechanismLigament2d moving = pivot.append(
        new MechanismLigament2d(
            "Moving",
            30,
            Units.radiansToDegrees(ArmConstants.START_ANGLE_RADS),
            6,
            new Color8Bit(Color.kYellow)));

	private boolean armEnabled = true;

    //private PowerPanel m_powerPanel = new PowerPanel();

    public Arm() {
        //LogManager.add("fudge facotr", ()->{return ArmConstants.armFudgeFactor;});
        // set the PID tolerance
        pid.setTolerance(TOLERANCE);

        // set the PID the starting position
        pid.setSetpoint(ArmConstants.START_ANGLE_RADS);

		// set the arm offset
		// assumes the arm is at its lowest position on startup
		// if the arm is not at its stowedSetpoint, then this will likely break the arm
		//OFFSET = getAngleRad() - ArmConstants.MIN_ANGLE_RADS;

        // make the encoder report arm angle in radians
        encoder.setDistancePerRotation(DISTANCE_PER_ROTATION);
        encoder.setPositionOffset(OFFSET);

        // consider each of the motors
        for (int i = 0; i < motors.length; i++) {
            // create the motor
            motors[i] = new TalonFX(ArmConstants.MOTOR_IDS[i]);
            // TODO: why is this done here? The slave should follow when neutral mode is set below.
            motors[i].setNeutralMode(NeutralModeValue.Brake);

            // i==0 is the master; the others are slaves
            if (i > 0) {
                // set slave mode
                // the master is MOTOR_IDS[0]
                // invert master for ids 2 and 3
                
                motors[i].setControl(new Follower(ArmConstants.MOTOR_IDS[0], (i >= motors.length / 2)));
            }
        }

        // common configuration for each motor
        // configure the master after the slaves have been linked so slaves will copy the same settings.
        motors[0].setNeutralMode(NeutralModeValue.Brake);
        motors[0].setInverted(false);
        motors[0].getConfigurator().apply(ArmConstants.currentConfig);

        // Phoenix v6 rotor position signal
        rotorPositionSignal = motors[0].getRotorPosition();

        // possibly set up simulations
        if (RobotBase.isSimulation()) {
            simulation = new SingleJointedArmSim(motorModel,
                    GEARING,
                    ArmConstants.MOMENT_OF_INERTIA,
                    ArmConstants.ARM_LENGTH,
                    ArmConstants.MIN_ANGLE_RADS,
                    ArmConstants.MAX_ANGLE_RADS,
                    true,
                    ArmConstants.START_ANGLE_RADS);

            // encodersim allows us to set the encoder values
            encoderSim = new DutyCycleEncoderSim(encoder);
            encoderSim.setDistance(ArmConstants.START_ANGLE_RADS);

            // put the display on the SmartDashboard
            // SmartDashboard.putData("ArmSim", wristDisplay);
        }
        Timer.delay(2);
		double cachedAngleRad = getAngleRad(); // don't get the angle five times
        SmartDashboard.putNumber("cached angle", getAngleRad());
		// some checks for the arm position
		if (cachedAngleRad < ArmConstants.MIN_ANGLE_RADS - ArmConstants.ANGLE_TOLERANCE || cachedAngleRad > ArmConstants.MAX_ANGLE_RADS + ArmConstants.ANGLE_TOLERANCE) {

			System.err.println("WARNING: THE ARM IS IN A SUPPOSEDLY UNREACHABLE POSITION AND HAS BEEN DISABLED. Please double check the arm constants and redeploy. Found: " + cachedAngleRad + ", Expected: " + ArmConstants.stowedSetpoint);
			armEnabled = false;
		}

        // TODO: remove when not needed.
        // Add some test commands
        if (Constants.DO_LOGGING) {
            LogManager.add("Arm/PositionError", () -> getAngleRad() - pid.getSetpoint(), Duration.ofSeconds(1));
            // pid setpoint and get radians

            ArrayList<Double> slave_errors = new ArrayList<Double>();
            for (TalonFX each_talon: motors) { // could use TalonFX as it originally was. tomato tomahto
                slave_errors.add(each_talon.getPosition().getValue()-motors[0].getPosition().getValue());
            }

            // LogManager.add("Arm/SlaveErrors(ticks)", () -> slave_errors);
        }
    
	//SmartDashboard.putBoolean("Arm Enabled", armEnabled);
    }

    /**
     * Get the arm angle using the absolute sensor or the motor encoder.
     * @return
     */
    double getRadians() {
        // using the REV absolute sensor is easy
        return encoder.getDistance();

        // using the motor encoder is more difficult
        // refresh the position (needed because we want it more often than 4 Hz)
        // rotorPositionSignal.refresh();
        // return Units.rotationsToRadians(rotorPositionSignal.getValue() / ArmConstants.GEARING);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("abs value", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("arm fudge factor", ArmConstants.armFudgeFactor);
        SmartDashboard.putNumber("arm fudge changes", ArmConstants.armFudgeFactorChanges);

        if(Math.abs(pid.getSetpoint() - ArmConstants.stowedSetpoint) < 0.05 && pid.atSetpoint()) {
				//motors[0].set(0);
        }
        SmartDashboard.putNumber("get Position", getPosition());
        // SmartDashboard.putData("arm pid", pid);
        // SmartDashboard.putBoolean("if at setpoint", pid.atSetpoint());
        // Disable the arm if it is out of range
		if (getAngleRad() < ArmConstants.MIN_ANGLE_RADS - ArmConstants.ANGLE_TOLERANCE || getAngleRad() > ArmConstants.MAX_ANGLE_RADS + ArmConstants.ANGLE_TOLERANCE) {
			System.err.println("WARNING: THE ARM IS IN A SUPPOSEDLY UNREACHABLE POSITION AND HAS BEEN DISABLED. Found: " + getAngleRad() + ", Expected: " + ArmConstants.stowedSetpoint);
			for (int i = 0; i < motors.length; i++) {
				// irrelevant for next line: motors[i].setNeutralMode(NeutralModeValue.Coast);
                System.err.println("hwat");
			}
            return;
		}

        // Clamp the PID setpoint in case an out of range value slips in ...
        double setpoint = pid.getSetpoint();
        if (setpoint < ArmConstants.MIN_ANGLE_RADS) {
            pid.setSetpoint(ArmConstants.MIN_ANGLE_RADS);
            setpoint = ArmConstants.MIN_ANGLE_RADS;
        }
        if (setpoint > ArmConstants.MAX_ANGLE_RADS) {
            pid.setSetpoint(ArmConstants.MAX_ANGLE_RADS);
            setpoint = ArmConstants.MAX_ANGLE_RADS;
        }


        // calculate the desired duty cycle
        // if(encoder.getDistance() < ArmConstants.MAX_ANGLE_RADS + .2 && encoder.getDistance() > ArmConstants.MIN_ANGLE_RADS - .2)  {
        dutyCycle = MathUtil.clamp(
                        pid.calculate(getAngleRad()) + feedforward.calculate(pid.getSetpoint(), 0),
                        -1,
                        1);
        // SmartDashboard.putNumber("pid output", pid.calculate(getAngleRad()));
        // SmartDashboard.putNumber("duty cycle", dutyCycle);

        // }
        // else
        // {
        //     for(int i=0;i<motors.length;i++) {
        //         motors[i].setNeutralMode(NeutralModeValue.Coast);
        //     }

        // }

        // use the Phoenix 6 version of setting the motor "power"
        motors[0].setControl(m_request.withOutput(dutyCycle));

        // report the arm angle in radians


        // TODO: Clean these up when not needed.
        // report dutycycle
       // SmartDashboard.putNumber("arm pow", dutyCycle);

        // these use the motor's internal encoder (not the REV absolute encoder)
        // report the rotor position. This should trigger an update so we get results faster than 4 times per second.)
        // SmartDashboard.putNumber("Rotor Signal", Units.rotationsToRadians(rotorPositionSignal.getValue()));
        // check the latency
        // SmartDashboard.putNumber("Rotor delay", rotorPositionSignal.getTimestamp().getLatency());
        
        // report the absolute position in rotations. Use the abs rotations to set the OFFSET.
        //SmartDashboard.putNumber("REV ABS", encoder.getAbsolutePosition());
        //SmartDashboard.putNumber("get Position", getPosition());
        // report whether the arm has reached its setpoint
        //SmartDashboard.putBoolean("at setpoint?", atSetpoint());
        // report the arm current
    }

    @Override
    public void simulationPeriodic() {
		if (!armEnabled) return;

        // Assuming the volts
        double voltsBattery = 12.8;

        // set the simulator's input voltage
        simulation.setInputVoltage(motors[0].get() * voltsBattery);

        // run the simulator for 20 milliseconds
        simulation.update(0.02);

        // set the arm angle in the display
        moving.setAngle(Units.radiansToDegrees(simulation.getAngleRads()));

        // update the DutyCycleEncoder
        encoderSim.setDistance(simulation.getAngleRads());

        // Calculate the current drawn by one of the motors
        double ampsPerMotor = simulation.getCurrentDrawAmps() / 4;

        // see https://docs.google.com/spreadsheets/d/1UiHZFYeZiHPAPIu39uRrskQuQYfvJ03UjLeQVq--Mzg/edit#gid=0
        // Arm motors uses channels 1, 2, 4, 5
        // m_powerPanel.setCurrent(1, ampsPerMotor);
        // m_powerPanel.setCurrent(2, ampsPerMotor);
        // m_powerPanel.setCurrent(4, ampsPerMotor);
        // m_powerPanel.setCurrent(5, ampsPerMotor);
    }

    /**
     * Sets the angle of the arm in radians.
     * @param angle (in radians)
     */
    public void setAngle(double angle) {
		if (!armEnabled) {
			System.out.println("WARNING: Set failed because the arm is disabled.");
			return;
		}

        // zero the integrator portion of the PID controller
        pid.reset();

        // clamp the angle to avoid problems
        angle = MathUtil.clamp(angle, ArmConstants.MIN_ANGLE_RADS, ArmConstants.MAX_ANGLE_RADS);

        // change the setpoint
        pid.setSetpoint(angle);
    }

    /**
     * Returns the angle of the wrist in radians.
     * @returns arm angle in radians
     */
    public double getAngleRad() {
        if(RobotBase.isSimulation()){
            return encoder.getDistance();
        }else{
            return getPosition();
        }
    }

    /**
     * Returns the angle of the arm in radians.
     * <p>
     * Stop chasing ghosts. This method just re-implements encoder.getDistance().
     * @return arm angle in radians with zero being horizontal.
     * @Deprecated use getAngleRad()
     */
    @Deprecated
    public double getPosition()  {
        double angle = -Units.rotationsToRadians(encoder.getAbsolutePosition() - encoder.getPositionOffset());
        return MathUtil.angleModulus(angle);
    }

    /**
     * Whether the arm has reached its setpoint
     * @return true when the setpoint is reached.
     */
    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    /**
     * Reclaim resources.
     * Needed for simulation and unit tests.
     */
    public void close() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].close();
        }

        encoder.close();
    }
}
