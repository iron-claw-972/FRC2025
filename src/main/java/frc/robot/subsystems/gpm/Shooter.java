package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LogManager;

import java.time.Duration;

public class Shooter extends SubsystemBase {
	// each of the shooter shafts is driven by one Neo Vortex motor
	protected static final DCMotor gearbox = DCMotor.getNeoVortex(1);
	/**
	 * Shooter gearing.
	 * <p>
	 * Each turn of the motor is two turns of the shooter wheel
	*/
	protected static final double gearRatio = 0.5;
	// get the motor freespeed
	private static final double rpmFreeSpeed = Units
			.radiansPerSecondToRotationsPerMinute(Shooter.gearbox.freeSpeedRadPerSec);

	// PID constants. PID system measures RPM and outputs motor power [-1,1]
	private static final double P = 0.0015 ;
	private static final double I = 0.00002;
	private static final double leftI = I;
	private static final double D = 0.00001;

	// FeedForward constants
	private static final double S = 0;
	private static final double V = 1.25 / rpmFreeSpeed;

	/**
	 * Tolerance in RPM.
	 * At 1500 rpm, the simulator gives 1519 rpm.
	 */
	private static final double TOLERANCE = 50;

	// 4-inch Colson wheels
	// private static final double MASS_COLSON = 0.245;
	// private static final double RADIUS_COLSON = Units.inchesToMeters(2.0);
	// private static final double MOI_COLSON = 0.5 * MASS_COLSON * RADIUS_COLSON * RADIUS_COLSON;

	// 4-inch Stealth
	// mass is 0.097 kg. About half of that is in the rim.
	// steel insert is 0.365 kg with radius of 3.25 inches and a 0.875 hole.
	private static final double MASS_RIM = 0.5 * 0.097;
	private static final double RADIUS_STEALTH = Units.inchesToMeters(2.0);
	private static final double MOI_STEALTH = MASS_RIM * RADIUS_STEALTH * RADIUS_STEALTH;

	/** Magic number to predict output velocity. <p> Found empirically. */
	private static final double OUTPUT_COEF = 0.64;

	// each motor spins 6 stealth wheels
	protected static final double MOI_SHAFT = MOI_STEALTH * 6;

	// left motor
	private final CANSparkFlex leftMotor = new CANSparkFlex(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
	/** PID controller uses RPM as input and outputs motor power */
	protected final PIDController leftPID = new PIDController(P, leftI, D);
	private FlywheelSim leftFlywheelSim;
	private double leftMotorSpeedSim = 0.0;
	private double leftPower = 0.0;

	// right motor
	private final CANSparkFlex rightMotor = new CANSparkFlex(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();
	/** PID controller uses RPM as input and outputs motor power */
	protected final PIDController rightPID = new PIDController(P, I, D);
	private FlywheelSim rightFlywheelSim;
	private double rightMotorSpeedSim = 0.0;
	private double rightPower = 0.0;
	private static double slipCoefficient = 0.91;
	private int spinRemainder = 0;

	// TODO: TUNE THIS
	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(S, V);

	public Shooter() {
		// set the RPM tolerance of the PID controllers
		leftPID.setTolerance(TOLERANCE);
		rightPID.setTolerance(TOLERANCE);
		// invert the right motor so +power sends the note out
		rightMotor.setInverted(false);
		leftMotor.setInverted(true);

		// are we simulating?
		if (RobotBase.isSimulation()) {
			// make the linear system simulation instances
			leftFlywheelSim = new FlywheelSim(gearbox, gearRatio, MOI_SHAFT);
			rightFlywheelSim = new FlywheelSim(gearbox, gearRatio, MOI_SHAFT);
		}
		
		if (Constants.DO_LOGGING) {
			LogManager.add("Shooter/MotorSpeedDifference", () -> getMotorSpeedDifference(), Duration.ofSeconds(1));
			LogManager.add("Shooter/LeftSpeedError", () -> leftPID.getSetpoint() - getLeftMotorSpeed(), Duration.ofSeconds(1));
			LogManager.add("Shooter/RightSpeedError", () -> rightPID.getSetpoint() - getRightMotorSpeed(), Duration.ofSeconds(1));

			LogManager.add("Shooter/VoltsLeft", () -> leftMotor.get() * Constants.ROBOT_VOLTAGE, Duration.ofSeconds(1));	
			
			LogManager.add("Shooter/VoltsRight", () -> rightMotor.get() * Constants.ROBOT_VOLTAGE, Duration.ofSeconds(1));
		
			LogManager.add("Shooter/Leftspd", () -> leftPID.getSetpoint() - getLeftMotorSpeed());
			LogManager.add("Shooter/Rightspd", () -> getRightMotorSpeed());
		}
	}

	@Override
	public void periodic() {

		//SmartDashboard.putBoolean("shooter setpoint", atSetpoint());

		// PID loop uses RPM
		double leftSpeed = getLeftMotorRPM();
		double rightSpeed = getRightMotorRPM();

		// A set and get of the NEO motors do not match, so keep powers around for simulation
		leftPower = leftPID.calculate(leftSpeed) + feedforward.calculate(leftPID.getSetpoint());
		rightPower = rightPID.calculate(rightSpeed) + feedforward.calculate(rightPID.getSetpoint());

		// set motor powers
		leftMotor.set(MathUtil.clamp(leftPower,-1,1));
		rightMotor.set(MathUtil.clamp(rightPower,-1,1));

		// report some values to the Dashboard
		// SmartDashboard.putNumber("left speed", /* shooterRPMToSpeed */ (leftSpeed));
		// SmartDashboard.putNumber("right speed", /* shooterRPMToSpeed */ (rightSpeed));
		// SmartDashboard.putBoolean("right setpoint", atSetpoint());
		// //SmartDashboard.putData("slip coefficient", slipCoefficient); /// FIXXX
		// SmartDashboard.putData("left Shooter PID", leftPID);
		// SmartDashboard.putData("right Shooter PID", rightPID);
	}

	@Override
	public void simulationPeriodic() {
		// assume the battery voltage is 12 volts
		double voltage = 12.0;

		// leftPower and rightPower are now member variables; cannot read them from
		// simulated encoder

		// set the system inputs
		leftFlywheelSim.setInputVoltage(leftPower * voltage);
		rightFlywheelSim.setInputVoltage(rightPower * voltage);

		// simulate the linear system (sets current motor angular velocity)
		leftFlywheelSim.update(0.020);
		rightFlywheelSim.update(0.020);

		// get the results
		leftMotorSpeedSim = leftFlywheelSim.getAngularVelocityRPM();
		rightMotorSpeedSim = rightFlywheelSim.getAngularVelocityRPM();

		// we would like to set the encoder velocities to those values, but REV does not
		// let us do that
	}

	/**
	 * Close the resources used by this instance.
	 */
	public void close() {
		leftMotor.close();
		rightMotor.close();
	}

	/**
	 * Converts an RPM to a speed at the surface of the shooter wheels.
	 * <p>
	 * Does not compensate for slipping.
	 *
	 * @param rpm the RPM to convert
	 * @return the converted value in m/s
	 * @see shooterSpeedToRPM
	 */
	public static double shooterRPMToSpeed(double rpm) {
		// factor in Gear Ratio
		return (1.0 / gearRatio) * (rpm / 60) * (RADIUS_STEALTH * 2 * Math.PI);
	}

	/**
	 * Converts a speed at the surface of the shooter wheels to an RPM value.
	 * <p>
	 * Does not compensate for slipping.
	 *
	 * @param speed the speed to convert in m/s
	 * @return the converted value in RPM
	 * @see shooterRPMToSpeed
	 */
	public static double shooterSpeedToRPM(double speed) {
		return gearRatio * (speed * 60) / (RADIUS_STEALTH * 2 * Math.PI);
	}

	/**
	 * Accounts for slip in a speed or RPM.
	 * <p>
	 * This method transforms a speed/RPM that does account for slip into one 
	 * that doesn't. As it's a simple proportional aproximation, it can be 
	 * applied to both speeds and RPMs.
	 * <p>
	 * This method should typically be used for gets. Example: 
	 * System.out.println(removeSlip(getLeftMotorRPM()));
	 * 
	 * @param speed The speed to add slip to.
	 * @return The output, accounting for slip.
	 * @see frc.robot.subsystems.gpm.Shooter.addSlip
	 */
	public static double removeSlip(double speed) {
		return speed * OUTPUT_COEF;
	}

	/**
	 * Accounts for slip in a speed or RPM.
	 * <p>
	 * This method transforms a speed/RPM that doesn't account for slip into 
	 * one that does. As it's a simple proportional aproximation, it can be 
	 * applied to both speeds and RPMs.
	 * <p>
	 * This method should typically be used for sets. Example: 
	 * setTargetRPM(addSlip(outputSpeed));
	 * 
	 * @param speed The speed to add slip to.
	 * @return The output, accounting for slip.
	 * @see frc.robot.subsystems.gpm.Shooter.removeSlip
	 */
	public static double addSlip(double output) {
		return output / OUTPUT_COEF;//*slipCoefficient;//*0.93;
	}

	/**
	 * Sets the speed both shooter motors try to spin up to independantly.
	 *
	 * @param speedLeft  the speed tho left motor will spin to in RPM
	 * @param speedRight the speed the right motor will spin to in RPM
	 */
	public void setTargetRPM(double speedLeft, double speedRight) {
		leftPID.setSetpoint(speedLeft);

		rightPID.setSetpoint(speedRight);
	}

	/**
	 * Sets the RPM both shooter motors try to spin up to.
	 * <p>
	 * Adds a constant spin of plus/minus 300 RPM if that makes sense.
	 *
	 * @param speed speed to spin to in RPM
	 * @see setTargetVelocity
	 */
	public void setTargetRPM(double speed) {
		double spin = speed < 300 ? 0 : ShooterConstants.SPIN;
		setTargetRPM(speed+spin, speed-spin);
	}

	/**
	 * Set the target velocity of a note exiting the shooter.
	 * <p>
	 * Empirical fit.
	 *
	 * @param output speed to shoot note in m/s
	 * @see setTargetRPM
	 */
	public void setTargetVelocity(double output) {
		// convert speed to RPM
		setTargetRPM(addSlip(shooterSpeedToRPM(output)));
	}

	/**
	 * Checks whether both motor PIDs are at their setpoints.
	 *
	 * @return boolean indicating whether both PIDs are at their setpoints
	 */
	public boolean atSetpoint() {
		return EqualsUtil.epsilonEquals(getLeftMotorRPM(),leftPID.getSetpoint(), TOLERANCE)&&
		EqualsUtil.epsilonEquals(getRightMotorRPM(),rightPID.getSetpoint(), TOLERANCE);
		//return leftPID.atSetpoint() && rightPID.atSetpoint();
	}

	/**
	 * Gets the RPM of the left motor, checking whether this is a simulation.
	 * <p>
	 * If a simulation, uses the simulated speed.
	 *
	 * @return the left motor's RPM
	 * @see getRightMotorRPM
	 * @see getLeftMotorSpeed
	 * @see atSetpoint
	 */
	public double getLeftMotorRPM() {
		// REV does not let us set the encoder velocity in a simulation, so use
		// simulated value
		if (RobotBase.isSimulation()) {
			// use the RPM calculated in the simulation
			return leftMotorSpeedSim;
		} else {
			// ask the encoder for the RPM
			return leftMotorEncoder.getVelocity();
		}
	}

	/**
	 * Gets the RPM of the right motor, checking whether this is a simulation.
	 * <p>
	 * If a simulation, uses the simulated RPM.
	 *
	 * @return the right motor's RPM
	 * @see getLeftMotorRPM
	 * @see getRightMotorSpeed
	 * @see atSetpoint
	 */
	public double getRightMotorRPM() {
		// REV does not let us set the encoder velocity in a simulation, so use
		// simulated value
		if (RobotBase.isSimulation()) {
			return rightMotorSpeedSim;
		} else {
			return rightMotorEncoder.getVelocity();
		}
	}

	/**
	 * Gets the speed at the surface of the left wheels.
	 * <p>
	 * If a simulation, returns the simulated speed.
	 *
	 * @return the left wheel's speed in m/s
	 * @see getLeftMotorSpeed
	 * @see getRightMotorRPM
	 * @see atSetpoint
	 */
	public double getLeftMotorSpeed() {
		return shooterRPMToSpeed(getLeftMotorRPM());
	}

	/**
	 * Gets the speed at the surface of the right wheels.
	 *
	 * @return the right wheel's speed in m/s
	 * @see getLeftMotorSpeed
	 * @see getRightMotorRPM
	 * @see atSetpoint
	 */
	public double getRightMotorSpeed() {
		return shooterRPMToSpeed(getRightMotorRPM());
	}

	/**
	 * Gets the difference in RPM between the left motor and right motor.
	 * 
	 * @return the left motor's RPM - the right motor-s RPM
	 * @see getMotorSpeedDifference
	 * @see getLeftMotorRPM
	 * @see getRightMotorRPM
	 * @see atSetpoint
	 */
	public double getMotorRPMDifference() {
		return getLeftMotorRPM() - getRightMotorRPM();
	}

	/**
	 * Gets the difference in speed at the circumference of the motor's wheels
	 * between the left motor and right motor.
	 * 
	 * @return the left motor's speed - the right motor's speed
	 * @see getMotorRPMDifference
	 * @see getLeftMotorSpeed
	 * @see getRightMotorSpeed
	 * @see atSetpoint
	 */
	public double getMotorSpeedDifference() {
		return getLeftMotorSpeed() - getRightMotorSpeed();
	}
	
	public void resetPID(){
		leftPID.reset();
		rightPID.reset();
	}
}
