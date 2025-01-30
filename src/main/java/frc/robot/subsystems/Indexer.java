package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.util.LogManager;
import frc.robot.util.LogManager.LogLevel;

public class Indexer extends SubsystemBase {
	enum Status {
		RUNNING, FINISHING, IDLE, ERROR
	};

	private Status status;
	private Timer timeSinceRequest;
	private boolean sensorToggled;

	private SparkMax motor;
	private DigitalInput sensor;
	private FlywheelSim flywheelSim;
	private boolean sensorSim; // it's a pretty simple sensor

	public Indexer() {
		if (Robot.isReal()) {
		motor = new SparkMax(IdConstants.INDEXER_MOTOR, MotorType.kBrushless); // FIXME: is it brushless?
		sensor = new DigitalInput(IdConstants.INDEXER_SENSOR);
		}

		timeSinceRequest = new Timer();
		sensorToggled = false;
		status = Status.IDLE;

		if (Robot.isSimulation()) {
			// see
			// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#what-is-state-space-notation
			flywheelSim = new FlywheelSim(new LinearSystem<N1, N1, N1>(
					new Matrix<N1, N1>(new SimpleMatrix(
							new double[] { -IndexerConstants.angularVelocity / IndexerConstants.angularAcceleration })),
					new Matrix<N1, N1>(new SimpleMatrix(new double[] { 1. / IndexerConstants.angularAcceleration })),
					new Matrix<N1, N1>(new SimpleMatrix(new double[] { 1 })),
					new Matrix<N1, N1>(new SimpleMatrix(new double[] { 0 }))), DCMotor.getNEO(1));
			sensorSim = false;
		}

		LogManager.logSupplier("Indexer sensor", () -> getSensorValue(), LogLevel.DEBUG);
		LogManager.logSupplier("Indexer motor", () -> getMotor(), LogLevel.DEBUG);
	}

	private void setMotor(double speed) {
		if (Robot.isReal()) {
			if (speed == 0)
				motor.stopMotor();
			else
				motor.set(speed);
		} else {
			flywheelSim.setInput(speed * Constants.ROBOT_VOLTAGE);
		}
	}

	/*
	 * @return the motor position in rotations
	 */
	private double getMotor() {
		if (Robot.isReal()) {
			return motor.getEncoder().getVelocity();
		} else {
			return flywheelSim.getAngularVelocityRPM();
		}
	}

	private boolean getSensorValue() {
		if (Robot.isReal()) {
			return sensor.get();
		} else {
			return sensorSim;
		}
	}

	@Override
	public void periodic() {
		switch (status) {
			case IDLE:
			case ERROR:
				setMotor(0);
				break;

			case RUNNING:
				setMotor(IndexerConstants.speed);

				// trigger after sensor goes on, broken, on; or __/--\__
				if (getSensorValue() == false) {
					sensorToggled = true;
				} else if (getSensorValue() == true && sensorToggled == true) {
					timeSinceRequest.reset(); // reuse the same timer
					status = Status.FINISHING;
				}

				if (timeSinceRequest.hasElapsed(IndexerConstants.timeout)) {
					timeSinceRequest.stop();
					LogManager.logFault("Indexer error", AlertType.kWarning);
					status = Status.ERROR; // if we've taken too long
				}

				break;

			case FINISHING:
				setMotor(IndexerConstants.speed);

				if (timeSinceRequest.hasElapsed(IndexerConstants.runForExtra)) {
					timeSinceRequest.stop();
					setMotor(0);
				}

				break;
		}
	}

	@Override
	public void simulationPeriodic() {

	}

	/*
	 * Run the indexer
	 */
	public void run() {
		status = Status.RUNNING;
		sensorToggled = false;
		timeSinceRequest.restart();
	}

	/*
	 * Stop the indexer.
	 */
	public void stop() {
		status = Status.IDLE;
		timeSinceRequest.stop();
	}

	/*
	 * Get the indexer status.
	 */
	public Status getStatus() {
		return status;
	}
}
