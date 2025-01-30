package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.util.LogManager;
import frc.robot.util.LogManager.LogLevel;

public class Indexer extends SubsystemBase {
	public enum Status {
		RUNNING, FINISHING, IDLE, ERROR
	};

	private Status status;
	private Timer timeSinceRequest;
	private boolean sensorToggled;

	private SparkMax motor;
	private DigitalInput sensor;
	private FlywheelSim flywheelSim;
	private DIOSim sensorSim;

	public Indexer() {
		motor = new SparkMax(IdConstants.INDEXER_MOTOR, MotorType.kBrushless);
		sensor = new DigitalInput(IdConstants.INDEXER_SENSOR);

		timeSinceRequest = new Timer();
		sensorToggled = false;
		status = Status.IDLE;

		if (Robot.isSimulation()) {
			flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
					IndexerConstants.momentOfInertia, IndexerConstants.gearRatio), DCMotor.getNEO(1));
			sensorSim = new DIOSim(sensor);
		}

		LogManager.logSupplier("Indexer sensor", () -> getSensorValue(), LogLevel.DEBUG);
		LogManager.logSupplier("Indexer motor", () -> getMotor(), LogLevel.DEBUG);
	}

	/**
	 * @return the motor position in rotations
	 */
	private double getMotor() {
		if (Robot.isReal()) {
			return motor.getEncoder().getVelocity();
		} else {
			return flywheelSim.getAngularVelocityRPM();
		}
	}

	/**
	 * Gets the sensor's state
	 * true means nothing is there, false means something is there
	 */
	private boolean getSensorValue() {
		if (Robot.isReal()) {
			return sensor.get();
		} else {
			return sensorSim.getValue();
		}
	}

	@Override
	public void periodic() {
		switch (status) {
			case IDLE:
			case ERROR:
				motor.stopMotor();
				break;

			case RUNNING:
				motor.set(IndexerConstants.speed);

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
				motor.set(IndexerConstants.speed);

				if (timeSinceRequest.hasElapsed(IndexerConstants.runForExtra)) {
					timeSinceRequest.stop();
					motor.stopMotor();
				}

				break;
		}
	}

	@Override
	public void simulationPeriodic() {
		flywheelSim.setInput(motor.getOutputCurrent() * Constants.ROBOT_VOLTAGE);
		flywheelSim.update(Constants.LOOP_TIME);
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
