package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase {
	enum Status {
		RUNNING, FINISHING, IDLE, ERROR
	};

	private Status status;
	private Timer timeSinceRequest;
	private boolean sensorToggled;

	private SparkMax motor;
	private DigitalInput sensor;

	public Indexer() {
		motor = new SparkMax(IdConstants.INDEXER_MOTOR, MotorType.kBrushless); // FIXME: is it brushless?
		sensor = new DigitalInput(IdConstants.INDEXER_SENSOR);
		timeSinceRequest = new Timer();
		sensorToggled = false;
		status = Status.IDLE;
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
				if (sensor.get() == false) {
					sensorToggled = true;
				} else if (sensor.get() == true && sensorToggled == true) {
					timeSinceRequest.reset(); // reuse the same timer
					status = Status.FINISHING;
				}

				if (timeSinceRequest.hasElapsed(IndexerConstants.timeout)) {
					timeSinceRequest.stop();
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
		this.status = Status.IDLE;
		this.timeSinceRequest.stop();
	}

	/*
	 * Get the indexer status.
	 */
	public Status getStatus() {
		return this.status;
	}
}
