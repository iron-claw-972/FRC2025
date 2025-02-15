package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.simulation.MockLaserCan;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.util.LogManager;
import frc.robot.util.LogManager.LogLevel;

public class Indexer extends SubsystemBase {
	private SparkMax motor;
	private MockLaserCan simSensor;
	private LaserCanInterface sensor;

	private FlywheelSim flywheelSim;

	// where the coral is for simulation
	// in meters
	private double simCoralPos;

	public Indexer() {
		motor = new SparkMax(IdConstants.INDEXER_MOTOR, MotorType.kBrushless);

		if (Robot.isSimulation()) {
			flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
					IndexerConstants.MOMENT_OF_INERTIA, IndexerConstants.GEAR_RATIO), DCMotor.getNEO(1));

			// have both interfaces availible
			simSensor = new MockLaserCan();
			sensor = simSensor;
		} else {
			sensor = new LaserCan(IdConstants.INDEXER_SENSOR);
		}
		simCoralPos = IndexerConstants.START_SIM_POS_AT; // initialize it anyway, it's easier

		LogManager.logSupplier("Indexer sensor", () -> getSensorValue(), LogLevel.DEBUG);
		LogManager.logSupplier("Indexer motor", () -> getMotor(), LogLevel.DEBUG);
	}

	/** Runs the indexer. */
	public void run() {
		motor.set(IndexerConstants.SPEED);
		simCoralPos = IndexerConstants.START_SIM_POS_AT;
	}

	/** Stops the indexer */
	public void stop() {
		motor.stopMotor();
	}

	/**
	 * @return the motor velocity in rotations per minute
	 */
	public double getMotor() {
		if (Robot.isReal()) {
			return motor.getEncoder().getVelocity() / IndexerConstants.GEAR_RATIO;
		} else {
			return flywheelSim.getAngularVelocityRPM();
		}
	}

	/**
	 * Gets the sensor's state
	 * true means nothing is there, false means something is there
	 * 
	 * @return the sensor's state
	 */
	public boolean getSensorValue() {
		return sensor.getMeasurement().distance_mm > IndexerConstants.MEASUREMENT_THRESHOLD;
	}

	@Override
	public void periodic() {
	}

	@Override
	public void simulationPeriodic() {
		flywheelSim.setInput(motor.get() * Constants.ROBOT_VOLTAGE);
		flywheelSim.update(Constants.LOOP_TIME);

		// pretend we have a fake coral
		simCoralPos += flywheelSim.getAngularVelocityRPM() / 60. * Constants.LOOP_TIME
				* IndexerConstants.WHEEL_CIRCUMFERENCE;

		// toggle the sensor (values are backwards because that's how the sensor works)
		simSensor.setMeasurementPartialSim(0, // IDK what status is; 0 == good???
				(simCoralPos < IndexerConstants.START_SIM_SENSOR_POS_AT
						|| simCoralPos > IndexerConstants.END_SIM_SENSOR_POS_AT)
								? IndexerConstants.MEASUREMENT_THRESHOLD * 2
								: 0,
				4 // IDK what ambient is; 4 is a good number
		);
	}
}
