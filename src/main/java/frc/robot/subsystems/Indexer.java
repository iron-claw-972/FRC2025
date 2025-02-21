package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.simulation.MockLaserCan;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.util.LogManager;
import frc.robot.util.LogManager.LogLevel;

public class Indexer extends SubsystemBase {
	private SparkFlex motor;
	private MockLaserCan simSensor;
	private LaserCanInterface sensor;

	private FlywheelSim flywheelSim;

	// where the coral is for simulation
	// in meters
	private double simCoralPos;

	public Indexer() {
		motor = new SparkFlex(IdConstants.INDEXER_MOTOR, MotorType.kBrushless);

		if (isSimulation()) {
			flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
					IndexerConstants.MOMENT_OF_INERTIA, IndexerConstants.GEAR_RATIO), DCMotor.getNEO(1));

			// have both interfaces availible
			simSensor = new MockLaserCan();
			sensor = simSensor;
		} else {
			sensor = new LaserCan(IdConstants.INDEXER_SENSOR);
		}
		simCoralPos = IndexerConstants.START_SIM_POS_AT; // initialize it anyway, it's easier

		LogManager.logSupplier("Indexer sensor", () -> isIndexerClear(), LogLevel.DEBUG);
		LogManager.logSupplier("Indexer motor", () -> getMotor(), LogLevel.DEBUG);
	}

	protected void setMotor(double power){
		motor.set(power);
	}

	/** Runs the indexer. */
	public void run() {
		setMotor(IndexerConstants.SPEED);
		simCoralPos = IndexerConstants.START_SIM_POS_AT;
	}

	/** Reverses the indexer. */
	public void reverse() {
		setMotor(-IndexerConstants.SPEED);
		simCoralPos = IndexerConstants.END_SIM_SENSOR_POS_AT;
	}

	/** Stops the indexer */
	public void stop() {
		setMotor(0);
	}

	/**
	 * @return the motor velocity in rotations per minute
	 */
	public double getMotor() {
		if (!isSimulation()) {
			return motor.getEncoder().getVelocity() / IndexerConstants.GEAR_RATIO;
		} else {
			return flywheelSim.getAngularVelocityRPM();
		}
	}

	/**
	 * Gets the LaserCAN's distance reading.
	 * If the distance is null, return 314,159
	 * 
	 * @return the distance, in millimeters
	 */
	private int getSensorValue() {
		var measurement = sensor.getMeasurement();
		int dist = (measurement == null) ? 314159 : measurement.distance_mm;
		return dist;
	}

	/**
	* Checks whether a coral is in the indexer.
	* True means nothing is there, false means something is there
	*
	* @return the sensor's state
	*/
	public boolean isIndexerClear() {
		return getSensorValue() > IndexerConstants.MEASUREMENT_THRESHOLD;
	}

	protected double getMotorSetpoint(){
		return motor.get();
	}

	@Override
	public void periodic() {
	}

	@Override
	public void simulationPeriodic() {
		flywheelSim.setInput(getMotorSetpoint() * Constants.ROBOT_VOLTAGE);
		flywheelSim.update(Constants.LOOP_TIME);

		// pretend we have a fake coral
		simCoralPos += flywheelSim.getAngularVelocityRPM() / 60. * Constants.LOOP_TIME
				* IndexerConstants.WHEEL_CIRCUMFERENCE;

		// toggle the sensor (values are backwards because that's how the sensor works)
		simSensor.setMeasurementPartialSim(0, // 0 == valid measurement
				(simCoralPos < IndexerConstants.START_SIM_SENSOR_POS_AT
						|| simCoralPos > IndexerConstants.END_SIM_SENSOR_POS_AT)
								? IndexerConstants.MEASUREMENT_THRESHOLD * 2
								: 0,
				1000 // IDK what this is exactly, but 1000 seems good
		);
	}

	/**
	 * Closes the motor and sets it to null
	 */
	protected void deleteMotor(){
		motor.close();
		motor = null;
	}

	public boolean isSimulation(){
		return RobotBase.isSimulation();
	}
}
