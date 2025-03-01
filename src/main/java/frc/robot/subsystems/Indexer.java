package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import au.grapplerobotics.simulation.MockLaserCan;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IndexerConstants;
// import frc.robot.util.LogManager;
// import frc.robot.util.LogManager.LogLevel;

public class Indexer extends SubsystemBase {
	private SparkFlex motor;
	//private TalonFX motor;
	private MockLaserCan simSensor;
	private LaserCanInterface sensor;

	private FlywheelSim flywheelSim;

	// where the coral is for simulation
	// in meters
	private double simCoralPos;

	public Indexer() {
		motor = new SparkFlex(IdConstants.INDEXER_MOTOR, MotorType.kBrushless);
		//motor = new TalonFX(IdConstants.INDEXER_MOTOR);
		if (Robot.isSimulation()) {
			flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
					IndexerConstants.MOMENT_OF_INERTIA, IndexerConstants.GEAR_RATIO), DCMotor.getNEO(1));

			// have both interfaces availible
			simSensor = new MockLaserCan();
			sensor = simSensor;
		} else {
			sensor = new LaserCan(IdConstants.INDEXER_SENSOR);
            try {
                sensor.setRangingMode(RangingMode.SHORT);
                sensor.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
                sensor.setRegionOfInterest(new RegionOfInterest(-4, -4, 8, 8));
            } catch (ConfigurationFailedException e) {
                DriverStation.reportError("Indexer LaserCan configuration error", true);
            }
		}
		simCoralPos = IndexerConstants.START_SIM_POS_AT; // initialize it anyway, it's easier

		// LogManager.logSupplier("Indexer sensor", () -> isIndexerClear(), LogLevel.DEBUG);
		// LogManager.logSupplier("Indexer motor", () -> getMotor(), LogLevel.DEBUG);
		//SmartDashboard.putNumber("indexer speed", 0);
	}

	/** Runs the indexer. */
	public void run() {
		motor.set(IndexerConstants.SPEED);
		simCoralPos = IndexerConstants.START_SIM_POS_AT;
	}

	/** Reverses the indexer. */
	public void reverse() {
		motor.set(-IndexerConstants.SPEED);
		simCoralPos = IndexerConstants.END_SIM_SENSOR_POS_AT;
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
	 * Gets the LaserCAN's distance reading.
	 * If the distance is null, return 314,159
	 * 
	 * @return the distance, in millimeters
	 */
	private int getSensorValue() {
		var measurement = sensor.getMeasurement();
		int dist = (measurement == null || measurement.status > 0) ? 314159 : measurement.distance_mm;
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

	@Override
	public void periodic() {
		//SmartDashboard.putBoolean("Indexer has coral ", isIndexerClear());
	}

	@Override
	public void simulationPeriodic() {
		flywheelSim.setInput(motor.get() * Constants.ROBOT_VOLTAGE);
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
}
