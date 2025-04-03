package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase {
	private TalonFX motor;
	private MockLaserCan simSensor;
	private LaserCanInterface sensor;

	private FlywheelSim flywheelSim;

	// where the coral is for simulation
	// in meters
	private double simCoralPos;

	private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

	public Indexer() {
		motor = new TalonFX(IdConstants.INDEXER_MOTOR);
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
	}

	/** Runs the indexer. */
	public void run() {
		motor.set(IndexerConstants.SPEED);
		simCoralPos = IndexerConstants.START_SIM_POS_AT;
	}

	public void slow(){
		motor.set(0.6);
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
		return inputs.velocity;
	}

	/**
	 * Gets the LaserCAN's distance reading.
	 * If the distance is null, return 314,159
	 * 
	 * @return the distance, in millimeters
	 */
	public int getSensorValue() {
		return inputs.sensorDistance;
	}

	/**
	* Checks whether a coral is in the indexer.
	* True means nothing is there, false means something is there
	*
	* @return the sensor's state
	*/
	@AutoLogOutput(key = "Intake/isIndexerClear")
	public boolean isIndexerClear() {
		return true;
	}

	@Override
	public void periodic() {
		//SmartDashboard.putBoolean("Indexer has coral ", isIndexerClear());

		if (Robot.isReal()) {
			inputs.velocity =  motor.getVelocity().getValueAsDouble() / IndexerConstants.GEAR_RATIO;
		} else {
			inputs.velocity = flywheelSim.getAngularVelocityRPM();
		}
		var measurement = sensor.getMeasurement();
		inputs.sensorDistance = (measurement == null || measurement.status > 0) ? 314159 : measurement.distance_mm;
		Logger.processInputs("Indexer", inputs);
		Logger.recordOutput("Indexer/indexer coral",isIndexerClear());
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
