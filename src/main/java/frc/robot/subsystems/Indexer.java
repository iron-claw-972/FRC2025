package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
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
	private SparkMax motor;
	private DigitalInput sensor;

	private FlywheelSim flywheelSim;
	private DIOSim sensorSim;

	public Indexer() {
		motor = new SparkMax(IdConstants.INDEXER_MOTOR, MotorType.kBrushless);
		sensor = new DigitalInput(IdConstants.INDEXER_SENSOR);

		if (Robot.isSimulation()) {
			flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
					IndexerConstants.momentOfInertia, IndexerConstants.gearRatio), DCMotor.getNEO(1));
			sensorSim = new DIOSim(sensor);
		}

		LogManager.logSupplier("Indexer sensor", () -> getSensorValue(), LogLevel.DEBUG);
		LogManager.logSupplier("Indexer motor", () -> getMotor(), LogLevel.DEBUG);
	}

	/**
	 * Set the indexer motor's speed
	 * setMotor(0) stops the indexer
	 * 
	 * @param speed the speed to set the motor to
	 */
	public void setMotor(double speed) {
		if (Math.abs(speed) <= 0.001) // if speed is really close to 0
			motor.stopMotor();
		else
			motor.set(speed);
	}

	/**
	 * @return the motor position in rotations
	 */
	public double getMotor() {
		if (Robot.isReal()) {
			return motor.getEncoder().getVelocity();
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
		if (Robot.isReal()) {
			return sensor.get();
		} else {
			return sensorSim.getValue();
		}
	}
	
	/**
	 * Set the sensor's state
	 * true means nothing is there, false means something is there
	 * Use for simulation (do NOT call on a real robot)
	 * 
	 * @param value the state to set the sensor to
	 */
	public void setSensorState(boolean value) {
		assert(Robot.isSimulation());
		sensorSim.setValue(value);
	};

	@Override
	public void periodic() { }

	@Override
	public void simulationPeriodic() {
		flywheelSim.setInput(motor.getOutputCurrent() * Constants.ROBOT_VOLTAGE);
		flywheelSim.update(Constants.LOOP_TIME);
	}
}
