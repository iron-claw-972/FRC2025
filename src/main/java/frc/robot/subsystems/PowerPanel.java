package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Power Distribution.
 * TODO: Either fully implement this with all subsystems, including swerve, or delete it
 * Using it would require rewriting swerve sim, and we can't use it without swerve, since that takes more current than anything else
 */
public class PowerPanel extends SubsystemBase {
	private final PowerDistribution PDH = new PowerDistribution();
	private PDPSim PDHSim; // not sure if this is only CTRE or what
	/** The simulated battery voltage */
	private double voltsBattery = 12.6;
	// assume the battery resistance is about 25 milohms + some wire resistance
	private static final double ohmsResistance = 0.030;
	
	public PowerPanel() {
		if (RobotBase.isSimulation()) {
			PDHSim = new PDPSim(PDH);

			PDHSim.setCurrent(18, 12.4); //this is an example
		}
	}

	public void close() {
		PDH.close();
	}

	@Override
	public void periodic() {
		// put the current draw on the SmartDashboard
		//SmartDashboard.putNumber("PDH Current (Amps)", PDH.getTotalCurrent());
		//SmartDashboard.putNumber("PDH Current for Arm (Amps)", PDH.getCurrent(1) + PDH.getCurrent(2) + PDH.getCurrent(4) + PDH.getCurrent(5));

		// simulate the voltage on the battery
		voltsBattery = 12.6 - PDH.getTotalCurrent() * ohmsResistance;
	}

	@Override
	public void simulationPeriodic() {

	}

	/**
	 * Return the battery voltage.
	 * Can be used in simulations.
	 * @return battery voltage
	 */
	public double getSimulatedBatteryVoltage() {
		return voltsBattery;
	}

	/**
	 * Return the current drawn by the PDH channel.
	 * @param channel
	 * @return current in amperes
	 */
	public double getCurrent(int channel) {
		return PDH.getCurrent(channel);
	}

	/**
	 * This method is used by simulators to set the channel current
	 * @param channel PDH port/channel
	 * @param current Channel current in amperes.
	 */
	public void setCurrent(int channel, double current) {
		PDHSim.setCurrent(channel, current);
	}

}
