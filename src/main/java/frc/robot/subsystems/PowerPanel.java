package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Power Distribution.
 * <p>
 * The primary goal PowerPanel is to provide feedback to the driver, operator, and programmers about how much current is drawn.
 * The real PDP/PDH measures the total current draw, and that value is reported on the SmartDashboard.
 * For the past 3 years, Iron Claw has fielded robots with excessive current draw.
 * At 2024 SVR, we were drawing 320 amperes and browning out the roboRIO.
 * Consider a 300 ampere draw: 12.6 volts - 300 amperes * 0.033 ohms = 2.6 volts.
 * <p>
 * It is also possible to do simulations that calculate current.
 * That work should be done, but it need not cover all subsystems (e.g., swerve) yet.
 * The total current will not be accurate, but that is a relatively minor issue right now.
 * Learning to do better simulations is an immediate benefit.
 * <p>
 * Robots will have either a CTRE PDP at CAN id 0 or a REV PDH at CAN id 1.
 * <p>
 * TODO: a map for subsystem loads to channel numbers
 */
public class PowerPanel extends SubsystemBase {
	/**
	 * The power distribution object.
	 * This construtor presumably searches for both the CTRE and REV modules at their default CAN ids.
	 * For our purposes, we expect a PDH.
	 */
	private final PowerDistribution pdh = new PowerDistribution();
	/** 
	 * The power distribution simulation object.
	 * <p>Used to set currents during simulations.
	 */
	private PDPSim pdhSim;
	/** The simulated battery voltage */
	private double voltsBattery = 12.6;
	/** 
	 * The simulated power distribution resistance. 
	 * <p>The battery resistance is about 25 milohms + some wire, breaker, and connector resistance.
	 */
	private static final double ohmsResistance = 0.030;

	/** Total current drawn from the battery. */
	private double currentTotal = 0.0;
	/** Peak current drawn fromt the battery. Half life of 5 seconds. */
	private double currentPeak = 0.0;

	/** clock ticks (increments of 20 ms) */
	private int ticks = 0;
	
	public PowerPanel() {
		// if we are simulating
		if (RobotBase.isSimulation()) {
			// make the simulation object.
			pdhSim = new PDPSim(pdh);

			// example of setting a current for a channel...
			setCurrent(18, 12.4);
		}

		// make the SmartDashboard keys now to avoid overruns later
		SmartDashboard.putNumber("PDH voltage", voltsBattery);
		SmartDashboard.putNumber("PDH total current", currentTotal);
		SmartDashboard.putNumber("PDH peak current", currentPeak);
		SmartDashboard.putNumber("Energy draw (percent)", 0.0);
	}

	/**
	 * Close the subsystem. This method is needed for unit testing.
	 */
	public void close() {
		pdh.close();
	}

	@Override
	public void periodic() {
		// get the current draw from the battery
		currentTotal = pdh.getTotalCurrent();

		// exponentially decay the previous peak current
		// say we want half the current remaining after 5 seconds (250 clock ticks)
		// (alpha)^250 = 0.5
		// 250 log(alpha) = log(0.5)
		// log(alpha) = (log 0.5) / 250
		// log(alpha) = -0.3010 / 250 = -0.0012
		// alpha = 0.99723
		currentPeak = currentPeak * 0.99723;

		// calculate the peak current
		currentPeak = Math.max(currentTotal, currentPeak);

		// bump the timer
		ticks++;
		// every so often, update the dashboard
		if (ticks % 10 == 0) {
			// put the voltage on the SmartDashboard. It is on the DriverStation, but it is not tracking the roboRIO voltage
			SmartDashboard.putNumber("PDH voltage", voltsBattery);

			// put the current draw on the SmartDashboard
			SmartDashboard.putNumber("PDH total current", currentTotal);
			SmartDashboard.putNumber("PDH peak current", currentPeak);

			// the PowerDistribution object also integrates power to get energy
			// Not supported on the REV PDH!
			double joules = pdh.getTotalEnergy();
			// An 18 ampere hour battery will have this amount of energy
			double joulesBattery = 18.0 * 3600.0 * 12.6;
			// Report how much of the battery has been used
			SmartDashboard.putNumber("Energy draw (percent)", joules/joulesBattery);
		}
	}

	@Override
	public void simulationPeriodic() {
		// This method relies on subsystems calculating a simulated current and setting the current draw for that channel.
		// If all subsystems do that, then we have an accurate total current.

		// as a test, every 20 seconds, impose a 100-ampere load on channel 3
		// remove this code when current draw simulations are working
		int ticksTest = ticks % 1000;
		if (ticksTest == 10) {
			setCurrent(3, 100.0);
		}
		if (ticksTest == 110) {
			setCurrent(3, 0.0);
		}

		// From the total current, estimate the battery voltage
		voltsBattery = 12.6 - currentTotal * ohmsResistance;

		// tell the PDH what the battery voltage is. Can access with pdh.getVoltage()
		pdhSim.setVoltage(voltsBattery);

		// This object can supply the battery voltage, and PDH can supply the battery voltage.
		// Code may get the battery voltage with RobotController.getBatteryVoltage()
		RoboRioSim.setVInVoltage(voltsBattery);
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
		return pdh.getCurrent(channel);
	}

	/**
	 * This method is used by simulators to set the channel current.
	 * <p>It should only be called duing simulation (when pdhSim is not null).
	 * @param channel PDH port/channel
	 * @param current Channel current in amperes.
	 */
	public void setCurrent(int channel, double current) {
		pdhSim.setCurrent(channel, current);
	}
}
