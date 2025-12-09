package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.VisionConstants;

/**
 * Shutdown all Orange Pis listed by hostname in
 * {@link frc.robot.constants.VisionConstants}
 */
public class ShutdownAllPis extends ParallelCommandGroup {
	public ShutdownAllPis() {
		ShutdownOrangePi[] commands =
			new ShutdownOrangePi[VisionConstants.ORANGEPI_HOSTNAMES.length];
		for (int i = 0; i < commands.length; i++) {
			commands[i] = new ShutdownOrangePi(VisionConstants.ORANGEPI_HOSTNAMES[i]);
		}

		addCommands(commands);
	}
}
