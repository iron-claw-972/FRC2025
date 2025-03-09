package frc.robot.commands.vision;

import java.io.IOException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

public class ShutdownOrangePi extends Command {
	private Process process;
	private boolean passwordTyped;
	private Pattern promptMatcher;

	public ShutdownOrangePi() {
		promptMatcher = Pattern.compile("password: ?$");
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public void initialize() {
		passwordTyped = false;
		if (Robot.isSimulation()) {
			// this will probably break on Windows systems so...
			System.out.println("What OrangePi? This is simulation!");
		} else {
			try {
				String[] commandString = new String[] { "ssh",
						"-o", "UserKnownHostsFile /dev/null",
						"-o", "StrictHostKeyChecking no",
						VisionConstants.ORANGEPI_USERNAME + "@" + VisionConstants.ORANGEPI_IP,
						"sudo", "shutdown", "now" };
				this.process = Runtime.getRuntime().exec(commandString);
			} catch (Exception e) {
				String message = e.getMessage() == null ? "unknown" : e.getMessage();
				System.out.println("Failed to shutdown OrangePi. Reason: " + message);
			}
		}
	}

	@Override
	public void execute() {
		if (this.process == null) return; // command creation failed for some reason
		if (this.passwordTyped) return; // don't try typing the password if we've already done it

		try {
			// read as much as possible
			int availibleBytes = this.process.getInputStream().available();
			byte[] buffer = new byte[availibleBytes];
			this.process.getInputStream().read(buffer, 0, availibleBytes);
			String asStr = new String(buffer);

			Matcher matches = promptMatcher.matcher(asStr);
			if (matches.find()) { // if we're at the prompt...
				this.process.getOutputStream().write((VisionConstants.ORANGEPI_PASSWORD + "\n").
					getBytes()); // ...type the password
				this.process.getOutputStream().flush();
				this.passwordTyped = true;
			}
		} catch (IOException e) {
			this.cancel();
		}
	}

	@Override
	public boolean isFinished() {
		return this.process == null || !this.process.isAlive();
	}

	@Override
	public void end(boolean interrupted) {
		if (this.process == null) return;
		
		if (interrupted) {
			this.process.destroy(); // end the command if we've been interrupted
		} else if (!this.process.isAlive()) {
			int exitValue = this.process.exitValue();
			if (exitValue != 0) // abnormal termination
				System.out.println("OrangePi shutdown failed with exit code " + exitValue + ".");
		}
	}
}
