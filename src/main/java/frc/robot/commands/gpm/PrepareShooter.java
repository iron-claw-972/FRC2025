package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.gpm.Shooter;

public class PrepareShooter extends InstantCommand {
	/**
	* Spins the shooter up to the target speed, and exits immediatly.
	*
	* @param shooter the shooter to spin up
	* @param target  the target speed (in m/s) to spin up to.
	* @see           frc.robot.commands.gpm.SetShooterSpeed
	*/
	public PrepareShooter(Shooter shooter, double target) {
		super(()->shooter.setTargetRPM(target), shooter);
	}
}
