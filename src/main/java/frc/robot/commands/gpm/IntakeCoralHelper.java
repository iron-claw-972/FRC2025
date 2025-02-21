package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * All the intake code except the elevator stuff.
 * Don't call this directly; use {@link frc.robot.commands.gpm.IntakeCoral}
 * instead.
 */
public class IntakeCoralHelper extends Command {
	private Intake intake;
	private Indexer indexer;

	private enum Phase {
		Acquiring, Intaking, Indexing, Done
	};

	private Phase phase;

	public IntakeCoralHelper(Intake intake, Indexer indexer) {
		this.intake = intake;
		this.indexer = indexer;
		addRequirements(intake, indexer);
	}

	@Override
	public void initialize() {
		intake.activate();
		intake.unstow();
		phase = Phase.Acquiring;
	}

	@Override
	public void execute() {
		switch (phase) {
			case Acquiring:
				if (intake.hasCoral()) {
					phase = Phase.Intaking;
					indexer.run();
				}
				break;
			case Intaking:
				if (!indexer.isIndexerClear()) {
					phase = Phase.Indexing;
					intake.deactivate();
					intake.stow();
				}
				break;
			case Indexing:
				if (indexer.isIndexerClear()) { // TODO: do we want to run for some extra time?
					phase = Phase.Done;
					indexer.stop();
				}
				break;
			case Done:
				break;
		}
	}

	@Override
	public boolean isFinished() {
		return phase == Phase.Done;
	}

	@Override
	public void end(boolean interrupted) {
		// in case it's interrupted
		intake.deactivate();
		intake.stow();
		indexer.stop();
	}
}
