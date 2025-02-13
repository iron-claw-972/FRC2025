package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// TODO: elevator controls
public class IntakeCoral extends Command {
	private Intake intake;
	private Indexer indexer;
	private Elevator elevator;

	private enum Phase {
		Acquiring, Intaking, Indexing, Done
	};

	private Phase phase;

	public IntakeCoral(Intake intake, Indexer indexer, Elevator elevator) {
		this.intake = intake;
		this.indexer = indexer;
		this.elevator = elevator;
		addRequirements(intake, indexer, elevator);
	}

	@Override
	public void initialize() {
		intake.activate();
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
				if (!indexer.getSensorValue()) {
					phase = Phase.Indexing;
					intake.deactivate();
				}
				break;
			case Indexing:
				if (indexer.getSensorValue()) { // TODO: do we want to run for some extra time?
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
		indexer.stop();
	}
}

