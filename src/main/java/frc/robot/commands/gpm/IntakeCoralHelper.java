package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

/**
 * All the intake code except the elevator stuff.
 * Don't call this directly; use {@link frc.robot.commands.gpm.IntakeCoral}
 * instead.
 */
public class IntakeCoralHelper extends Command {
	private Intake intake;
	private Indexer indexer;
	private Outtake outtake;

	private enum Phase {
		Acquiring, Intaking, Indexing, Detected, InOuttake, Done
	};

	private Phase phase;

	public IntakeCoralHelper(Intake intake, Indexer indexer, Outtake outtake) {
		this.intake = intake;
		this.indexer = indexer;
		this.outtake = outtake;
		addRequirements(intake, indexer);
		if(outtake != null){
			addRequirements(outtake);
		}
	}

	@Override
	public void initialize() {
		intake.activate();
		intake.unstow();
		indexer.run();
		phase = Phase.Acquiring;
		if(outtake != null) {
			outtake.setMotor(0.25);
		}
	}

	@Override
	public void execute() {

		switch (phase) {
			case Acquiring:
				if (intake.hasCoral()) {
					phase = Phase.Intaking;
				}
				break;
			case Intaking:
				if (!intake.hasCoral()) {
					phase = Phase.Indexing;
					intake.setSpeed(0.3);
				}
				break;
			case Indexing:
				if (!indexer.isIndexerClear()) {
					phase = Phase.Detected;
				}
				break;
			case Detected:
				if(outtake != null && outtake.coralLoaded() || outtake == null && indexer.isIndexerClear()){
					phase = Phase.InOuttake;
					intake.stow();
					intake.deactivate();
				}
				break;
			case InOuttake:
				if(outtake == null || outtake.coralEjecting()){
					phase = Phase.Done;
				}
				break;
			case Done:
				break;
		}
	}

	@Override
	public boolean isFinished() {
		return phase == Phase.Done || outtake != null && outtake.coralEjecting();
	}

	@Override
	public void end(boolean interrupted) {
		// in case it's interrupted
		intake.deactivate();
		intake.stow();
		indexer.stop();
		if(outtake != null){
			outtake.stop();
		}
	}
}
