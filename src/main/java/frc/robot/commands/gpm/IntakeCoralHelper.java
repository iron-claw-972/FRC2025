package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
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
		Acquiring, Intaking, Indexing, Detected, InOuttake, Waiting, Done
	};

	private Phase phase;

	private Timer waitTimer = new Timer();

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
		intake.enableLaserCan(true);
		if(outtake != null) {
			outtake.setMotor(0.25);
		}
		waitTimer.reset();
		waitTimer.stop();
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
				if(outtake.coralLoaded()){
					phase = Phase.InOuttake;
					intake.stow();
					intake.deactivate();
				}
				break;
			case InOuttake:
				break;
			case Waiting:
				if(waitTimer.hasElapsed(0.1)){
					phase = Phase.Done;
				}
				break;
			case Done:
				break;
		}
		if(outtake == null && phase == Phase.InOuttake || outtake != null && outtake.coralEjecting()){
			phase = Phase.Waiting;
			waitTimer.start();
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
		outtake.stop();
		intake.enableLaserCan(false);
		waitTimer.stop();
		waitTimer.reset();
	}
}
