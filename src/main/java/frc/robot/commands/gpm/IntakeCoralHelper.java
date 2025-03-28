package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

/**
 * All the intake code except the elevator stuff.
 * Don't call this directly; use {@link frc.robot.commands.gpm.IntakeCoral}
 * instead.
 */
public class IntakeCoralHelper extends Command {
	private Intake intake;
	private Indexer indexer;
	private Outtake outtake;
	private Arm arm;
	private Elevator elevator;

	private enum Phase {
		Acquiring, Intaking, Indexing, Detected, InOuttake, Done
	};

	private Phase phase;

	public IntakeCoralHelper(Intake intake, Indexer indexer, Outtake outtake, Arm arm, Elevator elevator) {
		this.intake = intake;
		this.indexer = indexer;
		this.outtake = outtake;
		this.arm = arm;
		this.elevator = elevator;
		addRequirements(intake, indexer, arm, elevator);
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
			outtake.setMotor(0.7);
		}
	}

	@Override
	public void execute() {
		if(outtake != null && outtake.coralLoaded()){
			phase = Phase.InOuttake;
		}
		switch (phase) {
			case Acquiring:
			case Intaking:
			case Indexing:
				if (!indexer.isIndexerClear()) {
					phase = Phase.Detected;
					intake.setSpeed(0.1);
					indexer.slow();
				}
				break;
			case Detected:
				if(indexer.isIndexerClear()){
					phase = Phase.InOuttake;
					intake.deactivate();
					intake.stow();
				}
				break;
			case InOuttake:
				if(outtake == null || outtake.coralLoaded()){
					phase = Phase.Done;
					elevator.setSetpoint(ElevatorConstants.INTAKE_STOW_SETPOINT);
					arm.setSetpoint(ArmConstants.STOW_SETPOINT);
					indexer.stop();
					if(outtake != null){
						outtake.setMotor(0.1);
					}
				}
				break;
			case Done:
				break;
		}
	}

	@Override
	public boolean isFinished() {
		return phase == Phase.Done && elevator.getPosition() > ElevatorConstants.SAFE_SETPOINT-0.025;
	}

	@Override
	public void end(boolean interrupted) {
		// in case it's interrupted
		intake.deactivate();
		intake.stow();
		indexer.stop();
		if(outtake != null){
			outtake.setMotor(0.02);
		}
	}
}
