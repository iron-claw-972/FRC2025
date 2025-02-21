package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Intake a coral.
 */
public class IntakeCoral extends SequentialCommandGroup {
	public IntakeCoral(Intake intake, Indexer indexer, Elevator elevator) {
		addCommands(
				new InstantCommand(() -> intake.unstow()),
				new MoveElevator(elevator, ElevatorConstants.INTAKE_SETPOINT),
				new IntakeCoralHelper(intake, indexer));
	}

}
