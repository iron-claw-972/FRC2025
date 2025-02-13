package frc.robot.commands.gpm;

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
				// TODO: do we want to parallelize the initial intake and moving elevator?
				new MoveElevator(elevator, ElevatorConstants.INTAKE_SETPOINT),
				new IntakeCoralHelper(intake, indexer));
	}
}
