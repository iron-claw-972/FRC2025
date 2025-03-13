package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

/**
 * Intake a coral.
 */
public class IntakeCoral extends SequentialCommandGroup {
	public IntakeCoral(Intake intake, Indexer indexer, Elevator elevator, Outtake outtake, Arm arm) {
		//TODO make MoveElevator and MoveArm sequential
		addCommands(
				new InstantCommand(() -> intake.unstow()),
				new ParallelCommandGroup(
				new MoveElevator(elevator, ElevatorConstants.INTAKE_SETPOINT),
				new MoveArm(arm, ArmConstants.START_ANGLE)),
				new IntakeCoralHelper(intake, indexer, outtake, arm, elevator));
	}

}
