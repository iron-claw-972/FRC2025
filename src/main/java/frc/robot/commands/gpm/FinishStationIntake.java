package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

public class FinishStationIntake extends SequentialCommandGroup {
    public FinishStationIntake(Intake intake, Indexer indexer, Elevator elevator, Outtake outtake) {
        addCommands(
            new InstantCommand(()-> intake.unstow(), intake),
            new MoveElevator(elevator, ElevatorConstants.INTAKE_SETPOINT),
            new WaitUntilCommand(() -> intake.isAtSetpoint()),
            new IntakeCoralHelper(intake, indexer, outtake)
        );
    }
}
