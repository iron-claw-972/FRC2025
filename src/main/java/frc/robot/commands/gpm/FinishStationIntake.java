package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.elevator.Elevator;

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
