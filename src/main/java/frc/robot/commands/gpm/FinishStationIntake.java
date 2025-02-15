package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class FinishStationIntake extends Command {
    // TODO: Intake needs to move down first, then it's basically the same as a
    // normal intake
    // Maybe use the IntakeCoralHelper command

    private Intake intake;
    private Indexer indexer;
    private Elevator elevator;

    public FinishStationIntake(Intake intake, Indexer indexer, Elevator elevator) {
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        addRequirements(intake, indexer, elevator);
        new IntakeCoralHelper(intake, indexer);
    }

    @Override
    public void initialize() {
        intake.unstow();
        elevator.setSetpoint(ElevatorConstants.INTAKE_SETPOINT);
    }

    @Override
    public boolean isFinished() {
        return indexer.getSensorValue(); // TODO Finish when indexer finishes detecting
        // Should we use a timer or what? we can't just do !getSensorValue(), right?
    }

    @Override
    public void end(boolean interrupted) {
        intake.stow();
        intake.deactivate();
        indexer.stop();
    }
}
