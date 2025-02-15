package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class FinishStationIntake extends Command {
    // TODO: Intake needs to move down first, then it's basically the same as a
    // normal intake
    // Maybe use the IntakeCoralHelper command

    private final Intake intake;
    private final Indexer indexer;
    private final Elevator elevator;
    private final Timer timer = new Timer();
    // TODO: Adjust TIMEOUT as needed
    private final double TIMEOUT = 2.0;

    public FinishStationIntake(Intake intake, Indexer indexer, Elevator elevator) {
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        addRequirements(intake, indexer, elevator);
    }

    @Override
    public void initialize() {
        intake.unstow();
        elevator.setSetpoint(ElevatorConstants.INTAKE_SETPOINT);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return !indexer.getSensorValue() || timer.hasElapsed(TIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stow();
        intake.deactivate();
        indexer.stop();
        timer.stop();
    }
}
