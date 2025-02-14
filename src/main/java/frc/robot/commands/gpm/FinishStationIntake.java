package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class FinishStationIntake extends Command {
    // TODO: finish and possibly rename
    // TODO: Intake needs to move down first, then it's basically the same as a normal intake
    // Maybe use the IntakeCoralHelper command

    private Intake intake;
    private Indexer indexer;
    private Elevator elevator;

    public FinishStationIntake(Intake intake, Indexer indexer, Elevator elevator){
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        addRequirements(intake, indexer, elevator);
    }

    @Override
    public void initialize(){
        intake.unstow();
        intake.activate();
        indexer.run();
        elevator.setSetpoint(ElevatorConstants.INTAKE_SETPOINT);
    }

    @Override
    public boolean isFinished(){
        return true; //TODO Finish when indexer finishes detecting
    }

    @Override
    public void end(boolean interrupted){
        intake.stow();
        indexer.stop();
    }
}
