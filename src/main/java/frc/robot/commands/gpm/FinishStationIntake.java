package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class FinishStationIntake extends Command {
    // TODO: finish and possibly rename

    Intake intake;
    Indexer indexer;
    Elevator elevator;

    public FinishStationIntake(Intake intake, Indexer indexer, Elevator elevator){
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        addRequirements(intake, indexer, elevator);
    }

    @Override
    public void initialize(){
        intake.setAngle(0); //TODO set to proper angle
        intake.activate();
        indexer.run();
        elevator.setSetpoint(0); //TODO set to intake position
    }

    @Override
    public boolean isFinished(){
        return true; //TODO how to know when finish?
    }

    @Override
    public void end(boolean interrupted){
        intake.stow();
        indexer.stop();
    }
}
