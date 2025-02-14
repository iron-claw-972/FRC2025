package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class ReverseMotors extends Command {
    // TODO: finish
    private Intake intake;
    private Indexer indexer;
    private Outtake outtake;
    public ReverseMotors(Intake intake, Indexer indexer, Outtake outtake){
        this.intake = intake;
        this.indexer = indexer;
        this.outtake = outtake;
        addRequirements(intake, indexer,outtake);
    }

    @Override
    public void initialize(){
        intake.setSpeed(-.5);
        indexer.reverse();
        //outtake.setSpeed(-.5);
    }

    @Override
    public boolean isFinished(){
        return true; //TODO Use a Timer
    }

    @Override
    public void end(boolean interrupted){
        intake.deactivate();;
        indexer.stop();
        //outtake.setSpeed(0);
    }
}
