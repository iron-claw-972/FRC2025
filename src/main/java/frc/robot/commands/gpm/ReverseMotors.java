package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

public class ReverseMotors extends Command {
    private Intake intake;
    private Indexer indexer;
    private Outtake outtake;

    public ReverseMotors(Intake intake, Indexer indexer, Outtake outtake) {
        this.intake = intake;
        this.indexer = indexer;
        this.outtake = outtake;
        addRequirements(intake, indexer);
        if(outtake != null){
            addRequirements(outtake);
        }
    }

    @Override
    public void initialize() {
        intake.setSpeed(-.5);
        indexer.reverse();
        if(outtake != null){
            outtake.reverse();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.deactivate();
        indexer.stop();
        if(outtake != null){
            outtake.stop();;
        }
    }
}
