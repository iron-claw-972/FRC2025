package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class ReverseMotors extends Command {
    private Intake intake;
    private Indexer indexer;
    private Outtake outtake;
    private final Timer timer = new Timer();

    private static final double EJECTION_TIME = 5.0;

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
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(EJECTION_TIME);
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
