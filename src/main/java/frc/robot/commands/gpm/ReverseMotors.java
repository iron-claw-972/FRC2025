package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class ReverseMotors extends Command {
    // TODO: finish
    private Intake intake;
    private Indexer indexer;
    private Outtake outtake; // TODO do we need an outtake here?
    private final Timer timer = new Timer();

    private static final double EJECTION_TIME = 5.0;

    public ReverseMotors(Intake intake, Indexer indexer, Outtake outtake) {
        this.intake = intake;
        this.indexer = indexer;
        this.outtake = outtake;
        addRequirements(intake, indexer, outtake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(-.5);
        indexer.reverse();
        // outtake.setSpeed(-.5);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(EJECTION_TIME)) {
            intake.deactivate();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(EJECTION_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        intake.deactivate();
        ;
        indexer.stop();
        // outtake.setSpeed(0);
    }
}
