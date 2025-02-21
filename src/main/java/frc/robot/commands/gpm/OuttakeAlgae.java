package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OuttakeAlgae extends Command {
    private Intake intake;

    private final Timer timer = new Timer();
    private final double EJECTION_TIME = 1.0;

    public OuttakeAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.activate();
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(EJECTION_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        intake.deactivate();
        intake.stow();
        timer.stop();
    }

}
