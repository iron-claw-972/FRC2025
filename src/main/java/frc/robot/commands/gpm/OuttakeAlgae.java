package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class OuttakeAlgae extends Command {
    private Intake intake;

    private final Timer timer = new Timer();
    private final double EJECTION_TIME = 5.0;

    public OuttakeAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
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
        intake.stow();
    }

}
