package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class OuttakeAlgaeIntake extends Command {
    private Intake intake;

    private final Timer timer = new Timer();
    private final double EJECTION_TIME = 1;

    public OuttakeAlgaeIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(IntakeConstants.ALGAE_OUTTAKE_POWER);
        intake.setAngle(50);
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
