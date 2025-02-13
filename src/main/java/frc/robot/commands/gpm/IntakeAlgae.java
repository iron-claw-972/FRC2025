package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeAlgae extends Command {
    private final Intake intake;
    private final Timer timer = new Timer();

    // FIXME: Modify to not use a timer. We should intake until the driver releases the button and interrrupts the command
    private static final double INTAKE_TIME = 5.0;

    public IntakeAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setAngle(90);
        intake.setSpeed(1.0);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(INTAKE_TIME)) {
            intake.deactivate();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(INTAKE_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        intake.deactivate();
    }
}