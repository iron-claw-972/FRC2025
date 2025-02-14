package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeAlgae extends Command {
    private final Intake intake;

    public IntakeAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setAngle(90);
        intake.setSpeed(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        intake.deactivate();
    }
}