package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeAlgae extends Command {
    private final Intake intake;

    public IntakeAlgae(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setAngle(IntakeConstants.ALGAE_SETPOINT);
        intake.setSpeed(IntakeConstants.ALGAE_INTAKE_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        intake.deactivate();
        intake.setSpeed(-0.05);
    }
}