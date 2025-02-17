package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class StartStationIntake extends Command {
    private Intake intake;

    public StartStationIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setAngle(IntakeConstants.STATION_SETPOINT);
        intake.activate();
    }

    @Override
    public boolean isFinished() {
        return intake.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        intake.deactivate();
        intake.stow();
    }
}
