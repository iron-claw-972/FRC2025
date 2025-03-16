package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class IntakeAlgaeArm extends Command {
    private final Outtake outtake;

    public IntakeAlgaeArm(Outtake outtake) {
        this.outtake = outtake;
        addRequirements(outtake);
    }

    @Override
    public void initialize() {
        outtake.intakeAlgaeReef();
    }

    @Override
    public void end(boolean interrupted) {
        outtake.setMotor(-0.1);
    }
}
