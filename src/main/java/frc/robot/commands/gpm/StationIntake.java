package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class StationIntake extends Command {
    private Outtake outtake;

    public StationIntake(Outtake outtake) {
        this.outtake = outtake;
        addRequirements(outtake);
    }

    @Override
    public void initialize() {
        outtake.setMotor(0.7);
    }

    @Override
    public boolean isFinished() {
        return outtake.coralLoaded();
    }

    @Override
    public void end(boolean interrupted) {
        outtake.setMotor(0.02);
    }


}
