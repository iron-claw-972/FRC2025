package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

public class OuttakeCoralNew extends Command {
    private Outtake outtake;
    public OuttakeCoralNew(Outtake outtake) {
        addRequirements(outtake);
        this.outtake = outtake;
    }

    public void initialize() {
        outtake.outtake();
    }

    public void end(boolean interrupted) {
        outtake.stop();
    }

    public boolean isFinished() {
        return outtake.isFinished();
    }
}
 