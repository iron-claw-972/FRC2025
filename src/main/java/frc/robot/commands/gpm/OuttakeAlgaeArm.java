package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class OuttakeAlgaeArm extends Command{
    private final Outtake outtake;

    private final Timer timer = new Timer();
    private final double EJECTION_TIME = 0.25;

    public OuttakeAlgaeArm(Outtake outtake) {
        this.outtake = outtake;
        addRequirements(outtake);
    }

    @Override
    public void initialize() {
        outtake.outtakeAlgae();
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(EJECTION_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
        timer.stop();
    }
}
