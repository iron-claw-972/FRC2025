package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class RemoveAlgae extends Command {
    private Outtake outtake;
    public RemoveAlgae(Outtake outtake){
        this.outtake = outtake;
        addRequirements(outtake);
    }

    @Override
    public void initialize(){
        outtake.removeAlgae();
    }

    @Override
    public void end(boolean interrupted){
        outtake.stop();
    }
}
