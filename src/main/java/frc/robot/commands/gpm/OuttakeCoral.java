package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

public class OuttakeCoral extends Command {
    // TODO: finish
    public OuttakeCoral(Outtake outtake){
        addRequirements(outtake);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        
    }


}
