package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ResetClimb extends Command {
    private Climb climb;

    public ResetClimb(Climb climb){
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize(){
        climb.reset(true);
    }
    @Override
    public void end(boolean interrupted){
        climb.reset(false);
    }

}
