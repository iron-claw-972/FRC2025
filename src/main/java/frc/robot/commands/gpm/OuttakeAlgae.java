package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class OuttakeAlgae extends Command {
    // TODO: finish
    Intake intake;
    public OuttakeAlgae(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setSpeed(-.5);
    }

    @Override
    public boolean isFinished(){
        return true; //TODO how do we know when algae is out or in?
    }

    @Override
    public void end(boolean interrupted){
        intake.setSpeed(0);
    }


}
