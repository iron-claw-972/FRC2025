package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OuttakeCoral extends Command {
    // TODO: finish
    Intake intake;
    public OuttakeCoral(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setSpeed(-.5);
    }

    @Override
    public boolean isFinished(){
        return intake.hasCoral();
    }

    @Override
    public void end(boolean interrupted){
        intake.setSpeed(0);
    }


}
