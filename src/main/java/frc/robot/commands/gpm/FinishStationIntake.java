package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class FinishStationIntake extends Command {
    // TODO: finish and possibly rename

    Intake intake;

    public FinishStationIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setAngle(0);
        intake.deactivate();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        initialize();
    }
}
