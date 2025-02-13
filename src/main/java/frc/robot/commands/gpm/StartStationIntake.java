package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class StartStationIntake extends Command {
    // TODO: finish and possibly rename

    Intake intake;

    public StartStationIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setAngle(10);
        intake.activate();
    }

    @Override
    public boolean isFinished(){
        return intake.hasCoral();
    }

    @Override
    public void end(boolean interrupted){
        intake.deactivate();
        intake.stow();
    }
}
