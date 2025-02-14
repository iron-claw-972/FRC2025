package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class OuttakeAlgae extends Command {
    private Intake intake;

    public OuttakeAlgae(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setSpeed(IntakeConstants.ALGAE_OUTTAKE_POWER);
    }

    @Override
    public boolean isFinished(){
        return true; //TODO Use a Timer
    }

    @Override
    public void end(boolean interrupted){
        intake.deactivate();;
        intake.stow();
    }


}
