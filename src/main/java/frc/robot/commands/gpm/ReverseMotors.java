package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class ReverseMotors extends Command {
    // TODO: finish
    Intake intake;
    Outtake outtake;
    public ReverseMotors(Intake intake, Outtake outtake){
        this.intake = intake;
        this.outtake = outtake;
        addRequirements(intake,outtake);
    }

    @Override
    public void initialize(){
        intake.setSpeed(-.5);
        //outtake.setSpeed(-.5);
    }

    @Override
    public boolean isFinished(){
        return true; //TODO when is this finished?
    }

    @Override
    public void end(boolean interrupted){
        intake.setSpeed(0);
        //outtake.setSpeed(0);
    }
}
