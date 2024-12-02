package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Intake.Mode;
import frc.robot.subsystems.gpm.StorageIndex;

import java.util.function.Consumer;

public class IntakeNote extends Command{

    private final Intake intake;
    private final StorageIndex storageIndex;
    private final Arm arm;

    private final Consumer<Boolean> rumbleConsumer;

    private final Timer timer = new Timer();
    private Boolean detectedNote = false;

	private boolean doStuff = true;

    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm, Consumer<Boolean> rumbleConsumer) {
        this.intake = intake;
        this.storageIndex = storageIndex;
        this.arm = arm;
        this.rumbleConsumer = rumbleConsumer;
		if (intake == null || storageIndex == null || arm == null || rumbleConsumer == null)
			doStuff = false;
		else {}
//			addRequirements(intake, storageIndex, arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.stop();
		if (doStuff) {
			detectedNote = false;
			intake.setMode(Mode.INTAKE);
			storageIndex.runIndex();
			arm.setAngle(ArmConstants.intakeSetpoint);
		}
    }

    @Override
    public void execute(){
		if (!doStuff) return;
        storageIndex.runIndex();
        if (intake.hasNote()){
            detectedNote = true;
        }
        if(!intake.hasNote() && detectedNote){
            timer.start();
        }
        rumbleConsumer.accept(detectedNote);
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(0.01); 
    }

    @Override
    public void end(boolean interupted){
        timer.stop();
        timer.reset();
		if (!doStuff) return;
        if(detectedNote){
            CommandScheduler.getInstance().schedule(new RunCommand(() -> {
                rumbleConsumer.accept(true);
            }).withTimeout(0.5).andThen(new InstantCommand(()->{
                rumbleConsumer.accept(false);
            })));
        }
        intake.setMode(Mode.DISABLED);
        storageIndex.stopIndex();
        arm.setAngle(ArmConstants.stowedSetpoint);
        detectedNote = false;
    }
    
}
