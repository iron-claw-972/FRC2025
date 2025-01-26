package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class ReverseMotors extends SequentialCommandGroup {
    public ReverseMotors(Intake intake, Outtake outtake){
        addCommands(
            new InstantCommand(()->outtake.reverse(), outtake),
            new WaitCommand(0.5),
            new InstantCommand(()->outtake.stop(), outtake)
        );
    }
}
