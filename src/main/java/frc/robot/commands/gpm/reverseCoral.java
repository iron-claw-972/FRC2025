package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.gpm.Elevator;
import frc.robot.subsystems.gpm.Outtake;

public class reverseCoral extends SequentialCommandGroup {
    public reverseCoral(Outtake outtake){
        
        addCommands(
            new InstantCommand(()->outtake.reverse(), outtake),
            new WaitCommand(0.5),
            new InstantCommand(()->outtake.stop(), outtake)
        );
    }
}
