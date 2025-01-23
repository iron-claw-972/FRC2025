package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.gpm.Elevator;
import frc.robot.subsystems.gpm.Outtake;

/** 
 * Command to outake coral. It also lowers the elevator.
 * If the command is interrupted, it does not guarantee the coral motor is stopped.
 */
public class OuttakeCoral extends SequentialCommandGroup {
    public OuttakeCoral(Outtake outtake, Elevator elevator){
        
        addCommands(
            // start the coral moving
            new InstantCommand(()->outtake.outtake(), outtake),
            // wainting 2 seconds? coral should eject faster than that
            new WaitCommand(2),
            // should the subsystem stop on its own? How is a misfeed detected?
            // why 2 instant commands? one would do.
            new InstantCommand(()->outtake.stop(), outtake),
            new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT), elevator)
        );
    }
}
