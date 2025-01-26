package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

/** 
 * Command to outake coral. It also lowers the elevator.
 * If the command is interrupted, it does not guarantee the coral motor is stopped.
 */
public class OuttakeCoral extends SequentialCommandGroup {
    public OuttakeCoral(Outtake outtake, Elevator elevator){
        
        addCommands(
            // Outtake the coral
            new OuttakeCoralBasic(outtake),
            new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT), elevator)
        );
    }
}
