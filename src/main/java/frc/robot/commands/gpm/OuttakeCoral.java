package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

public class OuttakeCoral extends SequentialCommandGroup {
    public OuttakeCoral(Outtake outtake, Elevator elevator){
        addCommands(
            new OuttakeCoralNew(outtake),
            new InstantCommand(()->elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT), elevator)
        );
    }
}
